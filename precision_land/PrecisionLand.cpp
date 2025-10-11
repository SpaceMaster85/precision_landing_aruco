#include "PrecisionLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

static const std::string kModeName = "PrecisionLandCustom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

PrecisionLand::PrecisionLand(rclcpp::Node& node)
    : ModeBase(node, kModeName)
    , _node(node)
{
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

    _target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", rclcpp::QoS(1).best_effort(),
        std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1)
    );

    _vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
        std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1)
    );

    loadParameters();
    reset();

    modeRequirements().manual_control = false;
}

// -------------------- Reset Internal State --------------------
void PrecisionLand::reset()
{
    _tag = ArucoTag{
        Eigen::Vector3d(NAN, NAN, NAN),  // position invalid
        Eigen::Quaterniond(1, 0, 0, 0), // default orientation
        rclcpp::Time(0)
    };
    _search_started = false;
    _search_waypoint_index = 0;
    _approach_altitude = NAN;
    _vel_x_integral = 0.0f;
    _vel_y_integral = 0.0f;
    _state = State::Idle;
    _target_lost_prev = false;
    _target_lost_timer = 0.0f;

    RCLCPP_INFO(_node.get_logger(), "PrecisionLand internal state reset");
}

// -------------------- Parameters --------------------
void PrecisionLand::loadParameters()
{
    _node.declare_parameter<float>("descent_vel", 1.0);
    _node.declare_parameter<float>("vel_p_gain", 1.5);
    _node.declare_parameter<float>("vel_i_gain", 0.0);
    _node.declare_parameter<float>("max_velocity", 3.0);
    _node.declare_parameter<float>("target_timeout", 3.0);
    _node.declare_parameter<float>("delta_position", 0.25);
    _node.declare_parameter<float>("delta_velocity", 0.25);
    _node.declare_parameter<float>("target_lost_debounce", 0.5);
    _node.declare_parameter<float>("search_altitude", 10.0);
    _node.declare_parameter<float>("layer_spacing", 2.0); 

    _node.get_parameter("descent_vel", _param_descent_vel);
    _node.get_parameter("vel_p_gain", _param_vel_p_gain);
    _node.get_parameter("vel_i_gain", _param_vel_i_gain);
    _node.get_parameter("max_velocity", _param_max_velocity);
    _node.get_parameter("target_timeout", _param_target_timeout);
    _node.get_parameter("delta_position", _param_delta_position);
    _node.get_parameter("delta_velocity", _param_delta_velocity);
    _node.get_parameter("target_lost_debounce", _target_lost_debounce);
    _node.get_parameter("search_altitude", _search_altitude);
    _node.get_parameter("layer_spacing", _search_layer_spacing); 

    RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
    RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
    RCLCPP_INFO(_node.get_logger(), "search_altitude: %f", _search_altitude);
    RCLCPP_INFO(_node.get_logger(), "layer_spacing: %f", _search_layer_spacing); 
}

// -------------------- Callbacks --------------------
void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
    _land_detected = msg->landed;
}

void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!_search_started) return;

    _tag = getTagWorld(ArucoTag{
        Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
        Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
        _node.now()
    });
}

// -------------------- Transform Utilities --------------------
PrecisionLand::ArucoTag PrecisionLand::getTagWorld(const ArucoTag& tag)
{
    Eigen::Matrix3d R;
    R << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;
    Eigen::Quaterniond quat_NED(R);

    Eigen::Vector3d vehicle_pos = _vehicle_local_position->positionNed().cast<double>();
    Eigen::Quaterniond vehicle_orient = _vehicle_attitude->attitude().cast<double>();

    Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_pos) * vehicle_orient;
    Eigen::Affine3d camera_transform = Eigen::Translation3d(Eigen::Vector3d::Zero()) * quat_NED;
    Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;
    Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;

    return ArucoTag{tag_world_transform.translation(), Eigen::Quaterniond(tag_world_transform.rotation()), tag.timestamp};
}

// -------------------- Activation / Deactivation --------------------
void PrecisionLand::onActivate()
{
    reset();
    generateSearchWaypoints();
    _search_started = true;
    switchToState(State::Search);
}

void PrecisionLand::onDeactivate()
{
    // No-op
}

// -------------------- Main Update --------------------
void PrecisionLand::updateSetpoint(float dt_s)
{
    bool target_lost = checkTargetTimeout();

    _target_lost_timer = target_lost ? _target_lost_timer + dt_s : 0.0f;
    bool failed = (_target_lost_timer > _target_lost_debounce);

    if (target_lost && !_target_lost_prev) {
        RCLCPP_INFO(_node.get_logger(), "Target lost: State %s", stateName(_state).c_str());
    } else if (!target_lost && _target_lost_prev) {
        RCLCPP_INFO(_node.get_logger(), "Target acquired");
    }
    _target_lost_prev = target_lost;

    switch (_state) {
    case State::Idle:
        break;

    case State::Search: {
        if (!std::isnan(_tag.position.x())) {
            _approach_altitude = _vehicle_local_position->positionNed().z();
            switchToState(State::Approach);
            break;
        }

        auto waypoint_position = _search_waypoints[_search_waypoint_index];
        _trajectory_setpoint->updatePosition(waypoint_position);

        if (positionReached(waypoint_position)) {
            _search_waypoint_index = (_search_waypoint_index + 1) % _search_waypoints.size();
        }
        break;
    }

    case State::Approach: {
        if (failed) {
            RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
            ModeBase::completed(px4_ros2::Result::ModeFailureOther);
            reset();
            switchToState(State::Idle);
            return;
        }

        auto target_position = Eigen::Vector3f(_tag.position.x(), _tag.position.y(), _approach_altitude);
        _trajectory_setpoint->updatePosition(target_position);

        if (positionReached(target_position)) {
            switchToState(State::Descend);
        }
        break;
    }

    case State::Descend: {
        if (failed) {
            RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
            ModeBase::completed(px4_ros2::Result::ModeFailureOther);
            reset();
            switchToState(State::Idle);
            return;
        }

        Eigen::Vector2f vel_xy = calculateVelocitySetpointXY();
        _trajectory_setpoint->update(
            Eigen::Vector3f(vel_xy.x(), vel_xy.y(), _param_descent_vel),
            std::nullopt,
            px4_ros2::quaternionToYaw(_tag.orientation)
        );

        if (_land_detected) {
            switchToState(State::Finished);
        }
        break;
    }

    case State::Finished:
        ModeBase::completed(px4_ros2::Result::Success);
        break;
    }
}

// -------------------- Velocity Controller --------------------
Eigen::Vector2f PrecisionLand::calculateVelocitySetpointXY()
{
    float delta_x = _vehicle_local_position->positionNed().x() - _tag.position.x();
    float delta_y = _vehicle_local_position->positionNed().y() - _tag.position.y();

    _vel_x_integral = std::clamp(_vel_x_integral + delta_x, -_param_max_velocity, _param_max_velocity);
    _vel_y_integral = std::clamp(_vel_y_integral + delta_y, -_param_max_velocity, _param_max_velocity);

    float vx = -((_param_vel_p_gain * delta_x) + (_param_vel_i_gain * _vel_x_integral));
    float vy = -((_param_vel_p_gain * delta_y) + (_param_vel_i_gain * _vel_y_integral));

    return Eigen::Vector2f(
        std::clamp(vx, -_param_max_velocity, _param_max_velocity),
        std::clamp(vy, -_param_max_velocity, _param_max_velocity)
    );
}

// -------------------- Timeout Check --------------------
bool PrecisionLand::checkTargetTimeout()
{
    if (!_tag.valid()) return true;
    return (_node.now().seconds() - _tag.timestamp.seconds()) > _param_target_timeout;
}

// -------------------- Search Waypoints --------------------
void PrecisionLand::generateSearchWaypoints()
{
    double start_x = 0.0;
    double start_y = 0.0;
    double current_z = _vehicle_local_position->positionNed().z();
    const double target_z = _search_altitude; // Always search at configured altitude
    const double max_radius = 2.0;
    const double layer_spacing = _search_layer_spacing; // jetzt als Parameter
    const int points_per_layer = 16;

    std::vector<Eigen::Vector3f> waypoints;
    int num_layers = std::max(1, static_cast<int>(std::abs(target_z - current_z) / layer_spacing));

    for (int layer = 0; layer < num_layers; ++layer) {
        std::vector<Eigen::Vector3f> layer_waypoints;

        // Outward spiral
        double radius = 0.0;
        for (int point = 0; point <= points_per_layer; ++point) {
            double angle = 2.0 * M_PI * point / points_per_layer;
            layer_waypoints.emplace_back(start_x + radius * cos(angle),
                                         start_y + radius * sin(angle),
                                         current_z);
            radius += max_radius / points_per_layer;
        }
        waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

        // Update altitude towards target
        if (current_z < target_z) current_z = std::min(current_z + layer_spacing, target_z);
        else current_z = std::max(current_z - layer_spacing, target_z);

        // Inward spiral (reversed)
        std::reverse(layer_waypoints.begin(), layer_waypoints.end());
        for (auto& wp : layer_waypoints) wp.z() = current_z;
        waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

        if (current_z < target_z) current_z = std::min(current_z + layer_spacing, target_z);
        else current_z = std::max(current_z - layer_spacing, target_z);
    }

    _search_waypoints = waypoints;
}

// -------------------- Position Check --------------------
bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
    Eigen::Vector3f delta = target - _vehicle_local_position->positionNed();
    return (delta.norm() < _param_delta_position) &&
           (_vehicle_local_position->velocityNed().norm() < _param_delta_velocity);
}

// -------------------- State Helpers --------------------
std::string PrecisionLand::stateName(State state)
{
    switch (state) {
        case State::Idle: return "Idle";
        case State::Search: return "Search";
        case State::Approach: return "Approach";
        case State::Descend: return "Descend";
        case State::Finished: return "Finished";
        default: return "Unknown";
    }
}

void PrecisionLand::switchToState(State state)
{
    RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
    _state = state;
}

// -------------------- Main --------------------
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}
