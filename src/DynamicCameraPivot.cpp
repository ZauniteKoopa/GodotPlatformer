#include "DynamicCameraPivot.h"

#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// Main function to bind methods
void DynamicCameraPivot::_bind_methods() {
    // Functions to bind
    ClassDB::bind_method(D_METHOD("rotate_camera"), &DynamicCameraPivot::rotate_camera);

    // Listeners to bind
    ClassDB::bind_method(D_METHOD("on_landed"), &DynamicCameraPivot::on_landed);

    // Set properties
    bind_properties();
}


// Constructor
DynamicCameraPivot::DynamicCameraPivot() {

}


// Destructor
DynamicCameraPivot::~DynamicCameraPivot() {

}


// Main function to call on start
void DynamicCameraPivot::_enter_tree() {
    initialize_current_node_pointers();

    set_position(target->get_global_position());
    double currentYPivot = target->get_global_position()[Vector3::AXIS_Y];

    yaw = get_global_basis().get_euler()[Vector3::AXIS_X];
    pitch = get_global_basis().get_euler()[Vector3::AXIS_Y];
}


// Main function to call every frame
void DynamicCameraPivot::_process(double delta) {

    if (!Engine::get_singleton()->is_editor_hint()) {

        // Set position
        Vector3 currentPosition = Vector3(
            target->get_global_position()[Vector3::AXIS_X],
            get_current_y_position(delta),
            target->get_global_position()[Vector3::AXIS_Z]
        );

        currentPosition += (verticalPositionOffset * Vector3(0, 1, 0));
        set_position(currentPosition);

        // Set rotation
        set_rotation(Vector3(get_transitioning_yaw(delta), pitch, 0));
    }
}


// Main event handler for when unit lands on the ground
void DynamicCameraPivot::on_landed() {
    // Set transitioning variables
    startYPosition = currentYPivot;
    finalYPosition = target->get_global_position()[Vector3::AXIS_Y];
    heightTransitionTimer = 0;
    transitionDuration = Math::abs(finalYPosition - currentYPivot) / transitionHeightSpeed;
    isTransitioningToGroundHeight = true;
}


// Main helper function to get the current Y position of camera
double DynamicCameraPivot::get_current_y_position(double delta) {
    // If you're not grounded, set isTransitioningToGroundHeight to false (why transition to ground if you're not grounded anymore)
    if (!target->is_grounded()) {
        isTransitioningToGroundHeight = false;
    }

    // If transitioning Y, gradually transition to position
    if (isTransitioningToGroundHeight) {
        // Increment timer and lerp
        heightTransitionTimer += delta;
        currentYPivot = Math::lerp(startYPosition, finalYPosition, heightTransitionTimer / transitionDuration);

        // If transition ends, set flag to false
        if (heightTransitionTimer >= transitionDuration) {
            isTransitioningToGroundHeight = false;
        }

    // Else do normal offsetting
    } else {
        // Calculate distance from current Y Pivot
        double currentTargetPosition = target->get_global_position()[Vector3::AXIS_Y];
        double distanceFromPivot = currentTargetPosition - currentYPivot;

        // If distance is bigger than free buffer, correct the current position to follow. Else, just return current pivot position
        if (Math::abs(distanceFromPivot) > verticalFreeSpaceBuffer) {
            double crossedFreeBufferLimit = currentYPivot + (Math::sign(distanceFromPivot) * verticalFreeSpaceBuffer);
            double pivotDelta = currentTargetPosition - crossedFreeBufferLimit;
            currentYPivot += pivotDelta;
        }
    }

    return currentYPivot;
}

// Main helper function to initialize node pointers
void DynamicCameraPivot::initialize_current_node_pointers() {
    if (target_path != nullptr) {
        target = get_node<PlatformerPackage3D>(target_path);
    }
}


// Main function to rotate the camera
void DynamicCameraPivot::rotate_camera(double deltaX, double deltaY) {
    // Just increment pitch
    pitch += deltaY;

    // Increment vertical yaw with limit
    yaw += deltaX;
    yaw = Math::clamp(yaw, Math::deg_to_rad(yawMinPivot), Math::deg_to_rad(yawMaxPivot));
}


// Main helper function to get transitioning yaw
double DynamicCameraPivot::get_transitioning_yaw(double delta) {
    double targetYaw = yaw + get_yaw_adjustment();
    double currentYaw = get_global_basis().get_euler()[Vector3::AXIS_X];
    double yawTransitionDelta = angleTransitionSpeed * delta;

    // If distance between current yaw and target yaw is less than delta, just return target yaw
    if (Math::abs(targetYaw - currentYaw) <= yawTransitionDelta) {
        return targetYaw;

    // Else, return a transitioning yaw
    } else {
        return currentYaw + (Math::sign(targetYaw - currentYaw) * yawTransitionDelta);
    }
}


// Main helper function to get current yaw adjustment for jump
double DynamicCameraPivot::get_yaw_adjustment() {
    if (target == nullptr) {
        return 0;
    }

    double clampedJumpSpeed = Math::clamp(target->get_current_vertical_speed(), -adjustmentSpeedLimit, 0.0);
    double jumpProgress = (clampedJumpSpeed + adjustmentSpeedLimit) / adjustmentSpeedLimit;
    return Math::lerp(-Math::deg_to_rad(fallingAngleAdjustment), 0, jumpProgress);
}


// --------------------------
// PROPERTIES
// -------------------------

// Vertical offsetting

void DynamicCameraPivot::set_vertical_position_offset(double p_value) {
    verticalPositionOffset = p_value;
}

double DynamicCameraPivot::get_vertical_position_offset() const {
    return verticalPositionOffset;
}


void DynamicCameraPivot::set_vertical_free_space_buffer(double p_value) {
    verticalFreeSpaceBuffer = p_value;
}

double DynamicCameraPivot::get_vertical_free_space_buffer() const {
    return verticalFreeSpaceBuffer;
}


void DynamicCameraPivot::set_transition_height_speed(double p_value) {
    transitionHeightSpeed = p_value;
}

double DynamicCameraPivot::get_transition_height_speed() const {
    return transitionHeightSpeed;
}


// Vertical Yaw

void DynamicCameraPivot::set_yaw_max_pivot(double p_value) {
    yawMaxPivot = p_value;
}

double DynamicCameraPivot::get_yaw_max_pivot() const {
    return yawMaxPivot;
}


void DynamicCameraPivot::set_yaw_min_pivot(double p_value) {
    yawMinPivot = p_value;
}

double DynamicCameraPivot::get_yaw_min_pivot() const {
    return yawMinPivot;
}


void DynamicCameraPivot::set_falling_angle_adjustment(double p_value) {
    fallingAngleAdjustment = p_value;
}

double DynamicCameraPivot::get_falling_angle_adjustment() const {
    return fallingAngleAdjustment;
}


void DynamicCameraPivot::set_adjustment_speed_limit(double p_value) {
    adjustmentSpeedLimit = p_value;
}

double DynamicCameraPivot::get_adjustment_speed_limit() const {
    return adjustmentSpeedLimit;
}


void DynamicCameraPivot::set_angle_transition_speed(double p_value) {
    angleTransitionSpeed = p_value;
}

double DynamicCameraPivot::get_angle_transition_speed() const {
    return angleTransitionSpeed;
}


// Nodes

void DynamicCameraPivot::set_target(NodePath p_value) {
    target_path = p_value;
}

NodePath DynamicCameraPivot::get_target() const {
    return target_path;
}



// General call to bind properties from the parent bind methods
void DynamicCameraPivot::bind_properties() {
    ClassDB::bind_method(D_METHOD("get_vertical_position_offset"), &DynamicCameraPivot::get_vertical_position_offset);
    ClassDB::bind_method(D_METHOD("set_vertical_position_offset"), &DynamicCameraPivot::set_vertical_position_offset);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::FLOAT, "vertical_offsetting/vertical_position_offset"), "set_vertical_position_offset", "get_vertical_position_offset");

    ClassDB::bind_method(D_METHOD("get_vertical_free_space_buffer"), &DynamicCameraPivot::get_vertical_free_space_buffer);
    ClassDB::bind_method(D_METHOD("set_vertical_free_space_buffer"), &DynamicCameraPivot::set_vertical_free_space_buffer);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::FLOAT, "vertical_offsetting/vertical_free_space_buffer"), "set_vertical_free_space_buffer", "get_vertical_free_space_buffer");

    ClassDB::bind_method(D_METHOD("get_transition_height_speed"), &DynamicCameraPivot::get_transition_height_speed);
    ClassDB::bind_method(D_METHOD("set_transition_height_speed"), &DynamicCameraPivot::set_transition_height_speed);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::FLOAT, "vertical_offsetting/transition_height_speed"), "set_transition_height_speed", "get_transition_height_speed");

    ClassDB::bind_method(D_METHOD("get_yaw_max_pivot"), &DynamicCameraPivot::get_yaw_max_pivot);
    ClassDB::bind_method(D_METHOD("set_yaw_max_pivot"), &DynamicCameraPivot::set_yaw_max_pivot);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::FLOAT, "yaw_pivoting/yaw_max_pivot"), "set_yaw_max_pivot", "get_yaw_max_pivot");

    ClassDB::bind_method(D_METHOD("get_yaw_min_pivot"), &DynamicCameraPivot::get_yaw_min_pivot);
    ClassDB::bind_method(D_METHOD("set_yaw_min_pivot"), &DynamicCameraPivot::set_yaw_min_pivot);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::FLOAT, "yaw_pivoting/yaw_min_pivot"), "set_yaw_min_pivot", "get_yaw_min_pivot");

    ClassDB::bind_method(D_METHOD("get_falling_angle_adjustment"), &DynamicCameraPivot::get_falling_angle_adjustment);
    ClassDB::bind_method(D_METHOD("set_falling_angle_adjustment"), &DynamicCameraPivot::set_falling_angle_adjustment);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::FLOAT, "yaw_pivoting/falling_angle_adjustment"), "set_falling_angle_adjustment", "get_falling_angle_adjustment");

    ClassDB::bind_method(D_METHOD("get_angle_transition_speed"), &DynamicCameraPivot::get_angle_transition_speed);
    ClassDB::bind_method(D_METHOD("set_angle_transition_speed"), &DynamicCameraPivot::set_angle_transition_speed);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::FLOAT, "yaw_pivoting/angle_transition_speed"), "set_angle_transition_speed", "get_angle_transition_speed");

    ClassDB::bind_method(D_METHOD("get_adjustment_speed_limit"), &DynamicCameraPivot::get_adjustment_speed_limit);
    ClassDB::bind_method(D_METHOD("set_adjustment_speed_limit"), &DynamicCameraPivot::set_adjustment_speed_limit);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::FLOAT, "yaw_pivoting/adjustment_speed_limit"), "set_adjustment_speed_limit", "get_adjustment_speed_limit");

    ClassDB::bind_method(D_METHOD("get_target"), &DynamicCameraPivot::get_target);
    ClassDB::bind_method(D_METHOD("set_target"), &DynamicCameraPivot::set_target);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::NODE_PATH, "target"), "set_target", "get_target");
}