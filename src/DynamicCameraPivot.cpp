#include "DynamicCameraPivot.h"

#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// Main function to bind methods
void DynamicCameraPivot::_bind_methods() {
    // Listeners to bind
    ClassDB::bind_method(D_METHOD("on_landed"), &DynamicCameraPivot::on_landed);

    // Set properties
    ClassDB::bind_method(D_METHOD("get_vertical_position_offset"), &DynamicCameraPivot::get_vertical_position_offset);
    ClassDB::bind_method(D_METHOD("set_vertical_position_offset"), &DynamicCameraPivot::set_vertical_position_offset);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::FLOAT, "vertical_position_offset"), "set_vertical_position_offset", "get_vertical_position_offset");

    ClassDB::bind_method(D_METHOD("get_vertical_free_space_buffer"), &DynamicCameraPivot::get_vertical_free_space_buffer);
    ClassDB::bind_method(D_METHOD("set_vertical_free_space_buffer"), &DynamicCameraPivot::set_vertical_free_space_buffer);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::FLOAT, "vertical_free_space_buffer"), "set_vertical_free_space_buffer", "get_vertical_free_space_buffer");

    ClassDB::bind_method(D_METHOD("get_target"), &DynamicCameraPivot::get_target);
    ClassDB::bind_method(D_METHOD("set_target"), &DynamicCameraPivot::set_target);
    ClassDB::add_property("DynamicCameraPivot", PropertyInfo(Variant::NODE_PATH, "target"), "set_target", "get_target");

}


// Constructor
DynamicCameraPivot::DynamicCameraPivot() {

}


// Destructor
DynamicCameraPivot::~DynamicCameraPivot() {

}


// Main function to call on start
void DynamicCameraPivot::_ready() {
    initialize_current_node_pointers();

    set_position(target->get_global_position());
    double currentYPivot = target->get_global_position()[Vector3::AXIS_Y];
}


// Main function to call every frame
void DynamicCameraPivot::_process(double delta) {

    if (!Engine::get_singleton()->is_editor_hint()) {

        target = get_node<Node3D>(target_path);
        Vector3 currentPosition = Vector3(
            target->get_global_position()[Vector3::AXIS_X],
            get_current_y_position(delta),
            target->get_global_position()[Vector3::AXIS_Z]
        );

        currentPosition += (verticalPositionOffset * Vector3(0, 1, 0));
        set_position(currentPosition);
    }
}


// Main event handler for when unit lands on the ground
void DynamicCameraPivot::on_landed() {
    // Set transitioning variables
    startYPosition = currentYPivot;
    finalYPosition = target->get_global_position()[Vector3::AXIS_Y];
    heightTransitionTimer = 0;
    transitionDuration = Math::abs(finalYPosition - currentYPivot) / transitionHeightSpeed;
    isTransitioningY = true;
}


// Main helper function to get the current Y position of camera
double DynamicCameraPivot::get_current_y_position(double delta) {
    // If transitioning Y, gradually transition to position
    if (isTransitioningY) {
        // Increment timer and lerp
        heightTransitionTimer += delta;
        currentYPivot = Math::lerp(startYPosition, finalYPosition, heightTransitionTimer / transitionDuration);

        // If transition ends, set flag to false
        if (heightTransitionTimer >= transitionDuration) {
            isTransitioningY = false;
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
        target = get_node<Node3D>(target_path);
    }
}




// --------------------------
// PROPERTIES
// -------------------------

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


void DynamicCameraPivot::set_target(NodePath p_value) {
    target_path = p_value;
}

NodePath DynamicCameraPivot::get_target() const {
    return target_path;
}