#include "DynamicCameraPivot.h"

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

}


// Main function to call every frame
void DynamicCameraPivot::_process(double delta) {

}


// Main event handler for when unit lands on the ground
void DynamicCameraPivot::on_landed() {

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