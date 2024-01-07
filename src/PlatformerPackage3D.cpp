#include "PlatformerPackage3D.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

// To print UtilityFunctions::print()

using namespace godot;

// Function to bind methods
void PlatformerPackage3D::_bind_methods() {
    // General methods to bind
    ClassDB::bind_method(D_METHOD("relative_run"), &PlatformerPackage3D::relative_run);

    // Bind properties
    bind_properties();
}

// General call to bind properties from the parent bind methods
void PlatformerPackage3D::bind_properties() {
    ClassDB::bind_method(D_METHOD("get_walking_speed"), &PlatformerPackage3D::get_walking_speed);
    ClassDB::bind_method(D_METHOD("set_walking_speed"), &PlatformerPackage3D::set_walking_speed);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "walking_speed"), "set_walking_speed", "get_walking_speed");

    ClassDB::bind_method(D_METHOD("get_camera_node_path"), &PlatformerPackage3D::get_camera_node_path);
    ClassDB::bind_method(D_METHOD("set_camera_node_path"), &PlatformerPackage3D::set_camera_node_path);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::NODE_PATH, "camera_node_path"), "set_camera_node_path", "get_camera_node_path");
}

// General constructor
PlatformerPackage3D::PlatformerPackage3D() {
    walking_speed = 10;
}

// Main function to clean up
PlatformerPackage3D::~PlatformerPackage3D() {

}

// Main function that plays on game start
void PlatformerPackage3D::_ready() {
    
}

// Main function that plays for each physics frame
void PlatformerPackage3D::_physics_process(double delta) {

}


// Main function to move a certain direction given a controller vector
void PlatformerPackage3D::relative_run(Vector2 controller_vector, double time_delta) {
    // Initialize camera if it has not been set up yet
    if (current_camera == nullptr) {
        initialize_current_camera();
    }

    // Get world directions based on current camera (forward = current_camera.transform.basis.z)
    Vector3 world_forward = -current_camera->get_global_transform().get_basis().get_column(2);
    Vector3 up_vector = Vector3(0, 1, 0);
    Vector3 world_right = world_forward.cross(up_vector);

    // Project vectors on a flat plane
    Vector3 origin = Vector3(0, 0, 0);
    Plane surface_movement_plane = Plane(up_vector, origin);
    world_forward = surface_movement_plane.project(world_forward).normalized();
    world_right = surface_movement_plane.project(world_right).normalized();

    // Get single movement delta vector and move
    Vector3 movement_delta = (controller_vector.x * world_right) + (controller_vector.y * world_forward);
    movement_delta = walking_speed * time_delta * movement_delta.normalized();
    move_and_collide(movement_delta);
}



// ------------------------------------------------
// PROPERTIES
// ------------------------------------------------

// Walking speed property setter / getter
void PlatformerPackage3D::set_walking_speed(const double p_walking_speed) {
    walking_speed = p_walking_speed;
}

double PlatformerPackage3D::get_walking_speed() const {
    return walking_speed;
}


// Camera Node Path setter / getter
void PlatformerPackage3D::set_camera_node_path(const NodePath p_node_path) {
    camera_node_path = p_node_path;
}

NodePath PlatformerPackage3D::get_camera_node_path() const {
    return camera_node_path;
}

void PlatformerPackage3D::initialize_current_camera() {
    if (camera_node_path != nullptr) {
        current_camera = get_node<Camera3D>(camera_node_path);
    }
}
