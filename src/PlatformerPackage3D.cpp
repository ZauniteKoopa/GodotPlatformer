#include "PlatformerPackage3D.h"

#include <godot_cpp/core/class_db.hpp>

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
void PlatformerPackage3D::relative_run(Vector2 controller_vector) {

}

// Walking speed property setter
void PlatformerPackage3D::set_walking_speed(const double p_walking_speed) {
    walking_speed = p_walking_speed;
}

// Walking speed property accessor
double PlatformerPackage3D::get_walking_speed() const {
    return walking_speed;
}
