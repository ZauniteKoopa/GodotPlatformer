#include "ForceArea.h"
#include "PlatformerPackage3D.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// Main constructor
ForceArea::ForceArea() {}


// Main destructor
ForceArea::~ForceArea() {}


// Main function to bind methods
void ForceArea::_bind_methods() {
    // Methods
    ClassDB::bind_method(D_METHOD("on_body_enter"), &ForceArea::on_body_enter);

    // Properties
    ClassDB::bind_method(D_METHOD("get_force_duration"), &ForceArea::get_force_duration);
    ClassDB::bind_method(D_METHOD("set_force_duration"), &ForceArea::set_force_duration);
    ClassDB::add_property("ForceArea", PropertyInfo(Variant::FLOAT, "force_duration"), "set_force_duration", "get_force_duration");

    ClassDB::bind_method(D_METHOD("get_force_magnitude"), &ForceArea::get_force_magnitude);
    ClassDB::bind_method(D_METHOD("set_force_magnitude"), &ForceArea::set_force_magnitude);
    ClassDB::add_property("ForceArea", PropertyInfo(Variant::FLOAT, "force_magnitude"), "set_force_magnitude", "get_force_magnitude");

    ClassDB::bind_method(D_METHOD("get_force_direction"), &ForceArea::get_force_direction);
    ClassDB::bind_method(D_METHOD("set_force_direction"), &ForceArea::set_force_direction);
    ClassDB::add_property("ForceArea", PropertyInfo(Variant::VECTOR3, "force_direction"), "set_force_direction", "get_force_direction");

    // Signals
    ADD_SIGNAL(MethodInfo("force_applied"));
}


// Main event handler function for when
void ForceArea::on_body_enter(Node3D* body) {
    PlatformerPackage3D* player = Object::cast_to<PlatformerPackage3D>(body);

    if(player) {
        // Convert local to global
        Vector3 globalDirection = to_global(localForceDirection) - get_global_position();

        // Apply force
        player->apply_speed_force(forceMagnitude * globalDirection.normalized(), forceDuration, true);
        emit_signal("force_applied");
    }
}



// --------------------------
// PROPERTIES
// -------------------------


void ForceArea::set_force_duration(const double p_value) {
    forceDuration = p_value;
}

double ForceArea::get_force_duration() const {
    return forceDuration;
}


void ForceArea::set_force_magnitude(const double p_value) {
    forceMagnitude = p_value;
}

double ForceArea::get_force_magnitude() const {
    return forceMagnitude;
}


void ForceArea::set_force_direction(const Vector3 p_value) {
    localForceDirection = p_value;
}

Vector3 ForceArea::get_force_direction() const {
    return localForceDirection;
}
