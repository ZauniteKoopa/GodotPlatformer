#include "PlatformerFeetSensor.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// Main constructor
PlatformerFeetSensor::PlatformerFeetSensor() {
    numGroundSensed = 0;
}

// Main destructor
PlatformerFeetSensor::~PlatformerFeetSensor() {

}

// Main function to bind methods to class
void PlatformerFeetSensor::_bind_methods() {
    // Bind listeners
    ClassDB::bind_method(D_METHOD("on_body_enter"), &PlatformerFeetSensor::on_body_enter);
    ClassDB::bind_method(D_METHOD("on_body_exit"), &PlatformerFeetSensor::on_body_exit);

    // Add signals
    ADD_SIGNAL(MethodInfo("landed"));
    ADD_SIGNAL(MethodInfo("fall_begin"));

}

// Main event handler function for when something enters this area
void PlatformerFeetSensor::on_body_enter(Node3D* body) {
    groundLock.lock();

    if (numGroundSensed == 0) {
        emit_signal("landed");
    }
    numGroundSensed++;

    groundLock.unlock();
}

// Main event handler function for when something exits this area
void PlatformerFeetSensor::on_body_exit(Node3D* body) {
    groundLock.lock();

    numGroundSensed--;
    if (numGroundSensed == 0) {
        emit_signal("fall_begin");
    }

    groundLock.unlock();
}
