#include "PlatformerFeetSensor.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// Main constructor
PlatformerFeetSensor::PlatformerFeetSensor() {
    numGroundSensed = 0;
    coyoteTimeDuration = 2.0;

    curCoyoteTimer.unref();
    coyoteTimeListener = Callable(this, "on_coyote_timeout");
    coyoteTimeDisabled = false;
}

// Main destructor
PlatformerFeetSensor::~PlatformerFeetSensor() {

}

// Main function to bind methods to class
void PlatformerFeetSensor::_bind_methods() {
    // Bind listeners
    ClassDB::bind_method(D_METHOD("on_body_enter"), &PlatformerFeetSensor::on_body_enter);
    ClassDB::bind_method(D_METHOD("on_body_exit"), &PlatformerFeetSensor::on_body_exit);
    ClassDB::bind_method(D_METHOD("on_coyote_timeout"), &PlatformerFeetSensor::on_coyote_timeout);
    ClassDB::bind_method(D_METHOD("disable_coyote_time"), &PlatformerFeetSensor::disable_coyote_time);

    // Add signals
    ADD_SIGNAL(MethodInfo("landed"));
    ADD_SIGNAL(MethodInfo("fall_begin"));

    // Set up properties
    ClassDB::bind_method(D_METHOD("get_coyote_time_duration"), &PlatformerFeetSensor::get_coyote_time_duration);
    ClassDB::bind_method(D_METHOD("set_coyote_time_duration"), &PlatformerFeetSensor::set_coyote_time_duration);
    ClassDB::add_property("PlatformerFeetSensor", PropertyInfo(Variant::FLOAT, "coyote_time_duration"), "set_coyote_time_duration", "get_coyote_time_duration");

}

// Main event handler function for when something enters this area
void PlatformerFeetSensor::on_body_enter(Node3D* body) {
    groundLock.lock();

    // If landed, emit signal and cancel current coyote timer
    if (numGroundSensed == 0) {
        emit_signal("landed");

        cancel_coyote_timer();
        coyoteTimeDisabled = false;
    }
    numGroundSensed++;

    groundLock.unlock();
}

// Main event handler function for when something exits this area
void PlatformerFeetSensor::on_body_exit(Node3D* body) {
    groundLock.lock();
    numGroundSensed--;

    // If no ground left to stand, cancel coyote timer if any exist and start a new timer
    if (numGroundSensed == 0) {
        cancel_coyote_timer();

        if (coyoteTimeDisabled) {
            emit_signal("fall_begin");
        } else {
            start_coyote_timer();
        }
    }

    groundLock.unlock();
}


// -----------------------------------
//  PROPERTIES
// -----------------------------------


void PlatformerFeetSensor::set_coyote_time_duration(const double duration) {
    coyoteTimeDuration = duration;
}


double PlatformerFeetSensor::get_coyote_time_duration() const {
    return coyoteTimeDuration;
}



// ----------------------------------
//   Coyote Time Functions
// ----------------------------------


// The main function to disable coyote time until the next landing
void PlatformerFeetSensor::disable_coyote_time() {
    coyoteTimeDisabled = true;
}


// Main coyote time sequence starter
void PlatformerFeetSensor::start_coyote_timer() {
    coyoteTimeLock.lock();

    // Create timer and connect
    Ref<SceneTreeTimer> coyoteTimer = get_tree()->create_timer(coyoteTimeDuration);
    coyoteTimer->connect("timeout", Callable(this, "on_coyote_timeout"));
    curCoyoteTimer = coyoteTimer;

    coyoteTimeLock.unlock();
}


// Main function to cancel current timer
void PlatformerFeetSensor::cancel_coyote_timer() {
    coyoteTimeLock.lock();

    if (curCoyoteTimer.is_valid()) {
        curCoyoteTimer->disconnect("timeout", coyoteTimeListener);
        curCoyoteTimer->set_time_left(0);
        curCoyoteTimer.unref();
    }

    coyoteTimeLock.unlock();
}


// Main function handler for coyote timeout
void PlatformerFeetSensor::on_coyote_timeout() {
    coyoteTimeLock.lock();
    groundLock.lock();

    // If coyote timer is non-null and numGroundSensed is still 0, emit signal
    if (curCoyoteTimer.is_valid() && numGroundSensed == 0) {
        emit_signal("fall_begin");
        curCoyoteTimer.unref();
    }

    groundLock.unlock();
    coyoteTimeLock.unlock();
}
