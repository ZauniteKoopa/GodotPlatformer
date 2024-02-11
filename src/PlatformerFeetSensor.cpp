#include "PlatformerFeetSensor.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// Main constructor
PlatformerFeetSensor::PlatformerFeetSensor() {
    coyoteTimeDuration = 2.0;

    curCoyoteTimer.unref();
    coyoteTimeListener = Callable(this, "on_coyote_timeout");
    coyoteTimeDisabled = false;

    floorMaxAngle = 45;
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

    // Set up properties
    ClassDB::bind_method(D_METHOD("get_floor_max_angle"), &PlatformerFeetSensor::get_floor_max_angle);
    ClassDB::bind_method(D_METHOD("set_floor_max_angle"), &PlatformerFeetSensor::set_floor_max_angle);
    ClassDB::add_property("PlatformerFeetSensor", PropertyInfo(Variant::FLOAT, "floor_max_angle"), "set_floor_max_angle", "get_floor_max_angle");

}

// Main event handler function for when something enters this area
void PlatformerFeetSensor::on_body_enter(Node3D* body) {
    // Check collision normal. If within floor threshold, then count it as ground
    Vector3 collisionNormal = get_collision_normal(body);
    Vector3 upVector = Vector3(0, 1, 0);
    double curAngle = Math::rad_to_deg(collisionNormal.angle_to(upVector));

    // Only increment counter if ground is actually valid ground
    if (curAngle <= floorMaxAngle) {
        groundLock.lock();

        // If landed, emit signal and cancel current coyote timer
        if (groundSensed.size() == 0) {
            emit_signal("landed");

            cancel_coyote_timer();
            coyoteTimeDisabled = false;
        }
        groundSensed.insert(body);

        groundLock.unlock();
    }
}

// Main event handler function for when something exits this area
void PlatformerFeetSensor::on_body_exit(Node3D* body) {
    // Check collision normal. If within floor threshold, then count it as ground
    Vector3 collisionNormal = get_collision_normal(body);
    Vector3 upVector = Vector3(0, 1, 0);
    double curAngle = Math::rad_to_deg(collisionNormal.angle_to(upVector));

    // Only decrement counter if ground is actually valid ground (THIS WILL BE A BUG IF THE GROUND YOU'RE STANDING ON TURNS INTO A WALL LIKE WALKING OFF A STEEP SLOPE.)
    groundLock.lock();

    if (groundSensed.find(body) != groundSensed.end()) {
        groundSensed.erase(body);

        // If no ground left to stand, cancel coyote timer if any exist and start a new timer
        if (groundSensed.size() == 0) {
            cancel_coyote_timer();

            if (coyoteTimeDisabled) {
                emit_signal("fall_begin");
            } else {
                start_coyote_timer();
            }
        }
    }

    groundLock.unlock();
}


// Main function to project movement on current ground plane
Vector3 PlatformerFeetSensor::project_movement_on_ground(Vector3 velocity) {
    // Get the ground plane
    Vector3 ground_normal = get_overall_ground_normal();
    Plane ground_plane = Plane(ground_normal, Vector3(0, 0, 0));

    // Do calculations based on plane projection
    double speed = velocity.length();
    Vector3 projectedVelocity = ground_plane.project(velocity).normalized();
    return projectedVelocity * speed;
}


// Main helper function to get the ground normal, MUST BE ON PHYSICS UPDATE
Vector3 PlatformerFeetSensor::get_overall_ground_normal() {
    // Check if you're grounded
    int tempNumGround;
    Vector3 upVector = Vector3(0, 1, 0);

    groundLock.lock();
    tempNumGround = groundSensed.size();
    groundLock.unlock();

    // If grounded, apply raycast check. else, return default normal (Vector3.up)
    if (tempNumGround > 0) {
        // Get all overlapping bodies. Start from floor max angle as the starting min
        TypedArray<Node3D> overlappingGround = get_overlapping_bodies();
        double minAngle = floorMaxAngle + 1;
        Vector3 answer = upVector;

        for (int i = 0; i < overlappingGround.size(); i++) {
            // Get current ground
            Object* g = overlappingGround[i];
            Node3D* curGround = Object::cast_to<Node3D>(g);

            // Get collision normal and compare to min angle. Always pick the normal with the smallest angle from UP
            Vector3 curNormal = get_collision_normal(curGround);
            double curAngle = Math::rad_to_deg(curNormal.angle_to(upVector));
            if (curAngle < minAngle) {
                minAngle = curAngle;
                answer = curNormal;
            }
        }

        return answer;
    } else {
        return upVector;
    }
}


// Main function to get the normal of one collision
Vector3 PlatformerFeetSensor::get_collision_normal(Node3D* collidedGround) {
    // Set up query and fire ray cast
    PhysicsDirectSpaceState3D* spaceState = get_world_3d()->get_direct_space_state();
    Ref<PhysicsRayQueryParameters3D> rayCastQuery = PhysicsRayQueryParameters3D::create(
        get_parent_node_3d()->get_global_position(),
        collidedGround->get_global_position()
    );

    Dictionary rayResult = spaceState->intersect_ray(rayCastQuery);

    // If you hit something, return the normal of that collision. Else, return the upVector
    if (rayResult.is_empty()) {
        return Vector3(0, 1, 0);
    } else {
        return rayResult["normal"];
    }
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


void PlatformerFeetSensor::set_floor_max_angle(const double angle) {
    floorMaxAngle = angle;
}


double PlatformerFeetSensor::get_floor_max_angle() const {
    return floorMaxAngle;
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

    // If coyote timer is non-null and num GroundSensed is still 0, emit signal
    if (curCoyoteTimer.is_valid() && groundSensed.size() == 0) {
        emit_signal("fall_begin");
        curCoyoteTimer.unref();
    }

    groundLock.unlock();
    coyoteTimeLock.unlock();
}
