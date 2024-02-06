#include "PlatformerPackage3D.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

// To print UtilityFunctions::print()

using namespace godot;

// Function to bind methods
void PlatformerPackage3D::_bind_methods() {
    // General methods to bind
    ClassDB::bind_method(D_METHOD("relative_run"), &PlatformerPackage3D::relative_run);
    ClassDB::bind_method(D_METHOD("start_jump"), &PlatformerPackage3D::start_jump);
    ClassDB::bind_method(D_METHOD("cancel_jump"), &PlatformerPackage3D::cancel_jump);
    ClassDB::bind_method(D_METHOD("process_timers"), &PlatformerPackage3D::process_timers);

    // Listeners to bind
    ClassDB::bind_method(D_METHOD("on_landed"), &PlatformerPackage3D::on_landed);
    ClassDB::bind_method(D_METHOD("on_fall_begin"), &PlatformerPackage3D::on_fall_begin);

    // Signals to bind
    ADD_SIGNAL(MethodInfo("jump_begin"));
    ADD_SIGNAL(MethodInfo("ledge_grab_begin"));

    // Bind properties
    bind_properties();
}

// General call to bind properties from the parent bind methods
void PlatformerPackage3D::bind_properties() {
    ClassDB::bind_method(D_METHOD("get_walking_speed"), &PlatformerPackage3D::get_walking_speed);
    ClassDB::bind_method(D_METHOD("set_walking_speed"), &PlatformerPackage3D::set_walking_speed);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "walking_speed"), "set_walking_speed", "get_walking_speed");

    ClassDB::bind_method(D_METHOD("get_walking_air_reduction"), &PlatformerPackage3D::get_walking_air_reduction);
    ClassDB::bind_method(D_METHOD("set_walking_air_reduction"), &PlatformerPackage3D::set_walking_air_reduction);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "walking_air_reduction"), "set_walking_air_reduction", "get_walking_air_reduction");

    ClassDB::bind_method(D_METHOD("get_camera_node_path"), &PlatformerPackage3D::get_camera_node_path);
    ClassDB::bind_method(D_METHOD("set_camera_node_path"), &PlatformerPackage3D::set_camera_node_path);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::NODE_PATH, "camera_node_path"), "set_camera_node_path", "get_camera_node_path");

    ClassDB::bind_method(D_METHOD("get_character_body_path"), &PlatformerPackage3D::get_character_body_path);
    ClassDB::bind_method(D_METHOD("set_character_body_path"), &PlatformerPackage3D::set_character_body_path);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::NODE_PATH, "character_body_path"), "set_character_body_path", "get_character_body_path");

    ClassDB::bind_method(D_METHOD("get_player_feet_path"), &PlatformerPackage3D::get_player_feet_path);
    ClassDB::bind_method(D_METHOD("set_player_feet_path"), &PlatformerPackage3D::set_player_feet_path);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::NODE_PATH, "player_feet_path"), "set_player_feet_path", "get_player_feet_path");

    ClassDB::bind_method(D_METHOD("get_long_jump_height"), &PlatformerPackage3D::get_long_jump_height);
    ClassDB::bind_method(D_METHOD("set_long_jump_height"), &PlatformerPackage3D::set_long_jump_height);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "long_jump_height"), "set_long_jump_height", "get_long_jump_height");

    ClassDB::bind_method(D_METHOD("get_short_jump_height"), &PlatformerPackage3D::get_short_jump_height);
    ClassDB::bind_method(D_METHOD("set_short_jump_height"), &PlatformerPackage3D::set_short_jump_height);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "short_jump_height"), "set_short_jump_height", "get_short_jump_height");

    ClassDB::bind_method(D_METHOD("get_player_gravity"), &PlatformerPackage3D::get_player_gravity);
    ClassDB::bind_method(D_METHOD("set_player_gravity"), &PlatformerPackage3D::set_player_gravity);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "player_gravity"), "set_player_gravity", "get_player_gravity");

    ClassDB::bind_method(D_METHOD("get_gravity_apex_modifier"), &PlatformerPackage3D::get_gravity_apex_modifier);
    ClassDB::bind_method(D_METHOD("set_gravity_apex_modifier"), &PlatformerPackage3D::set_gravity_apex_modifier);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "gravity_apex_modifier"), "set_gravity_apex_modifier", "get_gravity_apex_modifier");

    ClassDB::bind_method(D_METHOD("get_apex_speed_definition"), &PlatformerPackage3D::get_apex_speed_definition);
    ClassDB::bind_method(D_METHOD("set_apex_speed_definition"), &PlatformerPackage3D::set_apex_speed_definition);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "apex_speed_definition"), "set_apex_speed_definition", "get_apex_speed_definition");

    ClassDB::bind_method(D_METHOD("get_ledge_grab_horizontal_reach"), &PlatformerPackage3D::get_ledge_grab_horizontal_reach);
    ClassDB::bind_method(D_METHOD("set_ledge_grab_horizontal_reach"), &PlatformerPackage3D::set_ledge_grab_horizontal_reach);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "ledge_grab_horizontal_reach"), "set_ledge_grab_horizontal_reach", "get_ledge_grab_horizontal_reach");

    ClassDB::bind_method(D_METHOD("get_ledge_grab_vertical_reach"), &PlatformerPackage3D::get_ledge_grab_vertical_reach);
    ClassDB::bind_method(D_METHOD("set_ledge_grab_vertical_reach"), &PlatformerPackage3D::set_ledge_grab_vertical_reach);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "ledge_grab_vertical_reach"), "set_ledge_grab_vertical_reach", "get_ledge_grab_vertical_reach");

    ClassDB::bind_method(D_METHOD("get_auto_grab_vertical_offset"), &PlatformerPackage3D::get_auto_grab_vertical_offset);
    ClassDB::bind_method(D_METHOD("set_auto_grab_vertical_offset"), &PlatformerPackage3D::set_auto_grab_vertical_offset);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "auto_grab_vertical_offset"), "set_auto_grab_vertical_offset", "get_auto_grab_vertical_offset");

    ClassDB::bind_method(D_METHOD("get_max_wall_grab_angle_requirement"), &PlatformerPackage3D::get_max_wall_grab_angle_requirement);
    ClassDB::bind_method(D_METHOD("set_max_wall_grab_angle_requirement"), &PlatformerPackage3D::set_max_wall_grab_angle_requirement);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "max_wall_grab_angle_requirement"), "set_max_wall_grab_angle_requirement", "get_max_wall_grab_angle_requirement");

    ClassDB::bind_method(D_METHOD("get_max_wall_grab_vertical_speed"), &PlatformerPackage3D::get_max_wall_grab_vertical_speed);
    ClassDB::bind_method(D_METHOD("set_max_wall_grab_vertical_speed"), &PlatformerPackage3D::set_max_wall_grab_vertical_speed);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "max_wall_grab_vertical_speed"), "set_max_wall_grab_vertical_speed", "get_max_wall_grab_vertical_speed");

    ClassDB::bind_method(D_METHOD("get_jump_buffer_duration"), &PlatformerPackage3D::get_jump_buffer_duration);
    ClassDB::bind_method(D_METHOD("set_jump_buffer_duration"), &PlatformerPackage3D::set_jump_buffer_duration);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "jump_buffer_duration"), "set_jump_buffer_duration", "get_jump_buffer_duration");
}

// General constructor
PlatformerPackage3D::PlatformerPackage3D() {
    // Starting grounded movement
    walking_speed = 10;
    currentGroundInputDirection = Vector3(0, 0, 0);
    currentGroundMovement = Vector3(0, 0, 0);
    walking_acceleration = 500;
    walking_deceleration = 350;
    immediate_stop_speed = 5;

    // Starting vertical movement
    longJumpHeight = 6;
    shortJumpHeight = 3;
    playerGravity = 5;
    gravityApexModifier = 0.5;
    apexSpeedDefinition = 1.5;
    maxFallSpeed = 10;

    grounded = false;
    currentVerticalSpeed = 0;

    // Main jump buffer
    jumpBufferDuration = 0.01;
    jumpBufferTimer = new BufferTimer(jumpBufferDuration);

    // Ledge grab variables
    grabbingLedge = false;
    ledgeGrabHorizontalReach = 0.5;
    ledgeGrabVerticalReach = 0.2;
    maxWallGrabVerticalSpeed = 1;
    maxWallGrabAngleRequirement = 10;
    autoGrabVerticalOffset = 0.3;

    // Reference nodes (MUST BE SET TO NULL ON CONSTRUCTION)
    current_camera = nullptr;
    character_body = nullptr;
    player_feet = nullptr;

}


// Main function to clean up
PlatformerPackage3D::~PlatformerPackage3D() {
    delete jumpBufferTimer;
}


// Main function that plays on game start
void PlatformerPackage3D::_ready() {
    initialize_current_node_pointers();
    currentVerticalSpeed = 0;
}


// On frame update
void PlatformerPackage3D::process_timers(double delta) {
    if (!Engine::get_singleton()->is_editor_hint()) {
        jumpBufferTimer->update(delta);
    }
}


// Main function that plays for each physics frame
void PlatformerPackage3D::_physics_process(double delta) {
    // Check if you're in editor mode or in game mode. If in game mode, actually run it
    if (!Engine::get_singleton()->is_editor_hint()) {
        // Set the velocity and then move
        Vector3 velocityVector = calculate_horizontal_velocity(delta) + calculate_vertical_velocity(delta);

        // Move the enemy forward
        set_velocity(velocityVector);
        move_and_slide();

        // After moving, check for wall if you're on a wall ANND falling AND vertical speed < maxWallGrabVerticalSpeed
        if (can_interact_with_wall()) {
            if (!grabbingLedge) {
                handle_ledge_grab();
            }
        }
    }
}


// Main function to start a jump
void PlatformerPackage3D::start_jump() {
    // If already grounded, jump immediately
    if (grounded || grabbingLedge) {
        launch_jump(longJumpHeight);

    // Else, activate the buffer timer and initialize
    } else {
        jumpBufferTimer->reset();
        bufferedJumpHeight = longJumpHeight;
    }
}


// Main function to cancel a jump in midair 
void PlatformerPackage3D::cancel_jump() {
    if (!grounded) {
        double shortJumpVelocity = calculate_starting_jump_velocity(playerGravity, shortJumpHeight);
        currentVerticalSpeed = Math::min(currentVerticalSpeed, shortJumpVelocity);

        // If you were in jump buffering stage, lower buffer jump height to the canceled jump height 
        if (jumpBufferTimer->isRunning()) {
            bufferedJumpHeight = shortJumpHeight;
        }
    }
}


// Main function to launch a jump upon starting a jump
void PlatformerPackage3D::launch_jump(double jumpHeight) {
    currentVerticalSpeed = calculate_starting_jump_velocity(playerGravity, jumpHeight);
    grabbingLedge = false;
    emit_signal("jump_begin");
}


// Main function to move a certain direction given a controller vector
void PlatformerPackage3D::relative_run(Vector2 controller_vector, double time_delta) {
    // Get world directions based on current camera (forward = current_camera.transform.basis.z)
    Vector3 world_forward = -current_camera->get_global_transform().get_basis().get_column(2);
    Vector3 up_vector = Vector3(0, 1, 0);
    Vector3 world_right = world_forward.cross(up_vector);

    // Project vectors on a flat plane
    Vector3 origin = Vector3(0, 0, 0);
    Plane surface_movement_plane = Plane(up_vector, origin);
    world_forward = surface_movement_plane.project(world_forward).normalized();
    world_right = surface_movement_plane.project(world_right).normalized();

    // Change the current horizontal direction. Normalize it if it's not the zero vector
    currentGroundInputDirection = (controller_vector.x * world_right) + (controller_vector.y * world_forward);
    if (!is_zero(controller_vector.x) || !is_zero(controller_vector.y)) {
        currentGroundInputDirection.normalize();
    }
}


// Event handler for when the unit lands on the ground
void PlatformerPackage3D::on_landed() {
    // If jump buffering still active, just launch jump and cancel jump
    if (jumpBufferTimer->isRunning()) {
        launch_jump(Math::max(bufferedJumpHeight, shortJumpHeight + 0.5));
        jumpBufferTimer->cancel();

    // Else, just land on the ground
    } else {
        currentVerticalSpeed = 0;
        grounded = true;
    }
}


// Event handler for when the unit begins to fall
void PlatformerPackage3D::on_fall_begin() {
    grounded = false;
}


// Main helper function to handle ledge grab
//  Pre: assumes that is_on_wall() is true
void PlatformerPackage3D::handle_ledge_grab() {
    Vector3 wallNormal = get_wall_normal();
    double halfColliderHeight = 0.5 * get_collider_shape_height();
    double colliderRadius = get_collider_shape_radius();

    Vector3 verticalLedgeGrabLimitPosition = get_global_position() + Vector3(0, halfColliderHeight + ledgeGrabVerticalReach, 0);
    Vector3 maximumLedgeFrabReachPosition = verticalLedgeGrabLimitPosition + (-wallNormal * (ledgeGrabHorizontalReach + colliderRadius));

    // If nothing is in between the grab limit positions, then it's plausible to grab a ledge
    Dictionary rayLimitResults = cast_ray(verticalLedgeGrabLimitPosition, maximumLedgeFrabReachPosition);
    if (rayLimitResults.is_empty()) {
        // From maximum ledge grab reach position, move down to see if you're able to hit a ledge
        Vector3 lowerledgeGrabHorizontalReachPosition = maximumLedgeFrabReachPosition + (Vector3(0, -(halfColliderHeight + ledgeGrabVerticalReach), 0));
        Dictionary rayCollision = cast_ray(maximumLedgeFrabReachPosition, lowerledgeGrabHorizontalReachPosition);
        grabbingLedge = !rayCollision.is_empty();

        // If grabbing ledge, calculate ledge position and autosnap to that position
        if (grabbingLedge) {
            emit_signal("ledge_grab_begin");
            Vector3 ledgePosition = rayCollision["position"];
            ledgePosition += Vector3(0, -0.0001, 0);

            Vector3 playerLedgePosition = Vector3(get_global_position().x, ledgePosition.y, get_global_position().z);
            Dictionary ledgeCollision = cast_ray(playerLedgePosition, rayCollision["position"]);
            snap_to_ledge(ledgeCollision["position"], wallNormal);
        }

    } else {
        grabbingLedge = false;
    }
}


// Main private helper function to calculate the vertical component of velocity
Vector3 PlatformerPackage3D::calculate_vertical_velocity(double delta) {
    // Update currentVerticalSpeed based on gravity if not grounded
    if (!grounded && !grabbingLedge) {
        bool inApex = (currentVerticalSpeed > -apexSpeedDefinition && currentVerticalSpeed < apexSpeedDefinition);
        double curGravity = (inApex) ? gravityApexModifier * playerGravity : playerGravity;
        currentVerticalSpeed -= (curGravity * delta);
    }

    // Put fall velocity in a vector
    return Vector3(0, Math::max(currentVerticalSpeed, -maxFallSpeed), 0);
}


// Main private helper function to calculate the XZ component of velocity
Vector3 PlatformerPackage3D::calculate_horizontal_velocity(double delta) {
    double maxRunningSpeed = walking_speed;

    // If you're actually moving, adjust currentGroundMovement
    if (currentGroundInputDirection.length() > 0.01f && !grabbingLedge) {
        // If you're going really fast or you going in the same general direction as curGroundMovement, accelerate in that direction
        if (currentGroundMovement.length() > immediate_stop_speed || Math::rad_to_deg(currentGroundMovement.angle_to(currentGroundInputDirection)) < 60) {
            // Adjust currentGroundMovement
            currentGroundMovement += (delta * delta * walking_acceleration * currentGroundInputDirection);

            // If accelerating past maxSpeed, restrict it
            if (currentGroundMovement.length() > maxRunningSpeed) {
                currentGroundMovement = maxRunningSpeed * currentGroundMovement.normalized();
            }

        // Else, you can just make a hard turn
        } else {
            currentGroundMovement = (delta * delta * walking_acceleration * currentGroundInputDirection);
        }

        // Rotate
        character_body->look_at(get_global_position() + currentGroundMovement);

    // If you're not moving, decelerate
    } else {
        // If you can immediately stop. If you can't decelerate.
        if (currentGroundMovement.length() > immediate_stop_speed) {
            double targetSpeed = Math::max(currentGroundMovement.length() - (delta * delta * walking_deceleration), 0.0);
            currentGroundMovement = targetSpeed * currentGroundMovement.normalized();

        } else {
            currentGroundMovement = Vector3(0, 0, 0);
        }
    }

    Vector3 curHorizontalDelta = (grounded) ? currentGroundMovement : walking_air_reduction * currentGroundMovement;
    return player_feet->project_movement_on_ground(curHorizontalDelta);
}


// Main function to snap to a ledge
//  ledgePosition is the global position of the ledge
void PlatformerPackage3D::snap_to_ledge(Vector3 ledgePosition, Vector3 wallNormal) {
    Vector3 positionToSnapTo = ledgePosition + (wallNormal * (get_collider_shape_radius() + 0.01)) + Vector3(0, -autoGrabVerticalOffset, 0);
    set_global_position(positionToSnapTo);
    character_body->look_at(get_global_position() - wallNormal);

    set_velocity(Vector3(0, 0, 0));
    currentVerticalSpeed = 0;
}


// Main function to calculate the starting jump velocity given gravity and targeted height
double PlatformerPackage3D::calculate_starting_jump_velocity(double curGravity, double targetedHeight) {
    return Math::sqrt(2.0 * curGravity * targetedHeight);
}

// Main function to check if you can grab the current wall
bool PlatformerPackage3D::can_interact_with_wall() {
    // Check if there's a wall to interact with. if not, you can't interact with anything
    if (is_on_wall()) {
        // Get the angle of the wall (current wall from an extremely steep wall)
        Plane surface_movement_plane = Plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
        Vector3 wallNormal = get_wall_normal();
        Vector3 steepestWallNormal = surface_movement_plane.project(wallNormal);

        // You can only interact if wall is a cliff (not an cavern or a sliding mountain)
        bool canInteract = (wallNormal.y <= steepestWallNormal.y) || (wallNormal.angle_to(steepestWallNormal) < maxWallGrabAngleRequirement);

        return !grounded && currentVerticalSpeed <= maxWallGrabVerticalSpeed && canInteract;

    } else {
        return false;
    }
}


// Main helper function to check if a number is close enough to zero
bool PlatformerPackage3D::is_zero(double num) {
    return (num > -0.001) && (num < 0.001);
}


// Main helper function to cast a ray
Dictionary PlatformerPackage3D::cast_ray(Vector3 from, Vector3 to) {
    PhysicsDirectSpaceState3D* spaceState = get_world_3d()->get_direct_space_state();
    Ref<PhysicsRayQueryParameters3D> rayCastQuery = PhysicsRayQueryParameters3D::create(
        from,
        to,
        get_collision_mask()
    );
    return spaceState->intersect_ray(rayCastQuery);
}


// Main shape accessor functions
double PlatformerPackage3D::get_collider_shape_height() {
    Ref<Shape3D> mainColliderShape = shape_owner_get_shape(0, 0);
    Ref<CapsuleShape3D> mainCapsuleCollider = mainColliderShape;

    return mainCapsuleCollider->get_height();
}

double PlatformerPackage3D::get_collider_shape_radius() {
    Ref<Shape3D> mainColliderShape = shape_owner_get_shape(0, 0);
    Ref<CapsuleShape3D> mainCapsuleCollider = mainColliderShape;

    return mainCapsuleCollider->get_radius();
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


// Air reduction on walking
void PlatformerPackage3D::set_walking_air_reduction(const double reduction) {
    walking_air_reduction = reduction;
}

double PlatformerPackage3D::get_walking_air_reduction() const {
    return walking_air_reduction;
}


// Camera Node Path setter / getter
void PlatformerPackage3D::set_camera_node_path(const NodePath p_node_path) {
    camera_node_path = p_node_path;
}

NodePath PlatformerPackage3D::get_camera_node_path() const {
    return camera_node_path;
}


// Character Node Path Setter / Getter
void PlatformerPackage3D::set_character_body_path(const NodePath p_node_path) {
    character_body_path = p_node_path;
}

NodePath PlatformerPackage3D::get_character_body_path() const {
    return character_body_path;
}


// Player feet property
void PlatformerPackage3D::set_player_feet_path(const NodePath p_node_path) {
    player_feet_path = p_node_path;
}

NodePath PlatformerPackage3D::get_player_feet_path() const {
    return player_feet_path;
}


void PlatformerPackage3D::initialize_current_node_pointers() {
    if (camera_node_path != nullptr) {
        current_camera = get_node<Camera3D>(camera_node_path);
    }

    if (character_body_path != nullptr) {
        character_body = get_node<Node3D>(character_body_path);
    }

    if (player_feet_path != nullptr) {
        player_feet = get_node<PlatformerFeetSensor>(player_feet_path);
    }
}



// Player base jump height
void PlatformerPackage3D::set_long_jump_height(const double jump_height) {
    longJumpHeight = jump_height;
}

double PlatformerPackage3D::get_long_jump_height() const {
    return longJumpHeight;
}


// Player short jump height when player cancels jump
void PlatformerPackage3D::set_short_jump_height(const double jump_height) {
    shortJumpHeight = jump_height;
}

double PlatformerPackage3D::get_short_jump_height() const {
    return shortJumpHeight;
}

// Player downward acceleration
void PlatformerPackage3D::set_player_gravity(const double gravity) {
    playerGravity = gravity;
}

double PlatformerPackage3D::get_player_gravity() const {
    return playerGravity;
}

// Player apex modifier
void PlatformerPackage3D::set_gravity_apex_modifier(const double modifier) {
    gravityApexModifier = modifier;
}

double PlatformerPackage3D::get_gravity_apex_modifier() const {
    return gravityApexModifier;
}


// Player apex definition: how fast would the player be to be considered at the apex of a jump
//  between -APEX and APEX
void PlatformerPackage3D::set_apex_speed_definition(const double speed) {
    apexSpeedDefinition = speed;
}

double PlatformerPackage3D::get_apex_speed_definition() const {
    return apexSpeedDefinition;
}


// Set jump buffer
void PlatformerPackage3D::set_jump_buffer_duration(const double duration) {
    jumpBufferDuration = duration;
    jumpBufferTimer->setMaxDuration(duration);
}

double PlatformerPackage3D::get_jump_buffer_duration() const {
    return jumpBufferDuration;
}


// -------------------------------
// Ledge grab properties
// -------------------------------

void PlatformerPackage3D::set_ledge_grab_horizontal_reach(const double p_value) {
    ledgeGrabHorizontalReach = p_value;
}

double PlatformerPackage3D::get_ledge_grab_horizontal_reach() const {
    return ledgeGrabHorizontalReach;
}


void PlatformerPackage3D::set_ledge_grab_vertical_reach(const double p_value) {
    ledgeGrabVerticalReach = p_value;
}

double PlatformerPackage3D::get_ledge_grab_vertical_reach() const {
    return ledgeGrabVerticalReach;
}


void PlatformerPackage3D::set_auto_grab_vertical_offset(const double p_value) {
    autoGrabVerticalOffset = p_value;
}

double PlatformerPackage3D::get_auto_grab_vertical_offset() const {
    return autoGrabVerticalOffset;
}


// -------------------------------
// Wall interaction properties
// -------------------------------

void PlatformerPackage3D::set_max_wall_grab_angle_requirement(const double p_value) {
    maxWallGrabAngleRequirement = p_value;
}

double PlatformerPackage3D::get_max_wall_grab_angle_requirement() const {
    return maxWallGrabAngleRequirement;
}


void PlatformerPackage3D::set_max_wall_grab_vertical_speed(const double p_value) {
    maxWallGrabVerticalSpeed = p_value;
}

double PlatformerPackage3D::get_max_wall_grab_vertical_speed() const {
    return maxWallGrabVerticalSpeed;
}
