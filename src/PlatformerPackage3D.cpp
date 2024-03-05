#include "PlatformerPackage3D.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
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
    ClassDB::bind_method(D_METHOD("dash"), &PlatformerPackage3D::dash);
    ClassDB::bind_method(D_METHOD("get_current_vertical_speed"), &PlatformerPackage3D::get_current_vertical_speed);

    // Listeners to bind
    ClassDB::bind_method(D_METHOD("on_landed"), &PlatformerPackage3D::on_landed);
    ClassDB::bind_method(D_METHOD("on_fall_begin"), &PlatformerPackage3D::on_fall_begin);
    ClassDB::bind_method(D_METHOD("on_speed_force_expire"), &PlatformerPackage3D::on_speed_force_expire);
    ClassDB::bind_method(D_METHOD("on_dash_regained"), &PlatformerPackage3D::on_dash_regained);

    // Animator accessors
    ClassDB::bind_method(D_METHOD("is_grounded"), &PlatformerPackage3D::is_grounded);
    ClassDB::bind_method(D_METHOD("is_dashing"), &PlatformerPackage3D::is_dashing);
    ClassDB::bind_method(D_METHOD("is_wall_grabbing"), &PlatformerPackage3D::is_wall_grabbing);
    ClassDB::bind_method(D_METHOD("is_ledge_grabbing"), &PlatformerPackage3D::is_ledge_grabbing);
    ClassDB::bind_method(D_METHOD("is_skidding"), &PlatformerPackage3D::is_skidding);
    ClassDB::bind_method(D_METHOD("get_current_horizontal_speed"), &PlatformerPackage3D::get_current_horizontal_speed);

    // Signals to bind
    ADD_SIGNAL(MethodInfo("jump_begin"));
    ADD_SIGNAL(MethodInfo("ledge_grab_begin"));
    ADD_SIGNAL(MethodInfo("dash_started"));

    // Bind properties
    bind_properties();
}

// General constructor
PlatformerPackage3D::PlatformerPackage3D() {
    // Starting grounded movement
    max_walking_speed = 10;
    starting_walking_speed = 3;
    currentGroundInputDirection = Vector3(0, 0, 0);
    currentGroundMovement = Vector3(0, 0, 0);
    walking_acceleration = 750;
    walking_deceleration = 400;
    immediate_stop_speed = 3;

    // Starting vertical movement
    longJumpHeight = 2.5;
    skidJumpHeight = 3.5;
    shortJumpHeight = 1;
    extraJumpHeight = 1;

    maxExtraJumps = 1;
    curExtraJumpsDone = 0;
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

    // Applied forces
    appliedSpeedDecayRate = 20;
    appliedSpeedVector = Vector3(0, 0, 0);
    appliedSpeedActive = false;
    appliedSpeedTimeoutListener = Callable(this, "on_speed_force_expire");

    // Dashing
    dashCooldownListener = Callable(this, "on_dash_regained");

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
void PlatformerPackage3D::_enter_tree() {
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
            // Check if you can grabbing ledge
            if (!grabbingLedge) {
                handle_ledge_grab();
            }

            // If you didn't grab the ledge, slide on the wall
            grabbingWall = !grabbingLedge;
            dashing = false;
            if (!grabbingLedge) {
                currentGroundMovement = Vector3(0, 0, 0);
            }

        // Else, set grabbingWall to false
        } else {
            grabbingWall = false;
        }

        // Decay applied forces if not active anymore
        appliedSpeedLock.lock();
        if (!appliedSpeedActive && appliedSpeedVector.length() > 0.001) {
            double newAppliedSpeed = Math::max(0.0, appliedSpeedVector.length() - (appliedSpeedDecayRate * delta));
            appliedSpeedVector = appliedSpeedVector.normalized() * newAppliedSpeed;
        }
        appliedSpeedLock.unlock();
    }
}


// Main function to start a jump
void PlatformerPackage3D::start_jump() {
    // If already grounded, jump immediately
    if (grounded || grabbingLedge) {
        double curJumpHeight = (grounded && is_skidding()) ? skidJumpHeight : longJumpHeight;
        launch_jump(curJumpHeight);

    // If grabbing wall, do a wall jump
    } else if (grabbingWall) {
        apply_speed_force(calculate_wall_jump_force(currentGroundInputDirection), wallJumpSpeedDuration);
        launch_jump(wallJumpHeight);

    // If you have extra jumps left, launch_jump and increment
    } else if (curExtraJumpsDone < maxExtraJumps && !doesBufferOverrideExtraJump()) {
        launch_jump(extraJumpHeight);
        cancel_speed_force();
        currentGroundMovement = Vector3(0, 0, 0);
        curExtraJumpsDone++;

    // Else, activate the buffer timer and initialize
    } else {
        jumpBufferTimer->reset();

        // jump height is based on current consecutive ground jump
        currentConsecutiveGroundJump = (currentConsecutiveGroundJump + 1) % maxConsecutiveGroundJumps;
        double groundJumpHeight = longJumpHeight + (groundJumpHeightIncrease * currentConsecutiveGroundJump);
        bufferedJumpHeight = groundJumpHeight;
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


// Helper function to check if  jumpBuffer overrides extraJump (you jump on the ground via jump buffer instead of doing the extra jump)
bool PlatformerPackage3D::doesBufferOverrideExtraJump() {
    // Raycast underneath given that you know how fast you're going and how long the jump buffer timer is. If it hits something, let jump buffer take over. Else, apply extra jump
    Vector3 raySrc = player_feet->get_global_position();
    Vector3 rayDest = raySrc + (currentVerticalSpeed * jumpBufferDuration) * Vector3(0, 1, 0);
    Dictionary rayHitInfo = cast_ray(raySrc, rayDest);

    return !rayHitInfo.is_empty();
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


// Main function to dash if you're able to dash 
void PlatformerPackage3D::dash() {
    dashLock.lock();

    if (canDash && (grounded || curDashesUsed < maxNumAirDashes)) {
        // Clear ground and vertical movement and set up dash dir to either the direction you're moving or the forward
        currentGroundMovement = Vector3(0, 0, 0);
        currentVerticalSpeed = 0;
        Vector3 dashDir = (currentGroundInputDirection.length() < 0.01) ? -character_body->get_global_transform().get_basis().get_column(2) : currentGroundInputDirection.normalized();

        // Apply speed force
        apply_speed_force(dashDir * dashSpeed, get_dash_duration());
        emit_signal("dash_started");

        // Set up flags and timers
        if (!grounded) {
            curDashesUsed++;
        }

        // Set flags and timers
        canDash = false;
        dashing = true;
        dashCooldownTimer = get_tree()->create_timer(get_dash_duration() + timeBetweenDashes);
        dashCooldownTimer->connect("timeout", dashCooldownListener);
    }

    dashLock.unlock();
}


// Main function to apply speed force
void PlatformerPackage3D::apply_speed_force(Vector3 speedForceVector, double duration) {
    cancel_speed_force();

    appliedSpeedLock.lock();

    // Set speed
    appliedSpeedVector = speedForceVector;
    appliedSpeedActive = true;

    // Create timer and connect
    appliedSpeedTimer = get_tree()->create_timer(duration);
    appliedSpeedTimer->connect("timeout", appliedSpeedTimeoutListener);
    
    appliedSpeedLock.unlock();
}


// Main helper function to calculate the wall jump normal
//  Pre: player exists on wall, playerWorldInput is the direction the player wants to go to in the world, flatten in a flat plane
Vector3 PlatformerPackage3D::calculate_wall_jump_force(Vector3 playerWorldInput) {
    // Get wall vectors
    Vector3 wallNormal = get_wall_normal();
    Vector3 wallSurface = wallNormal.cross(Vector3(0, 1, 0));

    // Get dot product to see if player is moving towards the wall when jumping. If so, bounce it so vector faces wall normal
    double dotProduct = playerWorldInput.dot(wallNormal);
    if (dotProduct < 0.0) {
        playerWorldInput = playerWorldInput.bounce(wallNormal);
    }

    // Clamp the angle and calculate the 2 possible direction vectors
    double targetRads = Math::min(Math::deg_to_rad(maxWallJumpAngleVariance), (double)playerWorldInput.angle_to(wallNormal));
    Vector3 vectorCandidate1 = (Math::cos(targetRads) * wallNormal) + (Math::sin(targetRads) * wallSurface);
    Vector3 vectorCandidate2 = (Math::cos(targetRads) * wallNormal) - (Math::sin(targetRads) * wallSurface);

    // Pick which canidate is correct by seeing which one is angled closer (higher dot product)
    if (vectorCandidate1.dot(playerWorldInput) > vectorCandidate2.dot(playerWorldInput)) {
        return wallJumpSpeedForceMagnitude * vectorCandidate1;
    } else {
        return wallJumpSpeedForceMagnitude * vectorCandidate2;
    }

}


// Main function to cancel speed force
void PlatformerPackage3D::cancel_speed_force() {
    appliedSpeedLock.lock();

    // Set applied speed to 0
    appliedSpeedVector = Vector3(0, 0, 0);
    appliedSpeedActive = false;

    // Disable timer if it's still running
    if (appliedSpeedTimer.is_valid()) {
        appliedSpeedTimer->disconnect("timeout", appliedSpeedTimeoutListener);
        appliedSpeedTimer->set_time_left(0);
        appliedSpeedTimer.unref();
    }

    appliedSpeedLock.unlock();
}

// Main event handler when speed force expires
void PlatformerPackage3D::on_speed_force_expire() {
    appliedSpeedLock.lock();

    // Set flag off
    appliedSpeedActive = false;

    // Unref the timer
    if (appliedSpeedTimer.is_valid()) {
        appliedSpeedTimer.unref();
    }

    // Set dashing flag to false for animation (if you were wall jumping, dashing would've always been false)
    dashing = false;
    appliedSpeedLock.unlock();
}


// Main event handler when dash cooldown ends
void PlatformerPackage3D::on_dash_regained() {
    dashLock.lock();

    // set up flag
    canDash = true;

    // Unref dash cooldown timer
    if (dashCooldownTimer.is_valid()) {
        dashCooldownTimer.unref();
    }

    dashLock.unlock();
}


// Event handler for when the unit lands on the ground
void PlatformerPackage3D::on_landed() {
    // Reset curExtraJumpsDone each time
    curExtraJumpsDone = 0;

    // reset dashing
    dashLock.lock();
    curDashesUsed = 0;
    dashLock.unlock();

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
    double curMaxSpeed = (grabbingWall) ? maxWallGrabFallSpeed : maxFallSpeed;
    return Vector3(0, Math::max(currentVerticalSpeed, -curMaxSpeed), 0);
}


// Main private helper function to calculate the XZ component of velocity
Vector3 PlatformerPackage3D::calculate_horizontal_velocity(double delta) {
    // If grabbing wall, return zero vector
    if (grabbingWall) {
        return Vector3(0, 0, 0);
    }

    double maxRunningSpeed = max_walking_speed;

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
            currentGroundMovement = starting_walking_speed * currentGroundInputDirection;
        }

        // Rotate
        Plane surfacePlane = Plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
        character_body->look_at(character_body->get_global_position() + surfacePlane.project((currentGroundMovement + appliedSpeedVector).normalized()));

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
    curHorizontalDelta += appliedSpeedVector;
    return player_feet->project_movement_on_ground(curHorizontalDelta);
}


// Main function to snap to a ledge
//  ledgePosition is the global position of the ledge
void PlatformerPackage3D::snap_to_ledge(Vector3 ledgePosition, Vector3 wallNormal) {
    Vector3 positionToSnapTo = ledgePosition + (wallNormal * (get_collider_shape_radius() + 0.01)) + Vector3(0, -autoGrabVerticalOffset, 0);
    Plane flatPlane = Plane(Vector3(0, 1, 0), Vector3(0, 0, 0));

    set_global_position(positionToSnapTo);
    character_body->look_at(character_body->get_global_position() - flatPlane.project(wallNormal));
    currentGroundMovement = Vector3(0, 0, 0);

    set_velocity(Vector3(0, 0, 0));
    cancel_speed_force();
    currentVerticalSpeed = 0;
    dashing = false;
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

// Main function to get current falling speed
double PlatformerPackage3D::get_current_vertical_speed() const {
    return (grounded) ? 0 : currentVerticalSpeed;
}


// Main helper function to check if a number is close enough to zero
bool PlatformerPackage3D::is_zero(double num) {
    return (num > -0.001) && (num < 0.001);
}


// Main helper function to access dash speed
double PlatformerPackage3D::get_dash_duration() {
    return dashDistance / dashSpeed;
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


// Main definition on whether or not the payer is skidding or not
bool PlatformerPackage3D::is_skidding() const {
    return Math::rad_to_deg(currentGroundInputDirection.angle_to(currentGroundMovement + appliedSpeedVector)) > 120;
}


bool PlatformerPackage3D::is_grounded() const {
    return grounded;
}


bool PlatformerPackage3D::is_dashing() const {
    return dashing;
}


bool PlatformerPackage3D::is_wall_grabbing() const {
    return grabbingWall;
}


bool PlatformerPackage3D::is_ledge_grabbing() const {
    return grabbingLedge;
}


double PlatformerPackage3D::get_current_horizontal_speed() const {
    return currentGroundMovement.length();
}




// ------------------------------------------------
// PROPERTIES
// ------------------------------------------------

// Walking speed property setter / getter
void PlatformerPackage3D::set_max_walking_speed(const double p_max_walking_speed) {
    max_walking_speed = p_max_walking_speed;
}

double PlatformerPackage3D::get_max_walking_speed() const {
    return max_walking_speed;
}


void PlatformerPackage3D::set_starting_walking_speed(const double p_value) {
    starting_walking_speed = p_value;
}

double PlatformerPackage3D::get_starting_walking_speed() const {
    return starting_walking_speed;
}


void PlatformerPackage3D::set_walking_acceleration(const double p_value) {
    walking_acceleration = p_value;
}

double PlatformerPackage3D::get_walking_acceleration() const {
    return walking_acceleration;
}


void PlatformerPackage3D::set_walking_deceleration(const double p_value) {
    walking_deceleration = p_value;
}

double PlatformerPackage3D::get_walking_deceleration() const {
    return walking_deceleration;
}


void PlatformerPackage3D::set_immediate_stop_speed(const double p_value) {
    immediate_stop_speed = p_value;
}

double PlatformerPackage3D::get_immediate_stop_speed() const {
    return immediate_stop_speed;
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


// Skid jump
void PlatformerPackage3D::set_skid_jump_height(const double p_value) {
    skidJumpHeight = p_value;
}

double PlatformerPackage3D::get_skid_jump_height() const {
    return skidJumpHeight;
}


// Extra jumps
void PlatformerPackage3D::set_max_extra_jumps(const int p_value) {
    maxExtraJumps = p_value;
}

int PlatformerPackage3D::get_max_extra_jumps() const {
    return maxExtraJumps;
}

void PlatformerPackage3D::set_extra_jump_height(const double p_value) {
    extraJumpHeight = p_value;
}

double PlatformerPackage3D::get_extra_jump_height() const {
    return extraJumpHeight;
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


// -------------------------------
// Wall grab properties
// -------------------------------

void PlatformerPackage3D::set_max_wall_grab_fall_speed(const double p_value) {
    maxWallGrabFallSpeed = p_value;
}

double PlatformerPackage3D::get_max_wall_grab_fall_speed() const {
    return maxWallGrabFallSpeed;
}


void PlatformerPackage3D::set_wall_jump_speed_force_magnitude(const double p_value) {
    wallJumpSpeedForceMagnitude = p_value;
}

double PlatformerPackage3D::get_wall_jump_speed_force_magnitude() const {
    return wallJumpSpeedForceMagnitude;
}


void PlatformerPackage3D::set_wall_jump_speed_duration(const double p_value) {
    wallJumpSpeedDuration= p_value;
}

double PlatformerPackage3D::get_wall_jump_speed_duration() const {
    return wallJumpSpeedDuration;
}


void PlatformerPackage3D::set_wall_jump_height(const double p_value) {
    wallJumpHeight = p_value;
}

double PlatformerPackage3D::get_wall_jump_height() const {
    return wallJumpHeight;
}


void PlatformerPackage3D::set_max_wall_jump_angle_variant(const double p_value) {
    maxWallJumpAngleVariance = p_value;
}

double PlatformerPackage3D::get_max_wall_jump_angle_variant() const {
    return maxWallJumpAngleVariance;
}


// -------------------------------
// Dash properties
// -------------------------------


void PlatformerPackage3D::set_max_num_air_dashes(const int p_value) {
    maxNumAirDashes = p_value;
}

int PlatformerPackage3D::get_max_num_air_dashes() const {
    return maxNumAirDashes;
}


void PlatformerPackage3D::set_dash_distance(const double p_value) {
    dashDistance = p_value;
}

double PlatformerPackage3D::get_dash_distance() const {
    return dashDistance;
}


void PlatformerPackage3D::set_dash_speed(const double p_value) {
    dashSpeed = p_value;
}

double PlatformerPackage3D::get_dash_speed() const {
    return dashSpeed;
}


void PlatformerPackage3D::set_time_between_dashes(const double p_value) {
    timeBetweenDashes = p_value;
}

double PlatformerPackage3D::get_time_between_dashes() const {
    return timeBetweenDashes;
}





// General call to bind properties from the parent bind methods
void PlatformerPackage3D::bind_properties() {
    ClassDB::bind_method(D_METHOD("get_camera_node_path"), &PlatformerPackage3D::get_camera_node_path);
    ClassDB::bind_method(D_METHOD("set_camera_node_path"), &PlatformerPackage3D::set_camera_node_path);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::NODE_PATH, "camera_node_path"), "set_camera_node_path", "get_camera_node_path");

    ClassDB::bind_method(D_METHOD("get_character_body_path"), &PlatformerPackage3D::get_character_body_path);
    ClassDB::bind_method(D_METHOD("set_character_body_path"), &PlatformerPackage3D::set_character_body_path);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::NODE_PATH, "character_body_path"), "set_character_body_path", "get_character_body_path");

    ClassDB::bind_method(D_METHOD("get_player_feet_path"), &PlatformerPackage3D::get_player_feet_path);
    ClassDB::bind_method(D_METHOD("set_player_feet_path"), &PlatformerPackage3D::set_player_feet_path);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::NODE_PATH, "player_feet_path"), "set_player_feet_path", "get_player_feet_path");

    ClassDB::bind_method(D_METHOD("get_max_walking_speed"), &PlatformerPackage3D::get_max_walking_speed);
    ClassDB::bind_method(D_METHOD("set_max_walking_speed"), &PlatformerPackage3D::set_max_walking_speed);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "walking/max_walking_speed"), "set_max_walking_speed", "get_max_walking_speed");

    ClassDB::bind_method(D_METHOD("get_starting_walking_speed"), &PlatformerPackage3D::get_starting_walking_speed);
    ClassDB::bind_method(D_METHOD("set_starting_walking_speed"), &PlatformerPackage3D::set_starting_walking_speed);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "walking/starting_walking_speed"), "set_starting_walking_speed", "get_starting_walking_speed");

    ClassDB::bind_method(D_METHOD("get_walking_acceleration"), &PlatformerPackage3D::get_walking_acceleration);
    ClassDB::bind_method(D_METHOD("set_walking_acceleration"), &PlatformerPackage3D::set_walking_acceleration);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "walking/walking_acceleration"), "set_walking_acceleration", "get_walking_acceleration");

    ClassDB::bind_method(D_METHOD("get_walking_deceleration"), &PlatformerPackage3D::get_walking_deceleration);
    ClassDB::bind_method(D_METHOD("set_walking_deceleration"), &PlatformerPackage3D::set_walking_deceleration);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "walking/walking_deceleration"), "set_walking_deceleration", "get_walking_deceleration");

    ClassDB::bind_method(D_METHOD("get_immediate_stop_speed"), &PlatformerPackage3D::get_immediate_stop_speed);
    ClassDB::bind_method(D_METHOD("set_immediate_stop_speed"), &PlatformerPackage3D::set_immediate_stop_speed);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "walking/immediate_stop_speed"), "set_immediate_stop_speed", "get_immediate_stop_speed");

    ClassDB::bind_method(D_METHOD("get_walking_air_reduction"), &PlatformerPackage3D::get_walking_air_reduction);
    ClassDB::bind_method(D_METHOD("set_walking_air_reduction"), &PlatformerPackage3D::set_walking_air_reduction);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "walking/walking_air_reduction"), "set_walking_air_reduction", "get_walking_air_reduction");

    ClassDB::bind_method(D_METHOD("get_long_jump_height"), &PlatformerPackage3D::get_long_jump_height);
    ClassDB::bind_method(D_METHOD("set_long_jump_height"), &PlatformerPackage3D::set_long_jump_height);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "jumping/long_jump_height"), "set_long_jump_height", "get_long_jump_height");

    ClassDB::bind_method(D_METHOD("get_short_jump_height"), &PlatformerPackage3D::get_short_jump_height);
    ClassDB::bind_method(D_METHOD("set_short_jump_height"), &PlatformerPackage3D::set_short_jump_height);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "jumping/short_jump_height"), "set_short_jump_height", "get_short_jump_height");

    ClassDB::bind_method(D_METHOD("get_skid_jump_height"), &PlatformerPackage3D::get_skid_jump_height);
    ClassDB::bind_method(D_METHOD("set_skid_jump_height"), &PlatformerPackage3D::set_skid_jump_height);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "jumping/skid_jump_height"), "set_skid_jump_height", "get_skid_jump_height");

    ClassDB::bind_method(D_METHOD("get_max_extra_jumps"), &PlatformerPackage3D::get_max_extra_jumps);
    ClassDB::bind_method(D_METHOD("set_max_extra_jumps"), &PlatformerPackage3D::set_max_extra_jumps);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::INT, "jumping/max_extra_jumps"), "set_max_extra_jumps", "get_max_extra_jumps");

    ClassDB::bind_method(D_METHOD("get_extra_jump_height"), &PlatformerPackage3D::get_extra_jump_height);
    ClassDB::bind_method(D_METHOD("set_extra_jump_height"), &PlatformerPackage3D::set_extra_jump_height);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "jumping/extra_jump_height"), "set_extra_jump_height", "get_extra_jump_height");

    ClassDB::bind_method(D_METHOD("get_player_gravity"), &PlatformerPackage3D::get_player_gravity);
    ClassDB::bind_method(D_METHOD("set_player_gravity"), &PlatformerPackage3D::set_player_gravity);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "jumping/player_gravity"), "set_player_gravity", "get_player_gravity");

    ClassDB::bind_method(D_METHOD("get_gravity_apex_modifier"), &PlatformerPackage3D::get_gravity_apex_modifier);
    ClassDB::bind_method(D_METHOD("set_gravity_apex_modifier"), &PlatformerPackage3D::set_gravity_apex_modifier);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "jumping/gravity_apex_modifier"), "set_gravity_apex_modifier", "get_gravity_apex_modifier");

    ClassDB::bind_method(D_METHOD("get_apex_speed_definition"), &PlatformerPackage3D::get_apex_speed_definition);
    ClassDB::bind_method(D_METHOD("set_apex_speed_definition"), &PlatformerPackage3D::set_apex_speed_definition);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "jumping/apex_speed_definition"), "set_apex_speed_definition", "get_apex_speed_definition");

    ClassDB::bind_method(D_METHOD("get_jump_buffer_duration"), &PlatformerPackage3D::get_jump_buffer_duration);
    ClassDB::bind_method(D_METHOD("set_jump_buffer_duration"), &PlatformerPackage3D::set_jump_buffer_duration);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "jumping/jump_buffer_duration"), "set_jump_buffer_duration", "get_jump_buffer_duration");

    ClassDB::bind_method(D_METHOD("get_ledge_grab_horizontal_reach"), &PlatformerPackage3D::get_ledge_grab_horizontal_reach);
    ClassDB::bind_method(D_METHOD("set_ledge_grab_horizontal_reach"), &PlatformerPackage3D::set_ledge_grab_horizontal_reach);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "ledge_grab/ledge_grab_horizontal_reach"), "set_ledge_grab_horizontal_reach", "get_ledge_grab_horizontal_reach");

    ClassDB::bind_method(D_METHOD("get_ledge_grab_vertical_reach"), &PlatformerPackage3D::get_ledge_grab_vertical_reach);
    ClassDB::bind_method(D_METHOD("set_ledge_grab_vertical_reach"), &PlatformerPackage3D::set_ledge_grab_vertical_reach);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "ledge_grab/ledge_grab_vertical_reach"), "set_ledge_grab_vertical_reach", "get_ledge_grab_vertical_reach");

    ClassDB::bind_method(D_METHOD("get_auto_grab_vertical_offset"), &PlatformerPackage3D::get_auto_grab_vertical_offset);
    ClassDB::bind_method(D_METHOD("set_auto_grab_vertical_offset"), &PlatformerPackage3D::set_auto_grab_vertical_offset);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "ledge_grab/auto_grab_vertical_offset"), "set_auto_grab_vertical_offset", "get_auto_grab_vertical_offset");

    ClassDB::bind_method(D_METHOD("get_max_wall_grab_angle_requirement"), &PlatformerPackage3D::get_max_wall_grab_angle_requirement);
    ClassDB::bind_method(D_METHOD("set_max_wall_grab_angle_requirement"), &PlatformerPackage3D::set_max_wall_grab_angle_requirement);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "wall_slide/max_wall_grab_angle_requirement"), "set_max_wall_grab_angle_requirement", "get_max_wall_grab_angle_requirement");

    ClassDB::bind_method(D_METHOD("get_max_wall_grab_vertical_speed"), &PlatformerPackage3D::get_max_wall_grab_vertical_speed);
    ClassDB::bind_method(D_METHOD("set_max_wall_grab_vertical_speed"), &PlatformerPackage3D::set_max_wall_grab_vertical_speed);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "wall_slide/max_wall_grab_vertical_speed"), "set_max_wall_grab_vertical_speed", "get_max_wall_grab_vertical_speed");

    ClassDB::bind_method(D_METHOD("get_max_wall_grab_fall_speed"), &PlatformerPackage3D::get_max_wall_grab_fall_speed);
    ClassDB::bind_method(D_METHOD("set_max_wall_grab_fall_speed"), &PlatformerPackage3D::set_max_wall_grab_fall_speed);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "wall_slide/max_wall_grab_fall_speed"), "set_max_wall_grab_fall_speed", "get_max_wall_grab_fall_speed");

    ClassDB::bind_method(D_METHOD("get_wall_jump_speed_force_magnitude"), &PlatformerPackage3D::get_wall_jump_speed_force_magnitude);
    ClassDB::bind_method(D_METHOD("set_wall_jump_speed_force_magnitude"), &PlatformerPackage3D::set_wall_jump_speed_force_magnitude);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "wall_slide/wall_jump_speed_force_magnitude"), "set_wall_jump_speed_force_magnitude", "get_wall_jump_speed_force_magnitude");

    ClassDB::bind_method(D_METHOD("get_wall_jump_speed_duration"), &PlatformerPackage3D::get_wall_jump_speed_duration);
    ClassDB::bind_method(D_METHOD("set_wall_jump_speed_duration"), &PlatformerPackage3D::set_wall_jump_speed_duration);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "wall_slide/wall_jump_speed_duration"), "set_wall_jump_speed_duration", "get_wall_jump_speed_duration");

    ClassDB::bind_method(D_METHOD("get_wall_jump_height"), &PlatformerPackage3D::get_wall_jump_height);
    ClassDB::bind_method(D_METHOD("set_wall_jump_height"), &PlatformerPackage3D::set_wall_jump_height);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "wall_slide/wall_jump_height"), "set_wall_jump_height", "get_wall_jump_height");

    ClassDB::bind_method(D_METHOD("get_max_wall_jump_angle_variant"), &PlatformerPackage3D::get_max_wall_jump_angle_variant);
    ClassDB::bind_method(D_METHOD("set_max_wall_jump_angle_variant"), &PlatformerPackage3D::set_max_wall_jump_angle_variant);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "wall_slide/max_wall_jump_angle_variant"), "set_max_wall_jump_angle_variant", "get_max_wall_jump_angle_variant");

    ClassDB::bind_method(D_METHOD("get_max_num_air_dashes"), &PlatformerPackage3D::get_max_num_air_dashes);
    ClassDB::bind_method(D_METHOD("set_max_num_air_dashes"), &PlatformerPackage3D::set_max_num_air_dashes);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::INT, "dashing/max_num_air_dashes"), "set_max_num_air_dashes", "get_max_num_air_dashes");

    ClassDB::bind_method(D_METHOD("get_dash_speed"), &PlatformerPackage3D::get_dash_speed);
    ClassDB::bind_method(D_METHOD("set_dash_speed"), &PlatformerPackage3D::set_dash_speed);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "dashing/dash_speed"), "set_dash_speed", "get_dash_speed");

    ClassDB::bind_method(D_METHOD("get_dash_distance"), &PlatformerPackage3D::get_dash_distance);
    ClassDB::bind_method(D_METHOD("set_dash_distance"), &PlatformerPackage3D::set_dash_distance);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "dashing/dash_distance"), "set_dash_distance", "get_dash_distance");

    ClassDB::bind_method(D_METHOD("get_time_between_dashes"), &PlatformerPackage3D::get_time_between_dashes);
    ClassDB::bind_method(D_METHOD("set_time_between_dashes"), &PlatformerPackage3D::set_time_between_dashes);
    ClassDB::add_property("PlatformerPackage3D", PropertyInfo(Variant::FLOAT, "dashing/time_between_dashes"), "set_time_between_dashes", "get_time_between_dashes");
}
