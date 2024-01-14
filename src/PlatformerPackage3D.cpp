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

    // Listeners to bind
    ClassDB::bind_method(D_METHOD("on_landed"), &PlatformerPackage3D::on_landed);
    ClassDB::bind_method(D_METHOD("on_fall_begin"), &PlatformerPackage3D::on_fall_begin);

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
}

// General constructor
PlatformerPackage3D::PlatformerPackage3D() {
    // Starting grounded movement
    walking_speed = 10;
    currentHorizontalDirection = Vector3(0, 0, 0);

    // Starting vertical movement
    longJumpHeight = 6;
    shortJumpHeight = 3;
    playerGravity = 5;
    gravityApexModifier = 0.5;
    apexSpeedDefinition = 1.5;
    maxFallSpeed = 10;

    grounded = false;
    currentVerticalSpeed = 0;

    // Reference nodes (MUST BE SET TO NULL ON CONSTRUCTION)
    current_camera = nullptr;
    character_body = nullptr;

}

// Main function to clean up
PlatformerPackage3D::~PlatformerPackage3D() {
}

// Main function that plays on game start
void PlatformerPackage3D::_ready() {
    initialize_current_node_pointers();
    currentVerticalSpeed = 0;
}

// Main function that plays for each physics frame
void PlatformerPackage3D::_physics_process(double delta) {
    // Check if you're in editor mode or in game mode. If in game mode, actually run it
    if (!Engine::get_singleton()->is_editor_hint()) {
        // Set the velocity and then move
        set_velocity(calculate_horizontal_velocity(delta) + calculate_vertical_velocity(delta));
        move_and_slide();
    }
}


// Main function to start a jump
void PlatformerPackage3D::start_jump() {
    if (grounded) {
        currentVerticalSpeed = calculate_starting_jump_velocity(playerGravity, longJumpHeight);
    }
}

// Main function to cancel a jump in midair 
void PlatformerPackage3D::cancel_jump() {
    if (!grounded) {
        double shortJumpVelocity = calculate_starting_jump_velocity(playerGravity, shortJumpHeight);
        currentVerticalSpeed = Math::min(currentVerticalSpeed, shortJumpVelocity);
    }
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
    currentHorizontalDirection = (controller_vector.x * world_right) + (controller_vector.y * world_forward);
    if (!is_zero(controller_vector.x) || !is_zero(controller_vector.y)) {
        currentHorizontalDirection.normalize();
    }
}


// Event handler for when the unit lands on the ground
void PlatformerPackage3D::on_landed() {
    currentVerticalSpeed = 0;
    grounded = true;
}


// Event handler for when the unit begins to fall
void PlatformerPackage3D::on_fall_begin() {
    grounded = false;
}


// Main private helper function to calculate the vertical component of velocity
Vector3 PlatformerPackage3D::calculate_vertical_velocity(double delta) {
    // Update currentVerticalSpeed based on gravity if not grounded
    if (!grounded) {
        bool inApex = (currentVerticalSpeed > -apexSpeedDefinition && currentVerticalSpeed < apexSpeedDefinition);
        double curGravity = (inApex) ? gravityApexModifier * playerGravity : playerGravity;
        currentVerticalSpeed -= (curGravity * delta);
    }

    // Put fall velocity in a vector
    return Vector3(0, Math::max(currentVerticalSpeed, -maxFallSpeed), 0);
}


// Main private helper function to calculate the XZ component of velocity
Vector3 PlatformerPackage3D::calculate_horizontal_velocity(double delta) {
    double curSpeed = (grounded) ? walking_speed : walking_speed * walking_air_reduction;

    if (currentHorizontalDirection.length() > 0.01f) {
        character_body->look_at(get_global_position() + currentHorizontalDirection);
    }

    return curSpeed * currentHorizontalDirection;
}


// Main function to calculate the starting jump velocity given gravity and targeted height
double PlatformerPackage3D::calculate_starting_jump_velocity(double curGravity, double targetedHeight) {
    return Math::sqrt(2.0 * curGravity * targetedHeight);
}


// Main helper function to check if a number is close enough to zero
bool PlatformerPackage3D::is_zero(double num) {
    return (num > -0.001) && (num < 0.001);
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


void PlatformerPackage3D::initialize_current_node_pointers() {
    if (camera_node_path != nullptr) {
        current_camera = get_node<Camera3D>(camera_node_path);
    }

    if (character_body_path != nullptr) {
        character_body = get_node<Node3D>(character_body_path);
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
