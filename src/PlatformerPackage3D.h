#ifndef PLATFORMERPACKAGE3D_H
#define PLATFORMERPACKAGE3D_H

#include <godot_cpp/classes/character_body3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/classes/capsule_shape3d.hpp>
#include "PlatformerFeetSensor.h"
#include "BufferTimer.h"

namespace godot {
class PlatformerPackage3D : public CharacterBody3D {
    GDCLASS(PlatformerPackage3D, CharacterBody3D)

    private:
        // Horizontal movement
        double max_walking_speed;       // Max walking speed given no additional force is added (MAX)
        double walking_air_reduction;   // Movement reduction when in the air
        double starting_walking_speed;  // The starting walking speed when you first walk 
        double walking_acceleration;    // The rate at which you accelerate to max speed as you move
        double walking_deceleration;    // The rate at which you decelerate when not inputing but your character moves
        double immediate_stop_speed;    //  The minimum speed before you stop to a stand still
        Vector3 currentGroundInputDirection;
        Vector3 currentGroundMovement;

        // Pointers to camera
        NodePath camera_node_path;
        class Camera3D* current_camera;

        // Pointer to Character mesh
        NodePath character_body_path;
        class Node3D* character_body;

        // Pointer to Player feet
        NodePath player_feet_path;
        class PlatformerFeetSensor* player_feet;

        // Vertical movement
        double longJumpHeight;          // Max jump height
        double shortJumpHeight;         // Tap jump height
        double skidJumpHeight;          // Skid jump height
        double playerGravity;           // Gravity affecting the player, affects how heavy a jump feels
        double gravityApexModifier;     // Apex modifiers when a player is at the apex of the jump
        double apexSpeedDefinition;     // Vertical speed at which the player should be in to be considered in the apex of a jump
        double maxFallSpeed;            // Max fall speed

        // General wall behavior
        double maxWallGrabVerticalSpeed;    // The maximum vertical speed to begin clining on to a wall (you cannot be more)
        double maxWallGrabAngleRequirement; // The maximum angle to hang on to a wall (a straight wall is 0 degrees)

        // Ledge grab handling
        double ledgeGrabVerticalReach;      // How far up the player could reach for a ledge
        double ledgeGrabHorizontalReach;    // How far forward the player could reach for a ledge
        double autoGrabVerticalOffset;      // The offset to snap to from the ledge
        bool grabbingLedge;

        bool grounded;
        double currentVerticalSpeed;

        // Jump Buffering
        double jumpBufferDuration;          // How long the jump buffer time is
        BufferTimer* jumpBufferTimer;
        double bufferedJumpHeight;

    protected:
        // Main function to bind methods to class
        void static _bind_methods();

    public:
        PlatformerPackage3D();
        ~PlatformerPackage3D();

        // On game start
        void _ready() override;

        // On frame update
        void process_timers(double delta);

        // On physics update
        void _physics_process(double delta) override;

        // Main function to move on the XZ-axis given a 2D vector. (Y is relative forward / backward && X is relative left / right)
        void relative_run(Vector2 controller_vector, double time_delta);

        // Main function to start a jump
        void start_jump();

        // Main function to cancel a jump in midair
        void cancel_jump();

        // Event handler for when the unit lands on the ground
        void on_landed();

        // Event handler for when the unit begins to fall
        void on_fall_begin();


        // --------------------------
        // PROPERTIES
        // -------------------------

        // walking Speed property
        void set_max_walking_speed(const double p_walking_speed);
        double get_max_walking_speed() const;

        void set_starting_walking_speed(const double p_value);
        double get_starting_walking_speed() const;

        void set_walking_acceleration(const double p_value);
        double get_walking_acceleration() const;

        void set_walking_deceleration(const double p_value);
        double get_walking_deceleration() const;

        void set_immediate_stop_speed(const double p_value);
        double get_immediate_stop_speed() const;

        // Air reduction on walking
        void set_walking_air_reduction(const double reduction);
        double get_walking_air_reduction() const;

        // Camera Node Property
        void set_camera_node_path(const NodePath p_node_path);
        NodePath get_camera_node_path() const;

        // Character body property
        void set_character_body_path(const NodePath p_node_path);
        NodePath get_character_body_path() const;

        // Player feet property
        void set_player_feet_path(const NodePath p_node_path);
        NodePath get_player_feet_path() const;

        // Player base jump height
        void set_long_jump_height(const double jump_height);
        double get_long_jump_height() const;

        void set_short_jump_height(const double jump_height);
        double get_short_jump_height() const;

        void set_skid_jump_height(const double p_value);
        double get_skid_jump_height() const;

        // Player gravity and apex modifiers
        void set_player_gravity(const double gravity);
        double get_player_gravity() const;

        void set_gravity_apex_modifier(const double modifier);
        double get_gravity_apex_modifier() const;

        void set_apex_speed_definition(const double speed);
        double get_apex_speed_definition() const;

        // Set jump buffer
        void set_jump_buffer_duration(const double duration);
        double get_jump_buffer_duration() const;


        // -------------------------------
        // Ledge grab properties
        // -------------------------------
        void set_ledge_grab_vertical_reach(const double p_value);
        double get_ledge_grab_vertical_reach() const;

        void set_ledge_grab_horizontal_reach(const double p_value);
        double get_ledge_grab_horizontal_reach() const;

        void set_auto_grab_vertical_offset(const double p_value);
        double get_auto_grab_vertical_offset() const;

        
        // -------------------------------
        // Wall interaction properties
        // -------------------------------

        void set_max_wall_grab_vertical_speed(const double p_value);
        double get_max_wall_grab_vertical_speed() const;

        void set_max_wall_grab_angle_requirement(const double p_value);
        double get_max_wall_grab_angle_requirement() const;

    private:
        // Initializers
        void static bind_properties();
        void initialize_current_node_pointers();

        // Main function to launch a jump upon starting a jump
        void launch_jump(double jumpHeight);

        // Main function to snap to a ledge
        //  ledgePosition is the position of the ledge
        //  Wall normal is the normal of the wall
        void snap_to_ledge(Vector3 ledgePosition, Vector3 wallNormal);

        // Main private helper function to cast a ray
        Dictionary cast_ray(Vector3 from, Vector3 to);

        // Main function to check if you can grab the current wall
        bool can_interact_with_wall();

        // Function to check if you're skidding
        bool is_skidding();

        // Main helper function to handle ledge grab
        //  Pre: assumes that is_on_wall() is true
        void handle_ledge_grab();

        // Velocity calculation
        Vector3 calculate_vertical_velocity(double delta);
        Vector3 calculate_horizontal_velocity(double delta);
        double calculate_starting_jump_velocity(double curGravity, double targetedHeight);
        bool is_zero(double num);

        // Main shape accessor functions
        double get_collider_shape_height();
        double get_collider_shape_radius();
};
};

#endif
