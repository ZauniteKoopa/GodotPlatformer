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
        double walking_speed;
        double walking_air_reduction;
        Vector3 currentGroundInputDirection;
        Vector3 currentGroundMovement;
        double walking_acceleration;
        double walking_deceleration;
        double immediate_stop_speed;

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
        double longJumpHeight;
        double shortJumpHeight;
        double playerGravity;
        double gravityApexModifier;
        double apexSpeedDefinition;
        double maxFallSpeed;

        // General wall behavior
        double maxWallGrabVerticalSpeed;    // prop
        double maxWallGrabAngleRequirement; // prop

        // Ledge grab handling
        double ledgeGrabVerticalReach;  // Prop
        double ledgeGrabHorizontalReach;  // prop
        double autoGrabVerticalOffset; // prop
        bool grabbingLedge;

        bool grounded;
        double currentVerticalSpeed;

        // Jump Buffering
        double jumpBufferDuration;
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
        void set_walking_speed(const double p_walking_speed);
        double get_walking_speed() const;

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
