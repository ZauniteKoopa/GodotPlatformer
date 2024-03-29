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
        double max_walking_speed = 10;       // Max walking speed given no additional force is added (MAX)
        double walking_air_reduction = 0.7;   // Movement reduction when in the air
        double starting_walking_speed = 3;  // The starting walking speed when you first walk 
        double walking_acceleration = 750;    // The rate at which you accelerate to max speed as you move
        double walking_deceleration = 400;    // The rate at which you decelerate when not inputing but your character moves
        double immediate_stop_speed = 3;    //  The minimum speed before you stop to a stand still
        double turning_angle = 30;                  // The angle the player needs to move in to be considered "turning" (between 90 - x and 90 + x)
        double turning_acceleration_multiplier = 7; // Multiplier applied to acceleration to make turns sharp (Increase to make turning sharper. decrease to make it wider)
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
        double longJumpHeight = 2.5;          // Max jump height
        double shortJumpHeight = 1;         // Tap jump height
        double skidJumpHeight = 3.5;          // Skid jump height
        int maxExtraJumps = 1;                 // Number of extra jumps afforded
        double extraJumpHeight = 1.2;         // The height of the double jump
        double playerGravity = 5;           // Gravity affecting the player, affects how heavy a jump feels
        double gravityApexModifier = 0.5;     // Apex modifiers when a player is at the apex of the jump
        double apexSpeedDefinition = 1.5;     // Vertical speed at which the player should be in to be considered in the apex of a jump
        double maxFallSpeed = 12;            // Max fall speed
        
        int curExtraJumpsDone;

        // Flags for jump type (will turn to enum)
        bool didGroundJump = false;


        // Ground triple jump
        int currentConsecutiveGroundJump = 0;               // Current consecutive jump
        int maxConsecutiveGroundJumps = 3;                  // Max number of consecutive jump
        double groundJumpHeightIncrease = 1.5;              // Jump height increase
        const double GROUND_JUMP_TIMER_DURATION = 0.05;      // Duration of ground jump timer (you must jump within the next X seconds to get a consecutive jump)
        Ref<SceneTreeTimer> consecutiveGroundJumpTimer;     // Main timer to track when on ground
        Callable groundJumpTimeoutListener;                 // Main listener for ground jump timer timeout
        std::mutex groundJumpLock;                          // Ground jump lock
        bool inConsecutiveJump = false;                     // Flag for whether or not you're in a consecutive jump
        
        // Dashing
        int maxNumAirDashes = 1;                   // maximum number of dashes you can do in the air
        double dashSpeed = 12;                  // Dash speed
        double dashDistance = 2.5;              // Dash distance before speed decays off
        double timeBetweenDashes = 0.4;         // Time between dashes
        int curDashesUsed = 0;                  // Current amount of dashes used
        Ref<SceneTreeTimer> dashAnimationTimer; // Timer for when dash animation is active
        Ref<SceneTreeTimer> dashCooldownTimer;  // Dash cooldown timer
        bool canDash = true;                    // Can dash flag
        Callable dashCooldownListener;          // Main dash listener
        std::mutex dashLock;                    // Mutex dash used
        bool dashing = false;

        // General wall behavior
        double maxWallGrabVerticalSpeed;    // The maximum vertical speed to begin clining on to a wall (you cannot be more)
        double maxWallGrabAngleRequirement; // The maximum angle to hang on to a wall (a straight wall is 0 degrees)

        // Ledge grab handling
        double ledgeGrabVerticalReach = 0.2;      // How far up the player could reach for a ledge
        double ledgeGrabHorizontalReach = 0.5;    // How far forward the player could reach for a ledge
        double autoGrabVerticalOffset = 0.3;      // The offset to snap to from the ledge
        bool grabbingLedge = false;

        bool grounded = false;
        double currentVerticalSpeed = 0;

        // Jump Buffering
        double jumpBufferDuration = 0.01;          // How long the jump buffer time is
        BufferTimer* jumpBufferTimer;
        double bufferedJumpHeight;

        // Wall slide / jump
        bool grabbingWall = false;                    // Flag for grabbing wall
        double maxWallGrabFallSpeed = 2.5;            // The max fall speed when grabbing the wall
        double wallJumpSpeedForceMagnitude = 3.5;     // Magnitude of the horizontal force when jumping off of the wall
        double wallJumpSpeedDuration = 0.4;           // Duration of the horizontal force when jumping off a wall
        double wallJumpHeight = 2;                    // General height of the wall jump
        double maxWallJumpAngleVariance = 30;         // Angle of variance for when jumping off the wall

        // Applied forces
        std::mutex appliedSpeedLock;                 // Lock for applying speed forces
        Vector3 appliedSpeedVector;                  // Current applied speed force vector
        double appliedSpeedDecayRate = 20;                // Once flag is off, rate at which speed decays
        Ref<SceneTreeTimer> appliedSpeedTimer;       // current applied speed force timer
        bool appliedSpeedActive = false;             // Flag for if applied Speed force is active
        Callable appliedSpeedTimeoutListener;        // Main listener for appliedSeedTimeout

        
        // Respawn system
        double deathPlaneHeight = -6;
        bool dying = false;


    protected:
        // Main function to bind methods to class
        void static _bind_methods();

    public:
        PlatformerPackage3D();
        ~PlatformerPackage3D();

        // On game start
        void _enter_tree() override;

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

        // Main function to dash if you're able to
        void dash();

        // Event handler for when the unit lands on the ground
        void on_landed();

        // Event handler for when the unit begins to fall
        void on_fall_begin();

        // Main function to apply speed force
        //  Pre: speedForceVector is the direction and magnitude of the speed force affecting character,
        //       duration is the duration of the force before decaying
        void apply_speed_force(Vector3 speedForceVector, double duration);

        // Main function to cancel speed force
        void cancel_speed_force();

        // Main event handler when speed force expires
        void on_speed_force_expire();

        // Main event handler for when dash is re-enabled
        void on_dash_regained();

        // Main event handler for when consecutive ground jump timer runs out
        void on_ground_jump_timeout();

        // Main function to respawn
        void respawn();

        // Accessor functions for animations
        bool is_grounded() const;
        bool is_dashing() const;
        bool is_skidding() const;
        bool is_wall_grabbing() const;
        bool is_ledge_grabbing() const;
        double get_current_vertical_speed() const;
        double get_current_horizontal_speed() const;
        double get_current_walking_animation_speed() const;
        int get_current_ground_jump_number() const;


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

        void set_turning_angle(const double p_value);
        double get_turning_angle() const;

        void set_turning_acceleration_multiplier(const double p_value);
        double get_turning_acceleration_multiplier() const;

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

        void set_max_fall_speed(const double p_value);
        double get_max_fall_speed() const;

        // Extra jumps
        void set_max_extra_jumps(const int p_value);
        int get_max_extra_jumps() const;

        void set_extra_jump_height(const double p_value);
        double get_extra_jump_height() const;

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


        // -------------------------------
        // Wall grab properties
        // -------------------------------


        void set_max_wall_grab_fall_speed(const double p_value);
        double get_max_wall_grab_fall_speed() const;

        void set_wall_jump_speed_force_magnitude(const double p_value);
        double get_wall_jump_speed_force_magnitude() const;

        void set_wall_jump_speed_duration(const double p_value);
        double get_wall_jump_speed_duration() const;

        void set_wall_jump_height(const double p_value);
        double get_wall_jump_height() const;

        void set_max_wall_jump_angle_variant(const double p_value);
        double get_max_wall_jump_angle_variant() const;


        // -------------------------------
        // Dash properties
        // -------------------------------

        void set_max_num_air_dashes(const int p_value);
        int get_max_num_air_dashes() const;

        void set_dash_speed(const double p_value);
        double get_dash_speed() const;

        void set_dash_distance(const double p_value);
        double get_dash_distance() const;

        void set_time_between_dashes(const double p_value);
        double get_time_between_dashes() const;

        // -------------------------------
        // Consecutive Ground Jump Properties
        // -------------------------------

        void set_max_consecutive_ground_jumps(const int p_value);
        int get_max_consecutive_ground_jumps() const;

        void set_ground_jump_height_increase(const double p_value);
        double get_ground_jump_height_increase() const;


        // -------------------------------
        // Death Properties
        // -------------------------------

        void set_death_plane_height(const double p_value);
        double get_death_plane_height() const;

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

        // Main helper function to calculate the wall jump normal
        Vector3 calculate_wall_jump_force(Vector3 playerWorldInput);

        // Helper function to check if  jumpBuffer overrides extraJump (you jump on the ground via jump buffer instead of doing the extra jump)
        bool doesBufferOverrideExtraJump();

        // Helper function to calculate the height of the ground jump. After triggering this, the current consequetive jump will be incremented by 1
        double get_current_ground_jump_height();


        // Velocity calculation
        Vector3 calculate_vertical_velocity(double delta);
        Vector3 calculate_horizontal_velocity(double delta);
        double calculate_starting_jump_velocity(double curGravity, double targetedHeight);
        bool is_zero(double num);

        // Main shape accessor functions
        double get_collider_shape_height();
        double get_collider_shape_radius();

        // Dash speed accessor functions
        double get_dash_duration();
};
};

#endif
