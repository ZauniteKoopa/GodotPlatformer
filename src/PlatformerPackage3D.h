#ifndef PLATFORMERPACKAGE3D_H
#define PLATFORMERPACKAGE3D_H

#include <godot_cpp/classes/character_body3d.hpp>
#include <godot_cpp/classes/camera3d.hpp>

namespace godot {
class PlatformerPackage3D : public CharacterBody3D {
    GDCLASS(PlatformerPackage3D, CharacterBody3D)

    private:
        // Horizontal movement
        double walking_speed;
        double walking_air_reduction;
        Vector3 currentHorizontalDirection;

        // Pointers to cameras
        NodePath camera_node_path;
        class Camera3D* current_camera;

        // Vertical movement
        double longJumpHeight;
        double shortJumpHeight;
        double playerGravity;
        double gravityApexModifier;
        double apexSpeedDefinition;
        double maxFallSpeed;

        bool grounded;
        double currentVerticalSpeed;

    protected:
        // Main function to bind methods to class
        void static _bind_methods();

    public:
        PlatformerPackage3D();
        ~PlatformerPackage3D();

        // On game start
        void _ready() override;

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

        // walking Speed property
        void set_walking_speed(const double p_walking_speed);
        double get_walking_speed() const;

        // Air reduction on walking
        void set_walking_air_reduction(const double reduction);
        double get_walking_air_reduction() const;

        // Camera Node Property
        void set_camera_node_path(const NodePath p_node_path);
        NodePath get_camera_node_path() const;

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

    private:
        // Initializers
        void static bind_properties();
        void initialize_current_camera();

        // Velocity calculation
        Vector3 calculate_vertical_velocity(double delta);
        Vector3 calculate_horizontal_velocity(double delta);
        double calculate_starting_jump_velocity(double curGravity, double targetedHeight);
        bool is_zero(double num);
};
};

#endif
