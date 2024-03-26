#ifndef DYNAMICCAMERAPIVOT_H
#define DYNAMICCAMERAPIVOT_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include "PlatformerPackage3D.h"

using namespace godot;

namespace godot {
class DynamicCameraPivot : public Node3D {
    GDCLASS(DynamicCameraPivot, Node3D);

    private:
        // Vertical offseting
        double verticalPositionOffset = 0.5;        // Main offset from the target that the camera should be placed
        double verticalFreeSpaceBuffer = 1.5;       // The amount of free space afforded to the player's before moving the camera
        double currentYPivot;                       // Current Y pivot

        // Main transitioning flags
        bool isTransitioningToGroundHeight = false;       // main flag for transitioning to ground height
        bool isYawTransitioning = false;                  // Main flag for when yaw is transitioning

        // Main function to Y pivot transition quickly
        double transitionHeightSpeed = 5;           // Transition speed
        double startYPosition;                      // Starting speed from lerp
        double finalYPosition;                      // Final speed after lerp
        double heightTransitionTimer = 0;           // current timer
        double transitionDuration;                  // max duration it takes

        // Variables for X angle pivoting
        double yawMaxPivot = 45;                 // max x angle pivot
        double yawMinPivot = -45;                // min X angle pivot
        double fallingAngleAdjustment = 18;            // Jump angle adjustment. Falling will adjust x angle by -X 
        double adjustmentSpeedLimit = 8;            // Angle will adjust gradually when vertical speed is between -X and 0
        double angleTransitionSpeed = 2;            // Angle adjustment transitioning speed
        
        // Basic angular reference variables
        double yaw;
        double pitch;


        // Pointers to target for camera to follow
        NodePath target_path;
        class PlatformerPackage3D* target;


    protected:
        // Main function to bind methods to class
        void static _bind_methods();

    public:
        DynamicCameraPivot();
        ~DynamicCameraPivot();

        // On game start
        void _enter_tree() override;

        // On update, move the camera with the target
        void _process(double delta) override;

        // Listener function when target lands
        void on_landed();

        // Main function to rotate the camera
        void rotate_camera(double deltaX, double deltaY);



        // --------------------------
        // PROPERTIES
        // -------------------------

        // Vertical offsetting properties
        void set_vertical_position_offset(double p_value);
        double get_vertical_position_offset() const;

        void set_vertical_free_space_buffer(double p_value);
        double get_vertical_free_space_buffer() const;

        void set_transition_height_speed(double p_value);
        double get_transition_height_speed() const;


        // Vertical yaw angle properties
        void set_yaw_max_pivot(double p_value);
        double get_yaw_max_pivot() const;

        void set_yaw_min_pivot(double p_value);
        double get_yaw_min_pivot() const;

        void set_falling_angle_adjustment(double p_value);
        double get_falling_angle_adjustment() const;

        void set_adjustment_speed_limit(double p_value);
        double get_adjustment_speed_limit() const;

        void set_angle_transition_speed(double p_value);
        double get_angle_transition_speed() const;


        // Node pointers
        void set_target(NodePath p_value);
        NodePath get_target() const;

    private:
        // Main helper function to get the current Y position of camera
        double get_current_y_position(double delta);

        // Main helper function to initialize node pointers
        void initialize_current_node_pointers();

        // Main helper function to get transitioning yaw
        double get_transitioning_yaw(double delta);

        // Main helper function to get current yaw adjustment for jump
        double get_yaw_adjustment();

        // Main function to bind_properties
        void static bind_properties();
};
}

#endif
