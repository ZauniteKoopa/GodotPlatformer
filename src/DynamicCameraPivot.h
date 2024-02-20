#ifndef DYNAMICCAMERAPIVOT_H
#define DYNAMICCAMERAPIVOT_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/engine.hpp>

using namespace godot;

namespace godot {
class DynamicCameraPivot : public Node3D {
    GDCLASS(DynamicCameraPivot, Node3D);

    private:
        // Vertical offseting
        double verticalPositionOffset = 0.5;        // Main offset from the target that the camera should be placed
        double verticalFreeSpaceBuffer = 1.5;       // The amount of free space afforded to the player's before moving the camera
        double currentYPivot;                       // Current Y pivot

        // Main function to Y pivot transition quickly
        double transitionHeightSpeed = 5;
        double startYPosition;
        double finalYPosition;
        double heightTransitionTimer = 0;
        double transitionDuration;
        bool isTransitioningY = false;


        // Pointers to target for camera to follow
        NodePath target_path;
        class Node3D* target;


    protected:
        // Main function to bind methods to class
        void static _bind_methods();

    public:
        DynamicCameraPivot();
        ~DynamicCameraPivot();

        // On game start
        void _ready() override;

        // On update, move the camera with the target
        void _process(double delta) override;

        // Listener function when target lands
        void on_landed();



        // --------------------------
        // PROPERTIES
        // -------------------------

        void set_vertical_position_offset(double p_value);
        double get_vertical_position_offset() const;

        void set_vertical_free_space_buffer(double p_value);
        double get_vertical_free_space_buffer() const;

        void set_target(NodePath p_value);
        NodePath get_target() const;

    private:
        // Main helper function to get the current Y position of camera
        double get_current_y_position(double delta);

        // Main helper function to initialize node pointers
        void initialize_current_node_pointers();

};
}

#endif
