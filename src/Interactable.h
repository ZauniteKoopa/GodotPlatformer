#ifndef INTERACTABLE_H
#define INTERACTABLE_H

#include <godot_cpp/classes/static_body3d.hpp>
#include <godot_cpp/classes/node3d.hpp>

namespace godot {
class Interactable : public StaticBody3D {
    GDCLASS(Interactable, StaticBody3D)

    NodePath indicator_path;
    class Node3D* indicator;

    bool isInteractionSequence = false;

    protected:
        void static _bind_methods();

    public:
        Interactable();
        ~Interactable();

        // Main function to initialize variables at the start of the game
        void _enter_tree() override;

        // Main fuinction to highlight interactable. If already highlighted, do nothing
        virtual void highlight();


        // Main function to remove highlight from interactable. If not hightlighted do nothing
        virtual void remove_highlight();


        // Main function to interact with object. returns whether the player can still interact with other objects immediately after
        virtual bool interact();


        // ----------------------
        // Properties
        // ----------------------

        void set_indicator_path(const NodePath p_value);
        NodePath get_indicator_path() const;


        void set_is_interaction_sequence(const bool p_value);
        bool get_is_interaction_sequence() const;

    private:
        void initialize_node_pointers();

};
}

#endif