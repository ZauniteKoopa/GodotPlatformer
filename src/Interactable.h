#ifndef INTERACTABLE_H
#define INTERACTABLE_H

#include <godot_cpp/classes/node3d.hpp>

namespace godot {
class Interactable : public Node3D {
    GDCLASS(Interactable, Node3D)

    protected:
        void static _bind_methods();

    public:
        Interactable();
        ~Interactable();

        // Main fuinction to highlight interactable. If already highlighted, do nothing
        void highlight();


        // Main function to remove highlight from interactable. If not hightlighted do nothing
        void remove_highlight();


        // Main function to interact with object
        void interact();

};
}

#endif