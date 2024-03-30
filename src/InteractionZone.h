#ifndef INTERACTIONZONE_H
#define INTERACTIONZONE_H

#include <godot_cpp/classes/area3d.hpp>
#include <unordered_set>
#include <mutex>
#include "Interactable.h"

namespace godot {
class InteractionZone : public Area3D {
    GDCLASS(InteractionZone, Area3D)

    std::unordered_set<Interactable*> nearbyInteractables;
    std::mutex interactableProximityLock;
    Callable interactableDestroyedListener;
    Callable interactionSequenceEndListener;
    Interactable* curTarget = nullptr;
    bool canInteract = true;

    protected:
        void static _bind_methods();

    public:
        InteractionZone();
        ~InteractionZone();

        // Main function to process per frame
        void _process(double delta) override;

        // Main event handler function for when an entity enters area
        void on_body_enter(Node3D* body);


        // Main event handler function for when an entity exits area
        void on_body_exit(Node3D* body);


        // Main function to interact with the nearest object you're facing. Do nothing if nothing nearby is interactable
        void interact_with_target();


        // Main event handler for when interactable is destroyed
        void on_interactable_destroyed(Node* interactable);


        // Main event handler for when interaction sequence ends
        void on_interaction_sequence_end(Node* interactable);

    private:
        Interactable* get_nearest_interactable();

};
}

#endif
