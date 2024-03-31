#include "InteractionZone.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/input.hpp>

using namespace godot;

// Method binder
void InteractionZone::_bind_methods() {
    // Methods
    ClassDB::bind_method(D_METHOD("on_body_enter"), &InteractionZone::on_body_enter);
    ClassDB::bind_method(D_METHOD("interact_with_target"), &InteractionZone::interact_with_target);
    ClassDB::bind_method(D_METHOD("on_interactable_destroyed"), &InteractionZone::on_interactable_destroyed);
    ClassDB::bind_method(D_METHOD("on_interaction_sequence_end"), &InteractionZone::on_interaction_sequence_end);
    ClassDB::bind_method(D_METHOD("on_body_exit"), &InteractionZone::on_body_exit);

}

// Main constructor
InteractionZone::InteractionZone() {
    interactableDestroyedListener = Callable(this, "on_interactable_destroyed");
    interactionSequenceEndListener = Callable(this, "on_interaction_sequence_end");
}

// Main destructor
InteractionZone::~InteractionZone() {

}


// Main process function for highlighting
void InteractionZone::_process(double delta) {
    if (!Engine::get_singleton()->is_editor_hint()) {
        // Get current interactable for this frame and compare with current interactable for previous frame
        Interactable* newTarget = get_nearest_interactable();

        // If they're not the same, change the highlighting
        if (newTarget != curTarget) {
            if (newTarget != nullptr) {
                newTarget->highlight();
            }

            if (curTarget != nullptr) {
                curTarget->remove_highlight();
            }
        }

        // Update curTarget
        curTarget = newTarget;

        // Input handling
        if (Input::get_singleton()->is_action_just_pressed("interact")) {
            interact_with_target();
        }

    }
}


// Main event handler function for when an entity enters area
void InteractionZone::on_body_enter(Node3D* body) {
    Interactable* interactable = Object::cast_to<Interactable>(body);

    // If body is actually an interactable AND it's not in the hashset yet, add it in the list of nearby interactables
    interactableProximityLock.lock();

    if (interactable && nearbyInteractables.find(interactable) == nearbyInteractables.end()) {
        nearbyInteractables.insert(interactable);
        interactable->connect("destroyed", interactableDestroyedListener);
    }

    interactableProximityLock.unlock();
}


// Main event handler function for when an entity enters area
void InteractionZone::on_body_exit(Node3D* body) {
    Interactable* interactable = Object::cast_to<Interactable>(body);

    // If body is actually an interactable AND it's in the hashset, remove it from the list of nearby interactables
    interactableProximityLock.lock();

    if (interactable && nearbyInteractables.find(interactable) != nearbyInteractables.end()) {
        nearbyInteractables.erase(interactable);
        interactable->disconnect("destroyed", interactableDestroyedListener);
    }

    interactableProximityLock.unlock();
}


// Main function to interact with the nearest object you're facing. Do nothing if nothing nearby is interactable
void InteractionZone::interact_with_target() {
    if (canInteract) {
        Interactable* target = get_nearest_interactable();

        if (target != nullptr) {
            canInteract = target->interact();

            if (!canInteract) {
                target->connect("interaction_sequence_end", interactionSequenceEndListener);
            }
        }
    }
}


// Main event handler for when interactable is destroyed
void InteractionZone::on_interactable_destroyed(Node* interactableNode) {
    Interactable* interactable = Object::cast_to<Interactable>(interactableNode);

    // If body is actually an interactable AND it's in the hashset, remove it from the list of nearby interactables
    interactableProximityLock.lock();

    if (interactable && nearbyInteractables.find(interactable) != nearbyInteractables.end()) {
        nearbyInteractables.erase(interactable);
        interactable->disconnect("destroyed", interactableDestroyedListener);
    }

    interactableProximityLock.unlock();
}


// Main event handler for when interaction sequence ends: enable interaction again
void InteractionZone::on_interaction_sequence_end(Node* interactableNode) {
    Interactable* interactable = Object::cast_to<Interactable>(interactableNode);

    if (interactable) {
        canInteract = true;
        interactable->disconnect("interaction_sequence_end", interactionSequenceEndListener);
    }
}


// Private helper function to get the nearest interactable
Interactable* InteractionZone::get_nearest_interactable() {
    // Target to return
    Interactable* target = nullptr;

    // Variables to keep track of: you must face towards an interactable for it be valid. Prioritize the closest interactable
    double distanceFromPlayer = DBL_MAX;

    // Cache the forward and global position the player is in
    Vector3 currentPosition = get_global_position();
    Vector3 currentForward = -get_global_transform().get_basis().get_column(2);

    // Iterate over hash set
    interactableProximityLock.lock();

    for (auto cur = nearbyInteractables.begin(); cur != nearbyInteractables.end(); cur++) {
        Interactable* curInteractable = *cur;

        // Check if instance exists
        if (UtilityFunctions::is_instance_valid(curInteractable)) {
            // Calculate general distance vector
            Vector3 distanceVector = curInteractable->get_global_position() - currentPosition;
            double curDotProduct = (currentForward.normalized().dot(distanceVector.normalized()));
            double curDistance = distanceVector.length();

            // If dot prpoduct > 0, you face towards it. Only override target if distance is smaller
            if (curDotProduct > 0 && curDistance < distanceFromPlayer) {
                target = curInteractable;
                distanceFromPlayer = curDistance;
            }
        }
    }

    interactableProximityLock.unlock();

    return target;
}
