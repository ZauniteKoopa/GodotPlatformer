#include "Interactable.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;


// Method binder
void Interactable::_bind_methods() {
    // Methods
    ClassDB::bind_method(D_METHOD("highlight"), &Interactable::highlight);
    ClassDB::bind_method(D_METHOD("remove_highlight"), &Interactable::remove_highlight);
    ClassDB::bind_method(D_METHOD("interact"), &Interactable::interact);

    // Signals
    ADD_SIGNAL(MethodInfo("destroyed"));
}


// Constructor
Interactable::Interactable() {

}


// Destructor
Interactable::~Interactable() {

}


// Main fuinction to highlight interactable. If already highlighted, do nothing
void Interactable::highlight() {

}


// Main function to remove highlight from interactable. If not hightlighted do nothing
void Interactable::remove_highlight() {

}


// Main function to interact with object
void Interactable::interact() {

}
