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
    ADD_SIGNAL(MethodInfo("destroyed", PropertyInfo(Variant::OBJECT, "interactable")));
    ADD_SIGNAL(MethodInfo("interaction_sequence_end", PropertyInfo(Variant::OBJECT, "interactable")));
    ADD_SIGNAL(MethodInfo("interaction_sequence_start"));

    // Properties
    ClassDB::bind_method(D_METHOD("get_indicator_path"), &Interactable::get_indicator_path);
    ClassDB::bind_method(D_METHOD("set_indicator_path"), &Interactable::set_indicator_path);
    ClassDB::add_property("Interactable", PropertyInfo(Variant::NODE_PATH, "indicator_path"), "set_indicator_path", "get_indicator_path");

    ClassDB::bind_method(D_METHOD("get_is_interaction_sequence"), &Interactable::get_is_interaction_sequence);
    ClassDB::bind_method(D_METHOD("set_is_interaction_sequence"), &Interactable::set_is_interaction_sequence);
    ClassDB::add_property("Interactable", PropertyInfo(Variant::BOOL, "is_interaction_sequence"), "set_is_interaction_sequence", "get_is_interaction_sequence");
}


// Constructor
Interactable::Interactable() {

}


// Destructor
Interactable::~Interactable() {

}


// Main function that plays on game start
void Interactable::_enter_tree() {
    initialize_node_pointers();
}


// Main fuinction to highlight interactable. If already highlighted, do nothing
void Interactable::highlight() {
    if (indicator != nullptr) {
        indicator->set_visible(true);
    }
}


// Main function to remove highlight from interactable. If not hightlighted do nothing
void Interactable::remove_highlight() {
    if (indicator != nullptr) {
        indicator->set_visible(false);
    }
}


// Main function to interact with object
bool Interactable::interact() {
    emit_signal("interaction_sequence_start");
    return !isInteractionSequence;
}


void Interactable::initialize_node_pointers() {
    if (indicator_path != nullptr) {
        indicator = get_node<Node3D>(indicator_path);
    }
}



//-----------------------
// Properties
//-----------------------

void Interactable::set_indicator_path(const NodePath p_value) {
    indicator_path = p_value;
}

NodePath Interactable::get_indicator_path() const {
    return indicator_path;
}


void Interactable::set_is_interaction_sequence(const bool p_value) {
    isInteractionSequence = p_value;
}

bool Interactable::get_is_interaction_sequence() const {
    return isInteractionSequence;
}
