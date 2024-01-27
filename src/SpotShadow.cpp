#include "SpotShadow.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// Main function to bind methods
void SpotShadow::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_min_shadow_scale"), &SpotShadow::get_min_shadow_scale);
    ClassDB::bind_method(D_METHOD("set_min_shadow_scale"), &SpotShadow::set_min_shadow_scale);
    ClassDB::add_property("SpotShadow", PropertyInfo(Variant::FLOAT, "min_shadow_scale"), "set_min_shadow_scale", "get_min_shadow_scale");

    ClassDB::bind_method(D_METHOD("get_max_shadow_scale"), &SpotShadow::get_max_shadow_scale);
    ClassDB::bind_method(D_METHOD("set_max_shadow_scale"), &SpotShadow::set_max_shadow_scale);
    ClassDB::add_property("SpotShadow", PropertyInfo(Variant::FLOAT, "max_shadow_scale"), "set_max_shadow_scale", "get_max_shadow_scale");

    ClassDB::bind_method(D_METHOD("get_shadow_mesh_path"), &SpotShadow::get_shadow_mesh_path);
    ClassDB::bind_method(D_METHOD("set_shadow_mesh_path"), &SpotShadow::set_shadow_mesh_path);
    ClassDB::add_property("SpotShadow", PropertyInfo(Variant::NODE_PATH, "shadow_mesh_path"), "set_shadow_mesh_path", "get_shadow_mesh_path");
}


// Main constructor
SpotShadow::SpotShadow() {
    maxShadowScale = 1.0;
    minShadowScale = 0.2;
}


// Main destructor
SpotShadow::~SpotShadow() {

}


// Function that runs at the start of each game
void SpotShadow::_ready() {
    initialize_current_node_pointers();
}


// Process function that runs every frame
void SpotShadow::_process(double delta) {
    if (!Engine::get_singleton()->is_editor_hint()) {
        // Check if you even need to show the shadow
        bool showShadow = get_hit_length() < (get_length() - 0.01);
        shadowMesh->set_visible(showShadow);

        // If you need to show the shadow, send down a raycast
        if (showShadow) {
            // Set up query and fire ray cast down the raycast query
            PhysicsDirectSpaceState3D* spaceState = get_world_3d()->get_direct_space_state();
            Ref<PhysicsRayQueryParameters3D> rayCastQuery = PhysicsRayQueryParameters3D::create(
                get_global_position(),
                get_global_position() + (get_length() * get_global_transform().get_basis().get_column(2).normalized()),
                get_collision_mask()
            );
            Dictionary rayResult = spaceState->intersect_ray(rayCastQuery);

            // If hit something, set up shadow
            if (!rayResult.is_empty()) {
                // Make a basis
                Vector3 rayUp = rayResult["normal"];
                Vector3 rayForward = rayUp.cross(Vector3(1, 0, 0));
                Vector3 rayLeft = rayUp.cross(rayForward);

                shadowMesh->set_global_basis(Basis(rayLeft, rayUp, rayForward));
                double curShadowScale = Math::lerp(minShadowScale, maxShadowScale, 1.0 - (get_hit_length() / get_length()));
                shadowMesh->set_scale(Vector3(curShadowScale, 1.0, curShadowScale));
            } else {
                shadowMesh->set_visible(false);
            }
        }
    }
}


// Private helper function to initialize node pointers
void SpotShadow::initialize_current_node_pointers() {
    if (shadowMeshPath != nullptr) {
        shadowMesh = get_node<Node3D>(shadowMeshPath);
    }
}


// --------------------------
// PROPERTIES
// -------------------------

// Min Shadow scale property
void SpotShadow::set_min_shadow_scale(const double p_scale) {
    minShadowScale = p_scale;
}

double SpotShadow::get_min_shadow_scale() const {
    return minShadowScale;
}

// Max Shadow scale property
void SpotShadow::set_max_shadow_scale(const double p_scale) {
    maxShadowScale = p_scale;
}

double SpotShadow::get_max_shadow_scale() const {
    return maxShadowScale;
}

// Node path
void SpotShadow::set_shadow_mesh_path(const NodePath p_path) {
    shadowMeshPath = p_path;
}

NodePath SpotShadow::get_shadow_mesh_path() const {
    return shadowMeshPath;
}