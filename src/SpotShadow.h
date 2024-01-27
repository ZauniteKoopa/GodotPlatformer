#ifndef SPOTSHADOW_H
#define SPOTSHADOW_H

#include <godot_cpp/classes/spring_arm3d.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/node3d.hpp>

namespace godot {
class SpotShadow : public SpringArm3D {
    GDCLASS(SpotShadow, SpringArm3D)

    private:
        double minShadowScale;
        double maxShadowScale;

        // Reference to shadow property
        NodePath shadowMeshPath;
        Node3D* shadowMesh;

    protected:
        // Main function to bind methods to class
        void static _bind_methods();

    public:
        SpotShadow();
        ~SpotShadow();

        // Function that starts each game
        void _ready() override;

        // Process function that runs every frame
        void _process(double delta) override;

        // --------------------------
        // PROPERTIES
        // -------------------------

        // Min Shadow scale property
        void set_min_shadow_scale(const double p_scale);
        double get_min_shadow_scale() const;

        // Max Shadow scale property
        void set_max_shadow_scale(const double p_scale);
        double get_max_shadow_scale() const;

        // Node path
        void set_shadow_mesh_path(const NodePath p_path);
        NodePath get_shadow_mesh_path() const;

    private:
        void initialize_current_node_pointers();
};
}

#endif
