#ifndef FORCEAREA_H
#define FORCEAREA_H

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/node3d.hpp>

namespace godot {
class ForceArea : public Area3D {
    GDCLASS(ForceArea, Area3D)

    private:
        Vector3 localForceDirection = Vector3(1, 0, 0);
        double forceMagnitude = 8;
        double forceDuration = 1.5;

    protected:
        // Main function to bind methods to class
        void static _bind_methods();

    public:
        ForceArea();
        ~ForceArea();

        // Main event handler function for when an entity enters area
        void on_body_enter(Node3D* body);



        // --------------------------
        // PROPERTIES
        // -------------------------


        // Force duration properties
        void set_force_duration(const double p_value);
        double get_force_duration() const;


        // Force direction properties
        void set_force_direction(const Vector3 p_value);
        Vector3 get_force_direction() const;


        // Force magnitude properties
        void set_force_magnitude(const double p_value);
        double get_force_magnitude() const;
};
}

#endif
