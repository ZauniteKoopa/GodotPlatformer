#ifndef PLATFORMERPACKAGE3D_H
#define PLATFORMERPACKAGE3D_H

#include "godot_cpp/classes/character_body3d.hpp"

namespace godot {
class PlatformerPackage3D : public CharacterBody3D {
    GDCLASS(PlatformerPackage3D, CharacterBody3D)

    private:
        double walking_speed;

    protected:
        // Main function to bind methods to class
        void static _bind_methods();

    public:
        PlatformerPackage3D();
        ~PlatformerPackage3D();

        // On game start
        void _ready() override;

        // On physics update
        void _physics_process(double delta) override;

        // Speed property
        void set_walking_speed(const double p_walking_speed);
        double get_walking_speed() const;

        // Main function to move on the Z-axis given a 2D vector. (Y is relative forward / backward && X is relative left / right)
        void relative_run(Vector2 controller_vector);

    private:
        void static bind_properties();

};
};

#endif
