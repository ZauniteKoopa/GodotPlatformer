#ifndef PLATFORMERFEETSENSOR_H
#define PLATFORMERFEETSENSOR_H

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <mutex>

namespace godot {
class PlatformerFeetSensor : public Area3D {
    GDCLASS(PlatformerFeetSensor, Area3D)

    private:
        int numGroundSensed;
        std::mutex groundLock;

    protected:
        // Main function to bind methods to class
        void static _bind_methods();

    public:
        PlatformerFeetSensor();
        ~PlatformerFeetSensor();

        // Main event handler function for when something enters this area
        void on_body_enter(Node3D* body);

        // Main event handler function for when something exits this area
        void on_body_exit(Node3D* body);

};
}

#endif