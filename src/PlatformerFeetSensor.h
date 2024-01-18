#ifndef PLATFORMERFEETSENSOR_H
#define PLATFORMERFEETSENSOR_H

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/scene_tree_timer.hpp>
#include <mutex>

namespace godot {
class PlatformerFeetSensor : public Area3D {
    GDCLASS(PlatformerFeetSensor, Area3D)

    private:
        // Core ground sensor
        int numGroundSensed;
        std::mutex groundLock;

        // Coyote time variable
        std::mutex coyoteTimeLock;
        double coyoteTimeDuration;
        Ref<SceneTreeTimer> curCoyoteTimer;
        Callable coyoteTimeListener;
        
        bool coyoteTimeDisabled;


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

        // Main function to disable coyote time until the player lands on the ground the next time
        void disable_coyote_time();

        // Event handler for coyote timeout
        void on_coyote_timeout();


        // ---------------------------------
        //  Properties
        // ---------------------------------

        void set_coyote_time_duration(const double duration);
        double get_coyote_time_duration() const;

    private:
        // Private helper sequence to do coyote time sequence
        void start_coyote_timer();
        void cancel_coyote_timer();

};
}

#endif