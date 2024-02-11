#ifndef PLATFORMERFEETSENSOR_H
#define PLATFORMERFEETSENSOR_H

#include <godot_cpp/classes/area3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/scene_tree_timer.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <mutex>
#include <unordered_set>

namespace godot {
class PlatformerFeetSensor : public Area3D {
    GDCLASS(PlatformerFeetSensor, Area3D)

    private:
        // Core ground sensor
        std::unordered_set<Node3D*> groundSensed;
        std::mutex groundLock;

        // Coyote time variable
        std::mutex coyoteTimeLock;
        double coyoteTimeDuration;
        Ref<SceneTreeTimer> curCoyoteTimer;
        Callable coyoteTimeListener;
        bool coyoteTimeDisabled;

        // Manging angles at different grounds
        double floorMaxAngle;


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

        // Main function to project movement on current ground plane
        Vector3 project_movement_on_ground(Vector3 velocity);


        // ---------------------------------
        //  Properties
        // ---------------------------------

        void set_coyote_time_duration(const double duration);
        double get_coyote_time_duration() const;

        void set_floor_max_angle(const double angle);
        double get_floor_max_angle() const;

    private:
        // Private helper sequence to do coyote time sequence
        void start_coyote_timer();
        void cancel_coyote_timer();

        // Main function to get the OVERALL normal of the ground you're standing on
        Vector3 get_overall_ground_normal();

        // Main function to get the normal of one collision
        Vector3 get_collision_normal(Node3D* collidedGround);

};
}

#endif