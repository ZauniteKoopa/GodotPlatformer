#ifndef BUFFERTIMER_H
#define BUFFERTIMER_H

#include <mutex>

class BufferTimer {
    private:
        // The max duration of this timer before it running turns false
        double maxTimerDuration;

        // Current timer that's running
        double curTimer;

        // Flag to tell if timer is active or not
        bool running;

        // General lock around all variables
        std::mutex timerLock;

    public:
        // Constructor / Destructor
        BufferTimer(double duration);
        ~BufferTimer();

        // Main function to update the timer (this should be in the update loop of parent scripts)
        //  Pre: delta >= 0 (represents the amount of time passed in a frame)
        void update(double delta);


        // Main function to activate the timer if not activated already. Otherwise, it does nothing
        //  Post: returns whether the timer was activated or not
        bool activate();


        // Main function to cancel the timer if activated (setting running to false and no longer updating curTimer). Otherwise, do nothing
        //  Post: returns whether the timer was canceled or not
        bool cancel();


        // Main function to reset the timer (canceling the timer and running it again). Will always succeed
        void reset();


        // Main function to check if the timer was activated
        bool isRunning() const;


        // Main function to set a new max duration
        void setMaxDuration(double duration);
};

#endif
