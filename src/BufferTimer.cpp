#include "BufferTimer.h"

// Constructor / Destructor
BufferTimer::BufferTimer(double duration) {
    maxTimerDuration = duration;
    running = false;
    curTimer = 0;
}

BufferTimer::~BufferTimer() {}



// Main function to update the timer (this should be in the update loop of parent scripts)
//  Pre: delta >= 0 (represents the amount of time passed in a frame)
void BufferTimer::update(double delta) {
    timerLock.lock();

    if (running) {
        curTimer += delta;
        running = (curTimer < maxTimerDuration);
    }

    timerLock.unlock();
}


// Main function to activate the timer if not activated already. Otherwise, it does nothing
//  Post: returns whether the timer was activated or not
bool BufferTimer::activate() {
    timerLock.lock();

    bool willActivate = !running;

    if (willActivate) {
        curTimer = 0;
        running = true;
    }

    timerLock.unlock();
    return willActivate;
}


// Main function to cancel the timer if activated (setting running to false and no longer updating curTimer). Otherwise, do nothing
//  Post: returns whether the timer was canceled or not
bool BufferTimer::cancel() {
    timerLock.lock();

    bool willActivate = running;

    if (willActivate) {
        running = false;
    }

    timerLock.unlock();
    return willActivate;
}


// Main function to reset the timer (canceling the timer and running it again). Will always succeed
void BufferTimer::reset() {
    cancel();
    activate();
}


// Main function to check if the timer was activated
bool BufferTimer::isRunning() const {
    return running;
}


// Main function to set a new max duration
void BufferTimer::setMaxDuration(double duration) {
    maxTimerDuration = duration;
}
