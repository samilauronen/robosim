#pragma once
#include "Eigen/Dense"
#include "GLFW/glfw3.h"

enum class EventType
{
    KEY_DOWN,
    KEY_UP,
    MOUSE_MOVED,
    MOUSE_SCROLLED
};

struct Event
{
    EventType type;             // type of this event
    bool handled;               // whether this event has been handled or not
    Eigen::Vector2f mouse_pos;  // current mouse position
    double scroll_amount;       // amount of mouse scrolling
    int key;                    // keycode of keypress
    GLFWwindow* window;         // window associated with the event
};