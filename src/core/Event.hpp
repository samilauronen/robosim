#pragma once
#include "Eigen/Dense"
#include "GLFW/glfw3.h"

enum class EventType {
    KEY_DOWN,
    KEY_UP,
    MOUSE_MOVED,
    UPDATE
};

struct Event {
    EventType type;
    Eigen::Vector2f mouse_pos;  // current mouse position
    int key;                    // keycode of keypress
    GLFWwindow* window;         // window associated with the event
    float dt;                   // time since last event, used when event type is UPDATE
};
