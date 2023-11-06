#pragma once
#include "Application.hpp"
#include "Eigen/Dense"
#include "GLFW/glfw3.h"

#include "Event.hpp"

class EventDispatcher
{
public:
    EventDispatcher() = delete;
    EventDispatcher(const EventDispatcher&) = delete;
    EventDispatcher(EventDispatcher&&) = delete;
    ~EventDispatcher() = delete;

    static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
        assert(app_);
        Event ev;
        ev.key = button;
        ev.window = window;
        ev.type = action == GLFW_PRESS ? EventType::KEY_DOWN : EventType::KEY_UP;
        app_->handleEvent(ev);
    }
    static void MouseMovedCallback(GLFWwindow* window, double xpos, double ypos) {
        assert(app_);
        Event ev;
        ev.window = window;
        ev.type = EventType::MOUSE_MOVED;
        ev.mouse_pos = Eigen::Vector2f(xpos, ypos);
        app_->handleEvent(ev);
    }
    static void KeyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
        assert(app_);
        Event ev;
        ev.key = key;
        ev.window = window;
        ev.type = action == GLFW_PRESS ? EventType::KEY_DOWN : EventType::KEY_UP;
        app_->handleEvent(ev);
    };
    static void MouseScrolledCallback(GLFWwindow* window, double xoffset, double yoffset) {
        assert(app_);
        Event ev;
        ev.type = EventType::MOUSE_SCROLLED;
        ev.window = window;
        ev.scroll_amount = yoffset;
        app_->handleEvent(ev);
    }
    static void SetApplication(Application* application) {
        EventDispatcher::app_ = application;
    };
private:
    inline static Application* app_;
};