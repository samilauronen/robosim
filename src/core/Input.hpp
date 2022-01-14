#pragma once
#include "Application.hpp"

class Input
{
public:
    Input() = delete;
    Input(const Input&) = delete;
    Input(Input&&) = delete;
    ~Input() = delete;

    static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
        assert(app_);
        app_->mouseEvent(window, button, action, mods);
    }
    static void MouseMovedCallback(GLFWwindow* window, double xpos, double ypos) {
        assert(app_);
        app_->mouseMoved(window, xpos, ypos);
    }
    static void KeyboardCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
        assert(app_);
        app_->keyPressed(window, key, scancode, action, mods);
    };
    static void SetApplication(Application* application) {
        Input::app_ = application;
    };
private:
    inline static Application* app_;
};