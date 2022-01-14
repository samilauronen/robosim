#include "core/Application.hpp"

// entry point
int main(void) {
	Application app;
	app.createWindow(800, 600);
	app.run();
}