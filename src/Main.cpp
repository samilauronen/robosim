#include "core/App.hpp"

// entry point
int main(void) {
	App app;
	app.createWindow(800, 600);
	app.loop();
}