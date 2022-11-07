#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include "Testbot.h"


int main(int argc, char** argv) {


	if (!DeltaFKInitialize())
	{
		std::cout << "Fail to initialize Matlab FK" << std::endl;
	}

	webots::Robot* robot = new webots::Robot();

	int timeStep = (int)robot->getBasicTimeStep();

	Testbot deltahopper(robot);

	deltahopper.Init(); 

	while (robot->step(timeStep) != -1)
	{
		deltahopper.Update();
	};

	DeltaFKTerminate();
	delete robot;
	return 0;
}
