#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include "DeltaFK.h"
#include "vec3.h"
#include <webots/TouchSensor.hpp>
#include "Pid.hpp"
#include <webots/led.hpp>
#include "simpleInv.h"
#include <webots/DistanceSensor.hpp>
#include <webots/Keyboard.hpp>

#define GLOBAL_SAMPLING_PERIOD 1
#define DEFAULT_VIRTUAL_LEG_LENGTH 0.2864f
class Testbot;

enum State {
	LOADING = 0x00,    
	COMPRESSION = 0x01,    
	THRUST = 0x02,    
	UNLOADING = 0x03,    
	FLIGHT = 0x04    
};

class Testbot
{
private:

	webots::Robot* m_p_robot;
	webots::Motor* m_p_baseMotor[3];
	webots::PositionSensor* m_p_posSensor[3];
	webots::TouchSensor* m_p_touchSensor;
	webots::LED* m_p_led;
	webots::DistanceSensor* m_p_distanceSensor;
	webots::Keyboard* m_p_keyBoard;

	State m_state;

	mwArray* m_p_inputBuf;
	mwArray* m_p_FKBuf;//mwArray must be constructed after matlab initialization, so they couldn't be static. However,construction of mwarray is really slow. 
	mwArray* m_p_JacobBuf;

	Pid PidXdir;
	Pid PidYdir;

	vec3 m_footLastPos;
	vec3 m_footPos;
	vec3 m_footVel;

	vec3 m_footForce;

	

	double m_Jacobian[3][3] = { 0 };
	double m_JacobianTranspose[3][3] = { 0 };
	double m_JacobianInverse[3][3] = { 0 };

	double m_baseMotorFlightAngleSet[3] = { 0 };

	double hipangleDisplacement[3] = { 0 };

	double bodyVx;
	double bodyVy;


	int m_TsTick;
	int m_NewTsTick;

public:

	Testbot(webots::Robot* robot);

	void Init();
	void Update();
	void UpdateState();
	void UpdateVMCForce();
	void UpdateJointTorque();
	void UpdateFowardKinematics();
	void UpdateLegPose();
};

