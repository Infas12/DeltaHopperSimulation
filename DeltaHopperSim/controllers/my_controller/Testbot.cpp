#include "Testbot.h"
#include "math.h"

Testbot::Testbot(webots::Robot* robot) : m_p_robot(robot),
m_p_inputBuf(nullptr),
m_p_FKBuf(nullptr),
m_p_JacobBuf(nullptr),
m_footPos(0,0,-DEFAULT_VIRTUAL_LEG_LENGTH),
m_footLastPos(0, 0, -DEFAULT_VIRTUAL_LEG_LENGTH),
m_state(FLIGHT)
{
	m_p_baseMotor[0] = m_p_robot->getMotor("base_motor1");
	m_p_baseMotor[1] = m_p_robot->getMotor("base_motor2");
	m_p_baseMotor[2] = m_p_robot->getMotor("base_motor3");
	m_p_posSensor[0] = m_p_robot->getPositionSensor("base_motor1_sensor");
	m_p_posSensor[1] = m_p_robot->getPositionSensor("base_motor2_sensor");
	m_p_posSensor[2] = m_p_robot->getPositionSensor("base_motor3_sensor");
	m_p_touchSensor = m_p_robot->getTouchSensor("touch sensor");
	m_p_distanceSensor = m_p_robot->getDistanceSensor("distance sensor");
	m_p_led = m_p_robot->getLED("led");
	m_p_keyBoard = m_p_robot->getKeyboard();
}

void Testbot::Init() 
{

	//TODO : Modify motor index in model to 0 1 2
	int _timestep = (int)m_p_robot->getBasicTimeStep();
	for (int i = 0; i < 3; i++)
	{
		m_p_baseMotor[i]->enableTorqueFeedback(GLOBAL_SAMPLING_PERIOD);
		m_p_posSensor[i]->enable(GLOBAL_SAMPLING_PERIOD);
	}
	m_p_touchSensor->enable(GLOBAL_SAMPLING_PERIOD);
	m_p_distanceSensor->enable(GLOBAL_SAMPLING_PERIOD);
	m_p_keyBoard->enable(GLOBAL_SAMPLING_PERIOD);


	m_p_inputBuf =  new mwArray(1, 3, mxDOUBLE_CLASS, mxREAL);
	m_p_FKBuf = new mwArray();
	m_p_JacobBuf = new mwArray();
	//TODO: ADD DELETE!!!!

	PidXdir.kp = 10.0f;
	PidXdir.ki = 0.0f;
	PidXdir.kd = 0.0f;
	PidXdir.maxIOut = 2.0f;
	PidXdir.maxOut = 50.0f;
	PidXdir.Init();
	
	PidYdir.kp = 10.0f;
	PidYdir.ki = 0.0f;
	PidYdir.kd = 0.0f;
	PidYdir.maxIOut = 2.0f;
	PidYdir.maxOut = 50.0f;
	PidYdir.Init();

}

void Testbot::Update()
{
	
	UpdateFowardKinematics();
	UpdateState();
	//m_state = COMPRESSION;
	if (m_state != FLIGHT) {
		UpdateVMCForce();
		UpdateJointTorque();
		bodyVx = -m_footVel.x();
		bodyVy = -m_footVel.y();
	}else{
		UpdateLegPose();
	}
	

	//std::cout << m_state << std::endl;
	//std::cout << m_TsTick << std::endl;
	//std::cout << "dist feedback" << m_p_distanceSensor->getValue() << std::endl;

}


void Testbot::UpdateFowardKinematics()
{
	
	//update FK
	double _HipMotorPos[3] = { 0,0,0 };
	for (int i = 0; i < 3; i++)
	{
		_HipMotorPos[i] = m_p_posSensor[i]->getValue();
	}
	m_p_inputBuf -> SetData(_HipMotorPos, 3);
	//std::cout << *m_p_inputBuf << std::endl;
	DeltaFK(1,*m_p_FKBuf, *m_p_inputBuf);

	
	m_footLastPos = m_footPos;
	
	m_footPos.e[0] = m_p_FKBuf->Get(1, 1);
	m_footPos.e[1] = m_p_FKBuf->Get(1, 2);
	m_footPos.e[2] = m_p_FKBuf->Get(1, 3);

	m_footVel = (m_footPos - m_footLastPos)/ m_p_robot->getBasicTimeStep();

	//simple Jacobian
	for (int i = 0; i < 3; i++) 
	{
		_HipMotorPos[i] = _HipMotorPos[i] + 0.000001;
		m_p_inputBuf->SetData(_HipMotorPos, 3);
		DeltaFK(1, *m_p_JacobBuf, *m_p_inputBuf);
		for (int j = 0; j < 3; j++)
		{
			try
			{
				m_Jacobian[j][i] = (double)m_p_JacobBuf->Get(1, j+1) - (double)m_p_FKBuf->Get(1, j+1); //transpose the output, and remember that matlab index starts from 1.
			}
			catch(mwException e)
			{
				std::cout << e.what() << std::endl;
			}
			
			m_Jacobian[j][i] /= 0.000001;
			m_JacobianTranspose[i][j] = m_Jacobian[j][i];
		}
		_HipMotorPos[i] = _HipMotorPos[i] - 0.000001;
	}


}

static double test_lastLength = DEFAULT_VIRTUAL_LEG_LENGTH;
static bool thrust = false;


void Testbot::UpdateState()
{

	double _leglength = m_footPos.length();
	switch (m_state)
	{

		case LOADING:
		{
			if ((_leglength < DEFAULT_VIRTUAL_LEG_LENGTH) && (m_p_touchSensor->getValue() > 0.0f))
				m_state = COMPRESSION;
			break;
		}

		case COMPRESSION:
		{

			if (((m_footVel + m_footPos).length() > m_footPos.length()) && (m_p_touchSensor->getValue() > 0.0f))
				m_state = THRUST;
			break;
		}

		case THRUST:
		{
			if (_leglength > DEFAULT_VIRTUAL_LEG_LENGTH)
				m_state = UNLOADING;
			break;
		}

		case UNLOADING:
		{
			if (m_p_touchSensor->getValue() < 1.0f){
				m_state = FLIGHT;
				for (int i = 0; i < 3; i++) {
					m_baseMotorFlightAngleSet[i] = m_p_posSensor[i]->getValue();
				}
				m_TsTick = m_NewTsTick;
				m_NewTsTick = 0;
			}

			break;
		}

		case FLIGHT:
		{
			//m_p_led->set(0xffffff);
			if (m_p_touchSensor->getValue() > 0.0f||m_p_distanceSensor->getValue()<100.0f)
				m_state = LOADING;
			break;
		}
	}

	if (m_state != FLIGHT) {
		m_NewTsTick++;
	}


}

void Testbot::UpdateVMCForce()
{

	//TODO: ADD DIRECTION PID
	double _leglength = m_footPos.length();
	
	double _forceNorm = -800 * (_leglength - DEFAULT_VIRTUAL_LEG_LENGTH); //spring
	//_forceNorm -= 10 * m_footVel.length(); //damping

	if (m_state == THRUST)
	{
		_forceNorm = _forceNorm + 30;
	}

	m_footForce = _forceNorm * unit_vector(m_footPos);
	
	//std::cout << m_footForce << std::endl;

}

static double dbg_pos[3] = { 0.1,0.1,0.1 };

void Testbot::UpdateJointTorque()
{

	double _torque[3];
	matMultiplyVec3(m_JacobianTranspose, m_footForce, _torque);

	for (int i = 0; i < 3; i++)
	{
		m_p_baseMotor[i]->setTorque(_torque[i]);
	}

}

static double dir[4][2] = { 
	{ 0.0f, 0.0005f },//leg0
	{ -0.0003f,-0.0001731f },//leg1
	{ 0.0003f,-0.0001731f },//leg2
	{ 0.0f, 0.0002f }//freeze
};

//static double dir[4][2] = {
//	{ 0.0f, 0.0005f },//leg0
//	{ 0.0005f,-0.0f },//leg1
//	{ 0.0005f,0.0005f },//leg2
//	{ 0.0f, 0.000f }//freeze
//};

static int selectedDir = 3;

void Testbot::UpdateLegPose()
{
	switch (m_p_keyBoard->getKey()) {
		case webots::Keyboard::LEFT:
			selectedDir = 1;
			break;
		case  webots::Keyboard::RIGHT:
			selectedDir = 2;
			break;
		case webots::Keyboard::UP:
			selectedDir = 0;
			break;
		case webots::Keyboard::DOWN:
			selectedDir = 3;
			break;
	}
	
	std::cout << selectedDir << std::endl;

	simpleInv(m_Jacobian, m_JacobianInverse);

	double _vx0 = m_TsTick * bodyVx / 2.0;
	double _vy0 = m_TsTick * bodyVy / 2.0;

	double _vxf = 90.0 * (bodyVx - dir[selectedDir][0]);
	double _vyf = 90.0 * (bodyVy - dir[selectedDir][1]);

	//std::cout << "bodyVx" << bodyVx << std::endl;
	//std::cout << "bodyVy" << bodyVy << std::endl;
	//std::cout << "_vx0" << _vx0 << std::endl;
	//std::cout << "_vy0" << _vy0 << std::endl;
	//std::cout << "_vxf" << _vxf << std::endl;
	//std::cout << "_vyf" << _vyf << std::endl;

	vec3 tmp_footTargetPos(_vx0+_vxf, _vy0+_vyf, -DEFAULT_VIRTUAL_LEG_LENGTH);
	vec3 _deltaPos = (tmp_footTargetPos - m_footPos) / 20;
	
	double _deltaMotorPos[3];
	matMultiplyVec3(m_JacobianInverse, _deltaPos, _deltaMotorPos);

	for (int i = 0; i < 3; i++)
	{
		dbg_pos[i] += _deltaMotorPos[i];
		m_p_baseMotor[i]->setPosition(dbg_pos[i]);
	}

}
