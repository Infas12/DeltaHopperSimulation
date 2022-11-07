#ifndef PID_HPP
#define PID_HPP

class Pid
{
public:
    enum PidModeType
    {
        PID_POSITION = 0,
        PID_DELTA,
    };

    PidModeType mode;

    float kp;
    float ki;
    float kd;

    float maxOut;
    float maxIOut;

    float ref;
    float fdb;

    float result;
    float pResult;
    float iResult;
    float dResult;

    float dBuf[3];
    float err[3];

    Pid();
    Pid(PidModeType mode, float p, float i, float d, float max, float imax);

    void Init();
    void UpdateResult();
    void Clear();

    void UpdateIResult(float _err){iResult += ki * _err;iResult = LimitMax(iResult, maxIOut);}

	static float LimitMax(float _input, float _max)
	{
		if (_input > _max)
		{
			return _max;
		}
		else if (_input < -_max)
		{
			return -_max;
		}
		return _input;
	}

};


#endif
