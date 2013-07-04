/* A sensor that returns the heading direction of the current object. */

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include "robot.h"

#ifndef _COMPASS_SENSOR_H
#define _COMPASS_SENSOR_H

class COMPASS_SENSOR {

public:
	class ROBOT *containerRobot;
	double value;

public:
	COMPASS_SENSOR(ROBOT *cR);
	~COMPASS_SENSOR(void);
	double Get_Value(void);
	void Print(void);
	void Reset(void);
};

#endif
