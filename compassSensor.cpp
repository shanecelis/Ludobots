/* A sensor that returns the heading direction of the current object. */

#include "stdio.h"

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifndef _COMPASS_SENSOR_CPP
#define _COMPASS_SENSOR_CPP

#include "compassSensor.h"

COMPASS_SENSOR::COMPASS_SENSOR(ROBOT *cR) {

	containerRobot = cR;
	
	Reset();
}

COMPASS_SENSOR::~COMPASS_SENSOR(void) {

	if ( containerRobot )
		containerRobot = NULL;
}

double COMPASS_SENSOR::Get_Value(void) {

	// Return the compass sensor's value, which should range
	// between zero and one.

	// We calculate the heading of the robot based using
	// the first and second objects that comprise it.
	// If the robot is made up of one or fewer parts,
	// the compass sensor simply returns zero.

	if ( !containerRobot )

		return( 0.0 );

	if ( !containerRobot->objects[0] )

		return( 0.0 );

	if ( !containerRobot->objects[0]->body )

		return( 0.0 );

	if ( !containerRobot->objects[1] )

		return( 0.0 );

	if ( !containerRobot->objects[1]->body )

		return( 0.0 );

        const dReal *firstObjectPos  = dBodyGetPosition(containerRobot->objects[0]->body);
        const dReal *secondObjectPos = dBodyGetPosition(containerRobot->objects[1]->body);

        double vect1[2], vect2[2];

        vect1[0] = secondObjectPos[0] - firstObjectPos[0];
        vect1[1] = secondObjectPos[1] - firstObjectPos[1];

        double theta = atan( vect1[1] / vect1[0] );

        if ( (vect1[0]>=0) && (vect1[1]>=0) )

                theta = theta + 0.0;

        else if ( (vect1[0]<0) && (vect1[1]>=0) ) // Quadrant 2

                theta = theta + 3.14159;

        else if ( (vect1[0]>=0) && (vect1[1]<0) ) // Quadrant 4

                theta = theta + 2.0*3.14159;

        else //Quadrant 3

                theta = theta + 3.14159;

        // Convert theta from [0,2pi] to [0,1]
        theta = theta / (2.0*3.14159);

	value = theta;

        return( theta );
}

void COMPASS_SENSOR::Print(void) {

	printf("compass sensor: %3.3f\n",value);
}

void COMPASS_SENSOR::Reset(void) {

	value = 0.0;
}

#endif
