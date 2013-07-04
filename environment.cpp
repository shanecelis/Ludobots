#include "stdio.h"
#include "iostream"
#include "fstream"

#ifndef _ENVIRONMENT_CPP
#define _ENVIRONMENT_CPP

#include "envs.h"
#include "environment.h"
#include "constants.h"
#include "matrix.h"

ENVIRONMENT::ENVIRONMENT(ENVS *cE) {

	containerEnvs = cE;

	Initialize();

	numOtherObjects = 0;

	activeElement = -1;

	savedFileIndex = -1;
}

ENVIRONMENT::ENVIRONMENT(ENVS *cE, ENVIRONMENT *other) {

	containerEnvs = cE;
	
	Initialize();

	if ( other->lightSource )
		
		lightSource = new OBJECT(this,other->lightSource);

	numOtherObjects = other->numOtherObjects;
	for (int i=0;	i<numOtherObjects;	i++)
		if ( other->otherObjects[i] )
			otherObjects[i] = new OBJECT(this,
						other->otherObjects[i]);

	numRobots = other->numRobots;
	for (int i=0;	i<numRobots;	i++)
		if ( other->robots[i] )
			robots[i] = new ROBOT(this,other->robots[i]);

	activeElement = -1;

	savedFileIndex = -1;
}

ENVIRONMENT::ENVIRONMENT(ENVS *cE, ifstream *inFile) {

	containerEnvs = cE;

	Initialize();

	Load(inFile);

	savedFileIndex = -1;
}

ENVIRONMENT::~ENVIRONMENT(void) {

	Destroy();
}

void ENVIRONMENT::Activate_All(void) {

	if ( lightSource )

		lightSource->Activate();

	if ( otherObjects ) {
	
		for (int i=0;	i<numOtherObjects;	i++)

			if ( otherObjects[i] )
				otherObjects[i]->Activate();
	}

	if ( robots ) {

		for (int i=0;	i<numRobots;	i++)

			if ( robots[i] )
				robots[i]->Activate();
	}

	activeElement = -1;
}

void ENVIRONMENT::Activate_Light_Source(void) {

	// Light source already active
	if ( activeElement==0 )
		return;

	Deactivate_All();
	// Deactivate anything that's currently active

	activeElement = 0;

	Activate_Current_Element();
}

void ENVIRONMENT::Activate_Robot(int robotIndex) {

	if ( numRobots<(robotIndex+1) )
		return;

	Deactivate_All();
	// Deactivate anything that's currently active

	activeElement = 1 + numOtherObjects + robotIndex;

	Activate_Current_Element();
}

void ENVIRONMENT::Activate_Component(int robotIndex) {

	if ( numRobots < 1 )

		return;

	// Deactivate anything that's currently active
	Deactivate_All();

	if ( robots && robots[robotIndex] )

		robots[robotIndex]->Activate_Component();
}

void ENVIRONMENT::Active_Component_Copy(void) {

	if ( robots && robots[robotIndex] && (robots[robotIndex]->numObjects < MAX_COMPONENTS) )

		robots[robotIndex]->Active_Component_Copy();
}

void ENVIRONMENT::Active_Component_Delete(void) {

	if ( robots && robots[robotIndex] )

		robots[robotIndex]->Active_Component_Delete();
}

void ENVIRONMENT::Active_Component_Move(double x, double y, double z) {

	if ( robots && robots[robotIndex] )

		robots[robotIndex]->Active_Component_Move(x,y,z);
}

void ENVIRONMENT::Active_Component_Next(void) {

	Deactivate_Current_Component();

	if ( robots && robots[robotIndex] ) {

		robots[robotIndex]->Active_Component_Increment();

		robots[robotIndex]->Activate_Component();
	}
}

void ENVIRONMENT::Active_Component_Previous(void) {

	Deactivate_Current_Component();

	if ( robots && robots[robotIndex] ) {

		robots[robotIndex]->Active_Component_Decrement();

		robots[robotIndex]->Activate_Component();
	}
}

void ENVIRONMENT::Active_Component_Resize(double changeX, double changeY, double changeZ) {

	if ( robots && robots[robotIndex] ) 

		robots[robotIndex]->Active_Component_Resize(changeX,changeY,changeZ);
}

void ENVIRONMENT::Active_Joint_Range_Decrease(void) {

	if ( robots && robots[robotIndex] ) 

		robots[robotIndex]->Active_Joint_Range_Change(1/ROBOT_JOINT_RANGE_DELTA);
}

void ENVIRONMENT::Active_Joint_Range_Increase(void) {

	if ( robots && robots[robotIndex] ) 

		robots[robotIndex]->Active_Joint_Range_Change(ROBOT_JOINT_RANGE_DELTA);
}

void ENVIRONMENT::Active_Component_Rotate(double rotX, double rotY, double rotZ) {

	if ( robots && robots[robotIndex] ) 

		robots[robotIndex]->Active_Component_Rotate(rotX,rotY,rotZ);
}

void ENVIRONMENT::Active_Element_Copy(void) {

	if ( activeElement >= (1+numOtherObjects) )

		Robot_Copy();

	else
		Object_Copy();
}

void ENVIRONMENT::Active_Element_Delete(void) {

	// Only the last robot in the sequence can be deleted.

	if ( activeElement >= (1+numOtherObjects) )

		Robot_Delete();

	else
		Object_Delete();
}

void ENVIRONMENT::Active_Element_Move(double x, double y, double z) {

	if ( activeElement==0 ) {

		if ( lightSource )
			lightSource->Move(x,y,z);
	}

	else if ( activeElement <= numOtherObjects ) {

		int objectIndex = activeElement-1;

		if ( otherObjects && otherObjects[objectIndex] )

			otherObjects[objectIndex]->Move(x,y,z);
	}
	else {

		int robotIndex = activeElement-numOtherObjects-1;

		if ( robots && robots[robotIndex] )

			robots[robotIndex]->Move(x,y,z);
	}
}

void ENVIRONMENT::Active_Element_Next(void) {

	Deactivate_Current_Element();

	activeElement++;

	if ( activeElement > (numOtherObjects+numRobots) )
		activeElement = 0;

	Activate_Current_Element();
}

void ENVIRONMENT::Active_Element_Previous(void) {

	Deactivate_Current_Element();

	activeElement--;

	if ( activeElement < 0 )
		activeElement = numOtherObjects+numRobots;

	Activate_Current_Element();
}

void ENVIRONMENT::Active_Element_Resize(double changeX, double changeY, double changeZ) {

	if ( otherObjects && otherObjects[activeElement-1] && Active_Element_Is_Object() )

		otherObjects[activeElement-1]->Resize(changeX,changeY,changeZ);
}

void ENVIRONMENT::Active_Element_Rotate(double rotX, double rotY, double rotZ) {

	if ( otherObjects && otherObjects[activeElement-1] && Active_Element_Is_Object() )

		otherObjects[activeElement-1]->Rotate(rotX,rotY,rotZ);
}

void ENVIRONMENT::Add_Light_Source(void) {

	lightSource = new OBJECT(	SHAPE_RECTANGLE,
					LIGHT_SOURCE_LENGTH,
					LIGHT_SOURCE_LENGTH,
					LIGHT_SOURCE_LENGTH,
					0,
					LIGHT_SOURCE_DISTANCE,
					(LIGHT_SOURCE_LENGTH)/2.0,
					0,0,1
					);
	lightSource->containerEnvironment = this;

	activeElement = 0;
	lightSource->Activate();
}

void ENVIRONMENT::Add_Robot_Sandbox(void) {

	// parts for user-created robots

	if ( robots && (!robots[numRobots]) && (numRobots < MAX_ROBOTS) ) {

		robots[numRobots] = new ROBOT(this,ROBOT_SANDBOX);
		robots[numRobots]->Deactivate();

		numRobots++;
	}
}

void ENVIRONMENT::Add_Robot_Starfish(void) {

	if ( robots && (!robots[numRobots]) && (numRobots < MAX_ROBOTS) ) {

		robots[numRobots] = new ROBOT(this,ROBOT_STARFISH);
		robots[numRobots]->Deactivate();

		numRobots++;
	}
}

void ENVIRONMENT::Allow_Robot_To_Move(int timeStep) {

	if ( robots && robots[0] ) {

		robots[0]->Sensors_Update();
		robots[0]->Sensors_Add_Difference(robots[1]);
		robots[0]->Move(timeStep);
	}
}

void ENVIRONMENT::Connect_Robot_Joint(void) {

	if ( numRobots < 1 )
		return;

	if ( robots && robots[robotIndex] )

		robots[robotIndex]->Connect_Robot_Joint();
}

void ENVIRONMENT::Deactivate_All(void) {

	if ( lightSource )

		lightSource->Deactivate();

	if ( otherObjects ) {

		for (int i=0;	i<numOtherObjects;	i++)

			if ( otherObjects[i] )
				otherObjects[i]->Deactivate();
	}

	if ( robots ) {

		for (int i=0;	i<numRobots;	i++)

			if ( robots[i] )
				robots[i]->Deactivate();
	}

	activeElement = -1;

}

void ENVIRONMENT::Destroy_Simulated_Objects(void) {

	if ( lightSource )

		lightSource->Make_Incorporeal();

	if ( otherObjects ) {

		for (int i=0;	i<numOtherObjects;	i++)

			if ( otherObjects[i] )
				otherObjects[i]->Make_Incorporeal();
	}

	if ( robots ) {

		for (int i=0;	i<numRobots;	i++)

			if ( robots[i] )
				robots[i]->Make_Incorporeal();
	}
}

void ENVIRONMENT::Draw(void) {

	if ( lightSource )

		lightSource->Draw();
	
	if ( otherObjects ) {

		for (int i=0;	i<numOtherObjects;	i++)

			if ( otherObjects[i] )
				otherObjects[i]->Draw();
	}

	if ( robots ) {

		for (int i=0;	i<numRobots;	i++)

			if ( robots[i] )
				robots[i]->Draw();
	}
}

double ENVIRONMENT::Fitness_Get(void) {

	if ( robots && robots[0] ) {

		robots[0]->Sensors_Update();

		return( robots[0]->Fitness_Get(robots[1]) );
	}

	return( WORST_FITNESS );
}

MATRIX *ENVIRONMENT::Get_Sensor_Data(void) {

	if ( robots && robots[0] )

		return( robots[0]->Get_Sensor_Data() );
	else
		return( NULL );
}

void ENVIRONMENT::Hide_Light_Source(void) {

	if ( lightSource )
		lightSource->Hide();
}

void ENVIRONMENT::Hide_Other_Objects(void) {

	for (int i=0;i<numOtherObjects;i++)

		if ( otherObjects && otherObjects[i] )

			otherObjects[i]->Hide();
}

void ENVIRONMENT::Hide_Robot_Joints(void) {

	if ( robots ) {

		for (int i = 0; i < numRobots; i++ )

			if ( robots[i] )

				robots[i] -> Hide_Robot_Joints();
	}
}

void ENVIRONMENT::Hide_Robot(int robotIndex) {

	if ( robots && robots[robotIndex] )

		robots[robotIndex]->Hide();
}

void ENVIRONMENT::Record_Sensor_Data(int evaluationPeriod) {

	if ( robots && robots[0] )

		robots[0]->Record_Sensor_Data(evaluationPeriod);
}

int ENVIRONMENT::Robot_Joint_Is_Unattached(void) {

	int jointUnattached = false;

	for (int i=0; i<numRobots; i++) {

		if ( robots && robots[i] && robots[i]->Joint_Is_Unattached() )

			jointUnattached = true;
	}
	return jointUnattached;
}

void ENVIRONMENT::Robot_Joint_Unhide_Unattached(void) {

	for (int i=0; i<numRobots; i++) {

		if ( robots && robots[i] && robots[i]->Joint_Is_Unattached() )

			robots[i]->Unattached_Joints_Unhide();
	}
}

void ENVIRONMENT::Label(NEURAL_NETWORK *genome, int environmentIndex) {

	if ( robots && robots[0] )

		robots[0]->Label(genome,environmentIndex);
}

void ENVIRONMENT::Load(void) {

	SavedFile_FindNext();

	char fileName[100];
	sprintf(fileName,"SavedFiles/env%d.dat",savedFileIndex);

	ifstream *inFile = new ifstream(fileName);

	if ( !inFile->is_open() ) {

		printf("Can't open environment file to load.\n");
	}
	else {

		Destroy();

		Initialize();

		Load(inFile);

		printf("Environment %d loaded.\n",savedFileIndex);
	}

	inFile->close();
	delete inFile;
	inFile = NULL;
}

void ENVIRONMENT::Make_NonRobotObjects_Incorporeal(void) {

	if ( lightSource )

		lightSource->Make_Incorporeal();

	for (int i=0;i<numOtherObjects;i++)

		if ( otherObjects && otherObjects[i] )

			otherObjects[i]->Make_Incorporeal();
}

void ENVIRONMENT::Mark_Component(void) {

	if ( numRobots < 1 )
		return;

	if ( robots && robots[robotIndex] )

		robots[robotIndex]->Mark_Component();
}

void ENVIRONMENT::Mark_Object(void) {

	if ( otherObjects && otherObjects[activeElement] )

		otherObjects[activeElement]->Mark();
}

void ENVIRONMENT::Move(double x, double y, double z) {

	if ( lightSource )

		lightSource->Move(x,y,z);

	for (int i=0;	i<numOtherObjects;	i++)

		if ( otherObjects && otherObjects[i] )

			otherObjects[i]->Move(x,y,z);

	for (int i=0;	i<numRobots;	i++)

		if ( robots && robots[i] )

			robots[i]->Move(x,y,z);
}

void ENVIRONMENT::Num_Of_Sensors_Or_Motors_Changed(void) {

	// The user has added or removed a component from the robot.
	// This means that we must delete the target robot position,
	// because the initial and target robot positions now have
	// different numbers of sensors or motors.
	// The user will have to create a new copy of the initial
	// robot and place it as the target robot position.

	// Point at the target robot, if there is one.

	activeElement = (numOtherObjects+numRobots);

	// ...then delete it:

	Robot_Delete();

	if ( containerEnvs )

		containerEnvs->Num_Of_Sensors_Or_Motors_Changed();
}

void ENVIRONMENT::Prepare_For_Simulation(dWorldID world, 
					dSpaceID space) {

	if ( lightSource )

		lightSource->Make_Solid(world,space);

	for (int i=0;i<numOtherObjects;i++)

		if ( otherObjects && otherObjects[i] )

			otherObjects[i]->Make_Solid(world,space);

	if ( robots && robots[0] )

		robots[0]->Make_Physical(world,space);

	if ( robots && robots[0] )

		robots[0]->sensorDifferences = 0.0;

	if ( robots && robots[1] )

		robots[1]->Sensors_Update();
}

void ENVIRONMENT::Robot_Copy(void) {

	if ( !robots )
		return;

	if ( activeElement >= (1+numOtherObjects) ) {

		if ( numRobots < MAX_ROBOTS ) {
	
			ROBOT *currentRobot = 
			robots[activeElement-numOtherObjects-1];

			robots[numRobots] = 
			new ROBOT( currentRobot );

			currentRobot->Deactivate();
			
			robots[numRobots]->Activate();
			
			robots[numRobots]->Move(-0.1,0.1,0);

			activeElement = 1+numOtherObjects+numRobots;

			numRobots++;

			currentRobot = NULL;
		}

		Robots_Recolor('r');
	}
}

void ENVIRONMENT::Object_Copy(void) {

	if ( numOtherObjects >= MAX_OTHER_OBJECTS )

		return;

	if ( activeElement == 0 ) {

		otherObjects[numOtherObjects] = new OBJECT(this,lightSource);

		if ( lightSource )

			lightSource->Deactivate();
	}
	else {
		otherObjects[numOtherObjects] = new OBJECT(this,otherObjects[activeElement-1]);

		otherObjects[activeElement-1]->Deactivate();

	}

	otherObjects[numOtherObjects]->Set_Color(0.5,0.5,0.5);

	otherObjects[numOtherObjects]->Activate();

	otherObjects[numOtherObjects]->Move(-0.1,0.1,0);

	activeElement = 1 + numOtherObjects;

	numOtherObjects++;		
}

void ENVIRONMENT::Object_Delete(void) {

	// If there are no other objects to delete...
	if ( numOtherObjects == 0 )
		return;

	if ( !otherObjects )
		return;

	if ( !otherObjects[numOtherObjects-1] )
		return;

	// Only allow deletion of the last object in the set.
	if ( activeElement==numOtherObjects ) {

		delete otherObjects[numOtherObjects-1];
		otherObjects[numOtherObjects-1] = NULL;
		numOtherObjects--;
		activeElement--;

		if ( activeElement==0 ) {
			if ( lightSource ) 
				lightSource->Activate();
		}
		else {
			if ( otherObjects[numOtherObjects-1] )

				otherObjects[numOtherObjects-1]->Activate();
		}
	}
}

void ENVIRONMENT::Robot_Delete(void) {

	// If there's only one robot in the environment, don't delete it.
	if ( numRobots == 1 )
		return;

	if ( !robots )
		return;

	if ( !robots[numRobots-1] )
		return;

	// Only allow deletion of the last robot in the set.
	if ( activeElement == (numOtherObjects+numRobots) ) {
		
		delete robots[numRobots-1];
		robots[numRobots-1] = NULL;
		numRobots--;
		activeElement--;

		if ( robots[numRobots-1] )	

			robots[numRobots-1]->Activate();

		Robots_Recolor('r');
	}
}

void ENVIRONMENT::Robots_Recolor(char color) {

	// Lighter-colored robots are further ahead in the sequence

	if ( !robots )
		return;

	if ( numRobots > 1 ) {

		double colorIncrement = 0.7/(double(numRobots)-1.0);

		for (int i=0; i<numRobots; i++) {

			if ( color == 'r' || color == 'R') {

				robots[i]->Set_Color(
					1.0,
					i*colorIncrement,
					i*colorIncrement);
			}

			if ( color == 'g' || color == 'G') {
				robots[i]->Set_Color(
					i*colorIncrement,
					1.0,
					i*colorIncrement);
			}

			if ( color == 'b' || color == 'B') {
				robots[i]->Set_Color(
					i*colorIncrement,
					i*colorIncrement,
					1.0);
			}
		}
	}
}

void ENVIRONMENT::Robots_Set_Color(double r, double g, double b) {

	if ( !robots )

		return;

	for (int i=0;	i<numRobots;	i++)

		if ( robots[i] )

			robots[i]->Set_Color(r,g,b);
}

void ENVIRONMENT::Save(void) {

	int fileIndex = File_Index_Next_Available();

	char fileName[100];
	sprintf(fileName,"SavedFiles/env%d.dat",fileIndex);

	ofstream *outFile = new ofstream(fileName);

	Save(outFile);

	outFile->close();
	delete outFile;
	outFile = NULL;

	printf("Environment %d saved.\n",fileIndex);
}

void ENVIRONMENT::Save(ofstream *outFile) {

	if ( lightSource )
		lightSource->Save(outFile);

	if ( otherObjects ) {

		(*outFile) << numOtherObjects << "\n";
		for (int i=0;i<numOtherObjects;i++)
			if ( otherObjects[i] )
				otherObjects[i]->Save(outFile);
	}

	if ( robots ) {

		(*outFile) << numRobots << "\n";
		for (int i=0;i<numRobots;i++)
			if ( robots[i] )
				robots[i]->Save(outFile);
	}

	(*outFile) << activeElement << "\n";
}

double ENVIRONMENT::Sensor_Sum(void) {

	if ( robots && robots[0] )

		return( robots[0]->Sensor_Sum() );

	else

		return( 0.0 );
}

void ENVIRONMENT::Set_Color(double r, double g, double b) {

	if ( lightSource )

		lightSource->Set_Color(r,g,b);


	for (int i=0;	i<numOtherObjects;	i++)

		if ( otherObjects && otherObjects[i] )

			otherObjects[i]->Set_Color(r,g,b);

	Robots_Set_Color(r,g,b);
}

void ENVIRONMENT::Unhide_All(void) {

	if ( lightSource )

		lightSource->Unhide();


	if ( otherObjects )  {

		for (int i=0;	i<numOtherObjects;	i++)

			if ( otherObjects[i] )

				otherObjects[i]->Unhide();
	}

	if ( robots ) {

		for (int i=0;	i<numRobots;	i++)

			if ( robots[i] )

				robots[i]->Unhide();
	}
}

void ENVIRONMENT::Unmark_All(void) {

	if ( lightSource )
		lightSource->Unmark();

	if ( otherObjects ) {

		for (int i=0;	i<numOtherObjects;	i++)
			if ( otherObjects[i] )
				otherObjects[i]->Unmark();
	}

	if ( robots ) {

		for (int i=0;	i<numRobots;	i++)

			if ( robots[i] )
				robots[i]->Unmark_All();
	}
}

void ENVIRONMENT::Unmark_Component(void) {

	if ( numRobots < 1 )
		return;

	if ( robots && robots[robotIndex] )

		robots[robotIndex]->Unmark_Component();
}

void ENVIRONMENT::Unmark_Object(void) {

	if ( numRobots < 1 )
		return;

	if ( otherObjects && otherObjects[activeElement] )

		otherObjects[activeElement]->Unmark();
}

// ------------------ Private methods --------------------

void ENVIRONMENT::Activate_Current_Element(void) {

	if ( activeElement==0 ) {

		if ( lightSource )
			lightSource->Activate();
	}

	else if ( activeElement <= numOtherObjects ) {

		if ( otherObjects && otherObjects[activeElement-1] )

			otherObjects[activeElement-1]->Activate();
	}
	else {
		if ( robots && robots[activeElement-numOtherObjects-1] )

			robots[activeElement-numOtherObjects-1]->Activate();
	}
}

void ENVIRONMENT::Activate_Current_Component(int robotIndex) {

	if ( robots )

		robots[robotIndex]->Activate_Component();
}

int  ENVIRONMENT::Active_Element_Is_Object(void) {

	int isNotLightSource 	= activeElement > 0;
	int isNotRobot 		= activeElement < (1+numOtherObjects);

	return( isNotLightSource && isNotRobot );
}

void ENVIRONMENT::Deactivate_Current_Component(void) {

	if ( robots )

		robots[robotIndex]->Deactivate_Component();
}

void ENVIRONMENT::Deactivate_Current_Element(void) {

	if ( activeElement==0 ) {

		if ( lightSource )
			lightSource->Deactivate();
	}

	else if ( activeElement <= numOtherObjects ) {

		if ( otherObjects && otherObjects[activeElement-1] )

			otherObjects[activeElement-1]->Deactivate();
	}
	else {
		if ( robots && robots[activeElement-numOtherObjects-1] )

			robots[activeElement-numOtherObjects-1]->Deactivate();
	}
}

void ENVIRONMENT::Destroy(void) {

	if ( containerEnvs )

		containerEnvs = NULL;

	if ( lightSource ) {
		delete lightSource;
		lightSource = NULL;
	}

	if ( otherObjects ) {

		for (int i=0;	i<MAX_OTHER_OBJECTS;	i++) {

			if ( otherObjects[i] ) {
				delete otherObjects[i];
				otherObjects[i] = NULL;
			}
		}
	
		delete[] otherObjects;
		otherObjects = NULL;	
	}

	if ( robots ) {

		for (int i=0;	i<MAX_ROBOTS;	i++) {

			if ( robots[i] ) {
				delete robots[i];
				robots[i] = NULL;
			}
		}
	
		delete[] robots;
		robots = NULL;
	}
}

bool  ENVIRONMENT::File_Exists(char *fileName) {

	ifstream ifile(fileName);
	return ifile;
}

int  ENVIRONMENT::File_Index_Next_Available(void) {

	int fileIndex = 0;
	char fileName[100];
	sprintf(fileName,"SavedFiles/env%d.dat",fileIndex);
	while ( File_Exists(fileName) ) {
		fileIndex++;
		sprintf(fileName,"SavedFiles/env%d.dat",fileIndex);
	}

	return( fileIndex );
}

void ENVIRONMENT::Initialize(void) {

	lightSource = NULL;

	otherObjects = new OBJECT * [MAX_OTHER_OBJECTS];
	for (int i=0;	i<MAX_OTHER_OBJECTS;	i++)
		otherObjects[i] = NULL;

	numRobots = 0;
	robots = new ROBOT * [MAX_ROBOTS];
	for (int i=0;	i<MAX_ROBOTS;	i++)
		robots[i] = NULL;

	robotIndex = 0;
}

void ENVIRONMENT::Load(ifstream *inFile) {

	lightSource = new OBJECT(this,inFile);

	(*inFile) >> numOtherObjects;
	for (int i=0;			i<numOtherObjects;	i++)
		otherObjects[i] = new OBJECT(this,inFile);
	
	(*inFile) >> numRobots;
	for (int i=0;			i<numRobots;		i++)
		robots[i] = new ROBOT(this,inFile);	

	(*inFile) >> activeElement;
}

void ENVIRONMENT::Robot_Delete(int robotIndex) {

	if ( robots && robots[robotIndex] ) {
		delete robots[robotIndex];
		robots[robotIndex] = NULL;
	}
}

void ENVIRONMENT::SavedFile_FindNext(void) {

	savedFileIndex++;

	char fileName[100];
	sprintf(fileName,"SavedFiles/env%d.dat",savedFileIndex);

	if ( !File_Exists(fileName) )

		savedFileIndex = 0;
}

#endif
