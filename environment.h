#ifndef _ENVIRONMENT_H
#define _ENVIRONMENT_H

/* Each environment contains one robot, one light source, and zero or more other objects. */

#include "matrix.h"
#include "object.h"
#include "robot.h"

class ENVIRONMENT {

public:
	OBJECT 		*lightSource;
	int		numOtherObjects;
	OBJECT 		**otherObjects;
	int		numRobots;
	class ROBOT  	**robots;
	int		robotIndex;
	int    		activeElement;

private:
	int 		savedFileIndex;
	class ENVS	*containerEnvs;

public:
	ENVIRONMENT(ENVS *cE);
	ENVIRONMENT(ENVS *CE, ENVIRONMENT *other);
	ENVIRONMENT(ENVS *CE, ifstream *inFile);
	~ENVIRONMENT();
	void Activate_All(void);
	void Activate_Light_Source(void);
	void Activate_Robot(int robotIndex);
	void Activate_Component(int robotIndex);
	void Active_Component_Copy(void);
	void Active_Component_Delete(void);
	void Active_Component_Move(double x, double y, double z);
	void Active_Component_Next(void);
	void Active_Component_Previous(void);
	void Active_Component_Resize(double changeX, double changeY, double changeZ);
	void Active_Component_Rotate(double,double,double);
	void Active_Element_Copy(void);
	void Active_Element_Delete(void);
	void Active_Element_Move(double x, double y, double z);
	void Active_Element_Next(void);
	void Active_Element_Previous(void);
	void Active_Element_Resize(double changeX, double changeY, double changeZ);
	void Active_Element_Rotate(double,double,double);
	void Active_Joint_Range_Decrease(void);
	void Active_Joint_Range_Increase(void);
	void Add_Light_Source(void);
	void Add_Robot_Sandbox(void);
	void Add_Robot_Starfish(void);
	void Allow_Robot_To_Move(int timeStep);
	void Connect_Robot_Joint(void);
	void Deactivate_All(void);
	void Destroy_Simulated_Objects(void);
	void Draw(void);
	double Fitness_Get(void);
	MATRIX *Get_Sensor_Data(void);
	void Hide_Light_Source(void);
	void Hide_Other_Objects(void);
	void Hide_Robot(int robotIndex);
	void Hide_Robot_Joints(void);
	void Label(class NEURAL_NETWORK *genome, int environmentIndex);
	void Load(void);
	void Make_NonRobotObjects_Incorporeal(void);
	void Mark_Component(void);
	void Mark_Object(void);
	void Move(double x, double y, double z);
	void Num_Of_Sensors_Or_Motors_Changed(void);
	void Prepare_For_Simulation(dWorldID world, dSpaceID space);
	void Record_Sensor_Data(int evaluationPeriod);
	void Robot_Copy(void);
	void Robot_Delete(void);
	int  Robot_Joint_Is_Unattached(void);
	void Robot_Joint_Unhide_Unattached(void);
	void Robots_Recolor(char color);
	void Robots_Set_Color(double r, double g, double b);
	void Save(void);
	void Save(ofstream *outFile);
	double Sensor_Sum(void);
	void Set_Color(double r, double g, double b);
	void Unhide_All(void);
	void Unmark_All(void);
	void Unmark_Component(void);
	void Unmark_Object(void);

private:

	void Activate_Current_Element(void);
	void Activate_Current_Component(int robotIndex);
	int  Active_Element_Is_Object(void);
	void Component_Copy(void);
	void Deactivate_Current_Component(void);
	void Deactivate_Current_Element(void);
	void Destroy(void);
	bool File_Exists(char *fileName);
	int  File_Index_Next_Available(void);
	void Initialize(void);
	void Load(ifstream *inFile);
	void Object_Copy(void);
	void Object_Delete(void);
	void Robot_Delete(int robotIndex);
	void SavedFile_FindNext(void);
};

#endif
