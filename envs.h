#ifndef _ENVS_H
#define _ENVS_H

/* This class several environments:
	The sequence of design stages evolution must progress through.
	The current best robot (if any).
	And the robot that is currently being evaluated (if any).
 */

#include "optimizer.h"
#include "environment.h"
#include "tau.h"
#include "time.h"

class ENVS {

public:
	int viewMode;
	int simulateMode;
	int selectionLevel;
	int numberOfEnvs;
	ENVIRONMENT **taskEnvironments;
	OPTIMIZER   *optimizer;
	TAU *tau;	
	int recordingVideo;
	int speed;

private:
	int targetSensorValuesRecorded;
	int movieIndex;
	int frameIndex;
	int timeStepsSinceLastFrameCapture;
	int savedFileIndex;
	int activeEnvironment;
	float xyz[3];
	float hpr[3];
	int evaluationsSinceLastSave;
	int evaluationsSinceLastPreference;
	int currNumberOfEnvs;
	clock_t startTime;
	clock_t timeSinceLastWriteout;
	int randSeed;

public:
	ENVS(int rs);
	~ENVS();
	void Active_Element_Copy(void);
	void Active_Element_Delete(void);
	void Active_Element_Load(void);
	void Active_Element_Mark(void);
	void Active_Element_Unmark(void);
	void Active_Element_Move(double x, double y, double z);
	void Active_Element_Next(void);
	void Active_Element_Previous(void);
	void Active_Element_Resize(double changeX, double changeY, double changeZ);
	void Active_Element_Rotate(double rotX, double rotY, double rotZ);
	void Active_Element_Save(void);
	void Draw(void);
	void EvaluationPeriod_Decrease(dWorldID world, dSpaceID space, dJointGroupID contactgroup);
	void EvaluationPeriod_Increase(dWorldID world, dSpaceID space, dJointGroupID contactgroup);
	void Evolve(	dWorldID      world, 
			dSpaceID      space);
	int  In_Champ_Mode(void);
	int  In_Design_Mode(void);
	int  In_Evolution_Mode(void);
	int  In_TAU_Mode(void);
        void Joint_Connect(void);
        void Joint_Range_Decrease(void);
        void Joint_Range_Increase(void);
	void Load(int showGraphics);
	void Load_Pair(void);
	void Mode_Simulate_Set_Champ(dWorldID world, dSpaceID space);
	void Mode_Simulate_Set_Design(void);
	void Mode_Simulate_Set_Evolve(dWorldID world, dSpaceID space);
	void Mode_Simulate_Set_TAU(dWorldID world, dSpaceID space);
	void Mode_View_Set_Back(void);
	void Mode_View_Set_Side(void);
	void Mode_View_Set_Top(void);
	void MutationProbability_Decrease(void);
	void MutationProbability_Increase(void);
	void Num_Of_Sensors_Or_Motors_Changed(void);
	int  Pair_Available(void);
	void Prepare_To_Run(dWorldID world, dSpaceID space);
	void Reset(void);
        void Save(int showGraphics);
	void Selection_Level_Lower(void);
	void Selection_Level_Raise(void);
	void Show_Champ(	dWorldID      world, 
				dSpaceID      space);
	void Speed_Decrease(void);
	void Speed_Increase(void);
	NEURAL_NETWORK *TAU_Get_User_Favorite(void);
	int  TAU_Ready_To_Predict(void);
	double TAU_Score_Get(void);
	void TAU_Show_Robot_Pair( dWorldID world, dSpaceID space);
	void TAU_User_Has_Indicated_A_Preference(dWorldID world, dSpaceID space);
	void User_Wants_To_Provide_Preferences(dWorldID world, dSpaceID space);
        void User_Wants_To_Watch_Evolution(dWorldID world, dSpaceID space);
	void Video_Record(void);
	void Video_Start_Stop(void);
	void Viewpoint_Get(void);

private:
	void Activate_All(void);
	void Camera_Position_Load(ifstream *inFile, int showGraphics);
	void Camera_Position_Save(ofstream *outFile, int showGrahpics);
	void Check_For_Changed_Environment(dWorldID world, dSpaceID space);
	void Check_For_Pref(void);
	void Check_For_User_Request(void);
	void Check_Whether_To_Writeout(void);
	void Collect_Pref(char *fileName);
	void Create_Robot_Current_Best(	dWorldID      world, 
					dSpaceID      space);
	void Create_Robot_To_Evaluate(	dWorldID      world, 
					dSpaceID      space);
	void Deactivate_All(void);
	void Destroy(void);
	void Destroy_Simulated_Objects(void);
	int  Directory_Found(char *dirName);
	void Directory_Make(char *dirName);
	void End_Current_Mode(void);
	int  End_State_Missing(void);
	void Environment_Copy(void);
	void Environment_Delete(void);
	void Environment_Move(double x, double y, double z);
	void Environment_Next(void);
	void Environment_Previous(void);
	int  Eval_Finished(void);
	void Evolve_Allow_Robots_To_Move(dWorldID world, dSpaceID space);
	void Evolve_Times_Up(dWorldID world, dSpaceID space);
	void File_Delete(char *fileName);
	bool File_Exists(char *fileName);
	int  File_Index_Next_Available(void);
	void File_Rename(char *fileName, char *newFileName);
	double Fitness_Get(void);
	char Get_User_Request(char *fileName);
	void Hide_Robot_Joints(void);
	void Initialize(void);
	void Load_Environments(ifstream *inFile);
	void Load_Optimizer(ifstream *inFile);
	void Load_TAU(ifstream *inFile);
	void Move(double x, double y, double z);
	void Optimizer_Initialize(void);
	int  Optimizer_Robot_Mismatch(void);
	int  Ready_For_Champ_Mode(void);
	int  Ready_For_Design_Mode(void);
	int  Ready_For_Evolution_Mode(void);
	int  Ready_For_TAU_Mode(void);
	void Request_Evolving_Genomes(void);
	void Request_Pair(void);
	void Rescore_Population(void);
	void Reset(dWorldID world, dSpaceID space, dJointGroupID contactgroup);
	int  Robot_Being_Evaluated(void);
	void Save_Environments(ofstream *outFile);
	void Save_Fitness(string);
	void Save_Optimizer(ofstream *outFile, int saveTimeSeries);
	void Save_TAU(ofstream *outFile);
	void SavedFile_FindNext(void);
	void Send_Evolving_Genomes(void);
        void Send_Pair_For_Pref(void);
	void Sensor_Data_Receive(void);
	double Sensor_Sum(void);
	int  Target_Sensor_Values_Recorded(void);
	void TAU_Get_Controllers_From_Optimizer(void);
	void TAU_Load_Controller_Pair(ifstream *inFile);
	void TAU_Reset_User_Models(void);
	void TAU_Save_Controller_Pair(ofstream *outFile);
	void TAU_Send_Controllers_For_Evaluation(dWorldID world, dSpaceID space);
	void TAU_Store_Sensor_Data(void);
	void TAU_Store_User_Preference(void);
	void Unhide_All(void);
	void Video_Start(void);
	void Video_Stop(void);
	void Viewpoint_Set(	double x, double y, double z,
				double h, double p, double r);
};

#endif
