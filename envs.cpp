#include "stdio.h"
#include "sys/stat.h"
#include <time.h>	// for fitness timestamp

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

//#include <GL/glut.h>


#ifndef _ENVS_CPP
#define _ENVS_CPP

#include "envs.h"
#include "optimizer.h"

extern int	MODE_VIEW_SIDE;
extern int	MODE_VIEW_TOP;
extern int	MODE_VIEW_BACK;

extern int	MODE_SIMULATE_DESIGN;
extern int	MODE_SIMULATE_EVOLVE;
extern int	MODE_SIMULATE_CHAMP;
extern int	MODE_SIMULATE_TAU;

extern int	TIME_STEPS_PER_FRAME;

extern int	SELECTION_LEVEL_OBJECT;
extern int	SELECTION_LEVEL_ROBOT;
extern int	SELECTION_LEVEL_ENVIRONMENT;
extern int	SELECTION_LEVEL_ENVS;

extern int	MAX_ENVIRONMENTS;

extern double	MOVE_INCREMENT;

extern int	MAX_EVALS_BEFORE_SAVING;

extern double	WORST_FITNESS;

extern int	TIME_TO_CHECK_FOR_NEUTRAL_MUTATION;

extern int	AFPO_POP_SIZE;

extern int	ALG_VARIANT_FITS_ONLY;
extern int	ALG_VARIANT_PREFS_ONLY;

extern int	STARTING_EVALUATION_TIME;

ENVS::ENVS(int rs) {

	speed = 1;

	randSeed = rs;

	Initialize();

	char command[200];
	sprintf(command,"rm SavedFiles/writeout%d.txt",randSeed);
	system(command);

	sprintf(command,"rm SavedFiles/pair*.dat");
	system(command);

	sprintf(command,"rm SavedFiles/pref*.dat");
	system(command);

	sprintf(command,"rm SavedFiles/programResponse.txt");

}

ENVS::~ENVS(void) {

	Destroy();
}

void ENVS::Active_Element_Copy(void) {

	if ( simulateMode == MODE_SIMULATE_DESIGN )

		if ( selectionLevel == SELECTION_LEVEL_ENVIRONMENT ) {

			// TODO: Currently the ability to add another
			// environment is disabled, because it is not
			// clear how a user should provide preferences
			// between two robots, each of which behave
			// in multiple environments.

			// Environment_Copy();
		}
		else if ( selectionLevel == SELECTION_LEVEL_OBJECT )

			taskEnvironments[activeEnvironment]->Active_Element_Copy();

		else if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Active_Component_Copy();
}

void ENVS::Active_Element_Delete(void) {

	if ( simulateMode == MODE_SIMULATE_DESIGN )

		if ( selectionLevel == SELECTION_LEVEL_ENVIRONMENT )

			Environment_Delete();

		else if ( selectionLevel == SELECTION_LEVEL_OBJECT )

			taskEnvironments[activeEnvironment]->Active_Element_Delete();

		else if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Active_Component_Delete();
}

void ENVS::Active_Element_Load(void) {

	if ( simulateMode == MODE_SIMULATE_DESIGN ) {

		     if ( selectionLevel == SELECTION_LEVEL_ENVIRONMENT )

			taskEnvironments[activeEnvironment]->Load();

		else if ( selectionLevel == SELECTION_LEVEL_ENVS )
			Load(true);

		// Robots during design are always red.
		for (int i=0;	i<numberOfEnvs;	i++) {

//			taskEnvironments[i]->Robots_Set_Color(1,0,0);

			taskEnvironments[i]->Robots_Recolor('r');
		}
	}
}

void ENVS::Active_Element_Mark(void) {

	if ( simulateMode == MODE_SIMULATE_DESIGN )

		if ( selectionLevel == SELECTION_LEVEL_OBJECT )

			taskEnvironments[activeEnvironment]->Mark_Object();

		else if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Mark_Component();
}

void ENVS::Active_Element_Unmark(void) {

	if ( simulateMode == MODE_SIMULATE_DESIGN )

		if ( selectionLevel == SELECTION_LEVEL_OBJECT )

			taskEnvironments[activeEnvironment]->Unmark_Object();

		else if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Unmark_Component();
}

void ENVS::Active_Element_Move(double x, double y, double z) {

	if ( simulateMode == MODE_SIMULATE_DESIGN )

		if ( selectionLevel == SELECTION_LEVEL_ENVS )

			Move(x,y,z);

		else if ( selectionLevel == SELECTION_LEVEL_ENVIRONMENT )

			Environment_Move(x,y,z);

		else if ( selectionLevel == SELECTION_LEVEL_OBJECT )

			taskEnvironments[activeEnvironment]->Active_Element_Move(x,y,z);

		else if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Active_Component_Move(x,y,z);
}

void ENVS::Active_Element_Next(void) {

	if ( simulateMode == MODE_SIMULATE_TAU )

		Environment_Next();

	else if ( simulateMode == MODE_SIMULATE_DESIGN ) {

		if ( selectionLevel == SELECTION_LEVEL_ENVIRONMENT )

			Environment_Next();

		else if ( selectionLevel == SELECTION_LEVEL_OBJECT )

			taskEnvironments[activeEnvironment]->Active_Element_Next();

		else if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Active_Component_Next();
	}
}

void ENVS::Active_Element_Previous(void) {

	if ( simulateMode == MODE_SIMULATE_TAU )

		Environment_Previous();

	else if ( simulateMode == MODE_SIMULATE_DESIGN ) {

		if ( selectionLevel == SELECTION_LEVEL_ENVIRONMENT )

			Environment_Previous();

		else if ( selectionLevel == SELECTION_LEVEL_OBJECT )

			taskEnvironments[activeEnvironment]->Active_Element_Previous();

		else if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Active_Component_Previous();
	}
}

void ENVS::Active_Element_Resize(double changeX, double changeY, double changeZ) {

	if ( simulateMode == MODE_SIMULATE_DESIGN )

		if ( selectionLevel == SELECTION_LEVEL_OBJECT )

			taskEnvironments[activeEnvironment]->Active_Element_Resize(changeX,changeY,changeZ);

		else if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Active_Component_Resize(changeX,changeY,changeZ);
}

void ENVS::Active_Element_Rotate(double rotX, double rotY, double rotZ) {

	if ( simulateMode == MODE_SIMULATE_DESIGN )

		if ( selectionLevel == SELECTION_LEVEL_OBJECT )

			taskEnvironments[activeEnvironment]->Active_Element_Rotate(rotX,rotY,rotZ);

		else if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Active_Component_Rotate(rotX,rotY,rotZ);
}

void ENVS::Active_Element_Save(void) {

	     if ( selectionLevel == SELECTION_LEVEL_ENVIRONMENT )

		     taskEnvironments[activeEnvironment]->Save();

	     else if ( selectionLevel == SELECTION_LEVEL_ENVS )
		     Save(true);
}

void ENVS::Joint_Connect(void) {

	if ( simulateMode == MODE_SIMULATE_DESIGN )

		if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Connect_Robot_Joint();
}

void ENVS::Joint_Range_Decrease(void) {

	if ( simulateMode == MODE_SIMULATE_DESIGN )

//		if ( selectionLevel == SELECTION_LEVEL_OBJECT )
//
//			taskEnvironments[activeEnvironment]->Active_Object_Joint_Range_Decrease();
//
//		else if ( selectionLevel == SELECTION_LEVEL_ROBOT )

		if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Active_Joint_Range_Decrease();
}

void ENVS::Joint_Range_Increase(void) {

	if ( simulateMode == MODE_SIMULATE_DESIGN )

//		if ( selectionLevel == SELECTION_LEVEL_OBJECT )
//
//			taskEnvironments[activeEnvironment]->Active_Object_Joint_Range_Increase();
//
//		else if ( selectionLevel == SELECTION_LEVEL_ROBOT )

		if ( selectionLevel == SELECTION_LEVEL_ROBOT )

			taskEnvironments[activeEnvironment]->Active_Joint_Range_Increase();
}

void ENVS::Draw(void) {

	for (int i=0;	i<currNumberOfEnvs;	i++) {

		if ( taskEnvironments[i] )
			taskEnvironments[i]->Draw();
	}
}

void ENVS::EvaluationPeriod_Decrease(dWorldID world, dSpaceID space, dJointGroupID contactgroup) {

/*
	if ( In_Design_Mode() )
		return;

	Destroy_Simulated_Objects();

	optimizer->Reset_But_Keep_Best();
		
	if ( optimizer )

		optimizer->EvaluationPeriod_Decrease();
*/
}

void ENVS::EvaluationPeriod_Increase(dWorldID world, dSpaceID space, dJointGroupID contactgroup) {

/*
	if ( In_Design_Mode() )
		return;

	Destroy_Simulated_Objects();

	optimizer->Reset_But_Keep_Best();

	if ( optimizer )

		optimizer->EvaluationPeriod_Increase();
*/
}

void ENVS::Evolve(      dWorldID      world,
                        dSpaceID      space) {

        // If there is a robot being evaluated, and its time
        // is not yet up, allow it to move.

        if ( !Eval_Finished() )

		Evolve_Allow_Robots_To_Move(world,space);

        // If the time limit for the robot being evaluated has
        // expired, record its fitness, remove it from the simulator,
        // and prepare for the evaluation of the next robot.

        else
		Evolve_Times_Up(world,space);
}

int ENVS::In_Champ_Mode(void) {

	return( simulateMode == MODE_SIMULATE_CHAMP );
}

int ENVS::In_Design_Mode(void) {

	return( simulateMode == MODE_SIMULATE_DESIGN );
}

int ENVS::In_Evolution_Mode(void) {

	return( simulateMode == MODE_SIMULATE_EVOLVE );
}

int ENVS::In_TAU_Mode(void) {

	return( simulateMode == MODE_SIMULATE_TAU );
}

void ENVS::Mode_Simulate_Set_Champ(dWorldID world, dSpaceID space) {

	// Switch to visualizing the best robot so far.

	if ( !Ready_For_Champ_Mode() )

		return;

	End_Current_Mode();

	simulateMode = MODE_SIMULATE_CHAMP;

	// The best robot is always green.
	for (int i=0;	i<numberOfEnvs;	i++) {

		taskEnvironments[i]->Set_Color(0,1,0);

		taskEnvironments[activeEnvironment]->Robots_Recolor('g');
	}

	optimizer->Timer_Reset();

	Create_Robot_Current_Best(world,space);
}

void ENVS::Mode_Simulate_Set_Design(void) {

	// Switch to the designing of task environments.

	if ( !Ready_For_Design_Mode() )
		return;

	End_Current_Mode();

	simulateMode=MODE_SIMULATE_DESIGN;

	currNumberOfEnvs = numberOfEnvs;

	// Robots during design are always red.
	for (int i=0;	i<numberOfEnvs;	i++) {

		taskEnvironments[i]->Set_Color(1,0,0);

		taskEnvironments[activeEnvironment]->Robots_Recolor('r');
	}
}

void ENVS::Mode_Simulate_Set_Evolve(dWorldID world, dSpaceID space) {

	// Switch to evolution mode.

	if ( !Ready_For_Evolution_Mode() )

		return;

	End_Current_Mode();

	// Selection level is too low. 
	if ( selectionLevel == SELECTION_LEVEL_ROBOT )	// lowest level
		Selection_Level_Raise();

	if ( selectionLevel == SELECTION_LEVEL_OBJECT )
		Selection_Level_Raise();

	// Just finished defining the task environment.
	if ( In_Design_Mode() ) {

		// if the number of sensors or motors has changed,
		// delete the optimizer
		if ( Optimizer_Robot_Mismatch() ) {

			delete optimizer;
			optimizer = NULL;
		}

		Optimizer_Initialize();
	}

	simulateMode=MODE_SIMULATE_EVOLVE;

	// The evolving robots are always blue.
	for (int i=0;i<numberOfEnvs;i++) {

		taskEnvironments[i]->Set_Color(0,0,1);

		taskEnvironments[activeEnvironment]->Robots_Recolor('b');
	}

	// Set up next robot...
	Create_Robot_To_Evaluate(world,space);
	evaluationsSinceLastSave++;
        if ( evaluationsSinceLastSave == AFPO_POP_SIZE ) {
        	selectionLevel = SELECTION_LEVEL_ENVS;
        	Save(false);
        	evaluationsSinceLastSave = 0;
        }
}

void ENVS::Mode_Simulate_Set_TAU(dWorldID world, dSpaceID space) {

        // Switch to The Approximate User (TAU) mode. 

//	if ( !Ready_For_TAU_Mode() )
//		return;

	End_Current_Mode();

        simulateMode=MODE_SIMULATE_TAU;

	// In TAU mode, there are always two environments;
	// the user must indicate which of the two robots
	// she likes better.

	Environment_Copy();

        // The evolving robots are always yellow in TAU mode. 

        for (int i=0;i<numberOfEnvs;i++)

                taskEnvironments[i]->Set_Color(1,1,0);

	// To start, highlight the first of the two controllers.
	selectionLevel = SELECTION_LEVEL_ENVIRONMENT;
	taskEnvironments[0]->Activate_All();
	taskEnvironments[1]->Deactivate_All();
	activeEnvironment = 0;

	// ...and create the two robots to compare.
	TAU_Send_Controllers_For_Evaluation(world,space);
}

void ENVS::Mode_View_Set_Back(void) {

	viewMode = MODE_VIEW_BACK;

	Viewpoint_Set(0,-2.792,0.79,90,-10.5,0);
}

void ENVS::Mode_View_Set_Side(void) {

	viewMode = MODE_VIEW_SIDE;

	Viewpoint_Set(-7.274,1.658,2.16,0,-6,0);
}

void ENVS::Mode_View_Set_Top(void) {

	viewMode = MODE_VIEW_TOP;

	Viewpoint_Set(0,2,7,90,-90,0);
}

void ENVS::MutationProbability_Decrease(void) {

	if ( optimizer )

		optimizer->MutationProbability_Decrease();
}

void ENVS::MutationProbability_Increase(void) {

	if ( optimizer )

		optimizer->MutationProbability_Increase();
}

void ENVS::Num_Of_Sensors_Or_Motors_Changed(void) {

	if ( tau ) {

		delete tau;
		tau = NULL;
	}

	if ( optimizer ) {

		delete optimizer;
		optimizer = NULL;
	}
}

int  ENVS::Pair_Available(void) {

        char fileName[100];
        int fileIndex=0;
        sprintf(fileName,"SavedFiles/pair%d.dat",fileIndex++);
        while ( !File_Exists(fileName) && (fileIndex<10) ) 
                sprintf(fileName,"SavedFiles/pair%d.dat",fileIndex++);

	return( File_Exists(fileName) );
}

void ENVS::Prepare_To_Run(dWorldID world, dSpaceID space) {
	
	selectionLevel=SELECTION_LEVEL_ENVS;

	Load(false);
//	Mode_Simulate_Set_Evolve(world,space);
	Reset();
	Mode_Simulate_Set_Evolve(world,space);
	Deactivate_All();
	Activate_All();
	selectionLevel=SELECTION_LEVEL_ENVS;
}

void ENVS::Rescore_Population(void) {

	for (int i=0; i<AFPO_POP_SIZE; i++) {

		if (	optimizer	&&
			optimizer->genomes[i] &&
			optimizer->genomes[i]->sensorTimeSeries )

			optimizer->genomes[i]->Score_Set( tau->Score_Predict(optimizer->genomes[i]) );
	}
}

void ENVS::Request_Evolving_Genomes(void) {

	char fileName[100];
	sprintf(fileName,"SavedFiles/userRequest.txt");

	ofstream *outFile = new ofstream(fileName);

	(*outFile) << 'v' << '\n';

	outFile->close();
	delete outFile;
	outFile = NULL;
}

void ENVS::Request_Pair(void) {

        char fileName[100];
        sprintf(fileName,"SavedFiles/userRequest.txt");

        ofstream *outFile = new ofstream(fileName);

        (*outFile) << 'u' << '\n';

        outFile->close();
        delete outFile;
        outFile = NULL;
}

void ENVS::Reset(void) {

	if ( In_Evolution_Mode() ) {
		if ( optimizer )
			optimizer->Reset();
		if ( tau ) {
			delete tau;
			tau = NULL;
		}	
	}

	else if ( In_TAU_Mode() ) {
		if ( tau ) {
			delete tau;
			tau = NULL;
			optimizer->Reset_Genomes();
		}
	}

	Mode_Simulate_Set_Design();
}

void ENVS::Save(int showGraphics) {

        char fileName[100];
        sprintf(fileName,"SavedFiles/envs0.dat");

        ofstream *outFile = new ofstream(fileName);

        Camera_Position_Save(outFile,showGraphics);

        Save_Environments(outFile);

        Save_Optimizer(outFile,true);

        Save_TAU(outFile);

        outFile->close();
        delete outFile;
        outFile = NULL;
}

void ENVS::Selection_Level_Lower(void) {

	// Already at lowest level...
	if ( selectionLevel==SELECTION_LEVEL_ROBOT )
		return;

	if ( selectionLevel==SELECTION_LEVEL_OBJECT ) {

		taskEnvironments[activeEnvironment]->Deactivate_All();

		taskEnvironments[activeEnvironment]->Unmark_All();

		taskEnvironments[activeEnvironment]->Unhide_All();

		taskEnvironments[activeEnvironment]->Activate_Component(0);

		selectionLevel = SELECTION_LEVEL_ROBOT;
	}

	else if ( selectionLevel==SELECTION_LEVEL_ENVIRONMENT ) {

		taskEnvironments[activeEnvironment]->Deactivate_All();

		taskEnvironments[activeEnvironment]->Unmark_All();

		taskEnvironments[activeEnvironment]->Activate_Light_Source();

		selectionLevel = SELECTION_LEVEL_OBJECT;
	}

	else {  // selection level is envs
		
		Deactivate_All();

		if ( activeEnvironment<0 )
			activeEnvironment=0;

		taskEnvironments[activeEnvironment]->Activate_All();

		selectionLevel = SELECTION_LEVEL_ENVIRONMENT;
	}
}

void ENVS::Selection_Level_Raise(void) {

        // Only allow selection level change in design mode.
        if ( !In_Design_Mode() )
                return;

	// Already at highest level...
	if ( selectionLevel==SELECTION_LEVEL_ENVS )
		return;

	else if ( selectionLevel==SELECTION_LEVEL_ENVIRONMENT ) {

		taskEnvironments[activeEnvironment]->Deactivate_All();

		taskEnvironments[activeEnvironment]->Unmark_All();

		Activate_All();

		selectionLevel=SELECTION_LEVEL_ENVS;
	}

	else if ( selectionLevel==SELECTION_LEVEL_OBJECT ) {

		taskEnvironments[activeEnvironment]->Unmark_All();

		taskEnvironments[activeEnvironment]->Activate_All();

		selectionLevel=SELECTION_LEVEL_ENVIRONMENT;

		taskEnvironments[activeEnvironment]->Robots_Recolor('r');
	}

	else { // selectionLevel==SELECTION_LEVEL_ROBOT

		taskEnvironments[activeEnvironment]->Unmark_All();

		taskEnvironments[activeEnvironment]->Hide_Robot_Joints();

		taskEnvironments[activeEnvironment]->Activate_Robot(0);

		selectionLevel=SELECTION_LEVEL_OBJECT;

		taskEnvironments[activeEnvironment]->Robots_Recolor('r');
	}
}

void ENVS::Show_Champ(	dWorldID      world, 
			dSpaceID      space) {

	// If there is a champ being evaluated, and its time
	// is not yet up, allow it to move.

	if ( !Eval_Finished() ) {

		for (int i=0;	i<currNumberOfEnvs;	i++) {

			taskEnvironments[i]->Allow_Robot_To_Move(optimizer->timer);
		}

		optimizer->Timer_Update();
	}

	// If the time limit for the champ has
	// expired, replay it.
	else {

		optimizer->Print();

		Destroy_Simulated_Objects();

		optimizer->Timer_Reset();

		Create_Robot_Current_Best(world,space);
	}
}

void ENVS::Speed_Decrease(void) {

	if ( speed > 1 )
		speed--;
}

void ENVS::Speed_Increase(void) {

	if ( speed < 1000 )
		speed++;
}

NEURAL_NETWORK *ENVS::TAU_Get_User_Favorite(void) {

	if ( !tau )

		return( NULL );

	return( tau->Controller_Get_Best() );
}

int ENVS::TAU_Ready_To_Predict(void) {

	if ( !tau )

		return( false );

	return( tau->Ready_To_Predict() );
}

double ENVS::TAU_Score_Get(void) {

	// Get the current controller that was just evaluated,

	NEURAL_NETWORK *currentController = taskEnvironments[0]->robots[0]->neuralNetwork;

	// and return the predicted score of that controller.

	return( tau->Score_Predict(currentController) );
}

void ENVS::TAU_Show_Robot_Pair( dWorldID world, dSpaceID space) {

	if ( tau->timer<STARTING_EVALUATION_TIME ) {

		taskEnvironments[0]->Allow_Robot_To_Move(tau->timer);
		taskEnvironments[1]->Allow_Robot_To_Move(tau->timer);
	 	tau->timer++;	
	}

	// If the time limit for the robot pair has expired, replay it.
	else {

		Destroy_Simulated_Objects();

		tau->timer=0;

		TAU_Send_Controllers_For_Evaluation(world,space);
	}
}

void ENVS::TAU_User_Has_Indicated_A_Preference(dWorldID world, dSpaceID space) {

	// Only accept a user's preference if in TAU mode.

	if ( In_TAU_Mode() ) {

		TAU_Store_User_Preference();

		delete tau;
		tau = NULL;

		Destroy_Simulated_Objects();

		User_Wants_To_Provide_Preferences(world,space);
	}

}

void ENVS::User_Wants_To_Provide_Preferences(dWorldID world, dSpaceID space) {

        char fileName[100];
        sprintf(fileName,"SavedFiles/programResponse.txt");
	
	File_Delete(fileName);

	Destroy();
	Initialize();

	Request_Pair();

        while ( !File_Exists(fileName) );

	Load_Pair();

	File_Delete(fileName);
	
	Mode_Simulate_Set_TAU(world,space);
}

void ENVS::User_Wants_To_Watch_Evolution(dWorldID world, dSpaceID space) {

	char fileName[100];
	sprintf(fileName,"SavedFiles/programResponse.txt");

	File_Delete(fileName);

	Destroy();
	Initialize();

	Request_Evolving_Genomes();

	while ( !File_Exists(fileName) );

	ifstream *inFile = new ifstream(fileName);

	Load_Environments(inFile);
	
	Load_Optimizer(inFile);

	Load_TAU(inFile);

	inFile->close();
	delete inFile;
	inFile = NULL;

	File_Delete(fileName);

        selectionLevel=SELECTION_LEVEL_ENVS;

        Mode_Simulate_Set_Evolve(world,space);
}

void ENVS::Unhide_All(void) {

	if ( !taskEnvironments )

		return;

	for (int i=0;i<numberOfEnvs;i++)

		if ( taskEnvironments[i] )

			taskEnvironments[i]->Unhide_All();
}

void ENVS::Video_Record(void) {

/*
	if ( timeStepsSinceLastFrameCapture < TIME_STEPS_PER_FRAME ) {
		timeStepsSinceLastFrameCapture++;
		return;
	}

	printf("Recording frame %d in Movie%d.\n",frameIndex,movieIndex);

	int width = 352*2;
	int height = 288*2;

	char s[100];

	if ( frameIndex<10 )
		sprintf (s,"Movie%d/0000%d.ppm",movieIndex,frameIndex);
	else if ( frameIndex<100 )
		sprintf (s,"Movie%d/000%d.ppm",movieIndex,frameIndex);
	else if ( frameIndex<1000 )
		sprintf (s,"Movie%d/00%d.ppm",movieIndex,frameIndex);
	else if ( frameIndex<10000 )
		sprintf (s,"Movie%d/0%d.ppm",movieIndex,frameIndex);
	else if ( frameIndex<100000 )
		sprintf (s,"Movie%d/%d.ppm",movieIndex,frameIndex);

	FILE *f = fopen (s,"wb");
	fprintf (f,"P6\n%d %d\n255\n",width,height);

	void *buf = malloc( width * height * 3 );
	glReadPixels( 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, buf );

	for (int y=(height - 1); y>=0; y--) {
		for (int x=0; x<width; x++) {
			unsigned char *pixel = ((unsigned char *)buf)+((y*width+ x)*3);
			unsigned char b[3];
			b[0] = *pixel;
			b[1] = *(pixel+1);
			b[2] = *(pixel+2);
			fwrite(b,3,1,f);
		}
	}
	free(buf);
	fclose(f);

	frameIndex++;
	timeStepsSinceLastFrameCapture = 0;
*/

}

void ENVS::Video_Start_Stop(void) {

	if ( recordingVideo )

		Video_Stop();
	else
		Video_Start();
}

void ENVS::Viewpoint_Get(void) {

	dsGetViewpoint(xyz,hpr);

	printf("%3.3f %3.3f %3.3f %3.3f %3.3f %3.3f\n",
		xyz[0],xyz[1],xyz[2],hpr[0],hpr[1],hpr[2]);
}

// ---------------- Private methods -------------------

void ENVS::Activate_All(void) {

	if ( !taskEnvironments )

		return;

	for (int i=0;	i<numberOfEnvs;	i++)

		if ( taskEnvironments[i] )

			taskEnvironments[i]->Activate_All();
}

void ENVS::Camera_Position_Load(ifstream *inFile, int showGraphics) {

	(*inFile) >> xyz[0] >> xyz[1] >> xyz[2];

	(*inFile) >> hpr[0] >> hpr[1] >> hpr[2];

	if ( showGraphics )
		dsSetViewpoint(xyz,hpr);
}

void ENVS::Camera_Position_Save(ofstream *outFile, int showGraphics) {

	if ( showGraphics )
		dsGetViewpoint(xyz,hpr);

	(*outFile) << xyz[0] << " " << xyz[1] << " " << xyz[2] << "\n";

	(*outFile) << hpr[0] << " " << hpr[1] << " " << hpr[2] << "\n";
}

void ENVS::Check_For_Changed_Environment(dWorldID world, dSpaceID space) {

        char fileName[100];
        sprintf(fileName,"SavedFiles/changedEnvironment.dat");

        if ( File_Exists(fileName) ) {
		File_Delete(fileName);
		Destroy();
		Prepare_To_Run(world,space);
        }
}

void ENVS::Check_For_Pref(void) {

        char fileName[100];
        sprintf(fileName,"SavedFiles/pref.dat");

	if ( File_Exists(fileName) ) {

		Collect_Pref(fileName);

		File_Delete(fileName);

		Rescore_Population();
	}
}

void ENVS::Check_For_User_Request(void) {

	char fileName[100];
	sprintf(fileName,"SavedFiles/userRequest.txt");

	if ( File_Exists(fileName) ) {

		char userRequest = Get_User_Request(fileName);

		if ( userRequest == 'v' )

			Send_Evolving_Genomes();

		else if ( userRequest == 'u' )

			Send_Pair_For_Pref();
	}
}

void ENVS::Check_Whether_To_Writeout(void) {

        clock_t currTime = clock();

        double CPUSecondsSinceLastWriteout = ((double) (currTime - timeSinceLastWriteout)) / CLOCKS_PER_SEC;

	double CPUMinutesSinceLastWriteout = CPUSecondsSinceLastWriteout/60.0;

	// Write an update every 5 minutes.
        if ( CPUSecondsSinceLastWriteout > 5 ) {
//	if ( CPUMinutesSinceLastWriteout > 1 ) {

	        char fileName[200];
        	sprintf(fileName,"SavedFiles/writeout%d.txt",randSeed);

		ofstream *outFile = new ofstream(fileName,ios::app);

                // Write out elapsed hours...
                (*outFile) << double((clock()-startTime)/CLOCKS_PER_SEC)/(60.0*60.0) << "\t";

                // Write out elapsed minutes...
                (*outFile) << double((clock()-startTime)/CLOCKS_PER_SEC)/60.0 << "\t";

		// Write out elapsed seconds...
		(*outFile) << double((clock()-startTime)/CLOCKS_PER_SEC) << "\t";

		optimizer->Genome_Get_Best()->Writeout(outFile);

		(*outFile) << "\n";

		outFile->close();
		delete outFile;
		outFile = NULL;

		timeSinceLastWriteout = currTime;
	}
}

void ENVS::Collect_Pref(char *fileName) {

	ifstream *inFile = new ifstream(fileName);

	int firstControllerID;
	int secondControllerID;
	int pref;

	(*inFile) >> firstControllerID;
	(*inFile) >> secondControllerID;
	(*inFile) >> pref;

	inFile->close();
	delete inFile;
	inFile = NULL;

	tau->Store_Pref(firstControllerID,secondControllerID,pref);
}

void ENVS::Create_Robot_Current_Best( 	dWorldID      world, 
					dSpaceID      space) {

	currNumberOfEnvs = numberOfEnvs;

	NEURAL_NETWORK *bestGenome = optimizer->Genome_Get_Best();

	for (int i=0;	i<currNumberOfEnvs;	i++) {

		taskEnvironments[i]->Prepare_For_Simulation(world,space);

		taskEnvironments[i]->Label(bestGenome,i);
	}

	bestGenome = NULL;
}

void ENVS::Create_Robot_To_Evaluate(	dWorldID      world, 
					dSpaceID      space) {

	NEURAL_NETWORK *nextGenome = optimizer->Genome_Get_Next_To_Evaluate(TAU_Get_User_Favorite());

	for (int i=0;	i<currNumberOfEnvs;	i++) {

		taskEnvironments[i]->Prepare_For_Simulation(world,space);

		taskEnvironments[i]->Label(nextGenome,i);

		taskEnvironments[i]->Record_Sensor_Data(optimizer->evaluationPeriod);
	}

	nextGenome = NULL;
}

void ENVS::Deactivate_All(void) {

	if ( !taskEnvironments )

		return;

	for (int i=0;	i<numberOfEnvs;	i++)

		if ( taskEnvironments[i] )

			taskEnvironments[i]->Deactivate_All();
}

void ENVS::Destroy(void) {

        if ( optimizer ) {
                delete optimizer;
                optimizer = NULL;
        }

        for (int i=0;   i<MAX_ENVIRONMENTS;     i++) {

                if ( taskEnvironments[i] ) {
			taskEnvironments[i]->Destroy_Simulated_Objects();
                        delete taskEnvironments[i];
                        taskEnvironments[i] = NULL;
                }
        }
        delete[] taskEnvironments;
        taskEnvironments = NULL;

        if ( tau ) {
                delete tau;
                tau = NULL;
        }
}

void ENVS::Destroy_Simulated_Objects(void) {

	for (int i=0;i<currNumberOfEnvs;i++)

		taskEnvironments[i]->Destroy_Simulated_Objects();
}

int  ENVS::Directory_Found(char *dirName) {

	struct stat stFileInfo;
	bool blnReturn;
	int intStat;

	intStat = stat(dirName,&stFileInfo);
	if(intStat == 0) {
		blnReturn = true;
	} else {
		blnReturn = false;
	}

	return(blnReturn);
}

void ENVS::Directory_Make(char *dirName) {

        char command[500];

        sprintf(command,"mkdir %s",dirName);
        system(command);
}

void ENVS::End_Current_Mode(void) {

        // In some modes, the objects are physically simulated.
        // Switch them back to only being drawn.

        Destroy_Simulated_Objects();

        // If robot behavior was being simulated,
        // reset the timer in the optimizer.

        if ( optimizer )

                optimizer->Timer_Reset();

        // If in TAU mode, two environments were simulated,
        // so delete the second one.

        if ( In_TAU_Mode() )

                Environment_Delete();
}

int  ENVS::End_State_Missing(void) {

	int missing = false;

	int currentEnvironment = 0;

	while ( (currentEnvironment<numberOfEnvs) && 
		(!missing) ) {

		if ( taskEnvironments[currentEnvironment]->numRobots<2 )
			missing = true;
		else
			currentEnvironment++;
	}

	return( missing );
}

void ENVS::Environment_Copy(void) {

	// Maximum number of environments already exist. 
	if ( numberOfEnvs == MAX_ENVIRONMENTS )
		return;

	taskEnvironments[numberOfEnvs] = 

		new ENVIRONMENT(this,taskEnvironments[activeEnvironment]);

	taskEnvironments[activeEnvironment]->Deactivate_All();

//	taskEnvironments[numberOfEnvs]->Move(-MOVE_INCREMENT,MOVE_INCREMENT,0);
	taskEnvironments[numberOfEnvs]->Activate_All();

	activeEnvironment = numberOfEnvs;

	numberOfEnvs++;
	currNumberOfEnvs++;
}

void ENVS::Environment_Delete(void) {

	// Cannot remove only remaining environment.
	if ( numberOfEnvs == 1 )
		return;

	delete taskEnvironments[numberOfEnvs-1];
	taskEnvironments[numberOfEnvs-1] = NULL;

	numberOfEnvs--;
	currNumberOfEnvs--;

	activeEnvironment = numberOfEnvs-1;

	taskEnvironments[numberOfEnvs-1]->Activate_All();
}

void ENVS::Environment_Move(double x, double y, double z) {

	taskEnvironments[activeEnvironment]->Move(x,y,z);
}

void ENVS::Environment_Next(void) {

	taskEnvironments[activeEnvironment]->Deactivate_All();

	activeEnvironment++;

	if ( activeEnvironment==numberOfEnvs )
		activeEnvironment = 0;

	taskEnvironments[activeEnvironment]->Activate_All();
}

void ENVS::Environment_Previous(void) {

	taskEnvironments[activeEnvironment]->Deactivate_All();

	activeEnvironment--;

	if ( activeEnvironment==-1 )
		activeEnvironment = numberOfEnvs-1;

	taskEnvironments[activeEnvironment]->Activate_All();

}

int  ENVS::Eval_Finished(void) {

	return( optimizer->Time_Elapsed() );
}

void ENVS::Evolve_Allow_Robots_To_Move( dWorldID world,
                                        dSpaceID space) {

                for (int i=0;   i<currNumberOfEnvs;     i++)

                        taskEnvironments[i]->Allow_Robot_To_Move(optimizer->timer);

                optimizer->Timer_Update();
}

void ENVS::Evolve_Times_Up(dWorldID world, dSpaceID space) {

        //double fitness = taskEnvironments[0]->robots[0]->Fitness_Get(taskEnvironments[0]->robots[1]);
	double fitness = Fitness_Get();

	// TODO: At the moment, we only record the sensor data from the first (of possibly several)
	// environments for user modeling. It is not yet clear to me how a user provides
	// preferences between two robots, each of which acts in multiple environments.

        MATRIX *timeSeries = taskEnvironments[0]->Get_Sensor_Data();

        if ( TAU_Ready_To_Predict() )

                optimizer->Fitness_Sensor_Data_Score_Receive(TAU_Get_User_Favorite(),fitness,timeSeries,TAU_Score_Get());
        else
                optimizer->Fitness_Sensor_Data_Receive(TAU_Get_User_Favorite(),fitness,timeSeries);

        timeSeries = NULL;

        Destroy_Simulated_Objects();

        optimizer->Timer_Reset();

        // Set up next robot...
        Create_Robot_To_Evaluate(world,space);

        evaluationsSinceLastSave++;

        if ( evaluationsSinceLastSave == AFPO_POP_SIZE ) {

                selectionLevel = SELECTION_LEVEL_ENVS;
		Check_For_Changed_Environment(world,space);
		Check_For_User_Request();
                Check_For_Pref();
                evaluationsSinceLastSave = 0;
        }
}

void  ENVS::File_Delete(char *fileName) {

	if ( !File_Exists(fileName) )
		return;

        char command[500];

        sprintf(command,"rm %s",fileName);
        system(command);
}

bool  ENVS::File_Exists(char *fileName) {

	ifstream ifile(fileName);
	return ifile;
}

int  ENVS::File_Index_Next_Available(void) {

	int fileIndex = 0;
	char fileName[100];
	sprintf(fileName,"SavedFiles/envs%d.dat",fileIndex);
	while ( File_Exists(fileName) ) {
		fileIndex++;
		sprintf(fileName,"SavedFiles/envs%d.dat",fileIndex);
	}

	return( fileIndex );
}

void  ENVS::File_Rename(char *fileName, char *newFileName) {

        char command[500];

        sprintf(command,"mv %s %s",fileName,newFileName);
        system(command);
}

double ENVS::Fitness_Get(void) {

	// Take average fitness.
	double fitness = taskEnvironments[0]->Fitness_Get();
	for (int i=1;	i<currNumberOfEnvs;	i++)
		fitness = fitness + taskEnvironments[i]->Fitness_Get();
	fitness = fitness / double(currNumberOfEnvs);

/*
	// Multiply fitnesses.
	double fitness = fabs(taskEnvironments[0]->Fitness_Get());
	for (int i=1;	i<numberOfEnvs;	i++)
		fitness = fitness * fabs(taskEnvironments[i]->Fitness_Get());
	fitness = -fitness;
*/

/*
	// Take worst fitness.
	double fitness = +1000.0;
	for (int i=0;	i<numberOfEnvs;	i++) {
		double temp = taskEnvironments[i]->Fitness_Get();
		if ( temp < fitness )
			fitness = temp;
	}
*/

	return( fitness );
}

char ENVS::Get_User_Request(char *fileName) {

        ifstream *inFile = new ifstream(fileName);
        char userRequest;
        (*inFile) >> userRequest;
        inFile->close();
        delete inFile;
        inFile = NULL;

        File_Delete(fileName);

        return( userRequest );
}

void ENVS::Hide_Robot_Joints(void) {

	if ( !taskEnvironments )

		return;

	for (int i=0; i<numberOfEnvs; i++)

		if ( taskEnvironments[i] )

			taskEnvironments[i]->Hide_Robot_Joints();
}

void ENVS::Initialize(void) {

        simulateMode=MODE_SIMULATE_DESIGN;

        numberOfEnvs = 1;
        currNumberOfEnvs = 1;

        taskEnvironments = new ENVIRONMENT * [MAX_ENVIRONMENTS];

        for (int i=0;   i<MAX_ENVIRONMENTS;     i++)
                taskEnvironments[i] = NULL;

        taskEnvironments[0] = new ENVIRONMENT(this);

        taskEnvironments[0]->Add_Light_Source();

        taskEnvironments[0]->Add_Robot_Sandbox();

        taskEnvironments[0]->Set_Color(1,0,0);

        optimizer = NULL;

        tau = NULL;

        targetSensorValuesRecorded = false;

        recordingVideo = false;

        savedFileIndex = -1;

//        selectionLevel = SELECTION_LEVEL_OBJECT;
	selectionLevel = SELECTION_LEVEL_ENVS;

        activeEnvironment = 0;

        evaluationsSinceLastSave = 0;
        evaluationsSinceLastPreference = 0;

        startTime = clock();

        timeSinceLastWriteout = startTime;
}

void ENVS::Load(int showGraphics) {

	char fileName[100];
	sprintf(fileName,"SavedFiles/envs0.dat");
	printf("Loading %s\n",fileName);

	ifstream *inFile = new ifstream(fileName);

	if ( !inFile->is_open() ) {

		printf("Can't open envs file %d to load.\n",savedFileIndex);
	}
	else {

		Camera_Position_Load(inFile,showGraphics);
		Load_Environments(inFile);
		Load_Optimizer(inFile);
		Load_TAU(inFile);
		printf("envs loaded.\n");
	}

	inFile->close();
	delete inFile;
	inFile = NULL;
}

void ENVS::Load_Pair(void) {

        char fileName[100];
        sprintf(fileName,"SavedFiles/programResponse.txt");

	ifstream *inFile = new ifstream(fileName);

	Camera_Position_Load(inFile,true);

	Load_Environments(inFile);

	TAU_Load_Controller_Pair(inFile);

	inFile->close();
	delete inFile;
	inFile = NULL;
}

void ENVS::Load_Environments(ifstream *inFile) {

	if ( taskEnvironments ) {

		for (int i=0;i<numberOfEnvs;i++) {

			if ( taskEnvironments[i] ) {
		
				delete taskEnvironments[i];
				taskEnvironments[i] = NULL;
			}
		}
	}

	(*inFile) >> numberOfEnvs;

	if ( !taskEnvironments ) {

        	taskEnvironments = new ENVIRONMENT * [MAX_ENVIRONMENTS];

        	for (int i=0;   i<MAX_ENVIRONMENTS;     i++)
                	taskEnvironments[i] = NULL;
	}

	for (int i=0;	i<numberOfEnvs;	i++)

		taskEnvironments[i] = new ENVIRONMENT(this,inFile);
}

void ENVS::Load_Optimizer(ifstream *inFile) {

	int isOptimizer;
	(*inFile) >> isOptimizer;

	if ( optimizer )
		delete optimizer;

	if ( isOptimizer )
		optimizer = new OPTIMIZER(inFile);
	else 
		optimizer = NULL;
}

void ENVS::Load_TAU(ifstream *inFile) {

        int isTAU;
        (*inFile) >> isTAU;

        if ( tau )
                delete tau;

        if ( isTAU )
                tau = new TAU(inFile);
        else
                tau = NULL;
}

void ENVS::Move(double x, double y, double z) {

	for (int i=0;i<numberOfEnvs;i++)

		if ( taskEnvironments[i] )

			taskEnvironments[i]->Move(x,y,z);
}

void ENVS::Optimizer_Initialize(void) {

	if ( !optimizer ) {

		int numberOfSensors = 
		taskEnvironments[0]->robots[0]->Sensors_Number_Of();

		int numberOfMotors = 
		taskEnvironments[0]->robots[0]->Motors_Number_Of();
 
		// The genomes encode a synaptic weight from each
		// sensor to each hidden neuron, and from each
		// hidden neuron to each motor
		// of size = (inputs + outputs ) x hidden neurons
		optimizer = new OPTIMIZER(	numberOfSensors,
						numberOfMotors);
	}
}

int ENVS::Optimizer_Robot_Mismatch(void) {

	if ( !optimizer ) 
		return (1);
	
	int sensorMismatch =	optimizer->numSensors !=
                		taskEnvironments[0]->robots[0]->Sensors_Number_Of();

	int motorMismatch =	optimizer->numMotors !=
				taskEnvironments[0]->robots[0]->Motors_Number_Of();

	return ( sensorMismatch || motorMismatch );
}

int  ENVS::Ready_For_Champ_Mode(void) {

        // Already in champ mode.

        if ( In_Champ_Mode() )

                return( false );


	// Can't play back the best controller
	// if there aren't any.

	if ( !optimizer )

		return( false );

	if ( optimizer->Genome_Get_Best() == NULL )

		return( false );

        return( true );
}

int  ENVS::Ready_For_Design_Mode(void) {

        // Already in design mode.
        if ( In_Design_Mode() )
                
                return( false );

	return( true );
}

int  ENVS::Ready_For_Evolution_Mode(void) {

        // Already in evolve mode.
        if ( In_Evolution_Mode() )
                return( false );

        // No task for the robot to perform has yet been set.
        if ( End_State_Missing() ) {
                printf("No target state for the robot has been defined.\n");
                return( false );
        }

        // A robot isn't evolvable due to a construction problem
        for (int i=0; i<numberOfEnvs; i++) {
                if ( taskEnvironments[i]->Robot_Joint_Is_Unattached() ) {
                        printf("At least one robot joint is unconnected.\n");
                        while ( selectionLevel != SELECTION_LEVEL_ROBOT )
                                Selection_Level_Lower();
                        Viewpoint_Set(0,2.5,20,90,-90,0);
                        return( false );
                }
        }

        return( true );
}

int  ENVS::Ready_For_TAU_Mode(void) {

	// Already in TAU mode.
	if ( In_TAU_Mode() )

		return( false );

	// Only allow TAU if there is one task environment.
	if ( numberOfEnvs != 1 )

		return( false );

	// Only allow TAU if there are at least two controllers
	// available for the user to choose between.

	if ( !optimizer )

		return( false );

	if ( optimizer->Genomes_Num_Of_Evaluated() < 2 )

		return( false ); 

	return( true );
}

void ENVS::Reset(dWorldID world, dSpaceID space, dJointGroupID contactgroup) {

	Destroy_Simulated_Objects();

	dJointGroupEmpty(contactgroup);

	optimizer->Genome_Discard_Being_Evaluated();

	Create_Robot_To_Evaluate(world, space);
}

int  ENVS::Robot_Being_Evaluated(void) {

	int allAreBeingEvaluated = true;

	int currentEnvironment = 0;

	while (	(allAreBeingEvaluated) &&
		(currentEnvironment<currNumberOfEnvs) )

		if ( taskEnvironments[currentEnvironment]->robots[0]->In_Simulator() )

			currentEnvironment++;
		else
			allAreBeingEvaluated = false;

	return( allAreBeingEvaluated );
}

void ENVS::Save_Environments(ofstream *outFile) {

	(*outFile) << numberOfEnvs << "\n";

	for (int i=0;	i<numberOfEnvs;	i++)

		if ( taskEnvironments[i] )

			taskEnvironments[i]->Save(outFile);
}

void ENVS::Save_Fitness(string str) {

	char fileName[100];
	sprintf(fileName,"SavedFiles/fitness.dat");

	ofstream fitFile;
	fitFile.open (fileName, ios::app); 

	if ( str == "" )  {
		char fit[127];
		sprintf(fit,"%4.5f",Fitness_Get());
		fitFile << fit << "\n";
	} else {
		fitFile << str;
	}

	fitFile.close();

//	printf("Fitness saved.\n");
}

void ENVS::Save_Optimizer(ofstream *outFile, int saveTimeSeries) {

	if ( optimizer ) {
		(*outFile) << "1\n";
		optimizer->Save(outFile,saveTimeSeries);
	}
	else
		(*outFile) << "0\n";
}

void ENVS::Save_TAU(ofstream *outFile) {

        if ( tau ) {
                (*outFile) << "1\n";
                tau->Save(outFile);
        }
        else
                (*outFile) << "0\n";
}

void ENVS::SavedFile_FindNext(void) {

	savedFileIndex++;

	char fileName[100];
	sprintf(fileName,"SavedFiles/envs%d.dat",savedFileIndex);

	if ( !File_Exists(fileName) )

		savedFileIndex = 0;
}

void ENVS::Send_Evolving_Genomes(void) {

	char fileName[100];
	sprintf(fileName,"SavedFiles/tmp.txt");

        char newFileName[100];
        sprintf(newFileName,"SavedFiles/programResponse.txt");

        if ( File_Exists(newFileName) )
                File_Delete(newFileName);

	ofstream *outFile = new ofstream(fileName);

	Save_Environments(outFile);

	Save_Optimizer(outFile,false);

	Save_TAU(outFile);

	outFile->close();
	delete outFile;
	outFile = NULL;

	File_Rename(fileName,newFileName);
}

void ENVS::Send_Pair_For_Pref(void) {

        char fileName2[100];
        sprintf(fileName2,"SavedFiles/programResponse.txt");
        if ( File_Exists(fileName2) )
                return;

        char fileName[100];
        sprintf(fileName,"SavedFiles/tmp.dat");

        ofstream *outFile = new ofstream(fileName);

        Camera_Position_Save(outFile,false);

        Save_Environments(outFile);

        TAU_Save_Controller_Pair(outFile);

        outFile->close();
        delete outFile;
        outFile = NULL;

        char command[100];
        sprintf(command,"mv %s %s",fileName,fileName2);
        system(command);
}

void ENVS::Sensor_Data_Receive(void) {

	MATRIX *timeSeries = taskEnvironments[0]->Get_Sensor_Data();

	if ( optimizer )

		optimizer->Sensor_Data_Receive(timeSeries);
}

double ENVS::Sensor_Sum(void) {

	double sum = 0.0;

	for (int i=0;	i<numberOfEnvs;	i++)

		sum = sum + taskEnvironments[i]->Sensor_Sum();

	return( sum );
}

int  ENVS::Target_Sensor_Values_Recorded(void) {

	return( targetSensorValuesRecorded );
}

void ENVS::TAU_Get_Controllers_From_Optimizer(void) {

        if ( !tau )
                tau = new TAU;

	tau->Controllers_Select_From_Optimizer(optimizer);
}

void ENVS::TAU_Load_Controller_Pair(ifstream *inFile) {

        if ( !tau )
                tau = new TAU;

        tau->Controllers_Load_Pair(inFile);
}

void ENVS::TAU_Reset_User_Models(void) {

	if ( optimizer )

		optimizer->Scores_Reset();

//	if ( tau )

//		tau->User_Models_Reset();
}

void ENVS::TAU_Save_Controller_Pair(ofstream *outFile) {

	if ( !tau )
		tau = new TAU;

	tau->Controllers_Save_Pair(optimizer,outFile);
}

void ENVS::TAU_Send_Controllers_For_Evaluation(dWorldID world, dSpaceID space) {

	taskEnvironments[0]->Prepare_For_Simulation(world,space);

	taskEnvironments[0]->Label(tau->Controller_Pair_Get_First(),0);

	taskEnvironments[0]->Record_Sensor_Data(STARTING_EVALUATION_TIME);

        taskEnvironments[1]->Prepare_For_Simulation(world,space);

        taskEnvironments[1]->Label(tau->Controller_Pair_Get_Second(),1);

	taskEnvironments[1]->Record_Sensor_Data(STARTING_EVALUATION_TIME);

	// There are contact resolution errors if the non-robot objects in both
	// environments are created. So, remove the non-robot objects from
	// the second environment.
	taskEnvironments[1]->Make_NonRobotObjects_Incorporeal();
}

void ENVS::TAU_Store_Sensor_Data(void) {

        // Store the sensor time series data generated by the
        // two controllers.

        MATRIX *timeSeries = taskEnvironments[0]->Get_Sensor_Data();

        tau->Controller_First_Store_Sensor_Data(timeSeries);

        timeSeries = NULL;

        timeSeries = taskEnvironments[1]->Get_Sensor_Data();

        tau->Controller_Second_Store_Sensor_Data(timeSeries);

        timeSeries = NULL;
}

void ENVS::TAU_Store_User_Preference(void) {

	char fileName[100];
	sprintf(fileName,"SavedFiles/tmp2.dat");

	ofstream *outFile = new ofstream(fileName);

	(*outFile) << tau->controllers[0]->ID << " ";
	(*outFile) << tau->controllers[1]->ID << " ";

	(*outFile) << activeEnvironment;

	outFile->close();
	delete outFile;
	outFile = NULL;

	char fileName2[100];
	sprintf(fileName2,"SavedFiles/pref.dat");
	
	char command[100];
	sprintf(command,"mv %s %s",fileName,fileName2);
	system(command);
}

void ENVS::Video_Start(void) {

	movieIndex = 0;

	char dirName[100];

	sprintf(dirName,"Movie%d",movieIndex);

	while ( Directory_Found(dirName) ) {
		movieIndex++;
		sprintf(dirName,"Movie%d",movieIndex);
	}

	Directory_Make(dirName);

	frameIndex=1;
	timeStepsSinceLastFrameCapture = 0;

	recordingVideo = true;
}

void ENVS::Video_Stop(void) {

	char command[500];

	char fileName[500];
	sprintf(fileName,"Movie%d.bat",movieIndex);

	ofstream *outFile = new ofstream(fileName);

	(*outFile) << "cd Movie"<<movieIndex<<" \n";

	(*outFile) << "for f in *ppm ; do nice -n 20 convert -quality 100 $f `basename $f ppm`jpg; done \n";

	(*outFile) << "rm *.ppm \n";

	(*outFile) << "mencoder 'mf://*.jpg' -mf fps=60 -o Movie"<<movieIndex<<".avi -ovc lavc \n";

// TBD: leave the series of jpegs, so different frame rates can be run
//	(*outFile) << "rm *.jpg \n";

	outFile->close();
	delete outFile;
	outFile = NULL;

	sprintf(command,"chmod 777 Movie%d.bat",movieIndex);
	system(command);

	sprintf(command,"./Movie%d.bat &",movieIndex);
	system(command);

	recordingVideo = false;
}

void ENVS::Viewpoint_Set(	double x, double y, double z,
				double h, double p, double r) {

	xyz[0] = x;
	xyz[1] = y;
	xyz[2] = z;

	hpr[0] = h;
	hpr[1] = p;
	hpr[2] = r;

	dsSetViewpoint(xyz,hpr);
}
#endif
