#include "stdlib.h"
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <sys/time.h>	// for timestamp in keystroke file

#include "envs.h"

ENVS *envs;

static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;
static dGeomID ground;

extern int	RAND_SEED;
extern double	STEP_SIZE;
extern double 	MOVE_INCREMENT;
extern int	MODE_SIMULATE_DESIGN;
extern int	MODE_SIMULATE_EVOLVE;
extern int	MODE_SIMULATE_CHAMP;
extern double 	ROT_INCREMENT;

extern int      OBJECT_FACE_FRONT;
extern int      OBJECT_FACE_BACK;
extern int      OBJECT_FACE_LEFT;
extern int      OBJECT_FACE_RIGHT;
extern int      OBJECT_FACE_TOP;
extern int      OBJECT_FACE_BOTTOM;

extern int      ALG_VARIANT_FITS_ONLY;

dsFunctions 	Simulator_Create(void);
void 		Simulator_Destroy(void);

int		showGraphics;
int		randSeed = RAND_SEED;
char		capture_fname[64] = "SavedFiles/m3cmd.scr";
int		captureScript = false;
//int		captureScript = true;	// TBD: for student experiment
char		playback_fname[64] = "SavedFiles/m3cmd.scr";
FILE*		ifp;
//int		playbackScript = true;
int		playbackScript = false;
int		playbackDone = false;
struct		timeval currTime;

int Geoms_Can_Interpenetrate(dGeomID o1, dGeomID o2);
char Get_Next_Cmd(FILE* fp);
void Set_Touch_Sensor(dGeomID o);
void Resolve_Contact(dGeomID o1, dGeomID o2);
void Simulator_Initialize(void);
void Simulator_Destroy(void);
void Simulate(int pause);
void Change_EvaluationPeriod(char cmd);
void Joint_Range_Change(char cmd);
void Change_MutationProbability(char cmd);
void Change_Selection_Level(char cmd);
int Command_To_Change_EvaluationPeriod(char cmd);
int Command_To_Change_Joint_Range(char cmd);
int Command_To_Change_MutationProbability(char cmd);
int Command_To_Change_Selection_Level(char cmd);
int Command_To_Load_Or_Save_Environment(char cmd);
int Command_To_Move_Rapidly(char cmd);
int Command_To_Move_Slowly(char cmd);
int Command_To_Resize_Object(char cmd);
int Command_To_Rotate_Rapidly(char cmd);
int Command_To_Rotate_Slowly(char cmd);
int Command_To_Switch_ActiveElement(char cmd);
int Command_To_Switch_Mode(char cmd);
int Command_To_Switch_View(char cmd);
void Load_Or_Save_Environment(char cmd);
void Move_Rapidly(char cmd);
void Move_Slowly(char cmd);
void Resize_Object(char cmd);
void Rotate_Rapidly(char cmd);
void Rotate_Slowly(char cmd);
void Switch_ActiveElement(char cmd);
void Switch_Mode(char cmd);
void Switch_View(char cmd);
dsFunctions Simulator_Create(void);
void Print_Usage(void);
void Parse_Parameters(int argc, char **argv);
FILE* Open_File(char* fname, char mode[]);
void Close_File(FILE* fp);


int Geoms_Can_Interpenetrate(dGeomID o1, dGeomID o2) {

	int object1IsPartOfRobot = false;

	if ( o1!=ground ) {
		OBJECT *obj1 = (OBJECT *)dGeomGetData(o1);
		object1IsPartOfRobot = obj1->Is_Part_Of_Robot();
	}

	int object2IsPartOfRobot = false;

	if ( o2!=ground ) {
		OBJECT *obj2 = (OBJECT *)dGeomGetData(o2);
		object2IsPartOfRobot = obj2->Is_Part_Of_Robot();
	}

	// Robot body parts can interpenetrate; don't compute 
	// contacts between them. 
	// Don't bother computing contacts between different
	// parts of the environment, because they do not move.
	// TBD: Last statement not necessarily true; could have hinged parts
	// in environment. 

	// return true if both objects are part of a robot
	// or both objects are part of the environment

	return !(object1IsPartOfRobot ^ object2IsPartOfRobot);
}

char Get_Next_Cmd(FILE* fp) {

	// assumes the file is opened for reading

	char cmd, extra; 

	int filestop;

	if ( playbackScript )

		fscanf(fp,"%c",&cmd);

	if ( cmd == EOF ) {

		playbackDone = true;

	} else {
		// cue up the next command;  TBD: clean this up
		filestop = fscanf(fp,"%c",&extra);
		while ( extra != '\n' && filestop != EOF) {
			filestop = fscanf(fp,"%c",&extra);
		}
		if ( filestop == EOF )
			playbackDone = true; 
	}

	return cmd;
}

void Set_Touch_Sensor(dGeomID o) {

	OBJECT* obj = (OBJECT *)dGeomGetData(o);

	if ( obj )

		obj->Sensor_Touch_Set();
}

void Resolve_Contact(dGeomID o1, dGeomID o2) {

	int i,n;

	const int N = 10;
//	const int N = 1;
	// N is the flags field in dCollide (collision_kernel.cpp)
	dContact contact[N];
	n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));

	// Two objects that shouldn't interpenetrate have
	// collided with one another.
	if (n > 0) {

		Set_Touch_Sensor(o1);
		Set_Touch_Sensor(o2);

		for (i=0; i<n; i++) {

		contact[i].surface.slip1 = 0.01;
		contact[i].surface.slip2 = 0.01;
		contact[i].surface.mode = dContactSoftERP |
			dContactSoftCFM |
			dContactApprox1 |
			dContactSlip1 | dContactSlip2;
		contact[i].surface.mu = 50.0;

                double kp = 100.0/STEP_SIZE;
                double kd = 0.0;
                contact[i].surface.soft_erp = 	STEP_SIZE*kp/
						(STEP_SIZE*kp+kd);
                contact[i].surface.soft_cfm = 	1.0/(STEP_SIZE*kp+kd);

		dJointID c = dJointCreateContact (world,
						contactgroup,
						&contact[i]);
		dJointAttach (c,
		dGeomGetBody(contact[i].geom.g1),
		dGeomGetBody(contact[i].geom.g2));
		}
	}
}

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{

	if ( Geoms_Can_Interpenetrate(o1,o2) )
		return;

	Resolve_Contact(o1,o2);
}

static void start()
{
	dAllocateODEDataForThread(dAllocateMaskAll);

	static float xyz[3] = {0.8317f,-0.9817f,0.8000f};
	static float hpr[3] = {121.0000f,-27.5000f,0.0000f};
	dsSetViewpoint (xyz,hpr);
//	printf ("Press:\t'1' to save the current state to 'state.dif'.\n");
	envs->Mode_View_Set_Back();
}

void Simulator_Initialize(void) {

	dInitODE2(0);
	world = dWorldCreate();
	space = dHashSpaceCreate (0);
	contactgroup = dJointGroupCreate (0);
	dWorldSetGravity (world,0,0,-0.5);
	ground = dCreatePlane (space,0,0,1,0);
}

void Simulator_Destroy(void) {

	dGeomDestroy(ground);
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);
	dCloseODE();
}

void Simulate(int pause) {

	for (int i=0;i<envs->speed;i++) {

		if ( !pause ) {

			dSpaceCollide (space,0,&nearCallback);
			dWorldStep (world,STEP_SIZE);
			dJointGroupEmpty(contactgroup);

			if ( envs->In_Evolution_Mode() )

				envs->Evolve(world,space);

			else if ( envs->In_Champ_Mode() )

				envs->Show_Champ(world,space);

			else if ( envs->In_TAU_Mode() )

				envs->TAU_Show_Robot_Pair(world,space);
		}
	}
}

static void simLoop (int pause)
{
	envs->Draw();

        if ( envs->recordingVideo )

                envs->Video_Record();

	Simulate(pause);
}

void Change_EvaluationPeriod(char cmd) {

	     if ( cmd=='t' )
		envs->EvaluationPeriod_Decrease(world,space,contactgroup);

	else if ( cmd=='T' )
		envs->EvaluationPeriod_Increase(world,space,contactgroup);
}

void Joint_Range_Change(char cmd) {

	     if (cmd=='>')
		envs->Joint_Range_Increase();

	else if (cmd=='<')
		envs->Joint_Range_Decrease();
}

void Change_MutationProbability(char cmd) {

	     if ( cmd=='m' )
		envs->MutationProbability_Decrease();

	else if ( cmd=='M' )
		envs->MutationProbability_Increase();
}

void Change_Selection_Level(char cmd) {

	// Only allow selection level change in design mode.
        if ( !envs->In_Design_Mode() )
                return;

	     if ( cmd=='-' )
		envs->Selection_Level_Raise();

	else if ( cmd=='+' )
		envs->Selection_Level_Lower();
}

int Command_To_Change_EvaluationPeriod(char cmd) {

	return(	(cmd=='t') ||
		(cmd=='T') );
}

int Command_To_Change_Joint_Range(char cmd) {

	return(	(cmd=='>') ||
		(cmd=='<') );
}

int Command_To_Change_MutationProbability(char cmd) {

	return(	(cmd=='m') ||
		(cmd=='M') );
}

int Command_To_Change_Selection_Level(char cmd) {

	return(	(cmd=='-') ||
		(cmd=='+') );
}

int Command_To_Load_Or_Save_Environment(char cmd) {

	return(	(cmd=='f') ||	// Load
		(cmd=='F') );	// Save
}

int Command_To_Move_Rapidly(char cmd) {

	return(	(cmd=='W') ||
		(cmd=='A') ||
		(cmd=='S') ||
		(cmd=='D') ||
		(cmd=='E') ||
		(cmd=='Z') );
}

int Command_To_Move_Slowly(char cmd) {

	return(	(cmd=='w') ||
		(cmd=='a') ||
		(cmd=='s') ||
		(cmd=='d') ||
		(cmd=='e') ||
		(cmd=='z') );
}

int Command_To_Resize_Object(char cmd) {

	return(	(cmd=='j') ||
		(cmd=='J') ||
		(cmd=='k') ||
		(cmd=='K') ||
		(cmd=='l') ||
		(cmd=='L') );
}

int Command_To_Rotate_Rapidly(char cmd) {

	return(	(cmd=='P') ||
		(cmd=='Q') ||
		(cmd=='R') );
}

int Command_To_Rotate_Slowly(char cmd) {

	return(	(cmd=='p') ||
		(cmd=='q') ||
		(cmd=='r') );
}

int Command_To_Switch_ActiveElement(char cmd) {

	return(	(cmd=='[') ||
		(cmd==']') );
}

int Command_To_Switch_Mode(char cmd) {

	return(	(cmd=='v') || // E[v]olve mode
		(cmd=='u') || // [u]ser preference mode
		(cmd=='g') || // Desi[g]n mode
		(cmd=='h') ); // C[h]amp mode
}

int Command_To_Switch_View(char cmd) {

	return(	(cmd=='y') ||
		(cmd=='o') ||
		(cmd=='b') );
}

void Load_Or_Save_Environment(char cmd) {

	// Load from a file.

	     if (cmd=='f')
		envs->Active_Element_Load();

	// Save to a file.

	else if (cmd=='F') {

		//envs->Active_Element_Save();

		envs->Save(true);

		char fileName[100];
		sprintf(fileName,"SavedFiles/changedEnvironment.dat");

		ofstream *outFile = new ofstream(fileName);

		outFile->close();
		delete outFile;
		outFile = NULL;
	}
}

void Move_Rapidly(char cmd) {

	     if (cmd=='W') // Move forward
		envs->Active_Element_Move(0,10*+MOVE_INCREMENT,0);

	else if (cmd=='A') // Move left
		envs->Active_Element_Move(10*-MOVE_INCREMENT,0,0);

	else if (cmd=='S') // Move backward
		envs->Active_Element_Move(0,10*-MOVE_INCREMENT,0);

	else if (cmd=='D') // Move right
		envs->Active_Element_Move(10*+MOVE_INCREMENT,0,0);

	else if (cmd=='E') // Move up
		envs->Active_Element_Move(0,0,10*+MOVE_INCREMENT);

	else if (cmd=='Z') // Move down
		envs->Active_Element_Move(0,0,10*-MOVE_INCREMENT);
}

void Move_Slowly(char cmd) {

	     if (cmd=='w') // Move forward
		envs->Active_Element_Move(0,1*+MOVE_INCREMENT,0);

	else if (cmd=='a') // Move left
		envs->Active_Element_Move(1*-MOVE_INCREMENT,0,0);

	else if (cmd=='s') // Move backward
		envs->Active_Element_Move(0,1*-MOVE_INCREMENT,0);

	else if (cmd=='d') // Move right
		envs->Active_Element_Move(1*+MOVE_INCREMENT,0,0);

	else if (cmd=='e') // Move up
		envs->Active_Element_Move(0,0,1*+MOVE_INCREMENT);

	else if (cmd=='z') // Move down
		envs->Active_Element_Move(0,0,1*-MOVE_INCREMENT);
}

void Resize_Object(char cmd) {

	     if (cmd=='J') // Enlarge the first size dimension.
		envs->Active_Element_Resize(MOVE_INCREMENT,0,0);

	else if (cmd=='j') // Shrink the first size dimension.
		envs->Active_Element_Resize(-MOVE_INCREMENT,0,0);

	else if (cmd=='K') // Enlarge the second size dimension.
		envs->Active_Element_Resize(0,MOVE_INCREMENT,0);

	else if (cmd=='k') // Shrink the second size dimension.
		envs->Active_Element_Resize(0,-MOVE_INCREMENT,0);

	else if (cmd=='L') // Enlarge the third size dimension.
		envs->Active_Element_Resize(0,0,MOVE_INCREMENT);

	else if (cmd=='l') // Shrink the third size dimension.
		envs->Active_Element_Resize(0,0,-MOVE_INCREMENT);
}

void Rotate_Rapidly(char cmd) {

	     if (cmd=='P')      // Rotate about the  X axis
		envs->Active_Element_Rotate(10*ROT_INCREMENT,0,0);

	else if (cmd=='Q') // Rotate about the Y axis
		envs->Active_Element_Rotate(0,10*ROT_INCREMENT,0);

	else if (cmd=='R') // Rotate about the Z axis
		envs->Active_Element_Rotate(0,0,10*ROT_INCREMENT);
}

void Rotate_Slowly(char cmd) {

	     if (cmd=='p')	   // Rotate about the X axis
		envs->Active_Element_Rotate(ROT_INCREMENT,0,0);

	else if (cmd=='q') // Rotate about the Y axis
		envs->Active_Element_Rotate(0,ROT_INCREMENT,0);

	else if (cmd=='r') // Rotate about the Z axis
		envs->Active_Element_Rotate(0,0,ROT_INCREMENT);
}

void Switch_ActiveElement(char cmd) {

	     if (cmd=='[')
		envs->Active_Element_Previous();

	else if (cmd==']')
		envs->Active_Element_Next();
}

void Switch_Mode(char cmd) {

	// Switch to evol[v]e mode.

	if ( cmd=='v' ) {

		if ( showGraphics )

			envs->User_Wants_To_Watch_Evolution(world,space);
		else
			envs->Mode_Simulate_Set_Evolve(world,space);
	}

	// Switch to [u]ser preference mode.

	else if ( cmd=='u' ) {

		if ( showGraphics )

			envs->User_Wants_To_Provide_Preferences(world,space);
		else
			envs->Mode_Simulate_Set_TAU(world,space);
	}

	// Switch to desi[g]n mode.
	else if ( cmd=='g' )
		envs->Mode_Simulate_Set_Design();

	// Switch to c[h]amp mode.
	else if ( cmd=='h' )
		envs->Mode_Simulate_Set_Champ(world,space);
}

void Switch_View(char cmd) {

	// Set the s[y]de view
	     if ( cmd=='y' )
		envs->Mode_View_Set_Side();

	// Set the t[o]p view
	else if ( cmd=='o' )
		envs->Mode_View_Set_Top();

	// Set the [b]ack view
	else if ( cmd=='b' )
		envs->Mode_View_Set_Back();
}

static void command (int cmd)
{

	     if ( Command_To_Switch_Mode(cmd) )
		Switch_Mode(cmd);

	else if ( Command_To_Switch_View(cmd) )
		Switch_View(cmd);

	else if ( Command_To_Move_Slowly(cmd) )
		Move_Slowly(cmd);

	else if ( Command_To_Move_Rapidly(cmd) )
		Move_Rapidly(cmd);

	else if ( Command_To_Switch_ActiveElement(cmd) )
		Switch_ActiveElement(cmd);

	else if ( cmd=='c' )
		envs->Active_Element_Copy();

	else if ( cmd=='X' )
		envs->Active_Element_Delete();

	else if ( cmd=='(' )
		envs->Active_Element_Mark();

	else if ( cmd==')' )
		envs->Active_Element_Unmark();

	else if ( cmd==' ' )
		envs->TAU_User_Has_Indicated_A_Preference(world,space);

	else if ( cmd=='#' )
		envs->Reset();

	else if ( cmd=='}' )
		envs->Speed_Increase();

	else if ( cmd=='{' )
		envs->Speed_Decrease();

	else if ( cmd=='$' )
		exit(0);

	else if ( Command_To_Change_Joint_Range(cmd) )
		Joint_Range_Change(cmd);

	else if ( cmd=='@' )
		envs->Joint_Connect();

	else if ( Command_To_Change_MutationProbability(cmd) )
		Change_MutationProbability(cmd);

	else if ( Command_To_Change_EvaluationPeriod(cmd) )
		Change_EvaluationPeriod(cmd);

	else if ( cmd=='i' ) // Start or stop recording a mov[i]e
// TBD: disable for student experiment
//		printf("Movie recording is disabled\n");
		envs->Video_Start_Stop();
	else if ( Command_To_Load_Or_Save_Environment(cmd) )
		Load_Or_Save_Environment(cmd);

	else if ( Command_To_Resize_Object(cmd) )
		Resize_Object(cmd);

	else if ( Command_To_Rotate_Rapidly(cmd) )
		Rotate_Rapidly(cmd);

	else if ( Command_To_Rotate_Slowly(cmd) )
		Rotate_Slowly(cmd);

	else if ( Command_To_Change_Selection_Level(cmd) )
		Change_Selection_Level(cmd);

	if ( captureScript ) {
		// capture input commands, with timestamp
		char fname[64] = "SavedFiles/m3cmd.scr";
		gettimeofday(&currTime, NULL);
		FILE *ofp = fopen (fname,"a");
		fprintf (ofp,"%c %ld\n",cmd,currTime.tv_sec);
		fclose(ofp);
	}
}

dsFunctions Simulator_Create(void) {

	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = (char *)"textures";

	Simulator_Initialize();

	return( fn );
}

void Print_Usage(void) {
	printf("Command line arguments\n");
	printf("   -h or --help: display this message\n");
	printf("   -rand <n>: set random seed to 'n'\n");
	printf("   -null: disable graphics\n");
	printf("Commands to change modes\n");
	printf("   g: desi(g)n mode\n");
	printf("   v: evol(v)e mode\n");
	printf("   h: c(h)amp mode\n");
	printf("Commands to switch viewpoint\n");
	printf("   y: side view\n");
	printf("   o: t(o)p view\n");
	printf("   b: (b)ack view\n");
	printf("Commands in \"Design\" mode\n");
	printf("   [: switch to previous active element\n");
	printf("   ]: switch to next active element\n");
	printf("   -: raise selection level (robot, object, or environment)\n");
	printf("   +: lower selection level (robot, object, or environment)\n");
	printf("   c: copy\n");
	printf("   X: delete\n");
	printf("   w: forward;\tW: forward fast\n");
	printf("   a: left;\tA: left fast\n");
	printf("   s: backward;\tS: backward fast\n");
	printf("   d: right;\tD: right fast\n");
	printf("   e: up;\tE: up fast\n");
	printf("   z: down;\tZ: down fast\n");
	printf("   J: increase object 1st size dimension;\tj: reduce\n");
	printf("   K: increase object 2nd size dimension;\tk: reduce\n");
	printf("   L: increase object 3rd size dimension;\tl: reduce\n");
	printf("   P: rotate 30 degrees in x;\tp: rotate 3 degrees in x\n");
	printf("   Q: rotate 30 degrees in y;\tq: rotate 3 degrees in y\n");
	printf("   R: rotate 30 degrees in z;\tr: rotate 3 degrees in z\n");
	printf("   (: mark an object;\t): unmark\n");
	printf("   @: connect active joint to marked objects\n");
	printf("   >: increase joint range for joints;\t<: reduce\n");
	printf("Commands in \"evolve\" mode\n");
	printf("   M: increase mutation probability;\tm: decrease\n");
	printf("   T: increase evaluation period;\tt: decrease\n");
	printf("Other commands\n");
	printf("   f: load environment from next saved file (design mode only)\n");
	printf("   F: save environment to a file\n");
	printf("   i: start or stop recording a mov(i)e\n");
}

void Parse_Parameters(int argc, char **argv) {

        int currParam;

        for(currParam=0;currParam<argc;currParam++) {

                if ( strcmp(argv[currParam],"-r") == 0 )
                        randSeed = atoi(argv[currParam+1]);

		if ( strcmp(argv[currParam],"-null") == 0 )
			showGraphics = false;	

		if ( strcmp(argv[currParam],"-h") == 0 ||
		     strcmp(argv[currParam],"--help") == 0 )
			Print_Usage();
	}
}


FILE* Open_File(char* fname, char mode[]) {	// TBD: fix this

	FILE *fp;

	if ((fp = fopen(fname,mode)) == NULL) {

	    printf("Can't open %s.\n",fname);

	    exit(1);
	}

	return fp;
}

void Close_File(FILE* fp) {

	fclose(fp);
}

int main (int argc, char **argv)
{

	showGraphics = true;

	Parse_Parameters(argc,argv);

	srand(randSeed);

	dsFunctions fn = Simulator_Create();

	envs = new ENVS(randSeed);

	envs->Prepare_To_Run(world,space);

	if ( showGraphics ) {
		dsSimulationLoop (argc,argv,352*2,288*2,&fn);
	}
	else {
		while ( 1 )
			Simulate(false);
	}

	delete envs;      
	envs = NULL;

  	return 0;
}
