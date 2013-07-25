#ifndef _TAU_CPP
#define _TAU_CPP

#include "stdio.h"
#include "stdlib.h"
#include "tau.h"
#include "tauOptimizer.h"

extern double TAU_NO_SCORE;
extern int    AFPO_POP_SIZE;

TAU::TAU(void) {

	numControllers = 0;

	firstControllerIndex  = -1;
	secondControllerIndex = -1;

	controllers = NULL;

	preferences = NULL;

	tauOptimizer = NULL;

	nextControllerForTraining = 0;

	scoreMin = +1000000.0;
	scoreMax = -1000000.0;

	timer = 0;
}

TAU::TAU(ifstream *inFile) {

        (*inFile) >> numControllers;

	int controllersSaved;
	(*inFile) >> controllersSaved;

	if ( controllersSaved ) {

		controllers = new NEURAL_NETWORK * [numControllers];

		for (int i=0; i<numControllers; i++)

			controllers[i] = new NEURAL_NETWORK(inFile);
	}
	else
		controllers = NULL;

	int preferencesSaved;
	(*inFile) >> preferencesSaved;

	if ( preferencesSaved )
		preferences = new MATRIX(inFile);
	else
		preferences = NULL;
        
        (*inFile) >> firstControllerIndex;
        (*inFile) >> secondControllerIndex;

	int tauOptimizerSaved;

	(*inFile) >> tauOptimizerSaved;

	if ( tauOptimizerSaved )
		tauOptimizer = new TAU_OPTIMIZER(inFile);
	else
		tauOptimizer = NULL;

        (*inFile) >> nextControllerForTraining;
        (*inFile) >> scoreMin;
        (*inFile) >> scoreMax;
}

TAU::~TAU(void) {

	if ( controllers ) {

		for (int i=0;i<numControllers;i++) {

			if ( controllers[i] ) {

				delete controllers[i];
				controllers[i] = NULL;
			}
		}

		delete controllers;
		controllers = NULL;
	}

	if ( preferences ) {

		delete preferences;
		preferences = NULL;
	}

	if ( tauOptimizer ) {

		delete tauOptimizer;
		tauOptimizer = NULL;
	}
}

int  TAU::All_Required_Preferences_Supplied(void) {

	if ( preferences )

		return( !preferences->ValFoundOffTheDiagonal(0.0) );
	else
		return( false );
}

void TAU::Controller_First_Preferred(void) {

	// Fill the row corresponding to the winner with +1 values.
	for (int j=0;j<numControllers;j++)
		if (	(preferences->Get(firstControllerIndex,j)==0) &&
			(j != firstControllerIndex) )
			preferences->Set(firstControllerIndex,j,+1.0);

	// Fill the column corresponding to the winner with -1 values.
	for (int i=0;i<numControllers;i++)
		if (	(preferences->Get(i,firstControllerIndex)==0) &&
			(i != firstControllerIndex) )
			preferences->Set(i,firstControllerIndex,-1.0);

	Scores_Update();
}

void TAU::Controller_First_Store_Sensor_Data(MATRIX *sensorData) {

	Controller_Store_Sensor_Data(firstControllerIndex,sensorData);
}

NEURAL_NETWORK *TAU::Controller_Get_Best(void) {

	double bestScore = -1000.0;
	NEURAL_NETWORK *best = NULL;

	for (int i=0; i<numControllers; i++)

		if ( controllers[i]->Score_Get() > bestScore ) {
			bestScore = controllers[i]->Score_Get();
			best = controllers[i];
		}

	return( best );
}

NEURAL_NETWORK *TAU::Controller_Pair_Get_First(void) {

	// Get the first of the two controllers about to be evaluated.

	return( controllers[firstControllerIndex] ); 
}

NEURAL_NETWORK *TAU::Controller_Pair_Get_Second(void) {

	// Get the second of the two controllers about to be evaluated.

	return( controllers[secondControllerIndex] );
}

void TAU::Controller_Second_Preferred(void) {

        // Fill the row corresponding to the winner with +1 values.
        for (int j=0;j<numControllers;j++)
                if (    (preferences->Get(secondControllerIndex,j)==0) &&
                        (j != secondControllerIndex) )
                        preferences->Set(secondControllerIndex,j,+1.0);

        // Fill the column corresponding to the winner with -1 values.
        for (int i=0;i<numControllers;i++)
                if (    (preferences->Get(i,secondControllerIndex)==0) &&
                        (i != secondControllerIndex) )
                        preferences->Set(i,secondControllerIndex,-1.0);

        Scores_Update();
}

void TAU::Controller_Second_Store_Sensor_Data(MATRIX *sensorData) {

        Controller_Store_Sensor_Data(secondControllerIndex,sensorData);
}

void TAU::Controllers_Load_Pair(ifstream *inFile) {

	int numControllers = 2;

	controllers = new NEURAL_NETWORK * [numControllers];

	for (int i=0; i<numControllers; i++)

		controllers[i] = new NEURAL_NETWORK(inFile);

	firstControllerIndex = 0;

	secondControllerIndex = 1;
}

void TAU::Controllers_Save_Pair(OPTIMIZER *optimizer, ofstream *outFile) {

	Controllers_Select_From_Optimizer(optimizer);

	controllers[firstControllerIndex]->Save_ButNotSensorData(outFile);

	controllers[secondControllerIndex]->Save_ButNotSensorData(outFile);
}

void TAU::Controllers_Select_From_Optimizer(OPTIMIZER *optimizer) {

	int controllersNeededFromOptimizer = Controllers_Num_Needed_From_Optimizer();

	switch ( controllersNeededFromOptimizer ) {

	case 0:
		Controllers_Select_Two_From_TAU();
		break;

	case 1:
		Controllers_Select_One_From_TAU_One_From_Optimizer(optimizer);
		break;

	case 2:
		Controllers_Select_Two_From_Optimizer(optimizer);
		break;
	}
}

void TAU::Optimize(void) {

	// If the user has not yet supplied any preferences,
	// there is nothing to optimize.
 
	if ( !Ready_For_Optimization() )

		return;

	if ( !tauOptimizer )

		tauOptimizer = new TAU_OPTIMIZER();

	tauOptimizer->Optimize( Controllers_Available_For_Optimization() , controllers );
}

void TAU::Print(void) {

	printf("Number of controllers stored in TAU: %d.\n",numControllers);

	if ( numControllers > 0 ) {

		printf("Number of preferences supplied: %d.\n",Num_Prefs());	

		printf("Average preferences per controller: %3.3f.\n", double(Num_Prefs())/double(numControllers) );
	}
}

int TAU::Ready_To_Predict(void) {

	if ( !tauOptimizer )

		return( false );

	return( tauOptimizer->Ready_To_Predict() );
}

void   TAU::Save(ofstream *outFile) {

	(*outFile) << numControllers << "\n";

	if ( !controllers )
		(*outFile) << "0\n";
	else {
		(*outFile) << "1\n";
		for (int i=0;	i<numControllers; i++)

			controllers[i]->Save(outFile,true);
	}

	if ( !preferences )
		(*outFile) << "0\n";
	else {
		(*outFile) << "1\n";
		preferences->Write(outFile);
	}

	(*outFile) << firstControllerIndex << "\n";
        (*outFile) << secondControllerIndex << "\n";

	if ( !tauOptimizer )
		(*outFile) << "0\n";
	else {
		(*outFile) << "1\n";
		tauOptimizer->Save(outFile);
	}

	(*outFile) << nextControllerForTraining << "\n";
	(*outFile) << scoreMin << "\n";
	(*outFile) << scoreMax << "\n";
}

double TAU::Score_Predict(NEURAL_NETWORK *controller) {

	if ( !tauOptimizer )

		return( TAU_NO_SCORE );

	return( tauOptimizer->Score_Predict(controller) );
}

void TAU::Store_Pref(int firstID, int secondID, int pref) {

	firstControllerIndex = Find_Index(firstID);

	secondControllerIndex = Find_Index(secondID);

	if ( pref==0 )

		Controller_First_Preferred();

	else
		Controller_Second_Preferred();
}

void TAU::User_Models_Reset(void) {

	if ( tauOptimizer )

		tauOptimizer->User_Models_Reset(Controllers_Available_For_Optimization() ,controllers);
}

// --------------------------- Private methods -----------------------------

void TAU::Controller_Store(NEURAL_NETWORK *newController) {

        if ( !controllers )

                Storage_Initialize();
        else
                Storage_Expand();

        controllers[numControllers] = new NEURAL_NETWORK(newController);

	controllers[numControllers]->fitness = newController->fitness;

	controllers[numControllers]->sensorTimeSeries = new MATRIX(newController->sensorTimeSeries);

        numControllers++;
}

void TAU::Controller_Store_Sensor_Data(int controllerIndex, MATRIX *sensorData) {

	if ( controllers[controllerIndex] )

		controllers[controllerIndex]->Store_Sensor_Data(sensorData);
}

int  TAU::Controllers_Available_For_Optimization(void) {

	int numAvailable = 0;

	for (int i=0;	i<numControllers;	i++) {

		if (	 controllers[i] &&
			 controllers[i]->Score_Available() &&
			(controllers[i]->Get_Sensor_Data()!=NULL) )

			numAvailable++;
	}

	return( numAvailable );
}

void TAU::Controllers_Expand(void) {

	NEURAL_NETWORK **temp = new NEURAL_NETWORK * [numControllers+1];

	for (int i=0;i<numControllers;i++) {

		temp[i] = controllers[i];
		controllers[i] = NULL;
	}

	temp[numControllers] = NULL;

	delete controllers;
	controllers = temp;

	temp = NULL;
}

void TAU::Controllers_Initialize(void) {

	controllers = new NEURAL_NETWORK * [1];

	controllers[0] = NULL;
}

int  TAU::Controllers_Num_Needed_From_Optimizer(void) {

	// If no controllers have yet been stored in TAU,
	// request two controllers from the optimizer for comparison.

	if ( numControllers == 0 )

		return( 2 );

	// If there is a zero entry at i,j in the preferences matrix,
	// that indicates that controllers i and j have not yet been
	// compared, so we don't need any new controllers from the optimizer.
 
	if ( preferences->ValFoundOffTheDiagonal(0.0) )

		return(0);

	// If there are no zero entries off the diagonal, that means
	// that every pair of controllers in TAU has been compared.
	// So, we need a new one from the optimizer to compare against
	// those stored in TAU.

	else
		return(1); 
}

void TAU::Controllers_Print(void) {

	for (int i=0;i<numControllers;i++)

		controllers[i]->Print();
}

void TAU::Controllers_Select_One_From_TAU_One_From_Optimizer(OPTIMIZER *optimizer) {

                // Choose the best controller stored in TAU to be compared against
                // a new controller drawn from the optimizer.

		// Choose best controller from TAU.

		double bestScore = -1000.0;

		for (int i=0;i<numControllers;i++)

			if ( controllers[i]->Score_Get() > bestScore ) {

				bestScore = controllers[i]->Score_Get(); 
				firstControllerIndex = i;
			}

		// Choose second controller from optimizer.
		if ( numControllers > 0 ) {
		
			NEURAL_NETWORK *secondController = NULL;

			if ( !tauOptimizer )

				secondController = optimizer->Genome_Get_First();
			else 
				secondController = optimizer->Genome_Get_Best_But_Not(numControllers,controllers);
				
			Controller_Store( secondController );
		}
		else
			Controller_Store( optimizer->Genome_Get_First() );

                secondControllerIndex = numControllers-1;
}

void TAU::Controllers_Select_Two_From_Optimizer(OPTIMIZER *optimizer) {

                // Choose two random yet distinct controllers from the optimizer.

                NEURAL_NETWORK *firstController = optimizer->Genome_Get_First();

                Controller_Store(firstController);

                firstControllerIndex = numControllers-1;

		NEURAL_NETWORK *secondController = optimizer->Genome_Get_Second();

                Controller_Store( secondController );

                secondControllerIndex = numControllers-1;

                firstController = NULL;
                secondController = NULL;
}

void TAU::Controllers_Select_Two_From_TAU(void) {

	// Choose the most recently-added controller.

	secondControllerIndex = numControllers-1;


	// Choose best controller from TAU.

	double bestScore = -1000.0;

	for (int i=0;i<numControllers-1;i++)

		// Find the best controller...		
		if (	(controllers[i]->Score_Get() > bestScore) &&

			// ...that hasn't yet been evaluated against
			// the new controller.

			(preferences->Get(i,secondControllerIndex)==0) ) {

                	bestScore = controllers[i]->Score_Get();
                        firstControllerIndex = i;
                }
}

int  TAU::Find_Index(int ID) {

	for (int i=0; i<numControllers; i++)

		if ( controllers[i]->ID == ID )

			return( i );
}

int TAU::Num_Prefs(void) {

	if ( !preferences )

		return( 0 );

	int numZeros = preferences->NumOccurancesOf(0.0);

	int numNonZeroValues = numControllers*numControllers - numZeros;

	int numPrefs = int( double(numNonZeroValues) / 2.0 );

	return( numPrefs );
}

void TAU::Preferences_Expand(void) {

	preferences->AddColumn(0);

	preferences->AddRow(0);
}

void TAU::Preferences_Initialize(void) {

	preferences = new MATRIX(1,1,0);
}

void TAU::Preferences_Print(void) {

	preferences->Print();
}

int  TAU::Ready_For_Optimization(void) {

	// If the user has not indicated any preferences yet,
	// there is nothing to optimize.

	if ( !preferences )

		return( false );

	// Optimization can occur if there are at least 
	// two controllers that have a non-zero score 
	// assigned to them in the preferences matrix.

	return( preferences->ValuesDifferentFrom(0) >= 2 );
}

double TAU::Scale(double value, double min1, double max1,
                                           double min2, double max2) {

        if ( min1 < 0 )
                value = value - min1;
        else
                value = value + min1;

        return( (value*(max2-min2)/(max1-min1)) + min2 );
}

void TAU::Scores_Print(void) {

	for (int i=0; i<numControllers; i++)

		printf("%0.0f ",controllers[i]->Score_Get() );

	printf("\n");
}

void TAU::Scores_Update(void) {

	for (int i=0; i<numControllers; i++) {

		if ( preferences && controllers[i] ) {

			double score = preferences->SumOfRow(i);

			controllers[i]->Score_Set(score);

			if ( score < scoreMin )
				scoreMin = score;

			if ( score > scoreMax )
				scoreMax = score;
		}
	}

	for (int i=0; i<numControllers; i++) {

		double score = controllers[i]->Score_Get();

		score = Scale(score,scoreMin,scoreMax,0,1);

		controllers[i]->Score_Set(score);
	}

	Optimize();
}

void TAU::Storage_Expand(void) {

	Controllers_Expand();

	Preferences_Expand();
}

void TAU::Storage_Initialize(void) {

	Controllers_Initialize();

	Preferences_Initialize();
}

#endif
