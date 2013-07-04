#include "math.h"
#include "stdio.h"

#ifndef _OPTIMIZER_CPP
#define _OPTIMIZER_CPP

#include "optimizer.h"
#include "robot.h"

extern int	EVALUATION_TIME;
extern double	MUTATION_PROBABILITY;
extern int	EVALS_UNTIL_EVAL_TIME_EXTENDED;
extern int	STARTING_EVALUATION_TIME;
extern int	EVAL_TIME_EXTENSION_PERIOD;
extern double	WORST_FITNESS;
extern double   TAU_NO_SCORE;

extern int	AFPO_POP_SIZE;

OPTIMIZER::OPTIMIZER(int numberOfSensors, int numberOfMotors) {

	Initialize();

	numSensors = numberOfSensors;
	numMotors  = numberOfMotors;
}

OPTIMIZER::OPTIMIZER(ifstream *inFile) {

	Initialize();

	(*inFile) >> nextGenomeID;

	(*inFile) >> numSensors;
	(*inFile) >> numMotors;

	(*inFile) >> evaluationPeriod;

	(*inFile) >> mutationProbability;

	(*inFile) >> generation;

	int areGenomes;
	(*inFile) >> areGenomes;

	if ( areGenomes )

		Genomes_Load(inFile);
	else
		genomes = NULL;
}

OPTIMIZER::~OPTIMIZER(void) {

	Destroy();
}

void OPTIMIZER::EvaluationPeriod_Decrease(void) {

	evaluationPeriod--;

	if ( evaluationPeriod < 10 )

		evaluationPeriod = 10;
}

void OPTIMIZER::EvaluationPeriod_Increase(void) {

	evaluationPeriod++;
}

void OPTIMIZER::Fitness_Sensor_Data_Receive(		NEURAL_NETWORK *userFavorite,
							double fitness,
							MATRIX *timeSeries) {
        if ( genomeUnderEvaluation ) 

                genomeUnderEvaluation->Fitness_Sensor_Data_Set(fitness,timeSeries);
        else 
                Genome_Get_Next_To_Evaluate(userFavorite)->Fitness_Sensor_Data_Set(fitness,timeSeries);
}

void OPTIMIZER::Fitness_Sensor_Data_Score_Receive(	NEURAL_NETWORK *userFavorite,
							double fitness,
							MATRIX *timeSeries,
							double score) {
        if ( genomeUnderEvaluation )

                genomeUnderEvaluation->Fitness_Sensor_Data_Score_Set(fitness,timeSeries,score);
        else
                Genome_Get_Next_To_Evaluate(userFavorite)->Fitness_Sensor_Data_Score_Set(fitness,timeSeries,score);
}

void OPTIMIZER::Fitness_Receive(NEURAL_NETWORK *userFavorite, double fitness) {

	if ( genomeUnderEvaluation ) {

		genomeUnderEvaluation->Fitness_Set(fitness);
	}
	else {
		Genome_Get_Next_To_Evaluate(userFavorite)->Fitness_Set(fitness);
	}
}

void OPTIMIZER::Genome_Discard_Being_Evaluated(void) {

	if ( genomeUnderEvaluation ) {

		delete genomeUnderEvaluation;
		genomeUnderEvaluation = NULL;
	}
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get(int i) {

	return( genomes[i] );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_Best(void) {

	if ( !genomes )

		return( NULL );

	if ( !genomes[0] )

		return( NULL );

	return( genomes[0] );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_Best_But_Not(NEURAL_NETWORK *other) {

        int genomeIndex = 0;

        while ( (genomes[genomeIndex]==other) ||
                (genomes[genomeIndex]->fitness == other->fitness) ) {

                genomeIndex++;

                // If no appropriate controller can be found,
                // return a random one.
        }

        return( genomes[genomeIndex] );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_Best_But_Not(int numControllers, NEURAL_NETWORK **controllers) {

        int genomeIndex = 0; 

        int found = false;

        while ( !found  ) {

                // The genome has not yet been evaluated.

                        int otherIndex = 0;
                        int equal = false;

                        while ( (otherIndex<numControllers) && (!equal) ) {

                                if (    (genomes[genomeIndex]==controllers[otherIndex]) ||
                                        (genomes[genomeIndex]->fitness==controllers[otherIndex]->fitness) )

                                        equal = true;
                                else
                                        otherIndex++;
                        }

                        if ( equal ) {

                                // The controller is evaluated, but it's equal
                                // another controller in the external list.

                                genomeIndex++;
                        }
                        else {
                                // The controller is evaluated, and it's unique.
                                found = true;
                        }
        }

        return( genomes[genomeIndex] );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_Curr_To_Evaluate(void) {

	if ( !genomes )

		Genomes_Create();


	if ( !genomeUnderEvaluation )

		return ( Genome_Find_Next_Not_Evaluated() );

	else
		return genomeUnderEvaluation;
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_First(void) {

	return( genomes[0] );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_Most_Different(int numControllers, NEURAL_NETWORK **controllers) {

	int chosenGenomeIndex;
	NEURAL_NETWORK *chosenGenome = NULL;
	double maxDistance = -1000.0;

	for (int i=0;i<AFPO_POP_SIZE;i++) {

                if (    genomes[i]->evaluated &&
                        genomes[i]->sensorTimeSeries ) {

			double currDistance = genomes[i]->Min_Distance_To(numControllers,controllers);

			if ( currDistance > maxDistance ) {
				maxDistance = currDistance;
				chosenGenomeIndex = i;
				chosenGenome = genomes[i];
			}
		}
	}

	if ( !chosenGenome )
	
		return( controllers[0] );

	else
		return( chosenGenome );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_Most_Different_But_Not(NEURAL_NETWORK *thisOne, int numControllers, NEURAL_NETWORK **controllers) {

        int chosenGenomeIndex;
        NEURAL_NETWORK *chosenGenome = NULL;
        double maxDistance = -1000.0;

        for (int i=0;i<AFPO_POP_SIZE;i++) {

                if ( 	genomes[i]->evaluated && 
			genomes[i]->sensorTimeSeries && 
			(!genomes[i]->Is_The_Same_As(thisOne)) ) {

                        double currDistance = genomes[i]->Min_Distance_To(numControllers,controllers);

                        if ( currDistance > maxDistance ) {
                                maxDistance = currDistance;
                                chosenGenomeIndex = i;
                                chosenGenome = genomes[i];
                        }
                }
        }

        if ( !chosenGenome )
                return( controllers[0] );
        else
                return( chosenGenome );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_Next_To_Evaluate(NEURAL_NETWORK *userFavorite) {

	NEURAL_NETWORK *genomeToReturn;

	if ( !genomes )

		Genomes_Create();

	if ( genomeUnderEvaluation )

		genomeToReturn = genomeUnderEvaluation;

	else if ( Genomes_All_Evaluated() )

		Generation_Create_Next(userFavorite);

	genomeToReturn = Genome_Find_Next_Not_Evaluated();

	return( genomeToReturn );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_Random(void) {

	int genomeIndex = RandInt(0,AFPO_POP_SIZE-1);

	while ( genomes[genomeIndex]->fitness == 0.0 )

		genomeIndex = RandInt(0,AFPO_POP_SIZE-1);

	return( genomes[genomeIndex] );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_Random_But_Not(NEURAL_NETWORK *other) {

	// Return a random genome, but don't choose one that is equal
	// to genome `other'.

	int genomeIndex = RandInt(0,AFPO_POP_SIZE-1);
	int numberOfTries = 0;

	while (	(genomes[genomeIndex]==other) ||
		(genomes[genomeIndex]->fitness == 0.0) ||
		(genomes[genomeIndex]->fitness == other->fitness) ) {

		genomeIndex = RandInt(0,AFPO_POP_SIZE-1);

		numberOfTries++;

		// If no appropriate controller can be found,
		// return a random one.

		if ( numberOfTries >= 10000 )
			return( genomes[genomeIndex] );
	}

	return( genomes[genomeIndex] );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_Random_But_Not(int numControllers, NEURAL_NETWORK **controllers) {

	// Return a random genome, but make sure that it doesn't achieve
	// the same fitness as any in the external list of controllers
	// supplied as a parameter.

        int genomeIndex = RandInt(0,AFPO_POP_SIZE-1);

	int found = false; 

	int numberOfTries = 0;

	while ( !found  ) {

		// The genome has not yet been evaluated.

		if ( genomes[genomeIndex]->fitness == 0.0 )

			genomeIndex = RandInt(0,AFPO_POP_SIZE-1);

		else {
			int otherIndex = 0;
			int equal = false;

			while ( (otherIndex<numControllers) && (!equal) ) {
	
				if ( 	(genomes[genomeIndex]==controllers[otherIndex]) ||
					(genomes[genomeIndex]->fitness==controllers[otherIndex]->fitness) )

					equal = true;
				else
					otherIndex++;
			}

			if ( equal ) {

				// The controller is evaluated, but it's equal
				// another controller in the external list.

				genomeIndex = RandInt(0,AFPO_POP_SIZE-1);
			}
			else {
				// The controller is evaluated, and it's unique.
				found = true;
			}
		}

		numberOfTries++;

		// If no appropriate controllers can be found,
		// return a random one that has at least been evaluated.

		if ( numberOfTries >= 10000 ) {

			return( Genome_Get_Random_But_Not(controllers[0]) );
		}
	}

	return( genomes[genomeIndex] );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Get_Second(void) {

        return( genomes[1] );
}

void OPTIMIZER::Genome_Put_At_End(NEURAL_NETWORK *other) {

	delete genomes[AFPO_POP_SIZE-1];

	genomes[AFPO_POP_SIZE-1] = new NEURAL_NETWORK(other);
}

int  OPTIMIZER::Genomes_Num_Of_Evaluated(void) {

        if ( !genomes )

                return( 0 );

        int num = 0;

        for (int i=0; i< AFPO_POP_SIZE; i++)

                if ( (genomes[i]) && (genomes[i]->fitness != 0.0) )
                        num++;

        return( num );
}
 
void OPTIMIZER::MutationProbability_Decrease(void) {

	mutationProbability = mutationProbability /2.0;
}

void OPTIMIZER::MutationProbability_Increase(void) {

	mutationProbability = mutationProbability * 2.0;

	if ( mutationProbability > 1.0 )
		mutationProbability = 1.0;
}

void OPTIMIZER::Print(void) {

	printf("Generation %d: \n",generation);

	Genomes_Print();
	printf("\n");
}

void OPTIMIZER::Reset(void) {

	Destroy();

	Initialize();
}

void OPTIMIZER::Reset_Genomes(void) {

	for (int i=0;	i<AFPO_POP_SIZE;	i++)

		Genome_Get(i)->Reset();

	Timer_Reset();

	genomeUnderEvaluation = NULL;
}

void OPTIMIZER::Save(ofstream *outFile, int saveTimeSeries) {

	(*outFile) << nextGenomeID		<< "\n";

	(*outFile) << numSensors		<< "\n";
	(*outFile) << numMotors			<< "\n";

	(*outFile) << evaluationPeriod		<< "\n";
	(*outFile) << mutationProbability	<< "\n";

	(*outFile) << generation		<< "\n";

	if ( genomes ) {
		(*outFile) << "1"		<< "\n";
		Genomes_Save(outFile,saveTimeSeries);
	}
	else
		(*outFile) << "0"		<< "\n";
}

void OPTIMIZER::Score_Receive(double score) {

        if ( genomeUnderEvaluation ) {

                genomeUnderEvaluation->Score_Set(score);
        }
        else {
                Genome_Get_Curr_To_Evaluate()->Score_Set(score);
        }
}

void OPTIMIZER::Scores_Reset(void) {

	for (int i=0;	i<AFPO_POP_SIZE;	i++)

		if ( genomes[i] )

			genomes[i]->Score_Reset();
}

void OPTIMIZER::Sensor_Data_Receive(MATRIX *timeSeries) {

	if ( genomeUnderEvaluation )

		genomeUnderEvaluation->Store_Sensor_Data(timeSeries);
	else
		Genome_Get_Curr_To_Evaluate()->Store_Sensor_Data(timeSeries);

}

int OPTIMIZER::Time_Elapsed(void) {

	return (timer >= evaluationPeriod);

}

void OPTIMIZER::Timer_Reset(void) {

	timer = 0;
}

void OPTIMIZER::Timer_Update(void) {

	timer++;
}

// --------------------------- Private methods ------------------------

void OPTIMIZER::Destroy(void) {

	Genomes_Destroy();
}

int OPTIMIZER::FlipCoin(void) {

	return( Rand(0.0,1.0) < 0.5 );
}

void OPTIMIZER::Generation_Create_Next(NEURAL_NETWORK *userFavorite) {

	Genomes_Find_Pareto_Front();

	Genomes_Increase_Age();

	Genomes_Sort();

	Print();

	Genomes_Delete_Dominated();

	Genomes_Fill_Empty_Slots();

	//if ( userFavorite )
	//	Genomes_Inject_User_Favorite(userFavorite);

	Genomes_Inject_Random_Genome();

	generation++;
}

void OPTIMIZER::Genome_Copy(int genomeIndex, int parentID) {

	genomes[genomeIndex] = new NEURAL_NETWORK(genomes[parentID]);

	genomes[genomeIndex]->ID_Set(nextGenomeID++);
}

void OPTIMIZER::Genome_Create_Random(int genomeIndex) {

	genomes[genomeIndex] = 

		new NEURAL_NETWORK(nextGenomeID++,numSensors,numMotors,0);
}

void OPTIMIZER::Genome_Destroy(int genomeIndex) {

	delete genomes[genomeIndex];

	genomes[genomeIndex] = NULL;
}

int OPTIMIZER::Genome_Evaluated(int genomeIndex) {

	return( genomes[genomeIndex]->Evaluated() );
}

NEURAL_NETWORK *OPTIMIZER::Genome_Find_Next_Not_Evaluated(void) {

	int found = false;
	
	int genomeIndex = 0;

	while (	(!found) &&

		(genomeIndex < AFPO_POP_SIZE ) ) {

		if ( !genomes[genomeIndex]->Evaluated() ) 

			found = true;
		else 
			genomeIndex++;
	}

	return( genomes[genomeIndex] );
}

void OPTIMIZER::Genome_Load(int genomeIndex, ifstream *inFile) {

	genomes[genomeIndex] = 

		new NEURAL_NETWORK(inFile);
}

void OPTIMIZER::Genome_Print(int genomeIndex) {

	if ( genomes[genomeIndex] )

		genomes[genomeIndex]->Print();
}

int OPTIMIZER::Genomes_All_Evaluated(void) {

	int allEvaluated = true;

	int genomeIndex = 0;

	while ( (allEvaluated) && 

		(genomeIndex < AFPO_POP_SIZE) ) {

		if ( !Genome_Evaluated(genomeIndex) )

			allEvaluated = false;

		else
			genomeIndex++;
	}

	return( allEvaluated );
}

void OPTIMIZER::Genomes_Create(void) {

	genomes = new NEURAL_NETWORK * [ AFPO_POP_SIZE ];

	for (int i=0;	i<AFPO_POP_SIZE;	i++)

		Genome_Create_Random(i);
}

void OPTIMIZER::Genomes_Delete_Dominated(void) {

        for (int i=0;   i<AFPO_POP_SIZE;        i++)

		if ( genomes[i]->dominated ) {
			delete genomes[i];
			genomes[i] = NULL;
		}

}

void OPTIMIZER::Genomes_Destroy(void) {

        for (int i=0;   i<AFPO_POP_SIZE;        i++)

		Genome_Destroy(i);

	delete[] genomes;
	genomes = NULL;
}

void OPTIMIZER::Genomes_Fill_Empty_Slots(void) {

	int numNonDominated = 0;

	while ( genomes[numNonDominated] )

		numNonDominated++;

	int currentEmptySlot = numNonDominated;

	for (int i=currentEmptySlot; i<AFPO_POP_SIZE; i++) {

		int parentID = RandInt(0,numNonDominated-1);

		Genome_Copy(i,parentID);
		genomes[i]->Mutate();
	}
}

void OPTIMIZER::Genomes_Find_Pareto_Front(void) {

        for (int i=0;   i<AFPO_POP_SIZE;        i++)

                genomes[i]->dominated = false;

        for (int i=0;   i<AFPO_POP_SIZE;        i++)

                for (int j=0;   j<AFPO_POP_SIZE;        j++)

                        if (    (i!=j) &&
                                genomes[i]->Is_Dominated_By(genomes[j]) )

                                genomes[i]->dominated = true;

}

void OPTIMIZER::Genomes_Increase_Age(void) {

        for (int i=0;   i<AFPO_POP_SIZE;        i++)

                genomes[i]->age++;
}

void OPTIMIZER::Genomes_Inject_User_Favorite(NEURAL_NETWORK *userFavorite) {

	delete genomes[AFPO_POP_SIZE-2];

	genomes[AFPO_POP_SIZE-2] = new NEURAL_NETWORK(userFavorite);
}

void OPTIMIZER::Genomes_Inject_Random_Genome(void) {

	delete genomes[AFPO_POP_SIZE-1];

	Genome_Create_Random(AFPO_POP_SIZE-1);
}

void OPTIMIZER::Genomes_Load(ifstream *inFile) {

	genomes = new NEURAL_NETWORK * [ AFPO_POP_SIZE ];

	for (int i=0;	i<AFPO_POP_SIZE;	i++)

		Genome_Load(i,inFile);
}

void OPTIMIZER::Genomes_Print(void) {

	for (int i=0;	i<AFPO_POP_SIZE; 	i++) { 

		Genome_Print(i);

		if (	(i<(AFPO_POP_SIZE-1)) && 
			(!genomes[i]->dominated) &&
			(genomes[i+1]->dominated) )
			printf("\n");	
	}
	
	printf("\n");
}

void OPTIMIZER::Genomes_Save(ofstream *outFile, int saveTimeSeries) {

	for (int i=0;	i<AFPO_POP_SIZE;	i++)

		genomes[i]->Save(outFile,saveTimeSeries);
}

void OPTIMIZER::Genomes_Sort(void) {

	int swapped = true;

	while ( swapped ) {

		swapped = false;

		for (int i=1;	i<AFPO_POP_SIZE; i++) {

			if (	genomes[i-1]->Is_Inferior_To(genomes[i]) ) {

				Genomes_Swap(i-1,i);
				swapped = true;
			}
		}
	}
}

void OPTIMIZER::Genomes_Swap(int firstIndex, int secondIndex) {

	NEURAL_NETWORK *temp = genomes[firstIndex];
	genomes[firstIndex]  = genomes[secondIndex];
	genomes[secondIndex] = temp;
	temp = NULL;
}

void OPTIMIZER::Initialize(void) {

	genomes = NULL;
	genomeUnderEvaluation = NULL;

	nextGenomeID = 0;

	evaluationPeriod = STARTING_EVALUATION_TIME;

	mutationProbability = MUTATION_PROBABILITY;

	generation = 0;   

	Timer_Reset();
}

double OPTIMIZER::Rand(double min, double max) {

	// Return a random value in [min,max] with
	// a uniform distribution.

        double zeroToOne = ((double)rand()) / RAND_MAX;
        double returnVal;

        returnVal = (zeroToOne * (max-min)) + min;
        return returnVal;
}

int OPTIMIZER::RandInt(int min, int max) {

	if ( min == max )
		return( min );
	else {
		int val = (rand() % (max-min+1)) + min;
		if ( val > max )
			val = max;
		if ( val < min )
			val = min;	
		return( val );
	}
}

#endif
