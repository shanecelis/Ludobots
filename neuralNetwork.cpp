#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include <iomanip>

#ifndef _NEURAL_NETWORK_CPP
#define _NEURAL_NETWORK_CPP

#include "neuralNetwork.h"
#include "matrix.h"

extern double	TAU_NO_SCORE;
extern int	STARTING_EVALUATION_TIME;
extern int	ALG_VARIANT_PREFS_ONLY;

extern int	TAU_NUM_SENSOR_ROWS;

extern double	SENSOR_TYPE_COMPASS;
extern double	SENSOR_TYPE_LIGHT;
extern double	SENSOR_TYPE_PROPRIOCEPTIVE;
extern double	SENSOR_TYPE_TOUCH;

NEURAL_NETWORK::NEURAL_NETWORK(	int myID, int nS, int nM, int myAge ) {

	ID		= myID;
	fitness		= 0.0;
	evaluated	= false;

	numSensors	= nS;

	numMotors 	= nM;

	age = myAge;

	Initialize();

	weights		= new MATRIX(numSensors+numMotors,numMotors,0.0);

	weights->Randomize(-1.0,1.0);	
	weights->RoundToDecimalPlace(6);

	taus		= new MATRIX(1,numMotors,0.0);

	taus->Randomize(1.0,20.0);
	taus->RoundToDecimalPlace(6);
}

NEURAL_NETWORK::NEURAL_NETWORK(	NEURAL_NETWORK *other ) {

	ID		= other->ID;
	fitness		= 0.0;
	evaluated	= false;

	numSensors	= other->numSensors;

	numMotors  	= other->numMotors;

	age 		= other->age;

	Initialize();

        weights         = new MATRIX(other->weights);
        weights->RoundToDecimalPlace(6);

        taus        = new MATRIX(other->taus);
        taus->RoundToDecimalPlace(6);
}

NEURAL_NETWORK::NEURAL_NETWORK(ifstream *inFile) {

	int isWeights;
	(*inFile) >> isWeights;
	if ( isWeights )

		weights = new MATRIX(inFile);
	else
		weights = NULL;

        int isTaus;
        (*inFile) >> isTaus;
        if ( isTaus ) {
                taus = new MATRIX(inFile);
                taus->RoundToDecimalPlace(6);
        }
        else
                taus = NULL;

	(*inFile) >> numSensors;

        (*inFile) >> numMotors;

        (*inFile) >> ID;

	// Round the fitness value to a certain precision
	double tempFit;
	(*inFile) >> tempFit;
	fitness = tempFit;
	//Fitness_Set(tempFit);

        (*inFile) >> evaluated;
        (*inFile) >> age;

	Initialize();

	(*inFile) >> score;

        int isTimeSeries;
        (*inFile) >> isTimeSeries;
        if ( isTimeSeries )
                sensorTimeSeries = new MATRIX(inFile);
	else
                sensorTimeSeries = NULL;
}

NEURAL_NETWORK::~NEURAL_NETWORK(void) {

	if ( weights ) {
		delete weights;
		weights = NULL;
	}

        if ( taus ) {
                delete taus;
                taus = NULL;
        }

	if ( neuronValues ) {
		delete neuronValues;
		neuronValues = NULL;
	}

	if ( temp ) {
		delete temp;
		temp = NULL;
	}

	if ( sensorValues ) {
		delete sensorValues;
		sensorValues = NULL;
	}

	if ( sensorTypes ) {
		delete sensorTypes;
		sensorTypes = NULL;
	}

	if ( sensorTimeSeries ) {

		delete sensorTimeSeries;
		sensorTimeSeries = NULL;
	}
}

int    NEURAL_NETWORK::Age_Get(void) {

        return( age );
}

int    NEURAL_NETWORK::Evaluated(void) {

	return( evaluated );
}

int    NEURAL_NETWORK::Fitness_Equal_To(double fit) {

	return( fitness == fit );
}

double NEURAL_NETWORK::Fitness_Get(void) {

	return( fitness );
}

void   NEURAL_NETWORK::Fitness_Sensor_Data_Set(		double fit,
							MATRIX *timeSeries) {
        Fitness_Set(fit);

        Store_Sensor_Data(timeSeries);
}

void   NEURAL_NETWORK::Fitness_Sensor_Data_Score_Set(	double fit,
							MATRIX *timeSeries,
							double sc) {
	Fitness_Set(fit);

	Store_Sensor_Data(timeSeries);

	Score_Set(sc);
}

void   NEURAL_NETWORK::Fitness_Set(double fit) {

	// Round the fitness value to four decimal places.
        fit = fit * pow(10.0,double(6));
        fit = int(fit);
        fit = double(fit);
        fit = fit / pow(10.0,double(6));
	
	fitness = fit;

	evaluated = true;
}

int    NEURAL_NETWORK::Fitness_Worse_Than(double fit) {

	return( fitness < fit );
}

double NEURAL_NETWORK::Get_Motor_Neuron_Value(int motorIndex) {

	return( neuronValues->Get(0,motorIndex) );
}

MATRIX *NEURAL_NETWORK::Get_Sensor_Data(void) {

	return( sensorTimeSeries );
}

int  NEURAL_NETWORK::ID_Get(void) {

	return( ID );
}

void NEURAL_NETWORK::ID_Set(int myID) {

	ID = myID;
}

int  NEURAL_NETWORK::Is_Dominated_By(NEURAL_NETWORK *other) {

	double combinedFitness;
	double otherCombinedFitness;

	// If the TAU has predicted the score of one or both
	// genomes, use them.
	// If a genome has a score and the other one doesn't,
	// the genome with a score is likely to dominate the
	// one that doesn't. This ensures that, genomes without
	// predicted scores are gradually replaced in the population.

	combinedFitness =       1.0/(1.0+fitness);

	if ( Score_Available() ) {

		combinedFitness = combinedFitness * Score_Get();
	}

	otherCombinedFitness =  1.0/(1.0+other->fitness);

	if ( other->Score_Available() ) {

			otherCombinedFitness = otherCombinedFitness * other->Score_Get();
	}

	if ( combinedFitness == otherCombinedFitness ) {

		if ( age == other->age )

			return( ID < other->ID );
		else
			return( age > other->age );
	}	
	else {
		if ( age == other->age )

			return( combinedFitness  < otherCombinedFitness );
		else
			return( (combinedFitness < otherCombinedFitness) &&
				(age             > other->age) );
	}
}

int  NEURAL_NETWORK::Is_Inferior_To(NEURAL_NETWORK *other) {

	double combinedFitness, otherCombinedFitness;

        combinedFitness =       1.0/(1.0+fitness);

        if ( Score_Available() ) {

                combinedFitness = combinedFitness * Score_Get();
	}

        otherCombinedFitness =  1.0/(1.0+other->fitness);

        if ( other->Score_Available() ) {

                otherCombinedFitness = otherCombinedFitness * other->Score_Get();
	}

	if ( dominated == other->dominated )

		return( combinedFitness < otherCombinedFitness );

	else

		return( (dominated) && (!other->dominated) );
}

int  NEURAL_NETWORK::Is_The_Same_As(NEURAL_NETWORK *other) {

	return( ID == other->ID );
}

double NEURAL_NETWORK::Min_Distance_To(int numControllers, NEURAL_NETWORK **controllers) {

	int closestControllerIndex;
	double minDistance = 1000.0;
	NEURAL_NETWORK *closestController = NULL;

	for (int i=0; i<numControllers; i++) {

		if (	  controllers[i] &&
			  controllers[i]->sensorTimeSeries &&
			(!controllers[i]->Is_The_Same_As(this)) ) {

			int    timeStep = int(double(STARTING_EVALUATION_TIME)/2.0);

			double currDistance = 0.0;

			for (int j=0;j<11;j=j+2)

				currDistance = currDistance + 
						fabs(                 sensorTimeSeries->Get(timeStep,j) - 
						      controllers[i]->sensorTimeSeries->Get(timeStep,j) );

			currDistance = currDistance/6.0;

			if ( currDistance < minDistance ) {
				
				minDistance = currDistance;
				closestControllerIndex = i;
				closestController = controllers[i];
			}	
		}
	}

	return( minDistance );
}

void NEURAL_NETWORK::Mutate(void) {

	Mutate_Weights();

	Mutate_Taus();
}

void NEURAL_NETWORK::Print(void) {

	printf("fit:       %10.10f \t",fitness);
	printf("score:	   %5.5f \t",score);
	printf("ID:        %d \t",ID);
        printf("dominated: %d \t",dominated);
	printf("age:       %d \t",age);
	printf("\n");
}

void NEURAL_NETWORK::Print_Sensor_Data(void) {

	if ( sensorTimeSeries )

		sensorTimeSeries->Print(6);
}

void NEURAL_NETWORK::Record_Sensor_Data(int evaluationPeriod) {

	if ( !sensorTimeSeries )

		sensorTimeSeries = new MATRIX(evaluationPeriod,numSensors,0.0);
}

void NEURAL_NETWORK::Reset(void) {

	fitness = 0.0;
	score = TAU_NO_SCORE;
	evaluated = false;
}

void NEURAL_NETWORK::Save(ofstream *outFile, int saveTimeSeries) {

        if ( weights ) {
                (*outFile)                              << "1\n";
                weights->Write(outFile);
        }
        else
                (*outFile)                              << "0\n";

        if ( taus ) {
                (*outFile)                              << "1\n";
                taus->Write(outFile);
        }
        else
                (*outFile)                              << "0\n";

        (*outFile) << numSensors                        << "\n";

        (*outFile) << numMotors                         << "\n";

        (*outFile) << ID                                << "\n";
        (*outFile) << fitness                           << "\n";
        (*outFile) << evaluated                         << "\n";
        (*outFile) << age                               << "\n";
        (*outFile) << score                             << "\n";

        if ( sensorTimeSeries && saveTimeSeries ) {
                (*outFile)                              << "1\n";
                sensorTimeSeries->Write(outFile);
        }
        else
                (*outFile)                              << "0\n";
}

void NEURAL_NETWORK::Save_ButNotSensorData(ofstream *outFile) {

        if ( weights ) {
                (*outFile)                              << "1\n";
                weights->Write(outFile);
        }
        else
                (*outFile)                              << "0\n";

        if ( taus ) {
                (*outFile)                              << "1\n";
                taus->Write(outFile);
        }
        else
                (*outFile)                              << "0\n";

        (*outFile) << numSensors                        << "\n";

        (*outFile) << numMotors                         << "\n";

        (*outFile) << ID                                << "\n";
        (*outFile) << fitness                           << "\n";
        (*outFile) << evaluated                         << "\n";
        (*outFile) << age                               << "\n";
        (*outFile) << score                             << "\n";

	// Do not save sensor data
	(*outFile)					<< "0\n";
}

int    NEURAL_NETWORK::Score_Available(void) {

	return( Score_Get() != TAU_NO_SCORE ); 
}

double NEURAL_NETWORK::Score_Get(void) {

	return( score );
}

void NEURAL_NETWORK::Score_Reset(void) {

	score = TAU_NO_SCORE;
}

void NEURAL_NETWORK::Score_Set(double val) {

	score = val;
}

void NEURAL_NETWORK::Sensor_Set(int sensorIndex, double sensorValue, double sensorType) {

	sensorValues->vals[sensorIndex] = sensorValue;

	sensorTypes->vals[sensorIndex] = sensorType;
}

void NEURAL_NETWORK::Store_Sensor_Data(MATRIX *externalSensorData) {
	
	// Store sensor data generated externally by a copy of
	// this neural network, but only if there is no sensor
	// data stored here already.

	if ( !sensorTimeSeries )

		sensorTimeSeries = new MATRIX(externalSensorData);

	// If the incoming sensor data set was recorded for more
	// time steps compared to the sensor data currently stored,
	// replace it.
 
	else {

		int dataRecordedUntilForCurrentDataSet = sensorTimeSeries->FirstRowWithSum(0.0);

		int dataRecordedUntilForExternalDataSet = externalSensorData->FirstRowWithSum(0.0);

		if (    dataRecordedUntilForExternalDataSet >
			dataRecordedUntilForCurrentDataSet ) {

			delete sensorTimeSeries;

			sensorTimeSeries = new MATRIX(externalSensorData);
		}
	}
}

void NEURAL_NETWORK::Update(int timeStep) {

	if ( sensorTimeSeries )

		Store_Sensor_Data(timeStep);

	for (int i=0;	i<numMotors;	i++) {

		temp->vals[i] = 0;

		for (int s=0; s<numSensors; s++) {

			temp->vals[i] = temp->vals[i] +

						weights->vals[s*weights->width + i] *

						sensorValues->vals[s];
		}

		for (int m=0; m<numMotors; m++)

			temp->vals[i] = temp->vals[i] + 

						weights->vals[(numSensors+m)*weights->width + i] *

						neuronValues->vals[m];
	}

	// Scale neuron values to lie in [-1,1]

	for (int i=0;	i<numMotors;	i++)

		neuronValues->vals[i] = atan(temp->vals[i]) / (3.14159/2.0);
}

void NEURAL_NETWORK::Writeout(ofstream *outFile) {

        double pos0 = sensorTimeSeries->Get(STARTING_EVALUATION_TIME-1,11);
        double pos1 = sensorTimeSeries->Get(STARTING_EVALUATION_TIME-1,12);

	// Disance from the target object at [0,6]

	double finalDistance = sqrt( pow(pos0-0.0,2.0) + pow(pos1-6.0,2.0) );
	
	(*outFile) << finalDistance << "\t";

	(*outFile) << fitness << "\t";

	(*outFile) << score << "\t";

        (*outFile) << ID << "\t";

	(*outFile) << age << "\t";
}

// Private methods ---------------------------------------------

void NEURAL_NETWORK::Initialize(void) {

	neuronValues 	= new MATRIX(1,numMotors,0.0);

	temp 		= new MATRIX(1,numMotors,0.0);

	sensorValues	= new MATRIX(1,numSensors,0.0);

	sensorTypes     = new MATRIX(1,numSensors,0.0);

	dominated       = false;

	sensorTimeSeries = NULL;

	Score_Set(TAU_NO_SCORE);
}

void NEURAL_NETWORK::Mutate_Taus(void) {

        for (int j=0; j<numMotors; j++) {

		taus->Add(0,j,RandN()/(double(age)));
        }

        taus->RoundToDecimalPlace(6);
}

void NEURAL_NETWORK::Mutate_Weights(void) {

	for (int i=0;i<(numSensors+numMotors);i++) {

		for (int j=0;j<numMotors;j++) {

			weights->Add(i,j,RandN()/(double(age)));
		}
	}

	weights->RoundToDecimalPlace(6);
}

double NEURAL_NETWORK::Rand(double min, double max) {

        double zeroToOne = ((double)rand()) / RAND_MAX;
        double returnVal;

        returnVal = (zeroToOne * (max-min)) + min;
        return returnVal;
}

double NEURAL_NETWORK::RandN(void) {

        double u1 = Rand(0,1);
        double u2 = Rand(0,1);

        double z0 = sqrt( -2.0 * log(u1) ) * cos( 2.0 * 3.14159 * u2 );

        return( z0 );
}

double NEURAL_NETWORK::Scale(double value, double min1, double max1,
                                           double min2, double max2) {

        if ( min1 < 0 )
                value = value - min1;
        else
                value = value + min1;

        return( (value*(max2-min2)/(max1-min1)) + min2 );
}

void NEURAL_NETWORK::Store_Sensor_Data(int timeStep) {

	// Store sensor data that will be employed for user modeling.
	// Only store light and compass sensor values.

	for (int j=0; j<numSensors; j++) {

		if (	(sensorTypes->Get(0,j) == SENSOR_TYPE_LIGHT) ||
			(sensorTypes->Get(0,j) == SENSOR_TYPE_COMPASS) ) {

			double sensorValue = sensorValues->vals[j];

			sensorTimeSeries->vals[timeStep*sensorTimeSeries->width+j] = sensorValue;
		}
	}
}

#endif
