#ifndef _USER_MODEL_CPP
#define _USER_MODEL_CPP

#include "stdlib.h"
#include "stdio.h"
#include "math.h"

#include "userModel.h"

extern double TAU_NO_SCORE;
extern int    TAU_NUM_NEURONS;
extern int    STARTING_EVALUATION_TIME;
extern int    ALG_VARIANT_PREFS_ONLY;
extern int    TAU_NUM_SENSOR_ROWS;
extern int    TAU_BACK_PROP_TRAINING_ITERATIONS;

USER_MODEL::USER_MODEL(int numS) {

	numSensors = numS;

	int *layerSizes = new int[3];

	// All the sensors plus a bias input node.	
	layerSizes[0] = numSensors+1;

	layerSizes[1] = 3;
	layerSizes[2] = 1;

	ANN = new CBackProp(3,layerSizes,0.1,0.1);

	delete layerSizes;
	layerSizes = NULL;
}

USER_MODEL::USER_MODEL(ifstream *inFile) {

        (*inFile) >> numSensors;

	int isANN;

	(*inFile) >> isANN;

	if ( isANN )
		ANN = new CBackProp(inFile);
	else
		ANN = NULL;
}

USER_MODEL::~USER_MODEL(void) {

	if ( ANN ) {

		delete ANN;
		ANN = NULL;
	}
}

double USER_MODEL::Evaluate(int numControllers, NEURAL_NETWORK **controllers) {

	double score, scorePrediction;

	double totalError = 0.0;

	double *in = new double[numSensors+1];

	double *target = new double[1];

	for (int j=0;   j<TAU_BACK_PROP_TRAINING_ITERATIONS; j++) {

		for (int i=0;	i<numControllers;	i++) {

			MATRIX *sensorTimeSeries = controllers[i]->sensorTimeSeries;

			int sensorRow;

			score = controllers[i]->Score_Get();

			for (int m=0; m<numSensors; m++)

				in[m] = sensorTimeSeries->Get(int(double(STARTING_EVALUATION_TIME)/2.0),m);

			in[numSensors] = 1.0; // Bias node

			target[0] = score;

			ANN->bpgt(in,target);

			scorePrediction = ANN->Out(0);

			totalError = totalError + fabs(score-scorePrediction);

			sensorTimeSeries = NULL;
		}
	}

	delete in;
	delete target;

	return( totalError );
}

double USER_MODEL::Predict(MATRIX *sensorTimeSeries) {

	int i;

	double *in;

	// All the sensor values plus a bias input node.
	in = new double[numSensors+1];

	for (int m=0; m<numSensors; m++)

		in[m] = sensorTimeSeries->Get(int(double(STARTING_EVALUATION_TIME)/2.0),m);

	in[numSensors] = 1.0; // Bias neuron

        ANN->ffwd(in);

	delete in;

	return( pow(ANN->Out(0),0.3) );
}

void USER_MODEL::Save(ofstream *outFile) {

	(*outFile) << numSensors << "\n";

	if ( ANN ) {
		(*outFile) << "1\n";
		ANN->Save(outFile);
	}
	else
		(*outFile) << "0\n";
}

#endif
