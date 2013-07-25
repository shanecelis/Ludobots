#ifndef _TAU_OPTIMIZER_H
#define _TAU_OPTIMIZER_H

#include "neuralNetwork.h"
#include "userModel.h"

class TAU_OPTIMIZER {

private:
	USER_MODEL *model;

	double 	    modelError;

public:
	TAU_OPTIMIZER(void);
	TAU_OPTIMIZER(ifstream *inFile);
	~TAU_OPTIMIZER(void);
	void   Optimize(int numControllers, NEURAL_NETWORK **controllers);
	void   Print_Predictions(USER_MODEL *model, int numControllers, NEURAL_NETWORK **controllers);
	int    Ready_To_Predict(void);
	void   Save(ofstream *outFile);
	double Score_Predict( NEURAL_NETWORK *neuralNetwork);
	double Score_Predict1(NEURAL_NETWORK *neuralNetwork);
        double Score_Predict2(NEURAL_NETWORK *neuralNetwork);
	void   User_Models_Reset(int numControllers, NEURAL_NETWORK **controllers);
};

#endif
