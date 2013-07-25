#ifndef _TAU_H
#define _TAU_H

#include "neuralNetwork.h"
#include "optimizer.h"
#include "tauOptimizer.h"

class TAU {

public:
	int		timer;
	int              numControllers;
	NEURAL_NETWORK **controllers;
	MATRIX          *preferences;
	int		 firstControllerIndex;
	int		 secondControllerIndex;
	TAU_OPTIMIZER   *tauOptimizer;
	int		 nextControllerForTraining;
	double		 scoreMin;
	double		 scoreMax;

public:
	TAU(void);
	TAU(ifstream *inFile);
	~TAU(void);
	int		All_Required_Preferences_Supplied(void);
	void            Controller_First_Preferred(void);
	void		Controller_First_Store_Sensor_Data(MATRIX *sensorData);
	NEURAL_NETWORK *Controller_Get_Best(void);
	NEURAL_NETWORK *Controller_Pair_Get_First(void);
	NEURAL_NETWORK *Controller_Pair_Get_Second(void);
	void            Controller_Second_Preferred(void);
	void		Controller_Second_Store_Sensor_Data(MATRIX *sensorData);
	void		Controllers_Load_Pair(ifstream *inFile);
	void		Controllers_Save_Pair(OPTIMIZER *optimizer, ofstream *outFile);
        void            Controllers_Select_From_Optimizer(OPTIMIZER *optimizer);
	void		Optimize(void);
	void            Print(void);
	int		Ready_To_Predict(void);
	void		Save(ofstream *outFile);
	double		Score_Predict(NEURAL_NETWORK *controller);
	void		Store_Pref(int firstID, int secondID, int pref);
	void		User_Models_Reset(void);

private:
        void            Controller_Store(NEURAL_NETWORK *newController);
	void		Controller_Store_Sensor_Data(int controllerIndex, MATRIX *sensorData);
        int             Controllers_Available_For_Optimization(void);
	void            Controllers_Expand(void);
	void            Controllers_Initialize(void);
	int		Controllers_Num_Needed_From_Optimizer(void);
	void            Controllers_Print(void);
	void		Controllers_Select_One_From_TAU_One_From_Optimizer(OPTIMIZER *optimizer);
	void		Controllers_Select_Two_From_Optimizer(OPTIMIZER *optimizer);
	void		Controllers_Select_Two_From_TAU(void);
	int		Find_Index(int ID);
	int		Num_Prefs(void);
	void		Preferences_Expand(void);
	void		Preferences_Initialize(void);
	void		Preferences_Print(void);
	int		Ready_For_Optimization(void);
        double		Scale(	double value, double min1, double max1,
                             	double min2, double max2);
	void		Scores_Print(void);
	void		Scores_Update(void);
	void		Storage_Expand(void);
	void		Storage_Initialize(void);
};

#endif
