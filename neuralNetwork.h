// A neural network controls transforms the incoming sensor signals
// into outgoing motor commands.

#ifndef _NEURAL_NETWORK_H
#define _NEURAL_NETWORK_H

#include "matrix.h"

class NEURAL_NETWORK {

public:

	MATRIX  *weights;
	MATRIX  *taus;
	MATRIX  *neuronValues;
	MATRIX  *sensorValues;
	MATRIX  *sensorTypes;
	MATRIX  *temp;
	int	numSensors;
	int     numMotors;
	double  fitness;
	int	dominated;
        int     age;
        MATRIX *sensorTimeSeries;
	int	evaluated;
	int     ID;

private:
	double  score;

public:
	NEURAL_NETWORK(	int myID, int nS, int nM , int myAge );
	NEURAL_NETWORK( NEURAL_NETWORK *other );
	NEURAL_NETWORK(ifstream *inFile);
	~NEURAL_NETWORK();
	int    Age_Get(void);
	int    Evaluated(void);
	int    Fitness_Equal_To(double fit);
	double Fitness_Get(void);
	void   Fitness_Sensor_Data_Set(double fit, MATRIX *timeSeries);
	void   Fitness_Sensor_Data_Score_Set(double fit, MATRIX *timeSeries, double sc);
	void   Fitness_Set(double fit);
	int    Fitness_Worse_Than(double fit);
	double Get_Motor_Neuron_Value(int motorIndex);
	MATRIX *Get_Sensor_Data(void);
	int    ID_Get(void);
	void   ID_Set(int myID);
	int    Is_Dominated_By(NEURAL_NETWORK *other);
	int    Is_Inferior_To(NEURAL_NETWORK *other);
	int    Is_The_Same_As(NEURAL_NETWORK *other);
	double Min_Distance_To(int numControllers, NEURAL_NETWORK **controllers);
	void   Mutate(void);
	void   Print(void);
	void   Print_Sensor_Data(void);
	void   Record_Sensor_Data(int evaluationPeriod);
	void   Reset(void);
	void   Save(ofstream *outFile, int saveTimeSeries);
	void   Save_ButNotSensorData(ofstream *outFile);
	int    Score_Available(void);
	double Score_Get(void);
	void   Score_Reset(void);
	void   Score_Set(double val);
	void   Sensor_Set(int sensorIndex, double sensorValue, double sensorType);
	void   Store_Sensor_Data(MATRIX *externalSensorData);
	void   Update(int timeStep);
	void   Writeout(ofstream *outFile);

private:
	void	Initialize(void);
	void	Mutate_Taus(void);
	void    Mutate_Weights(void);
	double  Rand(double min, double max);
	double	RandN(void);
	double  Scale(double value, double min1, double max1,
                                    double min2, double max2);
	void    Store_Sensor_Data(int timeStep);
};

#endif
