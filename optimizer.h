
#ifndef _OPTIMIZER_H
#define _OPTIMIZER_H

#include "matrix.h"
#include "neuralNetwork.h"

class OPTIMIZER {

public:
	int 		timer;
	int 		evaluationPeriod;

	int 		numSensors;
	int 		numMotors;

	double		mutationProbability;

	long 		nextGenomeID;

	NEURAL_NETWORK **genomes;

	NEURAL_NETWORK *genomeUnderEvaluation;

	int		generation;

public:
	OPTIMIZER(int numberOfSensors, int numberOfMotors);
	OPTIMIZER(ifstream *inFile);
	~OPTIMIZER(void);
	void    	EvaluationPeriod_Decrease(void);
	void    	EvaluationPeriod_Increase(void);
	void		Fitness_Receive(NEURAL_NETWORK *userFavorite, double fitness);
	void		Fitness_Sensor_Data_Receive(NEURAL_NETWORK *userFavorite, double fitness, MATRIX *timeSeries);
	void		Fitness_Sensor_Data_Score_Receive(NEURAL_NETWORK *userFavorite, double fitness, MATRIX *timeSeries, double score);
	void		Genome_Discard_Being_Evaluated(void);
	NEURAL_NETWORK *Genome_Get(int i);
	NEURAL_NETWORK *Genome_Get_Best(void);
        NEURAL_NETWORK *Genome_Get_Best_But_Not(NEURAL_NETWORK *other);
	NEURAL_NETWORK *Genome_Get_Best_But_Not(int numControllers, NEURAL_NETWORK **controllers);
	NEURAL_NETWORK *Genome_Get_Curr_To_Evaluate(void);
	NEURAL_NETWORK *Genome_Get_First(void);
	NEURAL_NETWORK *Genome_Get_Most_Different(int numControllers, NEURAL_NETWORK **controllers);
        NEURAL_NETWORK *Genome_Get_Most_Different_But_Not(NEURAL_NETWORK *thisOne, int numControllers, NEURAL_NETWORK **controllers);
	NEURAL_NETWORK *Genome_Get_Next_To_Evaluate(NEURAL_NETWORK *userFavorite);
	NEURAL_NETWORK *Genome_Get_Random(void);
	NEURAL_NETWORK *Genome_Get_Random_But_Not(NEURAL_NETWORK *other);
	NEURAL_NETWORK *Genome_Get_Random_But_Not(int numControllers, NEURAL_NETWORK **controllers);
	NEURAL_NETWORK *Genome_Get_Second(void);
	void		Genome_Put_At_End(NEURAL_NETWORK *other);
	int		Genomes_Num_Of_Evaluated(void);
	void		Load(ifstream *inFile);
	void    	MutationProbability_Decrease(void);
	void    	MutationProbability_Increase(void);
	void    	Print(void);
	void		Reset(void);
	void		Reset_Genomes(void);
	void		Save(ofstream *outFile, int saveTimeSeries);
	void		Score_Receive(double score);
	void		Scores_Reset(void);
	void		Sensor_Data_Receive(MATRIX *timeSeries);
	int		Time_Elapsed(void);
	void		Timer_Reset(void);
	void		Timer_Update(void);

private:
	void    	Destroy(void);
	int		FlipCoin(void);
	void		Generation_Create_Next(NEURAL_NETWORK *userFavorite);
	void		Genome_Copy(int genomeIndex, int parentID);
	void    	Genome_Create_Random(int genomeIndex);
	void    	Genome_Destroy(int genomeIndex);
	int  		Genome_Evaluated(int genomeIndex);
	NEURAL_NETWORK *Genome_Find_Next_Not_Evaluated(void);
	void 		Genome_Load(int genomeIndex, ifstream *inFile);
	void		Genome_Print(int genomeIndex);
	int		Genomes_All_Evaluated(void);
	void		Genomes_Create(void);
	void		Genomes_Delete_Dominated(void);
	void		Genomes_Destroy(void);
	void		Genomes_Fill_Empty_Slots(void);
        void            Genomes_Find_Pareto_Front(void);
	void		Genomes_Increase_Age(void);
	void		Genomes_Inject_Random_Genome(void);
	void		Genomes_Inject_User_Favorite(NEURAL_NETWORK *userFavorite);
	void		Genomes_Load(ifstream *inFile);
	void    	Genomes_Print(void);
	void 		Genomes_Save(ofstream *outFile, int saveTimeSeries);
	void		Genomes_Sort(void);
	void		Genomes_Swap(int firstIndex, int secondIndex);
	void		Initialize(void);
	double		Rand(double min, double max);
	int     	RandInt(int min, int max);
};

#endif
