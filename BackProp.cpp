#include "BackProp.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>


//	initializes and allocates memory on heap
CBackProp::CBackProp(int nl,int *sz,double b,double a):beta(b),alpha(a)
{

	numl=nl;
	lsize=new int[numl];

	int i;
	for(i=0;i<numl;i++){
		lsize[i]=sz[i];
	}

	out = new double*[numl];

	for( i=0;i<numl;i++){
		out[i]=new double[lsize[i]];
	}

	delta = new double*[numl];

	for(i=1;i<numl;i++){
		delta[i]=new double[lsize[i]];
	}

	weight = new double**[numl];

	for(i=1;i<numl;i++){
		weight[i]=new double*[lsize[i]];
	}
	for(i=1;i<numl;i++){
		for(int j=0;j<lsize[i];j++){
			weight[i][j]=new double[lsize[i-1]+1];
		}
	}

	prevDwt = new double**[numl];

	for(i=1;i<numl;i++){
		prevDwt[i]=new double*[lsize[i]];

	}
	for(i=1;i<numl;i++){
		for(int j=0;j<lsize[i];j++){
			prevDwt[i][j]=new double[lsize[i-1]+1];
		}
	}

	srand((unsigned)(time(NULL)));
	for(i=1;i<numl;i++)
		for(int j=0;j<lsize[i];j++)
			for(int k=0;k<lsize[i-1]+1;k++)
				weight[i][j][k]=(double)(rand())/(RAND_MAX/2) - 1;//32767

	for(i=1;i<numl;i++)
		for(int j=0;j<lsize[i];j++)
			for(int k=0;k<lsize[i-1]+1;k++)
				prevDwt[i][j][k]=(double)0.0;

}

CBackProp::CBackProp(ifstream *inFile) {

        (*inFile) >> alpha;
        (*inFile) >> beta;
        (*inFile) >> numl;

        lsize=new int[numl];

        for (int i=0;i<numl;i++)
                (*inFile) >> lsize[i];

        out = new double*[numl];
        for(int i=0;i<numl;i++)
                out[i]=new double[lsize[i]];

        for(int i=0;i<numl;i++)
                for(int j=0;j<lsize[i];j++)
                        (*inFile) >> out[i][j];


        delta = new double*[numl];
        for(int i=1;i<numl;i++)
                delta[i]=new double[lsize[i]];

        for(int i=1;i<numl;i++)
                for(int j=0;j<lsize[i];j++)
                        (*inFile) >> delta[i][j];


        weight = new double**[numl];
        for(int i=1;i<numl;i++)
                weight[i]=new double*[lsize[i]];
        for(int i=1;i<numl;i++){
                for(int j=0;j<lsize[i];j++){
                        weight[i][j]=new double[lsize[i-1]+1];
                }
        }
 
        for(int i=1;i<numl;i++)
                for(int j=0;j<lsize[i];j++)
                        for(int k=0;k<lsize[i-1]+1;k++)
                                (*inFile) >> weight[i][j][k];

 
        prevDwt = new double**[numl];
        for(int i=1;i<numl;i++)
                prevDwt[i]=new double*[lsize[i]];
        for(int i=1;i<numl;i++)
                for(int j=0;j<lsize[i];j++)
                        prevDwt[i][j]=new double[lsize[i-1]+1];

        for(int i=1;i<numl;i++)
                for(int j=0;j<lsize[i];j++)
                        for(int k=0;k<lsize[i-1]+1;k++)
                                (*inFile) >> prevDwt[i][j][k];
}

CBackProp::~CBackProp()
{
	int i;

	//	free out
	for(i=0;i<numl;i++)
		delete[] out[i];
	delete[] out;

	//	free delta
	for(i=1;i<numl;i++)
		delete[] delta[i];
	delete[] delta;

	//	free weight
	for(i=1;i<numl;i++)
		for(int j=0;j<lsize[i];j++)
			delete[] weight[i][j];
	for(i=1;i<numl;i++)
		delete[] weight[i];
	delete[] weight;

	//	free prevDwt
	for(i=1;i<numl;i++)
		for(int j=0;j<lsize[i];j++)
			delete[] prevDwt[i][j];
	for(i=1;i<numl;i++)
		delete[] prevDwt[i];
	delete[] prevDwt;

	//	free layer info
	delete[] lsize;
}

//	sigmoid function
double CBackProp::sigmoid(double in)
{
		return (double)(1/(1+exp(-in)));
}

//	mean square error
double CBackProp::mse(double *tgt) const
{
	double mse=0;
	for(int i=0;i<lsize[numl-1];i++){
		mse+=(tgt[i]-out[numl-1][i])*(tgt[i]-out[numl-1][i]);
	}
	return mse/2;
}


//	returns i'th output of the net
double CBackProp::Out(int i) const
{
	return out[numl-1][i];
}

// feed forward one set of input
void CBackProp::ffwd(double *in)
{
	double sum;

	int i;

	//	assign content to input layer
	for(i=0;i<lsize[0];i++)
		out[0][i]=in[i];  // output_from_neuron(i,j) Jth neuron in Ith Layer

	//	assign output(activation) value 
	//	to each neuron usng sigmoid func
	for(i=1;i<numl;i++){				// For each layer
		for(int j=0;j<lsize[i];j++){		// For each neuron in current layer
			sum=0.0;
			for(int k=0;k<lsize[i-1];k++){		// For input from each neuron in preceeding layer
				sum+= out[i-1][k]*weight[i][j][k];	// Apply weight to inputs and add to sum
			}
			sum+=weight[i][j][lsize[i-1]];		// Apply bias
			out[i][j]=sigmoid(sum);				// Apply sigmoid function
		}
	}
}


//	backpropogate errors from output
//	layer uptill the first hidden layer
void CBackProp::bpgt(double *in,double *tgt)
{
	double sum;

	//	update output values for each neuron
	ffwd(in);

	int i;

	//	find delta for output layer
	for(i=0;i<lsize[numl-1];i++){
		delta[numl-1][i]=out[numl-1][i]*
		(1-out[numl-1][i])*(tgt[i]-out[numl-1][i]);
	}

	//	find delta for hidden layers	
	for(i=numl-2;i>0;i--){
		for(int j=0;j<lsize[i];j++){
			sum=0.0;
			for(int k=0;k<lsize[i+1];k++){
				sum+=delta[i+1][k]*weight[i+1][k][j];
			}
			delta[i][j]=out[i][j]*(1-out[i][j])*sum;
		}
	}

	//	apply momentum ( does nothing if alpha=0 )
	for(i=1;i<numl;i++){
		for(int j=0;j<lsize[i];j++){
			for(int k=0;k<lsize[i-1];k++){
				weight[i][j][k]+=alpha*prevDwt[i][j][k];
			}
			weight[i][j][lsize[i-1]]+=alpha*prevDwt[i][j][lsize[i-1]];
		}
	}

	//	adjust weights usng steepest descent	
	for(i=1;i<numl;i++){
		for(int j=0;j<lsize[i];j++){
			for(int k=0;k<lsize[i-1];k++){
				prevDwt[i][j][k]=beta*delta[i][j]*out[i-1][k];
				weight[i][j][k]+=prevDwt[i][j][k];
			}
			prevDwt[i][j][lsize[i-1]]=beta*delta[i][j];
			weight[i][j][lsize[i-1]]+=prevDwt[i][j][lsize[i-1]];
		}
	}
}

void CBackProp::Save(ofstream *outFile) {

	(*outFile) << alpha << "\n";
	(*outFile) << beta  << "\n";
	(*outFile) << numl  << "\n";

	for (int i=0;i<numl;i++)
		(*outFile) << lsize[i] << "\n";

        for(int i=0;i<numl;i++)
                for(int j=0;j<lsize[i];j++)
			(*outFile) << out[i][j] << "\n";

        for(int i=1;i<numl;i++)
                for(int j=0;j<lsize[i];j++)
                        (*outFile) << delta[i][j] << "\n";

        for(int i=1;i<numl;i++)
                for(int j=0;j<lsize[i];j++)
                        for(int k=0;k<lsize[i-1]+1;k++)
                                (*outFile) << weight[i][j][k] << "\n";

        for(int i=1;i<numl;i++)
                for(int j=0;j<lsize[i];j++)
                        for(int k=0;k<lsize[i-1]+1;k++)
                                (*outFile) << prevDwt[i][j][k] << "\n";
}
