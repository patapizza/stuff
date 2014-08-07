#include "../lib/VRPlib.h"
#include "../lib/LSBase.h"
#include "../lib/Scenario.h"
#include <algorithm>    // std::swap, std::max

#ifndef LSProgram_H
#define	LSProgram_H

using namespace CBLS;
using namespace std;


/*
	LSProgram specific implementation
	- acceptanceCriterion()
	- terminationCondition()
*/

// Basic local search
template <class S> 
class LSProgramBasic : public LSProgram<S> {
protected:
	int diversification;
public:
	LSProgramBasic(S &initialSolution, int (*endIter)(S &, S &), int max_iter, int max_distance, int diversification) : 
	LSProgram<S>(initialSolution, endIter) {
		this->max_iter = max_iter;
		this->max_distance = max_distance;
		this->diversification = diversification;
	}

	bool acceptanceCriterion(double evalCandidate, double evalCurrent) {
		if (evalCandidate < evalCurrent) 
			return true;
		else return (rand() % 100 + 1 <= diversification);	// % diversification
	}
	bool terminationCondition() {
		return (this->iter >= this->max_iter);		// "this->" needed to access parent protected variables
	}
	virtual double computeSolutionEval(S& solution, double *cost) {
		//solution.routeChange();
		*cost = this->computeSolutionCost(solution);
		return *cost + 10000*solution.getViolations();
	}
	/*
	virtual double computeSolutionCost(S& solution) {
		return 10000000 + solution.getCost() - 10000*solution.getProbaMassRequestsPlanned();	// NO NEGATIVE COST !!!!! otherwise problems with S.A.
	}
	*/
};

// Basic local search, extended with dynamic soft constraint weights adjustments
template <class S> 
class LSProgramBasicDynamicWeights : public LSProgramBasic<S> {
private:	
	int nc;				// number of soft constraints in S
	float *c_weight;	// soft constraints respective weights (indiced from 1 to nc)
	float delta;		// delta for weight adjustments
public:
	LSProgramBasicDynamicWeights(S &initialSolution, int (*endIter)(S &, S &), int max_iter, int max_distance, int diversification, float delta) : 
	LSProgramBasic<S>(initialSolution, endIter, max_iter, max_distance, diversification) {
		nc = initialSolution->nbConstraints();
		c_weight = new float [nc+1];
		for (int i=1; i<=nc; i++) c_weight[i] = 1.0;
		this->delta = delta;
	}
	virtual double computeSolutionEval(S& solution, double *cost) {
		*cost = this->computeSolutionCost(solution);
		float penalty = 0.0;
		updatePenaltyWeights(solution);
		for (int c=1; c<=nc; c++) 
			penalty += solution.getViolations(c) * c_weight[c];
		return *cost + penalty;
	}
	void updatePenaltyWeights(S& currentSolution) {
		for (int c=1; c<=nc; c++) {
			if (currentSolution.getViolations(c) > 0) 	// if constraint c is violated,
				//c_weight[c] *= (1+delta); 				// delta% increase of its penalty weight for the next evaluation
				c_weight[c] = min((float) 1000000.0, c_weight[c] * (1 + delta));
			else 										// otherwise
				//c_weight[c] /= (1+delta); 				// delta% decrease
				c_weight[c] = max((float) 0.000001, c_weight[c] / (1 + delta));
		}
	}

	void printWeights() {
		for (int c=1; c<=nc; c++)
			switch (c) {
				case CAPACITY_CONSTRAINT: cout << "c_weight[CAPACITY]: " << c_weight[c] << "\t\t"; break;
				case TW_CONSTRAINT: cout << "c_weight[TW]: " << c_weight[c] << "\t\t"; break;
			}
		cout << "\tdelta: " << delta << endl;
	}
};




/* Simulated Annealing */
// -------------------


inline void logGraphSA(int iter, float proba, float temp) {
	ofstream myfile;
	if(iter == 0) 
		myfile.open ("output_graph_SA.txt");
	else
		myfile.open ("output_graph_SA.txt", ios::app);
	myfile << iter << "\t" << proba << "\t" << temp << endl;
	myfile.close();
}

template <class S> 
class LSProgram_SimulatedAnnealing : public LSProgramBasic<S> {
protected:	
	float temp;
	float init_temp, real_init_temp;
	float min_temp, real_min_temp;
	float lindecrease;
	float cooling_factor;
	int nb_restarts = 0;
public:
	LSProgram_SimulatedAnnealing(S &initialSolution, int (*endIter)(S &, S &), int max_iter, int max_distance, float init_temp, float min_temp, float cooling_factor) : 
	LSProgramBasic<S>(initialSolution, endIter, max_iter, max_distance, 0) {
		this->init_temp = init_temp;
		temp = init_temp;
		this->min_temp = min_temp;
		//lindecrease = (init_temp-min_temp)/500;
		this->cooling_factor = cooling_factor;
		real_min_temp = min_temp;
		real_init_temp = init_temp;
	}
	virtual bool acceptanceCriterion(double evalCandidate, double evalCurrent) {
		if(evalCandidate == evalCurrent)
			return false;

		float probability = 0.0;
		if(temp > 0.0)
			probability = exp( -(1-evalCurrent/evalCandidate) / temp );	// relative probability
		//float probability = exp( -(eval_candidate-eval_curr) / temp );	// probability
		//float probability = exp( -(1-currentSolution.getCost()/candidateSolution.getCost()) / temp );
		//cout << "\ttemp: " << temp << "\t init_temp:" << init_temp << "\t proba: " << probability << "\t evalCurrent: " << evalCurrent << "\t evalCandidate: "<< evalCandidate<< endl << flush;
		updateTemp();
		//if (this->iter == 0)
			//logGraphSA(this->iter, min(probability, (float)1.0), temp);
		if (evalCandidate < evalCurrent || rand() % 100000 + 1 <= probability*100000 ) {	
			//if (probability < 1)
				//logGraphSA(this->iter, min(probability, (float)1.0), temp);
			return true;
		} 	
		else {
			return false;
		}
	}
	void updateTemp() {
		temp = max(min_temp, temp * cooling_factor); 	// exponenitial decrease
		//temp *= cooling_factor;
		//temp =  max(min_temp, temp - lindecrease);	// linear decrease
	}
	bool shouldWeRestart() {
		return temp <= min_temp;
	}
	void restart() {
		nb_restarts++;
		
		init_temp /= 2;
		min_temp /= 2;
		temp = init_temp;

		if (nb_restarts % 10 == 0) {
			//cout << "R" << flush;
			init_temp = real_init_temp;
			min_temp = real_min_temp;
		}
	}
};
/* Specialized Simulated Annealing, handling ScenarioPool evluation (for stochastic programming) */
template <class S, class ScenarioPoolType> 
class LSProgram_SimulatedAnnealing_ScenarioPool : public LSProgram_SimulatedAnnealing<S> {
private:
	ScenarioPoolType *sp;
public:
	LSProgram_SimulatedAnnealing_ScenarioPool(S &initialSolution, int (*endIter)(S &, S &), int max_iter, int max_distance, float init_temp, float min_temp, float cooling_factor, ScenarioPoolType *sp) : 
	LSProgram_SimulatedAnnealing<S>(initialSolution, endIter, max_iter, max_distance, init_temp, min_temp, cooling_factor) {
		this->sp = sp;
	}
	virtual double computeSolutionEval(S& solution, double *cost) {
		if(solution.getViolations() > 0) 
			return 10000*solution.getViolations();

		*cost = this->computeSolutionCost(solution);
		return *cost + 10000*solution.getViolations();	
	}
	virtual double computeSolutionCost(S& solution) {
		//return solution.getCost() + 10000*sp->evalSolution(solution);	
		return sp->evalSolution(solution); //- 100*solution.getNbRequestsPlanned();	
	}
};





template <class S> 
class LSProgram_SimulatedAnnealing_MinimizeRouteNumber : public LSProgram_SimulatedAnnealing<S>{
private:
public:
	LSProgram_SimulatedAnnealing_MinimizeRouteNumber(S &initialSolution, int (*endIter)(S &, S &), int max_iter, int max_distance, float init_temp, float min_temp, float cooling_factor) 
	: LSProgram_SimulatedAnnealing<S>(initialSolution, endIter, max_iter, max_distance, init_temp, min_temp, cooling_factor) {
	}
	double computeSolutionEval(S& solution, double *cost) {
		*cost = computeSolutionCost(solution);
		float eval = 0.0;
		int *routeN = solution.getpRouteN();

		// evaluation function 
		// [1] minimize the number of non-empty routes
		// [2] favor solutions containing routes with many customers and routes with fewer customers, instead of well-balanced routes 
		// 		idea from (Bent, Van Hentenryck), "A two-stage hybrid algorithm for the VRPTW"
		
		for (int r=1; r<NVeh+1; r++) {
			eval -= routeN[r]^2;			// [2]
		}
		return *cost + 100000*solution.getViolations() + eval;
	}

	double computeSolutionCost(S& solution) {
		return solution.getCost() + 10000*solution.getNbVehicle();	// [1]
	}
};



#endif