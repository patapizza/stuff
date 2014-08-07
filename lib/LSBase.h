#include <iostream>
#include "NM.h"


#ifndef LSBase_H
#define	LSBase_H

namespace CBLS {
/*
	template <class T> class var {
	private:
		T *val;
		float activity;
	public:
		explicit var(T val = NULL) {		// Initializes the variable
			this->val = val;
			activity = 1.0;
		}
		// ~var()								
		//		Use automatically generated stuff
		// var(const var& old_var)				
		//		Use automatically generated stuff
		// var operator = (const var& old_var)	
		//		Use automatically generated operator for assignment from another var object
		var operator = (T new_val) {		// assignment operation
			val = new_val;
		}

		std::ostream& operator << (const std::ostream& out_file) {
			out_file << val;
			return (out_file);
		}

	};
*/

	/*
	class solution {
	public:
		// Main methods 
		virtual void generateInitialSolution() =0; 
		virtual float getCost() =0;						// returns the solution cost
		virtual int getViolations(int c = 0) =0;		// # of violations in soft constraint c (if any); c = 0 for all constraints

		// Communication methods
		virtual int nbConstraints() =0;					// # of soft constraints in the current problem (used by LSProgram extensions)
		virtual int* getpPrevious() =0;
		virtual int* getpNext() =0;
		virtual int* getpVehicle() =0;
		virtual void routeChange(int k = 0) = 0;
	};
	*/

inline void logSolutionView(string sol) {
	/*
	ofstream myfile;
	myfile.open ("bestSolution.txt");
	myfile << sol;
	myfile.close();
	*/
}

#define		NONE			0
#define		UPDATE_EVAL		1
#define		STOP_LS			5

template <class S> class LSProgram {
protected:
	S *bestSolution;
	int (*endIter)(S &, S &); 	// function to be called on the best solution after each iteration (e.g. checking some time limits, online requests, printing stuffs, ..)
	int iter;
	int max_iter;
	int distance;
	int max_distance;
public:
	LSProgram(S &initialSolution, int (*endIter)(S &, S &)) {
		this->bestSolution = &initialSolution;
		this->endIter = endIter;
	}
	virtual bool terminationCondition() =0; 		// decides whether the LS procedure should stop or not
	virtual bool acceptanceCriterion(double evalCandidate, double evalCurrent) =0;	
													// tells whether current solution must be accepted, according to the incumbent solution and their evaluations
	virtual double computeSolutionEval(S& solution, double *cost) =0;
	virtual double computeSolutionCost(S& solution) { 
		return solution.getCost();
	}
	virtual bool shouldWeRestart() {return distance >= 2500;}
	virtual void restart() {}		// informs the meta heuristic that a restart occured
	virtual void improvement() {} 	// informs the meta heuristic that a new improving solution has been found

	void run(){
		bool callingStop = false;
		iter = 0; 
		distance = 0;
		S currentSolution(*bestSolution);			// invokes the copy constructor
		S neighborSolution(*bestSolution);			// invokes the copy constructor
		double evalCurrent, evalNeighbor, evalBest, costCurrent, costNeigbor, costBest;

		evalCurrent = computeSolutionEval(currentSolution, &costCurrent);
		evalNeighbor = evalCurrent;	costNeigbor = costCurrent;
		evalBest = evalCurrent;	costBest = costCurrent;
		neighborhoodManager<S> nm(&neighborSolution);
		//neighborhoodManager_Adaptative<S> nm(&neighborSolution);
		logSolutionView(currentSolution.toString() + to_string(costBest));									// log stuffs
		while (!terminationCondition() && !callingStop) {
			neighborSolution = currentSolution;
			nm.shakeSolution();
			//cout << "viol best: " << bestSolution->getViolations() << "\tviol current: " << currentSolution.getViolations() << "\tviol neighbor: " << neighborSolution.getViolations() << endl;
			evalNeighbor = computeSolutionEval(neighborSolution, &costNeigbor);
			if(evalNeighbor < evalCurrent)
				nm.neighborAccepted();
			//distance++;
			/* MOVE OR NOT */
			if (acceptanceCriterion(evalNeighbor, evalCurrent)) {
				currentSolution = neighborSolution;
				evalCurrent = evalNeighbor;	costCurrent = costNeigbor;
				distance++;
				nm.intensify();
				//cout << "accepted by criterion" << endl << flush;

				if(currentSolution.getViolations() < bestSolution->getViolations()) {
					*bestSolution = currentSolution;
					evalBest = evalCurrent;	costBest = costCurrent;
					distance = 0;
					improvement();
					logSolutionView(currentSolution.toString() + to_string(costBest));								// log stuffs
				}

				if (currentSolution.getViolations() == 0 && costCurrent < costBest) {
					//std::cout << "!" << std::flush;
					*bestSolution = currentSolution; 
					evalBest = evalCurrent;	costBest = costCurrent;
					distance = 0;
					improvement();
					logSolutionView(currentSolution.toString() + to_string(costBest));								// log stuffs
				} 
			}
			else {
				nm.diversify();
			} 

			if(distance >= max_distance) {			// restart from best solution if gone too far away
			//if(shouldWeRestart()) {
				currentSolution = *bestSolution; 
				evalCurrent = evalBest;	costCurrent = costBest;
				distance = 0;
				nm.intensify();
				restart();	// informs the LS engine meta heuristic that a restart operated
			}

			iter++;
			int r = endIter(*bestSolution, currentSolution);
			switch (r) {
				case STOP_LS:
					callingStop = true;
					//break;	---> break hidden so that UPDATE_EVAL is also executed
				case UPDATE_EVAL:	// if a significant modification has been done to the solutions throught endIter, update the evaluations
					evalBest = computeSolutionEval(*bestSolution, &costBest);
					evalCurrent = computeSolutionEval(currentSolution, &costCurrent);
					logSolutionView(bestSolution->toString() + to_string(costBest));					// it may also be of worth to log it
					break;
				case NONE:
					break;
			}
		}
		//cout << endl;
		nm.printNeighborhoodScores();
	}

};

}




#define ASSERT(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message << std::endl; \
            std::exit(EXIT_FAILURE); \
        } \
    } while (false)

#endif



