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

	template <class S> class LSProgram {
	protected:
		S *bestSolution;
		int iter;
	public:
		LSProgram(S *initialSolution) {
			this->bestSolution = initialSolution;
		}
		virtual bool terminationCondition() =0; 		// decides whether the LS procedure should stop or not
		virtual bool acceptanceCriterion(S& candidateSolution, S& currentSolution) =0;	
														// tells whether current solution must be accepted, according to the incumbent solution
		virtual float computeSolutionEval(S& solution) { // can be overloaded in derived class, for metaheuristic management
			return solution.getCost() + solution.getViolations();
		}

		virtual void run(){
			iter = 0;
			S currentSolution(*bestSolution);			// invokes the copy constructor
			S neighborSolution(*bestSolution);			// invokes the copy constructor
			neighborhoodManager<S> nm(&neighborSolution);
			while (false == terminationCondition()) {
				neighborSolution = currentSolution;
				nm.shakeSolution();
				if (acceptanceCriterion(neighborSolution, currentSolution)) {
					currentSolution = neighborSolution;
					//std::cout << "*";
					if (currentSolution.getViolations() == 0 && currentSolution.getCost() < bestSolution->getCost()) {
						std::cout << "!" << std::flush;
						*bestSolution = currentSolution;
					} 
				}
				iter++;
			}
			if (bestSolution->getViolations() > 0)	// if no feasible solution found, return the current solution
				*bestSolution = currentSolution;
		}


	};

}

#endif

