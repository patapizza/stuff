#include <iostream>
#include <vector> // std::vector
#include <utility> // std::pair, std::make_pair

using namespace std;

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
	class Solution {
	public:
		virtual void generateInitialSolution() =0; 
		virtual void shakeSolution() =0;					// obtains a neighboring solution by disturbing the current one
		virtual void restore() =0; 						// restores the solution as it was before the last shaking
		virtual float getCost() =0;						// returns the solution cost
		virtual float getViolations() =0;				// returns the violation penalty ≥ 0 (if any); zero means feasible
		float getEval() {
			return (getCost() + getViolations());
		}
	};

	template <class S> class LSProgram {
	protected:
		S *bestSolution;
		int iter;
	public:
		LSProgram(S *initialSolution) {
			this->bestSolution = initialSolution;
		}
		virtual bool terminationCondition() =0; 		// decides whether the LS procedure should stop or not
		virtual bool acceptanceCriterion(S& candidateSolution, S& incumbentSolution) =0;	
														// tells whether current solution must be accepted, according to the incumbent solution
		void run(){
			iter = 0;
			S incumbentSolution(*bestSolution);			// invokes the copy constructor
			S neighborSolution(*bestSolution);			// invokes the copy constructor
			while (false == terminationCondition()) {
				neighborSolution = incumbentSolution;
				neighborSolution.shakeSolution();
				if (acceptanceCriterion(neighborSolution, incumbentSolution)) {
					//cout << "*";
					incumbentSolution = neighborSolution;
					if (neighborSolution.getEval() < bestSolution->getEval()) {
						//cout << "!";
						*bestSolution = incumbentSolution;
					} 
				}
				iter++;
			}
		}


	};

    template <class S> class LSTabu : LSProgram {
    private:
        vector<pair<int, S>> t;
        int tenure;
    public:
        LSTabu(S *initialSolution) : LSProgram(S *initialSolution) {
            LSTabu(initialSolution, 1000000, 2);
        }
        LSTabu(S *initialSolution, iter, tenure) {
            this->iter = iter;
            this->tenure = tenure;
        }

        bool terminationCondition() {
            return this->iter == 0;
        }

        bool acceptanceCriterion(S &candidateSolution, S &incumbentSolution) {
            return acceptanceCriterion(candidateSolution);
        }

        bool acceptanceCriterion(S &candidateSolution) {
            for (vector<pair<int, S>>::iterator it = t.begin(); it != t.end(); ++it)
                if (it->second == candidateSolution)
                    return false;
            return true;
        }

        void expire_features() {
            int i = -1;
            for (vector<pair<int, S>>::iterator it = t.begin(); it != t.end() && it->first - iter == tenure; ++it)
                ++i;
            t.erase(t.begin(), t.begin() + i);
        }

        void run() {
            S candidateSolution(*bestSolution);
            while (false == terminationCondition()) {
                vector<S> neighbors = candidateSolution.fullNeighborhood();
                vector<S> legal;
                for (vector<S>::iterator it = neighbors.begin(); it != neighbors.end(); ++it)
                    if (acceptanceCriterion(*it))
                        legal.push_back(*it);
                candidateSolution = candidateSelection(legal);
                if (candidateSolution.getEval() < bestSolution->getEval())
                    *bestSolution = candidateSolution;
                t.push_back(make_pair(this->iter, candidateSolution));
                expire_features();
                --(this->iter);
            }
        }
    };

}


