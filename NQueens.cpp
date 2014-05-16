#include <iostream>
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cstring>	// memcpy

#include "LSBase.h"

using namespace CBLS;
using namespace std;

// Returns an unskewed random number between 0 and max exclusive.
int randmax(int max) {
    int div = RAND_MAX / max;
    int val;
    do {
        val = rand() / div;
    } while (val > max);
    return val;
}

class SolutionNQueens: public Solution {
    int n;
    int **board;
    int *queens;
    int *violations;
public:
	SolutionNQueens(int n) {
        this->n = n;
        this->board = new int*[n];
        this->queens = new int[n];
        this->violations = new int[n];
        for (int i = 0; i < n; i++)
            this->board[i] = new int[n];
		cout << "Generating initial solution... ";
		generateInitialSolution();
		cout << " ...done!\n"; 
	}

    SolutionNQueens(const SolutionNQueens &old) {
        n = old.n;
        board = new int*[n];
        queens = new int[n];
        violations = new int[n];
        memcpy(queens, old.queens, n * sizeof(int));
        memcpy(violations, old.violations, n * sizeof(int));
        for (int i = 0; i < n; i++) {
            board[i] = new int[n];
            memcpy(board[i], old.board[i], n * sizeof(int));
        }
    }

    SolutionNQueens &operator = (const SolutionNQueens &sol) {
        memcpy(queens, sol.queens, n * sizeof(int));
        memcpy(violations, sol.violations, n * sizeof(int));
        for (int i = 0; i < n; i++)
            memcpy(board[i], sol.board[i], n * sizeof(int));
        return *this;
    }

    ~SolutionNQueens() {
        delete [] board;
        delete [] queens;
        delete [] violations;
    }

    friend std::ostream &operator << (std::ostream &out, SolutionNQueens &s);

	void generateInitialSolution() {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++)
                board[i][j] = 0;
            queens[i] = randmax(n);
            board[i][queens[i]] = 1;
        }
        countViolations();
    }

    void countViolations() {
        for (int i = 0; i < n; i++) {
            violations[i] = 0;
            for (int j = i + 1; j < n; j++)
                if (queens[i] == queens[j] || j - i == abs(queens[i] - queens[j])) {
                    ++violations[i];
                    ++violations[j];
                }
        }
	}

	void shakeSolution() {
        int max = 0;
        int argmax = -1;
        for (int i = 0; i < n; i++)
            if (violations[i] > max) {
                max = violations[i];
                argmax = i;
            }
        int x = argmax;
        int y;
        do {
            y = randmax(n);
        } while (y == queens[x]);
        board[x][queens[x]] = 0;
        board[x][y] = 1;
        queens[x] = y;
        countViolations();
	}

	void restore() {
        // to implement
	}

	float getCost() {
        return 0.0;
	}

	float getViolations() {
        float violations = 0.0;
        for (int i = 0; i < n; i++)
            violations += this->violations[i];
        return violations;
	}

    vector<SolutionNQueens> fullNeighborhood() {
        vector<SolutionNQueens> solutions;
        return solutions;
    }
};

inline std::ostream &operator << (std::ostream &out, SolutionNQueens &s) {
    for (int i = 0; i < s.n; i++) {
        for (int j = 0; j < s.n; j++)
            out << s.board[i][j] << " ";
        out << endl;
    }
    out << "Violations: " << s.getViolations() << endl;
    return out;
}

//template <class S> class LSProgramNQueens : public LSProgram<S> {
template <class S> class LSProgramNQueens : public LSTabu<S> {
public:
	//LSProgramNQueens(S *initialSolution) : LSProgram<S>(initialSolution) {
    LSProgramNQueens(S *initialSolution) : LSTabu<S>(initialSolution) {
    }

	bool acceptanceCriterion(S &candidateSolution, S &incumbentSolution) {
        return candidateSolution.getEval() < incumbentSolution.getEval();
	}

	bool terminationCondition() {
        return this->bestSolution->getViolations() == 0 || this->iter == 1000000;
	}
};



int main() {
    srand(time(NULL));
	SolutionNQueens s(8);
	cout << "Initial solution: " << endl << s << endl;
	LSProgramNQueens<SolutionNQueens> p(&s);
	cout << "Running LS program..." << endl;
	p.run();
	cout << endl << "Best solution found: \n" << s << endl;
}
