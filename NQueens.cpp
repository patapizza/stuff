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
public:
	SolutionNQueens(int n) {
        this->n = n;
        this->board = new int*[n];
        this->queens = new int[n];
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
        memcpy(queens, old.queens, n * sizeof(int));
        for (int i = 0; i < n; i++) {
            board[i] = new int[n];
            memcpy(board[i], old.board[i], n * sizeof(int));
        }
    }

    SolutionNQueens &operator = (const SolutionNQueens &sol) {
        memcpy(queens, sol.queens, n * sizeof(int));
        for (int i = 0; i < n; i++)
            memcpy(board[i], sol.board[i], n * sizeof(int));
        return *this;
    }

    ~SolutionNQueens() {
        delete [] board;
        delete [] queens;
    }

    friend std::ostream &operator << (std::ostream &out, SolutionNQueens &s);

	void generateInitialSolution() {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++)
                board[i][j] = 0;
            queens[i] = randmax(n);
            board[i][queens[i]] = 1;
        }
	}

	void shakeSolution() {
        int x = randmax(n);
        int y = randmax(n);
        board[x][queens[x]] = 0;
        board[x][y] = 1;
        queens[x] = y;
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
            for (int j = i + 1; j < n; j++)
                if (queens[i] == queens[j] || j - i == abs(queens[i] - queens[j]))
                    ++violations;
        return violations;
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

template <class S> class LSProgramNQueens : public LSProgram<S> {
public:
	LSProgramNQueens(S *initialSolution) : LSProgram<S>(initialSolution) {
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
