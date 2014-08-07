#include <iostream>
#include <fstream>	// file I/O
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy

#include "../lib/VRPlib.h"
#include "../lib/LSProgram.h"
#include "../lib/LSBase.h"

using namespace CBLS;
using namespace std;


char *program_name;
char *instance_file;
int nb_iter;


void usage() {
	cerr << "Usage is " << program_name << " nb_iter instance_file \n";
	cerr << "ex: ./testVRP 100000 ../data/cordeau/vrp/old/p02" << endl;
	exit (8);
}

bool f_static(SolutionVRP &bestSolution, SolutionVRP &currentSolution) {
	return false;
}


int main(int argc, char *argv[]) {
	program_name = argv[0];
	if (argc < 3) usage();
	nb_iter = stoi(argv[1]);
	instance_file = argv[2];

	long t = time(NULL);
	srand(t);
	readInstanceFileCordeauLaporteVRPold(instance_file); 
	SolutionVRP s;
	while (s.activateOneRequest(-1)) {}
	cout << "Generating initial VRP solution ..." << flush;
	s.generateInitialSolution(); 
	cout << "Done.\n" << flush; 
	cout << "Initial solution: " << endl << s.toString() << endl << flush;

	//LSProgramBasic<SolutionVRP> p(&s, &f_static, nb_iter, 2500, 1); // 1% diversification
	//LSProgramBasicDynamicWeights<SolutionVRP> p(&s, &f_static, nb_iter, 2500, 1, 0.0001); // 1% diversification, delta=0.0001
	LSProgram_SimulatedAnnealing<SolutionVRP> p(s, &f_static, nb_iter, 5000, 100.0, 1.0, 0.95);	// init temp:2.0, min temp: 0.01, cooling factor 0.99
	//LSProgram_SimulatedAnnealing_MinimizeRouteNumber<SolutionVRP> p(&s, &f_static, nb_iter, 2500, 2.0, 0.01, 0.95);
	//LSProgram_SimulatedAnnealing_DynamicWeights<SolutionVRP> p(&s, &f_static, nb_iter, 2500, 10.0, 2.0, 0.95, 0.0001);

	cout << "Running LS program" << endl;
	p.run();
	cout << endl << "Terminated. Best solution found: \n" << s.toString() << endl;
	s.checkSolutionConsistency();
}
