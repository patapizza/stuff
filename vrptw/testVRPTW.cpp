#include <iostream>
#include <fstream>	// file I/O
#include <cstdlib>	// rand
#include <ctime>	// time,		 clock_t, clock, CLOCKS_PER_SEC */
#include <cmath>	// sqrt
#include <cstring>	// memcpy
#include <string> 	// stoi
#include <limits>	// numeric_limits
#include <iomanip>      // std::setw

#include "../lib/VRPlib.h"
#include "../lib/LSProgram.h"
#include "../lib/LSBase.h"
#include "../lib/tools.h"

using namespace CBLS;
using namespace std;

int lastReqTime;	// for readInstanceFilesVRPTW.o

extern int 	*number;	// number of the customer (in the test instance)
extern int NVeh;
extern int N;

extern int *verticesShuffle;

bool verbose = true;
char *program_name;
char *instance_file;
int nb_iter;
long t0;

void usage() {
	cerr << "Usage is " << program_name << " nb_iter instance_file \n";
	cerr << "Example: \t./testVRPTW 1000000 ../data/cordeau/vrptw/old/c208\n";
	cerr << "\t\t./testVRPTW 1000000 ../data/cordeau/vrptw/old/c208\n";
	exit (8);
}

int f_static(SolutionVRPTW &bestSolution, SolutionVRPTW &currentSolution) {
	static int it = 0, progression = 0;
	int r = NONE;
	if((++it % (int)floor(nb_iter / 100)) == 0) {
		cout.setf(std::ios::fixed);
		cout.precision(2);
		//if(verbose) cout << "Computation time: " << setw(7) << (float)(clock() - t0) / CLOCKS_PER_SEC << "sec\tStatus: " << ++progression << "%" << endl << flush;
		if(verbose) cout << ++progression << "% " << flush;
	}

	return false;
}


int main(int argc, char *argv[]) {
	program_name = argv[0];
	if (argc < 3) usage();
	nb_iter = stoi(argv[1]);
	instance_file = argv[2];

	t0 = clock();
	srand(time(NULL));
	cout << "Reading instance file..." << endl << flush;
	//readInstanceFileCordeauLaporteVRPTWold(instance_file); 
	readInstanceFileBentVanHentenryckDynVRPTW_Static(instance_file); 

	SolutionVRPTW s;
	while (s.activateOneMandatoryRequest(-1)) {}	
	cout << "Generating initial VRPTW solution ..." << flush;
	s.generateInitialSolution(); 
	cout << "Done.\n" << flush; 
	cout << "Initial solution: " << endl << s.toString() << endl << flush;
	s.checkSolutionConsistency();
	//s.printVertices();

	//LSProgramBasic<SolutionVRPTW> p(s, &f_static, nb_iter, 2500, 1); // 1% diversification
	//LSProgramBasicDynamicWeights<SolutionVRPTW> p(s, &f_static, nb_iter, 2500, 1, 0.0001); // 1% diversification, delta=0.0001
	LSProgram_SimulatedAnnealing<SolutionVRPTW> p(s, &f_static, nb_iter, 2500, 100.0, 0.1, 0.98);	// init temp:2.0, min temp: 0.01, cooling factor 0.99
	//LSProgram_SimulatedAnnealing_MinimizeRouteNumber<SolutionVRPTW> p(s, &f_static, nb_iter, 1000, 100.0, 1.0, 0.95);
	cout << "Running LS program" << endl;
	p.run(); 
	cout << endl << "Best solution found: \n" << s.toString() << endl;
	//p.printWeights();
	s.checkSolutionConsistency();

	cout << "seconds: " << (float)(clock() - t0) / CLOCKS_PER_SEC << endl;
	
}
