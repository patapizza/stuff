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
	cerr << "Example: ./testVRPTW 100000 ../data/cordeau/vrptw/old/c208\n";
	exit (8);
}


int main(int argc, char *argv[]) {
	program_name = argv[0];
	if (argc < 3) usage();
	nb_iter = stoi(argv[1]);
	instance_file = argv[2];

	long t = time(NULL);
	srand(t);
	cout << "Reading instance file..." << flush;
	readInstanceFileCordeauLaporteVRPTWold(instance_file); cout << "Done." << endl << flush;
	solutionVRPTW s;
	cout << "Initial solution: " << endl << s.toString() << endl << flush;

	//LSProgramBasic<solutionVRPTW> p(&s, nb_iter, 1); // 1% diversification
	LSProgramBasicDynamicWeights<solutionVRPTW> p(&s, nb_iter, 1, 0.001); // 1% diversification, delta=0.001
	cout << "Running LS program" << endl;
	p.run(); 
	cout << endl << "Terminated. Best solution found: \n" << s.toString() << endl;
	p.printWeights();
}
