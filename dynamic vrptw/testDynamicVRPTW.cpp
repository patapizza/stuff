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
#include "../lib/Scenario.h"
#include "../lib/LSProgram.h"
#include "../lib/LSBase.h"
#include "../lib/tools.h"

using namespace CBLS;
using namespace std;

extern int 	*number;	// number of the customer (in the test instance)
extern int 	N, NVeh, NVertices;
extern int 	*ts;			// time slot of the request
extern bool	*rejected;		// tells whether a request has been rejected
extern double 	*coordX, *coordY;

int lastReqTime;	// unit time of the last online request (so we can end the computation asap)

//#define		SECONDS_PER_TIME_UNIT	7.5 // like in Bent & VanHentenryck : 1 time unit equals 7.5 cpu second
#define			SECONDS_PER_TIME_UNIT	5.0

bool verbose = false;

int maxStaticTime = 60;
int poolSizeStatic = 100;
int nbIterReSampleStatic = 250;

int poolSizeDyn = 10;
int nbIterReSampleDyn = 100;
int scenarioHorizon = 1000;


char *program_name;
char *instance_file;
int nb_iter;
long t0;
double t, t_insert;	// in instance time units
clock_t t_0_c;
int elapsed_time = -1;
int nb_reject = 0;

ScenarioPool<SolutionVRPTW,ScenarioVRPTW> *sp;
//ScenarioPoolApprox<SolutionVRPTW> *sp;
ScenarioPool<SolutionVRPTW,ScenarioVRPTW> *spBig;

inline void logGraph(int iter, float x1, float x2 = -1) {
	ofstream myfile;
	if(iter == 0) 
		myfile.open ("output_graph.txt");
	else
		myfile.open ("output_graph.txt", ios::app);
	myfile << iter << "\t" << x1 << "\t" << x2 << endl;
	myfile.close();
}

int f_optimize_insert(SolutionVRPTW &bestSolution, SolutionVRPTW &currentSolution);
int f_dynamic(SolutionVRPTW &bestSolution, SolutionVRPTW &currentSolution) {	// function to be called after each iteration of the LS program
	static SolutionVRPTW trySolution;
	static int it = 0;
	int r = NONE;
	double elapsed_clocks = (double)(clock() - t0) / CLOCKS_PER_SEC;
	t = elapsed_clocks / SECONDS_PER_TIME_UNIT;

	currentSolution.fixSolution(t);
	bestSolution.fixSolution(t);
	sp->updatePool(t);

	if (t >= elapsed_time + 1) {
		elapsed_time = floor(t);
		if(verbose) cout << "Current time: " << setw(3) << elapsed_time << "\t# Rejected requests: "<< setw(2) << nb_reject << "\t";
		int i = 0;
		bool wasPlanned = false;
		while ((i = bestSolution.activateOneMandatoryRequest(elapsed_time))) {
			if(verbose) cout << "\tActivating " << number[i] << "-" << ts[i];
			trySolution = bestSolution;
			//trySolution.removeOptionalRequests();		// DESACTIVATE !! *************** ------- !!!!!!!!!!! !!!!!!!!!!!!!! %%%%%%%%
			if(!trySolution.isPlanned(i)) {
				trySolution.insertNewReq(i);
				//trySolution.checkSolutionConsistency();
			} else {
				wasPlanned = true;
				if(verbose) cout << "(!)";
				ASSERT(trySolution.isServed(i) == false, "argh");
				trySolution.routeChange(); // because i is now mandatory, it has a service duration
			}

			if(trySolution.getViolations() > 0) {		// if no direct feasible insertion, try some local search to restore feasibility
				t_insert = t;
				LSProgram_SimulatedAnnealing<SolutionVRPTW> p_insert(trySolution, &f_optimize_insert, 1000000, 1000, 0.0, 0.0, 0.95);
				p_insert.run();
				if(trySolution.getViolations() > 0) {	// if still not feasible, reject !
					bestSolution.rejectRequest(i, wasPlanned);
					if(verbose) cout << outputMisc::redExpr(true) << " --> Reject! " << outputMisc::resetColor();
					nb_reject ++;	
				}
				else {
					if(verbose) cout << outputMisc::greenExpr(true) << " OK.." << outputMisc::resetColor();
					bestSolution = trySolution;
				}
			} else {
				if(verbose) cout << outputMisc::greenExpr(true) << " OK" << outputMisc::resetColor();
				bestSolution = trySolution;
			}			//r = UPDATE_EVAL;	
		} if(verbose) cout << endl << flush;
		currentSolution = bestSolution;
		r = UPDATE_EVAL;	// log at every time unit
	}

	if(elapsed_time > lastReqTime) {
		bestSolution.fixSolution(t);
		bestSolution.routeChange();
		if(verbose) cout << "No more dynamic requests in the instance file. Exiting." << endl << endl;
		if(verbose) cout << endl << "Resulting solution: \n" << bestSolution.toString() << endl;
		if(verbose) cout << "Time in seconds (dynamic part): " << (float)(clock() - t0) / CLOCKS_PER_SEC << endl;
		cout << "#Rejects: " << nb_reject << " from " << instance_file << endl;
		//bestSolution.checkSolutionConsistency();
		exit(EXIT_SUCCESS);
	}

	if((++it % nbIterReSampleDyn) == 0) {
		sp->ReSamplePool(t);
		//cout << "RS" << flush;
		r = UPDATE_EVAL;
	}

	//bestSolution.checkSolutionConsistency();
	//currentSolution.checkSolutionConsistency();
	return r;
}

int f_dynamic_noInterOpt(SolutionVRPTW &bestSolution, SolutionVRPTW &currentSolution) {	// function to be called after each iteration of the LS program

	for (int t=0; t<160; t++) {
		bestSolution.fixSolution(t);
		if(verbose) cout << "Current time: " << setw(3) << t << "\t# Rejected requests: "<< setw(2) << nb_reject << "\t";
		int i = 0;
		while ((i = bestSolution.activateOneMandatoryRequest(t))) {
			if(verbose) cout << "\tActivating " << number[i];
			//bestSolution.removeOptionalRequests();
			//bestSolution.lazySchedule();
			bestSolution.insertNewRequests();
			bestSolution.checkSolutionConsistency();
			if(bestSolution.getViolations() > 0) {		// if no feasible insertion
				//cout << "Reject" << bestSolution.toString() << endl << flush;
				bestSolution.rejectRequest(i);
				if(verbose) cout << outputMisc::redExpr(true) << " --> Reject! " << i << outputMisc::resetColor();
				nb_reject ++;
				bestSolution.checkSolutionConsistency();
			} 
			else
				if(verbose) cout << outputMisc::greenExpr(true) << " OK" << outputMisc::resetColor();
		} if(verbose) cout << endl;
	}
	return UPDATE_EVAL;
}


int f_static(SolutionVRPTW &bestSolution, SolutionVRPTW &currentSolution) {
	static int it = 0, progression = 0;
	int r = NONE;
	double elapsed_clock_sec = (double)(clock() - t0) / CLOCKS_PER_SEC;
	it ++;

	if(progression < floor(elapsed_clock_sec*100/maxStaticTime)) {
		progression = floor(elapsed_clock_sec*100/maxStaticTime);
		cout.setf(std::ios::fixed);
		cout.precision(2);
		//if(verbose) cout << "Computation time: " << setw(7) << (float)(clock() - t0) / CLOCKS_PER_SEC << "sec\tStatus: " << ++progression << "%" << endl << flush;
		if(verbose) cout << progression << "% " << flush;
	}
	/*
	if(it % 10000 == 0 || it == 1) {
		logGraph(it, spBig->evalSolution(bestSolution));
	}
	*/

	if (elapsed_clock_sec >= maxStaticTime) {
		cout << "# iterations performed: " << it << "\t";
		return STOP_LS;
	}

	if(it % nbIterReSampleStatic == 0) {
		//sp->ReSamplePool(-1);
		sp->ReSamplePool(-10);	// -10 --> also sample a priori requests
		r = UPDATE_EVAL;		
	}
	//currentSolution.checkSolutionConsistency();

	return r;
}


int f_optimize_insert(SolutionVRPTW &bestSolution, SolutionVRPTW &currentSolution) {
	double elapsed_clocks = (double)(clock() - t0) / CLOCKS_PER_SEC;
	t = elapsed_clocks / SECONDS_PER_TIME_UNIT;

	currentSolution.fixSolution(t);
	bestSolution.fixSolution(t);

	//if (bestSolution.getViolations() == 0 || t - t_insert >= SECONDS_PER_TIME_UNIT/10)	// optimize new insertion for SECONDS_PER_TIME_UNIT/10 seconds
	if (bestSolution.getViolations() == 0 || t - t_insert >= 0.5)
		return STOP_LS;

	return NONE;
}

void usage() {
	cerr << "Usage is " << program_name << " nbSecondsStatic instanceFile [poolSizeDyn] [nbIterReSampleDyn] [scenarioHorizon] \n";
	exit (8);
}

int main(int argc, char *argv[]) {
	program_name = argv[0];
	if (argc < 3) usage();
	maxStaticTime = stoi(argv[1]);
	instance_file = argv[2];
	nb_iter = 1000000000;
/*
	if (argc >= 5) {
		poolSizeDyn = stoi(argv[3]);
		nbIterReSampleDyn = stoi(argv[4]);
	}
	if (argc >= 6)
		scenarioHorizon = stoi(argv[5]);
*/
	if (argc >= 5) {	// for static tests
		poolSizeStatic = stoi(argv[3]);
		nbIterReSampleStatic = stoi(argv[4]);
	}

	t0 = clock();
	srand(time(NULL));
	if(verbose) cout << "Reading instance file : " << instance_file;
	readInstanceFileBentVanHentenryckDynVRPTW(instance_file);


	SolutionVRPTW s;
	//s.activateAllPossibleRequests(-1);
	s.activateAllWaitingRequests();
	//while (s.activateOneMandatoryRequest(-1));			// elapsed_time = -1 before online work
	if(verbose) cout << "Generating initial VRPTW solution ..." << flush;
	s.generateInitialSolution(); 
	s.checkSolutionConsistency();

	//s.printVertices();
	


	if(verbose) cout << "Done.\n" << flush; 
	if(verbose) cout << "Initial solution: " << endl << s.toString() << endl << flush;

	//if(verbose) 
		cout << "Generating static scenario pool (size:" << poolSizeStatic <<" refreshRate: "<< nbIterReSampleStatic <<") ...\t" << flush;
	//												 size,timeH,reqH,currtime
	//sp = new ScenarioPool<SolutionVRPTW,ScenarioVRPTW>(poolSizeStatic, 1000, 1000, -1);
	sp = new ScenarioPool<SolutionVRPTW,ScenarioVRPTW>(poolSizeStatic, 1000, 1000, -10);	// -10 --> also sample a priori requests
	//sp = new ScenarioPoolApprox<SolutionVRPTW>(1, -1);
	if(verbose) cout << "Done.\n" << flush; 
	//cout << sp->toString() << endl << flush;

	//LSProgramBasic<SolutionVRPTW> p(&s, &f_static, nb_iter, 2500, 1); // 1% diversification
	//LSProgramBasicDynamicWeights<SolutionVRPTW> p(&s, &f_static, nb_iter, 2500, 1, 0.0001); // 1% diversification, delta=0.0001
	//LSProgram_SimulatedAnnealing<SolutionVRPTW> p(s, &f_static, nb_iter, 1000, 1.0, 0.01, 0.95);	// init temp:2.0, min temp: 0.01, cooling factor 0.99
	LSProgram_SimulatedAnnealing_ScenarioPool<SolutionVRPTW,ScenarioPool<SolutionVRPTW,ScenarioVRPTW>> p(s, &f_static, nb_iter, 1000, 0.0, 0.0, 0.95, sp);	
	//LSProgram_SimulatedAnnealing_ScenarioPool<SolutionVRPTW,ScenarioPoolApprox<SolutionVRPTW>> p(s, &f_static, nb_iter, 500, 0.0, 0.0, 0.95, sp);	

	//LSProgram_SimulatedAnnealing_MinimizeRouteNumber<SolutionVRPTW> p(&s, &f_static, nb_iter, 2500, 2.0, 0.01, 0.95);
	//LSProgram_SimulatedAnnealing_DynamicWeights<SolutionVRPTW> p(&s, &f_static, nb_iter, 2500, 10.0, 2.0, 0.95, 0.0001);
	cout << "Running LS program... " << flush; // << endl;
	p.run(); 
	cout << "Done. " << flush;
	if(verbose) cout << endl << "Best solution found: \n" << s.toString() << endl;
	//p.printWeights();
	s.checkSolutionConsistency();

	if(verbose) cout << "Time in seconds (static part):" << (float)(clock() - t0) / CLOCKS_PER_SEC << endl;

	if(s.getViolations() > 0) {
		cout << endl << "No feasible solution found ! Aborting dynamic part." << endl;
		exit(EXIT_SUCCESS);
	}

	spBig = new ScenarioPool<SolutionVRPTW,ScenarioVRPTW>(100000, 1000, 1000, -10);
	cout << "Evaluation on the big pool: " << spBig->evalSolution(s) << " rejects from instance " << instance_file << "\tTime(sec): " << (float)(clock() - t0) / CLOCKS_PER_SEC << endl;
	if(verbose) cout << "seconds: " << (float)(clock() - t0) / CLOCKS_PER_SEC << endl << endl;

	// DYNAMIC part	*************************************

/*

	if(verbose) cout << "Starting dynamic part... \tPool size: " << poolSizeDyn << "\tnbIterReSampleDyn: " << nbIterReSampleDyn << "\tscenarioHorizon: " << scenarioHorizon << endl; // << endl;
	t0 = clock();

	//cout << "\t# optional req. kept: " << s.desactivateNonPlannedOptionalRequests();

	delete sp;
	if(verbose) cout << "Generating dynamic scenario pool (size:" << poolSizeDyn << "  req horizon:" << scenarioHorizon <<") ...\t" << flush;
	sp = new ScenarioPool<SolutionVRPTW,ScenarioVRPTW>(poolSizeDyn, 1000, scenarioHorizon, -1);	// time horizon: 1000 --> i.e. disabled
	if(verbose) cout << "Done.\n" << flush; 

	//LSProgram_SimulatedAnnealing<SolutionVRPTW> p_dynamic(s, &f_dynamic, 1000000000, 1000, 10.0, 1.0, 0.95);
	//LSProgram_SimulatedAnnealing<SolutionVRPTW> p_dynamic(s, &f_dynamic, 1000000000, 1000, 0.0, 0.0, 0.95);
	LSProgram_SimulatedAnnealing_ScenarioPool<SolutionVRPTW,ScenarioPool<SolutionVRPTW,ScenarioVRPTW>> p_dynamic(s, &f_dynamic, 1000000000, 100, 1.0, 0.001, 0.95, sp);
	//LSProgram_SimulatedAnnealing_ScenarioPool<SolutionVRPTW,ScenarioPoolApprox<SolutionVRPTW>> p_dynamic(s, &f_dynamic, 1000000000, 1000, 0.0, 0.0, 0.95, sp);
	p_dynamic.run();
	//cout << endl << "Best solution found: \n" << s.toString() << endl;
	//cout << "From instance file... ->" << instance_file << endl;
*/
}
