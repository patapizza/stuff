#include <iostream>
#include <fstream>	// file I/O
#include <sstream>	// ostringstream
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy
#include <limits>   // numeric_limits
#include <iomanip>      // std::setw



#ifndef SCENARIO_H
#define	SCENARIO_H

#include "VRPlib.h"
#include "tools.h"

extern int N;			// number of customer requests, numbered: [NVeh+1..NVeh+N]; N = NVertices*nTimeSlots
extern int NVeh;		// number of vehicles, numbered: [1..NVeh]
extern int NVertices;	// number of customer vertices (different from N, because of time slots, there can be several requests per vertex)
extern int nTimeSlots;	// number of time slots (TS0 = a priori requests)
extern int *reqProba;	// probabitlity for that request to appear online (for a given time slot)
extern int 	*reqTime;	// time at which the request is revealed (at least, if it is revealed)
extern int 	*number;	// number of the customer (in the test instance)
extern double **c;		// travel costs/durations
extern float *demand;	// demand et each vertex
extern float *duration;	// service duration at each vertex
extern int   *e;		// start of time window at each vertex
extern int   *l;		// end of time window at each vertex

extern float *maxRevealTime;	// according to the time slot, latest time a vehicle can depart from the depot, serve the vertex 
								// and return to depot (lambda_c in Bent&VanHentenryck's paper)
								// defines an upper bound on the time at which a request could be revealed
extern float *minRevealTime;	// lower bound on which a request can be revealed
extern int 	*ts;			// time slot of the request
extern bool	*rejected;		// tells whether a request has been rejected


class ScenarioVRPTW {
private:
	bool *requests;	// demand matrix: requests[i]==true iff a request i in N (N = NVertices x nTimeSlots) is revealed 
					// (for a given vertex v in NVertices, there are several potential requests, i.e. one at each time slot)
	bool *revealed; // tells whether a request has been actually revealed or not
	int *requestTime;	// time at which the request arrives
	int scenarioID;	// provides a way to identify the scenarios
	int currtime;
	SolutionVRPTW tmpSol;	// temporary solution object used to assess a solution quality
	static int NScenarios;	// remembers the number of scenarios.
	static int Counter;		// Simply an incremental counter, used to deliver IDs
	OrderedStaticLinkedList<int,int>	*sampledRequestsList;
	//OrderedLinkedList<int,int>	*sampledRequestsList;
	int timeHorizon;
public:
	ScenarioVRPTW(int currtime, int timeHorizon);						// constructor ***
	ScenarioVRPTW(const ScenarioVRPTW& old_scenario);					// copy constructor ***
	ScenarioVRPTW& operator = (const ScenarioVRPTW& s);
	~ScenarioVRPTW();

	double evalSolution_nbReject(SolutionVRPTW& solution, int scenarioHorizon = 1000);

	void sampleScenario(int currtime);	// Re-sample the scenario (for instance, in case it is not valable anymore), according to current time
	void updateScenario(float t);		// Updates the scenario wrt the realizations in the instance file and the current time
	void ReSampleScenario(float t);		
	std::string& toString();
};


template <class Solution, class Scenario> 
class ScenarioPool {
protected:
	Scenario **scenarios;	
	int poolSize;
	int requestHorizon;
	int timeHorizon;
public:
	ScenarioPool(int size, int timeHorizon, int requestHorizon, int currtime) {
		this->poolSize = size;
		this->requestHorizon = requestHorizon;
		this->timeHorizon = timeHorizon;
		scenarios = new Scenario*[poolSize];
		for (int i=0; i<poolSize; i++)
			scenarios[i] = new Scenario(currtime, timeHorizon);
	}
	~ScenarioPool() {
		for (int i=0; i<poolSize; i++)
			delete scenarios[i];
		delete [] scenarios;
	}
	void updatePool(float t) {
		for(int i=0; i<poolSize; i++) {
			scenarios[i]->updateScenario(t);
		}
	}
	void ReSamplePool(float t) {
		/*
		if(proportion == 0.0)
			return;
		float nReSample = (float) poolSize * proportion;
		int chance  = rand() % 100 + 1;
		int nReS = floor(nReSample) + (chance <= (nReSample-floor(nReSample))*100); // decide probabilistically for the remaining fraction
		if(nReS > 0) {
			int fromPos = rand() % (poolSize-nReS+1);
			for(int i=fromPos; i<fromPos+nReS; i++) {
				scenarios[i]->ReSampleScenario(t);
			}		
		}
		*/
		for(int i=0; i<poolSize; i++) {
			scenarios[i]->ReSampleScenario(t);
		}
	}

	double evalSolution(Solution& solution) {
		double eval = 0.0;

		for (int i=0; i<poolSize; i++)
			eval += scenarios[i]->evalSolution_nbReject(solution, requestHorizon);

		//cout << "Solution average rejections on scenario pool (size: "<<poolSize<<"): " << (double)eval/poolSize << endl;
		return eval / poolSize;
	}



	Scenario **getpScenarios() { return scenarios; }

	int getSize() { return poolSize; }
	std::string& toString() {
		ostringstream out; 

		out << "Scenario Pool\n-------------" << endl;
		for(int i=0; i<poolSize; i++) 
			out << endl << scenarios[i]->toString();

		static string str = ""; str = out.str();
		return (str);
	}
};


template <class Solution> 
class ScenarioPoolApprox {
protected:
	float current_time;
	int nbIter;
	OrderedLinkedList<int,float>	*potentialRequestsList;
	SolutionVRPTW tmpSol;	// temporary solution object used to assess a solution quality
public:
	ScenarioPoolApprox(int nbIter, int currtime) {
		current_time = currtime;
		this->nbIter = nbIter;
		potentialRequestsList = new OrderedLinkedList<int,float>();	// ordered list of POTENTIAL requests
		for(int i=NVeh+1; i<NVeh+N+1; i++) {
			if(current_time < reqTime[i] && reqProba[i] > 0) {				// if the instance file said that the request is revealed (reqTime = inf if never revealed)
				potentialRequestsList->insert(i, minRevealTime[i]+(maxRevealTime[i]-minRevealTime[i])/2);
			}
		}
	}
	~ScenarioPoolApprox() {
	}
	void updatePool(float t) {
		current_time = t;

		potentialRequestsList->goFirst();
		do {
			if(potentialRequestsList->getNbElements() == 0)
				break;
			int i = potentialRequestsList->getCurrent();

			if(reqTime[i] <= t || maxRevealTime[i] < t) {				// if the instance file said that the request is revealed (reqTime = inf if never revealed)
					potentialRequestsList->removeCurrent(); // --> removes and performs a "MOVE TO NEXT"
			}
			else {
				break;
			}
		} while(potentialRequestsList->isCurrentNull() == false);
	}

	void ReSamplePool(float t) {
		//cout << "resample" << flush;
	}

	double evalSolution(Solution& solution) {
		double eval = 0.0;

		if(solution.getViolations() > 0)
			return 100000;

		for(int it=0; it < nbIter; it++) {
			tmpSol = solution;
			int i;
			potentialRequestsList->goFirst();
			do {
				if(potentialRequestsList->getNbElements() == 0)
					break;
				i = potentialRequestsList->getCurrent();
				int requestTime_ = (rand() % (int)floor(maxRevealTime[i]-minRevealTime[i])) + minRevealTime[i];
				tmpSol.fixSolution(requestTime_);		// ---> problem here ! what if the previous request simulated had a requestTime_ > than the current one
				tmpSol.activateMandatoryRequest(i);		//			solution: reinitialize the fixations each time.. 
				if(!tmpSol.isPlanned(i)) {
					tmpSol.insertNewReq(i);
				}
				if(tmpSol.getViolations() > 0) {
					tmpSol.removeVertex(i);
					eval += (double)reqProba[i] / 100;
				} else {
					int chance = rand() % 10000 + 1;
					float reqProbaWRTtime = (float)reqProba[i]/100 * max((float)1, max((float)0, maxRevealTime[i] - max(current_time, minRevealTime[i])) / (maxRevealTime[i]-minRevealTime[i]));
					if (chance > reqProbaWRTtime*10000)
						tmpSol.removeVertex(i);
				}
			} while(potentialRequestsList->moveNext());			
		}
		return eval / nbIter;
	}
};


#endif