#include <iostream>
#include <fstream>	// file I/O
#include <sstream>	// ostringstream
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy
#include <limits>   // numeric_limits
#include <iomanip>      // std::setw

#include "VRPlib.h"
#include "Scenario.h"
#include "tools.h"

using namespace std;

/* Class ScenarioVRPTW */
int ScenarioVRPTW::NScenarios = 0;
int ScenarioVRPTW::Counter = 0;

ScenarioVRPTW::ScenarioVRPTW(int currtime, int timeHorizon) {									// constructor ***	
	this->currtime = currtime;
	this->timeHorizon = timeHorizon;
	scenarioID = ++Counter;
	NScenarios++;
	// memory allocation
	requests = new bool[NVeh+N+1];		// demand matrix: requests[i]==true iff a request i in N (N = NVertices x nTimeSlots) is revealed in that scenario
	requestTime = new int[NVeh+N+1];	// time at which the request arrives
	revealed = new bool[NVeh+N+1];		// tells whether the request has been revealed or not
	// initialization
	for(int i=NVeh+1; i<NVeh+N+1; i++)
		requests[i] = false;

	sampledRequestsList = new OrderedStaticLinkedList<int,int>(150);	// ordered list of the SAMPLED requests
	//sampledRequestsList = new OrderedLinkedList<int,int>();	// ordered list of the SAMPLED requests

	// sampling according to probablities in reqProba
	sampleScenario(currtime);
}
ScenarioVRPTW::ScenarioVRPTW(const ScenarioVRPTW& old_scenario) : ScenarioVRPTW(old_scenario.currtime, old_scenario.timeHorizon) {		// copy constructor ***
	*this = old_scenario;	// copy instance data
}
ScenarioVRPTW& ScenarioVRPTW::operator = (const ScenarioVRPTW& s) {
	memcpy(requests, s.requests, (NVeh+N+1)*sizeof(bool));
	memcpy(requestTime, s.requestTime, (NVeh+N+1)*sizeof(int));
	currtime = s.currtime;
	timeHorizon = s.timeHorizon;
	return (*this);
}
ScenarioVRPTW::~ScenarioVRPTW() {									// destructor ***
	delete [] requests;
	delete [] requestTime;
	NScenarios--;
}

void ScenarioVRPTW::sampleScenario(int currtime) {
	for(int i=NVeh+1; i<NVeh+N+1; i++) {
		if(rejected[i])
			continue;
		if(reqTime[i] <= currtime) {				// if the instance file said that the request is revealed (reqTime = inf if never revealed)
				requests[i] = true;
				requestTime[i] = reqTime[i];
				revealed[i] = true;
				ASSERT(reqTime[i] <= maxRevealTime[i], "maxRevealTime error ! i="<< i <<" reqTime:"<<reqTime[i]<<"\tmaxRevealTime:"<<maxRevealTime[i]<<"\ti="<<number[i]);
		}
		else {										// otherwise guess  <----- HERE ARE COMPUTED SAMPLED REQUESTS
			float minReveal = max((float)currtime, minRevealTime[i]);
			requestTime[i] = numeric_limits<int>::max();
			if (minReveal <= (maxRevealTime[i] - (ts[i]==0))) {	// maxRevealTime[i] > -1 ensures that we do not sample a priori requests
				requests[i] = (rand()%100 +1 <= reqProba[i]);// * (maxRevealTime[i]-minReveal) / (maxRevealTime[i]-minRevealTime[i]) );	// the request proba is proportional to the elapsed time in the interval [minRevealTime,maxRevealTime]
				if(requests[i]) {			// if probability success
					revealed[i] = false;	// just a sample
					if(maxRevealTime[i] - minReveal >= 1.0)
						requestTime[i] = (rand() % (int)floor(maxRevealTime[i]-minReveal)) + floor(minReveal);
					else
						requestTime[i] = floor(minReveal);
					sampledRequestsList->insert(i, requestTime[i]);
				}
			}
		}
	}
	/*
	if(scenarioID==1) {
		cout << toString() << endl;

		int i;
		sampledRequestsList->goFirst();
		cout << "Element in list: " ;
		do {
			if(sampledRequestsList->getNbElements() == 0)
				break;
			i = sampledRequestsList->getCurrent();
			cout << "\ti=" << number[i] << "-" << ts[i] << ",t:" << requestTime[i];
		} while(sampledRequestsList->moveNext());
		cout << endl;
	}
	*/
}

std::string& ScenarioVRPTW::toString() {
	ostringstream out; 

	out << "Scenario " << scenarioID << ":" << endl; 
	for(int i=NVeh+1; i<NVeh+NVertices+1; i++) {
		out << "Vertex " << setw(3) << number[i] << "\t";
		for(int ts=0; ts<nTimeSlots; ts++) {
			//out << "\t" << outputMisc::boolColorExpr(requests[i+ts*NVertices]) << "TS" << ts <<" [" << minRevealTime[i+ts*NVertices]<<"<="<<requestTime[i+ts*NVertices]<<"<="<<maxRevealTime[i+ts*NVertices<<"]"] << outputMisc::resetColor();
			int r = min(999, requestTime[i+ts*NVertices]);
			out << "\t" << outputMisc::boolColorExpr(requests[i+ts*NVertices]) << "TS" << ts << "[" << setw(3) << r << "<=" << setw(3) << floor(maxRevealTime[i+ts*NVertices]) << "]" << outputMisc::resetColor();
		}
		out << endl;
	}

	out << "Scenario from ordered list: ";
	sampledRequestsList->goFirst();
	int n = 0;
	do {
		if(sampledRequestsList->getNbElements() == 0)
			break;
		int i = sampledRequestsList->getCurrent();
		out << number[i] << "-" << ts[i] << " ";
		n++;
	} while(sampledRequestsList->moveNext());
	out << "\t total:" << n << endl;

	static string str = ""; str = out.str();
	return (str);
}

void ScenarioVRPTW::ReSampleScenario(float t) {
	currtime = t;
	
	//delete sampledRequestsList;
	//sampledRequestsList = new OrderedLinkedList<int,int>();
	sampledRequestsList->clean();

	sampleScenario(t);
	
	//cout << "Scenario " << scenarioID << " re-sampled." << endl;
}

void ScenarioVRPTW::updateScenario(float t) {
	currtime = t;
	sampledRequestsList->goFirst();
	do {
		if(sampledRequestsList->getNbElements() == 0)
			break;
		int i = sampledRequestsList->getCurrent();
		ASSERT(i>NVeh && i <= NVeh+N+1, "argh");
		if(reqTime[i] <= t) {				// if the instance file said that the request is revealed (reqTime = inf if never revealed)
			requests[i] = true;
			requestTime[i] = reqTime[i];
			revealed[i] = true;
			sampledRequestsList->removeCurrent(); // --> removes and performs a "MOVE TO NEXT"
		}
		else if (t > maxRevealTime[i]) {	// the "else" is very important here
			requests[i] = false;
			sampledRequestsList->removeCurrent(); // --> removes and performs a "MOVE TO NEXT"
		}
		else if (floor(t) > requestTime[i]) {
			//requests[i] = false;
			sampledRequestsList->removeCurrent(); // --> removes and performs a "MOVE TO NEXT"
			float minReveal = max((float)currtime, minRevealTime[i]);

			//requests[i] = (rand()%100 +1 <= 100*(maxRevealTime[i]-minReveal) / (maxRevealTime[i]-minRevealTime[i]) );	// the request proba is proportional to the elapsed time in the interval [minRevealTime,maxRevealTime]
			//if(requests[i]) {			// if probability success
				requestTime[i] = (rand() % (int)floor(ceil(maxRevealTime[i])-minReveal)) + minReveal;
				sampledRequestsList->insert(i, requestTime[i]);
			//}

			sampledRequestsList->goFirst();
		}
		else {
			break;
		}
		
	} while(sampledRequestsList->isCurrentNull() == false);
}

double ScenarioVRPTW::evalSolution_nbReject(SolutionVRPTW &solution, int requestHorizon) {
	int nbReject = 0;

	if(solution.getViolations() > 0)
		return 100000;

	tmpSol = solution;
	//tmpSol.removeOptionalRequests();
	
	int i, j = 0;
	sampledRequestsList->goFirst();
	do {
		if(sampledRequestsList->getNbElements() == 0)
			break;
		i = sampledRequestsList->getCurrent();

		ASSERT(requests[i] && !revealed[i], "Wrong element in sampledRequestsList !! i="<<i);
		if(requestTime[i] - currtime > timeHorizon)
			break;
		//if(requestTime[i] > )
		tmpSol.fixSolution(requestTime[i]);
		/*
		if(tmpSol.noAvailableVehicle()) {
			j++;
			nbReject++;
			continue;
		}
		*/
		tmpSol.activateMandatoryRequest(i);
		if(!tmpSol.isPlanned(i)) {
			tmpSol.insertNewReq(i);
		}
		if(tmpSol.getViolations() > 0) {
			tmpSol.removeVertex(i);
			nbReject++;
		}
		j++;
	} while(sampledRequestsList->moveNext() && j < requestHorizon);
	/*
	for(int i=NVeh+1; i<NVeh+N+1; i++) {
		if(requests[i] && !revealed[i]) {	// if it is a sampled request
			tmpSol.activateMandatoryRequest(i);
			tmpSol.insertNewRequests();
			if(tmpSol.getViolations() > 0) {
				tmpSol.removeVertex(i);
				nbReject++;
			}
		}
	}
	*/
	return nbReject;
}

