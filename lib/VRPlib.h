#include <iostream>
#include <fstream>	// file I/O
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy
#include <set>		// set<int>

#include "LSBase.h"
#include "Scenario.h"

#ifndef VRPlib_H
#define	VRPlib_H

using namespace CBLS;

#define ALL_CONSTRAINTS			0
#define	CAPACITY_CONSTRAINT		1
#define	TW_CONSTRAINT			2

/* Handles instance files of Cordeau-Laporte in data/vrp/old ; deterministic VRP */
void readInstanceFileCordeauLaporteVRPold(const char *filename);

/* Handles instance files of Cordeau-Laporte in data/vrptw/old ; deterministic VRPTW*/
void readInstanceFileCordeauLaporteVRPTWold(const char *filename);

/* Handles instances files of Bent-Van Hentenryck in data/bent ; dynamic stochastic VRPTW */
void readInstanceFileBentVanHentenryckDynVRPTW(const char *filename);
void readInstanceFileBentVanHentenryckDynVRPTW_Static(const char *filename);



class SolutionVRP {
protected:
	/* Decision variables */
	int *previous;		// previous[i] = j iff request j is visited before i in the same route
	int *next;			// !!! -> Numbered from 1 to N
	int *vehicle;		// vehicle[i] = j iff request i is serviced by vehicle j
	/* Useful variables */
	int *pos;			// pos[v] = i iff v is the i'th serviced request on the route
	bool *active;		// tells whether a customer request is active in the solution (i.e. must be serviced)
	bool *fixed;		// tells whether a customer request is fixed in the solution (i.e. cannot be moved)
	bool *served;		// tells whether a customer request has been left (i.e. its waiting time cannot be changed anymore)
	bool *left;		// tells whether a customer request has been served (i.e. its service time cannot be changed anymore)
	bool *mandatory;		// tells whether a customer request is mandatory in the solution (stochastic programming purposes)
	/* invariants */
	int *routeLoad;		// routeLoad[i] = x iff vehicle i has a cumulative load of x
	double *routeCost;	// routeCost[r] = cost of route r 
	int *routeN;		// routeN[r] = n iff route r serves exactly n customer vertices
	int *firstFree;		// firstFree[r] = v iff request v is the first non fixed (and active) request on route r (0 if route fixed)
	int *routeNfree;	// routeNfree[r] = n iff route r serves exactly n NON FIEXED CUSTOMER vertices
	int *routeNFreeWaitingPlanned;	// routeNfree[r] = n iff route r serves exactly n non fixed WAITING vertices
	int *routeNunleft;	
public:
	SolutionVRP();												// constructor ***
	SolutionVRP(const SolutionVRP& old_solution);				// copy constructor ***
	SolutionVRP& operator = (const SolutionVRP& sol);
	~SolutionVRP();

	/* dynamic context */
	int activateOneMandatoryRequest(float currtime = -1);	// activates and set mandatory one new request according to current time and its arrival time; return the request number, 0 if no request to activate
	virtual void activateMandatoryRequest(int req);					// activates and set mandatory the request req
	int activateAllPossibleRequests(float currtime);// activates all the requests having a non-zero probability to appear from time currtime (returns the number of activated ones)
	void activateAllWaitingRequests();
	int desactivateNonPlannedOptionalRequests();	// desactivate all optional requests that a not part of the current planning
	void fixSolution(float currtime = -1);			// fixes the requests that have already been serviced at time currtime
	void rejectRequest(int v, bool remove=true);	// rejects one request: remove the associated vertices from the plan, desactivates it, updates counters
	void removeOptionalRequests();					// remove all non mandatory requests from the planning (just before online req insertion)

	/* initialization methods */
	void generateInitialSolution();
	void insertNewRequests(bool newSol = false);	// to be called whenever some requests have been activated (it is then inserted in a route); newSol=true if these are the first reqests activated
	void insertNewReq(int vertex);					// insert a newly activated vertex in a route

	/* selection functions */
	int randomInsertion(int vertex);
	int selectPlannedVertexAtCondition(function <bool(int)> condition, int upperBound, int route = 0);
	int select_Planned_Vertex(int route = 0);
	int select_Unleft_Vertex(int route = 0);
	//int select_Unplanned_Vertex();
	int select_Waiting_Planned_Vertex(int route = 0);
	int select_Waiting_Unplanned_Vertex();
	//int select_Planned_Optional_Vertex(int route = 0);
	void selectSegment(int k, int size, int& fromVertex, int& toVertex, int& len, bool zeroLength = true);
	int select_WorstViolations_Vertex();

	/* differentiation functions */
	int bestInsertion(int vertex, float& best_delta, float violationFactor = 1, int k = 0);
	int bestSegmentInsertion(int fromVertex, int toVertex, float& best_delta, float violationFactor = 1, int k = 0);
	virtual float deltaInsertSegment(int fromVertex, int toVertex, int before_i);
	virtual float getViolationsSegmentInsertion(int fromVertex, int toVertex, int before_i);

	/* altering functions */
	void insertVertex(int vertex, int before_i, bool remove);
	virtual void removeVertex(int vertex, bool notifyRouteChange = true);
	void insertSegment(int first, int last, int before_i, bool remove = true, bool isswap = false);
	void invertSegment(int first, int last);
	virtual void updateRouteInfos(int k = 0);
	void routeChange(int k = 0) { updateRouteInfos(k); }
	
	/* evaluation functions */
	virtual double getCost(int k = 0);
	virtual int getNbVehicle();
	virtual int getNbRequestsPlanned();
	virtual float getProbaMassRequestsPlanned();
	virtual float getViolations(int constraint = ALL_CONSTRAINTS, int vertex = 0);
	virtual float getSegmentViolations(int constraint, int fromVertex, int toVertex);
	int getNumberNonemptyRoutes();

	/* communication */
	int* getpPrevious() { return previous; }
	int* getpNext() { return next; }
	int* getpVehicle() { return vehicle; }
	int* getpRouteNfree() { return routeNfree; }
	int* getpRouteN() { return routeN; }
	int* getpFirstFree() { return firstFree; }
	int* getpPos() { return pos; }
	virtual int nbConstraints() { return 1; }
	bool isActive(int v) { return active[v]; }
	bool isFixed(int v) { return fixed[v]; }
	bool isPlanned(int v) { return vehicle[v]!=0; }
	bool isServed(int v) { return served[v]; }


	/* visualization and debugging */
	virtual std::string& toString();
	void printRouteN() { int n=0; for(int r=1; r<NVeh+1; r++) n += routeN[r]^2; cout << n << endl << flush; }
	virtual void checkSolutionConsistency();
	void logWaitingVertices();
};


class SolutionVRPTW: public SolutionVRP {
protected:
	float *h;			// arrival times
	float *b;			// service times
	float *w;			// waiting times
	float current_time;
public:
	SolutionVRPTW();												// constructor ***
	SolutionVRPTW(const SolutionVRPTW& old_solution);				// copy constructor ***
	SolutionVRPTW& operator = (const SolutionVRPTW& sol);
	~SolutionVRPTW();

	/* dynamic context */
	void fixSolution(float t);
	virtual void lazySchedule();

	/* altering functions */
	virtual void increaseWaitingTime(int vertex, float increment);
	virtual void decreaseWaitingTime(int vertex, float decrement);

	/* differentiation functions */
	virtual float getViolationsSegmentInsertion(int fromVertex, int toVertex, int before_i);
	virtual float getViolations(int constraint = ALL_CONSTRAINTS, int vertex = 0);
	virtual float getSegmentViolations(int constraint, int fromVertex, int toVertex);

	/* communication */
	virtual int nbConstraints() { return 2; }
	virtual void removeVertex(int vertex, bool notifyRouteChange = true);
	virtual void updateRouteInfos(int k = 0);
	virtual float computeRemainingWaitingTime(int v);

	/* visualization and debugging */
	virtual std::string& toString();
	virtual void printVertices();
	virtual void checkSolutionConsistency();
};


/*
class SolutionVRPTW_ScenarioPool: public SolutionVRPTW {
protected:
	float *h;			// arrival times
	float *b;			// service times
public:
	SolutionVRPTW_ScenarioPool();												// constructor ***
	SolutionVRPTW_ScenarioPool(const SolutionVRPTW_ScenarioPool& old_solution);				// copy constructor ***
	SolutionVRPTW_ScenarioPool& operator = (const SolutionVRPTW_ScenarioPool& sol);
	~SolutionVRPTW_ScenarioPool();

	
	virtual void updateRouteInfos(int k = 0);
};
*/




#endif

