#include <iostream>
#include <fstream>	// file I/O
#include <sstream>	// ostringstream
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy
#include <limits>   // numeric_limits
#include <iomanip>      // std::setw
#include <set>		// set<int>

#include "VRPlib.h"
#include "tools.h"

using namespace std;


/**** Problem data variables ****/
int N;		// number of vertices [1..N]
int NVeh;	// number of vehicles [1..NVeh]
int Q;		// vehicle max capacity
double 	**c;
double 	*coordX; 
double 	*coordY;
float 	*demand;	// demand et each vertex
int 	*number;	// number of the customer (in the test instance)


// Dynamic context
int 	*reqTime;		// time at which the request is revealed
int 	*ts;			// time slot of the request
int 	NVertices;		// number of customer vertices (different from N, because of time slots, there can be several requests per vertex)
int 	nTimeSlots;		// number of time bins
int 	NWaiting;		// number of waiting fake requests, i.e. vertices where the vehicle can go in order to wait for possible sorrounding requests
int 	*reqProba;		// probabitlity for that request to appear online (for a given time slot)
float 	*maxRevealTime;	// according to the time slot, latest time a vehicle can depart from the depot, serve the vertex 
						// and return to depot (lambda_c in Bent&VanHentenryck's paper)
						// defines an upper bound on the time at which a request could be revealed
float 	*minRevealTime;	// lower bound on which a request can be revealed
bool	*rejected;		// tells whether a request has been rejected


//int *verticesShuffle;

/*
inline std::ostream &operator << (std::ostream &out_file, solutionTSP& s) {
	if (s.getViolations() > 0) out_file << "Infeasible ! ";
	out_file << "Cost=" << s.getCost() << " ";
	for(int i=1; i<N+1; i++)
		out_file << s.step[i] << " ";
	return (out_file);
}*/



/* class SolutionVRP ********************************************************************************************
*	defines a solution for a TSP 
*/


SolutionVRP::SolutionVRP() {											// constructor ***
	active = new bool[NVeh+N+1+NWaiting];	// tells whether a customer request is active in the sol (i.e. must be serviced)
	for (int i=1; i<NVeh+1; i++) active[i] = true;
	for (int i=NVeh+1; i<NVeh+N+1+NWaiting; i++) active[i] = false;
	//rejected = new bool[NVeh+N+1];	// tells whether a customer request has been rejected yet (dynamic context only)
	//	for (int i=1; i<NVeh+N+1; i++) rejected[i] = false;
	fixed = new bool[NVeh+N+1+NWaiting];		// tells whether a customer request is fixed in the sol (i.e. cannot be moved)
	for (int i=1; i<NVeh+N+1+NWaiting; i++) fixed[i] = false;
	served = new bool[NVeh+N+1+NWaiting];		// tells whether a customer request has been served in the sol (i.e. its service time cannot be changed)
	for (int i=1; i<NVeh+N+1+NWaiting; i++) served[i] = false;
	left = new bool[NVeh+N+1+NWaiting];		// tells whether a customer request has been left in the sol (i.e. its waiting time cannot be changed)
	for (int i=1; i<NVeh+N+1+NWaiting; i++) left[i] = false;
	mandatory = new bool[NVeh+N+1+NWaiting];		// tells whether a customer request is mandatory in the sol (for stochastic programming purposes)
	for (int i=1; i<NVeh+1; i++)	mandatory[i] = true;
	for (int i=NVeh+1; i<NVeh+N+1+NWaiting; i++) mandatory[i] = false;
	previous = new int[NVeh+N+1+NWaiting];	// vertices from 1 to NVeh + N
	next = new int[NVeh+N+1+NWaiting];
	vehicle = new int[NVeh+N+1+NWaiting];	// vertices from 1 to Nveh + N 		// vehicle[i] = i iff i is a depot
									//										vehicle[i] = 0 iff i is not inserted
	for (int i=1; i<NVeh+1; i++) vehicle[i] = i; 	
	for (int i=NVeh+1; i<NVeh+N+1+NWaiting; i++) vehicle[i] = 0; 	// To begin with, no customer vertex inserted
	pos = new int[NVeh+N+1+NWaiting];		// vertices from 1 to Nveh + Nv 	// pos[v] = i iff v is the i'th serviced vertex on the route
	for (int i=1; i<NVeh+1; i++) pos[i] = 1;
	for (int i=NVeh+1; i<NVeh+N+1+NWaiting; i++) pos[i] = -1;
	routeLoad = new int [NVeh+1];	// routeLoad[k] = x iff vehicle k has a cumulative routeLoad of x
	routeCost = new double [NVeh+1];// routeCost[r] = x iff route r has a cost x
	routeN = new int [NVeh+1];		// routeN[r] = n iff route r serves exactly n customer vertices
	firstFree = new int [NVeh+1];	// firstFree[r] = v iff vertex v is the first non fixed (and active) vertex on route r (0 if route fixed)
	routeNfree = new int [NVeh+1];	// routeNfree[r] = n iff route r serves exactly n NON FIEXED CUSTOMER vertices
	routeNFreeWaitingPlanned = new int [NVeh+1]; // the route has exactly n non fixed WAITING vertices
	routeNunleft = new int [NVeh+1]; // the route has exactly n non left -yet- by their vehicle (dynamic context)
	/*
	verticesShuffle = new int[NVeh+N];
	for (int i=0; i<NVeh+N; i++)	// do not put the waiting vertices in this !
		verticesShuffle[i] = i+1;

	shuffleArray<int>(verticesShuffle, NVeh+N, 100);	
	*/
}
SolutionVRP::SolutionVRP(const SolutionVRP& old_solution) : SolutionVRP() {		// copy constructor ***		
	*this = old_solution;
}

SolutionVRP& SolutionVRP::operator = (const SolutionVRP& sol) {
	static int it = 0;
	memcpy(active, sol.active, (NVeh+N+1+NWaiting)*sizeof(bool));
	memcpy(fixed, sol.fixed, (NVeh+N+1+NWaiting)*sizeof(bool));
	memcpy(served, sol.served, (NVeh+N+1+NWaiting)*sizeof(bool));
	memcpy(left, sol.left, (NVeh+N+1+NWaiting)*sizeof(bool));
	memcpy(mandatory, sol.mandatory, (NVeh+N+1+NWaiting)*sizeof(bool));
	memcpy(previous, sol.previous, (NVeh+N+1+NWaiting)*sizeof(int));
	memcpy(next, sol.next, (NVeh+N+1+NWaiting)*sizeof(int));
	memcpy(vehicle, sol.vehicle, (NVeh+N+1+NWaiting)*sizeof(int));
	memcpy(pos, sol.pos, (NVeh+N+1+NWaiting)*sizeof(int));
	memcpy(routeLoad, sol.routeLoad, (NVeh+1)*sizeof(int));
	memcpy(routeCost, sol.routeCost, (NVeh+1)*sizeof(double));
	memcpy(routeN, sol.routeN, (NVeh+1)*sizeof(int));
	memcpy(firstFree, sol.firstFree, (NVeh+1)*sizeof(int));
	memcpy(routeNfree, sol.routeNfree, (NVeh+1)*sizeof(int));
	memcpy(routeNFreeWaitingPlanned, sol.routeNFreeWaitingPlanned, (NVeh+1)*sizeof(int));
	memcpy(routeNunleft, sol.routeNunleft, (NVeh+1)*sizeof(int));

	/*	
	if((++it) % 50 == 0)
		shuffleArray<int>(verticesShuffle, NVeh+N, 1);
	*/
	//updateRouteInfos();

	return (*this);
}
SolutionVRP::~SolutionVRP() {										// destructor ***
	delete [] active;
	delete [] fixed;
	delete [] served;
	delete [] left;
	delete [] mandatory;
	delete [] previous;
	delete [] next;
	delete [] vehicle;
	delete [] pos;
	delete [] routeLoad;
	delete [] routeCost;
	delete [] routeN;
	delete [] firstFree;
	delete [] routeNfree;
	delete [] routeNFreeWaitingPlanned;
	delete [] routeNunleft;
}

void SolutionVRP::insertNewRequests(bool newSol) {	// BEST INSERTION INITIAL SOL
	if (newSol) {
		for(int r=1; r<NVeh+1; r++)	{
			vehicle[r] = r;
			previous[r] = r;
			next[r] = r;
		}
		updateRouteInfos();
	}
		
	/* count the number of MANDATORY but UNPLANNED requests */
	int unplanned = 0;
	for (int i=NVeh+1; i<NVeh+N+1; i++) 
		if(active[i] && mandatory[i] && vehicle[i] == 0) unplanned++;

	
	/* Then assign best insertion to each customer vertex in turn (random version) */
	for (; unplanned>0; unplanned--) {
		int i = rand() % unplanned + 1;
		// select the i'th unplanned customer
		int vertex, count=0;
		for (int j=NVeh+1; j<NVeh+N+1; j++) {
			if (vehicle[j] == 0 && active[j] && mandatory[j]) {	// if j is unplanned
				count ++;
				if (count == i) {	// if j is the i'th unplanned vertex
					vertex = j;
					break;
				}
			}
			else continue;
		}
		insertNewReq(vertex);
	}
}

void SolutionVRP::insertNewReq(int vertex) {
	ASSERT(active[vertex] && vehicle[vertex]==0 && mandatory[vertex], "argh");
	float delta;
	int before_i = bestInsertion(vertex, delta);
	ASSERT(before_i > 0 , "argh");
	//cout << "\ninserting request " << number[vertex] << "-" << ts[vertex] << endl;
	//cout << endl << "Sol:" << toString();
	//cout << "delta: " << delta << " viol insertion: " << getViolationsSegmentInsertion(vertex, vertex, before_i) << endl;
	insertVertex(vertex, before_i, false);
	//cout << "sol after insert :" << toString() << endl << flush;
	//cout << "vertices :"; printVertices();
	//routeChange(vehicle[before_i]);
}

void SolutionVRP::generateInitialSolution() {
	insertNewRequests(true);
}

/* bestInsertion(int vertex)
	Find the best position to REinstert a single vertex "vertex"; best_delta gives the resulting cost difference
*/
int SolutionVRP::bestInsertion(int vertex, float& best_delta, float violationFactor, int k) {
	//return randomInsertion(vertex);
	return bestSegmentInsertion(vertex, vertex, best_delta, violationFactor, k);
}

/* bestSegmentInsertion(int vertex)
	Find the best position to REinstert a seglent od vertices; best_delta gives the resulting cost difference
*/
/*
int SolutionVRP::bestSegmentInsertion(int fromVertex, int toVertex, float& best_delta, float violationFactor, int k) {
	int before_i = -1;
	double min_delta = numeric_limits<double>::max();
	int x = 0;
	for (int i=1; i<NVeh+N+1; i++) {
		float delta = 0.0;
		if (vehicle[fromVertex] != 0 && pos[fromVertex] <= pos[i] && pos[i] <= pos[toVertex]+1) continue; 		// skip if wrong position
		if (vehicle[i] == 0 || !active[i] || fixed[i]) continue;  	// skip if vertex i not planned yet, disabled or fixed
		if (k !=0 && vehicle[i] != k) continue;						// skip if a specific route k was given
		delta = - c[previous[i]][i] 
				+ c[previous[i]][fromVertex]
				+ c[toVertex][i]
				//+ 1000*(routeN[vehicle[i]]==0)		// is the destination route empty (i.e. will it use a supplementary route ?)
				+ 10000 * min((float) 10000, getViolationsSegmentInsertion(fromVertex, toVertex, i));
		x++;
		if (delta <= min_delta) {
			before_i = i;
			min_delta = delta;
		} 
	} 
	best_delta = (float) min_delta;
	ASSERT(before_i > 0, "argh");
	return before_i;
}
*/
int SolutionVRP::bestSegmentInsertion(int fromVertex, int toVertex, float& best_delta, float violationFactor, int k) {
	int before_i = -1;
	float min_delta = numeric_limits<float>::max();
	float delta;
	//for (int i=1; i<NVeh+N+1; i++) {
	for(int r=1; r<NVeh+1; r++) {
		if (k !=0 && r != k) continue;						// skip if a specific route k was given
		if (fixed[k]) continue;
		// first check for before_i := r
		if ((vehicle[fromVertex] ==r && pos[fromVertex] <= pos[r] && pos[r] <= pos[toVertex]+1) == false)	
			if (!fixed[r] && active[r]) {
				delta = deltaInsertSegment(fromVertex, toVertex, r);
				if (delta < min_delta) {
					before_i = r;
					min_delta = delta;
				} 
			}	
		if (routeN[r] == 0) continue;

		// then check for every vertices in route r
		for(int i = firstFree[r]; i != r; i = next[i]) {
			if (vehicle[fromVertex] == r && pos[fromVertex] <= pos[i] && pos[i] <= pos[toVertex]+1) continue; 		// skip if wrong position
			if (!mandatory[toVertex] && number[toVertex]==number[i]) continue; 	// waiting version
			if (!mandatory[fromVertex] && number[fromVertex]==number[previous[i]]) continue; 	// waiting version
			delta = deltaInsertSegment(fromVertex, toVertex, i);
			if (delta < min_delta) {
				before_i = i;
				min_delta = delta;
			} 
		}	
	} 
	best_delta = min_delta;
	ASSERT(before_i > 0 , "argh");
	return before_i;
}
float SolutionVRP::deltaInsertSegment(int fromVertex, int toVertex, int before_i) {
	float delta = 0.0;
	ASSERT(before_i > 0, "argh");
	ASSERT((vehicle[fromVertex] == vehicle[before_i] && pos[fromVertex] <= pos[before_i] && pos[before_i] <= pos[toVertex]+1) == false, "wrong deltaInsertSegment call"); 		// stop if wrong position
	ASSERT(vehicle[before_i] != 0 && active[before_i] && !fixed[before_i], "wrong deltaInsertSegment call"); 
	delta = - c[previous[before_i]][before_i] 
			+ c[previous[before_i]][fromVertex]
			+ c[toVertex][before_i]
			//+ 1000*(routeN[vehicle[before_i]]==0)		// is the destination route empty (i.e. will it use a supplementary route ?)
			+ 10000 * getViolationsSegmentInsertion(fromVertex, toVertex, before_i);
	return delta;
}


int SolutionVRP::select_WorstViolations_Vertex() {
	int vertex = -1;
	float max_delta = numeric_limits<float>::min();

	for(int r=1; r<NVeh+1; r++) {
		for(int v = firstFree[r]; v != r; v = next[v]) {
			ASSERT(active[v], "argh");
			ASSERT(vehicle[v] != 0, "argh");
			ASSERT(!fixed[v] != 0, "argh");
			float viol = getViolations(0, v);
			float cost = c[previous[v]][v] + c[v][next[v]] - c[previous[v]][next[v]];
			if(viol*10000+cost > max_delta) {
				vertex = v;
				max_delta = viol*10000+cost;
			}
		}	
	} 
	return vertex;
}

int SolutionVRP::selectPlannedVertexAtCondition(function <bool(int)> condition, int upperBound, int route) {
	int v = -1, tmp = 0, x, elems_found = 0;

	x = (rand() % upperBound) + 1;

	while(elems_found < x) {
		for(int k=1; k<NVeh+1;k++) {
			if (route != k && route > 0)
				continue;

			if (condition(k) == true)
				elems_found++;	
			if (elems_found == x) {
				v = k;
				break;
			} 

			for (int i = next[k]; i != k; i = next[i]) {
				if (condition(i) == true)
					elems_found++;	
				if (elems_found == x) {
					v = i;
					break;	// break the for loop
				} 
			}	
			if (elems_found == x)
				break;	// break the while loop
		}
		if(0 == elems_found)
			return -1;
	}

	ASSERT(condition(v), "argh");

	return v;
}

int SolutionVRP::randomInsertion(int vertex) {
/*
	int before_i;
	do {
		before_i = rand() % (NVeh + N) + 1; // generate random number in [1..NVeh+N]
	} while (before_i == vertex || before_i == next[vertex] || vehicle[before_i]==0 || !active[before_i] || fixed[before_i] );
	return before_i;
*/	

	//shuffleArray<int>(verticesShuffle, NVeh+N, 1);
	/*
	int tmp, v;
	v = selectElement<int>(verticesShuffle, NVeh+N+1, [&](int e) {
		return e<NVeh+N+1 && e!=vertex && e!=next[vertex] && vehicle[e]!=0 && active[e] && !fixed[e] 
				&& (mandatory[vertex] || number[vertex] != number[e])  				//  request is a waiting req --> before_i is not located  --> waiting version
				&& (mandatory[vertex] || number[vertex] != number[previous[e]]);	//		on the same vertex (nor its previous !)
	}, &tmp);
	if(tmp == -1)
		return -1;
	return v;
	*/
	int Nfree = 0, v;
	for(int k=1; k<NVeh+1;k++)
		Nfree += routeNfree[k]+1;

	if(Nfree == 0)
		return -1;

	if(vehicle[vertex] > 0) {
		v = selectPlannedVertexAtCondition(
				[&](int e) { return !fixed[e] && e!=vertex && e!=next[vertex] 
					&& (mandatory[vertex] || number[vertex] != number[e])  				//  request is a waiting req --> before_i is not located  --> waiting version
					&& (mandatory[vertex] || number[vertex] != number[previous[e]]) ;}	//		on the same vertex (nor its previous !)	
			, Nfree);
	} else {
		v = selectPlannedVertexAtCondition(
				[&](int e) { return !fixed[e] 
					&& (mandatory[vertex] || number[vertex] != number[e])  				//  request is a waiting req --> before_i is not located  --> waiting version
					&& (mandatory[vertex] || number[vertex] != number[previous[e]]) ;}	//		on the same vertex (nor its previous !)	
			, Nfree);
	}

	if(v > 0) {
		ASSERT(active[v], "argh");
		ASSERT(vehicle[v] != 0, "argh");
		ASSERT(!fixed[v] != 0, "argh");
	}
	
	return v;
}

int SolutionVRP::select_Unleft_Vertex(int route) {
	int Nunleft = 0, v;

	for(int k=1; k<NVeh+1;k++)
		Nunleft += routeNunleft[k];

	if(Nunleft == 0)
		return -1;

	v = selectPlannedVertexAtCondition(
			[&](int e) { return e >= NVeh+1 && left[e]==false;	}
		, Nunleft, route);

	if(v > 0) {
		ASSERT(active[v], "argh");
		ASSERT(vehicle[v] != 0, "argh");
	}

	return v;
}


int SolutionVRP::select_Planned_Vertex(int route) {
	int Nfree = 0, v;

	for(int k=1; k<NVeh+1;k++)
		Nfree += routeNfree[k];

	if(Nfree == 0)
		return -1;

	v = selectPlannedVertexAtCondition(
			[&](int e) { return e >=NVeh+1 && fixed[e]==false;	}
		, Nfree, route);

	if(v > 0) {
		ASSERT(active[v], "argh");
		ASSERT(vehicle[v] != 0, "argh");
		ASSERT(!fixed[v] != 0, "argh");
	}

	return v;
}


int SolutionVRP::select_Waiting_Unplanned_Vertex() {
	int v;
	v = (rand() % NVertices) + NVeh+1;

	v += nTimeSlots*NVertices;
	while (vehicle[v] != 0)
		v += NVertices;

	ASSERT(NVeh+N+NWaiting >= v, "argh ! not enough waiting vertices !");
	return v;
}

int SolutionVRP::select_Waiting_Planned_Vertex(int route) {
	int v = -1, tmp = 0;
	
	int nbWaitings = 0;
	for (int r=1; r<NVeh+1; r++)
		nbWaitings += routeNFreeWaitingPlanned[r];

	if(nbWaitings == 0)
		return -1;
	
	v = selectPlannedVertexAtCondition(
			[&](int e) { return e >= NVeh+N+1 && !fixed[e]; }
		, nbWaitings, route);
	
	if(v > 0) {
		ASSERT(v >= NVeh+N+1, "argh");
		ASSERT(active[v], "argh");
		ASSERT(vehicle[v] != 0, "argh");
		ASSERT(!fixed[v] != 0, "argh");
	}

	return v;
}

/*
int SolutionVRP::select_Unplanned_Vertex() {
	//shuffleArray<int>(verticesShuffle, NVeh+N, 1);
	int tmp, v;
	v = selectElement<int>(verticesShuffle, NVeh+N+1, [&](int e) {
		return e>NVeh && e<NVeh+N+1 && vehicle[e]==0 && active[e] && !fixed[e];	}, &tmp);
	if(tmp == -1)
		return -1;
	return v;
}

int SolutionVRP::select_Planned_Optional_Vertex(int route) {
	//shuffleArray<int>(verticesShuffle, NVeh+N, 1);
	int tmp, v;
	if(route==0)
		v = selectElement<int>(verticesShuffle, NVeh+N+1, [&](int e) {
			return e>NVeh && e<NVeh+N+1 && vehicle[e]!=0 && active[e] && !fixed[e] && !mandatory[e];	}, &tmp);
	else 
		v = selectElement<int>(verticesShuffle, NVeh+N+1, [&](int e) {
			return e>NVeh && e<NVeh+N+1 && vehicle[e]==route && active[e] && !fixed[e] && !mandatory[e];	}, &tmp);
	if(tmp == -1)
		return -1;
	return v;
}
*/

/* selects a randomly sequence of at most 'size' vertices in valid route k, s.t. all vertices are valid;
	fromVertex is set to the first vertex of the segment (or sequence)
	toVertex is set to the last one
	len is set to the actual resulting number of vertices in the sequence (could be zero vertices !)
*/
void SolutionVRP::selectSegment(int k, int size, int& fromVertex, int& toVertex, int& len, bool zeroLength) {
	if (routeNfree[k] > 0) {								// if route k is non-empty and has still some non fixed customers
		int n_from = (rand() % routeNfree[k]) + 1; 
		int n_to;
		int max_len = min(size, routeNfree[k]-n_from+1);
		
		if(zeroLength) len = rand() % (max_len+1) ; 
		else len = rand() % max_len +1;

		// determine the segment
		n_to = n_from + len - 1;
		for (int v = firstFree[k]; v != k; v = next[v]) {	// find from_2
			n_from--;
			n_to--;
			if (n_from == 0) {
				fromVertex = v;
				if(len == 0) break;	// fromVertex must be determined even if len == 0
			} 
			if (n_from <= 0 && n_to == 0) {
				toVertex = v;
				break;
			} 
		}		
	}
	else
		len = 0;
}


/* Tells whether it is possible to insert a vertex before another vertex, w.r.t the constraints */
inline float SolutionVRP::getViolationsSegmentInsertion(int fromVertex, int toVertex, int before_i) {
	if(vehicle[fromVertex] == vehicle[before_i])	// if same route, delta is null
		return 0;

	float viol = 0;
	int segmentDemand = 0;
	for(int v=fromVertex; v!=next[toVertex]; v=next[v]) {
		segmentDemand += demand[v];
	}

	viol += (routeLoad[vehicle[before_i]]+segmentDemand > Q) - (routeLoad[vehicle[before_i]] > Q);	// before_i's route
	//viol += (routeLoad[fromVertex]-segmentDemand > Q) - (routeLoad[fromVertex] > Q);				// fromVertex's route
	return viol;
}


void SolutionVRP::insertVertex(int vertex, int before_i, bool remove) {
	/*
	if(remove == false) {						// if the vertex is unplanned
		unplannedValidSet.erase(vertex);
		plannedValidSet.insert(vertex);
	}*/
	insertSegment(vertex, vertex, before_i, remove, false);
}

void SolutionVRP::removeVertex(int vertex, bool notifyRouteChange) {
	ASSERT(vehicle[vertex] != 0, "error! trying to remove a non-planned vertex !");
	next[previous[vertex]] = next[vertex];
	previous[next[vertex]] = previous[vertex];
	if(notifyRouteChange)
		routeChange(vehicle[vertex]);
	vehicle[vertex] = 0;
	next[vertex] = -1;
	previous[vertex] = -1;

	//plannedValidSet.erase(vertex);
	//unplannedValidSet.insert(vertex);
}

/* moves a segment of vertices, beginning with vertex first to vertex last (first and last can be the same) */
/* if isswap==true, then we do not call routeChange, because a second moveSegment call will occur with the same two routes */
void SolutionVRP::insertSegment(int first, int last, int before_i, bool remove, bool isswap) {
	ASSERT(first >= NVeh+1 && last >= NVeh+1, "arghh");
	if (remove && pos[first] <= pos[before_i] && pos[before_i] <= pos[last]+1)
		return;
	//cout << "moveSegment: " << number[first] << " " << number[last] << " " << number[before_i] << " " << remove << " " << isswap << " " << endl << flush;
	int from_route = vehicle[first];

	if(remove) {
		ASSERT(!fixed[first], "first:" << number[first] << "maxRevealTime:"<< maxRevealTime[first]);
		ASSERT(!fixed[last], "last:" << number[last]);
	}
	ASSERT(!fixed[before_i], "before_i:" << number[before_i] << "-" << ts[before_i]);

	//if (pos[first] <= pos[before_i] && pos[before_i] <= pos[last]+1) return; // ---> bug

	if (remove) {	// removing from current position
		previous[next[last]] = previous[first];
		next[previous[first]] = next[last];
	}

	// inserting at new position
	previous[first] = previous[before_i];
	next[previous[before_i]] = first;
	previous[before_i] = last;
	next[last] = before_i;

	// updating route variables
	for(int curr=first; curr!=next[last]; curr=next[curr])
		vehicle[curr]=vehicle[before_i];


	// notify solution that some routes changed
	if(!isswap) {
		if (remove) routeChange(from_route); 
		if (from_route != vehicle[before_i]) 
			routeChange(vehicle[before_i]);
	}
}

/* inverts a vertex sequence begginning at 'first' and ending at 'last'
	e.g.  D-->A-->B-->C-->D-->E-->D, invertSegment(B,D) produces: D-->A-->D-->C-->B-->E-->D
*/
void SolutionVRP::invertSegment(int first, int last) {
	int prev_first = previous[first];
	int next_last = next[last];

	ASSERT(!fixed[first], "first:" << number[first]);
	ASSERT(!fixed[last], "last:" << number[last]);

	for(int v=first; v!=last; v=next[v]) {
		previous[v] = next[v];
	}
	int v1 = first;
	for(int v=next[first]; v!=last; v=previous[v]) {
		next[v] = v1;
		v1 = v;
	} next[last] = v1;

	next[prev_first] = last;
	previous[last] = prev_first;
	next[first] = next_last;
	previous[next_last] = first;

	routeChange(vehicle[first]);
}


inline double SolutionVRP::getCost(int k) {	// returns cost of route k (all route cost of k==0)
	float cost = 0;

	if(k == 0) {
		for(int r=1; r<NVeh+1; r++) 
			cost += routeCost[r];
	} 
	else
		return routeCost[k];

	return cost;
}

inline int SolutionVRP::getNbVehicle() {
	int n = 0;
	for(int r=1; r<NVeh+1; r++) 
			if(routeN[r] > 0) n++;	
	return n;
}

inline int SolutionVRP::getNbRequestsPlanned() {
	int n = 0;
	for(int r=1; r<NVeh+1; r++) 
			n += routeN[r];	
	return n;
}

inline float SolutionVRP::getProbaMassRequestsPlanned() {
	float p = 0;
	for(int i=NVeh+1; i<N+NVeh+1; i++) 
		if(vehicle[i] != 0 && !mandatory[i]) {
			p += (float)reqProba[i]/100;
		}	
	return p;
}


void SolutionVRP::updateRouteInfos(int k) {

	for (int r=1; r<NVeh+1; r++) {	

		if (k != 0 && k != r) continue;

		routeLoad[r] = 0;
		routeCost[r] = 0;
		routeN[r] = 0;
		firstFree[r] = 0;
		routeNfree[r] = 0;
		routeNFreeWaitingPlanned[r] = 0;
		routeNunleft[r] = 0;

		int i = next[r];
		int p = 1;
		while (i != r) {
			pos[i] = p; 
			routeLoad[r] += demand[i];
			routeCost[r] += c[previous[i]][i];
			routeN[r]++;
			if(!fixed[i]) {
				if(firstFree[r] == 0) firstFree[r] = i;
				routeNfree[r]++;
			}
			if(i >= NVeh+N+1 && !fixed[i])
				routeNFreeWaitingPlanned[r]++;
			if(!left[i])
				routeNunleft[r]++;
			i=next[i];
			p++;
		}
		pos[r] = p;
		routeCost[r] += c[previous[r]][r];
		if(firstFree[r] == 0 && !fixed[r]) {
			firstFree[r] = r;
		} 
		if(firstFree[r] == 0 && fixed[r]) {
			firstFree[r] = -1;
		}
	}
}


/* returns the number of violations of a given constraint; if vertex is set, then only the number of violations involving vertex */
inline float SolutionVRP::getViolations(int constraint, int vertex) {
	//if(constraint == ALL_CONSTRAINTS && vertex == 0)	// prefer invariant when possible
	//	return violations;
	return getSegmentViolations(constraint, vertex, vertex);
}		
inline float SolutionVRP::getSegmentViolations(int constraint, int fromVertex, int toVertex) {
	float v = 0;

	if(constraint == CAPACITY_CONSTRAINT || constraint == ALL_CONSTRAINTS) {
		if(fromVertex != 0) {
			return max(0, routeLoad[vehicle[fromVertex]] - Q);
		}

		for(int r=1; r<NVeh+1; r++) {	// should never be used, prefer invariant !
			//ASSERT(false, "should never be used, prefer invariant");
			if (routeLoad[r] > Q) v += routeLoad[r] - Q;
			//if (routeLoad[r] > Q) v++;
		}
	}
	return (v);
}


string& SolutionVRP::toString() {
	ostringstream out;
	out.setf(std::ios::fixed);
	out.precision(2);
	
	//out << "Capacity violations: " << getViolations(1) << endl;
	if (getViolations() > 0) out << "Infeasible ! ";
	out << "Cost=" << getCost() << " \n";
	//out << "Cost=" << getCost()-10000*getNbVehicle() << " \n";
	for(int r=1; r<NVeh+1; r++) {
		out << "Route " << setw(2) << r << " (c:" << setw(7) << routeCost[r]
			<< " q:" << outputMisc::redExpr(routeLoad[r]>Q) << setw(4) << routeLoad[r] << outputMisc::resetColor() << "):   D ";
		for(int i=next[r]; i!=r; i=next[i])
			out << setw(4) << number[i];
		out << "   D\n";
	}
	static string str = ""; str = out.str();
	return (str);
}


/* test if hard constraints are respected */
void SolutionVRP::checkSolutionConsistency() {

	// a customer is active IFF he's assigned to one vehicle
	for (int i=NVeh+1; i<NVeh+N+1; i++) {
		ASSERT(!mandatory[i] || active[i], "i="<<number[i]<<" vehicle[i]="<<vehicle[i]); 								// mandatory --> active
		ASSERT(!mandatory[i] || (vehicle[i] >= 1 && vehicle[i] <= NVeh), "i="<<number[i]<<" vehicle[i]="<<vehicle[i]);  // mandatory --> assigned
		ASSERT(active[i] || !(vehicle[i] >= 1 && vehicle[i] <= NVeh), "i="<<number[i]<<" vehicle[i]="<<vehicle[i]); 	// assigned <-- active	
	}

	// each customer must be serviced exactly once IFF it is active
	int *serviceCount = new int[NVeh+N+1+NWaiting];				// online new  ---> highly time consuming !!
	for (int i=1; i<NVeh+N+1+NWaiting; i++) serviceCount[i] = 0;
	for(int r=1; r<NVeh+1; r++) 
		for(int i=next[r]; i!=r; i=next[i]) {
			serviceCount[i]++; 
			ASSERT(i >= NVeh+1, "argh");
			ASSERT(vehicle[i] == r, "Error: " << number[i]<<"-"<<ts[i] << "thinks to be on another route ("<<vehicle[i]<<") than "<<r<<"!!");
		}
	for(int i=NVeh+1; i<NVeh+N+1+NWaiting; i++) {
		ASSERT(serviceCount[i] <= 1, "serviceCount["<< number[i] <<"]=" << serviceCount[i]);
		ASSERT(!mandatory[i] || serviceCount[i] == 1, "serviceCount["<< number[i] <<"]=" << serviceCount[i]); // mandatory --> serviced
		ASSERT(active[i] || serviceCount[i] == 0, "serviceCount["<< number[i] <<"]=" << serviceCount[i]); // serviced --> active
	}	
	delete [] serviceCount;
}

/* dynamic context: 
 * depending on the current time, activate one non-activated request (returns false if none found)
 */
int SolutionVRP::activateOneMandatoryRequest(float currtime) {
	int r = 0;
	// activate the sooner vertex

	int minReqTime = numeric_limits<int>::max();
	for (int i=1; i<NVeh+1+N; i++)
		if (reqTime[i] <= currtime && reqTime[i] < minReqTime) {
			if ((active[i] && mandatory[i]) || rejected[i]) continue;
			else {
				r = i;						// notify the calling process that a there actually was a customer to activate
				minReqTime = reqTime[i];

			}
		}
	activateMandatoryRequest(r);
	return r;
}

inline void SolutionVRP::activateMandatoryRequest(int req) {
	active[req] = true;				// activating
	mandatory[req] = true;			// say the request is mandatory, i.e. it MUST be part of the plan (noy only activated)
}

/* dynamic context: 
 * activates all the requests having a non-zero probability to appear (returns the number of activated ones)
 */
int SolutionVRP::activateAllPossibleRequests(float currtime) {		// used only for optional vertices (! not waiting vertices, nor mandatory ones)
	int r = 0;
	// activate vertices
	for (int i=1; i<NVeh+1; i++)
		active[i] = true;
	for (int i=NVeh+1; i<NVeh+1+N; i++)	{
		if (currtime <= maxRevealTime[i]) {
		//if (reqProba[i] > 0 && currtime <= maxRevealTime[i]) {
			if(ts[i] == 0)
				active[i] = reqTime[i] <= currtime;
			else active[i] = true;
			//unplannedValidSet.insert(i);
			r++;
		}
		mandatory[i] = false;
	}
	return r;
}

void SolutionVRP::activateAllWaitingRequests() {
	for (int i=NVeh+N+1; i<NVeh+N+1+NWaiting; i++)	
		active[i] = true;
}

int SolutionVRP::desactivateNonPlannedOptionalRequests() {
	int nbPlannedOptional = 0;
	for (int i=NVeh+1; i<NVeh+1+N; i++)	{
		if (!mandatory[i]) {	// if i is optional
			if (vehicle[i]!=0)	// if i is planned
				nbPlannedOptional++;
			else				// if i is unplanned
				active[i] = false;
		}
	}
	return nbPlannedOptional;
}

// rejects one request: remove the associated vertices from the plan, desactivates it, updates counters
void SolutionVRP::rejectRequest(int v, bool remove) {	
	if(remove && vehicle[v]!=0)
		removeVertex(v);

	active[v] = false;
	mandatory[v] = false;	// optional, since active is more restrictive
	//unplannedValidSet.erase(v);
	rejected[v] = true;
	routeChange();
}


void SolutionVRP::removeOptionalRequests() {
	for(int i=NVeh+1; i<NVeh+N+1; i++) {
		if(!mandatory[i] && vehicle[i]!=0 && !fixed[i] && active[i])
			removeVertex(i, false);
	}
	routeChange();
}


void SolutionVRP::fixSolution(float currtime) {
	// TODO !
}


void SolutionVRP::logWaitingVertices() {
	int *serviceCount = new int[NVeh+NVertices+1];			
	for (int i=1; i<NVeh+NVertices+1; i++) serviceCount[i] = 0;
	for(int r=1; r<NVeh+1; r++) 
		for(int i=next[r]; i!=r; i=next[i])
			serviceCount[number[i]]++; 

	ofstream myfile;
	myfile.open ("output_graph.txt");

	myfile << "-1" << "\t" << coordX[1] << "\t" << coordY[1] << "\t3" << endl;
	for (int v = NVeh+1; v < NVeh+NVertices+1; v++) {
		myfile << v << "\t" << coordX[v] << "\t" << coordY[v] << "\t" << serviceCount[v] << endl;
	}
	myfile.close();
}


