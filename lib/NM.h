
#include <algorithm>    // std::swap, std::max
#include <limits>    // limits

#include "../lib/tools.h"	

#ifndef NM_H
#define	NM_H


using namespace std;

extern int N;			// Number of customer verticesmatrix
extern int NVeh;		// Number of depot vertices
extern double **c;		// Arc costs 
extern int NWaiting;	// number of waiting fake requests, i.e. vertices where the vehicle can go in order to wait for possible sorrounding requests
extern int nbNonPlanned;// Number of non planned requests (which are active and non mandatory)

#define	K				9	// number of neighborhoods
#define	K_SIZE			5


#define	INSERT_OPTIONAL_REQ 		1
#define	REMOVE_OPTIONAL_REQ 		2

#define ADD_WAITING_TIME 			3
#define REMOVE_WAITING_TIME 		4

#define	EXCHANGE 					5
#define	RELOCATE_BEST_IMPROVEMENT	6


#define TWO_EXCHANGE				7 	// quite bad
#define	RELOCATE_WORST_VIOLATIONS	8 	// bad


#define	CROSS_EXCHANGE				9   // must stay the last one ! because of size

template <class S> class neighborhoodManager {
protected:
	int k = 1, k_size = 1, success[K+1]={0};

	S *solution;
	int *prev;		// previous[i] = j iff vertex j is visited before i in the same route
	int *next;		// !!! -> Numbered  [NVeh+1..NVeh+N]
	int *veh;		// vehicle[i] = j iff vertex j is serviced by vehicle i
	int *routeNfree;
	int *firstFree;
	int *pos;
	void localSearch(int k);
	void best_relocate();
	void relocate_best_improvement();
	void relocate_worst_violations();
	void exchange();
	void two_exchange();
	void cross_exchange(int size);
	//void insert_optional_req(int k_size);
	void insert_waiting_req(int k_size);
	//void remove_optional_req(int k_size);
	void remove_waiting_req(int k_size);
	void add_waiting_time(int k_size);
	void remove_waiting_time(int k_size);
public:
	neighborhoodManager(S* solution);
	virtual void intensify();
	virtual void diversify();
	virtual void printNeighborhoodScores();
	virtual void neighborAccepted();
	virtual void shakeSolution();
	void localSearch_3opt(int k, int size);
};

// applying adaptative algorithm as in "An Adaptative LNS heuristic for the PDPTW" - Ropke, Pisinger, 2005
#define 	MAX_SEGMENT_SIZE 	100
#define		REACTION_FACTOR 	0.1
template <class S> class neighborhoodManager_Adaptative : public neighborhoodManager<S> { 
 	public:
	double score[K+K_SIZE]		= {1};
	int segSuccess[K+K_SIZE]	= {0};
	int attempt[K+K_SIZE]		= {0};
	int segAttempt[K+K_SIZE]	= {0};
	int segment = 0;
	int last_size;

	neighborhoodManager_Adaptative(S* solution) : neighborhoodManager<S>(solution) {
		for(int k_=1; k_<K+K_SIZE; k_++) {
			score[k_] = 1;
			attempt[k_] = 0;
			segSuccess[k_] = 0;
			segAttempt[k_] = 0;
		}
	}
	virtual void shakeSolution() {
		int chance = rand() % 10000000 + 1;

		double cumul = 0;
		int k_;

		double tot_score = 0;
		for(k_=1; k_<K+K_SIZE; k_++)
			tot_score += score[k_];
		ASSERT(tot_score > 0, "argh");

		for(k_=1; k_<K+K_SIZE; k_++){				// choose the neighborhood to select, wrt their scores
			cumul += score[k_] / tot_score   * 10000000 ;
			if(chance <= cumul)
				break;
		}
		ASSERT(k_ < K+K_SIZE, "argh! no neighborhood selected !");

		this->k = min(K, k_);
		if(k_ > K_SIZE) 
			this->k_size = k_ - K_SIZE + 1;
		else 
			this->k_size = 1;

		last_size = this->k_size;
		attempt[k_]++;
		segAttempt[k_]++;
		neighborhoodManager<S>::shakeSolution();


		// at the end of a segment, update the scores and reinitialize the segment variables
		segment++;
		if(segment >= MAX_SEGMENT_SIZE) {			
			for(k_=1; k_<K+K_SIZE; k_++) {
				score[k_] = score[k_]*(1-REACTION_FACTOR) + REACTION_FACTOR * segSuccess[k_] / max((int)1, segAttempt[k_]);
				segAttempt[k_] = 0;
				segSuccess[k_] = 0;
				segment = 0;
			}

			cout << endl;
			for(int k_=1; k_<K+K_SIZE; k_++)
				cout << "\ts[" << k_ << "]:" << score[k_];

		}
	}
	virtual void neighborAccepted() {
		if(this->k < K) {
			this->success[this->k]++;
			segSuccess[this->k]++;
		} 
		else {
			this->success[this->k+last_size-1]++;
			segSuccess[this->k+last_size-1]++;
		}
		//cout << "neighbor accepted: " << k << endl << flush;
	}
	virtual void printNeighborhoodScores() {
		cout << endl;
		for (int k_=1; k_<K+K_SIZE; k_++) 
			cout << "\ts["<< min(k_, K) << "] = " << score[k_];
		cout << endl;
	}
};


/* implementation ---------------------------------------------------------(in header because of template)- */

template <class S>
neighborhoodManager<S>::neighborhoodManager(S* solution){
	this->solution = solution;
	prev = solution->getpPrevious();
	next = solution->getpNext();
	veh = solution->getpVehicle();
	routeNfree = solution->getpRouteNfree();
	firstFree = solution->getpFirstFree();
	pos = solution->getpPos();}

template <class S>
void neighborhoodManager<S>::shakeSolution(){
	//cout << "N" << k << " " << flush;
	// first check whether there is something to do, i.e. if there is at least one customer we can try to move
	int availableCust = 0;	
	/*
	for(int v=1; v<NVeh+1; v++)
		if(routeNfree[v] >=1)
			availableCust += routeNfree[v];
	if(availableCust == 0)
		return;
	*/
	switch (k) {
		case RELOCATE_BEST_IMPROVEMENT:
			relocate_best_improvement();
			break;
		case RELOCATE_WORST_VIOLATIONS:
			relocate_worst_violations();
			break;
		case EXCHANGE:
			exchange();
			break;
		case TWO_EXCHANGE:
			two_exchange();
			break;
		case CROSS_EXCHANGE:
			cross_exchange(k_size);
			break;
		case INSERT_OPTIONAL_REQ:
			//insert_optional_req(k_size);
			insert_waiting_req(k_size);
			break;
		case REMOVE_OPTIONAL_REQ:
			//remove_optional_req(k_size);
			remove_waiting_req(k_size);
			break;
		case ADD_WAITING_TIME:
			add_waiting_time(k_size);
			break;
		case REMOVE_WAITING_TIME:
			remove_waiting_time(k_size);
			break;
	}
}


template <class S> void neighborhoodManager<S>::intensify() {
	k = 1; 
	k_size = 1;
}
template <class S> void neighborhoodManager<S>::diversify() {
	k = k%K + 1;
	if(k == 1) 
		k_size = k_size%K_SIZE + 1;
}
template <class S> void neighborhoodManager<S>::neighborAccepted() {
	success[k]++;
	//cout << "neighbor accepted: " << k << endl << flush;
}
template <class S> void neighborhoodManager<S>::printNeighborhoodScores() {
	for (int k=1; k<K+1; k++) 
		cout << "success["<< k << "] = " << success[k] << endl;
}


template <class S>
void neighborhoodManager<S>::best_relocate() {
	int vertex = -1, before_i;
	float delta, max_delta = numeric_limits<float>::min();
	// SELECT the vertex involving the most violations and the worst cost
	for (int v=NVeh+1; v<NVeh+N+1; v++) {
		if (!solution->isActive(v) || veh[v]==0 || solution->isFixed(v)) continue;
		float viol = solution->getViolations(0, v);
		float cost = c[prev[v]][v] + c[v][next[v]] - c[prev[v]][next[v]];
		if(viol*10000+cost > max_delta) {
			vertex = v;
			max_delta = viol*10000+cost;
		}
	}
	if(vertex == -1) return;

	// MOVE it
	before_i = solution->bestInsertion(vertex, delta, 1.0);
	//if(delta < 0)
	solution->insertVertex(vertex, before_i, true);
}


/* 	Relocate Best Improvement: the simpliest neighborhood operator
	Moves ONE random vertex from its position to a :
		1) random different position (10%)
		2) best insert (90%)
	Implementation status: Fixed OK
	Todo:
*/
template <class S>
void neighborhoodManager<S>::relocate_best_improvement() {
	int vertex, before_i;
	float delta;
	// SELECT the vertex
	vertex = solution->select_Planned_Vertex();
	if(vertex == -1)
		return;
	// MOVE it
	int chance = rand() % 100 + 1;
	if (chance <= 10) {	// 1)
		before_i = solution->randomInsertion(vertex);
	} else {			// 2)
		before_i = solution->bestInsertion(vertex, delta, 1.0); 	// find the best position to re-insert it		
	}
	if (before_i == -1)
		return;
	solution->insertVertex(vertex, before_i, true);
}

/* 	relocate worst violations: variation of Relocate
	Moves THE VERTEX INVOLVING THE MOST VIOLATIONS AND THE WORST COST from its position to a :
		1) random different position (10%)
		2) best insert (90%)
	Implementation status: Fixed OK
	Todo:
*/
template <class S>
void neighborhoodManager<S>::relocate_worst_violations() {
	int vertex, before_i;
	float delta;
	
	vertex = solution->select_WorstViolations_Vertex();
	if(vertex == -1) 
		return;

	// MOVE it
	int chance = rand() % 100 + 1;
	if (chance <= 10) {	// 1)
		before_i = solution->randomInsertion(vertex);
	} else {			// 2)
		before_i = solution->bestInsertion(vertex, delta, 1.0); 	// find the best position to re-insert it		
	}
	if (before_i == -1)
		return;
	solution->insertVertex(vertex, before_i, true);
}


/* 	Exchange : from paper "A two-stage hybrid algorithm for the VRPTW", Bent & Van Hentenryck 
	exchange the positions of two vertices
	Implementation status: Fixed OK
	Todo:
*/
template <class S>
void neighborhoodManager<S>::exchange() {
	int availableCust = 0;
	for(int v=1; v<NVeh+1; v++)
		if(routeNfree[v] >=1)
			availableCust += routeNfree[v];
	if(availableCust < 2)
		return;
	int v1, v2;
	// SELECT two different vertices
	
	if((v1 = solution->select_Planned_Vertex()) == -1)
		return;

	do	 
		v2 = solution->select_Planned_Vertex();
	while (v1 == v2);
	

	solution->insertSegment(v1, v1, v2, true, false);
	solution->insertSegment(v2, v2, v1, true, false);
}


/* 	Two-exchange (2-opt with seq inversion) : from paper "A two-stage hybrid algorithm for the VRPTW", Bent & Van Hentenryck 
	Implementation status: Fixed OK
	Todo:
*/
template <class S>
void neighborhoodManager<S>::two_exchange() {
	int route, v1, v2;
	// check whether there are still at least one route with minimum three non-fixed customer vertices (otherwise it has no effect)
	int availableVeh = 0;
	for(int r=1; r<NVeh+1; r++) if(routeNfree[r] >=3) availableVeh++;
	if(availableVeh == 0) return;

	// choose a proper route
	do route = (rand() % NVeh) + 1; while (!solution->isActive(route) || solution->isFixed(route) || routeNfree[route]<3);	
	// choose two valid different vertices v1 and v2 in that route, s.t. they are not consecutive
	do {
		v1 = solution->select_Planned_Vertex(route);
		v2 = solution->select_Planned_Vertex(route); 
	} while(v1==v2 || pos[v1]-pos[v2] == 1 || pos[v1]-pos[v2] == -1);	
	if(pos[v1] > pos[v2]) {	// ensure v1 serviced before v2
		int tmp = v1;
		v1 = v2; v2 = tmp;
	}

	// perform the txo-exchange operation
	solution->invertSegment(v1, v2);
}





/* 	Cross-exchange : based on paper "A variable neighborhood search for the MDVRPTW", Polacek, Hartl and Doerner
	Exchanges two segments of maximum sizes 'size' between two different routes; a size can be zero; 
	Implementation status: Fixed OK
	Todo: move of one segment within the same route (the so-called OR-EXCHANGE)
*/
template <class S>
void neighborhoodManager<S>::cross_exchange(int size) {
	int from_1=-1, to_1, from_2=-1, to_2, r1, r2, len1=0, len2=0;

	// check whether there are still at least two available vehicles
	int availableVeh = 0;			
	for(int r=1; r<NVeh+1; r++) 
		if(solution->isActive(r) && !solution->isFixed(r)) 
			availableVeh++;
	if(availableVeh < 2) return;
	
	do r1 = (rand() % NVeh) + 1; while (!solution->isActive(r1) || solution->isFixed(r1));				// choose first route
	do r2 = (rand() % NVeh) + 1; while (r1 == r2 || !solution->isActive(r2) || solution->isFixed(r2));	// choose second DIFFERENT route	

	// select segments in both routes
	solution->selectSegment(r1, size, from_1, to_1, len1);
	solution->selectSegment(r2, size, from_2, to_2, len2);
	ASSERT(from_1 <= NVeh+N+NWaiting, "from_1="<<from_1);
	ASSERT(from_2 <= NVeh+N+NWaiting, "from_2="<<from_2);
	ASSERT(len1==0 || to_1<=NVeh+N+NWaiting, "to_1="<<to_1);
	ASSERT(len2==0 || to_2<=NVeh+N+NWaiting, "to_2="<<to_2);

	// moving segments
	int before_i_r1, before_i_r2;

	if(routeNfree[r1] == 0) before_i_r1 = r1;
	else if(len1 > 0) before_i_r1 = next[to_1]; 
	else before_i_r1 = from_1;

	if(routeNfree[r2] == 0) before_i_r2 = r2;
	else if(len2 > 0) before_i_r2 = next[to_2]; 
	else before_i_r2 = from_2;
	
	if(len1 > 0) solution->insertSegment(from_1, to_1, before_i_r2, true, false);
	if(len2 > 0) solution->insertSegment(from_2, to_2, before_i_r1, true, false);
}



/* A local search method operating on a route basis (route k)
	the method is a bounded version of the 3-opt operator; 
	the method is taken from "A variable neighborhood search for the MDVRPTW", Polacek, Hartl and Doerner
*/	// maybe flawed !!!! no real tests
template <class S>
void neighborhoodManager<S>::localSearch_3opt(int k, int size) {
	int best_before_i, best_fromVertex, best_toVertex;
	float delta, min_delta;
	for(int i=0; i<10; i++) {
		min_delta = numeric_limits<float>::max();
		for(int fromVertex=firstFree[k]; fromVertex!=k; fromVertex=next[fromVertex]) {	// for each customer vertex in route k
			for(int s=1; s<=size; s++) {		// for each size							
				int toVertex = fromVertex;
				for(int l=s-1; l>0; l--)
					if (next[toVertex] == k) break;
					else toVertex = next[toVertex];
				int before_i = solution->bestSegmentInsertion(fromVertex, toVertex, delta, 1.0, k);
				if (before_i == -1)
					return;
				if (delta < min_delta && before_i > 0) { 
					min_delta = delta;
					best_fromVertex = fromVertex;
					best_toVertex = toVertex;
					best_before_i = before_i;
				}
			}
		}
		//cout << "insertSegment:" << best_fromVertex << " " << best_toVertex << " " << best_before_i << endl << flush;
		if (min_delta >= 0) break;
		solution->insertSegment(best_fromVertex, best_toVertex, best_before_i, true, false);
		//cout << min_delta << endl;
	} 
}




/* 	Insert Optional Request: 
	Insert k_size random *non planned* request at :
		1) random position (10%)
		2) best insert (90%)
	Todo:
*/
template <class S>
//void neighborhoodManager<S>::insert_optional_req(int k_size) {
void neighborhoodManager<S>::insert_waiting_req(int k_size) {
	int vertex, before_i;
	float delta;
	int chance = rand() % 100 + 1;

	//for(int i=0; i<k_size; i++) {
		// SELECT the vertex
		//vertex = solution->select_Unplanned_Vertex();
		vertex = solution->select_Waiting_Unplanned_Vertex();
		if(vertex == -1)
			return;
		else
			ASSERT(veh[vertex]==0, "vertex = "<<vertex);
		// MOVE it
		if (chance <= 10) {	// 1)
			before_i = solution->randomInsertion(vertex);
		} else {			// 2)
			before_i = solution->bestInsertion(vertex, delta, 1.0); 	// find the best position to re-insert it		
		}
		if (before_i == -1)
			return;
		solution->insertVertex(vertex, before_i, false);
	//}
}


/* 	Remove Optional Request: 
	Remove k_size random *non planned* request from :
		1) random position (100%)
		2) worst violation position (0%)
	Todo: 2)
*/
template <class S>
//void neighborhoodManager<S>::remove_optional_req(int k_size) {
void neighborhoodManager<S>::remove_waiting_req(int k_size) {
	int vertex;

	//for(int i=0; i<k_size; i++) {
		// SELECT the vertex
		//vertex = solution->select_Planned_Optional_Vertex();
		vertex = solution->select_Waiting_Planned_Vertex();

		if(vertex == -1)
			return;
		else
			ASSERT(veh[vertex]!=0 && vertex >= NVeh+N+1, "vertex = "<<vertex);
			
		solution->removeVertex(vertex);
	//}
}



template <class S>
void neighborhoodManager<S>::add_waiting_time(int k_size) {
	int vertex;
	float remainingWaitingTime;

	//for(int i=0; i<k_size; i++) {
		// SELECT the vertex
		vertex = solution->select_Unleft_Vertex();

		if(vertex == -1)
			return;
		else
			ASSERT(veh[vertex]!=0, "vertex = "<<vertex);
		
		float increment = (rand() % 100 + 1) / 10;
		solution->increaseWaitingTime(vertex, increment);
	//}
}



template <class S>
void neighborhoodManager<S>::remove_waiting_time(int k_size) {
	int vertex;
	float remainingWaitingTime;

	//for(int i=0; i<k_size; i++) {
		// SELECT the vertex
		vertex = solution->select_Unleft_Vertex();
		remainingWaitingTime = solution->computeRemainingWaitingTime(vertex);

		if(vertex == -1)
			return;
		else
			ASSERT(veh[vertex]!=0, "vertex = "<<vertex);
		
		float decrement = (rand() % 100 + 1) / 10;
		decrement = min(remainingWaitingTime, decrement);
		solution->decreaseWaitingTime(vertex, decrement);
	//}
}


#endif