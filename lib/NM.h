//#include "VRPlib.h"

#ifndef NM_H
#define	NM_H


using namespace std;

extern int N;			// Number of customer verticesmatrix
extern int NVeh;		// Number of depot vertices
extern double **c;		// arc costs 


template <class S> class neighborhoodManager {
private:
	S *solution;
	int *prev;		// previous[i] = j iff vertex j is visited before i in the same route
	int *next;		// !!! -> Numbered  [NVeh+1..NVeh+N]
	int *veh;		// vehicle[i] = j iff vertex j is serviced by vehicle i
public:
	neighborhoodManager(S* solution);
	~neighborhoodManager();
	void shakeSolution();
};


/* implementation ---------------------------------------------------------(in header because of template)- */

template <class S>
neighborhoodManager<S>::neighborhoodManager(S* solution){
	this->solution = solution;
	prev = solution->getpPrevious();
	next = solution->getpNext();
	veh = solution->getpVehicle();
}

template <class S>
neighborhoodManager<S>::~neighborhoodManager(){

}

template <class S>
void neighborhoodManager<S>::shakeSolution(){
	/* Move a random vertex from its position to a 
			1) random different position (10%)
			2) best insert (90%)
	*/
	int vertex, before_i, from_route;

	// SELECT the vertex
	do	 // generate random number in [NVeh+1..NVeh+N] to select one ACTIVE customer vertex
		vertex = (rand() % N) + NVeh+1;
	while (!solution->isActive(vertex));
	from_route = veh[vertex]; 

	// MOVE it
	int chance = rand() % 100 + 1;
	if (chance < 10) {	// 1)
		do {
			before_i = rand() % (NVeh + N) + 1; // generate random number in [1..NVeh+N]
		} while (before_i == vertex || before_i == next[vertex] || !solution->isActive(before_i));
	} else {			// 2)
		before_i = solution->bestInsertion(vertex); 	// find the best position to re-insert it		
	}


	solution->insertVertex(vertex, before_i, true);

}



#endif