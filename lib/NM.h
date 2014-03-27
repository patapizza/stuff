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

	vertex = rand() % N + (NVeh + 1); // generate random number in [NVeh+1..NVeh+N] to select one customer vertex
	from_route = veh[vertex];		// remember the route

	int chance = rand() % 100 + 1;
	if (chance < 10) {	// 1)
		do {
			before_i = rand() % (NVeh + N) + 1; // generate random number in [1..NVeh+N]
		} while (before_i == vertex || before_i == next[vertex]);
	} else {			// 2)
		float min_delta = numeric_limits<float>::max();
		for (int i=1; i<NVeh+N+1; i++) {	// find the best position to re-insert it
			float delta = 0.0;
			if (i == vertex || i == next[vertex]) continue;
			delta =   c[prev[i]][i] 
					+ c[prev[i]][vertex]
					+ c[vertex][i];
			if (delta < min_delta) {
				before_i = i;
				min_delta = delta;
			} 
		} 
	}
	//cout << "moving " << vertex-NVeh << " to before " << before_i-NVeh << endl;
	// Remove from current position
	prev[next[vertex]] = prev[vertex];
	next[prev[vertex]] = next[vertex];

	// Insert elsewhere
	veh[vertex] = veh[before_i];
	next[vertex] = before_i;
	prev[vertex] = prev[before_i];
	prev[before_i] = vertex;
	next[prev[vertex]] = vertex;

	// notify solution that some routes changed
	solution->routeChange(veh[from_route]); 
	if (veh[from_route] != veh[before_i]) 
		solution->routeChange(veh[before_i]);
}


#endif