#include <iostream>
#include <fstream>	// file I/O
#include <sstream>	// ostringstream
#include <cstdlib>	// rand
#include <ctime>	// time
#include <cmath>	// sqrt
#include <cstring>	// memcpy
#include <limits>   // numeric_limits
#include <iomanip>	// setw, setfill
#include <algorithm>// std::max
#include <limits> 	// numeric_limits

#include "VRPlib.h"
#include "tools.h"

using namespace std;


/* Problem data variables */
extern int N;			// number of customer requests, numbered: [NVeh+1..NVeh+N]
extern int NVeh;		// number of vehicles, numbered: [1..NVeh]
extern int Q;			// vehicle max capacity
extern double **c;		// travel costs/durations
extern float *coordX; 
extern float *coordY;
extern float *demand;	// demand et each vertex
extern int 	*number;	// number of the customer (in the test instance)
float *duration;	// service duration at each vertex
int   *e;			// start of time window at each vertex
int   *l;			// end of time window at each vertex


// Dynamic context
extern int 	*reqTime;	// time at which the request is revealed (dynamic context)
extern int 	*ts;			// time slot of the request
extern int NVertices;			// number of customer vertices (different from N, because of time slots, there can be several requests per vertex)
extern int NWaiting;			// number of waiting fake requests, i.e. vertices where the vehicle can go in order to wait for possible sorrounding requests
extern int nTimeSlots;			// number of time bins
extern int *reqProba;			// probabitlity for that request to appear online (for a given time slot)
extern float *maxRevealTime;	// according to the time slot, latest time a vehicle can depart from the depot, serve the vertex 
								// and return to depot (lambda_c in Bent&VanHentenryck's paper)
								// defines an upper bound on the time at which a request could be revealed
extern float *minRevealTime;	// lower bound on which a request can be revealed
extern bool	*rejected;		// tells whether a request has been rejected

/*
inline std::ostream &operator << (std::ostream &out_file, solutionTSP& s) {
	if (s.getViolations() > 0) out_file << "Infeasible ! ";
	out_file << "Cost=" << s.getCost() << " ";
	for(int i=1; i<N+1; i++)
		out_file << s.step[i] << " ";
	return (out_file);
}*/


/* class SolutionVRPTW  ********************************************************************************************/

SolutionVRPTW::SolutionVRPTW() {									// constructor ***
	h = new float[NVeh+N+1+NWaiting];		// arrival times for every vehicle depot and customer vertices
	b = new float[NVeh+N+1+NWaiting];		// service times for every vehicle depot and customer vertices
	w = new float[NVeh+N+1+NWaiting];		// waiting time at every vehicle depot and customer vertices
	for (int i=1; i<NVeh+N+1+NWaiting; i++)
		w[i] = 0.0;

	current_time = 0;
}
SolutionVRPTW::SolutionVRPTW(const SolutionVRPTW& old_solution) : SolutionVRPTW() {		// copy constructor ***
	*this = old_solution;
}
SolutionVRPTW& SolutionVRPTW::operator = (const SolutionVRPTW& sol) {
	SolutionVRP::operator = (sol);
	memcpy(h, sol.h, (NVeh+N+1+NWaiting)*sizeof(float));
	memcpy(b, sol.b, (NVeh+N+1+NWaiting)*sizeof(float));
	memcpy(w, sol.w, (NVeh+N+1+NWaiting)*sizeof(float));
	return (*this);
}
SolutionVRPTW::~SolutionVRPTW() {										// destructor ***
	delete [] h;
	delete [] b;
	delete [] w;
}



inline float SolutionVRPTW::getViolationsSegmentInsertion(int fromVertex, int toVertex, int before_i) {
	float viol = SolutionVRP::getViolationsSegmentInsertion(fromVertex, toVertex, before_i);

	int r = vehicle[before_i];

	int prev = r;
	float prev_stime = e[r];
	float atime, stime;
	// vertices on the route
	for (int v = next[r]; v != r; v = next[v]) {
		if(v == fromVertex) {
			v = toVertex;
			continue;	// switch to next[toVertex] and continue
		}

		if(v == before_i) {
			for(int i=fromVertex; i!=next[toVertex]; i=next[i]) {
				atime = prev_stime + duration[prev]*mandatory[prev] + w[prev] + c[prev][i];
				stime = max((float) e[i], atime);

				if(stime > l[i] + 0.0001)		// is the new insertion violating the current position ?
					viol += (stime - l[i]);	

				prev_stime = stime;
				prev = i;
				if(i == toVertex)
					break;
			}
		}

		atime = prev_stime + duration[prev]*mandatory[prev] + w[prev] + c[prev][v];
		if (!served[v])	// if request already served, service time must remain the same
			stime = max((float) e[v], atime);
		else 
			stime = b[v];
		viol += max((float)(stime - (l[v]+0.0001)), (float) 0) - max((float)(b[v] - (l[v]+0.0001)), (float) 0);
		prev_stime = stime;
		prev = v;
	}
	// depot
	if(before_i == r) {
		for(int i=fromVertex; i!=next[toVertex]; i=next[i]) {
			atime = prev_stime + duration[prev]*mandatory[prev] + w[prev] + c[prev][i];
			stime = max((float) e[i], atime);

			if(stime > l[i] + 0.0001)		// is the new insertion violating the current position ?
				viol += (stime - l[i]);	

			prev_stime = stime;
			prev = i;
			if(i == toVertex)
				break;
		}
	}
	atime = prev_stime + duration[prev]*mandatory[prev] + w[prev] + c[prev][r];
	if (!served[r])	// if request already served, service time must remain the same
		stime = max((float) e[r], atime);
	else 
		stime = b[r];
	viol += max((float)(stime - (l[r]+0.0001)), (float) 0) - max((float)(b[r] - (l[r]+0.0001)), (float) 0);

	//cout << viol << endl;
 	return viol;
}


/* Tells whether it is possible to insert a vertex before another vertex, w.r.t the constraints */
/*
inline float SolutionVRPTW::getViolationsSegmentInsertion(int fromVertex, int toVertex, int before_i) {
	float viol = SolutionVRP::getViolationsSegmentInsertion(fromVertex, toVertex, before_i);

	int r = vehicle[before_i];

	// compute the TW violations created by the segment insertion in the route
	float arrival_time = b[previous[before_i]] + duration[previous[before_i]]*mandatory[previous[before_i]] + c[previous[before_i]][fromVertex];
	float service_time = max((float) e[fromVertex], arrival_time);

	for(int v=fromVertex; v!=next[toVertex]; v=next[v]) {
		if(service_time > l[v] + 0.0001)		// is the new insertion violating the current position ?
			viol += (service_time - l[v]);		
		if(v != toVertex) {
			arrival_time = service_time + duration[v]*mandatory[v] + c[v][next[v]];
			service_time = max((float) e[next[v]], arrival_time);
		}
	}

	float arrival_time_before_i = service_time + duration[toVertex]*mandatory[toVertex] + c[toVertex][before_i];
	float service_time_before_i = max((float) e[before_i], arrival_time_before_i);

	
	if (service_time_before_i > l[before_i] + 0.0001)	// does it makes before_i violated ?
		viol += (service_time_before_i - l[before_i]) - (b[before_i] - l[before_i]);	

	if(before_i != r) {
		float atime = service_time_before_i + duration[before_i]*mandatory[before_i] + c[before_i][next[before_i]];
		float stime = max((float) e[next[before_i]], atime);
		for(int v=next[before_i]; v!=r; v=next[v]) {
			if (stime > l[v] + 0.0001)
				viol += (stime - l[v]) - (b[v] - l[v]);
			atime = stime + duration[v]*mandatory[v] + c[v][next[v]];
			stime = max((float) e[next[v]], atime);
		} 
		if ((stime > l[r] + 0.0001))
			viol += (stime - l[r]) - (b[r] - l[r]);
	}
	return viol;
}
*/

void SolutionVRPTW::removeVertex(int vertex, bool notifyRouteChange) {
	SolutionVRP::removeVertex(vertex, notifyRouteChange);
	w[vertex] = 0.0;
}


// recomputes load, arrival and service times at route k (k==0: every routes)
inline void SolutionVRPTW::updateRouteInfos(int k) {	
	SolutionVRP::updateRouteInfos(k);

	for (int r=1; r<NVeh+1; r++) {
		if (k != 0 && k != r) continue;


		// Compute arrival and service times upon route k --> basic phase
		h[r] = 0;
		b[r] = 0;
		int i = next[r];
		while (i != r) {
			//h[i] = b[previous[i]] + duration[previous[i]]*mandatory[previous[i]] + c[previous[i]][i];
			h[i] = b[previous[i]] + duration[previous[i]]*mandatory[previous[i]] + w[previous[i]] + c[previous[i]][i];	// waiting version
			if (!served[i])	// if request already served, service time must remain the same
				b[i] = max((float) e[i], h[i]);
			ASSERT(active[i], "argh");
			i = next[i];
		} 
		//h[r] = d[previous[r]] + duration[previous[r]]*mandatory[previous[r]] + c[previous[r]][r];
		h[r] = b[previous[r]] + duration[previous[r]]*mandatory[previous[r]] + w[previous[r]] + c[previous[r]][r]; // waiting version
		b[r] = max((float) e[r], h[r]);
		
	}

}

// in waiting version, no lazy schedule needed
inline void SolutionVRPTW::lazySchedule() {
	for (int r=1; r<NVeh+1; r++) {
	// Update arrival and service times upon route k --> lazy phase --> only in dynamic execution !!!
		int i = previous[r];	// backward computation
		b[r] = (float) l[r];
		while (i != r) {
			if (!served[i])
				b[i] = b[i] + min(max((float)0.0, l[i]-b[i]), max((float)0.0, b[next[i]]-h[next[i]]));
			h[next[i]] = b[i] + duration[i]*mandatory[i] + c[i][next[i]];
			i = previous[i];
		} 
		float b_depart_depot = 0.0 + min(max((float)0.0, l[i]-b[i]), max((float)0.0, b[next[i]]-h[next[i]]));
		h[next[r]] = b_depart_depot + duration[r] + c[r][next[r]];
	}
}

void SolutionVRPTW::increaseWaitingTime(int vertex, float increment) {
	float b_, b_next, h_next;
	int r = vehicle[vertex];

	int i = previous[r];	// backward computation to determine the maximum waiting time possible at vertex
	b_next = (float) l[r];
	h_next = h[r];
	while (i != r) {
		ASSERT(!left[vertex], "argh");
		b_ = b[i] + min(max((float)0.0, l[i]-b[i]), max((float)0.0, b_next-h_next));
		if(i == vertex)
			break;
		b_next = b_;
		h_next = h[i];
		i = previous[i];
	} 

	w[vertex] = min(b_ - b[vertex], w[vertex]+increment);

	updateRouteInfos(r);
}

void SolutionVRPTW::decreaseWaitingTime(int vertex, float decrement) {
	w[vertex] = max((float)0.0, w[vertex]-decrement);

	updateRouteInfos(vehicle[vertex]);
}


inline float SolutionVRPTW::getViolations(int constraint, int vertex) {
	//if(constraint == ALL_CONSTRAINTS && vertex == 0)	// prefer invariant when possible
	//	return violations;
	return getSegmentViolations(constraint, vertex, vertex);
}	
inline float SolutionVRPTW::getSegmentViolations(int constraint, int fromVertex, int toVertex) {
	float viol = SolutionVRP::getSegmentViolations(constraint, fromVertex, toVertex);
	if(constraint == TW_CONSTRAINT || constraint == ALL_CONSTRAINTS) {
		if(fromVertex != 0) {
			for(int v=fromVertex; v!=next[toVertex]; v=next[v]) {
				if(active[v] && b[v] > l[v] + 0.0001) 
					//viol++;
					viol += b[v] - l[v];
			}
			return viol;
		}
			
		for (int i=1; i<NVeh+N+1; i++) {	// should never be used, prefer invariant !
			//ASSERT(false, "should never be used, prefer invariant");
			//if (active[i] && b[i] > l[i]) viol +=1000;
			if (active[i] && b[i] > l[i] + 0.0001) viol += b[i] - l[i];
		}
	}
	return (viol);
}


string& SolutionVRPTW::toString() {
	int vehicleNumber = 0, custNumber = 0;
	ostringstream out; 
	out.setf(std::ios::fixed);
	out.precision(1);
	
	updateRouteInfos();
	//lazySchedule(); // waiting version : no lazy

	for(int r=1; r<NVeh+1; r++) {
		custNumber += routeN[r];
		if(routeN[r] > 0) vehicleNumber++;
	}
	
	//out << "Capacity violations: " << getViolations(1) << endl << "TW violations: " << getViolations(2) << endl << flush;
	if (getViolations() > 0) out << "\033[0;31mInfeasible ! Violations: " << getViolations() << "\033[0;0m" << endl << flush;
	else out << outputMisc::greenExpr(true) << "Feasible solution found" << outputMisc::resetColor() << endl;
	
	out << "Cost = " << getCost() << " \t#Vehicles: " << vehicleNumber << " \t#Customers: " << custNumber << endl;
	//out << "Cost = " << getCost()-10000*getNbVehicle() << " \t#Vehicles: " << vehicleNumber << " \t#Customers: " << custNumber << endl;
	for(int r=1; r<NVeh+1; r++) {
		out << "Route " << setw(2) << r << " (c:" << setw(7) << routeCost[r]
			<< " q:" << outputMisc::redExpr(routeLoad[r]>Q) << setw(4) << routeLoad[r] << outputMisc::resetColor() 
			<< "): " << outputMisc::greenExpr(fixed[next[r]]) << "D \t" << outputMisc::resetColor();
		for(int i=next[r], n=1; i!=r; i=next[i], n++) {
			out << outputMisc::greenExpr(fixed[i]) << setw(3) << number[i] << "-" << outputMisc::blueExpr(!mandatory[i]) << ((ts[i]>=0)?to_string(ts[i]):"W") 
				<< outputMisc::resetColor() << "[" << outputMisc::greenExpr(served[i]) << outputMisc::redExpr(b[i]>l[i]+0.0001) 
				<< setw(6) << setfill('0') << b[i] << outputMisc::resetColor() << "+" << outputMisc::blueExpr(w[i] > 0) << setw(2) << setprecision(0) << w[i] << outputMisc::resetColor()
				<< setprecision(1) << "]" << " " << setfill(' ');
			if ((n%7) == 0) out << endl << "\t\t\t\t";
		}
		out << outputMisc::greenExpr(fixed[r]) << "    D" << outputMisc::resetColor() << "[" << outputMisc::greenExpr(served[r]) 
			<< outputMisc::redExpr(b[r]>l[r]+0.0001) << setw(6) << setfill('0') << b[r] << outputMisc::resetColor() << "]" << endl << setfill(' ');
	}

	updateRouteInfos();
	static string str = ""; str = out.str();
	return (str);
}


void SolutionVRPTW::printVertices() {
	cout << endl << "Requests summary\n---------------" << endl;
	for(int i=NVeh+1; i<NVeh+NVertices+1; i++) {
		cout << "Vertex (green:active blue:optional [reqTime <= l]) " << setw(3) << number[i] << "\t";
		for(int ts=0; ts<nTimeSlots; ts++) {
			//out << "\t" << outputMisc::boolColorExpr(requests[i+ts*NVertices]) << "TS" << ts <<" [" << minRevealTime[i+ts*NVertices]<<"<="<<requestTime[i+ts*NVertices]<<"<="<<maxRevealTime[i+ts*NVertices<<"]"] << outputMisc::resetColor();
			int v = i+ts*NVertices;
			int r = min(999, reqTime[v]);
			cout << "\t" << outputMisc::boolColorExpr(active[v]) << outputMisc::blueExpr(!mandatory[v] && active[v]) << "TS" << ts 
			 	<< "[" << setw(3)<< e[v]<<"<="  << setw(3) << r << "<=" << setw(3) << l[v] << "]" << outputMisc::resetColor() << setw(3) 
			 	<< (float)reqProba[v]/100 << "("<<v<<")"<<"<"<<minRevealTime[v]<<"," << setw(3) << floor(maxRevealTime[v])<<">";
		}
		if(NWaiting > 0) {
		int v = i+nTimeSlots*NVertices;
		cout << "\t" << outputMisc::boolColorExpr(active[v]) << outputMisc::blueExpr(!mandatory[v]) << "TS" << ts[v] 
		 	<< "[" << setw(3) << e[v] << "," << setw(3) << l[v] << "]" << outputMisc::resetColor() << "("<<v<<")" << " " << number[v];
		}
		cout << endl;
	}
}


/* test if hard constraints are respected */
void SolutionVRPTW::checkSolutionConsistency() {
	SolutionVRP::checkSolutionConsistency();

	// arrival times must precede service times
	for (int i=NVeh+1; i<NVeh+N+1+NWaiting; i++) {
		ASSERT(!(vehicle[i]!=0) || h[i] <= b[i] + 0.0001, "i="<<number[i] << " h="<<h[i]<<" b="<<b[i]); // planned --> good TW
		ASSERT(!(vehicle[i]!=0) || b[i] <= b[next[i]], "argh ! i = " << number[i] << "-" << ts[i]);
		ASSERT( false==(!fixed[i] && vehicle[i]>0) || !fixed[next[i]], "!fixed[i] must imply !fixed[next[i]]"); // i !fixed & planned --> next[i] !fixed
	}

	for(int r=1; r<NVeh+1; r++) 
		for(int i=next[r]; i!=r; i=next[i]) {
			//ASSERT(b[i] + duration[i]*mandatory[i] + c[i][next[i]] <= b[next[i]] + 0.0001, "wrong service time");
			ASSERT(b[i] + duration[i]*mandatory[i] + w[i] + c[i][next[i]] <= b[next[i]] + 0.0001, "wrong service time"); // waiting version
		}
}


/* dynamic context: 
 * any customer i already serviced of about to be serviced at time t becomes fixed
 */
void SolutionVRPTW::fixSolution(float t) {	
	bool update = false;

	current_time = t;

	//switch to lazy scheduling, because we want to delay as more as possible the vertex fixing
	//lazySchedule();	// waiting version : not needed with waiting times ! the algorithm chooses it self whether to be lazy or not on each vertex
	// depots vertices: because of time windows, depot can also be fixed (end of workday)
	for (int k=1; k<NVeh+1; k++) {
		if (fixed[k]==false && t + c[previous[k]][k] >= l[k]) {
			update = true;
			fixed[k] = true;
		}
		
		for(int i=next[k]; i!=k; i=next[i]) {
			ASSERT(active[i], "argh");
			if(fixed[i]==false && t + 0.0001 >= b[i] - duration[previous[i]]*mandatory[previous[i]] - c[previous[i]][i]) {	// i becomes fixed when vehicle must leaves previous[i]
				ASSERT(previous[i]<=NVeh || fixed[previous[i]], "i=" << number[i] <<" needs to be fixed, but previous[i]="<< number[previous[i]] <<"'s not !");
				
				update = true;
				
				fixed[i] = true;	
			}
			if(t >= b[i]) {
				served[i] = true;
				update = true;
			}

			if(left[i]==false && t >= b[i] + duration[i]*mandatory[i] + w[i]) {
				update = true;
				left[i] = true;
			}
			//if(fixed[i]==false)
			//	break;
		}
		
	}
	if(update)
		updateRouteInfos();	// because of firstFree[]
							// and also because we need to restore eager schedule
}

float SolutionVRPTW::computeRemainingWaitingTime(int v) {	
	return max((float) 0, min(w[v], b[v]+duration[v]*mandatory[v]+w[v] - current_time));
}




