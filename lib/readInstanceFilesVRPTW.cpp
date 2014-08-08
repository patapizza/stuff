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
extern int 	*reqTime;	// time at which the request is revealed (dynamic context)
extern float *duration;	// service duration at each vertex
extern int   *e;			// start of time window at each vertex
extern int   *l;			// end of time window at each vertex

extern int Nactive;
extern int NVehActive;

// Dynamic context
extern int NVertices;			// number of customer vertices (different from N, because of time slots, there can be several requests per vertex)
extern int nTimeSlots;			// number of time slot (including the pre time slot containing a priori requests)
extern int NWaiting;			// number of waiting fake requests, i.e. vertices where the vehicle can go in order to wait for possible sorrounding requests
extern int *reqProba;			// probabitlity for that request to appear online (for a given time slot)
extern float *maxRevealTime;	// according to the time slot, latest time a vehicle can depart from the depot, serve the vertex 
								// and return to depot (lambda_c in Bent&VanHentenryck's paper)
								// defines an upper bound on the time at which a request could be revealed
extern float *minRevealTime;	// lower bound on which a request can be revealed
extern int 	*ts;			// time slot of the request
extern bool	*rejected;		// tells whether a request has been rejected
extern int lastReqTime;	// unit time of the last online request (so we can end the computation asap)


/* Handles instance files of Cordeau-Laporte in data/vrptw/old 	-	static instances */
void readInstanceFileCordeauLaporteVRPTWold(const char *filename) {
	int i;
	std::string line;
	std::ifstream instance_file;
	instance_file.open(filename);
	if (instance_file.fail()) {
		std::cerr << endl << " Error: Unable to open " << filename << endl;
		exit (8);
	}

	instance_file >> i; 	// skip first integer
	instance_file >> NVeh;	// retrieve number of vehicles
	instance_file >> N;		// retrieve instance size
	c = new double*[NVeh+N+1];
	for(int i=0; i<NVeh+N+1; i++) c[i] = new double[NVeh+N+1];

	coordX 		= new float[NVeh+N+1]; 
	coordY 		= new float[NVeh+N+1];
	duration 	= new float[NVeh+N+1];
	demand 		= new float[NVeh+N+1];
	number 		= new int[NVeh+N+1];
	reqTime 	= new int[NVeh+N+1];
	for (int i=1; i<NVeh+N+1; i++) reqTime[i] = -1;
	e 			= new int[NVeh+N+1]; 
	l 			= new int[NVeh+N+1];

	std::getline(instance_file, line); // skip end of line

	instance_file >> i;			// skip int
	instance_file >> Q;			// retrieve max capacity

	// DEPOTS
	instance_file >> i;			// skip int
	instance_file >> coordX[1]; 	// retrieve x coordinate
	instance_file >> coordY[1]; 	// retrieve y coordinate
	instance_file >> duration[1];	// retrieve service duration
	instance_file >> demand[1];		// retrieve demand
	instance_file >> i; 			// skip int
	instance_file >> i; 			// skip int
	//instance_file >> i; 			// skip int 	---->    /!\ One int less to skip in the depot line
	instance_file >> e[1];			// retrieve start TW
	instance_file >> l[1];			// retrieve end TW
	for (int r=2; r<NVeh+1; r++) {
		coordX[r] = coordX[1];
		coordY[r] = coordY[1];
		duration[r] = duration[1];	
		demand[r] = demand[1];		
		e[r] = e[1];				
		l[r] = l[1];				
	} 

	// REQUESTS
	for (int j=NVeh+1; j<NVeh+N+1; j++) {
		instance_file >> number[j]; 	// retrieve vertex number
		instance_file >> coordX[j]; 	// retrieve x coordinate
		instance_file >> coordY[j]; 	// retrieve y coordinate
		instance_file >> duration[j];	// retrieve service duration
		instance_file >> demand[j];		// retrieve demand
		instance_file >> i; 			// skip int
		instance_file >> i; 			// skip int
		instance_file >> i; 			// skip int
		instance_file >> e[j];			// retrieve start TW
		instance_file >> l[j];			// retrieve end TW
	}

	instance_file.close();

	for (int i=1; i<NVeh+N+1; i++)
		for (int j=1; j<NVeh+N+1; j++)
			c[i][j] = std::sqrt(std::pow((coordX[i] - coordX[j]),2) + std::pow((coordY[i] - coordY[j]),2)); // compute Euclidean distances

	//for (int i=NVeh+1; i<NVeh+N+1; i++)
	//	cout << "demand[" << number[i] << "]=" << demand[i] << endl;

	for (int i=1; i<NVeh+N+1; i++)
		reqTime[i] = -1;
}


/* dynamic instances of Bent and VanHentenryck */
void readInstanceFileBentVanHentenryckDynVRPTW(const char *filename){
	//NVeh = 14;				// for example (not present in the instance file)

	int i, j; 
	string s, line;
	std::ifstream instance_file;
	instance_file.open(filename);
	if (instance_file.fail()) {
		std::cerr << endl << " Error: Unable to open " << filename << endl;
		exit (8);
	}

	//instance_file >> i; 	// skip first integer
	instance_file >> s;		// skip word "Customers:"
	instance_file >> NVertices;		// retrieve instance size
	instance_file >> s;		// skip word "Capacity:"
	instance_file >> Q;		// retrieve max capacity
	instance_file >> s;		// skip words "Time Bins"
	instance_file >> s;		// 
	instance_file >> nTimeSlots;		// retrieve nTimeSlots
	nTimeSlots --; // because nothing on the last time slot for these instances
	instance_file >> s;		// skip word "Vehicle"
	instance_file >> NVeh;		// retrieve NVeh

	//NVeh -= 2;

	N = NVertices * nTimeSlots;			//  	[00..99][100..199] [200..299] ... [(nTimeSlots-1)*100..nTimeSlots*100-1]
										//			TS0	     TS1 		TS2 			 TSn-1
										//   	  a priori   online	   online	...		online

	NWaiting = NVertices * 100;

	reqProba = new int[NVeh+N+1];

	c = new double*[NVeh+N+1+NWaiting];			// waiting version
	for(int i=0; i<NVeh+N+1+NWaiting; i++) 		// waiting version
		c[i] = new double[NVeh+N+1+NWaiting];	// waiting version

	number 		= new int[NVeh+N+1+NWaiting];		// waiting version 
	coordX 		= new float[NVeh+N+1+NWaiting]; 	// waiting version
	coordY 		= new float[NVeh+N+1+NWaiting];		// waiting version
	duration 	= new float[NVeh+N+1+NWaiting];
	demand 		= new float[NVeh+N+1+NWaiting];
	e 			= new int[NVeh+N+1+NWaiting]; 
	for(int i=0; i<NVeh+N+1+NWaiting; i++) e[i] = -1;	// for TimeBin management
	l 			= new int[NVeh+N+1+NWaiting];
	reqTime 	= new int[NVeh+N+1];


	std::getline(instance_file, line); // skip end of line
	std::getline(instance_file, line); // skip line
	std::getline(instance_file, line); // skip line

	// DEPOT DATA
	instance_file >> s;		// skip word "D"
	instance_file >> coordX[1]; 	// retrieve x coordinate
	instance_file >> coordY[1]; 	// retrieve y coordinate
	instance_file >> demand[1];		// retrieve demand
	instance_file >> e[1];			// retrieve start TW  = 0
	instance_file >> l[1]; 			// end of day is always 480 in these Bent-VanHentenryck instances
	instance_file >> duration[1];	// retrieve service duration
	number[1] = -1;					// depot as number -1 (because customer 0 exists in Bent-VanHentenryck instances)
	std::getline(instance_file, line); // skip end of line
	for (int r=2; r<NVeh+1; r++) {
		coordX[r] = coordX[1];
		coordY[r] = coordY[1];	
		demand[r] = demand[1];	
		e[r] = e[1];				
		l[r] = l[1];			
		duration[r] = duration[1];	
		number[r] = number[1];	
	} 
	for (int i=1; i<NVeh+1; i++)
		reqTime[i] = -1;
	// VERTICES DATA
	for (int j=NVeh+1; j<NVeh+NVertices+1; j++) {
		instance_file >> number[j]; 	// retrieve vertex number
		instance_file >> coordX[j]; 	// retrieve x coordinate
		instance_file >> coordY[j]; 	// retrieve y coordinate
		instance_file >> demand[j];		// retrieve demand
		instance_file >> e[j];			// retrieve start TW
		instance_file >> l[j];			// retrieve end TW
		instance_file >> duration[j];	// retrieve service duration
		instance_file >> reqProba[j];				// probability for the request to be known a priori
		instance_file >> reqProba[j + NVertices*1];	// probability that the request appends at time slot 1
		instance_file >> reqProba[j + NVertices*2];	// probability that the request appends at time slot 2
		//instance_file >> reqProba[j + NVertices*3];	// probability that the request appends at time slot 3
		std::getline(instance_file, line); // skip end of line
		reqTime[j] = numeric_limits<int>::max();	// set reqTime at a too late value for all vertices
	}

	// duplicate data for other time bins (demand, duration, etc. depend on the associated online request)
	for (int i=NVeh+NVertices+1; i<NVeh+N+1; i++) {
		coordX[i] = coordX[i - NVertices];
		coordY[i] = coordY[i - NVertices];		
		number[i] = number[i - NVertices];			
		demand[i] = demand[i - NVertices];			
		e[i] = e[i - NVertices];		
		l[i] = l[i - NVertices];	
		duration[i] = duration[i - NVertices];		
		reqTime[i] = numeric_limits<int>::max();	// initially set reqTime for all possible request at infinite value
	} 	

	for (int i=NVeh+N+1; i<NVeh+N+1+NWaiting; i++) {	// waiting version
		coordX[i] = coordX[i - NVertices];
		coordY[i] = coordY[i - NVertices];		
		number[i] = number[i - NVertices];	
	}


	for (int i=1; i<NVeh+N+1+NWaiting; i++)			// waiting version
		for (int j=1; j<NVeh+N+1+NWaiting; j++)		// waiting version
			c[i][j] = std::sqrt(std::pow((coordX[i] - coordX[j]),2) + std::pow((coordY[i] - coordY[j]),2)); // compute Euclidean distances

	/******** Retrieving dynamic request arrival times ********/

	// retrieve known requests
	instance_file >> s;						// skip words "Unknown Requests"
	instance_file >> s;						// 
	int apriori_req_number;
	instance_file >> apriori_req_number; 	// retrieve number of known requests
	//cout << "Known requests: " << apriori_req_number << "/" << N << endl << flush;
	std::getline(instance_file, line); 		// skip line
	std::getline(instance_file, line); 		// skip line

	
	for (int i=0; i<apriori_req_number; i++) {
		int vnumber, x;
		instance_file >> vnumber;
		int v = vnumber+NVeh+1;			// it belongs to the "pre time bin" TB0
		instance_file >> reqTime[v];	
		instance_file >> x; ASSERT(x==e[v], "error");			// retrieve start TW
		instance_file >> x; ASSERT(x==l[v], "error");			// retrieve end TW
		instance_file >> x; ASSERT(x==demand[v], "error");		// retrieve demand
		instance_file >> x; ASSERT(x==duration[v], "error");	// retrieve service duration
		std::getline(instance_file, line); // skip end of line
	}

	// now retrieve dynamic part
	instance_file >> s;						// skip words "Unknown Requests"
	instance_file >> s;						// 
	int dynamic_req_number;
	instance_file >> dynamic_req_number; 	// retrieve number of dynamic requests
	//cout << "Dynamic requests: " << dynamic_req_number << "/" << N << endl << flush;
	std::getline(instance_file, line); 		// skip line
	std::getline(instance_file, line); 		// skip line
	for (int i=0; i<dynamic_req_number; i++) {
		int vnumber;
		instance_file >> vnumber;
		
		int v, reqtime;
		instance_file >> reqtime;
		if (reqtime < 80)
			v = vnumber+NVeh+1 + NVertices*1;
		else if (reqtime < 160)
			v = vnumber+NVeh+1 + NVertices*2;
		else if (reqtime < 240)
			v = vnumber+NVeh+1 + NVertices*3;
		else 
			ASSERT(false, "Error in reqTime affectation");

		/*
		int v, ts;
		for (ts=1; ts < nTimeSlots; ts++) {
			v = vnumber+NVeh+1 + ts*NVertices;
			if (e[v] == -1)
				break;
		}
		ASSERT(ts <= nTimeSlots, "nTimeSlots exceeded");
		ASSERT(v <= N, "N exceeded");
		*/

		int x;
		//instance_file >> reqTime[v];
		reqTime[v] = reqtime;
		instance_file >> x;		// !! we do not record the actual e[v] on that online request here; it is supposed to be revealed online (since it is the reveal time)
		instance_file >> x;		//    idem with l[v] (even if it stays the same)	
		instance_file >> demand[v];		// retrieve demand
		instance_file >> duration[v];	// retrieve service duration

		std::getline(instance_file, line); // skip end of line

		if(i == dynamic_req_number-1)	// if last dyn req
			lastReqTime = reqtime;
	}

	instance_file.close();
	cout << endl;


	ts = new int[NVeh+N+1+NWaiting];
	maxRevealTime = new float[NVeh+N+1];// according to the time slot, latest time a vehicle can depart from the depot, serve the vertex 
										// and return to depot (lambda_c in Bent&VanHentenryck's paper)
										// defines an upper bound on the time at which a request could be revealed
	minRevealTime = new float[NVeh+N+1];// lower bound on which a request can be revealed

	for(int i=NVeh+1; i<NVeh+N+1; i++) {
		ts[i] = floor((i-NVeh-1) / NVertices);	// determines the time slot
		int ubTimeSlot = ts[i] * (l[1]-e[1]) / nTimeSlots - 1;	// another upper bound, corresponding to the time slot window : k*H/3 - 1 in the paper
		double maxTime = min(l[1] - c[i][1] - duration[i], (double) l[i]) - c[1][i];
		minRevealTime[i] = (ts[i]-1) * (l[1]-e[1]) / nTimeSlots;
		maxRevealTime[i] = min(maxTime, (double) ubTimeSlot);
		e[i] = max((float)e[i], minRevealTime[i]);
	}

	

	rejected = new bool[NVeh+N+1];	// tells whether a customer request has been rejected yet (dynamic context only)
		for (int i=1; i<NVeh+N+1; i++) rejected[i] = false;
	/*
	for (int i=1; i<NVeh+N+1; i++)
		cout << "reqTime[" << setw(2) << number[i] << "] = " << setw(10) << reqTime[i] 
			<< "\t TS" << floor((i-NVeh-1) / NVertices) << "\tDuration: " << duration[i] << " TW" << e[i] <<"-"<<l[i] << endl;
	*/
	/*
	for (int i=1; i<NVeh+N+1; i++)
		cout << "reqProba[" << number[i] << "] = " << reqProba[i] << endl;
	*/


	for(int i=NVeh+N+1; i<NVeh+N+1+NWaiting; i++){
		ts[i] = -1;
		e[i] = e[1];
		l[i] = l[1];
		duration[i] = 0;
		demand[i] = 0;
	}

}

void readInstanceFileBentVanHentenryckDynVRPTW_Static(const char *filename){
	readInstanceFileBentVanHentenryckDynVRPTW(filename);
	for (int i=1; i<NVeh+NVertices+1; i++)
		reqTime[i] = -1;
	for (int i=NVeh+NVertices+1; i<NVeh+N+1; i++)
		reqTime[i] = numeric_limits<int>::max();
}

