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
#include <algorithm>    // std::swap, std::max

using namespace std;

#ifndef TOOLS_H
#define	TOOLS_H



/* an assert statement, but with message */
#define ASSERT(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message << std::endl; \
            std::exit(EXIT_FAILURE); \
        } \
    } while (false)





/* handles a list of element of type E; the element are ordered by a value of type T
	!!! INSERTION in O(n) ! Only useful for incremental iteration over the ordered elements...
*/

template<typename E, typename T>
class OrderedLinkedList{
private:
	struct Node {
		E element;
		T value;
		Node *next;
		Node *prev;
	} *firstNode, *lastNode, *current;
	int nbElements;
public:
	OrderedLinkedList(){
		firstNode = new Node;
		firstNode->next = NULL;
		firstNode->prev = NULL;
		lastNode = firstNode;
		current = firstNode;
		nbElements = 0;
	}
	~OrderedLinkedList(){
		Node *next, *n;
		for(n=firstNode; n!=NULL; n=next) {
			next = n->next;
			delete n;
		}
	}
	void insert(E e, T orderValue) {
		if(nbElements == 0) {		// if insert in empty list
			firstNode->element = e;
			firstNode->value = orderValue;
		}
		else {
			Node *n;

			if(orderValue <= firstNode->value) {	// if insert at first position
				n = new Node;
				n->next = firstNode;
				n->prev = NULL;
				firstNode = n;
			} else {								// or insert at a middle place
				for(n=firstNode; n->next!=NULL; n=n->next)
					if(n->value < orderValue && orderValue <= n->next->value)
						break;
				Node *previousNode = n;
				n = new Node;
				n->next = previousNode->next;
				if(n->next != NULL)
					n->next->prev = n;
				previousNode->next = n;
				n->prev = previousNode;
				if(n->next == NULL)			// if insert at last position
					lastNode = n;
			}

			n->element = e;
			n->value = orderValue;
		}
		nbElements++;
	}
	void removeCurrent() {
		if(current != NULL) {
			if(current->prev != NULL)
				current->prev->next = current->next;
			if(current->next != NULL)
				current->next->prev = current->prev;

			if(current == firstNode)
				firstNode = current->next;
			if(current == lastNode)
				lastNode = current->prev;

			Node* n = current;
			current = current->next;	
			delete n;
		}
		nbElements--;
	}
	E getCurrent() {
		if(current == NULL)
			return false;
		return current->element;
	}
	bool isCurrentNull() { 
		if(current) {
			return false; 
		} else
			return true;
	}
	bool moveNext() { 
		if(current->next) {
			current = current->next;
			return true; 
		} else
			return false;
	}
	bool movePrevious() { 
		if(current->prev) {
			current = current->prev;
			return true; 
		} else
			return false;
	}
	void goFirst() { current = firstNode; }
	void goLast() { current = lastNode; }
	int getNbElements() { return nbElements; }
};


template<typename E, typename T>
class OrderedStaticLinkedList{
private:
	struct Node {
		E element;
		T value;
		int next = -1;
		int prev = -1;
		int nextFree;
	};
	int firstFree;
	int firstNode, lastNode, current;
	int nbElements, maxNbElements;
	Node* tabNodes;
public:
	OrderedStaticLinkedList(int nbNodes){
		maxNbElements = nbNodes;
		tabNodes = new Node [maxNbElements];
		for (int i=0; i<maxNbElements; i++) {
			tabNodes[i].nextFree = i+1;
			tabNodes[i].next = -1;
			tabNodes[i].prev = -1;
		}
		tabNodes[maxNbElements-1].nextFree = -1;
		firstFree = 0;
		firstNode = -1;
		lastNode = -1;
		current = -1;
		nbElements = 0;
	}
	~OrderedStaticLinkedList(){
		delete [] tabNodes;
	}
	void clean() {
		for (int i=0; i<maxNbElements; i++) {
			tabNodes[i].nextFree = i+1;
			tabNodes[i].next = -1;
			tabNodes[i].prev = -1;
		}
		tabNodes[maxNbElements-1].nextFree = -1;
		firstFree = 0;
		firstNode = -1;
		lastNode = -1;
		current = -1;
		nbElements = 0;
	}
	void insert(E e, T orderValue) {
		int n = firstFree;
		if(nbElements == 0) {		// if insert in empty list
			n = firstFree;
			tabNodes[n].element = e;
			tabNodes[n].value = orderValue;
			firstNode = n;
		}
		else {
			if(orderValue <= tabNodes[firstNode].value) {	// if insert at first position
				tabNodes[n].next = firstNode;
				tabNodes[n].prev = -1;
				firstNode = n;
			} else {
				int p;								// or insert at a middle place
				for(p=firstNode; tabNodes[p].next!=-1; p=tabNodes[p].next)
					if(tabNodes[p].value < orderValue && orderValue <= tabNodes[tabNodes[p].next].value)
						break;
				int previousNode = p;
				tabNodes[n].next = tabNodes[previousNode].next;
				if(tabNodes[n].next != -1)
					tabNodes[tabNodes[n].next].prev = n;
				tabNodes[previousNode].next = n;
				tabNodes[n].prev = previousNode;
				if(tabNodes[n].next == -1)			// if insert at last position
					lastNode = n;
			}

			tabNodes[n].element = e;
			tabNodes[n].value = orderValue;
		}

		firstFree = tabNodes[n].nextFree;
		tabNodes[n].nextFree = -1;
		
		nbElements++;

		ASSERT(nbElements < maxNbElements, "argh"); 
		ASSERT(firstFree >= 0, "argh");
	}
	void removeCurrent() {
		ASSERT(current >= 0, "argh");
		
		if(tabNodes[current].prev != -1)
			tabNodes[tabNodes[current].prev].next = tabNodes[current].next;
		if(tabNodes[current].next != -1)
			tabNodes[tabNodes[current].next].prev = tabNodes[current].prev;

		if(current == firstNode)
			firstNode = tabNodes[current].next;
		if(current == lastNode)
			lastNode = tabNodes[current].prev;

		tabNodes[current].nextFree = firstFree;
		firstFree = current;
		current = tabNodes[current].next;	
		nbElements--;
	}
	E getCurrent() {
		if(current == -1)
			return false;
		return tabNodes[current].element;
	}
	bool isCurrentNull() { 
		return current == -1;
	}
	bool moveNext() { 
		if(tabNodes[current].next != -1) {
			current = tabNodes[current].next;
			return true; 
		} else
			return false;
	}
	bool movePrevious() { 
		if(tabNodes[current].prev != -1) {
			current = tabNodes[current].prev;
			return true; 
		} else
			return false;
	}
	void goFirst() { current = firstNode; }
	void goLast() { current = lastNode; }
	int getNbElements() { return nbElements; }
};



/* find a random element in an array of predefined size such that
	the element returned fulfils the constraints in function "condition"
	returns that element and put the array index of the element in *index
	if no element found (respecting the constraints), then index is set to -1
	at return, &index contains the array index of the element selected (or something irrelevant if nothing selected) */
	//	!!! rand must been seeded and the array must be shuffled !!!!
template <typename T>
//inline int selectElement(T *array, int size, bool (*condition)(T element), int *index) {
inline int selectElement(T *array, int size, function <bool(T)> condition, int *index) {
	int start = rand() % size;
	int i = start;
	//cout << "start: "<< start << endl;
	do {
		if(condition(array[i]) == true) {
			*index = i;
			return array[i];
		}
		i = (i+1) % size;
	} while (i != start);
	*index = -1;
	return 0;
	/*
	int i;
	do {
		i = rand() % size;
	} while (!condition(array[i]));
	*index = i;
	return array[i];*/
}


template <typename T>
inline void shuffleArray(T* array, int size, int iter) {
	T tmp;
	int x;
	for(int j=0; j<iter; j++)
		for(int i=0; i<size; i++) {
			x = rand() % size;
			tmp = array[i];
			array[i] = array[x];
			array[x] = tmp;
		}
}


/* miscellaous output functions */
struct outputMisc {
	inline static const string boolColorExpr(bool expr) {	// set color to green if expr==true, red otherwise
		if (expr) return "\033[0;32m";
		else return "\033[0;31m"; 
	}
	inline static const string redExpr(bool expr) {
		if (expr) return "\033[0;31m";
		else return ""; 
	}
	inline static const string greenExpr(bool expr) {
		if (expr) return "\033[0;32m";
		else return ""; 
	}
	inline static const string blueBackExpr(bool expr) {
		if (expr) return "\033[0;44m";
		else return ""; 
	}
	inline static const string blueExpr(bool expr) {
		if (expr) return "\033[0;34m";
		else return ""; 
	}
	inline static const string resetColor() {
		return "\033[0;0m";
	}
};




#endif
