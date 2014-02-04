@import <Foundation/Foundation.h>



typedef enum { bool, int, float } var_t;

@interface Var : NSObject {
	var_t	val;
	float 	activity;
}
@property var_t val;
@property float activity;
@end // Var



@interface Solution : NSObject {
	NSMutableArray	*vars;
}
- (void) print;
- (float) cost;
@end // Solution


@interface Constraint : NSObject {
	Solution	*currentSolution;
	float 		activity;
}
@property (retain) Solution *currentSolution;
@property float activity;
- (bool) isTrue;
@end

@interface NeighborhoodManager : NSObject {
	Solution *currentSolution;
}
@property (retain) Solution *currentSolution;
- (void) shakeSolution;
@end

@interface LSProgram : NSObject {
	int iterNumber;
	Solution *incumbentSolution;
	Solution *currentSolution;
	Solution *bestSolution;
	NSMutableArray *constraints;
	NeighborhoodManager *neighborhoodManager;
}
@property (retain) Solution *currentSolution;
@property (retain) NeighborhoodManager *neighborhoodManager;
- (void) run;
- (void) addConstraint: (Constraint *) newConstraint;
- (bool) terminationCondition;
@end




