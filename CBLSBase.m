@import "CBLSBaseTypes.h"



@implementation Var
@synthesize val;
@synthesize activity;
- (NSString *) description {
	NSString *desc;
	desc = [NSString stringWithFormat: @"%@.%i=%@", name, nameIndex, val];
	return desc;
}
@end // Var



@implementation Solution
- (float) cost {
	return 0.0;
}
- (void) print {
	NSLog(@"Solution:");
	for(id *obj in vars) NSLog(@"%@", obj);
}
@end // Solution



@implementation Constraint
@synthesize currentSolution;
@synthesize activity;
- (bool) isTrue {
	return true;
}
@end // Constraint

@implementation NeighborhoodManager
@synthesize currentSolution;
- (void) shakeSolution {
	// Here disturb the current solution
}
@end // NeighborhoodManager

@implementation LSProgram
@synthesize currentSolution;
@synthesize neighborhoodManager;
- (id) init {
	if (self = [super init]) {
		iterNumber = 0;
		constraints = [NSMutableArray arrayWithCapacity: 5];
	}
}
- (void) addConstraint: (Constraint *) constraint {
	[constraints addObject: constraint];
}
- (bool) terminationCondition {
	return (iterNumber == 100); // For instance..
}
- (void) run {
	NSLog(@"Running LSProgram ...");
	incumbentSolution = [currentSolution copy];
	bestSolution = [currentSolution copy];
	while (false == [self terminationCondition]) {
		
		[self shakeSolution]; // disturb current solution

		if ([self acceptanceCriterion]) {
			[incumbentSolution release];
			incumbentSolution = [currentSolution copy];
			if ([currentSolution cost] < [bestSolution cost]) {
				[bestSolution release];
				bestSolution = [currentSolution copy];
			}
		} else {
			[[currentSolution vars] setArray:[[incumbentSolution vars] copy]];
		}

		iterNumber++;
	}
	NSLog(@"Terminated.");
	[bestSolution print];
}
@end // LSProgram



