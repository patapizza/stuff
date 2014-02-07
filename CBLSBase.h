enum var_type {
    BOOL,
    INT,
    FLOAT
};

struct var_t {
    var_type type;
    bool boolValue;
    int intValue;
    float floatValue;
};

/*
 * Alt. 1: templates
 * template <class var_t>
 * class IVar
 * {
 *     var_t val;
 *     float activity;
 *     public:
 *         virtual var_t getVal();
 *     etc.
 * };
 * This would be the template class (default class).
 * That being defined, specialization follows, upon necessity:
 * template <>
 * class IVar <float>
 * {
 *     float val;
 *     float activity;
 *     public:
 *         virtual float getVal();
 *     etc.
 * };
 * Usage:
 * IVar<int> getVal(); // template class
 * IVar<float> getVal(); // specialized definition
 * Left to see what's going on with inheritance.
 * Need to be implemented within the header file.
 */

/*
 * Alt. 2: anonymous union within a struct
 * struct var_t {
 *     union {
 *         bool boolValue;
 *         int intValue;
 *         float floatValue;
 *     };
 * };
 * Usage:
 * var_t blah;
 * blah.boolValue = true;
 */

class IVar
{
    var_t val;
    float activity;
    public:
        virtual var_t getVal() = 0;
        virtual void setVal(var_t newVal) = 0;
        virtual float getActivity() = 0;
        virtual void setActivity(float newActivity) = 0;
        virtual ~IVar() {}
};

class ISolution
{
    var_t *vars;
    public:
        virtual ~ISolution() {}
        virtual void print() = 0;
        virtual float cost() = 0;
};

class IConstraint
{
    ISolution *currentSolution;
    float activity;
    public:
        virtual ISolution *getCurrentSolution() = 0;
        virtual void setCurrentSolution(ISolution *newCurrentSolution) = 0;
        virtual ~IConstraint() {}
        virtual bool isTrue() = 0;
};

class INeighborhoodManager
{
    ISolution *currentSolution;
    public:
        virtual ISolution *getCurrentSolution() = 0;
        virtual void setCurrentSolution(ISolution *newCurrentSolution) = 0;
        virtual ~INeighborhoodManager() {}
        virtual void shakeSolution() = 0;
};

class ILSProgram
{
    int iterNumber;
    ISolution *incumbentSolution;
    ISolution *currentSolution;
    ISolution *bestSolution;
    IConstraint *constraints;
    INeighborhoodManager *neighborhoodManager;
    public:
        virtual ISolution *getCurrentSolution() = 0;
        virtual void setCurrentSolution(ISolution *newCurrentSolution) = 0;
        virtual INeighborhoodManager *getNeighborhoodManager() = 0;
        virtual void setNeighborhoodManager(INeighborhoodManager *newNeighborhoodManager) = 0;
        virtual ~ILSProgram() {}
        virtual void run() = 0;
        virtual void addConstraint(IConstraint *newConstraint) = 0;
        virtual bool terminalCondition() = 0;
};

class LSProgram : public ILSProgram
{
    public:
        ISolution *getCurrentSolution();
        void setCurrentSolution(ISolution *newCurrentSolution);
        INeighborhoodManager *getNeighborhoodManager();
        void setNeighborhoodManager(INeighborhoodManager *newNeighborhoodManager);
        void run();
        void addConstraint(IConstraint *newConstraint);
        bool terminalCondition();
};
