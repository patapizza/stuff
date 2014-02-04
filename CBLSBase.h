/* @TODO: declare var_t with template */
class IVar
{
    private:
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
    private:
        var_t *vars;
    public:
        virtual ~ISolution() {}
        virtual void print() = 0;
        virtual float cost() = 0;
};

class IConstraint
{
    private:
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
    private:
        ISolution *currentSolution;
    public:
        virtual ISolution *getCurrentSolution() = 0;
        virtual void setCurrentSolution(ISolution *newCurrentSolution) = 0;
        virtual ~INeighborhoodManager() {}
        virtual void shakeSolution() = 0;
};

class ILSProgram
{
    private:
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
