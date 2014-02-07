#include "CBLSBase.h"

class Var : public IVar
{
};

class Solution : public ISolution
{
};

class Constraint : public IConstraint
{
};

class NeighborhoodManager : public INeighborhoodManager
{
};

class LSProgram : public ILSProgram
{
    public:
        ~LSProgram() {
        };
};
