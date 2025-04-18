
// INCLUDE
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Analyses/StaticOptimization.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main()
{
    AnalyzeTool analyze("OpenSim/Examples/StaticOptimizationExample/arm26_Setup_StaticOptimization.xml");
    analyze.run();

    return 0;
}
