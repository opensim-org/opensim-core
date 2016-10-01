#include <iostream>

#include <OpenSim/OpenSim.h>
#include "FunctionComponentConnector.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//#define PAUSE 1

double testFunction(const State& s)
{
    return s.getTime();
}

void test()
{
    Model model;

    FunctionComponentConnector<double>* fun =
        new FunctionComponentConnector<double>(testFunction);
    model.addModelComponent(fun);

    ConsoleReporter* rep = new ConsoleReporter();
    rep->setName("reporter");
    rep->set_report_time_interval(0.1);
    rep->updInput("inputs").connect(fun->getOutput("output"));
    model.addComponent(rep);

    State& s = model.initSystem();

    RungeKuttaMersonIntegrator integrator(model.getSystem());
    Manager manager(model, integrator);
    integrator.setMaximumStepSize(0.1);
    manager.setInitialTime(0);
    manager.setFinalTime(1);
    manager.integrate(s);
}

int main()
{
    try
    {
        test();
    }
    catch (const std::exception& ex)
    {
        std::cout << "Exception: " << ex.what() << std::endl;
#if PAUSE
        system("pause");
#endif
        return 1;
    }
    catch (...)
    {
        std::cout << "Unrecognized exception " << std::endl;
#if PAUSE
        system("pause");
#endif
        return 1;
    }

#if PAUSE
    system("pause");
#endif

    return 0;
}