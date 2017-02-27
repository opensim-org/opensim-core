#include <iostream>

#include <OpenSim/OpenSim.h>
#include "FunctionComponentSocket.h"

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

    FunctionComponentSocket<double>* fun =
        new FunctionComponentSocket<double>(testFunction);
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
    s.setTime(0.0);
    manager.integrate(s, 1.0);
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
