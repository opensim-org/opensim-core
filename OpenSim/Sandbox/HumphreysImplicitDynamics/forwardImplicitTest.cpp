/* -------------------------------------------------------------------------- *
 *                             OpenSim:  forwardImplicitTest.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* 
 *  Below is an example of an OpenSim application that provides its own 
 *  main() routine.  
 */

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

struct momentArmAndLength {
    SimTK::Vec2 length;
    SimTK::Vec2 momentArm;
};




struct bodyResidual {
    double forceResidual = SimTK::NaN;
    double velocityResidual = SimTK::NaN;
    double forceMass = SimTK::NaN;
    SimTK::Mat22 df_dy = {SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN};
    SimTK::Mat22 df_dydot = {SimTK::NaN,SimTK::NaN,SimTK::NaN,SimTK::NaN};
};




// Get the Residual of the mass body.
bodyResidual calcBodyImplicitResidual(Model& osimModel, State& s, double velocityGuess, double accelGuess, Vec2 muscleForces)
{

    // Tried to use new implicit interface:
    //   (going to just use equation implementation for now)
    //const double qDotGuess = 0.0; const double uDotGuess = 0.0;
    // Set derivative guess.
    //coord.setStateVariableDerivativeGuess(s, "value", qDotGuess);
    //coord.setStateVariableDerivativeGuess(s, "speed", uDotGuess);

    /*Returns:
    /home/humphreysb/vanDenBogertMuscleTestSource/vanDenBogertMuscleTest.cpp:220:43: error: no viable conversion from 'SimTK::State' to
    'const std::string' (aka 'const basic_string<char>')
    coord.setStateVariableDerivativeGuess(s, "value", qDotGuess);
     */

    //  Get the "Rigid Body" Dynamics

    Vec2 momentArms={-1, 1};    //Moment Arms are hard coded here.
                                //Ajay suggests trying to see cost having
                                //opensim calculate them.

    //Calculate the mass force residual (for now by equations)
    double m=20;
    double forceMass = m*accelGuess;  //F=ma
    double forceRes = momentArms[0]*muscleForces[0] + momentArms[1]*muscleForces[1] - forceMass;

    //Calculate the Velocity Residual
    const CoordinateSet& modelCoordinateSet = osimModel.getCoordinateSet();
    double velRes = modelCoordinateSet[0].getSpeedValue(s) - velocityGuess;

    //Analytical Jacobian
    SimTK::Mat22 df_dydot = {0,-m,-1,0};

    //Create Output
    bodyResidual Results;
    Results.forceResidual=forceRes;
    Results.velocityResidual=velRes;
    Results.df_dydot=df_dydot;

    return Results;

}


//Compute the residuals on the full model.  This also noes the forward euler
// to go from yPGuess (y-prime guess) to ydotGuess
SimTK::Vector computeImplcitResiduals(Model& osimModel, State& s, Vector yPGuess, Vector u, double h){


    //Need to calculate yDotGuess using forward euler appoximation: ydot=(yp-y)/h
    //    (Note that I did this in 2 steps in my MATLAB implmentation.  Here
    //      I am doing it in one function so I only need to dynamic cast the
    //      muscle once.)

    // So let's get y (from s):

    //First get the muscle and recast
    const Set<Muscle>& muscleSet=osimModel.getMuscles();
    const auto* muscle1 = dynamic_cast<const VandenBogert2011Muscle*>(&muscleSet[0]);
    const auto* muscle2 = dynamic_cast<const VandenBogert2011Muscle*>(&muscleSet[1]);

    Vector y(6);
    y[0]=muscle1->getFiberLength(s);
    y[1]=muscle1->getActivation(s);
    y[2]=muscle2->getFiberLength(s);
    y[3]=muscle2->getActivation(s);

    const CoordinateSet& modelCoordinateSet = osimModel.getCoordinateSet();
    y[4]=modelCoordinateSet[0].getValue(s);
    y[5]=modelCoordinateSet[0].getSpeedValue(s);

    // Perform Forward Approximation
    Vector yDotGuess(6);
    yDotGuess=(yPGuess-y)/h;


    //Now let's get the residuals
    // Muscle1's Residuals
    const VandenBogert2011Muscle::ImplicitResidual muscle1Residual =muscle1->calcImplicitResidual(s, yDotGuess[0], yDotGuess[1], u[0],0);
    //Muscle2's Residuals
    const VandenBogert2011Muscle::ImplicitResidual muscle2Residual =muscle2->calcImplicitResidual(s, yDotGuess[2], yDotGuess[3], u[1],0);

    // Mass Residual
    Vec2 muscleForces = {muscle1Residual.forceTendon, muscle2Residual.forceTendon};
    bodyResidual massResidual=calcBodyImplicitResidual(osimModel, s, yDotGuess[4], yDotGuess[5], muscleForces);

    // Residual Vector
        SimTK::Vector residuals(6);
         residuals[0]=muscle1Residual.forceResidual,
         residuals[1]=muscle1Residual.activResidual,
         residuals[2]=muscle2Residual.forceResidual,
         residuals[3]=muscle2Residual.activResidual,
         residuals[4]=massResidual.forceResidual,
         residuals[5]=massResidual.velocityResidual;

        return residuals;
}





// =============================================================================
// ImplicitSystemDerivativeSolver
//        (adapted from Chris's testImplicitDifferentialEquations.cpp)
// =============================================================================

class ImplicitSystemDerivativeSolver : public OptimizerSystem {
public:

    /* Constructor class. Parameters passed are accessed in the objectiveFunc() class. */
    ImplicitSystemDerivativeSolver(int numParameters, State& s, Model& model, Vector& u, double h):
            OptimizerSystem(numParameters),
            numControls(numParameters),
            si(s),
            osimModel(model),
            excitation(u),
            timestep(h)
    {}

    int objectiveFunc(const Vector &new_yDotGuess, bool new_params, Real& f) const override {
        // No objective function
        f = 0;
        return 0;
    }

    int constraintFunc(const Vector &new_yDotGuess, bool newParams, Vector& constraints) const override {
        //The constraints function is just residuals
        Vector cons=computeImplcitResiduals(osimModel,si,new_yDotGuess,excitation,timestep);
        constraints=cons;
        return 0;
    }

private:
    int numControls;
    State& si;
    Model& osimModel;
    Vector& excitation;
    double timestep;
};





// Perform a forward dynamic step
SimTK::Vector forwardImplict(Model& osimModel, State& s, Vector yPGuess, Vector u, double h) {

    int numControls =6;

    // There must be a better way of doing this instead of creating a new
    // solver at each time step (how do I handle u)
    ImplicitSystemDerivativeSolver sys(6, s, osimModel, u, h);

    Vector lower_bounds(6);
    lower_bounds[0]=0.0;
    lower_bounds[1]=0.0;
    lower_bounds[2]=0.0;
    lower_bounds[3]=0.0;
    lower_bounds[4]=-1.0;
    lower_bounds[5]=-SimTK::Infinity;

    Vector upper_bounds(6);
    upper_bounds[0] = 10.0;
    upper_bounds[1] = 1.0;
    upper_bounds[2] = 10.0;
    upper_bounds[3] = 1.0;
    upper_bounds[4] = 10.0;
    upper_bounds[5] = SimTK::Infinity;

    sys.setParameterLimits( lower_bounds, upper_bounds );

    Optimizer opt(sys, SimTK::InteriorPoint); //Create the optimizer

    // Specify settings for the optimizer
    opt.setConvergenceTolerance(0.1);
    opt.useNumericalGradient(true, 1e-5);
    opt.useNumericalJacobian(true);
    opt.setMaxIterations(100);
    opt.setLimitedMemoryHistory(500);



    opt.optimize(yPGuess);  // Optimize it!


    //Now that we have a good new solution, let's update the state to match
    //First get the muscle and recast
    const Set<Muscle>& muscleSet=osimModel.getMuscles();
    const auto* muscle1 = dynamic_cast<const VandenBogert2011Muscle*>(&muscleSet[0]);
    const auto* muscle2 = dynamic_cast<const VandenBogert2011Muscle*>(&muscleSet[1]);
    muscle1->setFiberLength(s,yPGuess[0]);
    muscle1->setActivation(s,yPGuess[1]);
    muscle2->setFiberLength(s,yPGuess[2]);
    muscle2->setActivation(s,yPGuess[3]);
    //Now the mass
    const CoordinateSet& modelCoordinateSet = osimModel.getCoordinateSet();
    modelCoordinateSet[0].setValue(s,yPGuess[4]);
    modelCoordinateSet[0].setSpeedValue(s,yPGuess[5]);

    return yPGuess;

}

//______________________________________________________________________________
/**
 */
int main()
{


    //Create a model
    // BLOCK BODY
    const double mass = 20.0;
    Model model; model.setName("tug_of_war");
    model.setGravity(Vec3(0, 0, 0)); // No gravity
    auto* body = new OpenSim::Body("ptmass", mass, Vec3(0), Inertia(0));
    auto* slider = new SliderJoint(); slider->setName("slider");
    model.addBody(body);
    model.addJoint(slider);
    slider->updConnector("parent_frame").connect(model.getGround());
    slider->updConnector("child_frame").connect(*body);
    // Get a reference to the model's ground frame
    Ground& ground = model.updGround();

    //Add muscles
    VandenBogert2011Muscle* muscle1 = new VandenBogert2011Muscle();
    VandenBogert2011Muscle* muscle2 = new VandenBogert2011Muscle();

    // Specify the paths for the two muscles
    // Path for muscle 1
    muscle1->addNewPathPoint("muscle1-point1", ground, Vec3(0.0,0.05,-0.35));
    muscle1->addNewPathPoint("muscle1-point2", *body, Vec3(0.0,0.0,-0.05));
    // Path for muscle 2
    muscle2->addNewPathPoint("muscle2-point1", ground, Vec3(0.0,0.05,0.35));
    muscle2->addNewPathPoint("muscle2-point2", *body, Vec3(0.0,0.0,0.05));

    // Add the two muscles (as forces) to the model
    model.addForce(muscle1);
    model.addForce(muscle2);

    model.setUseVisualizer(false);

    // Initialize the system and get the default state
    SimTK::State& s = model.initSystem();

    SimTK::Vec2 activInit = {0.2,0.2};
    muscle1->setActivation(s,activInit[0]);
    muscle2->setActivation(s,activInit[1]);


    //Static Equilibrate Muscle
    //model.equilibrateMuscles(s);
    muscle1->computeInitialFiberEquilibrium(s);
    muscle2->computeInitialFiberEquilibrium(s);

    // Double check that residuals are 0
    SimTK::Vec2 muscle1Res = muscle1->getResidual(s,0.0,0.0,activInit[0]);
    SimTK::Vec2 muscle2Res = muscle2->getResidual(s,0.0,0.0,activInit[1]);

    double ml1 = muscle1->getFiberLength(s);
    double ml2 = muscle2->getFiberLength(s);
    double a1 = muscle1->getActivation(s);
    double a2 = muscle2->getActivation(s);

    cout << "muscleLength1: " << ml1 << " a1: " << a1 <<endl;
    cout << "muscleLength2: " << ml2 << " a2: " << a2 <<endl;
    cout << "m1Res: " << muscle1Res << endl;
    cout << "m2Res: " << muscle2Res << endl;

    //The equilibrium of the muscles (but not necessarily the system)
    Vector yInit(6);  //={ml1,a1,ml2,a2,0.0,0.0};  //s, sdot, s2,s2dot, x, v
    yInit[0]=ml1;
    yInit[1]=a1;
    yInit[2]=ml2;
    yInit[3]=a2;
    yInit[4]=0.0;
    yInit[5]=0.0;

    // Save the model to a file
    // osimModel.print("tugOfWar_model.osim");
    // osimModel.printDetailedInfo(si, std::cout);


    Vector u(2);
    u[0]=0.2;
    u[1]=0.2;

    Vector ynew=yInit;

    //Run a couple of forward steps.  Because I start in equilibrium, this
    //should not change.
    for (int i=1; i<10; i++){

        ynew=forwardImplict( model, s, ynew, u, 0.001);
        //u[1]=u[1]+0.05;
        //cout << u << endl;
        cout << ynew << endl;

    }

    std::cout << "OpenSim environment test completed successfully. You should see a block attached to two muscles visualized in a separate window." << std::endl;

    return 0;
}







