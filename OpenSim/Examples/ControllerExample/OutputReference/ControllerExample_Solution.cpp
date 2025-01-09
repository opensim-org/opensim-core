/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ControllerExample.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Chand T. John, Ajay Seth                                        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* 
 *  Below is an extension example of an OpenSim application that provides its own 
 *  main() routine.  It applies a controller to the forward simulation of a tug-of-war 
 *  between two muscles pulling on a block.
 */

// Author:  Chand T. John and Ajay Seth

//==============================================================================
//==============================================================================

// Include OpenSim and functions
#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"

// This allows us to use OpenSim functions, classes, etc., without having to
// prefix the names of those things with "OpenSim::".
using namespace OpenSim;

// This allows us to use SimTK functions, classes, etc., without having to
// prefix the names of those things with "SimTK::".
using namespace SimTK;


//______________________________________________________________________________
/**
 * The controller will try to make the model follow this position
 * in the z direction.
 */
double desiredModelZPosition( double t ) {
    // z(t) = 0.15 sin( pi * t )
    return 0.15 * sin( Pi * t );
}

/**
 * The controller will try to make the model follow this velocity
 * in the z direction.
 */
double desiredModelZVelocity( double t ) {
    // z(t) = 0.15 sin( pi * t )
    return 0.15 * Pi * cos( Pi * t );
}
//______________________________________________________________________________
/**
 * The controller will try to make the model follow this acceleration
 * in the z direction.
 */
double desiredModelZAcceleration( double t ) {
    // z''(t) = -(0.15*pi^2) sin( pi * t )
    return -0.15 * Pi * Pi * sin( Pi * t );
}

//______________________________________________________________________________
/**
 * This controller will try to track a desired trajectory of the block in
 * the tug-of-war model.
 */
class TugOfWarController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(TugOfWarController, Controller);

// This section contains methods that can be called in this controller class.
public:
    /**
     * Constructor
     *
     * @param aModel Model to be controlled
     * @param aKp Position gain by which the position error will be multiplied
     */
    TugOfWarController(double aKp, double aKv) : Controller(), kp( aKp ), kv( aKv ) 
    {
    }

    /**
     * This function is called at every time step for every actuator.
     *
     * @param s Current state of the system
     * @param controls Controls being calculated
     */
    void computeControls(const SimTK::State& s, SimTK::Vector &controls) const
    {
        // Get the current time in the simulation.
        double t = s.getTime();

        // Read the mass of the block.
        double blockMass = getModel().getBodySet().get( "block" ).getMass();

        // Get pointers to each of the muscles in the model.
        Muscle* leftMuscle = dynamic_cast<Muscle*>  ( &getActuatorSet().get(0) );
        Muscle* rightMuscle = dynamic_cast<Muscle*> ( &getActuatorSet().get(1) );

        // Compute the desired position of the block in the tug-of-war
        // model.
        double zdes  = desiredModelZPosition(t);

        // Compute the desired velocity of the block in the tug-of-war
        // model.
        double zdesv = desiredModelZVelocity(t);

        // Compute the desired acceleration of the block in the tug-
        // of-war model.
        double zdesa = desiredModelZAcceleration(t);

        // Get the z translation coordinate in the model.
        const Coordinate& zCoord = _model->getCoordinateSet().
            get( "blockToGround_zTranslation" );

        // Get the current position of the block in the tug-of-war
        // model.
        double z  = zCoord.getValue(s);
        
        // Get the current velocity of the block in the tug-of-war
        // model.
        double zv = zCoord.getSpeedValue(s);

        // Compute the correction to the desired acceleration arising
        // from the deviation of the block's current position from its
        // desired position (this deviation is the "position error").
        double pErrTerm = kp * ( zdes  - z  );
        
        // Compute the total desired velocity based on the initial
        // desired velocity plus the position error term we
        // computed above.
        double vErrTerm = kv * ( zdesv - zv );

        // Compute the total desired acceleration based on the initial
        // desired acceleration plus the position error term we
        // computed above.
        double desAcc = zdesa + vErrTerm + pErrTerm;

        // Compute the desired force on the block as the mass of the
        // block times the total desired acceleration of the block.
        double desFrc = desAcc * blockMass;

        // Get the maximum isometric force for the left muscle.
        double FoptL = leftMuscle->getMaxIsometricForce();

        // Get the maximum isometric force for the right muscle.
        double FoptR = rightMuscle->getMaxIsometricForce();

        // If desired force is in direction of one muscle's pull
        // direction, then set that muscle's control based on desired
        // force.  Otherwise, set the muscle's control to zero.
        double leftControl = 0.0, rightControl = 0.0;
        if( desFrc < 0 ) {
            leftControl = abs( desFrc ) / FoptL;
            rightControl = 0.0;
        }
        else if( desFrc > 0 ) {
            leftControl = 0.0;
            rightControl = abs( desFrc ) / FoptR;
        }
        // Don't allow any control value to be greater than one.
        if( leftControl > 1.0 ) leftControl = 1.0;
        if( rightControl > 1.0 ) rightControl = 1.0;

        // Thelen muscle has only one control
        Vector muscleControl(1, leftControl);
        // Add in the controls computed for this muscle to the set of all model controls
        leftMuscle->addInControls(muscleControl, controls);
        // Specify control for other actuator (muscle) controlled by this controller
        muscleControl[0] = rightControl;
        rightMuscle->addInControls(muscleControl, controls);
    }

// This section contains the member variables of this controller class.
private:

    /** Position gain for this controller */
    double kp;
    /** Velocity gain for this controller */
    double kv;

};


//______________________________________________________________________________
/**
 * Run a forward dynamics simulation with a controller attached to a model.
 * The model consists of a block attached by two muscles to two walls.  The
 * block can make contact with the ground.
 */
int main()
{
    try {

                // Need to load this DLL so muscle types are recognized.
        LoadOpenSimLibrary( "osimActuators" );

        // Create an OpenSim model from the model file provided.
        Model osimModel( "tugOfWar_model_ThelenOnly.osim" );

        // Define the initial and final simulation times.
        double initialTime = 0.0;
        double finalTime = 1.0;

        // Set gain for the controller.
        double kp = 1600.0; // position gain
        double kv = 80.0;   // velocity gain

        // Print the control gains and block mass.
        std::cout << std::endl;
        std::cout << "kp = " << kp << std::endl;
        std::cout << "kv = " << kv << std::endl;
        
        // Create the controller.
        TugOfWarController *controller = new TugOfWarController(kp, kv);

        // Give the controller the Model's actuators so it knows
        // to control those actuators.
        controller->setActuators( osimModel.updActuators() );

        // Add the controller to the Model.
        osimModel.addController( controller );

        // Initialize the system and get the state representing the
        // system.
        SimTK::State& si = osimModel.initSystem();

        // Define non-zero (defaults are 0) states for the free joint.
        CoordinateSet& modelCoordinateSet =
            osimModel.updCoordinateSet();
        // Get the z translation coordinate.
        Coordinate& zCoord = modelCoordinateSet.
            get( "blockToGround_zTranslation" );
        // Set z translation speed value.
        zCoord.setSpeedValue( si, 0.15 * Pi );

        // Define the initial muscle states.
        const Set<Muscle>& muscleSet = osimModel.getMuscles();
        ActivationFiberLengthMuscle* muscle1 = dynamic_cast<ActivationFiberLengthMuscle*>( &muscleSet.get(0) );
        ActivationFiberLengthMuscle* muscle2 = dynamic_cast<ActivationFiberLengthMuscle*>( &muscleSet.get(1) );
        if((muscle1 == NULL) || (muscle2 == NULL)){
            throw OpenSim::Exception("ControllerExample: muscle1 or muscle2 is not an ActivationFiberLengthMuscle and example cannot proceed.");
        }
        muscle1->setActivation(si, 0.01 ); // muscle1 activation
        muscle1->setFiberLength(si, 0.2 ); // muscle1 fiber length
        muscle2->setActivation(si, 0.01 ); // muscle2 activation
        muscle2->setFiberLength(si, 0.2 ); // muscle2 fiber length

        // Create the integrator and manager for the simulation.
        SimTK::RungeKuttaMersonIntegrator
            integrator( osimModel.getMultibodySystem() );
        integrator.setAccuracy( 1.0e-4 );

        Manager manager( osimModel, integrator );

        // Examine the model.
        osimModel.printDetailedInfo( si, std::cout );

        // Print out the initial position and velocity states.
        for( int i = 0; i < modelCoordinateSet.getSize(); i++ ) {
            std::cout << "Initial " << modelCoordinateSet[i].getName()
                << " = " << modelCoordinateSet[i].getValue( si )
                << ", and speed = "
                << modelCoordinateSet[i].getSpeedValue( si ) << std::endl;
        }

        // Integrate from initial time to final time.
        manager.setInitialTime( initialTime );
        manager.setFinalTime( finalTime );
        std::cout << "\n\nIntegrating from " << initialTime
            << " to " << finalTime << std::endl;
        manager.integrate( si );

        // Save the simulation results.
        auto controlsTable = osimModel.getControlsTable();
        STOFileAdapter::write(controlsTable, "tugOfWar_controls.sto");

        auto statesTable = manager.getStatesTable();
        STOFileAdapter::write(statesTable, "tugOfWar_states.sto");
    }
    catch (const std::exception &ex) {
        
        // In case of an exception, print it out to the screen.
        std::cout << ex.what() << std::endl;

        // Return 1 instead of 0 to indicate that something
        // undesirable happened.
        return 1;
    }

    // If this program executed up to this line, return 0 to
    // indicate that the intended lines of code were executed.
    return 0;
}
