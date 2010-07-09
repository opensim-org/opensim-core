// ControllerExample.cpp

/* Copyright (c)  2009 Stanford University
 * Use of the OpenSim software in source form is permitted provided that the following
 * conditions are met:
 *   1. The software is used only for non-commercial research and education. It may not
 *     be used in relation to any commercial activity.
 *   2. The software is not distributed or redistributed.  Software distribution is allowed 
 *     only through https://simtk.org/home/opensim.
 *   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
 *      presentations, or documents describing work in which OpenSim or derivatives are used.
 *   4. Credits to developers may not be removed from executables
 *     created from modifications of the source.
 *   5. Modifications of source code must retain the above copyright notice, this list of
 *     conditions and the following disclaimer. 
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 *  Below is an example of an OpenSim application that provides its own 
 *  main() routine.  This application is a forward simulation of tug-of-war between two
 *  muscles pulling on a block.
 */

// Author:  Chand T. John

//==============================================================================
//==============================================================================

// This line includes a large number of OpenSim functions and classes so that
// those things will be available to this program.
#include <OpenSim/OpenSim.h>

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
//////////////////////////////////////////////////////////////////////
// 1) Add a function to compute the desired velocity of the model   //
//    in the z direction.                                           //
//////////////////////////////////////////////////////////////////////
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

// This section contains methods that can be called in this controller class.
public:

	/**
	 * Constructor
	 *
	 * @param aModel Model to be controlled
	 * @param aKp Position gain by which the position error will be multiplied
	 */
	/////////////////////////////////////////////////////////////
	// 2) Add a parameter aKv for velocity gain to the         //
	//    argument list for this function.  Also add this      //
	//    parameter to the initializer list below so that a    //
	//    new member variable kv is initialized to the value   //
	//    of aKv.  Remember to add a line describing aKv in    //
	//    the comment above (below the line describing aKp).   //
	/////////////////////////////////////////////////////////////
	TugOfWarController( Model& aModel, double aKp ) : 
		Controller( aModel ), kp( aKp ) {

		// Read the mass of the block.
		blockMass = aModel.getBodySet().get( "block" ).getMass();
		std::cout << std::endl << "blockMass = " << blockMass
			<< std::endl;
	}

	/**
	 * This function is called at every time step for every actuator.
	 *
	 * @param s Current state of the system
	 * @param index Index of the current actuator whose control is being calculated
	 * @return Control value to be assigned to the current actuator at the current time
	 */
	virtual double computeControl( const SimTK::State& s, int index )
	const {

		// Get the current time in the simulation.
		double t = s.getTime();

		// Get a pointer to the current muscle whose control is being
		// calculated.
		Muscle* act = dynamic_cast<Muscle*>
			( &_actuatorSet.get( index ) );

		// Compute the desired position of the block in the tug-of-war
		// model.
		double zdes  = desiredModelZPosition(t);

		//////////////////////////////////////////////////////////////
		// 3) Compute the desired velocity of the block in the tug- //
		//    of-war model.  Create a new variable zdesv to hold    //
		//    this value.                                           //
		//////////////////////////////////////////////////////////////

		// Compute the desired acceleration of the block in the tug-
		// of-war model.
		double zdesa = desiredModelZAcceleration(t);

		// Get the z translation coordinate in the model.
		const Coordinate& zCoord = _model->getCoordinateSet().
			get( "blockToGround_zTranslation" );

		// Get the current position of the block in the tug-of-war
		// model.
		double z  = zCoord.getValue(s);

		//////////////////////////////////////////////////////////////
		// 4) Get the current velocity of the block in the tug-of-  //
		//    war model.  Create a new variable zv to hold this     //
		//    value.                                                //
		//////////////////////////////////////////////////////////////

		// Compute the correction to the desired acceleration arising
		// from the deviation of the block's current position from its
		// desired position (this deviation is the "position error").
		double pErrTerm = kp * ( zdes  - z  );

		//////////////////////////////////////////////////////////////
		// 5) Compute the correction to the desired acceleration    //
		//     arising from the deviation of the block's current    //
		//     velocity from its desired velocity (this deviation   //
		//     is the "velocity error").  Create a new variable     //
		//     vErrTerm to hold this value.                         //
		//////////////////////////////////////////////////////////////

		//////////////////////////////////////////////////////////////
		// 6) In the computation of desAcc below, add the velocity  //
		//    error term you created in item #5 above.  Please      //
		//    update the comment for desAcc below to reflect this   //
		//    change.                                               //
		//////////////////////////////////////////////////////////////

		// Compute the total desired acceleration based on the initial
		// desired acceleration plus the position error term we
		// computed above.
		double desAcc = zdesa + pErrTerm;

		// Compute the desired force on the block as the mass of the
		// block times the total desired acceleration of the block.
		double desFrc = desAcc * blockMass;

		// Get the maximum isometric force of the current muscle.
		double Fopt = act->getMaxIsometricForce();

		// Now, compute the control value for the current muscle.
		double newControl;

		// If desired force is in direction of current muscle's pull
		// direction, then set the muscle's control based on desired
		// force.  Otherwise, set the current muscle's control to
		// zero.
		if( desFrc < 0 && index == 0 || // index 0: left muscle
			desFrc > 0 && index == 1 )  // index 1: right muscle
			newControl = abs( desFrc ) / Fopt;
		else
			newControl = 0.0;

		// Don't allow the control value to be less than zero.
		if( newControl < 0.0 ) newControl = 0.0;

		// Don't allow the control value to be greater than one.
		if( newControl > 1.0 ) newControl = 1.0;

		// Return the final computed control value for the current
		// muscle.
		return newControl;
	}

// This section contains the member variables of this controller class.
private:

	/** Position gain for this controller */
	double kp;

	//////////////////////////////////////////////////////////////
	// 7) Add a member variable kv that is the velocity gain    //
	//    for this controller.                                  //
	//////////////////////////////////////////////////////////////

	/**
	 * Mass of the block in the tug-of-war model, used to compute the
	 * desired force on the block at each time step in a simulation
	 */
	double blockMass;
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
		double finalTime = 2.0;

		// Set gain for the controller.
		double kp = 100.0; // position gain

		////////////////////////////////////////////////////////
		// 8) Create a velocity gain variable kv above.       //
		//    Also, increase the gains to kp = 1600.0 and     //
		//    initialize kv to be 80.0.                       //
		////////////////////////////////////////////////////////

		// Print the control gains and block mass.
		std::cout << std::endl;
		std::cout << "kp = " << kp << std::endl;

		////////////////////////////////////////////////////////
		// 9) Print out the value of kv above.                //
		////////////////////////////////////////////////////////

		////////////////////////////////////////////////////////
		// 10) Add kv as an argument in the instantiation of  //
		//     the controller below.                          //
		////////////////////////////////////////////////////////

		// Create the controller.
		TugOfWarController *controller = new
			TugOfWarController( osimModel, kp );

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
		const Set<Actuator>& actuatorSet = osimModel.getActuators();
		Muscle* muscle1 =
			dynamic_cast<Muscle*>( &actuatorSet.get(0) );
		Muscle* muscle2 =
			dynamic_cast<Muscle*>( &actuatorSet.get(1) );
		muscle1->setActivation(si, 0.01 ); // muscle1 activation
		muscle1->setFiberLength(si, 0.2 ); // muscle1 fiber length
		muscle2->setActivation(si, 0.01 ); // muscle2 activation
		muscle2->setFiberLength(si, 0.2 ); // muscle2 fiber length


        // Compute initial conditions for muscles.
		//osimModel.equilibrateMuscles( si );

		// Create the integrator and manager for the simulation.
		SimTK::RungeKuttaMersonIntegrator
			integrator( osimModel.getMultibodySystem() );
		integrator.setAccuracy( 1.0e-4 );
		integrator.setAbsoluteTolerance( 1.0e-4 );
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
		osimModel.printControlStorage( "tugOfWar_controls.sto" );
		Storage statesDegrees( manager.getStateStorage() );
		statesDegrees.print( "tugOfWar_states.sto" );
		osimModel.updSimbodyEngine().convertRadiansToDegrees
			( statesDegrees );
		statesDegrees.setWriteSIMMHeader( true );
		statesDegrees.print( "tugOfWar_states_degrees.mot" );
	}
    catch ( std::exception ex ) {
		
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
