/* -------------------------------------------------------------------------- *
 *                         OpenSim:  mainFatigue.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan, Ajay Seth, Ayman Habib                              *
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
 *  Below is an example of an OpenSim main() routine.  
 *  This program is a forward simulation of a tug-of-war between two muscles 
 *  pulling on a block. One of the muscles fatigues and the other does not.
 */

//=============================================================================
//=============================================================================
#include <OpenSim/OpenSim.h>
#include "FatigableMuscle.h"
#include <OpenSim/Common/IO.h>
#include "OpenSim/Common/STOFileAdapter.h"

using namespace OpenSim;
using namespace SimTK;

//_____________________________________________________________________________
/**
 * Run a simulation of a sliding block being pulled by two muscle 
 */
int main()
{
    std::clock_t startTime = std::clock();

    try {
        ///////////////////////////////////////////////
        // DEFINE THE SIMULATION START AND END TIMES //
        ///////////////////////////////////////////////
        // Define the initial and final simulation times
        double initialTime = 0.0;
        double finalTime = 10.0;

        ///////////////////////////////////////////
        // DEFINE BODIES AND JOINTS OF THE MODEL //
        ///////////////////////////////////////////
        // Create an OpenSim model and set its name
        Model osimModel;
        osimModel.setName("tugOfWar");

        // GROUND FRAME

        // Get a reference to the model's ground body
        Ground& ground = osimModel.updGround();

        // Add display geometry to the ground to visualize in the GUI
        ground.attachGeometry(new Mesh("ground.vtp"));
        ground.attachGeometry(new Mesh("anchor1.vtp"));
        ground.attachGeometry(new Mesh("anchor2.vtp"));

        // BLOCK BODY

        // Specify properties of a 20 kg, 10cm length block body
        double blockMass = 20.0, blockSideLength = 0.1;
        Vec3 blockMassCenter(0);
        Inertia blockInertia = blockMass*Inertia::brick(blockSideLength, 
            blockSideLength, blockSideLength);

        // Create a new block body with the specified properties
        OpenSim::Body *block = new OpenSim::Body("block", blockMass, 
            blockMassCenter, blockInertia);

        // Add display geometry to the block to visualize in the GUI
        block->attachGeometry(new Mesh("block.vtp"));

        // FREE JOINT

        // Create a new free joint with 6 degrees-of-freedom (coordinates) 
        // between the block and ground bodies
        double halfLength = blockSideLength/2.0;
        Vec3 locationInParent(0, halfLength, 0), orientationInParent(0);
        Vec3 locationInBody(0, halfLength, 0), orientationInBody(0);
        FreeJoint *blockToGround = new FreeJoint("blockToGround", ground, 
            locationInParent, orientationInParent, 
            *block, locationInBody, orientationInBody);

        // Set the angle and position ranges for the free (6-degree-of-freedom)
        // joint between the block and ground frames.
        double angleRange[2] = {-SimTK::Pi/2, SimTK::Pi/2};
        double positionRange[2] = {-1, 1};
        blockToGround->updCoordinate(FreeJoint::Coord::Rotation1X).setRange(angleRange);
        blockToGround->updCoordinate(FreeJoint::Coord::Rotation2Y).setRange(angleRange);
        blockToGround->updCoordinate(FreeJoint::Coord::Rotation3Z).setRange(angleRange);
        blockToGround->updCoordinate(FreeJoint::Coord::TranslationX).setRange(positionRange);
        blockToGround->updCoordinate(FreeJoint::Coord::TranslationY).setRange(positionRange);
        blockToGround->updCoordinate(FreeJoint::Coord::TranslationZ).setRange(positionRange);

        // Add the block body to the model
        osimModel.addBody(block);
        osimModel.addJoint(blockToGround);

        ///////////////////////////////////////
        // DEFINE FORCES ACTING ON THE MODEL //
        ///////////////////////////////////////
        // MUSCLE FORCES

        // Create two new muscles
        double maxIsometricForce = 1000.0, optimalFiberLength = 0.2, 
               tendonSlackLength = 0.1,    pennationAngle = 0.0,  
               fatigueFactor = 0.30, recoveryFactor = 0.20;

        // fatigable muscle (Millard2012EquilibriumMuscle with fatigue)
        FatigableMuscle* fatigable = new FatigableMuscle("fatigable",
            maxIsometricForce, optimalFiberLength, tendonSlackLength, 
            pennationAngle, fatigueFactor, recoveryFactor);

        // original muscle model (muscle without fatigue)
        Millard2012EquilibriumMuscle* original = 
            new Millard2012EquilibriumMuscle("original",
                maxIsometricForce, optimalFiberLength, tendonSlackLength,
                pennationAngle);

        // Define the path of the muscles
        fatigable->addNewPathPoint("fatigable-point1", ground, 
            Vec3(0.0, halfLength, -0.35));
        fatigable->addNewPathPoint("fatigable-point2", *block, 
            Vec3(0.0, halfLength, -halfLength));

        original->addNewPathPoint("original-point1", ground, 
            Vec3(0.0, halfLength, 0.35));
        original->addNewPathPoint("original-point2", *block, 
            Vec3(0.0, halfLength, halfLength));

        // Define the default states for the two muscles
        // Activation
        fatigable->setDefaultActivation(0.01);
        original->setDefaultActivation(0.01);
        // Fiber length
        fatigable->setDefaultFiberLength(optimalFiberLength);
        original->setDefaultFiberLength(optimalFiberLength);

        // Add the two muscles (as forces) to the model
        osimModel.addForce(fatigable);
        osimModel.addForce(original);

        ///////////////////////////////////
        // DEFINE CONTROLS FOR THE MODEL //
        ///////////////////////////////////
        // Create a prescribed controller that simply supplies controls as 
        // a function of time.
        // For muscles, controls are normalized stoor-neuron excitations
        PrescribedController *muscleController = new PrescribedController();
        muscleController->setActuators(osimModel.updActuators());
    
        // Set the prescribed muscle controller to use the same muscle control function for each muscle
        muscleController->prescribeControlForActuator("fatigable", new Constant(1.0));
        muscleController->prescribeControlForActuator("original", new Constant(1.0));

        // Add the muscle controller to the model
        osimModel.addController(muscleController);

        // Add a Muscle analysis
        MuscleAnalysis* muscAnalysis = new MuscleAnalysis(&osimModel);
        Array<std::string> coords(blockToGround->getCoordinate(FreeJoint::Coord::TranslationZ).getName(),1);
        muscAnalysis->setCoordinates(coords);
        muscAnalysis->setComputeMoments(false);
        osimModel.addAnalysis(muscAnalysis);

        // Turn on the visualizer to view the simulation run live.
        osimModel.setUseVisualizer(false);

        //////////////////////////
        // PERFORM A SIMULATION //
        //////////////////////////

        // Initialize the system and get the state
        SimTK::State& si = osimModel.initSystem();

        // Init coords to 0 and lock the rotational degrees of freedom so the block doesn't twist
        CoordinateSet& coordinates = osimModel.updCoordinateSet();
        coordinates[0].setValue(si, 0);
        coordinates[1].setValue(si, 0);
        coordinates[2].setValue(si, 0);
        coordinates[3].setValue(si, 0);
        coordinates[4].setValue(si, 0); 
        coordinates[5].setValue(si, 0);
        coordinates[0].setLocked(si, true);
        coordinates[1].setLocked(si, true);
        coordinates[2].setLocked(si, true);
        // Last coordinate (index 5) is the Z translation of the block
        coordinates[4].setLocked(si, true); 

        // Compute initial conditions for muscles
        osimModel.equilibrateMuscles(si);

        // Create the force reporter
        ForceReporter* reporter = new ForceReporter(&osimModel);
        osimModel.updAnalysisSet().adoptAndAppend(reporter);
        // Create the manager
        Manager manager(osimModel);
        manager.setIntegratorAccuracy(1.0e-6);

        // Print out details of the model
        osimModel.printDetailedInfo(si, std::cout);

        // Integrate from initial time to final time
        si.setTime(initialTime);
        manager.initialize(si);
        std::cout<<"\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
        manager.integrate(finalTime);

        //////////////////////////////
        // SAVE THE RESULTS TO FILE //
        //////////////////////////////

        // Save the simulation results
        // Save the states
        auto statesTable = manager.getStatesTable();
        STOFileAdapter_<double>::write(statesTable, 
                                      "tugOfWar_fatigue_states.sto");

        auto forcesTable = reporter->getForcesTable();
        STOFileAdapter_<double>::write(forcesTable, 
                                      "tugOfWar_fatigue_forces.sto");

        // Save the muscle analysis results
        IO::makeDir("MuscleAnalysisResults");
        muscAnalysis->printResults("fatigue", "MuscleAnalysisResults");

        // To print (serialize) the latest connections of the model, it is 
        // necessary to finalizeConnections() first.
        osimModel.finalizeConnections();
        // Save the OpenSim model to a file
        osimModel.print("tugOfWar_fatigue_model.osim");
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }

    std::cout << "main() routine time = " << 1.e3*(std::clock()-startTime)/CLOCKS_PER_SEC << "ms\n";

    std::cout << "OpenSim example completed successfully.\n";
    return 0;
}
