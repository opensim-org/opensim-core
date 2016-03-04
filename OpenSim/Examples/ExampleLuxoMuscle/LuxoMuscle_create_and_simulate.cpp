/* -------------------------------------------------------------------------- *
 *                    OpenSim:  TugOfWar1_CreateModel.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Jeffrey A. Reinbolt, Ayman Habib, Ajay Seth, Jack Middleton,    *
 *            Samuel R. Hamner                                                *
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
 *  main() routine.  This application is a forward simulation of tug-of-war between two
 *  muscles pulling on a block.
 */

// Author:  Matt DeMers

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

//______________________________________________________________________________
/**
 * Material/Mechanical Constants
 */

//base
double baseMass = 1.0; // kg
double baseHeight = 0.1; // meters


//______________________________________________________________________________
/**
 * Helper methods
 */

void createLuxoJr(OpenSim::Model &model);


//______________________________________________________________________________
/**
 * main routine to run the model.
 */
int main()
{
    try {
        // Create an OpenSim model and set its name
        Model luxo;
        luxo.setName("LuxoMuscle");
        
        // This method takes an empty model and fills it with a working
        // Luxo Jr. lamp skeleton!
        createLuxoJr(luxo);
        
        // Turn on 3D visualization for this Luxo lamp model
        luxo.setUseVisualizer(true);
        
        // Pose the model
        State& state = luxo.initSystem();
        
        // Configure the 3D visualizer environment
        luxo.updMatterSubsystem().setShowDefaultGeometry(true);
        Visualizer& viz = luxo.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.GroundAndSky);
        
        
        // show the model!
        viz.report(state);
        
        // enter anything in the command prompt to quit
        std::cout << "Enter anything to quit." << std::endl;
        std::cin.get();
        
        
    }
    catch (OpenSim::Exception ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }
    catch (std::exception ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }

    std::cout << "OpenSim example completed successfully.\n";
    std::cin.get();
    return 0;
}



//______________________________________________________________________________
/**
 * Method for building the Luxo Jr articulating model. It sets up the system of
 * rigid bodies and joint articulations to define Luxo Jr lamp geometry.
 */
void createLuxoJr(OpenSim::Model &model){
    
    // Create base
    OpenSim::Body* base = new OpenSim::Body("base", baseMass, Vec3(0.0),
                Inertia::cylinderAlongY(0.1, baseHeight));
    
    // Add visible geometry
    base->attachMeshGeometry("Base.obj");
    
    // Define base to float relative to ground via free joint
    FreeJoint* base_ground = new FreeJoint("base_ground",
                // parent body, location in parent body, orientation in parent
                model.getGround(), Vec3(0.0), Vec3(0.0),
                // child body, location in child body, orientation in child
                *base, Vec3(0.0,-baseHeight/2.0,0.0),Vec3(0.0));
    
    // add base to model
    model.addBody(base); model.addJoint(base_ground);
    
    
}