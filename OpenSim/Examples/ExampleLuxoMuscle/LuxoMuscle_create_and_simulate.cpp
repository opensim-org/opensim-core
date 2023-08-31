/* -------------------------------------------------------------------------- *
 *                    OpenSim:  TugOfWar1_CreateModel.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
 *  Below is an example of an OpenSim application that builds a Luxo lamp with
 *  muscles and simulates hopping.*/

// Author:  Matt DeMers

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"

using namespace OpenSim;
using namespace SimTK;

//______________________________________________________________________________
/**
 * Material and Mechanical Constants:
 */

// TO DO: Find a way to set an ideal camera pose
UnitVec3 camera_offset_direction(1,1,1);
double camera_offset_distance = 2; // meters


// Intergrator Settings
//----------------------
double sim_time = 5; // seconds

// Muscle control settings
//------------------------
double low_excitation = 0.3; // fraction of effort at start
double high_excitation = 1.0 ; // fraction of effort during jump
double activate_time = 2.0; // time jumping maneuver starts
double deactivate_time = 3.0; // time jumping ends
double times[4] = { 0.0,
                    activate_time,
                    deactivate_time,
                    sim_time};
double excitations[4] = {low_excitation,
                         high_excitation,
                         low_excitation,
                         low_excitation};

// Skeleton Constants
//--------------------

double total_mass = 5; // kg
// change total_mass to increase mass of all body segments.
// the base and head are the most massive, totalling 0.5 body mass
// the rest is destributed in the linkages and brackets. All linkages
// are 1/32 of the total mass and all brackets are 4/32 of the total mass,
// ultimately adding up to 0.5 body mass.
double bar_mass = total_mass*0.5/16; // kg
double bracket_mass = total_mass*0.5*4/16; // kg

// joints will have softstops to resist hyperflexion/extension
double joint_softstop_stiffness = 0.5; // Newton-meters/degree
double joint_softstop_damping = 0.005; // Newton-meters/(degree/second)
double transition_region = 5.0; // degrees
double knee_flexion_min = 10.0, knee_flexion_max = 55; //
double back_extension_min = -60, back_extension_max = -10.0; //

// ground contact parameters
double stiffness = 1.0e8, dissipation = 5, friction = 0.8, viscosity = 0.1;





// Bodies
//--------

//base
double baseMass = total_mass*0.25; // kg
double baseHeight = 0.027225; // meters

//bracket
double bracket_location = 0.03; // meters
Vec3 posterior_bracket_hinge_location(-0.01, -0.01, 0.0); // meters
Vec3 anterior_bracket_hinge_location(0.01, 0.003, 0.0); // meters
Vec3 pivot_damping(0.02);

// leg bar
Vec3 leg_bar_dimensions(0.006980, 0.046992, 0.006979); // meters
Vec3 superior_bar_hinge_location(0.0,
                        0.5*(leg_bar_dimensions[1] - leg_bar_dimensions[0]),
                        0.0);
Vec3 inferior_bar_hinge_location = -1*superior_bar_hinge_location;

// leg Hlink
Vec3 leg_Hlink_dimensions(0.005211, 0.051740, 0.031919); // meters
Vec3 superior_Hlink_hinge_location(0.0,
                                   0.4*leg_Hlink_dimensions[1],
                                   0.0);
Vec3 inferior_Hlink_hinge_location(0.0,
                                   -0.45*leg_Hlink_dimensions[1],
                                   0.0);

// pelvis bracket
Vec3 pelvis_dimensions(0.027673, 0.044470, 0.009772); // meters
Vec3 inferior_pelvis_pin_location(-0.006, -0.015, 0.0);
Vec3 anterior_superior_pelvis_pin_location(0.0135, 0.002,0.0);
Vec3 posterior_superior_pelvis_pin_location(-0.004, 0.022, 0.0);

//  torso bars
Vec3 torso_bar_dimensions(0.003599, 0.049114, 0.003599);
Vec3 superior_torso_hinge_location(0.0,
                                   0.45*torso_bar_dimensions[1],
                                   0.0);
Vec3 inferior_torso_hinge_location(0.0,
                                   -0.45*torso_bar_dimensions[1],
                                   0.0);
Vec3 back_peg_center = inferior_torso_hinge_location*0.85;
Vec3 peg_z_offset(0.0,0.0,0.0178);

// shoulder bracket
Vec3 shoulder_dimensions(0.024053, 0.030492, 0.013800); // meters
Vec3 anterior_thoracic_joint_center(0.003418, -0.0163, 0.0);
Vec3 posterior_thoracic_joint_center(-0.01240, 0.0, 0.0);
Vec3 superior_shoulder_hinge_location(0.005,0.005,0.0);

// head lamp
double head_mass = 0.25*total_mass; // kg
Vec3 head_dimension(0.067823, 0.078413, 0.078412);
Vec3 cervicle_joint_center(-0.01717, -0.03506, 0.0);




// Muscles
//----------
double pennationAngle = 0.0;

double knee_extensor_F0 = 100.0,
       knee_extensor_lm0 = 0.02,
       knee_extensor_lts = 0.0;
Vec3 knee_extensor_insertion(0.0101, 0.0136, 0.0089),
     knee_extensor_origin(0.0, 0.0018, knee_extensor_insertion[2]);


double back_extensor_F0 = 100.0,
       back_extensor_lm0 = 0.035,
       back_extensor_lts = 0.01;
Vec3 back_extensor_origin = 0.5*peg_z_offset,
back_extensor_insertion = back_peg_center + 0.5*peg_z_offset;

// Frame locations for assistive device attachments
//--------------------------------------------------
Transform back_assist_origin_transform(back_peg_center); // in chest piece
Transform back_assist_insertion_transform(
                                  posterior_superior_pelvis_pin_location);

Transform knee_assist_origin_transform(Vec3(0)); // in posterior leg bar
Transform knee_assist_insertion_transform(Vec3(knee_extensor_insertion[0],
                                               knee_extensor_insertion[1],
                                               0.0));



//______________________________________________________________________________
/**
 * Helper methods
 */

void createLuxoJr(OpenSim::Model &model);


//______________________________________________________________________________
/**
 * main routine to run the model.
 */
int main(int argc, char* argv[]) {
    bool showVisualizer{true};
    if(argc > 1 && std::string{argv[1]} == "noVisualizer")
            showVisualizer = false;

    try {
        // Create an OpenSim model and set its name
        Model luxo;
        luxo.setName("LuxoMuscle");
        
        // This method takes an empty model and fills it with a working
        // Luxo Jr. lamp skeleton!
        createLuxoJr(luxo);
        std::cout << "Finished making Luxo Jr skeleton" << std::endl;
        
        // Turn on 3D visualization for this Luxo lamp model
        luxo.setUseVisualizer(showVisualizer);
        luxo.updDisplayHints().set_show_frames(true);
        
        // Pose the model
        State& state = luxo.initSystem();
        std::cout << "State initialized." << std::endl;
        
        // Configure the 3D visualizer environment
        Visualizer* viz{};
        if(showVisualizer) {
            luxo.updMatterSubsystem().setShowDefaultGeometry(false);
            viz = &luxo.updVisualizer().updSimbodyVisualizer();
            viz->setBackgroundType(viz->GroundAndSky);
            viz->setShowSimTime(true);
        }
        
        SimTK::Transform camera_pose(
                             camera_offset_distance*camera_offset_direction);
        camera_pose.updR().setRotationFromOneAxis(
                              camera_offset_direction.negate(), ZAxis);
        
        //viz.setCameraTransform(camera_pose);
        //viz.pointCameraAt(camera_look_at, Vec3(0,1,0));
        
        // show the model!
        if(showVisualizer)
            viz->report(state);
        
        // Create the force reporter for obtaining the forces applied to the
        // model during a forward simulation
        ForceReporter* reporter = new ForceReporter(&luxo);
        luxo.addAnalysis(reporter);
        
        //setup simulation
        // Create the manager managing the forward integration and its outputs
        Manager manager(luxo);
        manager.setIntegratorAccuracy(1.0e-6);
        
        // Print out details of the model
        luxo.printDetailedInfo(state, std::cout);
        
        
        // Integrate from initial time to final time
        
        
        state.setTime(0.0);
        manager.initialize(state);
        std::cout<<"Integrating for " << sim_time << " seconds" <<std::endl;
        manager.integrate(sim_time);
        std::cout<<"Integration finished."<<std::endl;
        
        //////////////////////////////
        // SAVE THE RESULTS TO FILE //
        //////////////////////////////
        
        // Save the model
        luxo.print("Luxo_Myo.osim");
        
        // Save the model states from forward integration
        auto statesTable = manager.getStatesTable();
        STOFileAdapter::write(statesTable, "luxo_states.sto");

        // Save the forces
        auto forcesTable = reporter->getForcesTable();
        STOFileAdapter::write(forcesTable, "luxo_forces.sto");
        
        std::cout << "OpenSim example completed successfully.\n";
        // enter anything in the command prompt to quit
        if(showVisualizer) {
            std::cout << "Enter anything to quit." << std::endl;
            std::cin.get();
        }
    }
    catch (const OpenSim::Exception& ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
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
    
    
    
    return 0;
}



//______________________________________________________________________________
/**
 * Method for building the Luxo Jr articulating model. It sets up the system of
 * rigid bodies and joint articulations to define Luxo Jr lamp geometry.
 */
void createLuxoJr(OpenSim::Model& model){
    
    // Create base
    //--------------
    OpenSim::Body* base = new OpenSim::Body("base", baseMass, Vec3(0.0),
                Inertia::cylinderAlongY(0.1, baseHeight));
    
    // Add visible geometry
    base->attachGeometry(new Mesh("Base_meters.obj"));
    
    
    // Define base to float relative to ground via free joint
    FreeJoint* base_ground = new FreeJoint("base_ground",
                // parent body, location in parent body, orientation in parent
                model.getGround(), Vec3(0.0), Vec3(0.0),
                // child body, location in child body, orientation in child
                *base, Vec3(0.0,-baseHeight/2.0,0.0),Vec3(0.0));
    
    // add base to model
    model.addBody(base); model.addJoint(base_ground);
    
    /*for (int i = 0; i<base_ground->get_CoordinateSet().getSize(); ++i) {
        base_ground->upd_CoordinateSet()[i].set_locked(true);
    }*/
    
    // Fix a frame to the base axis for attaching the bottom bracket
    SimTK::Transform* shift_and_rotate = new SimTK::Transform();
    //shift_and_rotate->setToZero();
    shift_and_rotate->set(Rotation(-1*SimTK::Pi/2,
                                   SimTK::CoordinateAxis::XCoordinateAxis()),
                          Vec3(0.0, bracket_location, 0.0));

    auto* pivot_frame_on_base = new PhysicalOffsetFrame("pivot_frame_on_base",
            *base, *shift_and_rotate);

    // Create bottom bracket
    //-----------------------
    OpenSim::Body* bottom_bracket = new OpenSim::Body("bottom_bracket",
                                            bracket_mass, Vec3(0.0),
                                            Inertia::brick(0.03, 0.03, 0.015));
    // add bottom bracket to model
    model.addBody(bottom_bracket);
    
    // Fix a frame to the bracket for attaching joint
    shift_and_rotate->setP(Vec3(0.0));
    auto* pivot_frame_on_bottom_bracket = new PhysicalOffsetFrame(
        "pivot_frame_on_bottom_bracket", *bottom_bracket, *shift_and_rotate);
    
    // Add visible geometry
    bottom_bracket->attachGeometry(new Mesh("bottom_bracket_meters.obj"));

    // Make bottom bracket to twist on base with vertical pin joint.
    // You can create a joint from any existing physical frames attached to
    // rigid bodies. One way to reference them is by name, like this...
    PinJoint* base_pivot = new PinJoint("base_pivot", *pivot_frame_on_base,
                                        *pivot_frame_on_bottom_bracket);

    base_pivot->addFrame(pivot_frame_on_base);
    base_pivot->addFrame(pivot_frame_on_bottom_bracket);
    // add base pivot joint to the model
    model.addJoint(base_pivot);
    
    // add some damping to the pivot
    // initialized to zero stiffness and damping
    BushingForce* pivotDamper = new BushingForce("pivot_bushing",
                                            *pivot_frame_on_base,
                                            *pivot_frame_on_bottom_bracket);

    pivotDamper->set_rotational_damping(pivot_damping);
    
    model.addForce(pivotDamper);
    
    // Create posterior leg
    //-----------------------
    OpenSim::Body* posteriorLegBar = new OpenSim::Body("posterior_leg_bar",
                                    bar_mass,
                                    Vec3(0.0),
                                    Inertia::brick(leg_bar_dimensions/2.0));
    
    posteriorLegBar->attachGeometry(new Mesh("Leg_meters.obj"));

    auto* posterior_knee_on_bottom_bracket = new PhysicalOffsetFrame(
        "posterior_knee_on_bottom_bracket",
            *bottom_bracket, Transform(posterior_bracket_hinge_location) );

    auto* posterior_knee_on_posterior_bar = new PhysicalOffsetFrame(
        "posterior_knee_on_posterior_bar",
            *posteriorLegBar, Transform(inferior_bar_hinge_location) );

    // Attach posterior leg to bottom bracket using another pin joint.
    // Another way to reference physical frames in a joint is by creating them
    // in place, like this...
    OpenSim::PinJoint* posteriorKnee = new OpenSim::PinJoint("posterior_knee",
                                    *posterior_knee_on_bottom_bracket,
                                    *posterior_knee_on_posterior_bar);
    // posteriorKnee will own and serialize the attachment offset frames 
    posteriorKnee->addFrame(posterior_knee_on_bottom_bracket);
    posteriorKnee->addFrame(posterior_knee_on_posterior_bar);

    
    // add posterior leg to model
    model.addBody(posteriorLegBar); model.addJoint(posteriorKnee);
    
    // allow this joint's coordinate to float freely when assembling constraints
    // the joint we create next will drive the pose of the 4-bar linkage
    posteriorKnee->updCoordinate().set_is_free_to_satisfy_constraints(true);
    
    // Create anterior leg Hlink
    //----------------------------
    OpenSim::Body* leg_Hlink = new OpenSim::Body("leg_Hlink",
                                    bar_mass,
                                    Vec3(0.0),
                                    Inertia::brick(leg_Hlink_dimensions/2.0));
    
    leg_Hlink->attachGeometry(new Mesh("H_Piece_meters.obj"));
    
    
    auto* anterior_knee_on_bottom_bracket = new PhysicalOffsetFrame(
        "anterior_knee_on_bottom_bracket",
            *bottom_bracket, Transform(anterior_bracket_hinge_location));

    
    auto* anterior_knee_on_anterior_bar = new PhysicalOffsetFrame(
        "anterior_knee_on_anterior_bar",
            *leg_Hlink, Transform(inferior_Hlink_hinge_location));


    // Connect anterior leg to bottom bracket via pin joint
    OpenSim::PinJoint* anterior_knee = new OpenSim::PinJoint("anterior_knee",
                                    *anterior_knee_on_bottom_bracket,
                                    *anterior_knee_on_anterior_bar);
    anterior_knee->addFrame(anterior_knee_on_bottom_bracket);
    anterior_knee->addFrame(anterior_knee_on_anterior_bar);

    
    // add anterior leg to model
    model.addBody(leg_Hlink); model.addJoint(anterior_knee);
    
    // this anterior knee joint defines the motion of the lower 4-bar linkage
    // set it's default coordinate value to a slightly flexed position.
    anterior_knee->updCoordinate().set_default_value(SimTK::Pi/6);
    
    // Create pelvis bracket
    //-----------------------
    OpenSim::Body* pelvisBracket = new OpenSim::Body("pelvis_bracket",
                                            bracket_mass,
                                            Vec3(0.0),
                                        Inertia::brick(pelvis_dimensions/2.0));
    
    pelvisBracket->attachGeometry(new Mesh("Pelvis_bracket_meters.obj"));
    
    // Connect pelvis to Hlink via pin joint
    SimTK::Transform pelvis_anterior_shift(
                                        anterior_superior_pelvis_pin_location);

    auto* anterior_hip_on_Hlink = new PhysicalOffsetFrame(
        "anterior_hip_on_Hlink",
        *leg_Hlink, Transform(superior_Hlink_hinge_location));

    auto* anterior_hip_on_pelvis = new PhysicalOffsetFrame(
            "anterior_hip_on_pelvis",
        *pelvisBracket, pelvis_anterior_shift);

    OpenSim::PinJoint* anteriorHip = new OpenSim::PinJoint("anterior_hip",
                                                    *anterior_hip_on_Hlink,
                                                    *anterior_hip_on_pelvis);
    anteriorHip->addFrame(anterior_hip_on_Hlink);
    anteriorHip->addFrame(anterior_hip_on_pelvis);
    
    // add anterior leg to model
    model.addBody(pelvisBracket); model.addJoint(anteriorHip);
    
    // since the previous, anterior knee joint drives the pose of the lower
    // 4-bar linkage, set the anterior hip angle such that it's free to satisfy
    // constraints that couple it to the 4-bar linkage.
    anteriorHip->updCoordinate().set_is_free_to_satisfy_constraints(true);
    
    // Close the loop for the lower, four-bar linkage with a constraint
    //------------------------------------------------------------------
    
    // Create and configure point on line constraint
    OpenSim::PointOnLineConstraint* posteriorHip =
        new OpenSim::PointOnLineConstraint();
    
    posteriorHip->connectSocket_line_body(*pelvisBracket);
    posteriorHip->setLineDirection(Vec3(0.0,0.0,1.0));
    posteriorHip->setPointOnLine(inferior_pelvis_pin_location);
    posteriorHip->connectSocket_follower_body(*posteriorLegBar);
    posteriorHip->setPointOnFollower(superior_bar_hinge_location);
    
    // add constraint to model
    model.addConstraint(posteriorHip);
    
    // Create chest piece
    //-----------------------
    OpenSim::Body* chest = new OpenSim::Body("chest_bar", bar_mass,
                                     Vec3(0.0),
                                     Inertia::brick(torso_bar_dimensions/2.0));
    
    chest->attachGeometry(new Mesh("Anterior_torso_bar.obj"));

    auto* anterior_torso_hinge_on_pelvis = new PhysicalOffsetFrame(
        "anterior_torso_hinge_on_pelvis",
        *pelvisBracket, Transform(anterior_superior_pelvis_pin_location) );

    auto* anterior_torso_hinge_on_chest = new PhysicalOffsetFrame(
        "anterior_torso_hinge_on_chest",
        *chest, Transform(inferior_torso_hinge_location) );
    
    // Attach chest piece to pelvice with pin joint
    OpenSim::PinJoint* anteriorTorsoHinge = new OpenSim::PinJoint(
                                              "anterior_torso_hinge",
                                              *anterior_torso_hinge_on_pelvis,
                                              *anterior_torso_hinge_on_chest);
    anteriorTorsoHinge->addFrame(anterior_torso_hinge_on_pelvis);
    anteriorTorsoHinge->addFrame(anterior_torso_hinge_on_chest);
    
    // add posterior leg to model
    model.addBody(chest); model.addJoint(anteriorTorsoHinge);
    
    // set torso rotation slightly anterior
    anteriorTorsoHinge->updCoordinate().setDefaultValue(-1*SimTK::Pi/4);
    
    // Create chest piece
    //-----------------------
    OpenSim::Body* back = new OpenSim::Body("back_bar", bar_mass,
                                     Vec3(0.0),
                                     Inertia::brick(torso_bar_dimensions/2.0));
    
    back->attachGeometry(new Mesh("Posterior_torso_bar.obj"));

    auto* posterior_torso_hinge_on_pelvis = new PhysicalOffsetFrame(
        "posterior_torso_hinge_on_pelvis",
        *pelvisBracket, Transform(posterior_superior_pelvis_pin_location) );

    auto* posterior_torso_hinge_on_back = new PhysicalOffsetFrame(
        "posterior_torso_hinge_on_back",
        *back, Transform(back_peg_center) );
    
    // Attach chest piece to pelvis with pin joint
    OpenSim::PinJoint* posteriorTorsoHinge = new OpenSim::PinJoint(
                                          "posterior_torso_hinge",
                                          *posterior_torso_hinge_on_pelvis,
                                          *posterior_torso_hinge_on_back);
    posteriorTorsoHinge->addFrame(posterior_torso_hinge_on_pelvis);
    posteriorTorsoHinge->addFrame(posterior_torso_hinge_on_back);
    
    // add posterior leg to model
    model.addBody(back); model.addJoint(posteriorTorsoHinge);
    
    // set posterior back joint to freely follow anterior joint through 4-bar
    // linkage coupling.
    posteriorTorsoHinge->updCoordinate()
                        .set_is_free_to_satisfy_constraints(true);
    
    // Create shoulder bracket
    //-----------------------
    OpenSim::Body* shoulderBracket = new OpenSim::Body("shoulder_bracket",
                                     bracket_mass,
                                     Vec3(0.0),
                                     Inertia::brick(shoulder_dimensions/2.0));

    shoulderBracket->attachGeometry(new Mesh("Shoulder_meters.obj"));
    // add anterior leg to model
    model.addBody(shoulderBracket);

    auto* anterior_thoracic_joint_on_chest = new PhysicalOffsetFrame(
        "anterior_thoracic_joint_on_chest",
        *chest, Transform(superior_torso_hinge_location) );

    auto* anterior_thoracic_joint_on_shoulder = new PhysicalOffsetFrame(
        "anterior_thoracic_joint_on_shoulder",
        *shoulderBracket, Transform(anterior_thoracic_joint_center));
    
    // Connect pelvis to Hlink via pin joint
    OpenSim::PinJoint* anteriorThoracicJoint =
                        new OpenSim::PinJoint("anterior_thoracic_joint",
                                       *anterior_thoracic_joint_on_chest,
                                       *anterior_thoracic_joint_on_shoulder);
    anteriorThoracicJoint->addFrame(anterior_thoracic_joint_on_chest);
    anteriorThoracicJoint->addFrame(anterior_thoracic_joint_on_shoulder);
    // add back joint
    model.addJoint(anteriorThoracicJoint);
    
    // since the previous, anterior thoracic joint drives the pose of the lower
    // 4-bar linkage, set the anterior shoulder angle such that it's free to
    // satisfy constraints that couple it to the 4-bar linkage.
    anteriorThoracicJoint->updCoordinate()
                          .set_is_free_to_satisfy_constraints(true);

    // Close the loop for the lower, four-bar linkage with a constraint
    //------------------------------------------------------------------
    // Create and configure point on line constraint
    OpenSim::PointOnLineConstraint* posteriorShoulder =
        new OpenSim::PointOnLineConstraint();
    
    posteriorShoulder->connectSocket_line_body(*shoulderBracket);
    posteriorShoulder->setLineDirection(Vec3(0.0,0.0,1.0));
    posteriorShoulder->setPointOnLine(posterior_thoracic_joint_center);
    posteriorShoulder->connectSocket_follower_body(*back);
    posteriorShoulder->setPointOnFollower(superior_torso_hinge_location);

    // add constraint to model
    model.addConstraint(posteriorShoulder);

    // Create and add luxo head
    OpenSim::Body* head = new OpenSim::Body("head", head_mass, Vec3(0),
            Inertia::cylinderAlongX(0.5*head_dimension[1], head_dimension[1]));

    head->attachGeometry(new Mesh("luxo_head_meters.obj"));
    head->attachGeometry(new Mesh("Bulb_meters.obj"));
    model.addBody(head);

    auto* cervical_joint_on_shoulder = new PhysicalOffsetFrame(
        "cervical_joint_on_shoulder",
        *shoulderBracket, Transform(superior_shoulder_hinge_location) );

    auto* cervical_joint_on_head = new PhysicalOffsetFrame(
        "cervical_joint_on_head",
        *head, Transform(cervicle_joint_center));

    // attach to shoulder via pin joint
    OpenSim::PinJoint* cervicalJoint = new OpenSim::PinJoint("cervical_joint",
                                  *cervical_joint_on_shoulder,
                                  *cervical_joint_on_head);

    cervicalJoint->addFrame(cervical_joint_on_shoulder);
    cervicalJoint->addFrame(cervical_joint_on_head);
    // add a neck joint
     model.addJoint(cervicalJoint);
    
    // lock the kneck coordinate so the head doens't spin without actuators or
    // passive forces
    cervicalJoint->updCoordinate().set_locked(true);

    // Coordinate Limit forces for restricting leg range of motion.
    //-----------------------------------------------------------------------
    CoordinateLimitForce* kneeLimitForce =
            new CoordinateLimitForce(
                     anterior_knee->getCoordinate().getName(),
                     knee_flexion_max, joint_softstop_stiffness,
                     knee_flexion_min, joint_softstop_stiffness,
                     joint_softstop_damping,
                     transition_region);
    model.addForce(kneeLimitForce);
    
    // Coordinate Limit forces for restricting back range motion.
    //-----------------------------------------------------------------------
    CoordinateLimitForce* backLimitForce =
    new CoordinateLimitForce(
                         anteriorTorsoHinge->getCoordinate().getName(),
                         back_extension_max, joint_softstop_stiffness,
                         back_extension_min, joint_softstop_stiffness,
                         joint_softstop_damping,
                         transition_region);
    model.addForce(backLimitForce);
    
    
    // Contact
    //-----------------------------------------------------------------------
    ContactHalfSpace* floor_surface = new ContactHalfSpace(SimTK::Vec3(0),
                                       SimTK::Vec3(0, 0, -0.5*SimTK::Pi),
                                       model.updGround(), "floor_surface");
    
    OpenSim::ContactMesh* foot_surface = new ContactMesh(
                                         "thin_disc_0.11_by_0.01_meters.obj",
                                          SimTK::Vec3(0),
                                          SimTK::Vec3(0),
                                          *base,
                                          "foot_surface");
    
    // add contact geometry to model
    model.addContactGeometry(floor_surface);
    model.addContactGeometry(foot_surface);
    
    // define contact as an elastic foundation force
    OpenSim::ElasticFoundationForce::ContactParameters* contactParameters =
            new OpenSim::ElasticFoundationForce::ContactParameters(
                                               stiffness,
                                               dissipation,
                                               friction,
                                               friction,
                                               viscosity);
    
    contactParameters->addGeometry("foot_surface");
    contactParameters->addGeometry("floor_surface");
    
    OpenSim::ElasticFoundationForce* contactForce =
                        new OpenSim::ElasticFoundationForce(contactParameters);
    contactForce->setName("contact_force");
    
    model.addForce(contactForce);
    
    
    
    // MUSCLES
    //-----------------------------------------------------------------------
    
    // add a knee extensor to control the lower 4-bar linkage
    Millard2012EquilibriumMuscle* kneeExtensorRight = new Millard2012EquilibriumMuscle(
                                            "knee_extensor_right",
                                            knee_extensor_F0, knee_extensor_lm0,
                                            knee_extensor_lts, pennationAngle);
    kneeExtensorRight->addNewPathPoint("knee_extensor_right_origin", *leg_Hlink,
                                  knee_extensor_origin);
    kneeExtensorRight->addNewPathPoint("knee_extensor_right_insertion",
                                      *bottom_bracket,
                                        knee_extensor_insertion);
    kneeExtensorRight->set_ignore_tendon_compliance(true);
    model.addForce(kneeExtensorRight);
    
    // create a left knee extensor
    Millard2012EquilibriumMuscle* kneeExtensorLeft =
            new Millard2012EquilibriumMuscle("knee_extensor_left",
                            knee_extensor_F0, knee_extensor_lm0,
                            knee_extensor_lts, pennationAngle);

    kneeExtensorLeft->addNewPathPoint("knee_extensor_left_origin", *leg_Hlink,
                                  knee_extensor_origin);
    kneeExtensorLeft->addNewPathPoint("knee_extensor_left_insertion",
                                      *bottom_bracket,
                                        knee_extensor_insertion);

    // flip the z coordinates of all path points
    PathPointSet& points = kneeExtensorLeft->updGeometryPath().updPathPointSet();
    for (int i=0; i<points.getSize(); ++i) {
        dynamic_cast<PathPoint&>(points[i]).upd_location()[2] *= -1;
    }

    kneeExtensorLeft->set_ignore_tendon_compliance(true);
    model.addForce(kneeExtensorLeft);
    
    // add a back extensor to control the upper 4-bar linkage
    Millard2012EquilibriumMuscle* backExtensorRight = new Millard2012EquilibriumMuscle(
                                            "back_extensor_right",
                                            back_extensor_F0, back_extensor_lm0,
                                            back_extensor_lts, pennationAngle);
    
    backExtensorRight->addNewPathPoint("back_extensor_right_origin", *chest,
                                      back_extensor_origin);
    backExtensorRight->addNewPathPoint("back_extensor_right_insertion", *back,
                                      back_extensor_insertion);
    backExtensorRight->set_ignore_tendon_compliance(true);
    model.addForce(backExtensorRight);
    
    // add another back extensor.
    Millard2012EquilibriumMuscle* backExtensorLeft =
            new Millard2012EquilibriumMuscle("back_extensor_left",
                                    back_extensor_F0, back_extensor_lm0,
                                    back_extensor_lts, pennationAngle);
    backExtensorLeft->addNewPathPoint("back_extensor_left_origin", *chest,
                                      back_extensor_origin);
    backExtensorLeft->addNewPathPoint("back_extensor_left_insertion", *back,
                                      back_extensor_insertion);

    PathPointSet& pointsLeft = backExtensorLeft->updGeometryPath()
        .updPathPointSet();
    for (int i=0; i<points.getSize(); ++i) {
        dynamic_cast<PathPoint&>(pointsLeft[i]).upd_location()[2] *= -1;
    }
    backExtensorLeft->set_ignore_tendon_compliance(true);
    model.addForce(backExtensorLeft);
    
    
    
    // MUSCLE CONTROLLERS
    //________________________________________________________________________
    
    // specify a piecwise linear function for the muscle excitations
    PiecewiseConstantFunction* x_of_t = new PiecewiseConstantFunction(3, times,
                                                                  excitations);
    
    
    PrescribedController* kneeController = new PrescribedController();
    kneeController->addActuator(*kneeExtensorLeft);
    kneeController->addActuator(*kneeExtensorRight);
    kneeController->prescribeControlForActuator(0, x_of_t);
    kneeController->prescribeControlForActuator(1, x_of_t->clone());
    
    model.addController(kneeController);
    
    PrescribedController* backController = new PrescribedController();
    backController->addActuator(*backExtensorLeft);
    backController->addActuator(*backExtensorRight);
    backController->prescribeControlForActuator(0, x_of_t->clone());
    backController->prescribeControlForActuator(1, x_of_t->clone());
    
    model.addController(backController);

    
    /* You'll find that these muscles can make Luxo Myo stand, but not jump.
     * Jumping will require an assistive device. We'll add two frames for
     * attaching a point to point assistive actuator.
     */
    
    
    // add frames for connecting a back assitance device between the chest
    // and pelvis
    PhysicalOffsetFrame* back_assist_origin_frame = new
        PhysicalOffsetFrame("back_assist_origin",
                            *chest,
                            back_assist_origin_transform);
    
    PhysicalOffsetFrame* back_assist_insertion_frame = new
        PhysicalOffsetFrame("back_assist_insertion",
                            *pelvisBracket,
                            back_assist_insertion_transform);
    
    model.addComponent(back_assist_origin_frame);
    model.addComponent(back_assist_insertion_frame);
    
    // add frames for connecting a knee assistance device between the posterior
    // leg and bottom bracket.
    PhysicalOffsetFrame* knee_assist_origin_frame = new
    PhysicalOffsetFrame("knee_assist_origin",
                        *posteriorLegBar,
                        knee_assist_origin_transform);
    
    PhysicalOffsetFrame* knee_assist_insertion_frame = new
    PhysicalOffsetFrame("knee_assist_insertion",
                        *bottom_bracket,
                        knee_assist_insertion_transform);
    
    model.addComponent(knee_assist_origin_frame);
    model.addComponent(knee_assist_insertion_frame);

    // Temporary: make the frame geometry disappear.
    for (auto& c : model.getComponentList<OpenSim::FrameGeometry>()) {
        const_cast<OpenSim::FrameGeometry*>(&c)->set_scale_factors(
                SimTK::Vec3(0.001, 0.001, 0.001));
    }
    
}
