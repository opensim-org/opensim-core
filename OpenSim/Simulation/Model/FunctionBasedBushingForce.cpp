/* -------------------------------------------------------------------------- *
 *                  OpenSim:  FunctionBasedBushingForce.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matt S. DeMers                                                  *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/LinearFunction.h>
#include "FunctionBasedBushingForce.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, and copy
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
FunctionBasedBushingForce::FunctionBasedBushingForce()
{
    setNull();
    constructProperties();
}

FunctionBasedBushingForce::FunctionBasedBushingForce(
        const std::string& name,
        const PhysicalFrame& frame1,
        const SimTK::Vec3& point1,
        const SimTK::Vec3& orientation1,
        const PhysicalFrame& frame2,
        const SimTK::Vec3& point2,
        const SimTK::Vec3& orientation2)
    : Super(name, frame1, point1, orientation1, frame2, point2, orientation2)
{
    setNull();
    constructProperties();
}

// Convenience constructor for zero value force functions.
FunctionBasedBushingForce::FunctionBasedBushingForce(const string&   name,
                                    const string&   frame1Name,
                                    const Vec3&     point1,
                                    const Vec3&     orientation1,
                                    const string&   frame2Name,
                                    const Vec3&     point2,
                                    const Vec3&     orientation2)
    : Super(name,
            frame1Name, point1, orientation1,
            frame2Name, point2, orientation2)
{
    setNull();
    constructProperties();
}

FunctionBasedBushingForce::FunctionBasedBushingForce(
        const std::string& name,
        const PhysicalFrame& frame1,
        const SimTK::Vec3& point1,
        const SimTK::Vec3& orientation1,
        const PhysicalFrame& frame2,
        const SimTK::Vec3& point2,
        const SimTK::Vec3& orientation2,
        const SimTK::Vec3& transStiffness,
        const SimTK::Vec3& rotStiffness,
        const SimTK::Vec3& transDamping,
        const SimTK::Vec3& rotDamping)
    : FunctionBasedBushingForce(name,
                                frame1, point1, orientation1,
                                frame2, point2, orientation2)
{
    // populate m_ii and f_ii as LinearFunction based on stiffness
    set_m_x_theta_x_function( LinearFunction( rotStiffness[0], 0.0) );
    set_m_y_theta_y_function( LinearFunction( rotStiffness[1], 0.0) );
    set_m_z_theta_z_function( LinearFunction( rotStiffness[2], 0.0) );
    set_f_x_delta_x_function( LinearFunction( transStiffness[0], 0.0) );
    set_f_y_delta_y_function( LinearFunction( transStiffness[1], 0.0) );
    set_f_z_delta_z_function( LinearFunction( transStiffness[2], 0.0) );
    set_rotational_damping(rotDamping);
    set_translational_damping(transDamping);
}

// Convenience constructor for linear functions.
FunctionBasedBushingForce::FunctionBasedBushingForce(
                                    const std::string& name,
                                    const string&    frame1Name,
                                    const Vec3&      point1,
                                    const Vec3&      orientation1,
                                    const string&    frame2Name,
                                    const Vec3&      point2,
                                    const Vec3&      orientation2,
                                    const Vec3&      transStiffness,
                                    const Vec3&      rotStiffness,
                                    const Vec3&      transDamping,
                                    const Vec3&      rotDamping)
    : FunctionBasedBushingForce(name,
                                frame1Name, point1, orientation1,
                                frame2Name, point2, orientation2)
{
    // populate m_ii and f_ii as LinearFunction based on stiffness
    set_m_x_theta_x_function( LinearFunction( rotStiffness[0], 0.0) );
    set_m_y_theta_y_function( LinearFunction( rotStiffness[1], 0.0) );
    set_m_z_theta_z_function( LinearFunction( rotStiffness[2], 0.0) );
    set_f_x_delta_x_function( LinearFunction( transStiffness[0], 0.0) );
    set_f_y_delta_y_function( LinearFunction( transStiffness[1], 0.0) );
    set_f_z_delta_z_function( LinearFunction( transStiffness[2], 0.0) );
    set_rotational_damping(rotDamping);
    set_translational_damping(transDamping);
}

//_____________________________________________________________________________
// Set the data members of this FunctionBasedBushingForce to their null values.
void FunctionBasedBushingForce::setNull()
{
    setAuthors("Matt DeMers");
}

//_____________________________________________________________________________
// Allocate and initialize properties.
void FunctionBasedBushingForce::constructProperties()
{
    //set all deflection functions to zero
    Constant zeroFunc = Constant(0.0);
    constructProperty_m_x_theta_x_function( zeroFunc );
    constructProperty_m_y_theta_y_function( zeroFunc );
    constructProperty_m_z_theta_z_function( zeroFunc );
    constructProperty_f_x_delta_x_function( zeroFunc );
    constructProperty_f_y_delta_y_function( zeroFunc );
    constructProperty_f_z_delta_z_function( zeroFunc );
    
    constructProperty_rotational_damping(Vec3(0));
    constructProperty_translational_damping(Vec3(0));
    
    constructProperty_moment_visual_scale( 1.0 );
    constructProperty_force_visual_scale( 1.0 );
    constructProperty_visual_aspect_ratio(1.0);
}

void FunctionBasedBushingForce::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    for (int i = 0; i < 3; i++) {
        _dampingMatrix[i][i] = get_rotational_damping(0)[i];
        _dampingMatrix[i + 3][i + 3] = get_translational_damping(0)[i];
    }
}
//=============================================================================
// COMPUTATION
//=============================================================================
/* Calculate the bushing force contribution due to its stiffness. */
SimTK::Vec6 FunctionBasedBushingForce::
    calcStiffnessForce(const SimTK::State& s) const
{
    // Calculate stiffness generalized forces of bushing by first computing
    // the deviation of the two frames measured by dq
    Vec6 dq = computeDeflection(s);

    Vec6 fk = Vec6(0.0);
    fk[0] = get_m_x_theta_x_function().calcValue(SimTK::Vector(1, dq[0]));
    fk[1] = get_m_y_theta_y_function().calcValue(SimTK::Vector(1, dq[1]));
    fk[2] = get_m_z_theta_z_function().calcValue(SimTK::Vector(1, dq[2]));
    fk[3] = get_f_x_delta_x_function().calcValue(SimTK::Vector(1, dq[3]));
    fk[4] = get_f_y_delta_y_function().calcValue(SimTK::Vector(1, dq[4]));
    fk[5] = get_f_z_delta_z_function().calcValue(SimTK::Vector(1, dq[5]));

    return -fk;
}

/* Calculate the bushing force contribution due to its damping. */
SimTK::Vec6 FunctionBasedBushingForce::
    calcDampingForce(const SimTK::State& s) const
{
    Vec6 dqdot = computeDeflectionRate(s);
    // _dampingMatrix is initialized from Properties in extendFinalizeFromProperties (issue #2512)
    return -_dampingMatrix * dqdot;
}

/* Compute the force contribution to the system and add in to appropriate
 * bodyForce and/or system generalizedForce. */
void FunctionBasedBushingForce::computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    // stiffness force
    Vec6 fk = calcStiffnessForce(s);
    // damping force
    Vec6 fv = calcDampingForce(s);

    // total bushing force in the internal basis of the deflection (dq) 
    Vec6 f = fk + fv;

    // convert internal forces to spatial and add then add to system
    // physical (body) forces
    addInPhysicalForcesFromInternal(s, f, bodyForces);
}

//=============================================================================
// Reporting
//=============================================================================
/* Names of the quantities (column labels) of the force value(s) reported
 */
OpenSim::Array<std::string> FunctionBasedBushingForce::getRecordLabels() const 
{
    OpenSim::Array<std::string> labels("");

    // Forces applied to the two frames of the bushing
    const std::string& frame1Name = getFrame1().getName();
    const std::string& frame2Name = getFrame2().getName();

    labels.append(getName() + "." + frame1Name + ".force.X");
    labels.append(getName() + "." + frame1Name + ".force.Y");
    labels.append(getName() + "." + frame1Name + ".force.Z");
    labels.append(getName() + "." + frame1Name + ".torque.X");
    labels.append(getName() + "." + frame1Name + ".torque.Y");
    labels.append(getName() + "." + frame1Name + ".torque.Z");
    labels.append(getName() + "." + frame2Name + ".force.X");
    labels.append(getName() + "." + frame2Name + ".force.Y");
    labels.append(getName() + "." + frame2Name + ".force.Z");
    labels.append(getName() + "." + frame2Name + ".torque.X");
    labels.append(getName() + "." + frame2Name + ".torque.Y");
    labels.append(getName() + "." + frame2Name + ".torque.Z");

    return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> FunctionBasedBushingForce::
    getRecordValues(const SimTK::State& s) const 
{
    SpatialVec F_GM( Vec3(0.0),Vec3(0.0) );
    SpatialVec F_GF( Vec3(0.0),Vec3(0.0) );

    // total bushing force in the internal basis of the deflection (dq) 
    Vec6 f = calcStiffnessForce(s) + calcDampingForce(s);
    
    convertInternalForceToForcesOnFrames(s, f, F_GF, F_GM);

    OpenSim::Array<double> values(1);

    values.append(3, &F_GF[1][0]);
    values.append(3, &F_GF[0][0]);
    values.append(3, &F_GM[1][0]);
    values.append(3, &F_GM[0][0]);

    return values;
}

//_____________________________________________________________________________
/**
 * Implement generateDecorations to make visuals to represent the spring.
 */

void FunctionBasedBushingForce::generateDecorations(
        bool                                        fixed, 
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         s,
        SimTK::Array_<SimTK::DecorativeGeometry>&   geometryArray) const
    {
        // invoke parent class method
        Super::generateDecorations(fixed, hints , s, geometryArray); 
        // draw frame1 as red
        SimTK::Vec3 frame1color(1.0, 0.0, 0.0);
        // draw frame2 as blue
        SimTK::Vec3 frame2color(0.0, 0.5, 1.0);
        // the moment on frame2 will be yellow
        SimTK::Vec3 moment2color(1.0, 1.0, 0.0);
        // the force on frame2 will be green
        SimTK::Vec3 force2color(0.0, 1.0, 0.0);

        // create decorative frames to be fixed on frame1 and frame2
        SimTK::DecorativeFrame decorativeFrame1(0.2);
        SimTK::DecorativeFrame decorativeFrame2(0.2);

        // get connected frames
        const PhysicalFrame& frame1 = getFrame1();
        const PhysicalFrame& frame2 = getFrame2();

        // attach frame 1 to body 1, translate and rotate it to the location of the bushing
        decorativeFrame1.setBodyId(frame1.getMobilizedBodyIndex());
        decorativeFrame1.setTransform(frame1.findTransformInBaseFrame());
        decorativeFrame1.setColor(frame1color);

        // attach frame 2 to body 2, translate and rotate it to the location of the bushing
        decorativeFrame2.setBodyId(frame2.getMobilizedBodyIndex());
        decorativeFrame2.setTransform(frame2.findTransformInBaseFrame());
        decorativeFrame2.setColor(frame2color);

        geometryArray.push_back(decorativeFrame1);
        geometryArray.push_back(decorativeFrame2);

        // if the model is moving and the state is adequately realized,
        // calculate and draw the bushing forces.
        if (!fixed && (s.getSystemStage() >= Stage::Dynamics)) {
            SpatialVec F_GM(Vec3(0.0), Vec3(0.0));
            SpatialVec F_GF(Vec3(0.0), Vec3(0.0));

            // total bushing force in the internal basis of the deflection (dq) 
            Vec6 f = calcStiffnessForce(s) + calcDampingForce(s);

            convertInternalForceToForcesOnFrames(s, f, F_GF, F_GM);

            // location of the bushing on frame2
            //SimTK::Vec3 p_b2M_b2 = frame2.findTransformInBaseFrame().p();
            SimTK::Vec3 p_GM_G = frame2.getTransformInGround(s).p();

            // Add moment on frame2 as line vector starting at bushing location
            SimTK::Vec3 scaled_M_GM(get_moment_visual_scale()*F_GM[0]);
            SimTK::Real m_length(scaled_M_GM.norm());
            SimTK::Real m_radius(m_length / get_visual_aspect_ratio() / 2.0);
            SimTK::Transform   X_m2cylinder(SimTK::Rotation(SimTK::UnitVec3(scaled_M_GM), SimTK::YAxis), p_GM_G + scaled_M_GM / 2.0);
            SimTK::DecorativeCylinder frame2Moment(m_radius, m_length / 2.0);
            frame2Moment.setTransform(X_m2cylinder);
            frame2Moment.setColor(moment2color);
            geometryArray.push_back(frame2Moment);

            // Add force on frame2 as line vector starting at bushing location
            SimTK::Vec3 scaled_F_GM(get_force_visual_scale()*F_GM[1]);
            SimTK::Real f_length(scaled_F_GM.norm());
            SimTK::Real f_radius(f_length / get_visual_aspect_ratio() / 2.0);
            SimTK::Transform   X_f2cylinder(SimTK::Rotation(SimTK::UnitVec3(scaled_F_GM), SimTK::YAxis), p_GM_G + scaled_F_GM / 2.0);
            SimTK::DecorativeCylinder frame2Force(f_radius, f_length / 2.0);
            frame2Force.setTransform(X_f2cylinder);
            frame2Force.setColor(force2color);

            geometryArray.push_back(frame2Force);
        }
    }
