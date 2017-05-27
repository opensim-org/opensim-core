/* -------------------------------------------------------------------------- *
 *                     OpenSim:  CoupledBushingForce.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include "CoupledBushingForce.h"
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "simbody/internal/Force_Custom.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
CoupledBushingForce::~CoupledBushingForce()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
CoupledBushingForce::CoupledBushingForce() : TwoFrameLinker<Force, PhysicalFrame>()
{
    setAuthors("Ajay Seth");
    constructProperties();
}

/* Convenience constructor */
CoupledBushingForce::CoupledBushingForce( const std::string& name,
                                          const std::string& frame1Name,
                                          const std::string& frame2Name,
                                          SimTK::Mat66 stiffnessMat,
                                          SimTK::Mat66 dampingMat)
    : TwoFrameLinker<Force, PhysicalFrame>(name, frame1Name, frame2Name)
{
    setAuthors("Ajay Seth");
    constructProperties();

    _stiffnessMatrix = stiffnessMat;
    _dampingMatrix = dampingMat;
    updatePropertiesFromMatrices();
}

//_____________________________________________________________________________
/*
 * Create properties 
 */
void CoupledBushingForce::constructProperties()
{
    // default bushing material properties
    // 6x6 stiffness matrix
    constructProperty_stiffness_row1(Vec6(0));
    constructProperty_stiffness_row2(Vec6(0));
    constructProperty_stiffness_row3(Vec6(0));
    constructProperty_stiffness_row4(Vec6(0));
    constructProperty_stiffness_row5(Vec6(0));
    constructProperty_stiffness_row6(Vec6(0));
    // 6x6 damping matrix
    constructProperty_damping_row1(Vec6(0));
    constructProperty_damping_row2(Vec6(0));
    constructProperty_damping_row3(Vec6(0));
    constructProperty_damping_row4(Vec6(0));
    constructProperty_damping_row5(Vec6(0));
    constructProperty_damping_row6(Vec6(0));
}

void CoupledBushingForce::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
    finalizeMatricesFromProperties();
}

//=============================================================================
// COMPUTATION
//=============================================================================
/* Compute the force contribution to the system and add in to appropriate
 * bodyForce and/or system generalizedForce.
 * CoupledBushingForce implementation based SimTK::Force::LinearBushing
 * developed and implemented by Michael Sherman. */
void CoupledBushingForce::computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    // Calculate stiffness generalized forces of bushing by first computing
    // the deviation of the two frames measured by dq
    Vec6 fk = -_stiffnessMatrix*computeDeflection(s);

    // velocity dependent force according to the speed of frame2
    // relative to frame1 
    Vec6 fv = -_dampingMatrix * computeDeflectionRate(s);

    // total bushing force in the internal basis of the deflection (dq) 
    Vec6 f = fk + fv; 

    // convert internal forces to spatial and add then add to system
    // physical (body) forces
    addInPhysicalForcesFromInternal(s, f, bodyForces);
}

/** Potential energy stored in the bushing is purely a function of the deflection
    from the rest position and the stiffness */
double CoupledBushingForce::computePotentialEnergy(const SimTK::State& s) const
{
    Vec6 dq = computeDeflection(s);
    // Energy stored in the linear bushing
    double U = 0.5*(~dq*_stiffnessMatrix*dq);
    return U;
}


//=============================================================================
// Reporting
//=============================================================================
/*
 * Provide names of the quantities (column labels) of the force value(s) to be
 * reported
 */
OpenSim::Array<std::string> CoupledBushingForce::getRecordLabels() const 
{
    OpenSim::Array<std::string> labels("");

    // get connected frames
    const PhysicalFrame& frame1 = getConnectee<PhysicalFrame>("frame1");
    const PhysicalFrame& frame2 = getConnectee<PhysicalFrame>("frame2");

    // Forces applied to underlying MobilizedBody which is a base PhysicalFrame
    std::string base1Name = frame1.findBaseFrame().getName();
    std::string base2Name = frame2.findBaseFrame().getName();

    labels.append(getName() + "." + base1Name + ".force.X");
    labels.append(getName() + "." + base1Name + ".force.Y");
    labels.append(getName() + "." + base1Name + ".force.Z");
    labels.append(getName() + "." + base1Name + ".torque.X");
    labels.append(getName() + "." + base1Name + ".torque.Y");
    labels.append(getName() + "." + base1Name + ".torque.Z");
    labels.append(getName() + "." + base2Name + ".force.X");
    labels.append(getName() + "." + base2Name + ".force.Y");
    labels.append(getName() + "." + base2Name + ".force.Z");
    labels.append(getName() + "." + base2Name + ".torque.X");
    labels.append(getName() + "." + base2Name + ".torque.Y");
    labels.append(getName() + "." + base2Name + ".torque.Z");

    return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> CoupledBushingForce::
getRecordValues(const SimTK::State& state) const 
{
    OpenSim::Array<double> values(1);

        // get connected frames
    const PhysicalFrame& frame1 = getConnectee<PhysicalFrame>("frame1");
    const PhysicalFrame& frame2 = getConnectee<PhysicalFrame>("frame2");

    const SimTK::Force::Custom &simtkSpring =
        (SimTK::Force::Custom &)(_model->getForceSubsystem().getForce(_index));

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    SimTK::Vector_<SimTK::Vec3> particleForces(0);
    SimTK::Vector mobilityForces(0);

    //get the net force added to the system contributed by the bushing
    simtkSpring.calcForceContribution(state, bodyForces, particleForces, mobilityForces);
    SimTK::Vec3 forces = bodyForces[frame1.getMobilizedBodyIndex()][1];
    SimTK::Vec3 torques = bodyForces[frame1.getMobilizedBodyIndex()][0];
    values.append(3, &forces[0]);
    values.append(3, &torques[0]);

    forces = bodyForces[frame2.getMobilizedBodyIndex()][1];
    torques = bodyForces[frame2.getMobilizedBodyIndex()][0];

    values.append(3, &forces[0]);
    values.append(3, &torques[0]);

    return values;
}

/* UTILITIES */
void CoupledBushingForce::finalizeMatricesFromProperties()
{
    _stiffnessMatrix = Mat66( ~get_stiffness_row1(),
                              ~get_stiffness_row2(),
                              ~get_stiffness_row3(),
                              ~get_stiffness_row4(),
                              ~get_stiffness_row5(),
                              ~get_stiffness_row6() );

    _dampingMatrix = Mat66( ~get_damping_row1(),
                            ~get_damping_row2(),
                            ~get_damping_row3(),
                            ~get_damping_row4(),
                            ~get_damping_row5(),
                            ~get_damping_row6() );
}

void CoupledBushingForce::updatePropertiesFromMatrices()
{
    upd_stiffness_row1() = _stiffnessMatrix(0);
    upd_stiffness_row2() = _stiffnessMatrix(1);
    upd_stiffness_row3() = _stiffnessMatrix(2);
    upd_stiffness_row4() = _stiffnessMatrix(3);
    upd_stiffness_row5() = _stiffnessMatrix(4);
    upd_stiffness_row6() = _stiffnessMatrix(5);

    upd_damping_row1() = _dampingMatrix(0);
    upd_damping_row2() = _dampingMatrix(1);
    upd_damping_row3() = _dampingMatrix(2);
    upd_damping_row4() = _dampingMatrix(3);
    upd_damping_row5() = _dampingMatrix(4);
    upd_damping_row6() = _dampingMatrix(5);
}
