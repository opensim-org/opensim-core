/* -------------------------------------------------------------------------- *
 *                        OpenSim:  WeldConstraint.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan, Ajay Seth                        *
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
#include "WeldConstraint.h"
#include <simbody/internal/Constraint_Weld.h>
#include <simbody/internal/MobilizedBody.h>

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
WeldConstraint::~WeldConstraint()
{
}
//_____________________________________________________________________________
/*
 * Default constructor.
 */
WeldConstraint::WeldConstraint() :
    TwoFrameLinker<Constraint, PhysicalFrame>()
{
    setNull();
}

WeldConstraint::WeldConstraint( const std::string& name,
                                const std::string& frame1Name,
                                const std::string& frame2Name)
    : TwoFrameLinker<Constraint, PhysicalFrame>(name, frame1Name, frame2Name)
{
    setNull();
}

WeldConstraint::WeldConstraint(const std::string &name,
    const PhysicalFrame& frame1,
    const SimTK::Vec3& locationInFrame1, const SimTK::Vec3& orientationInFrame1,
    const PhysicalFrame& frame2,
    const SimTK::Vec3& locationInFrame2, const SimTK::Vec3& orientationInFrame2) :
    WeldConstraint(name, 
        frame1, Transform(Rotation(BodyRotationSequence,
            orientationInFrame1[0], XAxis,
            orientationInFrame1[1], YAxis,
            orientationInFrame1[2], ZAxis), locationInFrame1),
        frame2, Transform(Rotation(BodyRotationSequence,
            orientationInFrame2[0], XAxis,
            orientationInFrame2[1], YAxis,
            orientationInFrame2[2], ZAxis), locationInFrame2) )
{}

WeldConstraint::WeldConstraint(const std::string &name,
    const PhysicalFrame& frame1, const SimTK::Transform& transformInFrame1,
    const PhysicalFrame& frame2, const SimTK::Transform& transformInFrame2)
    : TwoFrameLinker<Constraint, PhysicalFrame>(name, frame1, transformInFrame1,
                                                      frame2, transformInFrame2)
{
    setNull();
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the data members of this WeldConstraint to their null values.
 */
void WeldConstraint::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/*
 * Construct WeldConstraint's properties
 */
void WeldConstraint::constructProperties()
{
    //Default frames list is empty
    constructProperty_frames();
}

void WeldConstraint::
    extendAddToSystemAfterSubcomponents(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystemAfterSubcomponents(system);

    // Get underlying mobilized bodies
    const PhysicalFrame& f1 = getFrame1();
    const PhysicalFrame& f2 = getFrame2();

    SimTK::MobilizedBody b1 = f1.getMobilizedBody();
    SimTK::MobilizedBody b2 = f2.getMobilizedBody();
    // Build the transforms
    SimTK::Transform inb1 = f1.findTransformInBaseFrame();
    SimTK::Transform inb2 = f2.findTransformInBaseFrame();

    // Now create a Simbody Constraint::Weld
    SimTK::Constraint::Weld simtkWeld(b1, inb1, b2, inb2);
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
    assignConstraintIndex(simtkWeld.getConstraintIndex());
}

void WeldConstraint::
    setContactPointForInducedAccelerations(const SimTK::State &s, Vec3 point)
{
    // make sure we are at the position stage
    getSystem().realize(s, SimTK::Stage::Position);

    const PhysicalFrame& frame1 = getFrame1();
    const PhysicalFrame& frame2 = getFrame2();
    
    // For external forces we assume point position vector is defined 
    // wrt foot (i.e., frame2) because we are passing it in from a
    // prescribed force.
    // We must also get that point position vector wrt ground (i.e., frame1)
    Vec3 spoint = frame2.findStationLocationInAnotherFrame(s, point, frame1);

    SimTK::Transform in1(frame1.getTransformInGround(s).R(), spoint);
    SimTK::Transform in2(frame2.getTransformInGround(s).R(), point);

    // Add new internal PhysicalOffSetFrames that the Constraint can update 
    // without affecting any other components.
    // ONLY add them if they don't exist already
    if (!_internalOffset1.get()) {
        _internalOffset1.reset(new PhysicalOffsetFrame(frame1, in1));
        _internalOffset1->setName("internal_" + frame1.getName());
        updProperty_frames().adoptAndAppendValue(_internalOffset1.get());
        connectSocket_frame1(*_internalOffset1);
    }
    else { // otherwise it is already "wired" up so just update
        _internalOffset1->setOffsetTransform(in1);
    }

    if (!_internalOffset2.get()) {
        _internalOffset2.reset(new PhysicalOffsetFrame(frame2, in2));
        _internalOffset2->setName("internal_" + frame2.getName());
        updProperty_frames().adoptAndAppendValue(_internalOffset2.get());
        connectSocket_frame2(*_internalOffset2);
    }
    else {
        _internalOffset2->setOffsetTransform(in2);
    }
}
