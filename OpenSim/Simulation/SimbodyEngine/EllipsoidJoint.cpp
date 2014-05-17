/* -------------------------------------------------------------------------- *
 *                        OpenSim:  EllipsoidJoint.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include "EllipsoidJoint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

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
EllipsoidJoint::~EllipsoidJoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
EllipsoidJoint::EllipsoidJoint() :
	Joint()
{
	constructCoordinates();
	constructProperties();
}
//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
EllipsoidJoint::EllipsoidJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
				OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
				SimTK::Vec3 ellipsoidRadii, bool reverse) :
	Joint(name, parent, locationInParent,orientationInParent,
			body, locationInBody, orientationInBody, reverse)
{
	constructCoordinates();
	constructProperties();

	set_radii_x_y_z(ellipsoidRadii);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void EllipsoidJoint::constructProperties()
{
	setAuthors("Ajay Seth");
	SimTK::Vec3 radii(NaN);
	constructProperty_radii_x_y_z(radii);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the EllipsoidJoint's radii. If the the system is created, will attempt
 * to update the the default radii of the underlying MobilizedBody::Ellipsoid
 *
 * @param Vec3 of radii: X, Y, Z in the parent frame.
 */
void EllipsoidJoint::setEllipsoidRadii(const Vec3& radii)
{
	set_radii_x_y_z(radii);
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale a joint based on XYZ scale factors for the bodies.
 *
 * @param aScaleSet Set of XYZ scale factors for the bodies.
 * @todo Need to scale transforms appropriately, given an arbitrary axis.
 */
void EllipsoidJoint::scale(const ScaleSet& aScaleSet)
{
	Vec3 scaleFactors(1.0);

	// Joint knows how to scale locations of the joint in parent and on the body
	Super::scale(aScaleSet);

	// SCALING TO DO WITH THE PARENT BODY -----
	// Joint kinematics are scaled by the scale factors for the
	// parent body, so get those body's factors
	const string& parentName = getParentBody().getName();
	for (int i=0; i<aScaleSet.getSize(); i++) {
		Scale& scale = aScaleSet.get(i);
		if (scale.getSegmentName()==parentName) {
			scale.getScaleFactors(scaleFactors);
			break;
		}
	}

	SimTK::Vec3& ellipsoidRadii = upd_radii_x_y_z();
	for(int i=0; i<3; i++){ 
		// Scale the size of the mobilizer
		ellipsoidRadii[i] *= scaleFactors[i];
	}
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void EllipsoidJoint::addToSystem(SimTK::MultibodySystem& system) const
{
	// CREATE MOBILIZED BODY
	MobilizedBody::Ellipsoid mobod =
		createMobilizedBody<MobilizedBody::Ellipsoid>(getParentTransform(),
		                                              getChildTransform());
	mobod.setDefaultRadii(get_radii_x_y_z());
    // TODO: Joints require super class to be called last.
    Super::addToSystem(system);
}

void EllipsoidJoint::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);

	const SimbodyMatterSubsystem& matter = getModel().getMatterSubsystem();
	
	if (!matter.getUseEulerAngles(s)){
		const CoordinateSet& coordinateSet = get_CoordinateSet();

		double xangle = coordinateSet[0].getDefaultValue();
		double yangle = coordinateSet[1].getDefaultValue();
		double zangle = coordinateSet[2].getDefaultValue();
		Rotation r(BodyRotationSequence, xangle, XAxis, 
			                             yangle, YAxis, zangle, ZAxis);

		EllipsoidJoint* mutableThis = const_cast<EllipsoidJoint*>(this);
		matter.getMobilizedBody(getChildBody().getIndex()).setQToFitRotation(s, r);
	}
}

void EllipsoidJoint::setPropertiesFromState(const SimTK::State& state)
{
	Super::setPropertiesFromState(state);

    // Override default in case of quaternions.
    const SimbodyMatterSubsystem& matter = getModel().getMatterSubsystem();
    if (!matter.getUseEulerAngles(state)) {
        Rotation r = matter.getMobilizedBody(getChildBody().getIndex())
			.getBodyRotation(state);
        Vec3 angles = r.convertRotationToBodyFixedXYZ();

		const CoordinateSet& coordinateSet = get_CoordinateSet();

        coordinateSet[0].setDefaultValue(angles[0]);
        coordinateSet[1].setDefaultValue(angles[1]);
        coordinateSet[2].setDefaultValue(angles[2]);
    }
}

void EllipsoidJoint::generateDecorations
       (bool                                        fixed, 
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   geometryArray) const
    {
        // invoke parent class method
        Super::generateDecorations(fixed,hints,state,geometryArray); 
        // the frame on body 1 will be red
        SimTK::Vec3 frame1color(1.0,0.0,0.0);
        // the frame on body 2 will be blue
        SimTK::Vec3 frame2color(0.0,0.5,1.0);
        // the moment on body 2 will be yellow
        SimTK::Vec3 moment2color(1.0,1.0,0.0);
        // the force on body 2 will be green
        SimTK::Vec3 force2color(0.0,1.0,0.0);

		double dimension = get_radii_x_y_z().norm()/2;
        // create frames to be fixed on body 1 and body 2
        SimTK::DecorativeFrame childFrame(dimension);
        SimTK::DecorativeFrame parentFrame(dimension);

        // attach frame to body, translate and rotate it to the location of the joint
		childFrame.setBodyId(getChildBody().getIndex());
		childFrame.setTransform(getChildTransform());
		childFrame.setColor(frame1color);

        // attach frame to parent, translate and rotate it to the location of the joint
        parentFrame.setBodyId( getParentBody().getIndex() );
		parentFrame.setTransform(getParentTransform());
        parentFrame.setColor(frame2color);

		// Construct the visible Ellipsoid
		SimTK::DecorativeEllipsoid ellipsoid(get_radii_x_y_z());
		ellipsoid.setTransform(getParentTransform());
		ellipsoid.setColor(Vec3(0.0, 1.0, 1.0));

		geometryArray.push_back(childFrame);
        geometryArray.push_back(parentFrame);
		geometryArray.push_back(ellipsoid);

        // if the model is moving, calculate and draw motion.
        if(!fixed){
		}

    }