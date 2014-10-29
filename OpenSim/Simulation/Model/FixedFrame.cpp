/* -------------------------------------------------------------------------- *
 *                       OpenSim:  FixedFrame.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matt DeMers & Ayman Habib                                       *
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
#include "FixedFrame.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
//using namespace SimTK;
using namespace OpenSim;
using SimTK::Mat33;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
FixedFrame::FixedFrame() : RigidFrame()
{
	setNull();
	constructInfrastructure();
}

//_____________________________________________________________________________
/**
 * Constructors.
 */
FixedFrame::FixedFrame(const RigidFrame& parent_frame) : RigidFrame()
{
	setNull();
	constructInfrastructure();
	setParentFrame(parent_frame);
	
}

FixedFrame::FixedFrame(const RigidFrame& parent_frame, const SimTK::Transform&
        xform) : RigidFrame()
{
	setNull();
	constructInfrastructure();
	setParentFrame(parent_frame);
	setTransform(xform);

}
//_____________________________________________________________________________
/**
* Set a null frame as Identity rotation, 0 translation
*/
void FixedFrame::setNull()
{
	_isCacheInitialized = false;
	_transform.setToZero();
    _mbTransform.setToZero();
	setAuthors("Matt DeMers");
}
//_____________________________________________________________________________
/**
* construct properties
*/

void FixedFrame::constructProperties()
{
	SimTK::Vec3 zero(0.0, 0.0, 0.0);
	constructProperty_translation(zero);
	constructProperty_orientation(zero);
	// transform at default
}

void FixedFrame::constructConnectors()
{
	constructConnector<RigidFrame>("parent_frame");
}

void FixedFrame::doAddToSystem(SimTK::MultibodySystem& system) const
{
	// Traverse the tree of consecutive FixedFrame connections to determine 
	// this FixedFrame's root segment (Body/MobilizedBody).  We store that 
	// information instead of recomputing it on the fly when asked for information
	// about our connection to a root segment.

	initFixedFrameCache();

}
//=============================================================================
// FRAME COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
const SimTK::Transform& FixedFrame::getTransform() const
{
    // If properties have been updated, then update the cached transform object
    // to be in sync.
	_transform.updP() = get_translation();
	_transform.updR().setRotationToBodyFixedXYZ(get_orientation());
	return _transform;
}
void FixedFrame::setTransform(const SimTK::Transform& xform)
{
	_transform = xform;
    // Make sure properties are updated in case we either call gettters or
    // serialize after this call
	set_translation(xform.p());
	set_orientation(xform.R().convertRotationToBodyFixedXYZ());
    invalidate();
	
}

void FixedFrame::invalidate() const
{
    _isCacheInitialized = false;
    _mbTransform.setToNaN();
    if (!_model.empty())
    {
        _model->invalidateSystem();
    }    
}

bool FixedFrame::isPathToBaseValid() const
{
    //  check if I'm valid first
    if (_isCacheInitialized == 0) { return false; }

    // check if my parent is another FixedFrame
    const FixedFrame* parent = dynamic_cast<const FixedFrame*>(&getParentFrame());
    if (parent != 0)
    {
        // check if my parent FixedFrame is valid
        return parent->isPathToBaseValid();
    }
    // otherwise, I'm valid and my parent is a base segment
    return true;

}

void FixedFrame::initFixedFrameCache() const
{
	const RigidFrame& parent = getParentFrame();
	const OpenSim::FixedFrame* parentFixedFrame =
		+dynamic_cast<const OpenSim::FixedFrame *>(&parent);
	if (parentFixedFrame != 0)
	{
		// The parent frame is another FixedFrame
		// The parent FixedFrame must resolve its hierarchy before we ask
		// for its root MobilizedBodyIndex.
		// check if the FixedFrame has populated its root segments
		if (parentFixedFrame->isPathToBaseValid() == 0)
		{
			parentFixedFrame->initFixedFrameCache();
		}
		// all variables pointing to the parents root MobilizedBody should be valid
        _mbTransform = getTransform()*parentFixedFrame->getTransformInMobilizedBody();
	}
    else
    {
        // if I'm not on a FixedFrame, I'm on a Body or other root RigidFrame.  
        // Therefore, my root transform is the same as my local transform
        _mbTransform = getTransform();
    }

	// Ask my parent RigidFrame which root Body/MobilizedBody I'm attached to

	_index = parent.getMobilizedBodyIndex();
	
    _isCacheInitialized = true;
}
//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
void FixedFrame::setParentFrame(const RigidFrame& parent_frame) 
{ 
	updConnector<RigidFrame>("parent_frame").connect(parent_frame);
    invalidate();
}
const RigidFrame& FixedFrame::getParentFrame() const
{
    return getConnector<RigidFrame>("parent_frame").getConnectee();
}


//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________



//=============================================================================
// UTILITY
//=============================================================================



//=============================================================================
// I/O
//=============================================================================



