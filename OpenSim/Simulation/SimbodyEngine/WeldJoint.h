#ifndef __WeldJoint_h__
#define __WeldJoint_h__
//
// Author: Ajay Seth
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
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


// INCLUDE
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/ScaleSet.h>
#include "Joint.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing an Weld joint.  The underlying implementation 
 * in Simbody is a MobilizedBody::Weld.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API WeldJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(WeldJoint, Joint);

private:
	static const int _numMobilities = 0;
//=============================================================================
// DATA
//=============================================================================
protected:

	/** Weld has no properties*/


//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	WeldJoint();
	// Convenience constructor
	WeldJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
					OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, bool reverse=false);
	virtual ~WeldJoint();

	virtual int numCoordinates() const {return _numMobilities;};

	// SCALE
	virtual void scale(const ScaleSet& aScaleSet);

    // ModelComponent interface.
	void connectToModel(Model& aModel) OVERRIDE_11;
protected:
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

public:
	void createMobilizedBody(SimTK::Transform parentTransform, SimTK::Transform childTransform);

//=============================================================================
};	// END of class WeldJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WeldJoint_h__


