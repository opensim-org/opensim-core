#ifndef __FreeJoint_h__
#define __FreeJoint_h__
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
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/ScaleSet.h>
#include "Joint.h"
#include "Coordinate.h"
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a Free joint.  The underlying implementation 
 * in Simbody is a MobilizedBody::Free.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API FreeJoint : public Joint  
{

	static const int _numMobilities = 6;
//=============================================================================
// DATA
//=============================================================================
protected:

	/** Flag to use Euler angles to parameterize rotation of the body  */
	//PropertyBool _useEulerAnglesProp;
	//bool &_useEulerAngles;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	FreeJoint();
	// Convenience Constructor
	FreeJoint(const std::string &name, Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
		  Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, 
		  /*bool useEulerAngles=true,*/ bool reverse=false);
	
	/**
	 * Constructor that's more friendly to scripting with only primitive data
     * types or OpenSim types as arguments.
     */
	FreeJoint(const std::string &name, OpenSim::Body& parent, double locationInParent[], double orientationInParent[],
					OpenSim::Body& body, double locationInBody[], double orientationInBody[],
					/*bool useEulerAngles,*/ bool reverse) :
	Joint(name, parent, SimTK::Vec3(locationInParent),SimTK::Vec3(orientationInParent),
			body, SimTK::Vec3(locationInBody), SimTK::Vec3(orientationInBody), reverse)
{
	setNull();
	setupProperties();
	//_useEulerAngles = useEulerAngles;
	_body->setJoint(*this);
	setName(name);
}

	FreeJoint(const FreeJoint &aJoint);
	virtual ~FreeJoint();
	virtual Object* copy() const;

#ifndef SWIG
	FreeJoint& operator=(const FreeJoint &aJoint);
#endif

	void copyData(const FreeJoint &aJoint);
	virtual void setup(Model& aModel);

	virtual int numCoordinates() const {return _numMobilities;};

	// SCALE
	virtual void scale(const ScaleSet& aScaleSet);

	OPENSIM_DECLARE_DERIVED(FreeJoint, Joint);

protected:
    void createSystem(SimTK::MultibodySystem& system) const;
    void initState(SimTK::State& s) const;
    void setDefaultsFromState(const SimTK::State& state);

private:
	SimTK::MobilizedBodyIndex _masslessBodyIndex;
	void setNull();
	void setupProperties();


//=============================================================================
};	// END of class FreeJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __FreeJoint_h__


