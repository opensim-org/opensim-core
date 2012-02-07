#ifndef __BushingForce_h__
#define __BushingForce_h__

// BushingForce.h
// Author: Ajay Seth
/*
 * Copyright (c) 2010, Stanford University. All rights reserved. 
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
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include "Force.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a Bushing Force.
 * A Bushing Force is the force proportional to the deviation of two frames. 
 * One can think of the Bushing as being composed of 3 linear and 3 torsional
 * spring-dampers, which act along or about the bushing frames. 
 * The underlying Force in Simbody is a SimtK::Force::LinearBushing
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API BushingForce : public Force  
{

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	BushingForce();
	BushingForce( std::string body1Name, SimTK::Vec3 point1, SimTK::Vec3 orientation1,
		          std::string body2Name, SimTK::Vec3 point2, SimTK::Vec3 orientation2,
				  SimTK::Vec3 transStiffness, SimTK::Vec3 rotStiffness, SimTK::Vec3 transDamping, SimTK::Vec3 rotDamping );
	BushingForce(const BushingForce &aForce);
	virtual ~BushingForce();
	virtual Object* copy() const;
	BushingForce& operator=(const BushingForce &aForce);
	void copyData(const BushingForce &aForce);

	//SET 
	void setBody1ByName(std::string aBodyName);
	void setBody1BushingLocation(SimTK::Vec3 location, SimTK::Vec3 orientation=SimTK::Vec3(0));
	void setBody2ByName(std::string aBodyName);
	void setBody2BushingLocation(SimTK::Vec3 location, SimTK::Vec3 orientation=SimTK::Vec3(0));

	/** Potential energy is determine by the elastic energy storage of the bushing. */
	virtual double computePotentialEnergy(const SimTK::State& s) const;

	//-----------------------------------------------------------------------------
	// Reporting
	//-----------------------------------------------------------------------------
	/** 
	 * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const ;
	/**
	*  Provide the value(s) to be reported that correspond to the labels
	*/
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;

protected:
	virtual void setup(Model& aModel);
	/**
	 * Create a SimTK::Force::LinarBushing which implements this BushingForce.
	 */
	virtual void createSystem(SimTK::MultibodySystem& system) const;


private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class BushingForce
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __BushingForce_h__


