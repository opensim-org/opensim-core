#ifndef _SpringGeneralizedForce_h_
#define _SpringGeneralizedForce_h_
// SpringGeneralizedForce.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "osimActuatorsDLL.h"
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/CustomForce.h>

//=============================================================================
//=============================================================================
/**
 * An force that exerts a generalized force based on spring-like
 * characteristics (stiffness and viscosity).  
 *
 * @author Frank C. Anderson, Ajay Seth
 * @version 2.0
 */
namespace OpenSim {

class Coordinate;

class OSIMACTUATORS_API SpringGeneralizedForce : public CustomForce 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Name of coordinate to which the spring gen force is applied. */
	PropertyStr _propCoordinateName;
	/** Stiffness. */
	PropertyDbl _propStiffness;
	/** Rest length. */
	PropertyDbl _propRestLength;
	/** Viscosity. */
	PropertyDbl _propViscosity;

	// REFERENCES
	std::string& _coordName;
	double &_stiffness;
	double &_restLength;
	double &_viscosity;

	/** Corresponding generalized coordinate to which the coordinate actuator
       is applied. */
    Coordinate *_coord;

//=============================================================================
// METHODS
//=============================================================================
public:
	SpringGeneralizedForce(std::string aCoordinateName="");
	SpringGeneralizedForce(const SpringGeneralizedForce &aForce);
	~SpringGeneralizedForce();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();

public:

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
	SpringGeneralizedForce&
		operator=(const SpringGeneralizedForce &aForce);

	// Setup method to initialize coordinate reference
	void setup(Model& aModel);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// GENERALIZED COORDINATE
	void setCoordinate(Coordinate* aCoordinate);
	Coordinate* getCoordinate() const;
	// STIFFNESS
	void setStiffness(double aStiffness);
	double getStiffness() const;
	// REST LENGTH
	void setRestLength(double aRestLength);
	double getRestLength() const;
	// VISCOSITY
	void setViscosity(double aViscosity);
	double getViscosity() const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
protected:
	virtual void computeForce( const SimTK::State& s) const;

	OPENSIM_DECLARE_DERIVED(SpringGeneralizedForce, CustomForce);

	//=============================================================================
};	// END of class SpringGeneralizedForce

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __SpringGeneralizedForce_h__
