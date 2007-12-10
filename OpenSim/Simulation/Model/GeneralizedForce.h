#ifndef _GeneralizedForce_h_
#define _GeneralizedForce_h_
// GeneralizedForce.h
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDbl.h>
#include "AbstractActuator.h"


//=============================================================================
//=============================================================================
/**
 * A class that supports the application of a generalized force to a model.
 * This actuator has no states; the control is simply the force to
 * be applied to the model.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class AbstractCoordinate;
class ActuatorSet;
class AbstractSpeed;

class OSIMSIMULATION_API GeneralizedForce : public AbstractActuator
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Name of coordinate to which the generalized force is applied. */
	PropertyStr _propQName;
	/** Optimal force. */
	PropertyDbl _propOptimalForce;

	// REFERENCES
	std::string& _qName;
	double &_optimalForce;

	/** Corresponding generalized coordinate to which the generalized force
	is applied. */
	AbstractCoordinate *_q;

	/** Generalized speed to which the generalized force is applied.  For a
	generalized force to be applied, it must have both a coordinate and
	speed. */
	AbstractSpeed *_u;

	/** Temporary work array for holding generalized speeds. */
	double *_utmp;

	/** Excitation (control 0). */
	double _excitation;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	GeneralizedForce(std::string aCoordinateName="");
	GeneralizedForce(const GeneralizedForce &aGenForce);
	virtual ~GeneralizedForce();
	virtual Object* copy() const;
	void copyData(const GeneralizedForce &aGenForce);
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	GeneralizedForce& operator=(const GeneralizedForce &aGenForce);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// GENERALIZED COORDINATE
	void setQ(AbstractCoordinate* aQ);
	AbstractCoordinate* getQ() const;
	// OPTIMAL FORCE
	void setOptimalForce(double aOptimalForce);
	double getOptimalForce() const;
	// STRESS
	double getStress() const;

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void apply();

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeActuation();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	static ActuatorSet *CreateActuatorSetOfGeneralizedForcesForModel(Model *aModel,double aOptimalForce = 1,bool aIncludeLockedAndConstrainedCoordinates = true);

	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const;
	virtual bool isQValid() const;
	// Setup method to initialize Body references
	void setup(Model* aModel);

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();

	OPENSIM_DECLARE_DERIVED(GeneralizedForce, AbstractActuator);

//=============================================================================
};	// END of class GeneralizedForce

}; //namespace
//=============================================================================
//=============================================================================

#endif // __GeneralizedForce_h__


