#ifndef _JointMoment_h_
#define _JointMoment_h_
// JointMoment.h
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

/*  
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/PropertyDbl.h>
#include "GeneralizedForceAtv.h"


//=============================================================================
//=============================================================================
/**
 * This class is the same as GeneralizedForceAtv except that it adds a
 * new property (optinal_negative_force) that allows one to specify a
 * different flexion strength and extension strength.
 *
 * Controls: excitation
 * States: activation
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class OSIMACTUATORS_API JointMoment : public GeneralizedForceAtv
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Optimal negative force.  This optimal force parameter is used when
	the activation level is positive.  It is the mechanism used for having
	a different flexion and extension strenthgs */
	PropertyDbl _propOptimalNegForce;

	// REFERENCES
	double &_optimalNegForce;

//=============================================================================
// METHODS
//=============================================================================
public:
	JointMoment(std::string aQName="");
	JointMoment(const JointMoment &aActuator);
	virtual ~JointMoment();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	void copyData(const JointMoment &aActuator);

public:

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
	JointMoment&
		operator=(const JointMoment &aActuator);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// OPTIMAL NEGATIVE FORCE
	void setOptimalNegativeForce(double aOptNegForce);
	double getOptimalNegativeForce() const;

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeActuation();

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();

	OPENSIM_DECLARE_DERIVED(JointMoment, AbstractActuator);

//=============================================================================
};	// END of class JointMoment

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __JointMoment_h__
