#ifndef __AbstractConstraint_h__
#define __AbstractConstraint_h__

// AbstractConstraint.h
// Author: Ajay Seth
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include <iostream>
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Set.h>
#include <SimTKcommon.h>

namespace OpenSim {

class AbstractDynamicsEngine;
class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * A base class that specifies the interface for a Constraint.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractConstraint : public Object  
{
//=============================================================================
// DATA
//=============================================================================
protected:
	AbstractDynamicsEngine* _dynamicsEngine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractConstraint();
	AbstractConstraint(const AbstractConstraint &aConstraint);
	virtual ~AbstractConstraint();
	virtual Object* copy() const = 0;

#ifndef SWIG
	AbstractConstraint& operator=(const AbstractConstraint &aConstraint);
#endif
	
	void copyData(const AbstractConstraint &aConstraint);

	virtual AbstractDynamicsEngine* getDynamicsEngine() { return _dynamicsEngine; }
    virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual void updateFromConstraint(const AbstractConstraint &aConstraint) = 0;
	virtual bool getIsDisabled() const = 0;
	virtual bool setIsDisabled(bool isDisabled) = 0;
	virtual void calcConstraintForces(SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInParent, 
									  SimTK::Vector& mobilityForces) = 0;
	virtual void scale(const ScaleSet& aScaleSet) = 0;

	OPENSIM_DECLARE_DERIVED(AbstractConstraint, Object);

private:
	void setNull();

//=============================================================================
};	// END of class AbstractConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractConstraint_h__


