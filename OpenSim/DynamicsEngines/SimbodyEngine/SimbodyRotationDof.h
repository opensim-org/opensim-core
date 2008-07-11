#ifndef __SimbodyRotationDof_h__
#define __SimbodyRotationDof_h__

// SimbodyRotationDof.h
// Author: Peter Loan, Frank C. Anderson
/*
 * Copyright (c)  2006-2007, Stanford University. All rights reserved. 
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
#include <math.h>
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/AbstractDof.h>
#include <SimTKsimbody.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class expressing a rotational DOF.
 *
 * @author Peter Loan, Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API SimbodyRotationDof : public AbstractDof  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDblVec3 _axisProp;
	SimTK::Vec3 &_axis;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimbodyRotationDof();
	SimbodyRotationDof(const SimbodyRotationDof &aDof);
	virtual ~SimbodyRotationDof();
	virtual Object* copy() const;

#ifndef SWIG
	SimbodyRotationDof& operator=(const SimbodyRotationDof &aDof);
#endif
   void copyData(const SimbodyRotationDof &aDof);

	virtual void setAxis(const SimTK::Vec3& aAxis);
	virtual void getAxis(SimTK::Vec3& rAxis) const;
	const SimTK::Vec3& getAxis() const { return _axis; }
	virtual void getAxis(double rAxis[]) const { rAxis[0]=_axis[0]; rAxis[1]=_axis[1]; rAxis[2]=_axis[2]; }
	virtual const double* getAxisPtr() const { return &_axis[0]; }
	virtual double getValue();
	virtual DofType getMotionType() const { return Rotational; }

	virtual void peteTest();

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimbodyRotationDof
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodyRotationDof_h__


