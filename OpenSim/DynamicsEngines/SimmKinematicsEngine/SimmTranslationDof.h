#ifndef __SSimmTranslationDof_h__
#define __SimmTranslationDof_h__

// SimmTranslationDof.h
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
#include <string>
#include <math.h>
#include "osimSimmKinematicsEngineDLL.h"
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/AbstractDof.h>

namespace OpenSim {

#define TX_NAME "tx"
#define TY_NAME "ty"
#define TZ_NAME "tz"

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM translational DOF.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMMKINEMATICSENGINE_API SimmTranslationDof : public AbstractDof  
{
public:
	enum AxisIndex
	{
		xTranslation = 0,
		yTranslation,
		zTranslation
	};

//=============================================================================
// DATA
//=============================================================================
protected:
	SimTK::Vec3 _axis;
	AxisIndex _axisIndex;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmTranslationDof();
	SimmTranslationDof(const SimmTranslationDof &aDof);
	virtual ~SimmTranslationDof();
	virtual Object* copy() const;
	virtual void updateFromXMLNode();

#ifndef SWIG
	SimmTranslationDof& operator=(const SimmTranslationDof &aDof);
#endif
   void copyData(const SimmTranslationDof &aDof);

	virtual void setAxis(const SimTK::Vec3& rAxis);
	virtual void getAxis(SimTK::Vec3& rAxis) const;
	virtual const double* getAxisPtr() const { return &_axis[0]; }
	virtual double getValue();
	virtual DofType getMotionType() const { return Translational; }
	void getTranslation(double rVec[4]);
	AxisIndex getAxisIndex() const { return _axisIndex; }

protected:

private:
	void setNull();
//=============================================================================
};	// END of class SimmTranslationDof
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmTranslationDof_h__


