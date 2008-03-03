// rdCMC_Point.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS,
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
// THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef rdCMCPoint_h__
#define rdCMCPoint_h__


//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include "rdCMC_Task.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for specifying and computing parameters for tracking a point.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API rdCMC_Point : public rdCMC_Task
{

//=============================================================================
// DATA
//=============================================================================
public:

protected:
	/** Body on which the tracked point resides. */
	AbstractBody *_body;
	/** Location of the tracked point on the body expressed in the body-local
	coordinate frame. */
	SimTK::Vec3 _point;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	rdCMC_Point(AbstractBody *aBody,SimTK::Vec3& aPoint);
	virtual ~rdCMC_Point();
	void setNull();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computePositionError(double time,SimTK::Vec3& posErr);
	virtual void computeVelocityError(double time,SimTK::Vec3& velErr);
	virtual void computeDesiredAccelerations(double time,SimTK::Vec3& acc);
	virtual void computeJacobian();
	virtual void computeEffectiveMassMatrix();


//=============================================================================
};	// END of class rdCMC_Point
//=============================================================================
//=============================================================================

}; // end namespace

#endif // rdCMCPoint_h__


