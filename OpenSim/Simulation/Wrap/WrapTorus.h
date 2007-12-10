#ifndef __WrapTorus_h__
#define __WrapTorus_h__

// WrapTorus.h
// Author: Peter Loan
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
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDbl.h>
#include "AbstractWrapObject.h"

namespace OpenSim {

class VisibleObject;
class AbstractBody;
class AbstractDynamicsEngine;
class MusclePoint;
class MuscleWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A class implementing a torus for muscle wrapping.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API WrapTorus : public AbstractWrapObject
{

	struct CircleCallback {
		double p1[3], p2[3], r;
	};

//=============================================================================
// DATA
//=============================================================================

	PropertyDbl _innerRadiusProp;
	double& _innerRadius;

	PropertyDbl _outerRadiusProp;
	double& _outerRadius;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	WrapTorus();
	WrapTorus(const WrapTorus& aWrapTorus);
	virtual ~WrapTorus();
	virtual Object* copy() const;
#ifndef SWIG
	WrapTorus& operator=(const WrapTorus& aWrapTorus);
#endif
   void copyData(const WrapTorus& aWrapTorus);
	virtual const char* getWrapTypeName() const;
	virtual std::string getDimensionsString() const;

	virtual void scale(Array<double>& aScaleFactors) { }
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	virtual int wrapLine(Array<double>& aPoint1, Array<double>& aPoint2,
		const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const;

protected:
	void setupProperties();

private:
	void setNull();
	int findClosestPoint(double radius, double p1[], double p2[],
		double* xc, double* yc, double* zc,
		int wrap_sign, int wrap_axis) const;
	static void calcCircleResids(int numResid, int numQs, double q[],
		double resid[], int *flag2, void *ptr);

//=============================================================================
};	// END of class WrapTorus
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapTorus_h__


