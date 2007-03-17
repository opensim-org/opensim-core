#ifndef _Body_h_
#define _Body_h_
// Body.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

#include <OpenSim/Tools/Object.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Simulation/SIMM/PolyObject.h>

#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * A class which represents the physical parameters of a massive body.
 */
namespace OpenSim { 

class RDSIMULATION_API Body  : public PolyObject
{

//=============================================================================
// DATA
//=============================================================================
private:
	// PROPERTIES
	/** Mass properties */
	PropertyDbl			_propM;
	/** Moments of inertia */
	PropertyDblArray		_propI;
	/** Center of Mass of body relative to geometry's coordinate system */
	PropertyDblArray		_propCenterOfMass;

	// REFERENCES
	double&					_M;
	Array<double>&		_I;
	Array<double>&		_centerOfMass;
//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Body(double aM=1.0,double *aI=NULL);
	Body(const Body &aBody);
	Body(const std::string &aFileName);
	virtual ~Body();
	virtual Object* copy() const;
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Body& operator=(const Body &aBody);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setMass(double aM);
	double getMass();
	void setInertia(double *aI);
	void setInertia(double I1,double I2,double I3);
	void setInertia(double I1,double I2,double I3,
						double I12,double I23,double I31);
	void getInertia(double aI[6]);
	void getInertia(double aI[3][3]);
	void setCenterOfMass(const double aCenterOfMass[3]);
	void getCenterOfMass(double aCenterOfMass[3]) const;

	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
	void scaleBy(const double aScaleFactors[3]);
	//--------------------------------------------------------------------------
	// XML Support
	//--------------------------------------------------------------------------
	void setNull();
	void setupProperties();

};

}; //namespace

#endif //__Body_h__
