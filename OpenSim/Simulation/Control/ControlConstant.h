#ifndef _ControlConstant_h_
#define _ControlConstant_h_
// ControlConstant.h:
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


// INCLUDES
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Object.h>
#include "Control.h"


//=============================================================================
//=============================================================================
/**
 * A class that represents a constant control curve.  That is, the value
 * of the control curve is the same at any value of time.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class RDSIMULATION_API ControlConstant : public Control
{

//=============================================================================
// MEMBER DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Control value. */
	PropertyDbl _propX;

	// REFERENCES
	double &_x;

//=============================================================================
// METHODS
//=============================================================================
public:
	ControlConstant(double aX=0.0,const char *aName="UNKOWN");
	ControlConstant(const ControlConstant &aControl);
	virtual ~ControlConstant();
	virtual Object* copy() const;
private:
	void setNull();
	void copyData(const ControlConstant &aControl);
protected:
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	ControlConstant& operator=(const ControlConstant &aControl);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// PARAMETERS
	// Number
	virtual int getNumParameters() const;
	// Min
	virtual void setParameterMin(int aI,double aMin);
	virtual double getParameterMin(int aI) const;
	// Max
	virtual void setParameterMax(int aI,double aMax);
	virtual double getParameterMax(int aI) const;
	// Time and Neighborhood
	virtual double getParameterTime(int aI) const;
	virtual void getParameterNeighborhood(int aI,double &rTLower,double &rTUpper) const;
	// List
	virtual int getParameterList(double aT,Array<int> &rList);
	virtual int getParameterList(double aT1,double aT2,Array<int> &rList);
	// Value
	virtual void setParameterValue(int aI,double aP);
	virtual double getParameterValue(int aI) const;
	// CONTROL VALUE
	virtual void setControlValue(double aT,double aX);
	virtual double getControlValue(double aT);
	virtual double getControlValueMin(double aT=0.0);
	virtual void setControlValueMin(double aT,double aX);
	virtual double getControlValueMax(double aT=0.0);
	virtual void setControlValueMax(double aT,double aX);

//=============================================================================
};	// END of class ControlConstant

}; //namespace
//=============================================================================
//=============================================================================

#endif // __ControlConstant_h__
