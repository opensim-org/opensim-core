#ifndef _ControlLinear_h_
#define _ControlLinear_h_
// ControlLinear.h:
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
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include "Control.h"
#include "ControlLinearNode.h"


//=============================================================================
//=============================================================================
/**
 * A class that represents a piece-wise linear control curve.
 *
 * The curve is specified by an array of control nodes (see class
 * ControlLinearNode) that occur at monotonically increasing times.
 * The value of the control curve is computed by linearly interpolating
 * the values of the appropriate control nodes.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class RDSIMULATION_API ControlLinear : public Control
{

//=============================================================================
// MEMBER DATA
//=============================================================================
public:
	/** Default control node. */
	static const ControlLinearNode DEFAULT_NODE;

protected:
	// PROPERTIES
	/** Flag that indicates whether or not to linearly interpolate between
	nodes or use step functions. */
	PropertyBool _propUseSteps;
	/** Array of control nodes. */
	PropertyObjArray _propNodes;

	// REFERENCES
	bool &_useSteps;
	ArrayPtrs<ControlLinearNode> &_nodes;


	/** Utility node for speeding up searches for control values in
	getControlValue() and elsewhere.  Without this node, a control node would
	need to be contructed, but this is too expensive.  It is better to contruct
	a node up front, and then just alter the time. */
	ControlLinearNode _searchNode;


//=============================================================================
// METHODS
//=============================================================================
public:
	ControlLinear(ArrayPtrs<ControlLinearNode> *aX=NULL,
		const std::string &aName="UNKOWN");
	ControlLinear(DOMElement *aElement);
	ControlLinear(const ControlLinear &aControl);
	virtual ~ControlLinear();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
protected:
	virtual void setupProperties();
	
private:
	void setNull();
	void copyData(const ControlLinear &aControl);
	double extrapolateBefore(double aT) const;
	double extrapolateAfter(double aT) const;
	double extrapolateMinBefore(double aT) const;
	double extrapolateMinAfter(double aT) const;
	double extrapolateMaxBefore(double aT) const;
	double extrapolateMaxAfter(double aT) const;

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	ControlLinear& operator=(const ControlLinear &aControl);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setUseSteps(bool aTrueFalse);
	bool getUseSteps() const;
	// PARAMETERS
	// Number
	virtual int getNumParameters() const;
	// Parameter Min
	virtual void setParameterMin(int aI,double aMin);
	virtual double getParameterMin(int aI) const;
	// Parameter Max
	virtual void setParameterMax(int aI,double aMax);
	virtual double getParameterMax(int aI) const;
	// Parameter Time and Neighborhood
	virtual double getParameterTime(int aI) const;
	virtual void getParameterNeighborhood(int aI,double &rTLower,double &rTUpper) const;
	// Parmeter List
	virtual int getParameterList(double aT,Array<int> &rList);
	virtual int getParameterList(double aT1,double aT2,Array<int> &rList);
	// Parameter Value
	virtual void setParameterValue(int aI,double aP);
	virtual double getParameterValue(int aI) const;
	// CONTROL VALUE
	virtual void setControlValue(double aT,double aX);
	virtual double getControlValue(double aT);
	virtual double getControlValueMin(double aT=0.0);
	virtual void setControlValueMin(double aT,double aX);
	virtual double getControlValueMax(double aT=0.0);
	virtual void setControlValueMax(double aT,double aX);

	// NODE ARRAY
#ifndef SWIG
	const ArrayPtrs<ControlLinearNode>& getNodeArray() const;
#endif
	ArrayPtrs<ControlLinearNode>& getNodeArray(); 
	// Convenience methods
	virtual const double getFirstTime() const;
	virtual const double getLastTime() const;

	// SIMPLIFY
	virtual void
		simplify(const PropertySet &aProperties);

	OpenSim_DERIVED(ControlLinear, Control)

//=============================================================================
};	// END of class ControlLinear

}; //namespace
//=============================================================================
//=============================================================================

#endif // __ControlLinear_h__
