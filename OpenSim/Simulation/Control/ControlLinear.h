#ifndef _ControlLinear_h_
#define _ControlLinear_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ControlLinear.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyObjArray.h>
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

class OSIMSIMULATION_API ControlLinear : public Control {
OpenSim_DECLARE_CONCRETE_OBJECT(ControlLinear, Control);

//=============================================================================
// MEMBER DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Flag that indicates whether or not to linearly interpolate between
	nodes or use step functions. */
	PropertyBool _propUseSteps;
	/** Array of control nodes. */
	PropertyObjArray<ControlLinearNode> _propXNodes;
	PropertyObjArray<ControlLinearNode> _propMinNodes;
	PropertyObjArray<ControlLinearNode> _propMaxNodes;
	/** Position gain for PD follower filter. */
	PropertyDbl _propKp;
	/** Velocity gain for PD follower filter. */
	PropertyDbl _propKv;

	// REFERENCES
	bool &_useSteps;
	ArrayPtrs<ControlLinearNode> &_xNodes;
	ArrayPtrs<ControlLinearNode> &_minNodes;
	ArrayPtrs<ControlLinearNode> &_maxNodes;
	double &_kp;
	double &_kv;


	/** Utility node for speeding up searches for control values in
	getControlValue() and elsewhere.  Without this node, a control node would
	need to be contructed, but this is too expensive.  It is better to contruct
	a node up front, and then just alter the time. */
	ControlLinearNode _searchNode;

//=============================================================================
// METHODS
//=============================================================================
public:
	ControlLinear();
	ControlLinear(const ControlLinear &aControl);
	virtual ~ControlLinear();

	void copyData(const ControlLinear &aControl);
protected:
	virtual void setupProperties();
	
private:
	void setNull();
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
	// PROPERTIES
	// Flag indicating whether to interpolate between nodes using step functions
	// or linear interpolation
	void setUseSteps(bool aTrueFalse);
	bool getUseSteps() const;
	// Kp
	void setKp(double aKp);
	double getKp() const;
	// Kv
	void setKv(double aKv);
	double getKv() const;
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
	void clearControlNodes();
	ArrayPtrs<ControlLinearNode>& getControlValues() {
		return (_xNodes);
	}
	ArrayPtrs<ControlLinearNode>& getControlMinValues() {
		return (_minNodes);
	}
	ArrayPtrs<ControlLinearNode>& getControlMaxValues() {
		return (_maxNodes);
	}
	// Insert methods that allocate and insert a copy.
	// These are called from GUI to work around early garbage collection
	void insertNewValueNode(int index, const ControlLinearNode& newNode) {
		_xNodes.insert(index, newNode.clone());
	}
	void insertNewMinNode(int index, const ControlLinearNode& newNode) {
		_minNodes.insert(index, newNode.clone());
	}
	void insertNewMaxNode(int index, const ControlLinearNode& newNode) {
		_maxNodes.insert(index, newNode.clone());
	}
	// Convenience methods
	virtual const double getFirstTime() const;
	virtual const double getLastTime() const;

	// SIMPLIFY
	virtual void simplify(const PropertySet &aProperties);
	bool simplify(const double& cutoffFrequency, const double& distance);
	virtual void filter(double aT);

    // INTERPOLATE
    static double Interpolate(double aX1,double aY1,double aX2,double aY2,double aX);

private:
	void setControlValue(ArrayPtrs<ControlLinearNode> &aNodes,double aT,double aX);
	double getControlValue(ArrayPtrs<ControlLinearNode> &aNodes,double aT);
	double extrapolateBefore(const ArrayPtrs<ControlLinearNode> &aNodes,double aT) const;
	double extrapolateAfter(ArrayPtrs<ControlLinearNode> &aNodes,double aT) const;

//=============================================================================
};	// END of class ControlLinear

}; //namespace
//=============================================================================
//=============================================================================

#endif // __ControlLinear_h__
