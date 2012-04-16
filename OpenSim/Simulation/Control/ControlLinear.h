#ifndef _ControlLinear_h_
#define _ControlLinear_h_
// ControlLinear.h:
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
