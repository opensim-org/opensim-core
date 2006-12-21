// ControlLinear.cpp
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Signal.h>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/Property.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/PropertySet.h>
#include "ControlLinear.h"
#include "ControlLinearNode.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// STATIC CONSTANTS
//=============================================================================
const ControlLinearNode ControlLinear::DEFAULT_NODE(0.0,0.0,0.0,1.0);


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ControlLinear::~ControlLinear()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aX Pointer to an array of control nodes.  By default, the value of
 * aX is NULL.  If it is not NULL, the control nodes pointed to
 * by aX are copied.  This class keeps its own internal array of control
 * nodes.  The caller owns the array pointed to by aX, is free to use
 * this array in any way, and, if necessary, is responsible for deleting the
 * memory associated with aX.
 * @param aName Name of the control.
 *
 */
ControlLinear::
ControlLinear(ArrayPtrs<ControlLinearNode> *aX,const string &aName) :
	_useSteps(_propUseSteps.getValueBool()),
	_nodes((ArrayPtrs<ControlLinearNode>&)_propNodes.getValueObjArray()),
	_kp(_propKp.getValueDbl()),
	_kv(_propKv.getValueDbl())
{
	setNull();
	if(aX!=NULL) {
		_nodes = *aX;
	}
	setName(aName);
}
//_____________________________________________________________________________
/**
 * Construct a control from an XML Element.
 *
 * @param aElement XML element.
 */
ControlLinear::ControlLinear(DOMElement *aElement) :
	Control(aElement),
	_useSteps(_propUseSteps.getValueBool()),
	_nodes((ArrayPtrs<ControlLinearNode>&)_propNodes.getValueObjArray()),
	_kp(_propKp.getValueDbl()),
	_kv(_propKv.getValueDbl())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aControl Control to copy.
 */
ControlLinear::ControlLinear(const ControlLinear &aControl) :
	Control(aControl),
	_useSteps(_propUseSteps.getValueBool()),
	_nodes((ArrayPtrs<ControlLinearNode>&)_propNodes.getValueObjArray()),
	_kp(_propKp.getValueDbl()),
	_kv(_propKv.getValueDbl())
{
	setNull();
	copyData(aControl);
}
//_____________________________________________________________________________
/**
 * Construct a copy of this control.
 */
Object* ControlLinear::
copy() const
{
	return(new ControlLinear(*this));
}
//_____________________________________________________________________________
/**
 * Copy this control and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using the contructor for the DOMElement
 * in order to establish the relationship of the control with the
 * XML node.  Then, the assignment operator is used to set all member variables
 * of the copy to the values of this object.  Finally, the data members of
 * the copy are updated from the DOMElment using updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this actuator.
 */
Object* ControlLinear::
copy(DOMElement *aElement) const
{
	// ESTABLISH RELATIONSHIP WITH XML NODE
	ControlLinear *control = (ControlLinear *)this->copy();

	// ASSIGNMENT OPERATOR
	control->setXMLNode(aElement);

	// UPDATE BASED ON NODE
	control->updateFromXMLNode();

	return(control);
}


//=============================================================================
// CONSTRUCTION/DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the member data to their NULL values.
 */
void ControlLinear::
setNull()
{
	setType("ControlLinear");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ControlLinear::
setupProperties()
{
	_propUseSteps.setName("use_steps");
	_propUseSteps.setValue(false);
	_propertySet.append( &_propUseSteps );

	_propNodes.setName("nodes");
	ArrayPtrs<Object> nodes;
	_propNodes.setValue(nodes);
	_propertySet.append( &_propNodes );

	_propKp.setName("kp");
	_propKp.setValue(100);
	_propertySet.append( &_propKp );

	_propKp.setName("kv");
	_propKp.setValue(20);
	_propertySet.append( &_propKv );
}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified control.
 */
void ControlLinear::
copyData(const ControlLinear &aControl)
{
	_useSteps = aControl.getUseSteps();
	_nodes = aControl.getNodeArray();
	_kp = aControl.getKp();
	_kv = aControl.getKv();
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  Reference to the altered object.
 */
ControlLinear& ControlLinear::
operator=(const ControlLinear &aControl)
{
	// BASE CLASS
	Control::operator=(aControl);

	// DATA
	copyData(aControl);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// USE STEPS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not step functions are used between control nodes or
 * linear interpolation.  When step functions are used, the value of the
 * control curve between two nodes is the value of the node that occurs later
 * in time.
 *
 * @param aTrueFalse If true, step functions will be used to determine the
 * value between adjacent nodes.  If false, linear interpolation will be used.
 */
void ControlLinear::
setUseSteps(bool aTrueFalse)
{
	_useSteps = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not step functions are used between control nodes or
 * linear interpolation.  When step functions are used, the value of the
 * control curve between two nodes is the value of the node that occurs later
 * in time.
 *
 * @return True if steps functions are used.  False if linear interpolation
 * is used.
 */
bool ControlLinear::
getUseSteps() const
{
	return(_useSteps);
}

//-----------------------------------------------------------------------------
// KP
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set position gain for PD follower filter.  This value is relevant only if
 * the PD follower filter will be used.
 *
 * @param aKp Value of position gain for the PD follower filter.
 */
void ControlLinear::
setKp(double aKp)
{
	_kp = aKp;
}
//_____________________________________________________________________________
/**
 * Get position gain for PD follower filter.  This value is relevant only if
 * the PD follower filter will be used.
 *
 * @return Value of position gain for the PD follower filter.
 */
double ControlLinear::
getKp() const
{
	return(_kp);
}

//-----------------------------------------------------------------------------
// KV
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set velocity gain for PD follower filter.  This value is relevant only if
 * the PD follower filter will be used.
 *
 * @param aKv Value of velocity gain for the PD follower filter.
 */
void ControlLinear::
setKv(double aKv)
{
	_kv = aKv;
}
//_____________________________________________________________________________
/**
 * Get velocity gain for PD follower filter.  This value is relevant only if
 * the PD follower filter will be used.
 *
 * @return Value of velocity gain for the PD follower filter.
 */
double ControlLinear::
getKv() const
{
	return(_kv);
}

//-----------------------------------------------------------------------------
// NUMBER OF PARAMETERS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the number of parameters that are used to specify the control curve.
 *
 * @return Number of parameters.
 */
int ControlLinear::
getNumParameters() const
{
	return(_nodes.getSize());
}

//-----------------------------------------------------------------------------
// PARAMETER MIN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum value of a control parameter.
 *
 * @param aI Index of the parameter.
 * @param aMin Minimum value of the parameter.
 * @throws Exception if aI is invalid.
 */
void ControlLinear::
setParameterMin(int aI,double aMin)
{
	_nodes.get(aI)->setMin(aMin);
}
//_____________________________________________________________________________
/**
 * Get the minimum value of a control parameter.
 *
 * @param aI Index of the parameter.
 * @return Minimum value of the parameter.
 * @throws Exception if aI is invalid.
 */
double ControlLinear::
getParameterMin(int aI) const
{
	return(_nodes.get(aI)->getMin());
}

//-----------------------------------------------------------------------------
// PARAMETER MAX
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum value of a control parameter.
 *
 * @param aI Index of the parameter.
 * @param aMax Maximum value of the parameter.
 * @throws Exception if aI is invalid.
 */
void ControlLinear::
setParameterMax(int aI,double aMax)
{
	_nodes.get(aI)->setMax(aMax);
}
//_____________________________________________________________________________
/**
 * Get the maximum value of a control parameter.
 *
 * @param aI Index of the parameter.
 * @return Maximum value of the parameter.
 * @throws Exception if aI is invalid.
 */
double ControlLinear::
getParameterMax(int aI) const
{
	return(_nodes.get(aI)->getMax());
}

//-----------------------------------------------------------------------------
// PARAMETER TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the time at which a parameter is specified.
 *
 * Parameters for some types of control curves do not have a time at which
 * they are specified.  For example, in a Fourier series the control
 * parameters are the cooefficients in the expansion, and each term in
 * the expansion corresponds not to a specific time but to a frequency.
 * Another example is a constant that has the same value for all times.
 * In these cases, this method returns rdMath::NAN.
 *
 * @param aI Index of the parameter.
 * @return Time at which the control parameter occurs.  For ControlLinear
 * this value is not defined, and so rdMath::NAN is always returned.
 * @throws Exception if aI is invalid.
 */
double ControlLinear::
getParameterTime(int aI) const
{
	return(_nodes.get(aI)->getTime());
}

//-----------------------------------------------------------------------------
// PARAMETER NEIGHBORHOOD
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the time neighborhood (i.e., the lower and upper bounds of time)
 * in which a control parameter affects the value of the control curve.
 *
 * Changes in the specified parameter are guarranteed not to change the value
 * of the control curve below the lower bound time or above the upper bound
 * time.  If a parameter influences the value of the control curve for all
 * times, rdMath::MINUS_INFINITY and rdMath::PLUS_INFINITY are returned for
 * the upper and lower bound times, respectively.
 *
 * @param aI Index of the parameter.
 * @param rTLower Time below which the curve is not affected by the specified
 * parameter.  For ControlLinear, aTLower is the time of parameter aI-1 or of
 * aI if there is no parameter aI-1.  If there are no nodes at all or if
 * aI is invalid, aTLower is given the value rdMath::NAN.
 * @param rTUpper Time above which the curve is not affected by the specified
 * parameter.  For ControlLinear, aTUpper is the time of parameter aI+1 or of
 * aI if there is no parameter aI+1.  If there are no nodes at all or if
 * aI is invalid, aTUpper is given the value rdMath::NAN.
 * @throws Exception if aI is invalid.
 */
void ControlLinear::
getParameterNeighborhood(int aI,double &rTLower,double &rTUpper) const
{
	rTLower = rdMath::NAN;
	rTUpper = rdMath::NAN;

	// CHECK THAT THE NODE EXISTS
	// An exception is thrown if aI is out of bounds. 
	_nodes.get(aI);

	// NEIGHBORING NODES
	int size = _nodes.getSize();
	if(size==1) {
		rTLower = rdMath::MINUS_INFINITY;
		rTUpper = rdMath::PLUS_INFINITY;
		return;
	}
	int lower = aI - 1;
	if(lower<0) lower = 0;
	int upper;
	if(_useSteps) upper = aI;
	else  upper = aI + 1;
	if(upper>=size) upper = size-1;
	rTLower = _nodes.get(lower)->getTime();
	rTUpper = _nodes.get(upper)->getTime();
}

//-----------------------------------------------------------------------------
// PARAMETER LIST
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the list of parameters that affect the control curve at a
 * specified time.
 *
 * @param aT Time in question.
 * @param rList Array of control parameters that affect the curve at time aT.
 * For ControlLinear, if aT lies between two nodes, the indices of these
 * two nodes are returned; if aT equals the time at which a node occurs, the
 * index of that node is returned; if aT is less than the time of the first
 * node in the array, the index of the first node (i.e., 0) is returned;
 * if aT is greater than the time of the last node, the index of the last
 * node (i.e., size-1) is returned.
 * @return Number of parameters in the list.
 */
int ControlLinear::
getParameterList(double aT,Array<int> &rList)
{
	rList.setSize(0);

	// CHECK SIZE
	int size = _nodes.getSize();
	if(size<=0) return(0);

	// FIND THE NODE
	_searchNode.setTime(aT);
	int i = _nodes.searchBinary(_searchNode);

	// LESS THAN TIME OF FIRST NODE
	if(i<0) {
		rList.append(0);

	// GREATER THAN TIME OF LAST NODE
	} else if(i>=(size-1)) {
		rList.append(size-1);

	// EQUAL & LINEAR INTERPOLATION
	} else if((!_useSteps) && (_searchNode == (*_nodes.get(i)) )) {
		rList.append(i);

	// BETWEEN & LINEAR INTERPOLATION
	} else if(!_useSteps)  {
		rList.append(i);
		rList.append(i+1);

	// STEPS
	} else {
		rList.append(i+1);
	}

	return(rList.getSize());
}
//_____________________________________________________________________________
/**
 * Get the list of parameters that affect the control curve between two
 * specified times and that do NOT affect the control curve below the lower
 * of these two times.
 *
 * This method is useful when solving for a set of controls for a dynamic
 * simulation.  When solving for a set of controls, one always wants to
 * go forward in time.  Therefore, one does not want to change control
 * parameters that affect the control curve at past times.
 *
 * A control parameter is included in the list only if it affects
 * the control curve in the specified time interval AND does NOT
 * affect the control curve below the lower bound of the
 * specified time interval.  So, it is possible that some of the
 * parameters on the returned list could affect the control curve at
 * times greater than the upper bound of the specified time interval.
 *
 * @param aTLower Lower time bound.  The control curves are not affected
 * below this time by any of the returned parameters.
 * @param aTUpper Upper time bound.  The control curves may be affected
 * for times greater than this time.
 * @param rList List of control parameters (their indices to be exact) that
 * affect the curve between aTLower and aTUpper but not before aTLower.
 * @return Number of parameters indices in the list.
 */
int ControlLinear::
getParameterList(double aTLower,double aTUpper,Array<int> &rList)
{
	rList.setSize(0);

	// CHECK SIZE
	int size = _nodes.getSize();
	if(size<=0) return(0);

	// CHECK FOR VALID INTERVAL
	if(aTLower>aTUpper) return(0);

	// LOWER NODE
	_searchNode.setTime(aTLower);
	int iL = _nodes.searchBinary(_searchNode);
	if(iL==-1) {
		iL += 1;
	} else if(iL==(size-1)) {
		return(0);
	} else if( (*_nodes.get(iL)) == _searchNode ) {
		iL += 1;
	} else {
		iL += 2;
	}

	// UPPER NODE
	_searchNode.setTime(aTUpper);
	int iU = _nodes.searchBinary(_searchNode);
	if(iU==-1) {
		return(0);
	} else if( (*_nodes.get(iU)) < _searchNode) {
		iU += 1;
	}

	// FORM LIST
	while(iL<=iU) {
		if(iL>=size) return(rList.getSize());
		rList.append(iL);
		iL++;
	}

	return(rList.getSize());
}

//-----------------------------------------------------------------------------
// PARAMETER VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of a control parameter.
 *
 * @param aI Index of the parameter.
 * @param aX Value of the parameter.  For ControlLinear, the parameter
 * value is simply the value of the control node.
 * @see getNumParameters()
 * @throws Exception if aI is invalid.
 */
void ControlLinear::
setParameterValue(int aI,double aX)
{
	_nodes.get(aI)->setValue(aX);
}
//_____________________________________________________________________________
/**
 * Get the value of a control parameter.
 *
 * @param aI Index of the parameter.
 * @return Value of the parameter.  For ControlLinear, the parameter value
 * is simply the value of the control node.
 */
double ControlLinear::
getParameterValue(int aI) const
{
	return(_nodes.get(aI)->getValue());
}

//-----------------------------------------------------------------------------
// CONTROL VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this control curve at time aT.
 *
 * This method adds a set of control parameters at the specified time unless
 * the specified time equals the time of an existing control node, in which
 * case the parameters of that control node are changed.
 *
 * @param aT Time at which to set the control.
 * @param aX Control value.
 */
void ControlLinear::
setControlValue(double aT,double aX)
{
	ControlLinearNode node(aT,aX,getDefaultParameterMin(),getDefaultParameterMax());
	int lower = _nodes.searchBinary(node);

	// NO NODE
	if(lower<0) {
		_nodes.insert(0, (ControlLinearNode*)node.copy() );

	// CHECK NODE
	} else {

		int upper = lower + 1;

		// EQUAL TO LOWER NODE
		if( (*_nodes[lower]) == node) {
			_nodes[lower]->setTime(aT);
			_nodes[lower]->setValue(aX);

		// NOT AT END OF ARRAY
		} else if(upper<_nodes.getSize()) {

			// EQUAL TO UPPER NODE
			if( (*_nodes[upper]) == node) {
				_nodes[upper]->setTime(aT);
				_nodes[upper]->setValue(aX);

			// NOT EQUAL
			} else {
				_nodes.insert(upper, (ControlLinearNode*)node.copy() );
			}

		// AT END OF ARRAY
		} else {
			_nodes.append( (ControlLinearNode*)node.copy() );
		}
	}
}
//_____________________________________________________________________________
/**
 * Get the value of this control at time aT.
 *
 * @param aT Time at which to get the control.
 * @return Control value.  If the value of the curve is not defined,
 * rdMath::NAN is returned.  If the control is set to extrapolate,
 * getExtraplate, and the time is before the first node or
 * after the last node, then an extrapolation is performed to determin
 * the value of the control curve.  Otherwise, the value of either the
 * first control node or last control node is returned.
 */
double ControlLinear::
getControlValue(double aT)
{
	// CHECK SIZE
	int size = _nodes.getSize();
	if(size<=0) return(rdMath::NAN);

	// GET NODE
	_searchNode.setTime(aT);
	int i = _nodes.searchBinary(_searchNode);

	// BEFORE FIRST
	double value;
	if(i<0) {
		if(getExtrapolate()) {
			value = extrapolateBefore(aT);
		} else {
			value = _nodes[0]->getValue();
		}

	// AFTER LAST
	} else if(i>=(size-1)) {
		if(getExtrapolate()) {
			value = extrapolateAfter(aT);
		} else {
			value = _nodes.getLast()->getValue();
		}

	// IN BETWEEN
	} else {

		// LINEAR INTERPOLATION
		if(!_useSteps) {
			double t1,v1,t2,v2;
			t1 = _nodes[i]->getTime();
			v1 = _nodes[i]->getValue();
			t2 = _nodes[i+1]->getTime();
			v2 = _nodes[i+1]->getValue();
			value = rdMath::Interpolate(t1,v1,t2,v2,aT);

		// STEPS
		} else {
			// Eran: Changed semantics of piecewise constant controls so that
			// the control value stored at time t(i+1) is applied to the time
			// interval (t(i),t(i+1)] *exclusive* of time t(i).
			// This was essential to get forward simulation to match cmcgait simulation
			// much better.  During cmcgait simulation of interval [t1,t2] when the
			// integrator reaches time t2 it would pick up the control value at t2
			// because it had yet to compute the piecewise linear control value
			// at time t3.  During forward simulation, when the integrator reaches t2
			// the control at t3 is known but for consistency with cmcgait we need to
			// use the control value at t2.  Hence the (t(i),t(i+1)] choice.
			if (aT == _nodes[i]->getTime()) value = _nodes[i]->getValue();
			else value = _nodes[i+1]->getValue();
		}
	}

	return(value);
}
//_____________________________________________________________________________
/**
 * Extrapolate the value of the control curve before the first node.
 *
 * Currently, simple linear extrapolation using the first two nodes is
 * used.
 *
 * @param aT Time at which to evalute the control curve.
 * @return Extrapolated value of the control curve.
 */
double ControlLinear::
extrapolateBefore(double aT) const
{
	if(_nodes.getSize()<=0) return(rdMath::NAN);
	if(_nodes.getSize()==1) return(_nodes[0]->getValue());

	double t1,v1,t2,v2;
	t1 = _nodes[0]->getTime();
	v1 = _nodes[0]->getValue();
	t2 = _nodes[1]->getTime();
	v2 = _nodes[1]->getValue();
	double value = rdMath::Interpolate(t1,v1,t2,v2,aT);

	return(value);
}
//_____________________________________________________________________________
/**
 * Extrapolate the value of the control curve after the last node.
 *
 * Currently, simple linear extrapolation using the last two nodes is
 * used.
 *
 * @param aT Time at which to evalute the control curve.
 * @return Extrapolated value of the control curve.
 */
double ControlLinear::
extrapolateAfter(double aT) const
{
	int size = _nodes.getSize();
	if(size<=0) return(rdMath::NAN);
	if(size==1) return(_nodes[0]->getValue());

	int n1 = size - 2;
	int n2 = size - 1;
	double t1,v1,t2,v2;
	t1 = _nodes[n1]->getTime();
	v1 = _nodes[n1]->getValue();
	t2 = _nodes[n2]->getTime();
	v2 = _nodes[n2]->getValue();
	double value = rdMath::Interpolate(t1,v1,t2,v2,aT);

	return(value);
}

//-----------------------------------------------------------------------------
// CONTROL VALUE MINIMUM
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum value of this control curve at time aT.
 *
 * This method adds a set of control parameters at the specified time unless
 * the specified time equals the time of an existing control node, in which
 * case the parameters of that control node are changed.
 *
 * @param aT Time at which to set the control.
 * @param aMin Minimum allowed control value.
 */
void ControlLinear::
setControlValueMin(double aT,double aMin)
{
	ControlLinearNode node(aT,0.0,getDefaultParameterMin(),getDefaultParameterMax());
	node.setMin(aMin);
	int lower = _nodes.searchBinary(node);

	// NO NODE
	if(lower<0) {
		_nodes.insert(0, (ControlLinearNode*)node.copy() );

	// CHECK NODE
	} else {

		int upper = lower + 1;

		// EQUAL TO LOWER NODE
		if( (*_nodes[lower]) == node) {
			_nodes[lower]->setTime(aT);
			_nodes[lower]->setMin(aMin);

		// NOT AT END OF ARRAY
		} else if(upper<_nodes.getSize()) {

			// EQUAL TO UPPER NODE
			if( (*_nodes[upper]) == node) {
				_nodes[upper]->setTime(aT);
				_nodes[upper]->setMin(aMin);

			// NOT EQUAL
			} else {
				_nodes.insert(upper, (ControlLinearNode*)node.copy() );
			}

		// AT END OF ARRAY
		} else {
			_nodes.append( (ControlLinearNode*)node.copy() );
		}
	}
}
//_____________________________________________________________________________
/**
 * Get the minimum allowed value of this control at time aT.
 *
 * @param aT Time at which to get the control.
 * @return Minimum allowed control value.  If the value of the curve is not defined,
 * rdMath::NAN is returned.  If the control is set to extrapolate,
 * getExtraplate, and the time is before the first node or
 * after the last node, then an extrapolation is performed to determin
 * the value of the control curve.  Otherwise, the value of either the
 * first control node or last control node is returned.
 */
double ControlLinear::
getControlValueMin(double aT)
{
	// CHECK SIZE
	int size = _nodes.getSize();
	if(size<=0) return(rdMath::NAN);

	// GET NODE
	_searchNode.setTime(aT);
	int i = _nodes.searchBinary(_searchNode);

	// BEFORE FIRST
	double t1,v1,t2,v2;
	double value;
	if(i<0) {
		if(getExtrapolate()) {
			value = extrapolateMinBefore(aT);
		} else {
			value = _nodes[0]->getMin();
		}

	// AFTER LAST
	} else if(i>=(size-1)) {
		if(getExtrapolate()) {
			value = extrapolateMinAfter(aT);
		} else {
			value = _nodes.getLast()->getMin();
		}

	// IN BETWEEN
	} else {

		// LINEAR INTERPOLATION
		if(!_useSteps) {
			t1 = _nodes[i]->getTime();
			v1 = _nodes[i]->getMin();
			t2 = _nodes[i+1]->getTime();
			v2 = _nodes[i+1]->getMin();
			value = rdMath::Interpolate(t1,v1,t2,v2,aT);

		// STEPS
		} else {
			value = _nodes[i+1]->getMin();
		}
	}

	return(value);
}
//_____________________________________________________________________________
/**
 * Extrapolate the value of the control curve before the first node.
 *
 * Currently, simple linear extrapolation using the first two nodes is
 * used.
 *
 * @param aT Time at which to evalute the control curve.
 * @return Extrapolated value of the control curve.
 */
double ControlLinear::
extrapolateMinBefore(double aT) const
{
	if(_nodes.getSize()<=0) return(rdMath::NAN);
	if(_nodes.getSize()==1) return(_nodes[0]->getMin());

	double t1,v1,t2,v2;
	t1 = _nodes[0]->getTime();
	v1 = _nodes[0]->getMin();
	t2 = _nodes[1]->getTime();
	v2 = _nodes[1]->getMin();
	double value = rdMath::Interpolate(t1,v1,t2,v2,aT);

	return(value);
}
//_____________________________________________________________________________
/**
 * Extrapolate the value of the control curve after the last node.
 *
 * Currently, simple linear extrapolation using the last two nodes is
 * used.
 *
 * @param aT Time at which to evalute the control curve.
 * @return Extrapolated value of the control curve.
 */
double ControlLinear::
extrapolateMinAfter(double aT) const
{
	int size = _nodes.getSize();
	if(size<=0) return(rdMath::NAN);
	if(size==1) return(_nodes[0]->getMin());

	int n1 = size - 2;
	int n2 = size - 1;
	double t1,v1,t2,v2;
	t1 = _nodes[n1]->getTime();
	v1 = _nodes[n1]->getMin();
	t2 = _nodes[n2]->getTime();
	v2 = _nodes[n2]->getMin();
	double value = rdMath::Interpolate(t1,v1,t2,v2,aT);

	return(value);
}


//-----------------------------------------------------------------------------
// CONTROL VALUE MAXIMUM
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum value of this control curve at time aT.
 *
 * This method adds a set of control parameters at the specified time unless
 * the specified time equals the time of an existing control node, in which
 * case the parameters of that control node are changed.
 *
 * @param aT Time at which to set the control.
 * @param aMax Maximum allowed control value.
 */
void ControlLinear::
setControlValueMax(double aT,double aMax)
{
	ControlLinearNode node(aT,0.0,getDefaultParameterMin(),getDefaultParameterMax());
	node.setMax(aMax);
	int lower = _nodes.searchBinary(node);

	// NO NODE
	if(lower<0) {
		_nodes.insert(0, (ControlLinearNode*)node.copy() );

	// CHECK NODE
	} else {

		int upper = lower + 1;

		// EQUAL TO LOWER NODE
		if( (*_nodes[lower]) == node) {
			_nodes[lower]->setTime(aT);
			_nodes[lower]->setMax(aMax);

		// NOT AT END OF ARRAY
		} else if(upper<_nodes.getSize()) {

			// EQUAL TO UPPER NODE
			if( (*_nodes[upper]) == node) {
				_nodes[upper]->setTime(aT);
				_nodes[upper]->setMax(aMax);

			// NOT EQUAL
			} else {
				_nodes.insert(upper, (ControlLinearNode*)node.copy() );
			}

		// AT END OF ARRAY
		} else {
			_nodes.append( (ControlLinearNode*)node.copy() );
		}
	}
}
//_____________________________________________________________________________
/**
 * Get the maximum allowed value of this control at time aT.
 *
 * @param aT Time at which to get the control.
 * @return Maximum allowed control value.  If the value of the curve is not defined,
 * rdMath::NAN is returned.  If the control is set to extrapolate,
 * getExtraplate, and the time is before the first node or
 * after the last node, then an extrapolation is performed to determin
 * the value of the control curve.  Otherwise, the value of either the
 * first control node or last control node is returned.
 */
double ControlLinear::
getControlValueMax(double aT)
{
	// CHECK SIZE
	int size = _nodes.getSize();
	if(size<=0) return(rdMath::NAN);

	// GET NODE
	_searchNode.setTime(aT);
	int i = _nodes.searchBinary(_searchNode);

	// BEFORE FIRST
	double t1,v1,t2,v2;
	double value;
	if(i<0) {
		if(getExtrapolate()) {
			value = extrapolateMaxBefore(aT);
		} else {
			value = _nodes[0]->getMax();
		}

	// AFTER LAST
	} else if(i>=(size-1)) {
		if(getExtrapolate()) {
			value = extrapolateMaxAfter(aT);
		} else {
			value = _nodes.getLast()->getMax();
		}

	// IN BETWEEN
	} else {

		// LINEAR INTERPOLATION
		if(!_useSteps) {
			t1 = _nodes[i]->getTime();
			v1 = _nodes[i]->getMax();
			t2 = _nodes[i+1]->getTime();
			v2 = _nodes[i+1]->getMax();
			value = rdMath::Interpolate(t1,v1,t2,v2,aT);

		// STEPS
		} else {
			value = _nodes[i+1]->getMax();
		}
	}

	return(value);
}
//_____________________________________________________________________________
/**
 * Extrapolate the value of the control curve before the first node.
 *
 * Currently, simple linear extrapolation using the first two nodes is
 * used.
 *
 * @param aT Time at which to evalute the control curve.
 * @return Extrapolated value of the control curve.
 */
double ControlLinear::
extrapolateMaxBefore(double aT) const
{
	if(_nodes.getSize()<=0) return(rdMath::NAN);
	if(_nodes.getSize()==1) return(_nodes[0]->getMax());

	double t1,v1,t2,v2;
	t1 = _nodes[0]->getTime();
	v1 = _nodes[0]->getMax();
	t2 = _nodes[1]->getTime();
	v2 = _nodes[1]->getMax();
	double value = rdMath::Interpolate(t1,v1,t2,v2,aT);

	return(value);
}
//_____________________________________________________________________________
/**
 * Extrapolate the value of the control curve after the last node.
 *
 * Currently, simple linear extrapolation using the last two nodes is
 * used.
 *
 * @param aT Time at which to evalute the control curve.
 * @return Extrapolated value of the control curve.
 */
double ControlLinear::
extrapolateMaxAfter(double aT) const
{
	int size = _nodes.getSize();
	if(size<=0) return(rdMath::NAN);
	if(size==1) return(_nodes[0]->getMax());

	int n1 = size - 2;
	int n2 = size - 1;
	double t1,v1,t2,v2;
	t1 = _nodes[n1]->getTime();
	v1 = _nodes[n1]->getMax();
	t2 = _nodes[n2]->getTime();
	v2 = _nodes[n2]->getMax();
	double value = rdMath::Interpolate(t1,v1,t2,v2,aT);

	return(value);
}

//-----------------------------------------------------------------------------
// NODE ARRAY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the array of control nodes.
 *
 * @return Array of nodes.
 */
const ArrayPtrs<ControlLinearNode>& ControlLinear::
getNodeArray() const
{
	return(_nodes);
}
//_____________________________________________________________________________
/**
 * Get the array of control nodes.
 *
 * @return Array of nodes.
 */
ArrayPtrs<ControlLinearNode>& ControlLinear::
getNodeArray()
{
	return(_nodes);
}
//_____________________________________________________________________________
/**
 * Get const ref to the time corresponding to first node
 *
 * @return const ref to time corresponding to first node.
 */
const double ControlLinear::getFirstTime() const
{
	const ControlLinearNode *node=getNodeArray().get(0);
	return node->getTime();
}
//_____________________________________________________________________________
/**
 * Get const ref to the time corresponding to Last node
 *
 * @return const ref to time corresponding to last node.
 */
const double ControlLinear::getLastTime() const
{
	const ControlLinearNode *node=getNodeArray().getLast();
	return node->getTime();
}

//-----------------------------------------------------------------------------
// SIMPLIFY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Simplify the control curve.
 *
 * The number of control nodes is reduced by first applying a lowpass filter
 * to the sequence of control nodes using a specified cutoff frequency and
 * then removing nodes that keep the curve within a specified distance
 * to the low-pass filtered curve. 
 * 
 * @param aProperties Property set containing the needed properties for
 * this method.  The property set should contain:\n
 * \tTYPE          NAME
 * \tPropertyDbl cutoff_frequency\n
 * \tPropertyDbl distance\n\n
 * @throws Exception if an error is encountered.
 */
void ControlLinear::
simplify(const PropertySet &aProperties)
{
	// INITIAL SIZE
	int size = _nodes.getSize();
	cout<<"\nControlLinear.simplify: initial size = "<<size<<".\n";
	
	// GET THE NODE TIMES
	int i;
	Array<double> t(0.0,size);
	for(i=0;i<size;i++) {
		t[i] = _nodes[i]->getTime();
	}

	// SEARCH FOR THE MINIMUM TIME INTERVAL
	double dt,dtMin=rdMath::PLUS_INFINITY;
	for(i=0;i<(size-1);i++) {
		dt = t[i+1] - t[i];
		if(dt<dtMin) {
			dtMin = dt;
			if(dtMin<=rdMath::ZERO) {
				string msg = "ControlLinear.simplify: zero or negative dt!";
				throw(Exception(msg,__FILE__,__LINE__));
			}
		}
	}
	//cout<<"ControlLinear.simplify: dtMin="<<dtMin<<endl;

	// RESAMPLE THE NODE VALUES
	int n = (int)(1.0 + (t[size-1] - t[0])/dtMin);
	double time;
	Array<double> x(0.0,n);
	t.setSize(n);
	//cout<<"ControlLinear.simplify: resampling using "<<n<<" points.\n";
	for(time=t[0],i=0;i<n;i++,time+=dtMin) {
		t[i] = time;
		x[i] = getControlValue(time);
	}

	// FILTER
	double cutoffFrequency = aProperties.get("cutoff_frequency")->getValueDbl();
	if(cutoffFrequency < rdMath::ZERO) {
		throw(Exception());
	}
	Array<double> xFilt(0.0,n);
	int order = 50;
	if(order>(n/2)) order = n/2;
	if(order<10) {
		cout<<"ControlLinear.simplify: WARN- too few data points ";
		cout<<"(n="<<n<<") to filter "<<getName()<<".\n";
	} else {
		if(order<20) {
			cout<<"ControlLinear.simplify: WARN- order of FIR filter had to be ";
			cout<<"low due to small number of data points ";
			cout<<"(n="<<n<<") in control "<<getName()<<".\n";
		}
		cout<<"ControlLinear.simplify: lowpass filtering with a ";
		cout<<"cutoff frequency of "<<cutoffFrequency<<" and order of ";
		cout<<order<<".\n"; 
		Signal::LowpassFIR(order,dtMin,cutoffFrequency,n,&x[0],&xFilt[0]);
	}

	// REMOVE POINTS
	double distance = aProperties.get("distance")->getValueDbl();
	cout<<"ControlLinear.simplify: reducing points with distance tolerance = ";
	cout<<distance<<".\n";
	Signal::ReduceNumberOfPoints(distance,t,xFilt);	

	// CLEAR OLD NODES
	_nodes.trim();
	_nodes.setSize(0);

	// ADD NEW NODES
	int newSize = t.getSize();
	char name[Object::NAME_LENGTH];
	ControlLinearNode *node;
	for(i=0;i<newSize;i++) {
		node = new ControlLinearNode(t[i],xFilt[i]);
		sprintf(name,"%d",i);
		node->setName(name);
		_nodes.append(node);
	}

	cout<<"ControlLinear.simplify: final size = "<<_nodes.getSize()<<".\n";
}


//-----------------------------------------------------------------------------
// FILTER CONTROL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Filter the control curve at a particular time using a PD follower filter.
 *
 * @param aT Time at which to compute a new, filtered control value
 */
void ControlLinear::
filter(double aT)
{
	// CHECK WHETHER FILTER IS ON
	// TO DO - should we print some error/warning message here?
	if (!_filterOn) return;

	// CHECK SIZE
	// TO DO - should we print some error/warning message here?
	int size = _nodes.getSize();
	if(size<=0) return;

	// FIND CONTROL NODE
	// Find the control node at time aT
	_searchNode.setTime(aT);
	int i = _nodes.searchBinary(_searchNode);
	// The following property is true after binary search:
	// _nodes[i].getValue() <= getControlValue(aT)
	// i.e. the node whose index (i) was returned is the node
	// that occurs immediately before, or exactly at, the time aT.
	// An equivalent property is that
	// _searchNode >= (*_nodes.get(i))
	// which is computed below as the "nodeOccursAtGivenTime" variable.

	// COMPUTE AND SET CONTROL VALUE

	// HANDLE CASE WITH LESS THAN TWO PREVIOUS CONTROL NODES
	// If there are less than two control nodes occurring before
	// time aT, then set the value zero.  The PD follower needs
	// at least two nodes to occur before the time aT in order to
	// compute a new control value for the time aT.
	// The first if statement represents the following cases:
	// i < 0: aT occurs before the first node
	// i == 0: the first node occurs before or at aT
	if (i <= 0) {
		setControlValue(aT, 0.0);
		return;
	}
	// True iff _nodes[i] occurs at aT
	bool nodeOccursAtGivenTime = (_searchNode == (*_nodes.get(i)));
	// This if statement represents the case where the second
	// node occurs at aT.
	if ((i == 1) && nodeOccursAtGivenTime) {
		setControlValue(aT, 0.0);
		return;
	}

	// HANDLE ALL OTHER CASES
	double dt, dtPrev, xPrev, xPrevPrev;
	// If the time of the node at index i is equal to aT (where
	// "equal" is determined by the operator== function of the
	// ControlLinearNode class):
	// (i <= 1 cases were handled above)
	if (nodeOccursAtGivenTime) {
		dt = _nodes[i]->getTime() - _nodes[i-1]->getTime();
		dtPrev = _nodes[i-1]->getTime() - _nodes[i-2]->getTime();
		xPrev = _nodes[i-1]->getValue();
		xPrevPrev = _nodes[i-2]->getValue();

	// If the time of the node at index i is less than aT:
	} else {
		dt = aT - _nodes[i]->getTime();
		dtPrev = _nodes[i]->getTime() - _nodes[i-1]->getTime();
		xPrev = _nodes[i]->getValue();
		xPrevPrev = _nodes[i-1]->getValue();
	}

	// GET CURRENT CONTROL VALUE
	// aT occurs before first node
	double xDes = getControlValue(aT);

	// COMPUTE AND SET NEW FILTERED CONTROL VALUE
	double xDotPrev = (xPrev - xPrevPrev) / dtPrev;
	double xDotDotPrev = -_kv*xDotPrev + _kp*(xDes - xPrev);
	double x = xPrev + xDotPrev*dt + 0.5*xDotDotPrev*dt*dt;

	// Set the control value to the newly computed value
	setControlValue(aT, x);
}
