/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ControlLinear.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */
#include <OpenSim/Common/Signal.h>
#include <OpenSim/Common/PropertySet.h>
#include "ControlLinear.h"

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
ControlLinear::~ControlLinear()
{
}
//_____________________________________________________________________________

ControlLinear::
ControlLinear() :
    _useSteps(_propUseSteps.getValueBool()),
    _xNodes((ArrayPtrs<ControlLinearNode>&)_propXNodes.getValueObjArray()),
    _minNodes((ArrayPtrs<ControlLinearNode>&)_propMinNodes.getValueObjArray()),
    _maxNodes((ArrayPtrs<ControlLinearNode>&)_propMaxNodes.getValueObjArray()),
    _kp(_propKp.getValueDbl()),
    _kv(_propKv.getValueDbl())
{
    setNull();
}
//_____________________________________________________________________________
ControlLinear::ControlLinear(const ControlLinear &aControl) :
    Control(aControl),
    _useSteps(_propUseSteps.getValueBool()),
    _xNodes((ArrayPtrs<ControlLinearNode>&)_propXNodes.getValueObjArray()),
    _minNodes((ArrayPtrs<ControlLinearNode>&)_propMinNodes.getValueObjArray()),
    _maxNodes((ArrayPtrs<ControlLinearNode>&)_propMaxNodes.getValueObjArray()),
    _kp(_propKp.getValueDbl()),
    _kv(_propKv.getValueDbl())
{
    setNull();
    copyData(aControl);
}

//=============================================================================
// CONSTRUCTION/DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
void ControlLinear::
setNull()
{
    setupProperties();
}
//_____________________________________________________________________________
void ControlLinear::
setupProperties()
{
    _propUseSteps.setName("use_steps");
    _propUseSteps.setValue(false);
    _propertySet.append( &_propUseSteps );

    ArrayPtrs<ControlLinearNode> nodes;
    _propXNodes.setName("x_nodes");
    _propXNodes.setValue(nodes);
    _propertySet.append( &_propXNodes );

    _propMinNodes.setName("min_nodes");
    _propMinNodes.setValue(nodes);
    _propertySet.append( &_propMinNodes );

    _propMaxNodes.setName("max_nodes");
    _propMaxNodes.setValue(nodes);
    _propertySet.append( &_propMaxNodes );

    _propKp.setName("kp");
    _propKp.setValue(100);
    _propertySet.append( &_propKp );

    _propKv.setName("kv");
    _propKv.setValue(20);
    _propertySet.append( &_propKv );
}
//_____________________________________________________________________________
void ControlLinear::
copyData(const ControlLinear &aControl)
{
    _useSteps = aControl.getUseSteps();
    _xNodes = aControl._xNodes;
    _minNodes = aControl._minNodes;
    _maxNodes = aControl._maxNodes;
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
void ControlLinear::
setUseSteps(bool aTrueFalse)
{
    _useSteps = aTrueFalse;
}
//_____________________________________________________________________________
bool ControlLinear::
getUseSteps() const
{
    return(_useSteps);
}

//-----------------------------------------------------------------------------
// KP
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlLinear::
setKp(double aKp)
{
    _kp = aKp;
}
//_____________________________________________________________________________
double ControlLinear::
getKp() const
{
    return(_kp);
}

//-----------------------------------------------------------------------------
// KV
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlLinear::
setKv(double aKv)
{
    _kv = aKv;
}
//_____________________________________________________________________________
double ControlLinear::
getKv() const
{
    return(_kv);
}

//-----------------------------------------------------------------------------
// NUMBER OF PARAMETERS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
int ControlLinear::
getNumParameters() const
{
    return(_xNodes.getSize());
}

//-----------------------------------------------------------------------------
// PARAMETER MIN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlLinear::
setParameterMin(int aI,double aMin)
{
    _minNodes.get(aI)->setValue(aMin);
}
//_____________________________________________________________________________
double ControlLinear::
getParameterMin(int aI) const
{
    return(_minNodes.get(aI)->getValue());
}

//-----------------------------------------------------------------------------
// PARAMETER MAX
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlLinear::
setParameterMax(int aI,double aMax)
{
    _maxNodes.get(aI)->setValue(aMax);
}
//_____________________________________________________________________________
double ControlLinear::
getParameterMax(int aI) const
{
    return(_maxNodes.get(aI)->getValue());
}

//-----------------------------------------------------------------------------
// PARAMETER TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
double ControlLinear::
getParameterTime(int aI) const
{
    return(_xNodes.get(aI)->getTime());
}

//-----------------------------------------------------------------------------
// PARAMETER NEIGHBORHOOD
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlLinear::
getParameterNeighborhood(int aI,double &rTLower,double &rTUpper) const
{
    rTLower = SimTK::NaN;
    rTUpper = SimTK::NaN;

    // CHECK THAT THE NODE EXISTS
    // An exception is thrown if aI is out of bounds. 
    _xNodes.get(aI);

    // NEIGHBORING NODES
    int size = _xNodes.getSize();
    if(size==1) {
        rTLower = -SimTK::Infinity;
        rTUpper =  SimTK::Infinity;
        return;
    }
    int lower = aI - 1;
    if(lower<0) lower = 0;
    int upper;
    if(_useSteps) upper = aI;
    else  upper = aI + 1;
    if(upper>=size) upper = size-1;
    rTLower = _xNodes.get(lower)->getTime();
    rTUpper = _xNodes.get(upper)->getTime();
}

//-----------------------------------------------------------------------------
// PARAMETER LIST
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
int ControlLinear::
getParameterList(double aT,Array<int> &rList)
{
    rList.setSize(0);

    // CHECK SIZE
    int size = _xNodes.getSize();
    if(size<=0) return(0);

    // FIND THE NODE
    _searchNode.setTime(aT);
    int i = searchBinary(_xNodes, _searchNode);

    // LESS THAN TIME OF FIRST NODE
    if(i<0) {
        rList.append(0);

    // GREATER THAN TIME OF LAST NODE
    } else if(i>=(size-1)) {
        rList.append(size-1);

    // EQUAL & LINEAR INTERPOLATION
    } else if((!_useSteps) && (_searchNode.isEqual((*_xNodes.get(i))))) {
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
int ControlLinear::
getParameterList(double aTLower,double aTUpper,Array<int> &rList)
{
    rList.setSize(0);

    // CHECK SIZE
    int size = _xNodes.getSize();
    if(size<=0) return(0);

    // CHECK FOR VALID INTERVAL
    if(aTLower>aTUpper) return(0);

    // LOWER NODE
    _searchNode.setTime(aTLower);
    int iL = searchBinary(_xNodes, _searchNode);
    if(iL==-1) {
        iL += 1;
    } else if(iL==(size-1)) {
        return(0);
    } else if( (*_xNodes.get(iL)).isEqual(_searchNode) ) {
        iL += 1;
    } else {
        iL += 2;
    }

    // UPPER NODE
    _searchNode.setTime(aTUpper);
    int iU = searchBinary(_xNodes, _searchNode);
    if(iU==-1) {
        return(0);
    } else if( (*_xNodes.get(iU)).isLessThan(_searchNode) ) {
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
void ControlLinear::
setParameterValue(int aI,double aX)
{
    _xNodes.get(aI)->setValue(aX);
}
//_____________________________________________________________________________
double ControlLinear::
getParameterValue(int aI) const
{
    return(_xNodes.get(aI)->getValue());
}

//-----------------------------------------------------------------------------
// UTILITY
//-----------------------------------------------------------------------------
void ControlLinear::
setControlValue(ArrayPtrs<ControlLinearNode> &aNodes,double aT,double aValue)
{
    ControlLinearNode node(aT,aValue);
    int lower = searchBinary(aNodes, node);

    // NO NODE
    if(lower<0) {
        aNodes.insert(0, node.clone() );

    // CHECK NODE
    } else {

        int upper = lower + 1;

        // EQUAL TO LOWER NODE
        if( (*aNodes[lower]).isEqual(node) ) {
            aNodes[lower]->setTime(aT);
            aNodes[lower]->setValue(aValue);

        // NOT AT END OF ARRAY
        } else if(upper<aNodes.getSize()) {

            // EQUAL TO UPPER NODE
            if( (*aNodes[upper]).isEqual(node) ) {
                aNodes[upper]->setTime(aT);
                aNodes[upper]->setValue(aValue);

            // NOT EQUAL
            } else {
                aNodes.insert(upper, node.clone());
            }

        // AT END OF ARRAY
        } else {
            aNodes.append(node.clone());
        }
    }
}

double ControlLinear::
getControlValue(ArrayPtrs<ControlLinearNode> &aNodes,double aT)
{
    // CHECK SIZE
    int size = aNodes.getSize();
    // CMC expects NaN's to be returned if the Control set size is zero
    if(size<=0) return(SimTK::NaN);

    // GET NODE
    _searchNode.setTime(aT);
    int i = searchBinary(aNodes, _searchNode);

    // BEFORE FIRST
    double value;
    if(i<0) {
        if(!_useSteps && getExtrapolate()) {
            value = extrapolateBefore(aNodes, aT);
        } else {
            value = aNodes[0]->getValue();
        }

    // AFTER LAST
    } else if(i>=(size-1)) {
        if(!_useSteps && getExtrapolate()) {
            value = extrapolateAfter(aNodes, aT);
        } else {
            value = aNodes.getLast()->getValue();
        }

    // IN BETWEEN
    } else {

        // LINEAR INTERPOLATION
        if(!_useSteps) {
            double t1,v1,t2,v2;
            t1 = aNodes[i]->getTime();
            v1 = aNodes[i]->getValue();
            t2 = aNodes[i+1]->getTime();
            v2 = aNodes[i+1]->getValue();
            value = Interpolate(t1,v1,t2,v2,aT);

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
            if (aT == aNodes[i]->getTime()) value = aNodes[i]->getValue();
            else value = aNodes[i+1]->getValue();
        }
    }

    return(value);
}

double ControlLinear::
extrapolateBefore(const ArrayPtrs<ControlLinearNode> &aNodes,double aT) const
{
    if(aNodes.getSize()<=0) return(SimTK::NaN);
    if(aNodes.getSize()==1) return(aNodes[0]->getValue());

    double t1,v1,t2,v2;
    t1 = aNodes[0]->getTime();
    v1 = aNodes[0]->getValue();
    t2 = aNodes[1]->getTime();
    v2 = aNodes[1]->getValue();
    double value = Interpolate(t1,v1,t2,v2,aT);

    return(value);
}

double ControlLinear::
extrapolateAfter(ArrayPtrs<ControlLinearNode> &aNodes,double aT) const
{
    int size = aNodes.getSize();
    if(size<=0) return(SimTK::NaN);
    if(size==1) return(aNodes[0]->getValue());

    int n1 = size - 2;
    int n2 = size - 1;
    double t1,v1,t2,v2;
    t1 = aNodes[n1]->getTime();
    v1 = aNodes[n1]->getValue();
    t2 = aNodes[n2]->getTime();
    v2 = aNodes[n2]->getValue();
    double value = Interpolate(t1,v1,t2,v2,aT);

    return(value);
}

int ControlLinear::
searchBinary(const ArrayPtrs<ControlLinearNode>& nodes,
        const ControlLinearNode& value) const
{
    const int size = nodes.getSize();
    if(size <= 0) {
        return -1;
    }

    int lo = 0;
    int hi = size - 1;
    int mid = -1;

    while(lo <= hi) {
        mid = (lo + hi) / 2;
        const ControlLinearNode& candidate = *nodes[mid];

        if(value.isLessThan(candidate)) {
            hi = mid - 1;
        } else if(candidate.isLessThan(value)) {
            lo = mid + 1;
        } else {
            break;
        }
    }

    if(mid < 0) {
        return -1;
    }

    if(value.isLessThan(*nodes[mid])) {
        mid--;
    }

    if(mid < 0) {
        return -1;
    }

    return mid;
}

//-----------------------------------------------------------------------------
// CONTROL VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlLinear::
setControlValue(double aT,double aX)
{
    setControlValue(_xNodes,aT,aX);
}
//_____________________________________________________________________________
double ControlLinear::
getControlValue(double aT)
{
    return getControlValue(_xNodes,aT);
}
//_____________________________________________________________________________
double ControlLinear::
extrapolateBefore(double aT) const
{
    return extrapolateBefore(_xNodes,aT);
}
//_____________________________________________________________________________
double ControlLinear::
extrapolateAfter(double aT) const
{
    return extrapolateAfter(_xNodes,aT);
}

//-----------------------------------------------------------------------------
// CONTROL VALUE MINIMUM
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlLinear::
setControlValueMin(double aT,double aMin)
{
    setControlValue(_minNodes,aT,aMin);
}
//_____________________________________________________________________________
double ControlLinear::
getControlValueMin(double aT)
{
    if(_minNodes.getSize()==0)
        return _defaultMin;
    else
        return getControlValue(_minNodes,aT);
}
//_____________________________________________________________________________
double ControlLinear::
extrapolateMinBefore(double aT) const
{
    return extrapolateBefore(_minNodes,aT);
}
//_____________________________________________________________________________
double ControlLinear::
extrapolateMinAfter(double aT) const
{
    return extrapolateAfter(_minNodes,aT);
}


//-----------------------------------------------------------------------------
// CONTROL VALUE MAXIMUM
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlLinear::
setControlValueMax(double aT,double aMax)
{
    setControlValue(_maxNodes,aT,aMax);
}
//_____________________________________________________________________________
double ControlLinear::
getControlValueMax(double aT)
{
    if(_minNodes.getSize()==0)
        return _defaultMax;
    else
        return getControlValue(_maxNodes,aT);
}
//_____________________________________________________________________________
double ControlLinear::
extrapolateMaxBefore(double aT) const
{
    return extrapolateBefore(_maxNodes,aT);
}
//_____________________________________________________________________________
double ControlLinear::
extrapolateMaxAfter(double aT) const
{
    return extrapolateAfter(_maxNodes,aT);
}

//-----------------------------------------------------------------------------
// NODE ARRAY
//-----------------------------------------------------------------------------
void ControlLinear::
clearControlNodes()
{
    _xNodes.setSize(0);
}
//_____________________________________________________________________________
double ControlLinear::getFirstTime() const
{
    const ControlLinearNode *node=_xNodes.get(0);
    return node->getTime();
}
//_____________________________________________________________________________
double ControlLinear::getLastTime() const
{
    const ControlLinearNode *node=_xNodes.getLast();
    return node->getTime();
}

//-----------------------------------------------------------------------------
// SIMPLIFY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlLinear::
simplify(const PropertySet &aProperties)
{
    // INITIAL SIZE
    int size = _xNodes.getSize();
    log_info("ControlLinear.simplify: initial size = {}.", size);
    
    // GET THE NODE TIMES
    int i;
    Array<double> t(0.0,size);
    for(i=0;i<size;i++) {
        t[i] = _xNodes[i]->getTime();
    }

    // SEARCH FOR THE MINIMUM TIME INTERVAL
    double dt,dtMin= SimTK::Infinity;
    for(i=0;i<(size-1);i++) {
        dt = t[i+1] - t[i];
        if(dt<dtMin) {
            dtMin = dt;
            if(dtMin<=SimTK::Zero) {
                string msg = "ControlLinear.simplify: zero or negative dt!";
                throw(Exception(msg,__FILE__,__LINE__));
            }
        }
    }

    // RESAMPLE THE NODE VALUES
    int n = (int)(1.0 + (t[size-1] - t[0])/dtMin);
    double time;
    Array<double> x(0.0,n);
    t.setSize(n);
    for(time=t[0],i=0;i<n;i++,time+=dtMin) {
        t[i] = time;
        x[i] = getControlValue(time);
    }

    // FILTER
    double cutoffFrequency = aProperties.get("cutoff_frequency")->getValueDbl();
    if(cutoffFrequency < SimTK::Zero) {
        throw(Exception());
    }
    Array<double> xFilt(0.0,n);
    int order = 50;
    if(order>(n/2)) order = n/2;
    if(order<10) {
        log_warn("ControlLinear.simplify: too few data points (n={}) to filter {}.",
            n, getName());
    } else {
        if(order<20) {
            log_warn("ControlLinear.simplify:  order of FIR filter had to be"
                " low due to small number of data points (n={}) in control {}.",
                n, getName());
        }
        log_info("ControlLinear.simplify: lowpass filtering with a cutoff "
                 "frequency of {} and order of {}.", cutoffFrequency, order); 
        Signal::LowpassFIR(order,dtMin,cutoffFrequency,n,&x[0],&xFilt[0]);
    }

    // REMOVE POINTS
    double distance = aProperties.get("distance")->getValueDbl();
    log_info("ControlLinear.simplify: reducing points with distance tolerance = {}.", distance);
    Signal::ReduceNumberOfPoints(distance,t,xFilt); 

    // CLEAR OLD NODES
    _xNodes.trim();
    _xNodes.setSize(0);

    // ADD NEW NODES
    int newSize = t.getSize();
    ControlLinearNode *node;
    for(i=0;i<newSize;i++) {
        char name[32];
        node = new ControlLinearNode(t[i],xFilt[i]);
        snprintf(name, 32, "%d", i);
        node->setName(name);
        _xNodes.append(node);
    }

    log_info("ControlLinear.simplify: final size = {}.", _xNodes.getSize());
}

bool ControlLinear::
simplify(const double& cutoffFrequency, const double& distance)
{
    PropertySet params;
    params.append(new PropertyDbl("cutoff_frequency", cutoffFrequency));    // Will be deleted by the destructor
    params.append(new PropertyDbl("distance", distance));
    try {
        simplify(params);
        return true;
    }
    catch(const Exception&) {
        return(false);
    }
}
//-----------------------------------------------------------------------------
// FILTER CONTROL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void ControlLinear::
filter(double aT)
{
    // CHECK WHETHER FILTER IS ON
    // TO DO - should we print some error/warning message here?
    if (!_filterOn) return;

    // CHECK SIZE
    // TO DO - should we print some error/warning message here?
    int size = _xNodes.getSize();
    if(size<=0) return;

    // FIND CONTROL NODE
    // Find the control node at time aT
    _searchNode.setTime(aT);
    int i = searchBinary(_xNodes, _searchNode);
    // The following property is true after binary search:
    // _xNodes[i].getValue() <= getControlValue(aT)
    // i.e. the node whose index (i) was returned is the node
    // that occurs immediately before, or exactly at, the time aT.
    // An equivalent property is that
    // _searchNode >= (*_xNodes.get(i))
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
    // True iff _xNodes[i] occurs at aT
    bool nodeOccursAtGivenTime = (_searchNode.isEqual((*_xNodes.get(i))));
    // This if statement represents the case where the second
    // node occurs at aT.
    if ((i == 1) && nodeOccursAtGivenTime) {
        setControlValue(aT, 0.0);
        return;
    }

    // HANDLE ALL OTHER CASES
    double dt, dtPrev, xPrev, xPrevPrev;
    // If the time of the node at index i is equal to aT (where
    // "equal" is determined by the isEqual function of the
    // ControlLinearNode class):
    // (i <= 1 cases were handled above)
    if (nodeOccursAtGivenTime) {
        dt = _xNodes[i]->getTime() - _xNodes[i-1]->getTime();
        dtPrev = _xNodes[i-1]->getTime() - _xNodes[i-2]->getTime();
        xPrev = _xNodes[i-1]->getValue();
        xPrevPrev = _xNodes[i-2]->getValue();

    // If the time of the node at index i is less than aT:
    } else {
        dt = aT - _xNodes[i]->getTime();
        dtPrev = _xNodes[i]->getTime() - _xNodes[i-1]->getTime();
        xPrev = _xNodes[i]->getValue();
        xPrevPrev = _xNodes[i-1]->getValue();
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

//_____________________________________________________________________________
double ControlLinear::
Interpolate(double aX1,double aY1,double aX2,double aY2,double aX)
{
    double y;
    double dx = aX2 - aX1;
    if(fabs(dx)<SimTK::Zero) {
        y = aY1;
    } else {
        double dy = aY2 - aY1;
        double m = dy / dx;
        y = aY1 + m*(aX-aX1);
    }
    return(y);
}
