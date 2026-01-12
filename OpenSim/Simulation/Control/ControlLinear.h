#ifndef OPENSIM_CONTROL_LINEAR_H_
#define OPENSIM_CONTROL_LINEAR_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ControlLinear.h                          *
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


// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyObjArray.h>
#include "Control.h"
#include "ControlLinearNode.h"


//=============================================================================
//=============================================================================

namespace OpenSim { 

/**
 * A class that represents a piece-wise linear control curve.
 *
 * The curve is specified by an array of control nodes (see class
 * ControlLinearNode) that occur at monotonically increasing times.
 * The value of the control curve is computed by linearly interpolating
 * the values of the appropriate control nodes.
 *
 * For this Control, <i>parameters</i> are the values of the
 * ControlLinearNode's.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
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
    need to be constructed, but this is too expensive.  It is better to construct
    a node up front, and then just alter the time. */
    ControlLinearNode _searchNode;

//=============================================================================
// METHODS
//=============================================================================
public:
    ControlLinear();
    ControlLinear(const ControlLinear &aControl);
    virtual ~ControlLinear();

    /**
     * Copy the member variables of the specified ControlLinear over
     * to this ControlLinear.
     */
    void copyData(const ControlLinear &aControl);
protected:
    /**
     * Connect properties to local pointers.
     */
    void setupProperties() override;
    
private:
    /**
     * %Set the member data to their NULL values.
     */
    void setNull();
    /**
     * Extrapolate the value of the control curve before the first node.
     *
     * Currently, simple linear extrapolation using the first two nodes is
     * used.
     *
     * @param aT Time at which to evaluate the control curve.
     * @return Extrapolated value of the control curve.
     */
    double extrapolateBefore(double aT) const;
    /**
     * Extrapolate the value of the control curve after the last node.
     *
     * Currently, simple linear extrapolation using the last two nodes is
     * used.
     *
     * @param aT Time at which to evaluate the control curve.
     * @return Extrapolated value of the control curve.
     */
    double extrapolateAfter(double aT) const;
    /**
     * Extrapolate the value of the control curve before the first node.
     *
     * Currently, simple linear extrapolation using the first two nodes is
     * used.
     *
     * @param aT Time at which to evaluate the control curve.
     * @return Extrapolated value of the control curve.
     */
    double extrapolateMinBefore(double aT) const;
    /**
     * Extrapolate the value of the control curve after the last node.
     *
     * Currently, simple linear extrapolation using the last two nodes is
     * used.
     *
     * @param aT Time at which to evaluate the control curve.
     * @return Extrapolated value of the control curve.
     */
    double extrapolateMinAfter(double aT) const;
    /**
     * Extrapolate the value of the control curve before the first node.
     *
     * Currently, simple linear extrapolation using the first two nodes is
     * used.
     *
     * @param aT Time at which to evaluate the control curve.
     * @return Extrapolated value of the control curve.
     */
    double extrapolateMaxBefore(double aT) const;
    /**
     * Extrapolate the value of the control curve after the last node.
     *
     * Currently, simple linear extrapolation using the last two nodes is
     * used.
     *
     * @param aT Time at which to evaluate the control curve.
     * @return Extrapolated value of the control curve.
     */
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
    /**
     * Sets whether or not step functions are used between control nodes or
     * linear interpolation.  When step functions are used, the value of the
     * control curve between two nodes is the value of the node that occurs
     * <b>later</b> in time.
     *
     * @param aTrueFalse If true, step functions will be used to determine the
     * value between adjacent nodes.  If false, linear interpolation will be used.
     */
    void setUseSteps(bool aTrueFalse);

    /**
      * @see setUseSteps()
      */
    bool getUseSteps() const;

    /**
     * Sets the position gain for PD follower filter.  This value is relevant
     * only if the PD follower filter will be used.
     *
     * @see setFilterOn()
     *
     * @param aKp Value of position gain for the PD follower filter.
     */
    void setKp(double aKp);
    /// @see setKp()
    double getKp() const;

    /**
     * Sets the velocity gain for PD follower filter.  This value is relevant
     * only if the PD follower filter will be used.
     *
     * @see setFilterOn()
     *
     * @param aKv Value of velocity gain for the PD follower filter.
     */
    void setKv(double aKv);
    /// @see setKv()
    double getKv() const;
    // PARAMETERS
    int getNumParameters() const override;

    void setParameterMin(int aI,double aMin) override;
    double getParameterMin(int aI) const override;

    void setParameterMax(int aI,double aMax) override;
    double getParameterMax(int aI) const override;
    /**
     * Get the time at which a parameter (control curve value) is specified.
     *
     * Not for minimum or maximum values of parameters; only for specified
     * values of the control curve, as set via setParameterValue() or
     * setControlValue().
     *
     * @param aI Index of the parameter.
     * @throws Exception if aI is invalid.
     */
    double getParameterTime(int aI) const override;
    /**
     * @param aI Index of the parameter.
     * @param rTLower The time of parameter aI-1 or of
     * aI if there is no parameter aI-1.  If there are no ControlLinearNode's
     * at all or if aI is invalid, rTLower is given the value SimTK::NaN.
     * @param rTUpper The time of parameter aI+1 or of
     * aI if there is no parameter aI+1.  If there are no ControlLinearNode's
     * at all or if aI is invalid, rTUpper is given the value SimTK::NaN.
     */
    void getParameterNeighborhood(int aI,double &rTLower,double &rTUpper) const override;

    /**
     * @param aT time
     * @param rList If aT lies between two nodes, the indices of these
     * two nodes are returned; if aT equals the time at which a node occurs, the
     * index of that node is returned; if aT is less than the time of the first
     * node in the array, the index of the first node (i.e., 0) is returned;
     * if aT is greater than the time of the last node, the index of the last
     * node (i.e., size-1) is returned.
     */
    int getParameterList(double aT,Array<int> &rList) override;
    int getParameterList(double aT1,double aT2,Array<int> &rList) override;

    /**
     * @param aI Index of the parameter.
     * @param aP The parameter value is simply the value of
     * the aI-th ControlLinearNode (which is the value of the control curve).
     */
    void setParameterValue(int aI,double aP) override;
    /// @see setParameterValue()
    double getParameterValue(int aI) const override;

    // CONTROL VALUE
    /**
     * This method adds a set of control parameters at the specified time unless
     * the specified time equals the time of an existing ControlLinearNode,
     * in which case the parameters of that control node are changed.
     */
    void setControlValue(double aT,double aX) override;
    double getControlValue(double aT) override;
    double getControlValueMin(double aT=0.0) override;
    /**
     * This method adds a set of control parameters at the specified time unless
     * the specified time equals the time of an existing control node, in which
     * case the parameters of that control node are changed.
     */
    void setControlValueMin(double aT,double aX) override;
    double getControlValueMax(double aT=0.0) override;
    /**
     * This method adds a set of control parameters at the specified time unless
     * the specified time equals the time of an existing control node, in which
     * case the parameters of that control node are changed.
     */
    void setControlValueMax(double aT,double aX) override;
    
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
    /// Called from GUI to work around early garbage collection.
    void insertNewValueNode(int index, const ControlLinearNode& newNode) {
        _xNodes.insert(index, newNode.clone());
    }
    /// Called from GUI to work around early garbage collection.
    void insertNewMinNode(int index, const ControlLinearNode& newNode) {
        _minNodes.insert(index, newNode.clone());
    }
    /// Called from GUI to work around early garbage collection.
    void insertNewMaxNode(int index, const ControlLinearNode& newNode) {
        _maxNodes.insert(index, newNode.clone());
    }
    // Convenience methods
    /**
     * The time corresponding to the first ControlLinearNode.
     */
    double getFirstTime() const override;
    /**
     * The time corresponding to the last ControlLinearNode
     */
    double getLastTime() const override;

    // SIMPLIFY
    /**
     * The number of control nodes is reduced by first applying a lowpass filter
     * to the sequence of control nodes using a specified cutoff frequency and
     * then removing nodes that keep the curve within a specified distance
     * to the low-pass filtered curve.
     *
     * The PropertySet should contain:\n
     * <table>
     * <tr><td>TYPE</td><td>NAME</td></tr>
     * <tr><td>PropertyDbl</td><td>cutoff_frequency</td></tr>
     * <tr><td>PropertyDbl</td><td>distance</td></tr>
     * </table>
     *
     * @param aProperties PropertySet containing the needed properties for
     * this method.
     * @throws Exception if an error is encountered.
     */
    void simplify(const PropertySet &aProperties) override;
    /**
     * Another interface to simplify that:
     * (1) does not require properties, and (2) returns bool on failure
     * for a more graceful batch simplification.
     */
    bool simplify(const double& cutoffFrequency, const double& distance);

    /**
     * Filter the control curve at a particular time using a PD follower filter.
     *
     * @see setFilterOn()
     *
     * @param aT Time at which to compute a new, filtered control value
     */
    void filter(double aT) override;

    /**
     * Linearly interpolate or extrapolate given two points.
     *
     * @param aX1 X coordinate of point 1.
     * @param aY1 Y coordinate of point 1.
     * @param aX2 X coordinate of point 2.
     * @param aY2 Y coordinate of point 2.
     * @param aX X coordinate whose corresponding Y coordinate is desired.
     * @return Y value corresponding to aX.
     */
    static double Interpolate(double aX1,double aY1,double aX2,double aY2,double aX);

private:
    void setControlValue(ArrayPtrs<ControlLinearNode> &aNodes,double aT,double aX);
    double getControlValue(ArrayPtrs<ControlLinearNode> &aNodes,double aT);
    double extrapolateBefore(const ArrayPtrs<ControlLinearNode> &aNodes,double aT) const;
    double extrapolateAfter(ArrayPtrs<ControlLinearNode> &aNodes,double aT) const;
    int searchBinary(const ArrayPtrs<ControlLinearNode>& nodes,
            const ControlLinearNode& value) const;

//=============================================================================
};  // END of class ControlLinear

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_CONTROL_LINEAR_H_
