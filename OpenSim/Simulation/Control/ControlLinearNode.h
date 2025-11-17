#ifndef OPENSIM_CONTROL_LINEAR_NODE_H_
#define OPENSIM_CONTROL_LINEAR_NODE_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ControlLinearNode.h                        *
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
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyDbl.h>

//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * A control node used to reconstruct a piecewise linear control.
 *
 * The member variables consist of a time, a value, a minimum value, and
 * a maximum value.  So that an Array<T> can be instantiated for
 * ControlLinearNode, this class implements a default constructor, a copy
 * constructor, the assignment operator (=), the equality operator (==),
 * and the less than operator (<).  The time at which a control node
 * occurs is used to determine the results of the operators == and <.
 *
 * @author Frank C. Anderson
 * @version 1.0
 * @see ControlLinear
 */
class OSIMSIMULATION_API ControlLinearNode : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(ControlLinearNode, Object);

//=============================================================================
// MEMBER DATA
//=============================================================================
public:
    /** Equality tolerance. */
    //static double _EqualityTolerance;

protected:
    // PROPERTIES
    /** Time at which the node occurs. */
    PropertyDbl _propT;
    /** Value of the node (may represent control value or min or max bounds, depending on which curve it's in). */
    PropertyDbl _propValue;

    // REFERENCES
    /** Reference to the value of the T property. */
    double &_t;
    /** Reference to the value of the X property. */
    double &_value;

//=============================================================================
// METHODS
//=============================================================================
public:
    ControlLinearNode(double aT=0.0,double aValue=0.0);
    ControlLinearNode(const ControlLinearNode &aNode);

    virtual ~ControlLinearNode();
private:
    void setNull();
    void setupProperties();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    ControlLinearNode& operator=(const ControlLinearNode &aControl);

    friend std::ostream& operator<<(std::ostream &aOut,
        const ControlLinearNode &aControlLinearNode);
#endif
    //--------------------------------------------------------------------------
    // SET AND GET
    //--------------------------------------------------------------------------
    //static void SetEqualityTolerance(double aTol);
    //static double GetEqualityTolerance();
    void setTime(double aT);
    double getTime() const;
    void setValue(double aValue);
    double getValue() const;

    //--------------------------------------------------------------------------
    // UTILITY
    //--------------------------------------------------------------------------
    bool isEqual(const ControlLinearNode &aControl) const;
    bool isLessThan(const ControlLinearNode &aControl) const;
    char* toString();

//=============================================================================
};  // END of class ControlLinearNode

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_CONTROL_LINEAR_NODE_H_
