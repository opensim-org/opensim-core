#ifndef OPENSIM_CONTROL_CONSTANT_H_
#define OPENSIM_CONTROL_CONSTANT_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ControlConstant.h                         *
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
#include "Control.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * A class that represents a constant control curve.  That is, the value
 * of the control curve is the same at any value of time.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API ControlConstant : public Control {
OpenSim_DECLARE_CONCRETE_OBJECT(ControlConstant, Control);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(value, double,
		"The constant control value.");



//=============================================================================
// METHODS
//=============================================================================
public:
    /**
     * @param aX Constant value of the control.
     * @param aName Name of the control.
     */
    ControlConstant(double value=0.0, const char *aName="UnspecifiedName");
	

private:
    //--------------------------------------------------------------------------
	// Implement ModelComponent interface.
	//--------------------------------------------------------------------------
	void setNull();
	void constructProperties();
	

public:
	//--------------------------------------------------------------------------
	// Implement Control interface.
	//--------------------------------------------------------------------------
    int getNumParameters() const OVERRIDE_11; 

	// Min
	void setParameterMin(int aI,double aMin) OVERRIDE_11;
	double getParameterMin(int aI) const OVERRIDE_11;
	// Max
	void setParameterMax(int aI,double aMax) OVERRIDE_11;
	double getParameterMax(int aI) const OVERRIDE_11;
	// Time and Neighborhood
    /**
     * For ControlConstant, parameters are not associated with any specific time.
     *
     * @param aI Index of the parameter.
     * @return SimTK::NaN
     */
    double getParameterTime(int aI) const OVERRIDE_11;
    /**
     * @param aI Index of the parameter.
     * @param rTLower -SimTK::Infinity
     * @param rTUpper SimTK::Infinity
     */
    void getParameterNeighborhood(int aI,double &rTLower,double &rTUpper) const OVERRIDE_11;

    /**
     * @param rList Parameter at index 0 (i.e., the value of the constant)
     * is the only parameter on the list.
     */
    int getParameterList(double aT,Array<int> &rList) OVERRIDE_11;
    int getParameterList(double aT1,double aT2,Array<int> &rList) OVERRIDE_11;

    /**
     * @param aI Only 0 is valid for ControlConstant.
     * @param aX The constant value of this control curve.
     */
    void setParameterValue(int aI,double aP) OVERRIDE_11;
    /**
     * @see setParameterValue()
     * @param aI Only 0 is valid for ControlConstant.
     * @return The constant value of this control curve.
     */
    double getParameterValue(int aI) const OVERRIDE_11;

    /**
     * @param aT Not used since the control value is constant in time.
     * @param aX Control value.
     */
    void setControlValue(double aT,double aX) OVERRIDE_11;
    /**
     * @param aT Not used since the control value is constant in time.
     */
    double getControlValue(double aT) OVERRIDE_11;
	double getControlValueMin(double aT=0.0) OVERRIDE_11;
	void setControlValueMin(double aT,double aX) OVERRIDE_11;
	double getControlValueMax(double aT=0.0) OVERRIDE_11;
	void setControlValueMax(double aT,double aX) OVERRIDE_11;

//=============================================================================
};	// END of class ControlConstant

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_CONTROL_CONSTANT_H_
