#ifndef OPENSIM_CONTROL_H_
#define OPENSIM_CONTROL_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Control.h                             *
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
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>


//=============================================================================
//=============================================================================
namespace OpenSim { 

template <class T> class Array;

/**
 * A class that represents a control in a dynamic simulation.
 *
 * This class is intended to be the base class for different types of controls,
 * so many of its methods are virtual.
 *
 * In general, a control consists of a set of parameters.  These parameters
 * are used to reconstruct a control curve.  For example, a control may be
 * represented by a constant, a series of step functions, a set of linearly
 * interpolated values, a set of spline control points, coefficients in
 * a Fourier series, etc.
 *
 * Because there is not necessarily a 1-to-1 correspondence between the
 * parameters used to represent a control curve and the value of the
 * control curve, there are two basic ways to access the content of a control:
 * getParameter() gets the value of a parameter, and getValue() gets the
 * value at a particular time.
 *
 * A distinction is also made between controls that control a model and
 * controls that control some other aspect of a simulation.  For example,
 * a control for the excitation level of a muscle is a "model"
 * control.  The value of this type of control is queried during the
 * course of a simulation.  On the other hand, a control for
 * the final time of a simulation is not usually a "model" control.
 * Nor is a control for the initial value of a state variable, even if that
 * state variable is the initial value of a muscle activation.  These
 * "non-model" controls are used to set things before a simulation ever
 * begins and are not queried during the course of a simulation.  The
 * number of model controls can be queried by a call to
 * Model::getNumControls().
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API Control : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(Control, Object);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
    // PROPERTIES
    /** Flag that specifies whether or not this control is a model control. */
    PropertyBool _propIsModelControl;
    /** Flag that specifies whether or not this control should use
    extrapolation for times outside the time range of the control parameters. */
    PropertyBool _propExtrapolate;
    /** Default parameter minimum. */
    PropertyDbl _propDefaultMin;
    /** Default parameter maximum. */
    PropertyDbl _propDefaultMax;
    /** Flat that indicates whether PD follower filter is on. */
    PropertyBool _propFilterOn;

    // REFERENCES TO PROPERTY VALUES
    /** Reference to the value of the IsModelControl property. */
    bool &_isModelControl;
    /** Reference to the value of the Extrapolate flag. */
    bool &_extrapolate;
    /** Reference to the value of the DefaultMin property. */
    double &_defaultMin;
    /** Reference to the value of the DefaultMax property. */
    double &_defaultMax;
    /** Reference to the value of the PropFilterOn property. */
    bool &_filterOn;


//=============================================================================
// METHODS
//=============================================================================
public:
    Control(const char *aName="UNKNOWN");
    Control(const Control &aControl);
    virtual ~Control();

private:
    void setNull();
protected:
    virtual void setupProperties();
    /**
     * Copy the member variables of the specified Control over to this Control
     * instance.
     */
    void copyData(const Control &aControl);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    Control& operator=(const Control &aControl);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    /**
     * Sets whether or not this control is a model control.  A model control is
     * a control that is expected by a model. Controls that are not model
     * controls may be, for example, controls that are used to set up a
     * simulation.  Such examples might include an initial state of a model
     * (e.g., joint angle, joint angular velocity, ...) or the final time of
     * a simulation.
     *
     * @param aTrueFalse If true, the control is treated as a model control.
     * If false, the control is not treated as a model control.
     */
    void setIsModelControl(bool aTrueFalse);
    /// @see setIsModelControl()
    bool getIsModelControl() const;
    /**
     * Sets whether or not to extrapolate for control curve evaluations that
     * are outside the region of confidence for a control.
     *
     * @param aTrueFalse If true, extrapolate when needed and possible to
     * determine the value of the control curve.
     */
    void setExtrapolate(bool aTrueFalse);
    /// @see setExtrapolate()
    bool getExtrapolate() const;

    /**
     * Sets whether or not to apply a PD (proportional-derivative)
     * filter to the control values.
     *
     * @param aTrueFalse If true, will apply a filter to the control
     * values.  If false, a filter will not be used.
     */
    void setFilterOn(bool aTrueFalse);
    /// @see setFilterOn()
    bool getFilterOn() const;
    // PARAMETERS
    /**
     * Returns the number of parameters that are used to specify the
     * control curve.
     */
    virtual int getNumParameters() const = 0;
    // Default Parameter Min
    /**
     * Sets the default minimum value of a control parameter.
     * The default minimum is used when no minimum value is specified.
     *
     * @param aMin Minimum value.
     */
    void setDefaultParameterMin(double aMin);
    /// @see setDefaultParameterMin()
    double getDefaultParameterMin() const;
    // Default Parameter Max
    /**
     * Sets the default maximum value of a control parameter.
     * The default maximum is used when no maximum value is specified.
     *
     * @param aMax Maximum value.
     */
    void setDefaultParameterMax(double aMax);
    /// @see setDefaultParameterMax()
    double getDefaultParameterMax() const;
    // Parameter Min
    /**
     * Sets the minimum value that a control parameter  can take on.
     *
     * @param aI Index of the parameter.
     * @param aMin Minimum value the parameter can have.
     * @throws Exception if aI is invalid.
     */
    virtual void setParameterMin(int aI,double aMin) = 0;
    /**
      * @see setParameterMin()
      * @param aI Index of the parameter for which the minimum value is desired.
      * @throws Exception if aI is invalid.
      */
    virtual double getParameterMin(int aI) const = 0;
    // Parameter Max
    /**
     * %Set the maximum value that a control parameter can take on.
     *
     * @param aI Index of the parameter.
     * @param aMax Maximum value the parameter can have.
     * @throws Exception if aI is invalid.
     */
    virtual void setParameterMax(int aI,double aMax) = 0;
    /**
     * @see setParameterMax()
     * @param aI Index of the parameter.
     * @throws Exception if aI is invalid.
     */
    virtual double getParameterMax(int aI) const = 0;
    // Parameter Neighborhood
    /**
     * Gets the time at which a parameter is specified.
     *
     * Parameters for some types of control curves do not have a time at which
     * they are specified.  For example, in a Fourier series the control
     * parameters are the coefficients in the expansion, and each term in
     * the expansion corresponds not to a specific time but to a frequency.
     * Another example is a constant that has the same value for all times.
     * In these cases, this method returns SimTK::NaN.
     *
     * @param aI Index of the parameter.
     * @throws Exception if aI is invalid.
     */
    virtual double
        getParameterTime(int aI) const = 0;
    /**
     * Gets the time neighborhood (i.e., the lower and upper bounds of time)
     * in which a control parameter affects the value of the control curve.
     *
     * Changes in the specified parameter are guaranteed not to change the value
     * of the control curve below the lower bound time or above the upper bound
     * time.  If a parameter influences the value of the control curve for all
     * times, -%SimTK::Infinity and %SimTK::Infinity are returned for
     * the upper and lower bound times, respectively.
     *
     * @param aI Index of the parameter.
     * @param rTLower Time below which the curve is not affected by the
     * specified parameter.
     * @param rTUpper Time above which the curve is not affected by the
     * specified parameter.
     * @throws Exception if aI is invalid.
     */
    virtual void
        getParameterNeighborhood(int aI,double &rTLower,double &rTUpper) const=0;
    /**
     * Gets the list of parameters that affect the control curve at a
     * specified time.
     *
     * @param aT Time in question.
     * @param rList The returned list of parameters.
     * @return Length of rList.
     */
    virtual int
        getParameterList(double aT,Array<int> &rList) = 0;
    /**
     * Gets the list of parameters that affect the control curve between two
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
     * @param aTLower Lower time bound.
     * @param aTUpper Upper time bound.
     * @param rList List of indices of the control parameters that
     * affect the curve between aTLower and aTUpper but not before aTLower.
     * @return Length of rList.
     */
    virtual int
        getParameterList(double aTLower,double aTUpper,Array<int> &rList) = 0;

    // Parameter Value
    /// @see setParameterValue()
    virtual double getParameterValue(int aI) const = 0;
    /**
     * Sets the value of a control parameter.
     *
     * @param aI Index of the parameter.
     * @param aX Value of the parameter. Meaning depends on the subclass.
     * @throws Exception if aI is invalid.
     */
    virtual void setParameterValue(int aI,double aX) = 0;

    // Control Value
    /**
     * Gets the value of this control at time aT.
     * If the value of the curve is not defined,
     * SimTK::NaN is returned.  If the control is set to extrapolate,
     * (see getExtrapolate()), and the time is before that of the first node or
     * after that of the last node, then an extrapolation is performed to
     * determine the value of the control curve.  Otherwise, the value of
     * either the first control node or last control node is returned.
     *
     * @param aT Time at which to get the control.
     */
    virtual double getControlValue(double aT=0.0) = 0;
    /**
     * Sets the value of this control curve at time aT.
     *
     * @param aT Time at which to set the control.
     * @param aX Control value.
     */
    virtual void setControlValue(double aT,double aX) = 0;
    /**
     * Gets the minimum allowed value of this control at time aT.
     *
     * @param aT Time at which to get the control.
     * @return Minimum allowed control value.  If the value of the curve
     * is not defined,
     * _defaultMin is returned.  If the control is set to extrapolate,
     * (see getExtraplate()), and the time is before the first node or
     * after the last node, then an extrapolation is performed to determine
     * the value of the control curve.  Otherwise, the value of either the
     * first control node or last control node is returned.
     */
    virtual double getControlValueMin(double aT=0.0) = 0;
    /**
     * Sets the minimum value of this control curve at time aT.
     *
     * @param aT Time at which to set the control.
     * @param aMin Minimum allowed control value at time aT.
     */
    virtual void setControlValueMin(double aT,double aMin) = 0;
    /**
     * Gets the maximum allowed value of this control at time aT.
     *
     * @param aT Time at which to get the control.
     * @return Maximum allowed control value.  If the value of the curve is not defined,
     * _defaultMax is returned.  If the control is set to extrapolate,
     * getExtraplate, and the time is before the first node or
     * after the last node, then an extrapolation is performed to determine
     * the value of the control curve.  Otherwise, the value of either the
     * first control node or last control node is returned.
     */
    virtual double getControlValueMax(double aT=0.0) = 0;
    /**
     * Sets the maximum value of this control curve at time aT.
     *
     * @param aT Time at which to set the control.
     * @param aMax Maximum allowed control value.
     */
    virtual void setControlValueMax(double aT,double aMax) = 0;
    // Convenience methods to get first and last time.
    /**
     * Gets the first time for which a parameter is specified. Should be
     * overridden by derived classes that have a defined min time.
     *
     * @return 0.
     */
    virtual double getFirstTime() const;
    /**
     * Gets the last time for which a parameter is specified. Should be overridden
     * by derived classes that have a defined max time.
     *
     * @return 0.
     */
    virtual double getLastTime() const;

    // UTILITY
    /**
     * Simplify the control (e.g., reduce the number of points in the control
     * curve) based on a set of specified properties.  Each implementation
     * is free to require whatever properties are needed to perform
     * the simplification.  Refer to the documentation in derived classes
     * to see what properties are required.
     *
     * @param aProperties PropertySet used to perform the simplify
     * operation.
     * @throw Exception This method does nothing.  It must be overridden
     * in derived classes.
     */
    virtual void
        simplify(const PropertySet &aProperties);
    /**
     * Filter the control curve at a particular time.
     *
     * @param aT Time at which to compute a new, filtered control value
     */
    virtual void filter(double aT);

//=============================================================================
};  // END of class Control

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Control_h__
