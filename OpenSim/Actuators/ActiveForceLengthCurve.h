#ifndef OPENSIM_ACTIVE_FORCE_LENGTH_CURVE_H_
#define OPENSIM_ACTIVE_FORCE_LENGTH_CURVE_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  ActiveForceLengthCurve.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
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

// INCLUDE
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>
#include <OpenSim/Common/SmoothSegmentedFunction.h>

#include <Simbody.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {

//==============================================================================
//                        ACTIVE FORCE LENGTH CURVE
//==============================================================================

/**
 This class serves as a serializable ActiveForceLengthCurve, for use in 
 muscle models. 
 

  @author Matt Millard

 */
class OSIMACTUATORS_API ActiveForceLengthCurve : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(ActiveForceLengthCurve, ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(min_norm_active_fiber_length, double,
        "normalized fiber length which the steep ascending limb starts");

    OpenSim_DECLARE_PROPERTY(transition_norm_fiber_length, double, 
        "normalized fiber length which the steep ascending limb transitions"
        " to the shallow ascending limb");

    OpenSim_DECLARE_PROPERTY(max_norm_active_fiber_length, double, 
        "normalized fiber length which the descending limb ends");

    OpenSim_DECLARE_PROPERTY(shallow_ascending_slope, double, 
        "slope of the shallow ascending limb");

    OpenSim_DECLARE_PROPERTY(minimum_value, double,
        "minimum value of the active force length curve");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor creates an curve with the default property values, 
    and assigns it a default name **/
    ActiveForceLengthCurve();

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    /**
     Constructs a C2 continuous active force length curve. The active force 
     length curve requries 5 different properties and a name in order to 
     construct a curve:

     @param minActiveNormFiberLength 
                The normalized fiber length where the steep ascending limb of 
                the active force length curve transitions to the minimumValue 
                and has a first and second derivative of 0.

     @param transitionNormFiberLength   
                The normalized fiber length where the steep ascending limb 
                transitions to the shallow ascending limb.

     @param maxActiveNormFiberLength   
                The normalized fiber length where the descending limb 
                transitions to the minimum value and has a first and second 
                derivative of 0.
            
     @param shallowAscendingSlope
                The slope of the shallow ascending limb.
 
     @param minValue
                The minimum value of the active force length curve. Note that 
                if you are using an equilibrium model that this value must be 
                greater than 0, as a value of 0 will cause a singularity in the 
                muscle dynamic equations.

    @param muscleName
                The name of the muscle this curve belongs to. This name is used
                to create the name of this curve, which is formed simply by 
                appending "_ActiveForceLengthCurve" to the string in muscleName.
                This name is used for making intelligible error messages and 
                also for naming the XML version of this curve when it is 
                serialized.

      \image html fig_ActiveForceLengthCurve.png
        
    <B>Conditions:</B>
    \verbatim
        0 < minActiveFiberLength < transitionFiberLength < 1 < maxActiveFiberLength 
        0 <= shallowAscendingSlope < 1/(1-transitionFiberLength) 
        0 <= minimumValue 
    \endverbatim
    
    <B>Computational Costs</B>
    \verbatim 
        ~20,500 flops
    \endverbatim    

    <B> Default Parameter Values </B>
    
    The default parameters have been chosen so that the resulting curve
    closely matches the active force length curve for human sarcomeres as 
    documented by Nigg and Herzog. The descending limb has been adjusted to 
    match the in-vitro human fiber data reported by Gollapudi and Lin. The
    default shoulder value is set to a rather high value of 0.1. This relatively
    large value is in place to ensure that muscle model dynamic equations with
    an active-force-length singularity do not take an unreasonable amount of 
    time to simulate (simulation time grows inversely as the value of the active 
    force length curve approaches 0). Muscle model formulations that do not have
    this singularity (for example the Millard2012AccelerationMuscle) do, and 
    should set the `minValue' to be 0.
    
    \verbatim
        minActiveNormFiberLength ....... 0.4441
        transitionNormFiberLength ...... 0.6259
        maxActiveNormFiberLength ....... 1.8123
        shallowAscendingSlope .......... 0.8616
        minValue ....................... 0.1
    \endverbatim    

    <B> Example </B>
    @code
        ActiveForceLengthCurve falCurve3(0.5, 0.75,1.5,0.75,0.01,"soleus");
        double falVal  = falCurve3.calcValue(1.0);
        double dfalVal = falCurve3.calcDerivative(1.0,1);
    @endcode

    Note that this object should be updated through the set methods provided. 
    These set methods will take care of rebuilding the curve correctly. If you
    modify the properties directly, the curve will not be rebuilt, and upon
    calling a function like calcValue, calcDerivative, or printCurveToCSVFile
    an exception will be thrown because the curve is out of date with its 
    properties.

    <B>References</B>
    \verbatim
    Gollapudi, S.K., and Lin, D.C.(2009) Experimental determination of sarcomere
    force-length relationship in type-1 human skeletal muscle fibers. 
    J.Biomechanics, 42, 2011-2016. 

    Nigg, BM., and Herzog, W. Biomechanics of the Musculo-skeletal System. 
    Wiley 1994. ISBN 0 471 94444 0
    \endverbatim

    */
    ActiveForceLengthCurve( double minActiveNormFiberLength, 
                            double transitionNormFiberLength,
                            double maxActiveNormFiberLength,
                            double shallowAscendingSlope,
                            double minValue,
                            const std::string& muscleName);

    // uses default destructor, copy constructor, copy assignment

    /**
    @returns the normalized fiber length where the active force length 
            steep ascending limb transitions to the minimum activation value and
            simultaneously achives a first and second derivative of 0
    */
    double getMinActiveFiberLength() const;
    
    /**
    @returns the normalized fiber length where the steep ascending limb 
            transitions to the shallow ascending limb of the active force 
            length curve.
    */
    double getTransitionFiberLength() const;

    /**
    @returns the normalized fiber length where the active force length 
            descending limb transitions to the minimum activation value and
            simultaneously achives a first and second derivative of 0
    */
    double getMaxActiveFiberLength() const;

    /**
    @returns the slope of the shallow ascending limb 
            (d active_force_length/d normalized_fiber_length)
    */
    double getShallowAscendingSlope() const;

    /**
    @returns the minimum active force length value permitted in the simulation.
            This value must be non-zero for an equilibrium model.
    */
    double getMinValue() const;
  

    /**
        @param minActiveNormFiberLength 
            The normalized fiber length where the steep ascending limb of 
            the active force length curve transitions to the minimumValue 
            and has a first and second derivative of 0. 
        @param transitionNormFiberLength   
            The normalized fiber length where the steep ascending limb 
            transitions to the shallow ascending limb. 
        @param maxActiveNormFiberLength   
            The normalized fiber length where the descending limb 
            transitions to the minimum value and has a first and second 
            derivative of 0.
       param shallowAscendingSlope
                The slope of the shallow ascending limb.

        <B>Conditions</B>
        \verbatim
        0 < minActiveFiberLength < transitionFiberLength < 1 < maxActiveFiberLength
        0 <= shallowAscendingSlope < 1/(1-transitionFiberLength) 
        \endverbatim

        <B>Computational Cost</B>
        Curve is rebuilt at a cost of ~20,500 flops. 

    */
    void setActiveFiberLengths( double minActiveNormFiberLength,
                                double transitionNormFiberLength,
                                double maxActiveNormFiberLength,
                                double shallowAscendingSlope);


    /**
    @param minValue
                The minimum value of the active force length curve. Note that 
                if you are using an equilibrium model that this value must be 
                greater than 0, as a value of 0 will cause a singularity in the 
                muscle dynamic equations.

    <B>Computational Costs</B>
        Curve is rebuilt at a cost of ~20,500 flops. 
    */
    void setMinValue(double minValue);

    


    /**
    Calculates the value of the curve evaluated at 'normFiberLength'.

    @param normFiberLength : the normalized length of the muscle fiber
    @return the value of the active force length curve 

    <B>Computational Costs</B>
    \verbatim
        x in curve domain  : ~282 flops
        x in linear section:   ~5 flops
    \endverbatim

    */
    double calcValue(double normFiberLength) const;

    /**
    Calculates the derivative of the active force length multiplier w.r.t. 
    normalized fiber length. 

    @param normFiberLength : the normalized length of the muscle fiber
    @param order           : the order of the derivative. Only values of 0,1 and
                             2 are acceptable.
    @return the derivative active force length curve w.r.t. normalized 
            fiber length
    
    <B>Computational Costs</B>       
    \verbatim
        x in curve domain  : ~391 flops
        x in linear section:   ~2 flops       
    \endverbatim

    */
    double calcDerivative(double normFiberLength, int order) const;

    /**
       This function returns a SimTK::Vec2 that contains in its 0th element
       the lowest value of the curve domain, and in its 1st element the highest
       value in the curve domain of the curve. Outside of this domain the curve
       is approximated using linear extrapolation. 

       @return The minimum and maximum value of the domain, x, of the curve 
                  y(x). Within this range y(x) is a curve, outside of this range
                  the function y(x) is a C2 (continuous to the second 
                  derivative) linear extrapolation*/
    SimTK::Vec2 getCurveDomain() const;

    /**This function will generate a csv file with a name that matches the 
       curve name (e.g. "bicepfemoris_fiberActiveForceLengthCurve.csv"). This 
       function is not const to permit the curve to be rebuilt if it is out of
       date with its properties.
       
       @param path The full path to the location. Note '/' slashes must be used,
            and do not put a '/' after the last folder.

       The file will contain the following columns:
       
       \verbatim
       Col# 1, 2,     3,       4,  
            x, y, dy/dx, d2y/dx2,
       \endverbatim
       
       The curve will be sampled from its linear extrapolation region
       (the region less than minActiveFiberLength), through 
       the curve, out to the other linear extrapolation region
       (the region greater than maxActiveFiberLength). The width of 
       each linear extrapolation region is 10% of the entire range of x, or 
       0.1*(x1-x0).

       The curve is sampled quite densely: the active force length has
       500+20 rows    

       <B>Computational Costs</B>
       \verbatim
            ~487,000 flops
       \endverbatim

       <B>Example</B>
       To read the csv file with a header in from Matlab, you need to use 
       csvread set so that it will ignore the header row. This is accomplished
       by using the extra two numerical arguments for csvread to tell the 
       function to begin reading from the 1st row, and the 0th index (csvread
       is 0 indexed). This is necessary to skip reading in the text header
       \verbatim
        data=csvread('bicepfemoris_fiberActiveForceLengthCurve.csv',1,0);
       \endverbatim

       */
       void printMuscleCurveToCSVFile(const std::string& path);

       void ensureCurveUpToDate();
//==============================================================================
// PRIVATE
//==============================================================================
private:
    /*
    This object extends the ModelComponent interface so that we can make use
    of the 'addToSystem' function, which we are using to create the active
    force length curve (using SmoothSegmentedFunctionFactory), which is an 
    expensive operation, just once prior to simulation. 
    
    Thus the user is allowed to set the properties of this curve until just 
    before the simulation begins. Just prior to the simulation starts 
    'addToSystem' is called, and then this object will build the 
    SmoothSegmentedFunction that defines the curve the user requested
    */

    ///ModelComponent Interface required function
  	void connectToModel(Model& aModel) OVERRIDE_11;
    ///ModelComponent Interface required function
	void initStateFromProperties(SimTK::State& s) const OVERRIDE_11;
    /**
    ModelComponent is being used for this one function, which is called just
    prior to a simulation beginning. This is the ideal time to actually
    create the active force length curve because

    \li The curve parameters cannot change anymore
    \li This function is only called just prior to simulation, so the expensive
        task of creating the curve will only be done when it is absolutely 
        necessary

    */
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

    ///ModelComponent Interface required function
    void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11 {}
    
    void setNull();
    void constructProperties();

    /**
        This function will take all of the current property values, and if they
        have changed since the last time the curve was built, and build a curve.


        <B>Computational Costs</B>
        \verbatim 
            Curve Construction Costs :   ~20,500 flops
        \endverbatim

    */
    void buildCurve();
    
    SmoothSegmentedFunction   m_curve;
    
};

}

#endif // OPENSIM_ACTIVE_FORCE_LENGTH_CURVE_H_
