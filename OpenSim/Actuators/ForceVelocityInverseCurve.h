#ifndef OPENSIM_FORCE_VELOCITY_INVERSE_CURVE_H_
#define OPENSIM_FORCE_VELOCITY_INVERSE_CURVE_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ForceVelocityInverseCurve.h                    *
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

#include <OpenSim/Actuators/osimActuatorsDLL.h>


// INCLUDE
#include <simbody/internal/common.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>
#include <OpenSim/Common/SmoothSegmentedFunction.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {
//==============================================================================
//                      FORCE VELOCITY INVERSE CURVE
//==============================================================================
/**
 This class serves as a serializable ForceVelocityInverseCurve, for use in 
 equilibrium muscle models. 
 
 \image html fig_ForceVelocityInverseCurveUPDATE.png

 Note that this object should be updated through the set methods provided. 
 These set methods will take care of rebuilding the curve correctly. If you
 modify the properties directly, the curve will not be rebuilt, and upon
 calling a function like calcValue, calcDerivative, or printCurveToCSVFile
 an exception will be thrown because the curve is out of date with its 
 properties.

  @author Matt Millard

 */
class OSIMACTUATORS_API ForceVelocityInverseCurve : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(ForceVelocityInverseCurve, ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(concentric_slope_at_vmax, double,
        "curve slope at the maximum normalized "
        "concentric contraction velocity (norm. velocity of -1)");

    OpenSim_DECLARE_PROPERTY(concentric_slope_near_vmax, double,
        "Slope just prior to the maximum normalized"
        "concentric (shortening) velocity (e.g. -1 < norm velocity < -0.8)");

    OpenSim_DECLARE_PROPERTY(isometric_slope, double, 
        "curve slope at isometric (normalized fiber velocity of 0)");

    OpenSim_DECLARE_PROPERTY(eccentric_slope_at_vmax, double, 
        "curve slope at the maximum normalized "
        "eccentric (lengthening) contraction velocity (norm. velocity of 1)");

    OpenSim_DECLARE_PROPERTY(eccentric_slope_near_vmax, double,
        "Slope just prior to the maximum normalized"
        "eccentric (lengthening) velocity (e.g. 0.8 < norm velocity < 1)");

    OpenSim_DECLARE_PROPERTY(max_eccentric_velocity_force_multiplier, double, 
        "curve value at the maximum normalized "
        "eccentric contraction velocity");

    OpenSim_DECLARE_PROPERTY(concentric_curviness, double,
        "concentric curve bend, from "
        "linear to maximum bend  (0-1)");

    OpenSim_DECLARE_PROPERTY(eccentric_curviness, double,
        "eccentric curve bend, from "
        "linear to maximum bend  (0-1)");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================

    /** Default constructor creates an curve with the default property values,
     and assigns it a default name **/
    ForceVelocityInverseCurve();

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    /**
     Constructs a C2 continuous inverse force velocity curve. The inverse force 
     velocity curve requries the 6 properties and a name to define the curve.
     These properties are IDENTICAL to the ones used to specify the 
     force velocity curve. 
     
     

     @param concentricSlopeAtVmax
                See ForceVelocityCurve for documentation. Note that if a value 
                of 0 is used for this parameter an exception will be thrown 
                when the curve is created.

     @param concentricSlopeNearVmax
                See ForceVelocityCurve for documentation. Note that if a value 
                of 0 is used for this parameter an exception will be thrown 
                when the curve is created. 

     @param isometricSlope   
                See ForceVelocityCurve for documentation. 

     @param eccentricSlopeAtVmax
                See ForceVelocityCurve for documentation. Note that if a value 
                of 0 is used for this parameter an exception will be thrown 
                when the curve is created.

    @param eccentricSlopeNearVmax
                See ForceVelocityCurve for documentation. Note that if a value 
                of 0 is used for this parameter an exception will be thrown 
                when the curve is created.                 

     @param maxEccentricVelocityForceMultiplier
                See ForceVelocityCurve for documentation. 
                
     @param concentricCurviness
                See ForceVelocityCurve for documentation. 

     @param eccentricCurviness
                See ForceVelocityCurve for documentation. 


     @param muscleName
                The name of the muscle this curve belongs to. This name is used
                to create the name of this curve, which is formed simply by 
                appending "_ForceVelocityInverseCurve" to the string in muscleName.
                This name is used for making intelligible error messages and 
                also for naming the XML version of this curve when it is 
                serialized.

      <B>Conditions:</B>
        \verbatim
            1a)  0 < concentricSlopeAtVmax < 1            
             b)  concentricSlopeAtVmax < concentricSlopeNearVmax < 1

            2a) 1 < isometricSlope
            2b) (maxEccentricVelocityForceMultiplier-1)/1 < isometricSlope            
            3)  0 < eccentricSlopeAtVmax < (maxEccentricVelocityForceMultiplier-1)/1

            4) 1 < maxEccentricVelocityForceMultiplier 

            5) 0 <= concentricCurviness <= 1            
            6) 0 <= eccentricCurviness <= 1
        \endverbatim

        <B>Computational Costs</B>
        \verbatim 
            ~8,200 flops
        \endverbatim

        <B> Default Parameter Values </B>
        \verbatim
            concentricSlopeAtVmax ...................  = 0.1 
            concentricSlopeNearVmax .................  = 0.1
            isometricSlope        ...................  = 5
            eccentricSlopeAtVmax  ...................  = 0.1
            maxEccentricVelocityForceMultiplier  ...  = 1.8
            concentricCurviness   ..................  = 0.1
            eccentricCurviness    ..................  = 0.75
        \endverbatim

    <B> Example </B>
    @code
        ForceVelocityInverseCurve fvInvCurve3(0.1,0.1,5,0.1,1.8,0.1,0.75,"soleus");
        double fvInvVal  = fvInvCurve3.calcValue(1.0);
        double dfvInvVal = fvInvCurve3.calcDerivative(1.0,1);
    @endcode

    */
    ForceVelocityInverseCurve(  double concentricSlopeAtVmax, 
                                double concentricSlopeNearVmax,
                                double isometricSlope,
                                double eccentricSlopeAtVmax,
                                double eccentricSlopeNearVmax,
                                double maxEccentricVelocityForceMultiplier,
                                double concentricCurviness,
                                double eccentricCurviness,
                                const std::string& muscleName);


    /**
    @returns    The slope of the force velocity curve at the maximum
                normalized contraction velocity (-1). 
    */
     double getConcentricSlopeAtVmax() const;

    /**
    @returns    The slope of the force velocity curve close to the maximum
                normalized contraction velocity. 
    */
     double getConcentricSlopeNearVmax() const;

     /**
     @returns   The slope of the force velocity curve at a normalized 
                contraction velocity of 0.
     */
     double getIsometricSlope() const;

     /**
     @returns   The slope of the force velocity curve at the maximum eccentric
                (lengthening) contraction velocity (1).
     */
     double getEccentricSlopeAtVmax() const;

    /**
    @returns    The slope of the force velocity curve close to the maximum
                normalized eccentric contraction velocity. 
    */
     double getEccentricSlopeNearVmax() const;


     /**
     @returns   The value of the force velocity multiplier at the maximum 
                eccentric (lengthening) velocity.
     */
     double getMaxEccentricVelocityForceMultiplier() const;

     /**
     @returns   The value of the curviness of the concentric curve, where 0
                represents a nearly straight line segment, and 1 represents 
                a curve with the maximum bend possible given the 
                concentricMinSlope, and the isometricMaxSlope. 
     */
     double getConcentricCurviness() const;

     /**
     @returns   The value of the curviness of the eccentric curve, where 0
                represents a nearly straight line segment, and 1 represents 
                a curve with the maximum bend possible given the 
                eccentricMinSlope, and the isometricMaxSlope.
     */
     double getEccentricCurviness() const;
     
     /**
     @param aConcentricSlopeAtVmax
        the slope of the force velocity curve at the maximum concentric 
        contraction velocity (1).
     @param aConcentricSlopeNearVmax
        the slope of the force velocity curve near the maximum concentric 
        contraction velocity.
     @param aIsometricSlope 
        the slope of the force velocity curve at a contraction velocity of 0.
     @param aEccentricSlopeAtVmax 
        the slope of the force velocity curve at the maximum eccentric 
        (lengthening) contraction velocity (1).
     @param aEccentricSlopeNearVmax 
        the slope of the force velocity curve near the maximum eccentric 
        (lengthening) contraction velocity.
     @param aMaxForceMultiplier 
        the value of the force velocity curve, or the value of the force 
        velocity multiplier, at the maximum eccentric (lengthening) velocity
     
     <B>Conditions</B>
      \verbatim
            1a)  0 < concentricSlopeAtVmax < 1            
            1b) concentricSlopeAtVmax < concentricSlopeNearVmax < 1
            2a) 1 < isometricSlope
            2b) (maxEccentricVelocityForceMultiplier-1)/1 < isometricSlope            
            3a)  0 < eccentricSlopeAtVmax < (maxEccentricVelocityForceMultiplier-1)/1
            3b)  eccentricSlopeAtVmax < eccentricSlopeNearVmax < (maxEccentricVelocityForceMultiplier-1)/1
            4) 1 < maxEccentricVelocityForceMultiplier 
      \endverbatim

      <B>Computational Cost</B>
      The curve is rebuilt at a cost of ~8,200 flops

     */
     void setCurveShape(double aConcentricSlopeAtVmax,
                        double aConcentricSlopeNearVmax,
                        double aIsometricSlope,
                        double aEccentricSlopeAtVmax,
                        double aEccentricSlopeNearVmax,
                        double aMaxForceMultiplier);

     /**
     @param aConcentricCurviness      
        The value of the curviness of the concentric curve, where 0 represents a 
        nearly straight line segment, and 1 represents a curve with the maximum 
        bend possible given the concentricMinSlope, and the isometricMaxSlope. 

        <B>Computational Cost</B>
      The curve is rebuilt at a cost of ~8,200 flops
     */
     void setConcentricCurviness(double aConcentricCurviness);

     /**
     @param aEccentricCurviness
        The value of the curviness of the eccentric curve, where 0 represents a 
        nearly straight line segment, and 1 represents a curve with the maximum 
        bend possible given the eccentricMinSlope, and the isometricMaxSlope.

        <B>Computational Cost</B>
      The curve is rebuilt at a cost of ~8,200 flops
     */
     void setEccentricCurviness(double aEccentricCurviness);

    /**
    Calculates the value of the curve evaluated at 'aForceVelocityMultiplier'. 

    @param aForceVelocityMultiplier: the force velocity multiplier to evaluate
        the force velocity curve for the corresponding fiber velocity
        

    @return the value of the curve evaluated at aForceVelocityMultiplier

    <B>Computational Costs</B>
    \verbatim
        x in curve domain  : ~282 flops
        x in linear section:   ~5 flops
    \endverbatim

    */
    double calcValue(double aForceVelocityMultiplier) const;

    /**
    Calculates the derivative of the force-velocity inverse curve w.r.t. 
    the force velocity multiplier. 

    @param aForceVelocityMultiplier: the force velocity multiplier to evaluate
        the force velocity curve for the corresponding fiber velocity

    @param order           : the order of the derivative. Only values of 0,1 and
                             2 are acceptable.

    @return the derivative of the force-velocity inverse curve w.r.t. normalized 
            fiber velocity
    
    <B>Computational Costs</B>       
    \verbatim
        x in curve domain  : ~391 flops
        x in linear section:   ~2 flops       
    \endverbatim

    */
    double calcDerivative(double aForceVelocityMultiplier, int order) const;

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
       curve name (e.g. "bicepfemoris_fiberForceVelocityInverseCurve.csv").
       This function is not const to permit the curve to be rebuilt if it is out 
       of date with its properties.
       
       @param path The full path to the location. Note '/' slashes must be used,
            and do not put a '/' after the last folder.

       The file will contain the following columns:
       
       \verbatim
       Col# 1, 2,     3,       4,  
            x, y, dy/dx, d2y/dx2,
       \endverbatim
       
       The curve will be sampled from its linear extrapolation region
       (the region with normalized fiber velocities < -1), through 
       the curve, out to the other linear extrapolation region
       (the region with normalized fiber velocities > 1). The width of 
       each linear extrapolation region is 10% of the entire range of x, or 
       0.1*(x1-x0).

       The curve is sampled quite densely: there are 200+20 rows    

       <B>Computational Costs</B>
       \verbatim
            ~194,800 flops
       \endverbatim

       <B>Example</B>
       To read the csv file with a header in from Matlab, you need to use 
       csvread set so that it will ignore the header row. This is accomplished
       by using the extra two numerical arguments for csvread to tell the 
       function to begin reading from the 1st row, and the 0th index (csvread
       is 0 indexed). This is necessary to skip reading in the text header
       \verbatim
        data=csvread('bicepfemoris_fiberForceVelocityInverseCurve.csv',1,0);
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
    of the 'addToSystem' function, which we are using to create the 
    curve (using SmoothSegmentedFunctionFactory), which is an 
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
    create the curve because

    \li The curve parameters cannot change anymore
    \li This function is only called just prior to simulation, so the expensive
        task of creating the curve will only be done when it is absolutely 
        necessary

    */
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

    ///ModelComponent Interface required function
    void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11 {};
    
    void setNull();
    void constructProperties();

    /**
        This function will take all of the current property values
        and build a curve.


        <B>Computational Costs</B>
        \verbatim 
            Curve Construction Costs :   ~20,500 flops
        \endverbatim

    */
    void buildCurve();
    

    SmoothSegmentedFunction   m_curve;
   
};

}

#endif // OPENSIM_FORCE_VELOCITY_INVERSE_CURVE_H_
