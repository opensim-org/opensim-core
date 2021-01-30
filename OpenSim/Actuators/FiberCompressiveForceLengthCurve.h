#ifndef OPENSIM_FIBER_COMPRESSIVE_FORCE_LENGTH_CURVE_H_
#define OPENSIM_FIBER_COMPRESSIVE_FORCE_LENGTH_CURVE_H_
/* -------------------------------------------------------------------------- *
 *                OpenSim:  FiberCompressiveForceLengthCurve.h                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/SmoothSegmentedFunction.h>

#ifdef SWIG
    #ifdef OSIMACTUATORS_API
        #undef OSIMACTUATORS_API
        #define OSIMACTUATORS_API
    #endif
#endif

namespace OpenSim {
//==============================================================================
//                   FIBER COMPRESSIVE FORCE LENGTH CURVE
//==============================================================================

/**
 This class serves as a serializable FiberCompressiveForceLengthCurve, 
 which is used to ensure that the fiber cannot generate force at, nor shorten
 beyond a minimum normalized length.
 
 \image html fig_FiberCompressiveForceLengthCurve.png
 
  Note that this object should be updated through the set methods provided. 
 These set methods will take care of rebuilding the curve correctly. If you
 modify the properties directly, the curve will not be rebuilt, and upon
 calling a function like calcValue, calcDerivative, or printCurveToCSVFile
 an exception will be thrown because the curve is out of date with its 
 properties.

  @author Matt Millard

 */
class OSIMACTUATORS_API FiberCompressiveForceLengthCurve : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(FiberCompressiveForceLengthCurve, 
                                Function);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(norm_length_at_zero_force, double, 
        "Normalized fiber length at zero force");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(stiffness_at_zero_length, double, 
        "Fiber stiffness at zero length");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(curviness, double, 
        "Fiber curve bend, from linear to maximum bend (0-1)");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor creates an object with a default name that doesn't
    yet define a curve. **/
    FiberCompressiveForceLengthCurve();

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    /** Constructs a C2 continuous compressive fiber force length curve. This
    curve is used in the fiber model as a means of ensuring that the fiber
    cannot generate a tensile force at, nor shorten beyond, some minimum length.
    Details to appear in Millard et al. 2012.

    @param normLengthAtZeroForce
            The normalized fiber length at which the compressive element
            begins to engage. Normalized length is defined as
            length_norm = l/l0, where l is the length of the fiber,
            and l0 is the resting length of the fiber.

    @param stiffnessAtZeroLength
            This is the stiffness of the compressive elastic force length
            spring when the fiber reaches a normalized length of 0.

    @param curviness
            A dimensionless parameter between [0-1] that controls how
            the curve is drawn: 0 will create a curve that is
            very close to a straight line segment while a value of 1 will
            create a curve that smoothly fills the corner formed by the
            linear extrapolation of 'stiffnessAtZeroLength' and the
            x axis as shown in the figure.

    @param muscleName
            The name of the muscle this curve belongs to. This name is used
            to create the name of this curve, which is formed simply by
            appending "_FiberCompressiveForceLengthCurve" to the string in
            muscleName. This name is used for making intelligible error
            messages and also for naming the XML version of this curve when
            it is serialized.

    <B>Conditions</B>
    \verbatim
        normLengthAtZeroForce > 0
        stiffnessAtZeroLength < -1/normLengthAtZeroForce
        0 <= curviness <= 1
    \endverbatim

    <B>Computational Costs</B>
    \verbatim
        ~174,100 flops
    \endverbatim

    <B>Default Parameter Values</B>

    \verbatim
        normLengthAtZeroForce   = 0.6
        stiffnessAtZeroLength   = -8.4
        curviness               = 0.5
    \endverbatim
    */
    FiberCompressiveForceLengthCurve( double normLengthAtZeroForce, 
                            double stiffnessAtZeroLength,
                            double curviness,
                            const std::string& muscleName);



               
             
    /**
    @returns    The normalized fiber length at which the compressive element
                begins to engage. Normalized length is defined as 
                length_norm = l/l0, where l is the length of the fiber,
                and l0 is the resting length of the fiber.    
    */
     double getNormLengthAtZeroForce() const;

     /**
     @returns   This is the stiffness of the compressive elastic force length
                spring when the fiber reaches a normalized length of 0. If this
                optional property is not specified, a suitable value will be 
                calculated and used instead.
     */
     double getStiffnessAtZeroLengthInUse() const;

     /**
     @returns   A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure. If this optional property
                is not specified, a suitable value will be calculated and used
                instead.
     */
     double getCurvinessInUse() const;

     /**
    @param aNormLengthAtZeroForce
            Sets the normalized fiber length at which the compressive element
            begins to engage. Normalized length is defined as 
            length_norm = l/l0, where l is the length of the fiber,
            and l0 is the resting length of the fiber. This length must be 
            greater than 0.

    <B>Cost </B>
    The curve is rebuilt at a cost of ~174,100 flops
    */
     void setNormLengthAtZeroForce(double aNormLengthAtZeroForce);

     /**
     @param aStiffnessAtZeroLength
            Sets the stiffness of the compressive elastic force length
            spring when the fiber reaches a normalized length of 0. This 
            stiffness must be less than -1/normLengthAtZeroForce
     
     @param aCurviness  
                A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure.
     <B>Cost </B>
     The curve is rebuilt at a cost of ~174,100 flops
     */     
     void setOptionalProperties(double aStiffnessAtZeroLength, 
                                double aCurviness);

     /**
     @returns true if the optional properties are empty and the fitted curve is
              being used. This function returns false if the optional properties
              are filled and are being used to construct the curve.
     */
     bool isFittedCurveBeingUsed() const;


    /**
    Calculates the value of the curve evaluated at the desired normalized fiber
    length.  

    @param aNormLength: 
                The normalized fiber length used to evaluate the fiber 
                compressive force length curve for the corresponding normalized 
                force. Here aNormLength = l/l0, where l is the length of the fiber and 
                l0 is the resting length of the fiber.  Thus normalized length 
                of 1.0 means the fiber is at its resting length.      

    @return the value of the normalized force generated by the fiber

    <B>Computational Costs</B>
    \verbatim
        x in curve domain  : ~282 flops
        x in linear section:   ~5 flops
    \endverbatim

    */
    double calcValue(double aNormLength) const;

 
    /** Implement the generic OpenSim::Function interface **/
    double calcValue(const SimTK::Vector& x) const override
    {
        return calcValue(x[0]);
    }

    /**
    Calculates the derivative of the fiber force length curve w.r.t. 
    to the normalized fiber length. 

    @param aNormLength: 
                The normalized fiber length used to evaluate the compressive 
                fiber force length curve for the corresponding normalized force. Here 
                aNormLength = l/l0, where l is the length of the fiber and l0 
                is the resting length of the fiber.  Thus normalized length of 
                1.0 means the fiber is at its resting length.

    @param order: the order of the derivative. Only values of 0,1 and 2 are 
                  acceptable.

    @return the derivative of the normalized fiber force length curve w.r.t. 
        normalized fiber length

    <B>Computational Costs</B>       
    \verbatim
        x in curve domain  : ~391 flops
        x in linear section:   ~2 flops       
    \endverbatim

    */
    double calcDerivative(double aNormLength, int order) const;

    /// If possible, use the simpler overload above.
    double calcDerivative(const std::vector<int>& derivComponents,
                          const SimTK::Vector& x) const override;

    /**     
    @param aNormLength
                Here aNormLength = l/l0, where l is the length 
                of the fiber and l0 is the resting length of the fiber.  
                Thus normalized length of 1.0 means the fiber is at its 
                resting length.
    
    @return Computes the normalized area under the curve. For this curve, 
            this quantity corresponds to the normalized potential energy stored 
            in the fiber compressive force length spring - simply 
            multiply this quantity by the number of NormForce*NormDistance
            (where NormForce corresponds to the number of
            Newtons that 1 normalized force corresponds to, and NormDistance
            is the distance in meters that a normalized value of 1 corresponds
            to) to obtain the potential energy stored in the fiber in units of 
            Joules.

    <B>Computational Costs</B>    

    \verbatim
        x in curve domain  : ~13 flops
        x in linear section: ~19 flops
    \endverbatim

    */
    double calcIntegral(double aNormLength) const;


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
       curve name (e.g. "bicepfemoris_FiberCompressiveForceLengthCurve.csv");
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
        data=csvread('bicepfemoris_fiberCompressiveForceLengthCurve.csv',1,0);
       \endverbatim

       */
       void printMuscleCurveToCSVFile(const std::string& path);

       void ensureCurveUpToDate();
//==============================================================================
// PRIVATE
//==============================================================================
private:
    /**
    //--------------------------------------------------------------------------
    <B> OpenSim::Function Interface </B>
    //--------------------------------------------------------------------------
    Create the underlying SimTK::Function that implements the calculations
    necessary for this curve. */
    SimTK::Function* createSimTKFunction() const override;
    
    void setNull();
    void constructProperties();

    /**
        This function will take all of the current parameter values and use 
        them to build a curve.

        <B>Computational Costs</B>
        \verbatim 
            Curve Construction Costs :   ~20,500 flops
        \endverbatim

    */
    void buildCurve( bool computeIntegral = false );
    

    SmoothSegmentedFunction   m_curve;
    double m_stiffnessAtZeroLengthInUse;
    double m_curvinessInUse;
    bool m_isFittedCurveBeingUsed;
};

}

#endif // OPENSIM_FIBER_COMPRESSIVE_FORCE_LENGTH_CURVE_H_
