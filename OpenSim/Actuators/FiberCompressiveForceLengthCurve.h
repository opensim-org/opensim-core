#ifndef OPENSIM_FIBER_COMPRESSIVE_FORCE_LENGTH_CURVE_H_
#define OPENSIM_FIBER_COMPRESSIVE_FORCE_LENGTH_CURVE_H_

/* Author: Matthew Millard
/*
 * Permission is hereby granted, free of charge, to any person obtaining a    *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction, including without limitation  *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,   *
 * and/or sell copies of the Software, and to permit persons to whom the      *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,    *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR      *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE  *
 * USE OR OTHER DEALINGS IN THE SOFTWARE.                                     *
 * -------------------------------------------------------------------------- */

// INCLUDE
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <simbody/internal/common.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/MuscleCurveFunctionFactory.h>
#include <OpenSim/Common/MuscleCurveFunction.h>

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
 
  @author Matt Millard

 */
class OSIMACTUATORS_API FiberCompressiveForceLengthCurve : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(FiberCompressiveForceLengthCurve, 
                                ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(norm_length_at_zero_force, double, 
        "Normalized fiber length at zero force");
    OpenSim_DECLARE_PROPERTY(stiffness_at_zero_length, double, 
        "Fiber stiffness at zero length");
    OpenSim_DECLARE_PROPERTY(curviness, double, 
        "Fiber curve bend, from linear to maximum bend (0-1)");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor creates an object with a default name that doesn't
    yet define a curve. **/
    FiberCompressiveForceLengthCurve();

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    /**
     Constructs a C2 continuous compressive fiber force length curve. This curve
     is used in the fiber model as a means of ensuring that the fiber cannot
     generate a tensile force at, nor shorten beyond, some minimum length.
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

      <B>Conditions:</B>
        \verbatim
            normLengthAtZeroForce > 0
            stiffnessAtZeroLength < -1/normLengthAtZeroForce
            0 <= curviness <= 1
        \endverbatim

        <B>Computational Costs</B>
        \verbatim 
            ~174,100 flops
        \endverbatim

    <B> Default Parameter Values </B>

         \verbatim
             normLengthAtZeroForce   = 0.6 
             stiffnessAtZeroLength   = -8.4, 
             curviness               = 0.5)
         \endverbatim

    <B>Example:</B>

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
     double getNormLengthAtZeroForce();

     /**
     @returns   This is the stiffness of the compressive elastic force length
                spring when the fiber reaches a normalized length of 0.  
     */
     double getStiffnessAtZeroLength();

     /**
     @returns   A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure.
     */
     double getCurviness();

     /**
    @param aNormLengthAtZeroForce
            Sets the normalized fiber length at which the compressive element
            begins to engage. Normalized length is defined as 
            length_norm = l/l0, where l is the length of the fiber,
            and l0 is the resting length of the fiber. This length must be 
            greater than 0.

    */
     void setNormLengthAtZeroForce(double aNormLengthAtZeroForce);

     /**
     @param aStiffnessAtZeroLength
            Sets the stiffness of the compressive elastic force length
            spring when the fiber reaches a normalized length of 0. This 
            stiffness must be less than -1/normLengthAtZeroForce
     */
     void setStiffnessAtZeroLength(double aStiffnessAtZeroLength);

     /**
     @param aCurviness  
                A dimensionless parameter between [0-1] that controls how 
                the curve is drawn: 0 will create a curve that is
                very close to a straight line segment while a value of 1 will 
                create a curve that smoothly fills the corner formed by the 
                linear extrapolation of 'stiffnessAtOneNormForce' and the
                x axis as shown in the figure.
     */
     void setCurviness(double aCurviness);

    /**
    Calculates the value of the curve evaluated at the desired normalized fiber
    length. Note that if the curve is out of date it is rebuilt 
    (at a cost of ~20,500 flops). 

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

    /**
    Calculates the derivative of the fiber force length curve w.r.t. 
    to the normalized fiber length. Note that if the curve is out of date it is 
    rebuilt (at a cost of ~20,500 flops).

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

    /**
       This function returns a SimTK::Vec2 that contains in its 0th element
       the lowest value of the curve domain, and in its 1st element the highest
       value in the curve domain of the curve. Outside of this domain the curve
       is approximated using linear extrapolation. Note that  if the curve is 
       out of date is rebuilt (which will cost ~20,500 flops).

       @return The minimum and maximum value of the domain, x, of the curve 
                  y(x). Within this range y(x) is a curve, outside of this range
                  the function y(x) is a C2 (continuous to the second 
                  derivative) linear extrapolation*/
    SimTK::Vec2 getCurveDomain() const;

    /**This function will generate a csv file with a name that matches the 
       curve name (e.g. "bicepfemoris_FiberCompressiveForceLengthCurve.csv");
       Note that  if the curve is out of date is rebuilt 
       (which will cost ~20,500 flops).
       
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
       void printMuscleCurveToCSVFile(const std::string& path) const;

//==============================================================================
// PRIVATE
//==============================================================================
private:
    /*
    This object extends the ModelComponent interface so that we can make use
    of the 'createSystem' function, which we are using to create the 
    curve (using MuscleCurveFunctionFactory), which is an 
    expensive operation, just once prior to simulation. 
    
    Thus the user is allowed to set the properties of this curve until just 
    before the simulation begins. Just prior to the simulation starts 
    'createSystem' is called, and then this object will build the 
    MuscleCurveFunction that defines the curve the user requested
    */

    ///ModelComponent Interface required function
    void setup(Model& aModel) OVERRIDE_11;
    ///ModelComponent Interface required function
    void initState(SimTK::State& s) const OVERRIDE_11;
    /**
    ModelComponent is being used for this one function, which is called just
    prior to a simulation beginning. This is the ideal time to actually
    create the curve because

    \li The curve parameters cannot change anymore
    \li This function is only called just prior to simulation, so the expensive
        task of creating the curve will only be done when it is absolutely 
        necessary

    */
    void createSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

    ///ModelComponent Interface required function
    void setDefaultsFromState(const SimTK::State& state) OVERRIDE_11 {};
    
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
    void buildCurve();

    MuscleCurveFunction   m_curve;
    bool                  m_curveUpToDate;
};

}

#endif // OPENSIM_FIBER_COMPRESSIVE_FORCE_LENGTH_CURVE_H_