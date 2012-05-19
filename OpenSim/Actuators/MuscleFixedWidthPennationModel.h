#ifndef OPENSIM_MUSCLEFIXEDWIDTHPENNATIONMODEL_H_
#define OPENSIM_MUSCLEFIXEDWIDTHPENNATIONMODEL_H_
// MuscleFixedWidthPennationModel.h
// Author: Matthew Millard
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
//#include "OpenSim/Simulation/osimSimulationDLL.h"
#include "Simbody.h"
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Common/Object.h>

namespace OpenSim {

/**
This is a class that acts as a utility class that contains all of the necessary
kinematic equations required to use a fixed width parallelogram pennation model 
to compute the way the muscle fibers deform as the muscle contracts. A 
fixed width parallelogram pennation model makes a number of assumptions:

1. The width of the parallelogram stays constant \n
2. The area of the parallelogram stays constant. \n
3. Fibers are all parallel, and of the same length \n


The parallelogram pennation model achieves a constant area by permitting the 
parallelogram to shear while maintaining a constant height. The constant area
property is intended to mimic the constant volume property of muscle, which 
arises because muscle is nearly all water.

\image html fig_MuscleFixedWidthPennationModel.png

This class has been coded to eliminate duplicate calculations. Thus if a 
function depends on pennationAngle, or cos(pennationAngle), it will request
this quantity rather than re-compute this value itself.

<B>ASSUMPTIONS</B>

1) This pennation model is compatible with muscle models that assume that the 
effect of the pennation of the fiber on the transmission of force from the fiber
to the tendon is multiplicative.  That is

fiberForceAlongTendon = fiberForce * f()

Where 'f()' is the function that relates the pennation of the fibers to how
much force is transmitted from the fiber to the tendon.\n

2) This pennation model assumes that fiber length, and whole muscle length are
known.

@author Matt Millard
@version 0.0

*/
class OSIMACTUATORS_API MuscleFixedWidthPennationModel: public Object{
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleFixedWidthPennationModel, Object);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(optimal_fiber_length, double,
        "optimal or resting fiber length in units of meters");            
    OpenSim_DECLARE_PROPERTY(optimal_pennation_angle, double,
        "pennation angle in radians of a fiber at its optimal length");                     
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /**        
    @param optimalFiberLength       The length of the fiber (meters) at
                                    its optimal length

    @param optimalPennationAngle    The angle of the fiber (radians) when 
                                    it is at its optimal angle

    @param caller       The name of the function calling this one. This 
                        string is used to help the user debug their model
                        when an assertion fails.

    Constructs a ParallelogramPennationModel object, which dictates how the
    movement of the fiber affects its orientation, and how force is 
    transferred from the fiber to the tendon.

    <B>Conditions:</B>
    \verbatim
        0 <  optimalFiberLength
        0 <= optimalPennationAngle < SimTK::Pi/2
    \endverbatim

    <B>Example:</B>
    \code
        double optFibLen = 0.1;
        double optPenAng = SimTK::Pi/4.0;
        string caller = "yourFunctionNameHere";
        MuscleFixedWidthPennationModel fibKin = 
                            MuscleFixedWidthPennationModel(
                                optFibLen, optPenAng, caller);
    \endcode

                                        
    <B>Computational Costs</B>
    \verbatim
    _______________________________________________________________________
                        Trig     Comp.   Div.    Mult.   Add.    Assign.
    _______________________________________________________________________
                        1        2               1
    \endverbatim


    */
    MuscleFixedWidthPennationModel(double optimalFiberLength, 
                                            double optimalPennationAngle,
                                            std::string& caller);
    ///Default constructor. Populates member data with NaN's and other
    ///obviously wrong values
    MuscleFixedWidthPennationModel();

        

    /**
        @returns height of the paralleogram (m)
    */
    double getParallelogramHeight() const;

    /**
        @returns optimal fiber length (m) 
    */
    double getOptimalFiberLength() const;

    /**
        @returns the optimal pennation angle (radians) 
    */
    double getOptimalPennationAngle() const;

    /**

    @param fiberLength  The length of the fiber (meters)
    @param cosPennationAngle The cosine of the pennation angle (unitless)
    @returns The projected length of the fiber along the tendon axis
    */
    double calcFiberLengthAlongTendon(double fiberLength, 
                                        double cosPennationAngle) const;

    /**
    @param fiberLength  The length of the fiber (meters)
    @param fiberVelocity The stretch velocity of the fiber (meters/s)
    @param sinPennationAngle The sine of the pennation angle (unitless)
    @param cosPennationAngle The cosine of the pennation angle (unitless)
    @param pennationAngularVelocity The angular velocity of the pennation
                                    angle.
    */
    double calcFiberVelocityAlongTendon(double fiberLength, 
        double fiberVelocity, double sinPennationAngle, 
        double cosPennationAngle, double pennationAngularVelocity) const;

    /**
    This function calculates the pennation angle of the fiber given its
    current length.

    @param fiberLength  the length of the fiber (meters)  

    @param caller       The name of the function calling this one. This 
                        string is used to help the user debug their model
                        when an assertion fails.
    @returns            the current pennation angle (radians)

    <B>Conditions:</B>
    \verbatim
        0 <  fiberLength 
        parallelogramHeight < fiberLength
    \endverbatim


    <B>Example:</B>
    \code
        double optFibLen = 0.1;
        double optPenAng = SimTK::Pi/4.0;
        string caller = "yourFunctionNameHere";
        MuscleFixedWidthPennationModel fibKin = 
                            MuscleFixedWidthPennationModel(
                                optFibLen, optPenAng, caller);

        double fibLen = optFibLen*2;
        double penAng = fibKin.calcPennationAngle(fibLen,caller);
    \endcode

    <B>Computational Costs</B>
    \verbatim
    _______________________________________________________________________
                        Trig     Comp.   Div.    Mult.   Add.    Assign.
    _______________________________________________________________________
                        1        2       1       1               2
    \endverbatim

    */
    double calcPennationAngle(double fiberLength, std::string& caller) const;
        
    /**
    This function computes the angular velocity of the fiber.
       
    @param tanPennationAngle The result of tan(pennationAngle)

    @param fiberLength      The length of the fiber (meters)

    @param fiberVelocity    The lengthening/shortening velocity of the fiber
                            in (meters/sec)

    @param caller           The name of the function calling this one. This 
                            string is used to help the user debug their model
                            when an assertion fails.

    @returns                 The angular velocity of the fiber (rad/s)

    <B>Conditions:</B>
    \verbatim
        0 <  fiberLength
    \endverbatim

    <B>Example:</B>
    \code
        double optFibLen = 0.1;
        double optPenAng = SimTK::Pi/4.0;
        string caller = "yourFunctionNameHere";
        MuscleFixedWidthPennationModel fibKin = 
                            MuscleFixedWidthPennationModel(
                                optFibLen, optPenAng, caller);

        double fibLen = optFibLen*2;           
        double penAng = fibKin.calcPennationAngle(fibLen,caller);
        double fibVel = optFibLen*3/1.0; //3 fiber lengths / second

        //See constructor example to create fibKin
        double penAngVel = fibKin.calcPennationAngularVelocity(
                                        tan(penAng),fibLen,fibVel,caller);
    \endcode

    <B>Computational Costs</B>
    \verbatim
    _______________________________________________________________________
                        Trig     Comp.   Div.    Mult.   Add.    Assign.
    _______________________________________________________________________
                                    1      1       1               1         
    \endverbatim

    */
    double calcPennationAngularVelocity(double tanPennationAngle,
                                        double fiberLength,  
                                        double fiberVelocity,
                                        std::string &caller) const;

    /**
    This function computes the length of the tendon given the length of 
    the muscle, the length of the fiber, and the current pennation angle.

    @param cosPennationAngle    The cosine of the current pennation angle

    @param fiberLength          The length of the fiber (meters)

    @param muscleLength         The length of the whole muscle (meters)

    @returns length of the tendon (meters)

    <B>Example:</B>
    \code
        double optFibLen = 0.1;
        double optPenAng = SimTK::Pi/4.0;
        string caller = "yourFunctionNameHere";
        MuscleFixedWidthPennationModel fibKin = 
                            MuscleFixedWidthPennationModel(
                                optFibLen, optPenAng, caller);

        double fibLen = optFibLen*2;
        double penAng = fibKin.calcPennationAngle(fibLen,caller);
        double mclLen = optFibLen*3;

        double tdnLen = fibKin.calcTendonLength(cos(penAng),fibLen, mclLen);
    \endcode

    <B>Computational Costs</B>
    \verbatim
    _______________________________________________________________________
                        Trig     Comp.   Div.    Mult.   Add.    Assign.
    _______________________________________________________________________
                                                    1       1        1         
    \endverbatim

    */
    double calcTendonLength(double cosPennationAngle,
                            double fiberLength, 
                            double muscleLength) const;

    /**
    This function computes the lengthening velocity (or stretch velocity) of
    the tendon.

    @param cosPennationAngle       The cosine of the current pennation angle

    @param sinPennationAngle       The sine of the current pennation angle

    @param pennationAngularVelocity The angular velocity of the fiber (rad/s)  

    @param fiberLength      The length of the fiber (meters)

    @param fiberVelocity    The lengthening/shortening velocity of the 
                            fiber (m/s)

    @param muscleVelocity   The lengthening/shortening velocity of the 
                            path the of the muscle (m/s)

    @returns the rate of tendon stretching (m/s)

    <B>Example:</B>
    \code
          
        double optFibLen = 0.1;
        double optPenAng = SimTK::Pi/4.0;
        string caller = "yourFunctionNameHere";
        MuscleFixedWidthPennationModel fibKin = 
                            MuscleFixedWidthPennationModel(
                                optFibLen, optPenAng, caller);

        double fibLen = optFibLen*2;            
        double penAng = fibKin.calcPennationAngle(fibLen,caller);

        double fibVel = optFibLen*3/1.0; //3 fiber lengths / second
        double penAngVel = fibKin.calcPennationAngularVelocity(
                                tan(penAng),fibLen,caller);
        double mclLen = optFibLen*3;
        double mclVel = optFibLen*4/1.0; //4 fiber lengths/sec

            
        double tdnVel = fibKin.calcTendonVelocity(cos(penAng),sin(penAng),
                                            penAngVel,fibLen,fibVel,mclVel);
    \endcode

      

    <B>Computational Costs</B>
    \verbatim
    _______________________________________________________________________
                        Trig     Comp.   Div.    Mult.   Add.    Assign.
    _______________________________________________________________________
                                                    3       2        1         
    \endverbatim
    */
    double calcTendonVelocity(double cosPennationAngle,
                                double sinPennationAngle,
                                double pennationAngularVelocity,
                                double fiberLength,    
                                double fiberVelocity,
                                double muscleVelocity) const;

       

    /**
    The partial derivative of the pennation angle with respect to fiber 
    length.

    @param fiberLength      The length of the fiber (meters)

    @param caller       The name of the function calling this one. This 
                        string is used to help the user debug their model
                        when an assertion fails.

    @returns the partial derivative of fiber angle 
                    w.r.t. fiber length (rad/m)

    <B>Conditions:</B>
    \verbatim
        parallelogramHeight < fiberLength
    \endverbatim

    <B>Example:</B>
    \code
        double optFibLen = 0.1;
        double optPenAng = SimTK::Pi/4.0;
        string caller = "yourFunctionNameHere";
        MuscleFixedWidthPennationModel fibKin = 
                            MuscleFixedWidthPennationModel(
                                optFibLen, optPenAng, caller);
            
        double fibLen = optFibLen*2;
        double DphiDlce = fibKin.calc_DpennationAngle_DfiberLength(fibLen, 
                                                                    caller);
    \endcode

    <B>Computational Costs</B>
    \verbatim
    _______________________________________________________________________
                        Sqrt     Comp.   Div.    Mult.   Add.    Assign.
    _______________________________________________________________________
                        1        2       2       5       1       5
    \endverbatim

    */
    double calc_DpennationAngle_DfiberLength(double fiberLength,                                            
                                            std::string& caller) const;
                
    /**
    The partial derivative of tendon length with respect to fiber length.

    @param fiberLength          The length of the fiber (meters) 

    @param sinPennationAngle    The sin(pennationAngle)

    @param cosPennationAngle    The cos(pennationAngle)

    @param DpennationAngle_DfiberLength The partial derivative of 
                                pennation angle w.r.t. fiber length (rad/m)

    @param caller       The name of the function calling this one. This 
                        string is used to help the user debug their model
                        when an assertion fails.
        
    @returns    the partial derivative of tendon length 
                w.r.t. fiber length in (m/m).

    <B>Conditions:</B>
    \verbatim
        0 <  fiberLength
        parallelogramHeight < fiberLength
    \endverbatim

    <B>Example:</B>
    \code
        double optFibLen = 0.1;
        double optPenAng = SimTK::Pi/4.0;
        string caller = "yourFunctionNameHere";
        MuscleFixedWidthPennationModel fibKin = 
                            MuscleFixedWidthPennationModel(
                                optFibLen, optPenAng, caller);

        double fibLen = optFibLen*2;
        double penAng = fibKin.calcPennationAngle(fibLen,caller);
        double DphiDlce = fibKin.calc_DpennationAngle_DfiberLength(fibLen, 
                                                                    caller);

        double DtdnDlce = fibKin.calc_DtendonLength_DfiberLength(fibLen,
                                    sin(penAng), cos(penAng),DphiDlce,caller);
    \endcode

    <B>Computational Costs</B>
    \verbatim
    _______________________________________________________________________
                                Comp.   Div.    Mult.   Add.    Assign.
    _______________________________________________________________________
                                2               2       1       1
    \endverbatim

    */
    double calc_DtendonLength_DfiberLength(double fiberLength, 
                                        double sinPennationAngle,
                                        double cosPennationAngle,
                                        double DpennationAngle_DfiberLength,                                           
                                        std::string& caller) const;

    double calcFiberLength(  double muscleLength, 
                                double tendonLength, 
                                std::string& caller) const;

    double   calcFiberVelocity( double fiberLength,
                                double cosPennation,
                                double sinPennation,
                                double muscleLength,
                                double tendonLength,
                                double muscleVelocity, 
                                double tendonVelocity,                                                                              
                                std::string& caller) const;


private:
    void setNull();
    void constructProperties();
    void ensurePropertiesSet() const;

    /**The height of the parallelogram (in meters). Since the width is 
    constant, the height of the parallelogram is also constant.*/
    double m_parallelogramHeight;
};
}
#endif //OPENSIM_MUSCLEFIXEDWIDTHPENNATIONMODEL_H_
