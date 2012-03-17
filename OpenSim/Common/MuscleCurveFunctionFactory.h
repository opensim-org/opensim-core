#ifndef OPENSIM_MUSCLECURVEFUNCTIONFACTORY_H_
#define OPENSIM_MUSCLECURVEFUNCTIONFACTORY_H_

// MuscleCurveFunctionFactory.h
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
#include "osimCommonDLL.h"

#include "MuscleCurveFunction.h"
#include "QuinticBezierCurveSet.h"

#include "Simbody.h"

#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>


using namespace SimTK;
using namespace std;

namespace OpenSim {

/**
This is a class that acts as a user friendly wrapper to QuinticBezerCurveSet
to build specific kinds of muscle curves using C2 continuous sets of quintic
Bezier curves. In general the functions in this class take parameters
that would be intuitive to biomechanists who simulate human musculoskeletal 
systems, and returns a MuscleCurveFunction which is capable of evaluating the
value, derivatives and optionally the integral of the desired function (or 
actually relation as the case may be). 

Each curve is made up of linearly extrapolated C2 quintic Bezier curves x(u), 
and y(u), as shown in the figure below. These quintic curves span 2 points, and 
achieve the desired derivative at its end points. The degree of curviness can be 
varied from 0 to 1 (0, 0.75 and 1.0 are shown in the figure in grey, blue and 
black respectively), and will make the curve approximate a line when set to 0 
(grey), and approximate a curve that hugs the intersection of the lines that are 
defined by the end points locations and the slopes at the end of each curve 
segment (red lines). Although you do not need to set all of this information 
directly, for some of the curves it is useful to know that both the slope and 
the curviness parameter may need to be altered to achieve the desired shape.


\image html quinticCornerSections.png




*/
class OSIMCOMMON_API MuscleCurveFunctionFactory
//class MuscleCurveFunctionFactory
{


    public:

        


        /**
        This is a function that will produce a C2 (continuous to the second
        derivative) active force length curve. 


        @param lce0   Normalized fiber length at the left-most shoulder of the 
                      active force-length curve. The value of the active force
                      length curve for lce < lce0 will be equal to the value
                      set in shoulderVal. Normally lce0 is 0.5.
        
        @param lce1   Normalized fiber length at the transition point between 
                      the ascending limb and the plateau region of the active 
                      force length curve. Values for lce1 can vary.
        
        @param lce2   Normalized fiber length at the maximum active force length
                      curve value of 1. Normally lce2 is by definition 1.
        
        @param lce3   Normalized fiber length of the at the right most shoulder
                      of the active-force length curve. The value of the active
                      force length curve for lce > lce2 will be equal to the 
                      value of shoulderVal. Normally lce3 is 1.5

        @param shoulderVal    The minimum value of the active force length 
                              curve. A physiological non-equibrium muscle model
                              would have this value set to 0. An equilibrium 
                              muscle model would have a non-zero lower bound on 
                              this value of 0.1 typically. shoulderVal must be 
                              greater than, or equal to 0.
                            
        @param plateauSlope   The slope of the plateau of the active force
                              length curve between lce1 and lce2. This parameter
                              can vary depending on the muscle model, but a 
                              value of 0.5 is a good place to start.

        @param curviness  The dimensionless 'curviness' parameter that 
                          can vary between 0 (a line) to 1 (a smooth, but 
                          sharply bent elbow). A value of 0 will yield an active 
                          force length curve that is composed of slightly curved 
                          line segments. A value of 1 will yield an active force
                          length curve that is smoothly rounded.

        @param computeIntegral If this is true, the integral for this curve
                               is numerically calculated and splined. If false, 
                               this integral is not computed, and a call to 
                               .calcIntegral will throw an exception
        @param mclName The name of the muscle this curve is for
        @return MuscleCurveFunction object

                \image html falCurve.png

        <B>Conditions:</B>
        \verbatim
            0 < lce0 < lce1 < lce2 < lce3 
            shoulderVal >= 0
            plateauSlope > 0
            0 <= curviness <= 0
        \endverbatim

        <B>Example:</B>
        @code
            double lce0 = 0.5;
            double lce1 = 0.75;
            double lce2 = 1;
            double lce3 = 1.5;
            double shoulderVal  = 0.1;
            double plateauSlope = 0.75;
            double curviness    = 0.9;

            MuscleCurveFunction fiberfalCurve = MuscleCurveFunctionFactory::
                createFiberActiveForceLengthCurve(lce0, lce1, lce2, lce3, 
                              shoulderVal, plateauSlope, curviness,false,"test");
            fiberfalCurve.printMuscleCurveToFile();
        @endcode
        

        */
        static MuscleCurveFunction createFiberActiveForceLengthCurve(
            double lce0, double lce1, double lce2, double lce3, 
            double shoulderVal, double plateauSlope, double curviness,
            const bool computeIntegral, const string mclName);   

        /**
        This function will generate a C2 continous (continuous to the second 
        derivative) force velocity curve of a single muscle fiber. The main 
        function of this element is to model the amount the force enhancement or 
        attenuation that is associated with contracting at a particular velocity.
        
        @param fmaxE  The normalized maximum force the fiber can generate when 
                      is being stretched. This value is reported to range 
                      between 1.1 and 1.8 in the literature, though all values
                      are above 1.

        @param dydxC  The slope of the fv(dlce(t)/dt) curve at the maximum 
                      normalized concentric contraction velocity. Although 
                      physiologically the value of dydxC at the maximum 
                      concentric contracton velocity is by definition 0, a value
                      of 0 is rarely used. If you are using an equilbrium type 
                      model this term must be positive and greater than zero so
                      that the fv curve can be inverted.
                      <br /><br />
                      Minimum Value: 0
                      Maximum Value: dydxC < 1 
                      <br /><br />
                      Note that the stiffness (and thus simulation time) of an 
                      eqilibrium-type muscle model is affected by the size of 
                      this term: the closer to zero it is, the stiffer the model 
                      is (but only when (dlce(t)/dt)/vmax approaches -1).
                              
        @param dydxIso  The slope of the fv curve when dlce(t)/dt = 0. 
                        <br /><br />
                        Minimim Value: dydxIso > 1.0
                        Maximum Value: dydxIso < Inf
                        
        @param dydxE    The analogous term of dydxC parameter but for the 
                        eccentric portion of the force-velocity curve. As with
                        the dydxC term, the physiologically accurate value for
                        this parameter is 0, though a value of 0 is rarely used
                        in muscle models.  If you are using an equilbrium type 
                        model this term must be positive and greater than zero 
                        so that the fv curve can be inverted. 
                        <br /><br />
                        Minimum Value: 0
                        Maximum Value: dydxC < (fmaxE-1).
                        <br /><br />
                        As with the dydxC term, 
                        the size of this term also affects the stiffness of the 
                        integration problem for equilibrium-type muscle models: 
                        the closer to zero this term is, the stiffer the model 
                        will be (but only when (dlce(t)/dt)/vmax approaches 1.

        @param concCurviness    The dimensionless 'curviness' parameter that 
                                can vary between 0 (a line) to 1 (a smooth, but 
                                sharply bent elbow). This parameter affects only
                                the concentric side of the fv curve.

        @param eccCurviness     The dimensionless 'curviness' parameter that 
                                can vary between 0 (a line) to 1 (a smooth, but 
                                sharply bent elbow). This parameter affects only 
                                the eccentric side of the fv curve.

        @param computeIntegral  If this is true, the integral for this curve
                                is numerically calculated and splined. If false, 
                                this integral is not computed, and a call to 
                                MuscleCurveFunction::calcIntegral() will throw 
                                an exception

        @param mclName          The name of the muscle this curve is for.
        @return MuscleCurveFunction object
        
                \image html fvCurve.png


        <B>Conditions:</B>
        \verbatim
            0 <= dydxC < 1
            1 < dydxIso
            dydxE < (fmaxE-1) 
            0<= concCurviness <=0
            0 <= eccCurviness <= 0
        \endverbatim

        <B>Explanation</B>
        @code
            double fmaxE = 1.8;
            double dydxC = 0.1;
            double dydxE = 0.1;
            double dydxIso= 5;
            double concCurviness = 0.1;
            double eccCurviness = 0.75;

            MuscleCurveFunction fiberFVCurve = MuscleCurveFunctionFactory::
                createFiberForceVelocityCurve(fmaxE, dydxC, dydxIso, dydxE, 
                                concCurviness,  eccCurviness,false,"test");
            fiberFVCurve.printMuscleCurveToFile();
        @endcode             
        */
        static MuscleCurveFunction createFiberForceVelocityCurve(
            double fmaxE, double dydxC, double dydxIso, double dydxE, 
            double concCurviness, double eccCurviness,
            const bool computeIntegral, const string mclName);

        /**
        This function will generate a C2 continuous (continuous to the 2nd
        derivative) inverse curve that the function 
        createFiberForceVelocityCurve generates. The inverse force velocity 
        curve is required by every equilibrium muscle model in order to compute
        the derivative of fiber velocity. To generate the inverse force velocity
        curve simply call this function with EXACTLY the same parameter values
        that you used to generate the force velocity curve. See the parameter
        descriptions for createFiberForceVelocityCurve, as the parameters for
        the inverse function are identical.

        \image html fvInvCurve.png

        */
        static MuscleCurveFunction createFiberForceVelocityInverseCurve(
            double fmaxE, double dydxC, double dydxIso, double dydxE, 
            double concCurviness, double eccCurviness, 
            const bool computeIntegral, const string mclName);

        /**
        This element will generate a C2 continuous (continuous to the 2nd
        derivative) compressive force profile curve as a function of pennation.
        A muscle model with this element usually places this element parallel to
        the fiber.The main function of this element is to prevent the fiber from 
        achieving a pennation angle of pi/2 radians. This type of element is 
        necessary for a parallelogram pennated equilibrium muscle models because 
        without it, the muscle model can deform to the point where a pennation 
        angle of pi/2 radians is reached, which causes a singularity in the 
        model.


        @param phi0 The pennation angle at which the compressive force element
                    starts to engage . When the pennation angle is greater than 
                    phi0, the compressive element is generating a force. When the 
                    pennation angle is less than phi0, the compressive element 
                    generates no force.

        @param kiso This is the maximum stiffness of the compressive element, 
                    which occurs when the fiber is pennated by 90 degrees

        @param curviness    The dimensionless 'curviness' parameter that 
                            can vary between 0 (a line) to 1 (a smooth, but 
                            sharply bent elbow)

        @param computeIntegral  If this is true, the integral for this curve
                                is numerically calculated and splined. If false, 
                                this integral is not computed, and a call to 
                                .calcIntegral will throw an exception

        @param mclName  : The name of the muscle this curve is for

        @return MuscleCurveFunction object

        \image html fcphiCurve.png

        <B>Conditions:</B>
        \verbatim
            0 < phi0 < SimTK::Pi/2
            kiso > 1/(SimTK::Pi/2-phi0)
            0 <= curviness <= 0
        \endverbatim


        <B>Examples</B>
        @code
            double phi0 = (SimTK::Pi/2)*(8.0/9.0);
            double kiso = 8.389863790885878;
            double c    = 0.0;

            MuscleCurveFunction fiberCEPhiCurve = MuscleCurveFunctionFactory::
                createFiberCompressiveForcePennationCurve(phi0,kiso,c,
                true,"test");
            fiberCEPhiCurve.printMuscleCurveToFile();
        @endcode
        */
        static MuscleCurveFunction 
            createFiberCompressiveForcePennationCurve(
                double phi0, double kiso, double curviness, 
                const bool computeIntegral, const string mclName);

        /**
        This element will generate a C2 continuous (continuous to the 2nd
        derivative) compressive force profile curve as a function of 
        cos(pennation).

        A muscle model with this element usually places this element in line 
        with the tendon. The main function of this element is to prevent the 
        fiber from achieving a pennation angle of pi/2 radians. This type of 
        element is necessary for a parallelogram pennated muscle models because 
        without it, the muscle model can deform to the point where a pennation 
        angle of pi/2 radians is reached, which causes a singularity in the 
        model.

        
        @param  cosPhi0 The cosine of the pennation angle at which the 
                        compressive force element starts to engage. When the 
                        cos of the pennation angle is greater than cosPhi0, the 
                        compressive element generates no force. When cos of the
                        pennation angle is less than cosPhi0, the compressive 
                        element generates a compressive force.

        @param kiso This is the maximum stiffness of the compressive element, 
                    which occurs when cosPhi is zero. This parameter must be
                    negative
                    cos
        @param curviness    The dimensionless 'curviness' parameter that 
                            can vary between 0 (a line) to 1 (a smooth, but 
                            sharply bent elbow)

        @param computeIntegral  If this is true, the integral for this curve
                                is numerically calculated and splined. If false, 
                                this integral is not computed, and a call to 
                                .calcIntegral will throw an exception

        @param mclName  The name of the muscle this curve is for

        @return MuscleCurveFunction object

        \image html fcCosPhiCurve.png


        <B>Conditions:</B>
        \verbatim
            0 < cosPhi0
            kiso > 1/(cosPhi0)
            0 <= curviness <= 0
        \endverbatim


        <B>Examples</B>
        @code
            double cosPhi0 = cos( (80.0/90.0)*SimTK::Pi/2);
            double kiso    = -1.2/(cosPhi0);
            double c       = 0.5;

            MuscleCurveFunction fiberCECosPhiCurve = MuscleCurveFunctionFactory::
                createFiberCompressiveForceCosPennationCurve(cosPhi0,kiso,
                                                           c,true,"test");
            fiberCEPhiCurve.printMuscleCurveToFile();
        @endcode

        

        */
        static MuscleCurveFunction 
            createFiberCompressiveForceCosPennationCurve(
                double cosPhi0, double kiso, double curviness, 
                const bool computeIntegral, const string mclName);


        /**
        This element will generate a C2 continous (continuous to the second 
        derivative) curve that models a compressive force profile that is a 
        function of fiber length. The main function of
        this element is to prevent the fiber from achieving an unrealistically
        short length. This type of element is necessary for equilibrium-type 
        muscle models because of the editing that is done to the active force
        length curve that endows an equilibrium model fiber with the ability to
        to generate force when a physiological fiber cannot.



        @param l0   The normalized fiber length at which the compressive element 
                    starts to engage. When the fiber is shorter than l0, the 
                    compressive element is generating a force. When the fiber 
                    length is longer than l0, the compressive element generates
                    no force.

        @param kiso This is the maximum stiffness of the compressive element, 
                    which occurs when the fiber has a length of 0, under a load 
                    of 1 maximum isometric unit of force.

        @param curviness    The dimensionless 'curviness' parameter that 
                            can vary between 0 (a line) to 1 (a smooth, but 
                            sharply bent elbow)

        @param computeIntegral  If this is true, the integral for this curve
                                is numerically calculated and splined. If false, 
                                this integral is not computed, and a call to 
                                .calcIntegral will throw an exception

        @param mclName  The name of the muscle this curve is for

        @return MuscleCurveFunction object

        \image html fpeCurve.png

        <B>Conditions:</B>
        \verbatim
            e0 > 0
            kiso > 1/(e0)
            0 <= curviness <= 1
        \endverbatim

        <B>Example:</B>
        @code
            double lmax = 0.6;
            double kiso = -8.389863790885878;
            double c    = 0.1;//0.0;

            MuscleCurveFunction fiberCECurve = MuscleCurveFunctionFactory::
                createFiberCompressiveForceLengthCurve(lmax,kiso,c,true,"test");
            fiberCECurve.printMuscleCurveToFile();
        @endcode

        */
        static MuscleCurveFunction 
            createFiberCompressiveForceLengthCurve(double l0, double kiso, 
            double curviness, const bool computeIntegral, const string mclName);

         /**
        This function will generate a C2 continuous curve that fits a fiber's 
        tensile force length curve. 

        @param e0   The fiber strain at which the fiber develops 1 unit of 
                    normalized force (1 maximum isometric force). Note that the 
                    '1' is left off. Thus an e0 of 0.6 means that the fiber 
                    will develop an 1 normalized force unit when it is strained 
                    by 60% of its resting length, or to a normalized length of 
                    1.6

        @param kiso   The normalized stiffness (or slope) of the fiber curve 
                      when the fiber is strained by e0 (or has a length of 1+e0)
                      under a load of 1 maximum isometric unit of force.

        @param curviness    The dimensionless 'curviness' parameter that 
                            can vary between 0 (a line) to 1 (a smooth, but 
                            sharply bent elbow)

        @param computeIntegral  If this is true, the integral for this curve
                                is numerically calculated and splined. If false, 
                                this integral is not computed, and a call to 
                                .calcIntegral will throw an exception

        @param mclName  The name of the muscle this curve is for

        @return MuscleCurveFunction object


        \image html fcLengthCurve.png

        <B>Conditions:</B>
        \verbatim
            e0 > 0
            kiso > 1/e0
            0 <= curviness <= 0
        \endverbatim

        <B>Example:</B>
        @code
            double e0      = 0.6;
            double kiso    = 8.389863790885878;
            double c       = 0.65;

            MuscleCurveFunction fiberFLCurve = MuscleCurveFunctionFactory::
                                              createFiberForceLengthCurve(e0,
                                              kiso, c, true,"test");
            fiberFLCurve.printMuscleCurveToFile();
        @endcode

        */
        static MuscleCurveFunction createFiberForceLengthCurve(double e0, 
                        double kiso, double curviness,
                        const bool computeIntegral, const string mclName);

        /**
        Will generate a C2 continous (continuous to the second derivative) 
        curve in a MuscleFunctionObject object that fits a tendon's tensile 
        force length curve. 



        @param e0   The tendon strain at which the tendon develops 1 unit of 
                    normalized force (1 maximum isometric force). Note that the 
                    '1' is left off. Thus an e0 of 0.04 means that the tendon 
                    will develop an 1 normalized force unit when it is strained 
                    by 4% of its resting length, or to a normalized length of 
                    1.04

        @param kiso     The normalized stiffness (or slope) of the tendon curve 
                        when the tendon is strained by e0 
                        (or has a length of 1+e0) under a load of 1 maximum
                        isometric unit of force.

        @param curviness    The dimensionless 'curviness' parameter that 
                            can vary between 0 (a line) to 1 (a smooth, but 
                            sharply bent elbow)

        @param computeIntegral  If this is true, the integral for this curve
                                is numerically calculated and splined. If false, 
                                this integral is not computed, and a call to 
                                .calcIntegral will throw an exception

        @param mclname  The name of the muscle this curve is for

        @return MuscleCurveFunction

        \image html fseCurve.png

        <B>Conditions:</B>
        \verbatim
            e0 > 0
            kiso > 1/e0
            0 <= curviness <= 0
        \endverbatim


        <B>Example:</B>
        @code
            double e0   = 0.04;
            double kiso = 42.79679348815859;
            double c    = 0.75;
    
            MuscleCurveFunction tendonCurve = MuscleCurveFunctionFactory::
                                                createTendonForceLengthCurve(
                                                  e0,kiso,c,true,"test");
            tendonCurve.printMuscleCurveToFile();  
        @endcode

        
        */
        static MuscleCurveFunction 
           createTendonForceLengthCurve(double e0,double kiso,double curviness,
           const bool computeIntegral, const string mclname);

    private:
        /**
        This scales the users value of curviness to be between [0+delta, 1-delta]
        because if curviness is allowed to equal 0 or 1, the second derivative 
        becomes quite violent and the resulting curve is difficult to fit 
        splines to.

        @param curviness
        @retval a scaled version of curviness

        */
        static double scaleCurviness(double curviness);


};

}

#endif //OPENSIM_MUSCLECURVEFUNCTIONFACTORY_H_