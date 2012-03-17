#ifndef OPENSIM_QUINTICBEZIERCURVESET_H_
#define OPENSIM_QUINTICBEZIERCURVESET_H_

// QuinticBezierCurveSet.h
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

#include "Simbody.h"
//#include "SimTKcommon/internal/SystemGuts.h"
#include <cstdio>
#include <iostream>
#include <fstream>
//#include <OpenSim\OpenSim.h>
#include <cmath>

using namespace SimTK;
using namespace std;

namespace OpenSim {

/**
This is a low level Quintic Bezier curve class that contains functions to design
continuous sets of 'C' shaped Bezier curves, and to evaluate their values and 
derivatives. A set in this context is used to refer to 2 or more quintic Bezier
curves that are continuously connected to eachother to form one smooth curve,
hence the name QuinticBezierSet.

In the special case when this class is being used to generate and evaluate 
2D Bezier curves, that is x(u) and y(u), there are also functions to evaluate 
y(x), the first six derivatives of y(x), and also the first integral of y(x). 

This class was not designed to be a stand alone Quintic Bezier class, but rather
was developed out of necessity to model muscles. I required curves that, when 
linearly extrapolated, were C2 continuous, and by necessity I had to use
quintic Bezier curves. In addition, the curves I was developing were functions
in x,y space, allowing many of the methods (such as the evaluation of y(x) given
that x(u) and y(u), the derivatives of y(x), and its first integral) to be 
developed, though in general this can't always be done.

I have parcelled all of these tools into their own class so that others may more 
easily use and develop this starting point for their own means. I used the 
following text during the development of this code:

Mortenson, Michael E (2006). Geometric Modeling Third Edition. Industrial Press 
Inc., New York. Chapter 4 was quite helpful.

<B>Future Upgrades</B>

1. Analytical Inverse to x(u). 

I think this is impossible because it is not possible, in general, to find the 
roots to a quintic polynomial, however, this fact may not preclude forming the 
inverse curve. The impossibility of finding the roots to a quintic polynomial
was proven by Abel (Abel's Impossibility Theorem) and Galois

http://mathworld.wolfram.com/QuinticEquation.html

At the moment I am approximating the curve u(x) using cubic splines to return
an approximate value for u(x), which I polish using Newton's method to the
desired precision.

2. Analytical Integral of y(x)

This is possible using the Divergence Theorem applied to 2D curves. A nice 
example application is shown in link 2 for computing the area of a closed 
cubic Bezier curve. While I have been able to get the simple examples to work,
I have failed to successfully compute the area under a quintic Bezier curve
correctly. I ran out of time trying to fix this problem, and so at the present 
time I am numerically computing the integral at a number of knot points and 
then evaluating the spline to compute the integral value.

a. http://en.wikipedia.org/wiki/Divergence_theorem
b. http://objectmix.com/graphics/133553-area-closed-bezier-curve.html

3. calcU

Currently the Bezier curve value and its derivative are computed separately to
evaluate f and df in the Newton iteration to solve for u(x). Code optimized to
compute both of these quantites at the same time could cut the cost of 
evaluating x(u) and dx/du in half. Since these quantities are evaluated in an
iterative loop, this one change could yield substantial computational savings.

4. calcIndex

The function calcIndex computes which spline the contains the point of interest. 
This function has been implemented assuming a small number of Bezier curve sets,
and so it simply linearly scans through the sets to determine the correct one to
use. This function should be upgraded to use the bisection method if large 
quintic Bezier curve sets are desired.

5. The addition of additional Bezier control point design algorithms, to create
'S' shaped curves, and possibly do subdivision.

6. Low level Code Optimization

I have exported all of the low level code as optimized code from Maple. Although
the code produced using this means is reasonably fast, it is usally possible
to obtain superiour performance (and sometimes less round off error) by 
doing this work by hand.

*/
class OSIMCOMMON_API QuinticBezierCurveSet
//class QuinticBezierCurveSet
{


    public:

         /**
        This function will compute the u value that correesponds to the given x
        for a quintic Bezier curve. 

        @param ax         The x value
        @param bezierPtsX The 6 Bezier point values
        @param splineUX   The spline for the approximate x(u) curve
        @param tol        The desired tolerance on u.   
        @param maxIter    The maximum number of Newton iterations allowed
        @param caller     The string name of the parent calling this function

        This function will compute the u value that correesponds to the given x
        for a quintic Bezier curve. This is accomplished by using an approximate 
        spline inverse of u(x) to get a very good initial guess, and then one or 
        two Newton iterations to polish the answer to the desired tolerance.
        
        <B>COMPUTATIONAL COSTS</B>
        \verbatim
        Cost
                    Comparisons     Div     Mult     Additions   Assignments
        splineGuess log(n,2)                2        3           1
        (n currently set to 100)

        Newton Iter
            f                               21       20          13 
            df                              20       19          11
            update  4               1                3           6    
            total   4               1       41       42          30
        \endverbatim

        To evaluate u to SimTK::Eps*100 this typically involves 2 Newton 
        iterations, yielding a total cost of

        \verbatim
                    Comparisons     Div     Mult    Additions   Assignments
        eval U      7+8=15          2       82      42          60
        \endverbatim




        <B>Example:</B>
        @code
            double xVal = 2;

            //Choose the control points
            SimTK::Vector vX(6);
            vX(0) = -2;
            vX(1) = 0;
            vX(2) = 0;
            vX(3) = 4;
            vX(4) = 4;
            vX(5) = 6;

            SimTK::Vector x(100);
            SimTK::Vector u(100);
        
            //Create the splined approximate inverse of u(x)
            for(int i=0; i<100; i++){
                u(i) = ( (double)i )/( (double)(100-1) );
                x(i) = QuinticBezierCurveSet::
                    calcQuinticBezierCurveVal(u(i),vX,"test"); 
            }
            SimTK::Spline splineUX = SimTK::SplineFitter<Real>::
                fitForSmoothingParameter(3,x,u,0).getSpline();

            //Now evalutate u at the given xVal
            double u = QuinticBezierCurveSet::
                     calcU(xVal,vX, splineUX, 1e-12,20,"test");

            @endcode
        */
        static double calcU(double ax, const SimTK::Vector& bezierPtsX, 
            const SimTK::Spline& splineUX, double tol, int maxIter,
            const string caller);



        /**
        Given a set of Bezier curve control points, return the index of the
        set of control points that x lies within.

        @param x            A value that is interpolated by the set of Bezier 
                            curves
        @param bezierPtsX   A matrix of 6xn Bezier control points 
        @param caller       The string name of the parent calling this function

        Given a set of Bezier curve control points, return the index of the
        set of control points that x lies within. This function has been coded
        assuming a small number of Bezier curve sets (less than 10), and so,
        it simply scans through the Bezier curve sets until it finds the correct
        one. 

        <B>COMPUTATIONAL COSTS</B>
        \verbatim
        Cost: n comparisons, for a quintic Bezier curve with n-spline sections

                        Comp    Div     Mult        Add      Assignments
        Cost            3*n+2                       1*n      3                         
        \endverbatim



        <B>Example:</B>
        @code
            SimTK::Matrix mX(6,2);

            //The first set of spline points
                mX(0,0) = -2;
                mX(1,0) = -1;
                mX(2,0) = -1;
                mX(3,0) = 1;
                mX(4,0) = 1;
                mX(5,0) = 2;
            //The second set of spline points
                mX(0,1) = 2;
                mX(1,1) = 3;
                mX(2,1) = 3;
                mX(3,1) = 5;
                mX(4,1) = 5;
                mX(5,1) = 6;

            //The value of x for which we want the index for
            double xVal = 1.75;
            int idx  = QuinticBezierCurveSet::calcIndex(xVal,mX,"test");
        @endcode


        */
        static int calcIndex(double x, const SimTK::Matrix& bezierPtsX,
                                                  const string caller);


        
        

        /**
        Calculates the value of a quintic Bezier curve at value u.

        @param u        The independent variable of a Bezier curve, which ranges 
                        between 0.0 and 1.0.
        @param pts      The locations of the control points in 1 dimension.
        @param caller   The string name of the parent calling this function
        @return         The value of the Bezier curve located at u.

        Calculates the value of a quintic Bezier curve at value u. This 
        calculation is acheived by mulitplying a row vector comprised of powers 
        of u, by the 6x6 coefficient matrix associated with a quintic Bezier 
        curve, by the vector of Bezier control points, pV, in a particular 
        dimension. The code to compute the value of a quintic bezier curve has
        been optimized to have the following cost:

        <B>COMPUTATIONAL COSTS</B>
        \verbatim
        Multiplications     Additions   Assignments
        21                  20          13
        \endverbatim



        The math this function executes is decribed in pseudo code as the 
        following:

        \verbatim
            uV = [u^5 u^4 u^3 u^2 u 1];
        
            cM = [ -1     5   -10    10    -5     1; 
                    5   -20    30   -20     5     0; 
                  -10    30   -30    10     0     0; 
                   10   -20    10     0     0     0; 
                   -5     5     0     0     0     0;
                    1     0     0     0     0     0 ];
            pV = [x1; x2; x3; x4; x5; x6];

            xB = (uV*cM)*pV
        \endverbatim



        <B>Example:</B>
            @code
            double u = 0.5;

            //Choose the control points
            SimTK::Vector vX(6);
            vX(0) = -2;
            vX(1) = 0;
            vX(2) = 0;
            vX(3) = 4;
            vX(4) = 4;
            vX(5) = 6;

            yVal = QuinticBezierCurveSet::
                     calcQuinticBezierCurveVal(u,vX,"test");
            @endcode


        */
        static double calcQuinticBezierCurveVal(double u, 
                                  const SimTK::Vector& pts,const string caller);

        /**
        Calculates the value of a quintic Bezier derivative curve at value u. 
        @param u        The independent variable of a Bezier curve, which ranges 
                        between 0.0 and 1.0.
        @param pts      The locations of the control points in 1 dimension.
        @param order    The desired order of the derivative. Order must be >= 1
        @param caller   The string name of the parent calling this function
        @return         The value of du/dx of Bezier curve located at u.

        Calculates the value of a quintic Bezier derivative curve at value u. 
        This calculation is acheived by taking the derivative of the row vector
        uV and multiplying it by the 6x6 coefficient matrix associated with a 
        quintic Bezier curve, by the vector of Bezier control points, pV, in a 
        particular dimension.

        Pseudo code for the first derivative (order == 1) would be
        \verbatim
            uV = [5*u^4 4*u^3 3*u^2 2u 1 0];
        
            cM = [ -1     5   -10    10    -5     1;
                    5   -20    30   -20     5     0;
                  -10    30   -30    10     0     0;
                   10   -20    10     0     0     0;
                   -5     5     0     0     0     0;
                    1     0     0     0     0     0 ];
            pV = [x1; x2; x3; x4; x5; x6];

            dxdu = (uV*cM)*pV
        \endverbatim

        Note that the derivative of uV only needed to be computed to compute
        dxdu. This process is continued for all 5 derivatives of x(u) until 
        the sixth and all following derivatives, which are 0. Higher derivatives
        w.r.t. to U are less expensive to compute than lower derivatives.

        <B>COMPUTATIONAL COSTS</B>
        \verbatim
                    Divisions   Multiplications Additions   Assignments
            dx/du               20              19          11
            d2x/du2             17              17          9
            d3y/du3             14              14          6
        \endverbatim


        <B>Example:</B>
            @code
            double u = 0.5;

            //Choose the control points
            SimTK::Vector vX(6);
            vX(0) = -2;
            vX(1) = 0;
            vX(2) = 0;
            vX(3) = 4;
            vX(4) = 4;
            vX(5) = 6;

            double dxdu  =calcQuinticBezierCurveDerivU(u,vX,1,"test");
            @endcode
        */
        static double calcQuinticBezierCurveDerivU(double u, 
                           const SimTK::Vector& pts,int order, 
                           const string caller);

        /**
        Calculates the value of dydx of a quintic Bezier curve derivative at u.

        @param u        The u value of interest. Note that u must be [0,1]                
        @param xpts     The 6 control points associated with the x axis
        @param ypts     The 6 control points associated with the y axis
        @param order    The order of the derivative. Currently only orders from 1-6 
                        can be evaluated
        @param caller   The string name of the parent calling this function
        @retval         The value of (d^n y)/(dx^n) evaluated at u

        Calculates the value of dydx of a quintic Bezier curve derivative at u.
        Note that a 2D Bezier curve can have an infinite number of derivatives, 
        because x and y are functions of u. Thus

        \verbatim
        dy/dx = (dy/du)/(dx/du)
        d^2y/dx^2 = d/du(dy/dx)*du/dx
                  = [(d^2y/du^2)*(dx/du) - (dy/du)*(d^2x/du^2)]/(dx/du)^2 
                    *(1/(dx/du))
        etc.
        \endverbatim

        <B>COMPUTATIONAL COSTS</B>

        This obviously only functions when the Bezier curve in question has a 
        finite derivative. Additionally, higher order derivatives are more 
        numerically expensive to evaluate than lower order derivatives. For 
        example, here are the number of operations required to compute the
        following derivatives
        \verbatim
        dy/dx       Divisions   Multiplications Additions   Assignments
            dy/du               20              19          11
            dx/du               20              19          11
            dy/dx   1        
            total   1           40              38          22

        d2y/dx2     Divisions   Multiplications Additions   Assignments
            dy/du               20              19          11
            dx/du               20              19          11
            d2y/du2             17              17          9
            d2x/du2             17              17          9
            d2y/dx2 2           4               1           3    
            total   2           78              73          23

        d3y/dx3     Divisions   Multiplications Additions   Assignments
            dy/du               20              19          11
            dx/du               20              19          11
            d2y/du2             17              17          9
            d2x/du2             17              17          9
            d3y/du3             14              14          6
            d3x/du3             14              14          6

            d3y/dx3 4           16              5           6
            total   4           118             105         58

        d4y/dx4     Divisions   Multiplications Additions   Assignments
            dy/du               20              19          11
            dx/du               20              19          11
            d2y/du2             17              17          9
            d2x/du2             17              17          9
            d3y/du3             14              14          6
            d3x/du3             14              14          6
            d4y/du4             11              11          3
            d4x/du4             11              11          3

            d4y/dx4 5           44              15          13
            total   5           168             137         71

        d5y/dx5     Divisions   Multiplications Additions   Assignments
            dy/du               20              19          11
            dx/du               20              19          11
            d2y/du2             17              17          9
            d2x/du2             17              17          9
            d3y/du3             14              14          6
            d3x/du3             14              14          6
            d4y/du4             11              11          3
            d4x/du4             11              11          3
            d5y/du5             6               6           1
            d5x/du5             6               6           1 

            d5y/dx5 7           100             36          28
            total   7           236             170         88  

        d6y/dx6
            dy/du               20              19          11
            dx/du               20              19          11
            d2y/du2             17              17          9
            d2x/du2             17              17          9
            d3y/du3             14              14          6
            d3x/du3             14              14          6
            d4y/du4             11              11          3
            d4x/du4             11              11          3
            d5y/du5             6               6           1
            d5x/du5             6               6           1 

            d6y/dx6 9           198             75          46
            total   9           334             209         106

        etc ...
        \endverbatim

        <B>Example:</B>
            @code
            SimTK::Vector vX(6), vY(6);

            double u = 0.5;
        
            vX(0) = 1;
            vX(1) = 1.01164;
            vX(2) = 1.01164;
            vX(3) = 1.02364;
            vX(4) = 1.02364;
            vY(5) = 1.04;

            vY(0) = 0;
            vY(1) = 3e-16;
            vY(2) = 3e-16;
            vY(3) = 0.3;
            vY(4) = 0.3;
            vY(5) = 1;


            d2ydx2 = QuinticBezierCurveSet::calcQuinticBezierCurveDerivDYDX(
                     u,vX, vY, 2,"test");
            @endcode

        */        
        static double  calcQuinticBezierCurveDerivDYDX(double u,
              const SimTK::Vector& xpts, const SimTK::Vector& ypts, int order,
              const string caller);

        
        /**
        Calculates the location of quintic Bezier curve control points to 
        create a C shaped curve.

        @param x0       First intercept x location
        @param y0       First intercept y location
        @param dydx0    First intercept slope
        @param x1       Second intercept x location
        @param y1       Second intercept y location
        @param dydx1    Second intercept slope
        @param curviness A parameter that ranges between 0 and 1 to denote a 
                         straight line or a curve
        @param caller   The string name of the parent calling this function
        @return a SimTK::Matrix of 6 points Matrix(6,2) that correspond to the 
                         X, and Y control points for a quintic Bezier curve that
                         has the above properties

        Calculates the location of quintic Bezier curve control points to 
        create a C shaped curve that intersects points 0 (x0, y0) and point 1
        (x1, y1) with slopes dydx0 and dydx1 respectively, and a second 
        derivative of 0. The curve that results can approximate a line 
        (curviness = 0), or in a smooth C shaped curve (curviniess = 1)

        The current implementation of this function is not optimized in anyway
        and has the following costs:

        <B>COMPUTATIONAL COSTS</B>
        \verbatim
        Divisions   Multiplication  Additions   Assignments
        1           13              9              23
        \endverbatim


        <B>Example:</B>
            @code
            double x0 = 1;
            double y0 = 0;
            double dydx0 = 0;
            double x1 = 1.04;
            double y1 = 1;
            double dydx1 = 43;
            double c = 0.75;

            SimTK::Matrix p0 = QuinticBezierCurveSet::
               calcQuinticBezierCornerControlPoints(x0, y0, dydx0,x1,y1,dydx01,
                                                                     c,"test");
            @endcode

        */
        static SimTK::Matrix calcQuinticBezierCornerControlPoints(double x0, 
            double y0, double dydx0, double x1, double y1, double dydx1, 
            double curviness, const string caller);

        /**
        This function numerically integrates the Bezier curve y(x).

        @param vX       Values of x to evaluate the integral of y(x) 
        @param ic0      The initial value of the integral
        @param intAcc   Accuracy of the integrated solution
        @param uTol     Tolerance on the calculation of the intermediate u term
        @param uMaxIter Maximum number of iterations allowed for u to reach its
                        desired tolerance.
        @param mX           The 6xn matrix of Bezier control points for x(u)
        @param mY           The 6xn matrix of Bezier control points for y(u)
        @param aSplineUX    The array of spline objects that approximate u(x) on
                            each Bezier interval
        @param caller       The string name of the parent calling this function
        @param flag_intLeftToRight  Setting this flag to true will evaluate the
                integral from the left most point to the right most 
                point. Setting this flag to false will cause the 
                integral to be evaluated from right to left.
        @return SimTK::Matrix Col 0: X vector, Col 1: int(y(x))


        This function numerically integrates the Bezier curve y(x), when really 
        both x and y are specified in terms of u. Evaluate the integral at the 
        locations specified in vX and return the result. 

        <B>COMPUTATIONAL COSTS</B>

        This the expense of this function depends on the number of points in
        vX, the points for which the integral y(x) must be calculated. The 
        integral is evaluated using a Runge Kutta 45 integrator, and so each
        point requires 6 function evaluations. 
        (http://en.wikipedia.org/wiki/Dormand%E2%80%93Prince_method)

        The cost of evaluating y(x), which has 3 Bezier curves,
        each with a splined approximate inverse of 100 points, once is:
        \verbatim
                    Comp        Div     Mult    Additions   Assignments
        calcIdx     3*3+2=11                    1*3=3       3
        calcU       15          2       82      42          60
        calcQuinticBezierCurveVal
                                        21      20          13
        Total       26          2       103     65          76
        \endverbatim

        Ignoring the costs associated with the integrator itself, and assuming
        that the integrator evaluates the function 6 times per integrated point,
        the cost of evaluating the integral at each point in vX is:

        \verbatim
                        Comp        Div     Mult    Additions      Assignments
        RK45 on 1pt  6*(26          2       103      65              76)
        Total           156         12      618      390            456
        \endverbatim

        Typically the integral is evaluated 100 times per section in order to 
        build an accurate spline-fit of the integrated function. Once again,
        ignoring the overhead of the integrator, the function evaluations alone
        for the current example would be

        \verbatim
        RK45 on 100pts per section, over 3 sections
                    Comp        Div     Mult        Additions      Assignments        
             3*100*(156         12      618         390            456
        Total       46,800      3600    185,400     117,000        136,000
        \endverbatim

        The example below is quite involved, but just so it can show you an
        example of how to create the array of Spline objects that approximate 
        the function u(x). Although the example has been created for only 1
        Bezier curve set, simply changing the size and entries of the matricies
        _mX and _mY will allow multiple sets to be integrated.


        <B>Example:</B>
        @code
        double INTTOL = 1e-12;
        double UTOL   = 1e-14;
        int    MAXITER= 10;

        SimTK::Matrix _mX(6,1), _mY(6,1);
        _mX(0)= 1;
        _mX(1)= 1.01164;
        _mX(2)= 1.01164;
        _mX(3)= 1.02364;
        _mX(4)= 1.02364;
        _mX(5)= 1.04;

        _mY(0) = 0;
        _mY(1) = 3.10862e-16;
        _mY(2) = 3.10862e-16;
        _mY(3) = 0.3;
        _mY(4) = 0.3;
        _mY(5) = 1;

        _numBezierSections = 1;
        bool _intx0x1 = true; //Says we're integrating from x0 to x1
        //////////////////////////////////////////////////
        //Generate the set of splines that approximate u(x)
        //////////////////////////////////////////////////
        SimTK::Vector u(NUM_SAMPLE_PTS); //Used for the approximate inverse
        SimTK::Vector x(NUM_SAMPLE_PTS); //Used for the approximate inverse

        //Used to generate the set of knot points of the integral of y(x)    
        SimTK::Vector xALL(NUM_SAMPLE_PTS*_numBezierSections-(_numBezierSections-1));
        _arraySplineUX.resize(_numBezierSections);
        int xidx = 0;

        for(int s=0; s < _numBezierSections; s++){
            //Sample the local set for u and x
            for(int i=0;i<NUM_SAMPLE_PTS;i++){
                u(i) = ( (double)i )/( (double)(NUM_SAMPLE_PTS-1) );
                x(i) = QuinticBezierCurveSet::
                    calcQuinticBezierCurveVal(u(i),_mX(s),_name);            
                if(_numBezierSections > 1){
                    //Skip the last point of a set that has another set of points
                    //after it. Why? The last point and the starting point of the
                    //next set are identical in value.
                    if(i<(NUM_SAMPLE_PTS-1) || s == (_numBezierSections-1)){
                        xALL(xidx) = x(i);
                        xidx++;
                    }
                }else{
                    xALL(xidx) = x(i);                
                    xidx++;
                }
            }
            //Create the array of approximate inverses for u(x)    
            _arraySplineUX[s] = SimTK::SplineFitter<Real>::
                fitForSmoothingParameter(3,x,u,0).getSpline();
        }

       
        //////////////////////////////////////////////////
        //Compute the integral of y(x) and spline the result    
        //////////////////////////////////////////////////

        SimTK::Vector yInt =  QuinticBezierCurveSet::
            calcNumIntBezierYfcnX(xALL,0,INTTOL, UTOL, MAXITER,_mX, _mY,
            _arraySplineUX,_name);
    
        if(_intx0x1==false){
            yInt = yInt*-1;
            yInt = yInt - yInt(yInt.nelt()-1);
        }

        _splineYintX = SimTK::SplineFitter<Real>::
                fitForSmoothingParameter(3,xALL,yInt,0).getSpline();
        
        @endcode


        */
        static SimTK::Matrix calcNumIntBezierYfcnX(const SimTK::Vector& vX, 
            double ic0, double intAcc, double uTol, int uMaxIter,
            const SimTK::Matrix& mX, const SimTK::Matrix& mY,
            const SimTK::Array_<SimTK::Spline>& aSplineUX, 
            bool flag_intLeftToRight,const string caller);


   private:
        

        /**
        This function will print cvs file of the column vector col0 and the 
        matrix data.
       
        @param col0     A vector that must have the same number of rows as the 
                        data matrix. This column vector is printed as the first 
                        column
        @param data     A matrix of data
        @param filename The name of the file to print
        */
        static void printMatrixToFile(const SimTK::Vector& col0, 
                                    const SimTK::Matrix& data, string filename);

        static void printBezierSplineFitCurves(
            const SimTK::Function_<double>& curveFit,SimTK::Matrix& ctrlPts, 
            SimTK::Vector& xVal, SimTK::Vector& yVal, string filename);        

        /**
        This function will return a value that is equal to u, except when u is
        outside of[0,1], then it is clamped to be 0, or 1
        @param u    The parameter to be clamped
        @retval u but restricted to 0,1.
        */
        static double clampU(double u);

};

}

#endif //OPENSIM_QUINTICBEZIERCURVESET_H_