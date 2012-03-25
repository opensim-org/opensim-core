#ifndef OPENSIM_MUSCLECURVEFUNCTION_H_
#define OPENSIM_MUSCLECURVEFUNCTION_H_

// MuscleCurveFunction.h
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

#include "QuinticBezierCurveSet.h"

namespace OpenSim { 

    /**
    This class contains the quintic Bezier curves, x(u) and y(u), that have been
    created by MuscleCurveFitter to follow a physiologically meaningful muscle
    characteristic.
    */
    class OSIMCOMMON_API MuscleCurveFunction : public SimTK::Function_<double>
//    class MuscleCurveFunction : public SimTK::Function_<double>

    {
    private:
       

        /**Array of spline fit functions X(u) for each Bezier elbow*/
        SimTK::Array_<SimTK::Spline> _arraySplineUX;        
        /**Spline fit of the integral of the curve y(x)*/
        SimTK::Spline _splineYintX;
        /**Bezier X1,...,Xn control point locations. Control points are 
        stored in 6x1 vectors in the order above*/
        const SimTK::Matrix _mX;
        /**Bezier Y1,...,Yn control point locations. Control points are 
        stored in 6x1 vectors in the order above*/
        const SimTK::Matrix _mY;
        /**The number of quintic Bezier curves that describe the relation*/
        int _numBezierSections;

        /**The minimum value of the domain*/
        const double _x0;
        /**The maximum value of the domain*/
        const double _x1;
        /**The minimum value of the range*/
        const double _y0;
        /**The maximum value of the range*/
        const double _y1;
        /**The slope at _x0*/
        const double _dydx0;
        /**The slope at _x1*/
        const double _dydx1;
        /**This is the users */
        const bool _computeIntegral;

        /**This variable, when true, indicates that the user wants the integral
        from left to right (x0 to x1). If it is false, the integral from right
        to left (x1 to x0) is computed*/
        const bool _intx0x1;
        /**The name of the function**/
        const std::string _name;

    public:
       //MuscleCurveFunction();
       /**
       Creates a quintic spline of f = y(x).

       @param mX         The matrix of quintic Bezier x point locations (6xn). 
       @param mY         The matrix of quintic Bezier y point locations (6xn).
       @param x0         The minimum x value
       @param x1         The maximum x value
       @param y0         The value of y(x) at x=x0
       @param y1         The value of y(x) at x=x1
       @param dydx0      The value of dy/dx at x=x0
       @param dydx1      The value of dy/dx at x=x1

       @param computeIntegral  If this is true, the integral is numerically
                               calculated and splined. If false, this integral
                               is not computed, and a call to .calcIntegral will
                               throw an exception

       @param intx0x1       If this is true, the integral of the curve will be
                            computed from x0-x1, with an initial condition of 0
                            at x0. If this flag is false, the integral will be 
                            computed from x1-x0 with the initial condition at 
                            x1 of 0.

       @param name          The name of the data this MuscleCurveFunction 

       <B>Computational Costs</B>


       Costs for an m-section Quintic Bezier Curve. Generating the integral 
       curve is not cheap, and so should only be used when if it will be 
       evaluated during a simulation. The following costs are for a splined
       Bezier curve that consists of m sections.
       \verbatim
       =========================================================================
       WITHOUT INTEGRAL
       _________________________________________________________________________
                        Function    Comp    Div     Mult    Add     Assignments 
       _________________________________________________________________________
        member assign                                               M:2, 9
        curve gen:      m,m*100     m       m*100           m       m*100*(4) 
                                 +m*100(3)                  +m*100*(3)

        Function detail
            Evaluations Function
            m           SimTK::SplineFitter<Real>::
                            fitForSmoothingParameter(3,x,u,0).getSpline();
            Cost:       ?

            m*100       QuinticBezierCurveSet::
                            calcQuinticBezierCurveVal
            Cost:                                   Mult     Add   Assignments
                                                    21       20    13       

        Total           ~typically  > 2100*m multiplications, additions,
                                    > 1000*m assignments
                                    > 100*m divisions
       _________________________________________________________________________    
                    Comp        Div     Mult        Add         Assignments
        Total:      m+m*100(3)  m*100   m*100*21    m*100*20    m*100*13
                                                    +m+m*100*3  +m*100*4+9+M:2                             
                    + m*Cost(SimTK::SplineFitter ...)
       =========================================================================
        ADDITIONAL COST OF COMPUTING THE INTEGRAL CURVE
        
                               Comp Div     Mult  Add      Assign       
       RK45 Fn.Eval     m*100*(156  12      618   390      456)
       RK45 Overhead    m*100*(?    ?       ?     ?         ? )
       Spline cost      m*100*(?    ?       ?     ?         ? )

       Total:           ~typically > 100,000's mult, additions, assignments
                                   > 40,000 comparisions 
                                   > 3000 divisions

       =========================================================================
        M: Matrix
        V: Vector

        N.B. These costs are dependent on QuinticBezierCurveSet
       \endverbatim

              */
       MuscleCurveFunction(const SimTK::Matrix& mX,const SimTK::Matrix& mY, 
          double x0, double x1,double y0, double y1,double dydx0, double dydx1,
          const bool computeIntegral, const bool intx0x1, const std::string name);

       /**Calculates the value of the curve this object represents.

       @param x The domain point of interest
       @returns The value of the curve


       The curve is parameterized as a set of Bezier curves. If x is within the
       domain of these Bezier curves they will be evaluated. If x is outside
       of the domain of these Bezier curves a linear extrapolation will be 
       evalulated


       <B>Computational Costs</B>


       For an m-section Bezier curve this function has the following costs
       \verbatim
       ________________________________________________________________________
       If x is in the Bezier Curve
                             Name     Comp.   Div.    Mult.   Add.    Assign.
        _______________________________________________________________________
       QuinticBezierCurveSet::
                        calcIndex     3*m+2                   1*m     3   
                           *calcU     15      2       82      42      60  
        calcQuinticBezierCurveVal                     21      20      13
                            total  15+3*m+2   2       103     62+1*m  76

        *Approximate. Uses iteration
       ________________________________________________________________________
       If x is in the linear region

                             Name     Comp.   Div.    Mult.   Add.    Assign.
                                         1               1      2     1
       ________________________________________________________________________
       \endverbatim
       
       */
       double calcValue(double x) const;

       /**
       Refer to the documentation for calcValue(double x) 
       because this function is identical in function to 
       calcValue(double x), but requires different inputs. 
       This is a required virtual function required because this class extends 
       the SimTK::Function interface.
       */
        double calcValue(const SimTK::Vector& x) const; /*virtual*/

       /**Calculates the value of the derivative of the curve this object 
       represents. 

       @param x     The domain point of interest
       @param order The order of the derivative to compute. Note that order must
                    be between 0 and 6. Calling 0 just calls calcValue. Note that
                    higher order derivatives are substantially more expensive
                    to compute than lower order ones, with each subsequent 
                    increase in order approximately doubling the required 
                    number of flops (this ignores some overhead that is 
                    associated with each function evaluation)
       @return The value of the d^ny/dx^n th derivative evaluated at x         
       
       If x is within the domain of these Bezier curves they will be evaluated. 
       If x is outside of the domain of these Bezier curves a linear 
       extrapolation will be evalulated
        
       <B>Computational Costs</B>


       For an m-section Bezier curve this function has the following costs
       \verbatim
       ________________________________________________________________________
       If x is in the Bezier Curve, and dy/dx is being evaluated
                             Name     Comp.   Div.    Mult.   Add.    Assign.
        _______________________________________________________________________
        Overhead:
       QuinticBezierCurveSet::
                        calcIndex     3*m+2                   1*m     3   
                           *calcU     15      2       82      42      60  
        Derivative Evaluation:
        **calcQuinticBezierCurveDYDX                  21      20      13
                            dy/du                     20      19      11
                            dx/du                     20      19      11
                            dy/dx             1

                            total    17+3*m   3       143     m+100   98

        *Approximate. Uses iteration
        **Higher order derivatives cost more
       ________________________________________________________________________
       If x is in the linear region

                             Name     Comp.   Div.    Mult.   Add.    Assign.
                                         1               1      2     1
       ________________________________________________________________________
       \endverbatim
     

       */
       double calcDerivative(double x, int order) const;       

       /** Refer to the documentation for calcDerivative(double x, int order) 
       because this function is identical in function to 
       calcDerivative(double x, int order), but requires different inputs. 
       This is a required virtual function required because this class extends 
       the SimTK::Function interface.*/
       double calcDerivative(
                               const SimTK::Array_<int>& derivComponents, 
                               const SimTK::Vector& x) const; /*virtual*/ 

       /**This will return the size of the vector that the 
       calcValue(const SimTK::Vector& x) require. This is a required virtual 
       function required because this class extends the SimTK::Function 
       interface, though is only needed if you call 
       
       double calcValue(const SimTK::Vector& x) const;

       or

       double calcDerivative( const SimTK::Array_<int>& derivComponents, 
                               const SimTK::Vector& x) const;

        Since this class is implementing strictly scalar functions you can use
        the simplified versions of calcValue(double x) and 
        calcDerivative(double x, int order) instead.

       */
       int getArgumentSize() const; /*virtual*/ 

       /**@return The maximum order derivative that this object is capable of 
       returning*/
       /*virtual*/ int getMaxDerivativeOrder() const;

       /**This will return the value of the integral of this objects curve 
       evaluated at x. 
       
       @param x the domain point of interest
       @return the value of the functions integral evaluated at x

       The integral is approximate, though its errors are small.
       The integral is computed by numerically integrating the function when
       the constructor for this class is called (if computeIntegral is true) and 
       then splining the result, thus the regions between the knot points may
       have some error in them. A very fine mesh of points is used to create the
       spline so the errors will be small

       <B>Computational Costs</B>
       \verbatim
       ________________________________________________________________________
       If x is in the Bezier Curve, and dy/dx is being evaluated
                             Name     Comp.   Div.    Mult.   Add.    Assign.
        _______________________________________________________________________
                 *spline.calcValue     7               2       3       1

        *Approximate cost of evaluating a cubic spline with 100 knots, where
        the bisection method is used to find the correct index
       ________________________________________________________________________
       If x is in the linear region

                     Name              Comp.   Div.    Mult.   Add.    Assign.
                     *spline.calcValue  1               2       3       1        
                     integral eval      2               4       5       1
                     total              3               6       8       2

        *Approximate cost of evaluating a cubic spline at its last knot point
       ________________________________________________________________________
       \endverbatim

       */
       double calcIntegral(const double x) const;
       
       /**
        Returns a bool that indicates if the integral curve has been computed.

        @return true if the integral of this function is available, false if
                it has not been computed.
       */
       bool isIntegralAvailable() const;

       /**
       Returns a bool that indicates if the integral computed is compuated left
       to right, or right to left.

       @return true if the integral was computed left to right, and false if the
               integral was computed right to left. Note that the output of
               this function is only valid if isIntegralAvailable() returns
               true.
       */
       bool isIntegralComputedLeftToRight() const;

       /**
       Returns a string that is the name for this curve, which is set at the 
       time of construction and cannot be changed after construction.

       @return The string name this object was given during construction*/
       std::string getName() const;

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

       /**This function will generate a csv file (of 'name_curveName.csv', where 
       name is the one used in the constructor) of the muscle curve, and 
       'curveName' corresponds to the function that was called from
       MuscleCurveFunctionFactory to create the curve.
       
        @param path The full path to the location. Note '/' slashes must be used,
            and do not put a '/' after the last folder.

       For example the tendon 
       curve for a muscle named 'glutmax' will be:
       
       'glutmax_tendonForceLengthCurve.csv'

       The file will contain the following columns:
       
       \verbatim
       Col# 1, 2,     3,       4,       5,       6,       7,       8, 9,
            x, y, dy/dx, d2y/dx2, d3y/dx3, d4y/dx4, d5y/dx5, d6y/dx6, iy
       \endverbatim

       Where iy is the integral of y(x). If the curve has been set not to have
       an integral, this column will not exist.
       
       The curve will be sampled from its linear extrapolation region, through 
       the curve, out to the other linear extrapolation region. The width of 
       each linear extrapolation region is 10% of the entire range of x, or 
       0.1*(x1-x0).

       The number of rows used will vary from curve to curve. Each quintic 
       Bezier curve section will have 1000 samples + 20 samples for the linear 
       extrapolation region. Some muscle curves (the tendon, parallel elements, 
       compressive elements) consist of only 1 elbow, and so these matrices will 
       have only 1000+20 rows. The force velocity curve is made up of 2 elbows 
       and will have 2000+20 rows. The active force length curve has 5 elbows, 
       and so its sampled matrix will have 5000+20 rows    

       <B>Computational Costs</B>


        For a curve with m quintic Bezier curve sections:
       \verbatim
        _______________________________________________________________________
                             Name     Comp.   Div.    Mult.   Add.    Assign.
        _______________________________________________________________________

                        *overhead    (17+3*m     2    82      42+m    63)*7
                                      119+21*m  14    574     294+7m  441

                        calcValue                     21      20      13
            calcDerivative: dy/dx                1    40      38      22          
                          : d2y/dx2              2    78      73      23
                          : d3y/dx3              4    118     105     58
                          : d4y/dx4              5    168     137     71
                          : d5y/dx5              7    236     170     88
                          : d6y/dx6              9    334     209     106

                     **calcIntegral    7               2       3       1             

                   total per point     126+21*m  42   1571    1049    823
                   total per elbow   126k+21k*m  42k  1571k   1049k   823k
                   
            *Approximate. Overhead associated with finding the correct Bezier
                          spline section, and evaluating u(x). 
                          Assumes 2 Newton iterations in calcU

            **Approximate. Includes estimated cost of evaluating a cubic spline
                          with 100 knots
       \endverbatim

       */
       void printMuscleCurveToFile(std::string path) const;
       
///@cond       
       /**
       THIS FUNCTION IS PUBLIC FOR TESTING ONLY 
                   DO NOT USE THIS!

       This function will generate a SimTK::Matrix populated with samples of
       the muscle curves values, derivatives (up to 6) and its first integral. 
       The matrix has the following columns:
       
       \verbatim
       Col# 1, 2,     3,       4,       5,       6,       7,       8, 9,
            x, y, dy/dx, d2y/dx2, d3y/dx3, d4y/dx4, d5y/dx5, d6y/dx6, iy
       \endverbatim

       Where iy is the integral of y(x). If the curve has been set not to have
       an integral, this column will not exist.
       
       The curve will be sampled from its 
       linear extrapolation region, through the curve, out to the other linear
       extrapolation region. The width of each linear extrapolation region is
       10% of the entire range of x, or 0.1*(x1-x0).

       The rows used will vary from curve to curve. Each quintic Bezier curve
       section will have 100 samples + 20 samples for the linear extrapolation
       region. Some muscle curves (the tendon, parallel elements, compressive 
       elements) consist of only 1 elbow, and so these matrices will have only 
       100+20 rows. The force velocity curve is made up of 2 elbows and will 
       have 200+20 rows. The active force length curve has 5 elbows, and so its
       sampled matrix will have 500+20 rows  
       */

       SimTK::Matrix calcSampledMuscleCurve() const;
       ///@endcond

    private:



       


        /**
        This function will print cvs file of the column vector col0 and the 
        matrix data
       
        @param data A matrix of data
        @param path The desired path to the folder to write the file
        @param filename The name of the file to print
        */
        static void printMatrixToFile(SimTK::Matrix& data,SimTK::Array_<std::string>,
            std::string path, std::string filename);

        

    };

}



#endif //OPENSIM_MUSCLECURVEFUNCTION_H_
