/* -------------------------------------------------------------------------- *
 *              OpenSim:  testSmoothSegmentedFunctionFactory.cpp              *
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

/* 
    Below is a basic bench mark simulation for the SmoothSegmentedFunctionFactory
    class, a class that enables the easy generation of C2 continuous curves 
    that define the various characteristic curves required in a muscle model
 */

// Author:  Matthew Millard

//==============================================================================
// INCLUDES
//==============================================================================


#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/SegmentedQuinticBezierToolkit.h>
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>


#include <SimTKsimbody.h>
#include <ctime>
#include <fstream>
#include <string>
#include <stdio.h>


using namespace std;
using namespace OpenSim;
using namespace SimTK;


/**
This function will print cvs file of the column vector col0 and the matrix 
 data

@params col0: A vector that must have the same number of rows as the data matrix
                This column vector is printed as the first column
@params data: A matrix of data
@params filename: The name of the file to print
*/
void printMatrixToFile(const SimTK::Vector& col0, 
    const SimTK::Matrix& data, string filename)
{
    
    ofstream datafile;
    datafile.open(filename.c_str());

    for(int i = 0; i < data.nrow(); i++){
        datafile << col0(i) << ",";
        for(int j = 0; j < data.ncol(); j++){
            if(j<data.ncol()-1)
                datafile << data(i,j) << ",";
            else
                datafile << data(i,j) << "\n";
        }   
    }
    datafile.close();
} 


/**
This function will print cvs file of the matrix 
 data

@params data: A matrix of data
@params filename: The name of the file to print
*/
void printMatrixToFile( const SimTK::Matrix& data, string filename)
{
    ofstream datafile;
    datafile.open(filename.c_str());

    for(int i = 0; i < data.nrow(); i++){
        for(int j = 0; j < data.ncol(); j++){
            if(j<data.ncol()-1)
                datafile << data(i,j) << ",";
            else
                datafile << data(i,j) << "\n";
        }   
    }
    datafile.close();
}

/**
    This function computes a standard central difference dy/dx. If 
    extrap_endpoints is set to 1, then the derivative at the end points is 
    estimated by linearly extrapolating the dy/dx values beside the end points

 @param x domain vector
 @param y range vector
 @param extrap_endpoints: (false)   Endpoints of the returned vector will be 
                                    zero, because a central difference
                                    is undefined at these endpoints
                            (true)  Endpoints are computed by linearly 
                                    extrapolating using a first difference from 
                                    the neighboring 2 points
 @returns dy/dx computed using central differences
*/
SimTK::Vector calcCentralDifference(SimTK::Vector x, SimTK::Vector y, 
                                               bool extrap_endpoints){
 

    SimTK::Vector dy(x.size());
    double dx1,dx2;
    double dy1,dy2;
    int size = x.size();
    for(int i=1; i<x.size()-1; i++){
        dx1 = x(i)-x(i-1);
        dx2 = x(i+1)-x(i);
        dy1 = y(i)-y(i-1);
        dy2 = y(i+1)-y(i);
        dy(i)= 0.5*dy1/dx1 + 0.5*dy2/dx2;
    }

    if(extrap_endpoints == true){
        dy1 = dy(2)-dy(1);
        dx1 = x(2)-x(1);
        dy(0) = dy(1) + (dy1/dx1)*(x(0)-x(1));

        dy2 = dy(size-2)-dy(size-3);
        dx2 = x(size-2)-x(size-3);
        dy(size-1) = dy(size-2) + (dy2/dx2)*(x(size-1)-x(size-2));
    }
    return dy;
}

/**
    This function computes a standard central difference dy/dx at each point in
    a vector x, for a SmoothSegmentedFunction mcf, to a desired tolerance. This 
    function will take the best step size at each point to minimize the 
    error caused by taking a numerical derivative, and the error caused by
    numerical rounding error:

    For a step size of h/2 to the left and to the right of the point of 
    interest the error is
    error = 1/4*h^2*c3 + r*f(x)/h,                  (1)
         
    Where c3 is the coefficient of the 3rd order Taylor series expansion
    about point x. Thus c3 can be computed if the order + 2 derivative is
    known
        
        c3 = (d^3f(x)/dx^3)/(6)                        (2)
        
    And r*f(x)/h is the rounding error that occurs due to the central 
    difference.

    Taking a first derivative of 1 and solving for h yields

    h = (r*f(x)*2/c3)^(1/3)

    Where r is SimTK::Eps

     @param x domain vector
     @param mcf the SmoothSegmentedFunction of interest
     @param order the order of the numerical derivative
     @param tolerance desired tolerance on the returned result
     @returns dy/dx computed using central differences
*/
SimTK::Vector calcCentralDifference(SimTK::Vector x, SmoothSegmentedFunction& mcf 
                                                   ,double tol, int order){
 

    SimTK::Vector dyV(x.size());
    SimTK::Vector yV(x.size());

    double y = 0;
    // double dy = 0;
    double dyNUM = 0;
    // double err= 0;
    double h = 0;
    double xL = 0;
    double xR = 0;

    double c3 = 0;
    double fL = 0;
    double fR = 0;
    // double rootEPS = sqrt(SimTK::Eps);

    double y_C3min = 1e-10;
    double y_C3max = 1e1;


    for(int i=0; i<x.size(); i++){
        yV(i) = mcf.calcDerivative(x(i),order-1);
    }
   

    for(int i=0; i< x.size(); i++){
        
        c3 = abs(mcf.calcDerivative(x(i),order+2));
        
        //singularity prevention
        if(abs(c3) < y_C3min)
            c3 = y_C3min;
        //Compute h
        y  = abs(mcf.calcDerivative(x(i), order-1));
        //preventing 0 from being assigned to y
        if(y < y_C3min)
            y = y_C3min;

        //Dumb check
        if(y/c3 < y_C3min){
            c3 = 1;
            y = y_C3min;
        }
        if(y/c3 > y_C3max){
            c3 = 1;
            y = y_C3max;
        }

        h  = pow( ( (SimTK::Eps*y*2.0)/(c3) ) , 1.0/3.0);
       
        //Now check that h to the left and right are at least similar
        //If not, take the smallest one.
        xL = x(i)-h/2;
        xR = x(i)+h/2;

        fL = mcf.calcDerivative(xL, order-1);
        fR = mcf.calcDerivative(xR, order-1);

        //Just for convenience checking ...
        dyNUM = (fR-fL)/h;
        /*dy    = */mcf.calcDerivative(x(i),order);
        // err   = abs(dy-dyNUM);

        /*if(err > tol && abs(dy) > rootEPS && order <= 2){
            err = err/abs(dy);
            if(err > tol)
                cout << "rel tol exceeded" << endl;           
        }*/

        dyV(i) = dyNUM;

    }


    return dyV;
}

/**
    This function tests numerically for continuity of a curve. The test is 
    performed by taking a point on the curve, and then two points (called the 
    shoulder points) to the left and right of the point in question. The value
    of the functions derivative is evaluated at each of the shoulder points and
    used to linearly extrapolate from the shoulder points back to the original 
    point. If the original point and the linear extrapolations of each of the 
    shoulder points agree within tol, then the curve is assumed to be 
    continuous.


    @param x        Values to test for continuity
    @param yx       The SmoothSegmentedFunction to test
    @param order    The order of the curve of SmoothSegmentedFunction to test
                    for continuity
    @param minTol   The minimum error allowed - this prevents the second order
                    error term from going to zero
    @param taylorErrorMult  This scales the error tolerance. The default error
                            tolerance is the 2nd-order Taylor series
                            term.
*/
bool isFunctionContinuous(SimTK::Vector xV, 
                      SmoothSegmentedFunction yV, int order, 
                      double minTol,
                      double taylorErrorMult)
{
    bool flag_continuous = true;

    double xL = 0;      // left shoulder point
    double xR = 0;      // right shoulder point
    double yL = 0;      // left shoulder point function value
    double yR = 0;      // right shoulder point function value
    double dydxL = 0;   // left shoulder point derivative value
    double dydxR = 0;   // right shoulder point derivative value

    double xVal = 0;    //x value to test
    double yVal = 0;    //Y(x) value to test

    double yValEL = 0;  //Extrapolation to yVal from the left
    double yValER = 0;  //Extrapolation to yVal from the right

    double errL = 0;
    double errR = 0;

    double errLMX = 0;
    double errRMX = 0;


    for(int i =1; i < xV.nelt()-1; i++){
        xVal = xV(i);
        yVal = yV.calcDerivative(xVal, order);

        xL = 0.5*(xV(i)+xV(i-1));
        xR = 0.5*(xV(i)+xV(i+1));

        yL = yV.calcDerivative(xL,order);
        yR = yV.calcDerivative(xR,order);

        dydxL = yV.calcDerivative(xL,order+1);
        dydxR = yV.calcDerivative(xR,order+1);

        
        yValEL = yL + dydxL*(xVal-xL);
        yValER = yR - dydxR*(xR-xVal);

        errL = abs(yValEL-yVal);
        errR = abs(yValER-yVal);

        errLMX = abs(yV.calcDerivative(xL,order+2)*0.5*(xVal-xL)*(xVal-xL));
        errRMX = abs(yV.calcDerivative(xR,order+2)*0.5*(xR-xVal)*(xR-xVal));

        errLMX*=taylorErrorMult;
        errRMX*=taylorErrorMult;

        if(errLMX < minTol)
            errLMX = minTol;

        if(errRMX < minTol)
            errRMX = minTol; // to accommodate numerical
                             //error in errL

        if(errL > errLMX || errR > errRMX){            
            flag_continuous = false;
        }
    }

    return flag_continuous;
}


/**
This function will scan through a vector and determine if it is monotonic or
not

@param y the vector of interest
@param multEPS The tolerance on the monotonicity check, expressed as a scaling of
                SimTK::Eps
@return true if the vector is monotonic, false if it is not
*/
bool isVectorMonotonic(SimTK::Vector y, int multEPS)
{
    double dir = y(y.size()-1)-y(0);
    bool isMonotonic = true;

    if(dir < 0){
        for(int i =1; i <y.nelt(); i++){
            if(y(i) > y(i-1)+SimTK::Eps*multEPS){
                isMonotonic = false;
                //printf("Monotonicity broken at idx %i, since %fe-16 > %fe-16\n",
                //        i,y(i)*1e16,y(i-1)*1e16);
                printf("Monotonicity broken at idx %i, since "
                    "y(i)-y(i-1) < tol, (%f*SimTK::Eps < SimTK::Eps*%i) \n",
                        i,((y(i)-y(i-1))/SimTK::Eps), multEPS);
            }
        }
    }
    if(dir > 0){
        for(int i =1; i <y.nelt(); i++){
            if(y(i) < y(i-1)-SimTK::Eps*multEPS){
                isMonotonic = false;
                printf("Monotonicity broken at idx %i, since "
                    "y(i)-y(i-1) < -tol, (%f*SimTK::Eps < -SimTK::Eps*%i) \n",
                        i,((y(i)-y(i-1))/SimTK::Eps), multEPS);
            }
        }
    }
    if(dir == 0){
        isMonotonic = false;
    }

    return isMonotonic;
}

/**
This function will compute the numerical integral of y(x) using the trapezoidal
method

@param x the domain vector
@param y the range vector, of y(x), evaluated at x
@param flag_TrueIntForward_FalseIntBackward 
    When this flag is set to true, the integral of y(x) will be evaluated from
    left to right, starting with int(y(0)) = 0. When this flag is false, then
    y(x) will be evaluated from right to left with int(y(n)) = 0, where n is 
    the maximum number of elements.                                            
@return the integral of y(x)
*/
SimTK::Vector calcTrapzIntegral(SimTK::Vector x, SimTK::Vector y, 
                                bool flag_TrueIntForward_FalseIntBackward)
{
    SimTK::Vector inty(y.size());
    inty = 0;


    // int startIdx = 1;
    int endIdx = y.size()-1;

    if(flag_TrueIntForward_FalseIntBackward == true){
               
        double width = 0;
        for(int i = 1; i <= endIdx; i=i+1){
            width = abs(x(i)-x(i-1));
            inty(i) = inty(i-1) +  width*(0.5)*(y(i)+y(i-1));
        }

    }else{
        
        double width = 0;            
        for(int i = endIdx-1; i >= 0; i=i-1){
            width = abs(x(i)-x(i+1));
            inty(i) = inty(i+1) +  width*(0.5)*(y(i)+y(i+1));
        }
    }

     
    return inty;
}

/**
   @param a The first vector
   @param b The second vector
   @return Returns the maximum absolute difference between vectors a and b
*/
double calcMaximumVectorError(SimTK::Vector a, SimTK::Vector b)
{
    double error = 0;
    double cerror=0;
    for(int i = 0; i< a.nelt(); i++)
    {
        cerror = abs(a(i)-b(i));
        if(cerror > error){
            error = cerror;
        }           
    }
    return error;
}


void testQuinticBezier_Exceptions(){
    cout <<"**************************************************"<<endl;
    cout << "   TEST: Bezier Curve Exceptions" << endl;
    string name  = "testQuinticBezier_Exceptions()";

    //Generate a Bezier curve
    SimTK::Vec6 xPts = {
        0,
        0.5,
        0.5,
        0.75,
        0.75,
        1};

    SimTK::Vec6 yPts = {
        0,
        0.125,
        0.125,
        0.5,
        0.5,
        1};

    SimTK::Array_<SimTK::Vec6> xPtsSet{xPts};
    SimTK::Array_<SimTK::Vec6> yPtsSet{yPts};

    SimTK::Vector u(100);
    SimTK::Vector x(100);
    SimTK::Array_< SimTK::Spline > aSplineUX(1);
    for(int i=0; i<100; i++){
        u(i) = ((double)i)/((double)99);
        x(i)=SegmentedQuinticBezierToolkit::calcQuinticBezierCurveVal(u(i), xPts);
    }

    aSplineUX[0] = SimTK::SplineFitter<Real>::
            fitForSmoothingParameter(3,x,u,0).getSpline();
    //Now we have a curve.

    //=========================================================================
    //Test exceptions for calcQuinticBezierCornerControlPoints
    //=========================================================================
    double x0 = 0;
    double x1 = 1;
    double y0 = 0;
    double y1 = 1;
    double dydx0 = 0;
    double dydx1 = 1;
    double curviness = 0;

    double curvinessEX1 = 1.01; //illegal value 
    double curvinessEX2 = -0.01; //illegal value 


    double dydx0EX1 = 0; //illegal pair
    double dydx1EX1 = 0.1;

    SimTK_TEST_MUST_THROW(/*SegmentedQuinticBezierToolkit::ControlPointsXY test = */SegmentedQuinticBezierToolkit::
                     calcQuinticBezierCornerControlPoints(x0, y0, dydx0, 
                                        x1, y1,dydx1, curvinessEX1));

    SimTK_TEST_MUST_THROW(/*SegmentedQuinticBezierToolkit::ControlPointsXY test = */SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(x0, y0, dydx0, 
                            x1, y1,dydx1, curvinessEX2));

    SimTK_TEST_MUST_THROW(/*SegmentedQuinticBezierToolkit::ControlPointsXY test = */SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(x0, y0, dydx0EX1, 
                            x1, y1,dydx1EX1, curviness));

    //=========================================================================
    //Test exceptions for calcIndex
    //=========================================================================

    double xEX1 = xPts[0]-0.01; //This is not in the set.
    double xEX2 = xPts[5]+0.01; //This is not in the set.

    SimTK_TEST_MUST_THROW(/*int t = */SegmentedQuinticBezierToolkit::
                            calcIndex(xEX1, xPtsSet));
    SimTK_TEST_MUST_THROW(/*int t = */SegmentedQuinticBezierToolkit::
                            calcIndex(xEX2, xPtsSet));

    //=========================================================================
    //Test exceptions for calcU
    //=========================================================================
    
    //xEX1 is not within the curve, and so the Newton iteration will not
    //converge
    SimTK_TEST_MUST_THROW(/*double uPt = */SegmentedQuinticBezierToolkit::
                 calcU(xEX1, xPts, aSplineUX[0], 1e-8, 10));

    //=========================================================================
    //Test exceptions for calcQuinticBezierCurveVal
    //=========================================================================

    double uEX1 = -0.01; //illegal
    double uEX2 = 1.01;  //illegal

    SimTK_TEST_MUST_THROW(/*double tst = */SegmentedQuinticBezierToolkit::
        calcQuinticBezierCurveVal(uEX1,xPts));
    SimTK_TEST_MUST_THROW(/*double tst = */SegmentedQuinticBezierToolkit::
        calcQuinticBezierCurveVal(uEX2,xPts));

    //=========================================================================
    //Test exceptions for calcQuinticBezierCurveDerivU
    //=========================================================================

    SimTK_TEST_MUST_THROW(/*double tst = */SegmentedQuinticBezierToolkit::
        calcQuinticBezierCurveDerivU(uEX1, xPts, (int)1));
    SimTK_TEST_MUST_THROW(/*double tst = */SegmentedQuinticBezierToolkit::
        calcQuinticBezierCurveDerivU(uEX2, xPts, (int)1));
    SimTK_TEST_MUST_THROW(/*double tst = */SegmentedQuinticBezierToolkit::
        calcQuinticBezierCurveDerivU(0.5, xPts, -1));

    //=========================================================================
    //Test exceptions for calcQuinticBezierCurveDerivDYDX
    //=========================================================================
    
    SimTK_TEST_MUST_THROW(/*double test= */SegmentedQuinticBezierToolkit::
        calcQuinticBezierCurveDerivDYDX(uEX1,xPts,yPts,1));
    SimTK_TEST_MUST_THROW(/*double test= */SegmentedQuinticBezierToolkit::
        calcQuinticBezierCurveDerivDYDX(uEX2,xPts,yPts,1));
    SimTK_TEST_MUST_THROW(/*double test= */SegmentedQuinticBezierToolkit::
        calcQuinticBezierCurveDerivDYDX(0.5,xPts,yPts,-1));
    SimTK_TEST_MUST_THROW(/*double test= */SegmentedQuinticBezierToolkit::
        calcQuinticBezierCurveDerivDYDX(0.5,xPts,yPts,7));
    
    //=========================================================================
    //Test exceptions for calcNumIntBezierYfcnX
    //=========================================================================
    //There are none.
    cout << "    passed. All exceptions are functioning as intended" << endl;
    cout <<"**************************************************"<<endl;
}


/**
    This function will create a quintic Bezier curve y(x) and sample it, its 
    first derivative w.r.t. U (dx(u)/du and dy(u)/du), and its first derivative
    w.r.t. to X and print it to the screen.
*/
void testQuinticBezier_DU_DYDX()
{
    cout <<"**************************************************"<<endl;
    cout << "   TEST: Bezier Curve Derivative DU" << endl;
    string name  = "testQuinticBezier_DU_DYDX()";

    //Generate a Bezier curve
    SimTK::Vec6 xPts = {
        0,
        0.5,
        0.5,
        0.75,
        0.75,
        1};

    SimTK::Vec6 yPts = {
        0,
        0.125,
        0.125,
        0.5,
        0.5,
        1};

    SimTK::Array_<SimTK::Vec6> xPtsSet{xPts};
    SimTK::Array_<SimTK::Vec6> yPtsSet{yPts};

        double val = 0;
        double d1 = 0;
        double d2 = 0;
        double d3 = 0;
        double d4 = 0;
        double d5 = 0;
        double d6 = 0;

        double u = 0;

        int steps = 100;

        SimTK::Matrix analyticDerXU(steps,8);
        SimTK::Matrix analyticDerYU(steps,8);
        SimTK::Vector uV(steps);
        for(int i = 0; i<steps; i++){
            //int i = 10;
            u = (double)i/(steps-1);
            uV(i) = u;

            val= SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveVal(u,xPts);
            d1 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,xPts,1);
            d2 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,xPts,2);
            d3 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,xPts,3);
            d4 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,xPts,4);
            d5 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,xPts,5);
            d6 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,xPts,6);

            analyticDerXU(i,0) = u;
            analyticDerXU(i,1) = val;
            analyticDerXU(i,2) = d1;
            analyticDerXU(i,3) = d2;
            analyticDerXU(i,4) = d3;
            analyticDerXU(i,5) = d4;
            analyticDerXU(i,6) = d5;
            analyticDerXU(i,7) = d6;

            val= SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveVal(u,yPts);
            d1 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,yPts,1);
            d2 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,yPts,2);
            d3 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,yPts,3);
            d4 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,yPts,4);
            d5 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,yPts,5);
            d6 = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivU(u,yPts,6);

            analyticDerYU(i,0) = u;
            analyticDerYU(i,1) = val;
            analyticDerYU(i,2) = d1;
            analyticDerYU(i,3) = d2;
            analyticDerYU(i,4) = d3;
            analyticDerYU(i,5) = d4;
            analyticDerYU(i,6) = d5;
            analyticDerYU(i,7) = d6;

        }

        int mxDU = 6-1;
        SimTK::Matrix numericDer(analyticDerXU.nrow(), mxDU);
        SimTK::Matrix errorDer(analyticDerXU.nrow(), mxDU);

        double tol = (double)(1.0/steps);
        tol = tol*tol*50; 
        //Numerical error in a central difference increases with the 
        //square of h. 
        //http://en.wikipedia.org/wiki/Finite_difference

        for(int i=0;i<mxDU;i++){
            numericDer(i) = calcCentralDifference(analyticDerXU(0),
                                                    analyticDerXU(i+1),true); 
            errorDer(i) = abs(analyticDerXU(i+2)-numericDer(i));
            //errorDer(i)= abs( errorDer(i).elementwiseDivide(numericDer(i)) );
            //The end points can't be tested because a central difference
            //cannot be accurately calculated at these locations
            for(int j=1; j<analyticDerXU.nrow()-1; j++){
                SimTK_TEST(abs(errorDer(j,i))<tol);
                //if(errorDer(j,i)>tol)
                //printf("Error > Tol: (%i,%i): %f > %f\n",j,i,errorDer(j,i),tol);
            }
        }
        //cout << errorDer << endl;

        printf("...absolute tolerance of %f met\n", tol);

        cout << "   TEST: Bezier Curve Derivative DYDX to d6y/dx6" << endl;
        SimTK::Matrix numericDerXY(analyticDerXU.nrow(), 6);
        SimTK::Matrix analyticDerXY(analyticDerXU.nrow(),6);

        for(int i=0; i< analyticDerXU.nrow(); i++)
        {
            analyticDerXY(i,0) = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivDYDX(uV(i),xPts,yPts,1);
            analyticDerXY(i,1) = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivDYDX(uV(i),xPts,yPts,2);
            analyticDerXY(i,2) = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivDYDX(uV(i),xPts,yPts,3);
            analyticDerXY(i,3) = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivDYDX(uV(i),xPts,yPts,4);
            analyticDerXY(i,4) = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivDYDX(uV(i),xPts,yPts,5);
            analyticDerXY(i,5) = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveDerivDYDX(uV(i),xPts,yPts,6);
        }

        //Generate numerical derivative curves for the first 3 derivatives
        numericDerXY(0) = calcCentralDifference(analyticDerXU(1),
                                                    analyticDerYU(1),true);
        numericDerXY(1) = calcCentralDifference(analyticDerXU(1),
                                                    analyticDerXY(0),true);
        numericDerXY(2) = calcCentralDifference(analyticDerXU(1),
                                                    analyticDerXY(1),true);
        numericDerXY(3) = calcCentralDifference(analyticDerXU(1),
                                                    analyticDerXY(2),true);
        numericDerXY(4) = calcCentralDifference(analyticDerXU(1),
                                                    analyticDerXY(3),true);
        numericDerXY(5) = calcCentralDifference(analyticDerXU(1),
                                                    analyticDerXY(4),true);

        //Create the matrix of errors
        SimTK::Matrix errorDerXY(analyticDerXU.nrow(), 6);
        errorDerXY = abs(analyticDerXY-numericDerXY);
        errorDerXY = errorDerXY.elementwiseDivide(
                                abs(analyticDerXY+numericDerXY));

        double relTol = 5e-2;

        int relTolExceeded = 0;

        for(int j=0;j<6;j++){
            //can't test the first and last entries because a central diff.
            //cannot calculate these values accurately.
            for(int i=1;i<analyticDerXU.nrow()-1;i++){
                if(errorDerXY(i,j)>relTol){
                 //printf("Error > Tol: (%i,%i): %f > %f\n",i,j,
                 //                      errorDerXY(i,j),relTol);
                 relTolExceeded++;
                }
            }
        }
        //cout << relTolExceeded << endl;

        //The relative tolerance gets exceeded occasionally in locations of
        //rapid change in the curve. Provided there are only a few locations
        //where the relative tolerance of 5% is broken, the curves should be
        //regarded as being good. Ten errors out of a possible 100*6 data points
        //seems relatively small.
        SimTK_TEST(relTolExceeded < 10);

        //printMatrixToFile(analyticDerXY,"analyticDerXY.csv");
        //printMatrixToFile(numericDerXY,"numericDerXY.csv");
        //printMatrixToFile(errorDerXY,"errorDerXY.csv");
        printf("   ...relative tolerance of %f not exceeded more than %i times\n"
               "      across all 6 derivatives, with 100 samples each\n",
                relTol, 10);
        cout <<"**************************************************"<<endl;


}

/**
    This function will generate a series of quintic Bezier control points 
    that form a C2 corner and print the results to the command window
*/
void sampleBezierCornerGeneration()
{
    string name = "testBezierCornerGeneration()";
    double x0 = 0;
    double y0 = 0;
    double dydx0 = 0.01;
    double x1 = 1;
    double y1 = 1;
    double dydx1 = 2;
    double curviness = 0.5;

    
    
    
    SegmentedQuinticBezierToolkit::ControlPointsXY xyPts = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCornerControlPoints(x0,y0,dydx0, x1,y1,dydx1,curviness);

    cout << "XY Corner Control Points: X = " << endl;
    cout << xyPts.x << ", Y = " << xyPts.y << endl;
}
/**
    This function will sample and print a SimTK::Function to file
*/
void samplePrintExtrapolatedFunction(const SimTK::Function_<double>& fcn, 
    double x0, double x1, int num, string name)
{
    SimTK::Vector tcX(num);
    SimTK::Matrix tcY(num,3);
    SimTK::Array_<int> dx(1), ddx(2);
    dx[0]=0;
    ddx[0]=0;
    ddx[1]=0;

    double tcD = (x1-x0)/(num-1);
    SimTK::Vector x(1);

    for(int i=0; i<num; i++){
        tcX(i) = x0 + i*tcD;
        x(0)   = tcX(i);
        tcY(i,0)  = fcn.calcValue(x);
        tcY(i,1)  = fcn.calcDerivative(dx,x);
        tcY(i,2)  = fcn.calcDerivative(ddx,x);
    }
    printMatrixToFile(tcX,tcY,name);
}



/**
 1. The SmoothSegmentedFunction's derivatives will be compared against numerically 
    calculated derivatives to ensure that the errors between the 2 are small

*/
void testMuscleCurveDerivatives(SmoothSegmentedFunction mcf,SimTK::Matrix mcfSample,
                                double tol)
{
    cout << "   TEST: Derivative correctness " << endl;
    int maxDer = 4;//mcf.getMaxDerivativeOrder() - 2;
   
    SimTK::Matrix numSample(mcfSample.nrow(),maxDer);  
    SimTK::Matrix relError(mcfSample.nrow(),maxDer);
    

    // double bigTol = sqrt(SimTK::Eps);

    for(int i=0; i < maxDer; i++){
        //Compute the relative error
        numSample(i)=calcCentralDifference(mcfSample(0),mcf,tol,i+1);

        relError(i)= abs(mcfSample(i+2)-numSample(i));  

        //compute a relative error where possible
        for(int j=0; j < relError.nrow(); j++){
            if(abs(mcfSample(j,i+2)) > tol){
                relError(j,i) = relError(j,i)/mcfSample(j,i+2);
            }
        }

    }



    // <<"relError "<<endl;
    //cout << relError << endl;

    SimTK::Vector errRelMax(6);
    errRelMax = 0;
    SimTK::Vector errAbsMax(6);
    errAbsMax = 0;
    double absTol = 5*tol;

    bool flagError12=false;
    SimTK::Vector tolExceeded12V(mcfSample.nrow());
    tolExceeded12V = 0;
    
    int tolExceeded12 = 0;
    int tolExceeded34 = 0;
    for(int j=0;j<maxDer;j++){
        
        for(int i=0; i<mcfSample.nrow(); i++){
            if(relError(i,j) > tol && mcfSample(i,j+2) > tol){
                if(j <= 1){
                    tolExceeded12++;
                    tolExceeded12V(i)=1;
                    flagError12=true;
                }
                if(j>=2)
                    tolExceeded34++;                
            }
            if(mcfSample(i,j+2) > tol)
            if(errRelMax(j) < abs(relError(i,j)))
                    errRelMax(j) = abs(relError(i,j));

            //This is a harder test: here we're comparing absolute error
            //so the tolerance margin is a little higher
            if(relError(i,j) > absTol && mcfSample(i,j+2) <= tol){
                if(j <= 1){
                    tolExceeded12++;
                    tolExceeded12V(i)=1;
                    flagError12=true;
                }
                if(j>=2)
                    tolExceeded34++;                           
            }

            if(mcfSample(i,j+2) < tol)
            if(errAbsMax(j) < abs(relError(i,j)))
                    errAbsMax(j) = abs(relError(i,j));
         
        
        }

        if(flagError12 == true){
            printf("Derivative %i Rel Error Exceeded:\n",j);
            printf("x        dx_relErr dx_calcVal dx_sample"
                    " dx2_relErr dx2_calcVal dx2_sample\n");
            for(int i=0; i<mcfSample.nrow(); i++){
                if(tolExceeded12V(i) == 1){
                   printf("%f %f %f  %f %f   %f    %f",
                    mcfSample(i,0),relError(i,0),mcfSample(i,2),numSample(i,0),
                                  relError(i,1),mcfSample(i,3),numSample(i,1));
                    
                }
            }
        }
        flagError12=false;
        tolExceeded12V = 0;
    }
    




    SimTK_TEST(tolExceeded12 == 0);

     //   printMatrixToFile(mcfSample,"analyticDerivatives.csv");
     //   printMatrixToFile(numSample,"numericDerivatives.csv");
     //   printMatrixToFile(numError,"numAnalyticError.csv");
     //   cout << "Matrices Printed" << endl;

  printf("   passed: A tolerance of %fe-3 reached with a maximum relative error\n"
         "           of %fe-3 and %fe-3 for the first two derivatives\n"
         "           at points where it is possible to compute the relative\n"
         "           error.                                                \n\n"
         "           At points where the relative error couldn't be computed,\n"
         "           due to divide by 0, the first two derivatives met an \n"
         "           absolute tolerance of %fe-3, with a maximum value of \n"
         "           %fe-3 and %fe-3. \n\n"
         "           Derivatives of order 3 through %i exceeded the \n"
         "           relative or absolute tolerance (as appropriate) %i times.\n",
            tol*1e3, errRelMax(0)*1e3, errRelMax(1)*1e3, absTol*1e3,
            errAbsMax(0)*1e3, errAbsMax(1)*1e3, maxDer, tolExceeded34);
     cout << endl;

    
}
 /*
 2. The SmoothSegmentedFunction's integral, when calculated, will be compared against 
    a numerically computed integral (using the trapezoidal method)
 */

void testMuscleCurveIntegral(SmoothSegmentedFunction mcf,SimTK::Matrix mcfSample)
{


    //2. Integral test
    if(mcf.isIntegralAvailable()){
        cout << "   TEST: Integral correctness " << endl;
        SimTK::Vector intyTRAPZ = calcTrapzIntegral(mcfSample(0), mcfSample(1), 
                                        mcf.isIntegralComputedLeftToRight());
        //cout << intyTRAPZ << endl;
        //The error of the trapezoidal integration method is
        //no more than (width^2/2)*sup(f'(x)), which we approximate by
        //http://en.wikipedia.org/wiki/Numerical_integration#Conservative_.28a
        //_priori.29_error_estimation

        //Compute the width between each of the samples, because the width
        //of this sample dictates how accurate the numerical derivative is
        SimTK::Vector xWidth(mcfSample.nrow());

        double xWidthMax = 0;
        for(int i=0; i<mcfSample.nrow()-1; i++){
            xWidth(i) = mcfSample(i+1,0)-mcfSample(i,0);
            if(xWidth(i) > xWidthMax){
                xWidthMax = xWidth(i);
            }

        }
        xWidth(mcfSample.nrow()-1) = xWidth(mcfSample.nrow()-2);

        //cout << xWidth << endl;
        //cout <<endl;

        SimTK::Vector intyCumError(xWidth.size());
        double supdf = 0;

        if(mcf.isIntegralComputedLeftToRight()){
            //Get the vector of accumulated errors left to right
            intyCumError(0) = xWidth(0)*xWidth(0)*0.5*mcfSample(0,1);
            for(int i=1; i< intyCumError.nelt(); i++){
                supdf = max(mcfSample(i,1),mcfSample(i-1,1));
                intyCumError(i) = intyCumError(i-1)
                                  + xWidth(i)*xWidth(i)*0.5*supdf;
            }
            //margin of error
            intyCumError = intyCumError*2.0;
            intyCumError += intyCumError(intyCumError.size()-1)*1.05;
            
        }
        else{
            //Get the vector of accumulated errors right to left
            int eidx = intyCumError.size()-1;
            intyCumError(eidx) = xWidth(eidx)*xWidth(eidx)*0.5*mcfSample(eidx,1);
            for(int i=eidx-1; i>=0; i--){
                supdf = max(mcfSample(i,1),mcfSample(i+1,1));
                intyCumError(i) = intyCumError(i+1)
                                  + xWidth(i)*xWidth(i)*0.5*supdf;
            }
            //margin of error
            intyCumError = intyCumError*2.0;
            intyCumError += intyCumError(0)*1.05;
            
        }
        

        //cout << intyCumError << endl;
        //cout << endl;

        double maxError = 0;
        double mxErr = 0;
        double mnErr = 1e10;

        SimTK::Vector error(mcfSample.nrow());
        //double relErrorTol = 0.01;

        int intyCol = 2+6;//mcf.getMaxDerivativeOrder();
        int errCtr = 0;
        for(int i=0; i< intyCumError.nelt(); i++){
            error(i)=abs( intyTRAPZ(i)-mcfSample(i,intyCol) );
            maxError = intyCumError(i);

            if(error(i)>mxErr)
                mxErr=error(i);
            if(mnErr>error(i))
                mnErr=error(i);


            
            if(error(i)>maxError){
                printf("Tol exceeded (%i), %f e-6> %f e-6\n",i,error(i)*1e6,
                     maxError*1e6);
                errCtr++;
            }
        }
        SimTK_TEST(errCtr==0);
        //cout << error << endl;
        printf("   passed: integral agrees with trapz within a factor of 2.05 \n"
               "      of the theoretical accuracy of trapz, with a maximum \n"
               "      error of %fe-6\n",mxErr*1e6);
        cout << endl;
    }

}

/*
 3. The MuscleCurveFunctions function value, first and second derivative curves
    will be numerically tested for continuity.
*/ 
void testMuscleCurveC2Continuity(SmoothSegmentedFunction mcf,
                                       SimTK::Matrix mcfSample)
{
    cout << "   TEST: C2 Continuity " << endl;

    int multC0 = 5;
    int multC1 = 50;
    int multC2 = 100;

    bool c0 = isFunctionContinuous(mcfSample(0), mcf, 0, 1e-6, multC0);
    bool c1 = isFunctionContinuous(mcfSample(0), mcf, 1, 1e-6, multC1);
    bool c2 = isFunctionContinuous(mcfSample(0), mcf, 2, 1e-6, multC2);

    SimTK_TEST(c0);
    SimTK_TEST(c1);
    SimTK_TEST(c2);

    printf( "   passed: C2 continuity established to a multiple\n"
            "           of the next Taylor series error term.\n "
            "           C0,C1, and C2 multiples: %i,%i and %i\n",
                        multC0,multC1,multC2);
    cout << endl;
}

/*
 4. The MuscleCurveFunctions which are supposed to be monotonic will be
    tested for monotonicity.
*/
void testMonotonicity(SimTK::Matrix mcfSample)
{
    cout << "   TEST: Monotonicity " << endl;
    int multEps = 10;
    bool monotonic = isVectorMonotonic(mcfSample(1),10);
    SimTK_TEST(monotonic);
    printf("   passed: curve is monotonic to %i*SimTK::Eps",multEps);
    cout << endl;
}

//______________________________________________________________________________
/**
 * Create a muscle bench marking system. The bench mark consists of a single muscle 
 * spans a distance. The distance the muscle spans can be controlled, as can the 
 * excitation of the muscle.
 */
int main(int argc, char* argv[])
{
    

    try {
        SimTK_START_TEST("Testing SmoothSegmentedFunctionFactory");

        cout << endl;
        cout <<"**************************************************"<<endl;
        cout <<"          TESTING SegmentedQuinticBezierToolkit           "<<endl;
        cout <<"  (a class that SmoothSegmentedFunctionFactory uses)  "<<endl;
        cout <<"**************************************************"<<endl;
        
        testQuinticBezier_DU_DYDX();
        testQuinticBezier_Exceptions();

        //Functions to facilitate manual debugging        
        //sampleQuinticBezierValDeriv();
        //sampleBezierCornerGeneration();
        //generatePrintMuscleCurves();

        //1. Create every kind of curve that SmoothSegmentedFunctionFactory can make
        //2. Test the properties of the curve numerically
        //3. Test that the curve fulfills the contract implied by the 
        //   function that created it

            string filePath = "C:/mjhmilla/Stanford/dev";
            double tolDX = 5e-3;
            double tolDXBig = 1e-2;
            double tolBig = 1e-6;
            double tolSmall = 1e-12;
        ///////////////////////////////////////
        //TENDON CURVE
        ///////////////////////////////////////
            cout <<"**************************************************"<<endl;
            cout <<"TENDON CURVE TESTING                              "<<endl;
            double e0   = 0.04;
            double kiso = 1.5/e0;
            double c    = 0.5;//0.75;    
            double ftoe = 1.0/3.0;
            auto tendonCurve_ptr = std::unique_ptr<SmoothSegmentedFunction>{
                SmoothSegmentedFunctionFactory::
                createTendonForceLengthCurve(e0,
                                             kiso,
                                             ftoe,
                                             c,
                                             true,
                                             "test_tendonCurve")};
            auto& tendonCurve = *tendonCurve_ptr;
            SimTK::Matrix tendonCurveSample
                =tendonCurve.calcSampledMuscleCurve(6,1.0,1+e0);
            //tendonCurve.printMuscleCurveToCSVFile(filePath);

        //0. Test that each curve fulfills its contract at the end points.
            cout << "   Keypoint Testing" << endl;
            SimTK::Vec2 tendonCurveDomain = tendonCurve.getCurveDomain();
            SimTK_TEST_EQ_TOL(tendonCurve.calcValue(tendonCurveDomain(0)), 
                              0, tolSmall);
            SimTK_TEST_EQ_TOL(tendonCurve.calcValue(tendonCurveDomain(1)), 
                             ftoe, tolSmall);

            SimTK_TEST_EQ_TOL(tendonCurve.calcValue(1.0)       ,0.0,tolSmall);  
            SimTK_TEST_EQ_TOL(tendonCurve.calcDerivative(1.0,1),0.0,tolBig);
            SimTK_TEST_EQ_TOL(tendonCurve.calcDerivative(1.0,2),0.0,tolBig);

            SimTK_TEST_EQ_TOL(tendonCurve.calcValue(1+e0)       ,1.0 ,tolSmall);  
            SimTK_TEST_EQ_TOL(tendonCurve.calcDerivative(1+e0,1),kiso,tolBig);
            SimTK_TEST_EQ_TOL(tendonCurve.calcDerivative(1+e0,2),0   ,tolBig);
            cout << "   passed" << endl;
            cout << endl;
        //1. Test each derivative sample for correctness against a numerically
        //   computed version
            testMuscleCurveDerivatives(tendonCurve,tendonCurveSample,tolDXBig);

        //2. Test each integral, where computed for correctness.
            testMuscleCurveIntegral(tendonCurve, tendonCurveSample);

        //3. Test numerically to see if the curve is C2 continuous
            testMuscleCurveC2Continuity(tendonCurve,tendonCurveSample);
        //4. Test for monotonicity where appropriate
            testMonotonicity(tendonCurveSample);

        //5. Testing Exceptions
            cout << endl;
            cout << "   Exception Testing" << endl;
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* tendonCurveEX
                                    = */SmoothSegmentedFunctionFactory::
                  createTendonForceLengthCurve(0,kiso,ftoe,c,true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* tendonCurveEX
                                    = */SmoothSegmentedFunctionFactory::
                  createTendonForceLengthCurve(e0,(1/e0),ftoe,c,true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* tendonCurveEX
                                    = */SmoothSegmentedFunctionFactory::
                  createTendonForceLengthCurve(e0,kiso,ftoe,-0.01,true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* tendonCurveEX
                                    = */SmoothSegmentedFunctionFactory::
                  createTendonForceLengthCurve(e0,kiso,ftoe,1.01,true,"test"));
            cout << "    passed" << endl;

        ///////////////////////////////////////
        //FIBER FORCE LENGTH CURVE
        ///////////////////////////////////////
            cout <<"**************************************************"<<endl;
            cout <<"FIBER FORCE LENGTH CURVE TESTING                  "<<endl;
            double e0f      = 0.6;
            double kisof    = 8.389863790885878;
            double cf       = 0.65;
            double klow     = 0.5*(1.0/e0f);
            auto fiberFLCurve_ptr = std::unique_ptr<SmoothSegmentedFunction>{
                SmoothSegmentedFunctionFactory::
                createFiberForceLengthCurve(0.0,
                                            e0f,
                                            klow,
                                            kisof,
                                            cf,
                                            true,
                                            "test_fiberForceLength")};
            auto& fiberFLCurve = *fiberFLCurve_ptr;

            SimTK::Matrix fiberFLCurveSample 
                            = fiberFLCurve.calcSampledMuscleCurve(6,1.0,1.0+e0f);

        //0. Test that each curve fulfills its contract.
            cout << "   Keypoint Testing" << endl;

            SimTK::Vec2 fiberFLCurveDomain = fiberFLCurve.getCurveDomain();
            SimTK_TEST_EQ_TOL(fiberFLCurveDomain(0), 1, tolSmall);
            SimTK_TEST_EQ_TOL(fiberFLCurveDomain(1), (1+e0f), tolSmall);


            SimTK_TEST_EQ_TOL(fiberFLCurve.calcValue(1.0)       ,0.0,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberFLCurve.calcDerivative(1.0,1),0.0,tolBig);
            SimTK_TEST_EQ_TOL(fiberFLCurve.calcDerivative(1.0,2),0.0,tolBig);

            SimTK_TEST_EQ_TOL(fiberFLCurve.calcValue(1+e0f)       ,1.0 ,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberFLCurve.calcDerivative(1+e0f,1),kisof,tolBig);
            SimTK_TEST_EQ_TOL(fiberFLCurve.calcDerivative(1+e0f,2),0   ,tolBig);
            cout << "   passed" << endl;
            cout << endl;
        //1. Test each derivative sample for correctness against a numerically
        //   computed version
            testMuscleCurveDerivatives(fiberFLCurve,fiberFLCurveSample,tolDX);

        //2. Test each integral, where computed for correctness.
            testMuscleCurveIntegral(fiberFLCurve,fiberFLCurveSample);

        //3. Test numerically to see if the curve is C2 continuous
            testMuscleCurveC2Continuity(fiberFLCurve,fiberFLCurveSample);
        //4. Test for monotonicity where appropriate

            testMonotonicity(fiberFLCurveSample);

        //5. Testing Exceptions
            cout << endl;
            cout << "   Exception Testing" << endl;
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFLCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                  createFiberForceLengthCurve(0.0,0,klow,kisof,cf,true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFLCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                  createFiberForceLengthCurve(0.0,e0f,klow,1/e0f,cf,true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFLCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                  createFiberForceLengthCurve(0.0,e0f,klow,kisof,-0.01,true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFLCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                  createFiberForceLengthCurve(0.0,e0f,klow,kisof,1.01,true,"test"));
            cout << "    passed" << endl;
        ///////////////////////////////////////
        //FIBER COMPRESSIVE FORCE LENGTH
        ///////////////////////////////////////
            cout <<"**************************************************"<<endl;
            cout <<"FIBER COMPRESSIVE FORCE LENGTH CURVE TESTING      "<<endl;


            double lmax = 0.6;
            double kce  = -8.389863790885878;
            double cce  = 0.5;//0.0;
            auto fiberCECurve_ptr = std::unique_ptr<SmoothSegmentedFunction>{
                SmoothSegmentedFunctionFactory::
                createFiberCompressiveForceLengthCurve(lmax,
                                                       kce,
                                                       cce,
                                                       true,
                                                       "test_fiberCompressive"
                                                       "ForceLengthCurve")};
            auto& fiberCECurve = *fiberCECurve_ptr;
            //fiberCECurve.printMuscleCurveToFile("C:/mjhmilla/Stanford/dev"
            //    "/OpenSim_LOCALPROJECTS/MuscleLibrary_Bench_20120210/build");
        SimTK::Matrix fiberCECurveSample 
                            = fiberCECurve.calcSampledMuscleCurve(6,0,lmax);

        //0. Test that each curve fulfills its contract.
            cout << "   Keypoint Testing" << endl;

            SimTK::Vec2 fiberCECurveDomain = fiberCECurve.getCurveDomain();
            SimTK_TEST_EQ_TOL(fiberCECurveDomain(0), 0, tolSmall);
            SimTK_TEST_EQ_TOL(fiberCECurveDomain(1), lmax, tolSmall);

            SimTK_TEST_EQ_TOL(fiberCECurve.calcValue(lmax)       ,0.0,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberCECurve.calcDerivative(lmax,1),0.0,tolBig);
            SimTK_TEST_EQ_TOL(fiberCECurve.calcDerivative(lmax,2),0.0,tolBig);

            SimTK_TEST_EQ_TOL(fiberCECurve.calcValue(0)       ,1.0 ,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberCECurve.calcDerivative(0,1),kce,tolBig);
            SimTK_TEST_EQ_TOL(fiberCECurve.calcDerivative(0,2),0   ,tolBig);
            cout << "   passed" << endl;
            cout << endl;
        //1. Test each derivative sample for correctness against a numerically
        //   computed version
            testMuscleCurveDerivatives(fiberCECurve,fiberCECurveSample,tolDX);

        //2. Test each integral, where computed for correctness.
            testMuscleCurveIntegral(fiberCECurve,fiberCECurveSample);

        //3. Test numerically to see if the curve is C2 continuous
            testMuscleCurveC2Continuity(fiberCECurve,fiberCECurveSample);
        //4. Test for monotonicity where appropriate

            testMonotonicity(fiberCECurveSample);
        //5. Testing Exceptions
            cout << endl;
            cout << "   Exception Testing" << endl;
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCECurveEX 
                                    = */SmoothSegmentedFunctionFactory::
              createFiberCompressiveForceLengthCurve(0,kce,cce,true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCECurveEX 
                                    = */SmoothSegmentedFunctionFactory::
              createFiberCompressiveForceLengthCurve(lmax,-1/lmax,cce,true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCECurveEX 
                                    = */SmoothSegmentedFunctionFactory::
              createFiberCompressiveForceLengthCurve(lmax,kce,-0.01,true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCECurveEX 
                                    = */SmoothSegmentedFunctionFactory::
              createFiberCompressiveForceLengthCurve(lmax,kce,1.01,true,"test"));
            cout << "    passed" << endl;

        ///////////////////////////////////////
        //FIBER COMPRESSIVE PHI CURVE
        ///////////////////////////////////////
            cout <<"**************************************************"<<endl;
            cout <<"FIBER COMPRESSIVE FORCE PHI CURVE TESTING      "<<endl;

            double phi0 = (SimTK::Pi/2)*(1.0/2.0);
            double phi1 = SimTK::Pi/2;
            double kphi  = 8.389863790885878;
            double cphi  = 0.0;  
            auto fiberCEPhiCurve_ptr = std::unique_ptr<SmoothSegmentedFunction>{
                SmoothSegmentedFunctionFactory::          
                createFiberCompressiveForcePennationCurve(phi0,
                                                          kphi,
                                                          cphi,
                                                          true,
                                                          "test_fiberCompress"
                                                          "iveForcePennationC"
                                                          "urve")};
            auto& fiberCEPhiCurve = *fiberCEPhiCurve_ptr;
        
            SimTK::Matrix fiberCEPhiCurveSample 
                            = fiberCEPhiCurve.calcSampledMuscleCurve(6,phi0,phi1);

        //0. Test that each curve fulfills its contract.
            cout << "   Keypoint Testing" << endl;

            SimTK_TEST_EQ_TOL(fiberCEPhiCurve.calcValue(phi0)       ,0.0,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberCEPhiCurve.calcDerivative(phi0,1),0.0,tolBig);
            SimTK_TEST_EQ_TOL(fiberCEPhiCurve.calcDerivative(phi0,2),0.0,tolBig);

            SimTK_TEST_EQ_TOL(fiberCEPhiCurve.calcValue(phi1)       ,1.0 ,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberCEPhiCurve.calcDerivative(phi1,1),kphi,tolBig);
            SimTK_TEST_EQ_TOL(fiberCEPhiCurve.calcDerivative(phi1,2),0   ,tolBig);
            cout << "   passed" << endl;
            cout << endl;
        //1. Test each derivative sample for correctness against a numerically
        //   computed version
            testMuscleCurveDerivatives(fiberCEPhiCurve,fiberCEPhiCurveSample,tolDX);

        //2. Test each integral, where computed for correctness.
            testMuscleCurveIntegral(fiberCEPhiCurve,fiberCEPhiCurveSample);

        //3. Test numerically to see if the curve is C2 continuous
            testMuscleCurveC2Continuity(fiberCEPhiCurve,fiberCEPhiCurveSample);
        //4. Test for monotonicity where appropriate
            testMonotonicity(fiberCEPhiCurveSample);
        //5. Testing Exceptions
            cout << endl;
            cout << "   Exception Testing" << endl;
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCEPhiCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberCompressiveForcePennationCurve(0,kphi,cphi,
                                                          true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCEPhiCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberCompressiveForcePennationCurve(SimTK::Pi/2,kphi,cphi,
                                                          true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCEPhiCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                    createFiberCompressiveForcePennationCurve(phi0,
                    1.0/(SimTK::Pi/2-phi0),cphi, true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCEPhiCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberCompressiveForcePennationCurve(phi0,kphi,-0.01,
                                                          true,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCEPhiCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberCompressiveForcePennationCurve(phi0,kphi,1.01,
                                                          true,"test"));
            cout << "    passed" << endl;

        ///////////////////////////////////////
        //FIBER COMPRESSIVE COSPHI CURVE
        ///////////////////////////////////////
            cout <<"**************************************************"<<endl;
            cout <<"FIBER COMPRESSIVE FORCE COSPHI CURVE TESTING      "<<endl;

            double cosPhi0 = cos( (80.0/90.0)*SimTK::Pi/2);
            double kcosPhi  = -1.2/(cosPhi0);
            double ccosPhi  = 0.5;
            auto fiberCECosPhiCurve_ptr =
                std::unique_ptr<SmoothSegmentedFunction>{
                SmoothSegmentedFunctionFactory::
                createFiberCompressiveForceCosPennationCurve(cosPhi0,
                                                             kcosPhi,
                                                             ccosPhi,
                                                             true,
                                                             "test_fiberCompre"
                                                             "ssiveForceCosPen"
                                                             "nationCurve")};
            auto& fiberCECosPhiCurve = *fiberCECosPhiCurve_ptr;

            SimTK::Matrix fiberCECosPhiCurveSample 
                            = fiberCECosPhiCurve.calcSampledMuscleCurve(6,0,cosPhi0);

        //0. Test that each curve fulfills its contract.
            cout << "   Keypoint Testing" << endl;

            SimTK_TEST_EQ_TOL(fiberCECosPhiCurve.calcValue(cosPhi0),0.0,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberCECosPhiCurve.calcDerivative(cosPhi0,1),0.0,tolBig);
            SimTK_TEST_EQ_TOL(fiberCECosPhiCurve.calcDerivative(cosPhi0,2),0.0,tolBig);

            SimTK_TEST_EQ_TOL(fiberCECosPhiCurve.calcValue(0)       ,1.0 ,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberCECosPhiCurve.calcDerivative(0,1),kcosPhi,tolBig);
            SimTK_TEST_EQ_TOL(fiberCECosPhiCurve.calcDerivative(0,2),0   ,tolBig);
            cout << "   passed" << endl;
            cout << endl;
        //1. Test each derivative sample for correctness against a numerically
        //   computed version
            testMuscleCurveDerivatives(fiberCECosPhiCurve,fiberCECosPhiCurveSample,tolDX);

        //2. Test each integral, where computed for correctness.
            testMuscleCurveIntegral(fiberCECosPhiCurve,fiberCECosPhiCurveSample);

        //3. Test numerically to see if the curve is C2 continuous
            testMuscleCurveC2Continuity(fiberCECosPhiCurve,fiberCECosPhiCurveSample);
        //4. Test for monotonicity where appropriate

            testMonotonicity(fiberCECosPhiCurveSample);
        //5. Test exceptions
            cout << endl;
        cout << "   Exception Testing" << endl;
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCECosPhiCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberCompressiveForceCosPennationCurve(0,kcosPhi,
                                                        ccosPhi, true,"test"));

            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCECosPhiCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberCompressiveForceCosPennationCurve(1,kcosPhi,
                                                        ccosPhi, true,"test"));

            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCECosPhiCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberCompressiveForceCosPennationCurve(cosPhi0,1/kcosPhi,
                                                        ccosPhi, true,"test"));

            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCECosPhiCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberCompressiveForceCosPennationCurve(cosPhi0,kcosPhi,
                                                        -0.01, true,"test"));

            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberCECosPhiCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberCompressiveForceCosPennationCurve(cosPhi0,kcosPhi,
                                                        1.01, true,"test"));
            cout << "    passed" << endl;
        ///////////////////////////////////////
        //FIBER FORCE-VELOCITY CURVE
        ///////////////////////////////////////
            cout <<"**************************************************"<<endl;
            cout <<"FIBER FORCE VELOCITY CURVE TESTING                "<<endl;

            double fmaxE = 1.8;
            double dydxC = 0.1;
            double dydxNearC = 0.15;
            double dydxE = 0.1;
            double dydxNearE = 0.1+0.0001;
            double dydxIso= 5;
            double concCurviness = 0.1;
            double eccCurviness = 0.75;

            auto fiberFVCurve_ptr = std::unique_ptr<SmoothSegmentedFunction>{
                SmoothSegmentedFunctionFactory::
                createFiberForceVelocityCurve(fmaxE, 
                                              dydxC,
                                              dydxNearC, 
                                              dydxIso,
                                              dydxE,
                                              dydxNearE,
                                              concCurviness,
                                              eccCurviness,
                                              false,
                                              "test_fiberForceVelocityCurve")};
            auto& fiberFVCurve = *fiberFVCurve_ptr;
            //fiberFVCurve.printMuscleCurveToCSVFile(filePath);

            SimTK::Matrix fiberFVCurveSample 
                            = fiberFVCurve.calcSampledMuscleCurve(6,-1.0,1.0);

        //0. Test that each curve fulfills its contract.
            cout << "   Keypoint Testing" << endl;

            SimTK_TEST_EQ_TOL(fiberFVCurve.calcValue(-1),0.0,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberFVCurve.calcDerivative(-1,1),dydxC,tolBig);
            SimTK_TEST_EQ_TOL(fiberFVCurve.calcDerivative(-1,2),0.0,tolBig);

            SimTK_TEST_EQ_TOL(fiberFVCurve.calcValue(0),1.0,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberFVCurve.calcDerivative(0,1),dydxIso,tolBig);
            SimTK_TEST_EQ_TOL(fiberFVCurve.calcDerivative(0,2),0.0,tolBig);

            SimTK_TEST_EQ_TOL(fiberFVCurve.calcValue(1),fmaxE,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberFVCurve.calcDerivative(1,1),dydxE,tolBig);
            SimTK_TEST_EQ_TOL(fiberFVCurve.calcDerivative(1,2),0.0,tolBig);

            cout << "   passed" << endl;
            cout << endl;
        //1. Test each derivative sample for correctness against a numerically
        //   computed version
            testMuscleCurveDerivatives(fiberFVCurve,fiberFVCurveSample,tolDX);

        //2. Test each integral, where computed for correctness.
            testMuscleCurveIntegral(fiberFVCurve,fiberFVCurveSample);

        //3. Test numerically to see if the curve is C2 continuous
            testMuscleCurveC2Continuity(fiberFVCurve,fiberFVCurveSample);
        //4. Test for monotonicity where appropriate

            testMonotonicity(fiberFVCurveSample);
        //5. Exception testing
            cout << endl;    
            cout << "   Exception Testing" << endl;
                
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityCurve(1, 
                                dydxC,dydxNearC,dydxIso, dydxE, dydxNearE, 
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityCurve(fmaxE, 
                                -0.01,dydxNearC,dydxIso, dydxE,  dydxNearE,
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityCurve(fmaxE, 
                                1.01, dydxNearC, dydxIso, dydxE,  dydxNearE,
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityCurve(fmaxE, 
                                dydxC, dydxNearC, 1.0, dydxE, dydxNearE, 
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityCurve(fmaxE, 
                                dydxC, dydxNearC, dydxIso, -0.01, dydxNearE, 
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityCurve(fmaxE, 
                                dydxC, dydxNearC, dydxIso, (fmaxE-1), dydxNearE, 
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityCurve(fmaxE, 
                                dydxC, dydxNearC, dydxIso, dydxE, dydxNearE, 
                                -0.01,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityCurve(fmaxE, 
                                dydxC, dydxNearC, dydxIso, dydxE, dydxNearE, 
                                1.01,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityCurve(fmaxE, 
                                dydxC, dydxNearC, dydxIso, dydxE, dydxNearE, 
                                concCurviness,  -0.01,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityCurve(fmaxE, 
                                dydxC, dydxNearC, dydxIso, dydxE, dydxNearE, 
                                1.01,  1.01,false,"test"));
            
            cout << "    passed" << endl;

        ///////////////////////////////////////
        //FIBER FORCE-VELOCITY INVERSE CURVE
        ///////////////////////////////////////
            cout <<"**************************************************"<<endl;
            cout <<"FIBER FORCE VELOCITY INVERSE CURVE TESTING        "<<endl;

            auto fiberFVInvCurve_ptr = std::unique_ptr<SmoothSegmentedFunction>{
                SmoothSegmentedFunctionFactory::
                createFiberForceVelocityInverseCurve(fmaxE,
                                                     dydxC,
                                                     dydxNearC, 
                                                     dydxIso, 
                                                     dydxE,
                                                     dydxNearE,
                                                     concCurviness,
                                                     eccCurviness,
                                                     false,
                                                     "test_fiberForceVelocity"
                                                     "InverseCurve")};
            auto& fiberFVInvCurve = *fiberFVInvCurve_ptr;
            //fiberFVInvCurve.printMuscleCurveToFile(filePath);

            SimTK::Matrix fiberFVInvCurveSample 
                            = fiberFVInvCurve.calcSampledMuscleCurve(6,0,fmaxE);

        //0. Test that each curve fulfills its contract.
            cout << "   Keypoint Testing" << endl;

            SimTK_TEST_EQ_TOL(fiberFVInvCurve.calcValue(0),-1,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberFVInvCurve.calcDerivative(0,1),1/dydxC,tolBig);
            SimTK_TEST_EQ_TOL(fiberFVInvCurve.calcDerivative(0,2),0.0,tolBig);
                                    
            SimTK_TEST_EQ_TOL(fiberFVInvCurve.calcValue(1),0,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberFVInvCurve.calcDerivative(1,1),1/dydxIso,tolBig);
            SimTK_TEST_EQ_TOL(fiberFVInvCurve.calcDerivative(1,2),0.0,tolBig);
                                     
            SimTK_TEST_EQ_TOL(fiberFVInvCurve.calcValue(fmaxE),1,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberFVInvCurve.calcDerivative(fmaxE,1),1/dydxE,tolBig);
            SimTK_TEST_EQ_TOL(fiberFVInvCurve.calcDerivative(fmaxE,2),0.0,tolBig);

            cout << "   passed" << endl;
            cout << endl;
        //1. Test each derivative sample for correctness against a numerically
        //   computed version
            testMuscleCurveDerivatives(fiberFVInvCurve,fiberFVInvCurveSample,tolDX);

        //2. Test each integral, where computed for correctness.
            testMuscleCurveIntegral(fiberFVInvCurve,fiberFVInvCurveSample);

        //3. Test numerically to see if the curve is C2 continuous
            testMuscleCurveC2Continuity(fiberFVInvCurve,fiberFVInvCurveSample);
        //4. Test for monotonicity where appropriate

            testMonotonicity(fiberFVInvCurveSample);

        //5. Testing the exceptions

            //5. Exception testing
            cout << endl;
                cout << "   Exception Testing" << endl;
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityInverseCurve(1, 
                                dydxC, dydxNearC, dydxIso, dydxE, dydxNearE, 
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityInverseCurve(fmaxE, 
                                0,  dydxNearC, dydxIso, dydxE, dydxNearE,
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityInverseCurve(fmaxE, 
                                1.01,  dydxNearC, dydxIso, dydxE, dydxNearE,
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityInverseCurve(fmaxE, 
                                dydxC,  dydxNearC, 1.0, dydxE, dydxNearE,
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityInverseCurve(fmaxE, 
                                dydxC,  dydxNearC, dydxIso, 0, dydxNearE,
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityInverseCurve(fmaxE, 
                                dydxC,  dydxNearC, dydxIso, (fmaxE-1), dydxNearE,
                                concCurviness,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityInverseCurve(fmaxE, 
                                dydxC,  dydxNearC, dydxIso, dydxE, dydxNearE,
                                -0.01,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityInverseCurve(fmaxE, 
                                dydxC,  dydxNearC, dydxIso, dydxE, dydxNearE,
                                1.01,  eccCurviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityInverseCurve(fmaxE, 
                                dydxC,  dydxNearC, dydxIso, dydxE, dydxNearE,
                                concCurviness,  -0.01,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* fiberFVCurveEX 
                                    = */SmoothSegmentedFunctionFactory::
                createFiberForceVelocityInverseCurve(fmaxE, 
                                dydxC,  dydxNearC, dydxIso, dydxE, dydxNearE,
                                1.01,  1.01,false,"test"));
            
            cout << "    passed" << endl;


        //6. Testing the inverse of the curve - is it really an inverse?
            cout << endl;
            cout << "   TEST: Inverse correctness:fv(fvinv(fv)) = fv" << endl;
            

            double fv = 0;
            double dlce = 0;
            double fvCalc = 0;
            double fvErr = 0;
            double fvErrMax = 0;
            for(int i = 0; i < fiberFVInvCurveSample.nrow(); i++){
                fv = fiberFVCurveSample(i,0);
                dlce = fiberFVInvCurve.calcValue(fv);
                fvCalc = fiberFVCurve.calcValue(dlce);
                fvErr = abs(fv-fvCalc);
                if(fvErrMax < fvErr)
                    fvErrMax = fvErr;

                SimTK_TEST( fvErr < tolBig);
            }
            printf("   passed with a maximum error of %fe-12",fvErrMax*1e12);
        ///////////////////////////////////////
        //FIBER ACTIVE FORCE-LENGTH CURVE
        ///////////////////////////////////////
            cout << endl;
            cout << endl;
            cout <<"**************************************************"<<endl;
            cout <<"FIBER ACTIVE FORCE LENGTH CURVE TESTING        "<<endl;
            double lce0 = 0.4;
            double lce1 = 0.75;
            double lce2 = 1;
            double lce3 = 1.6;
            double shoulderVal  = 0.05;
            double plateauSlope = 0.75;//0.75;
            double curviness    = 0.75;
            auto fiberfalCurve_ptr = std::unique_ptr<SmoothSegmentedFunction>{
                SmoothSegmentedFunctionFactory::
                createFiberActiveForceLengthCurve(lce0,
                                                  lce1,
                                                  lce2,
                                                  lce3, 
                                                  shoulderVal,
                                                  plateauSlope,
                                                  curviness,
                                                  false,
                                                  "test_fiberActiveForceLength"
                                                  "Curve")};
            auto& fiberfalCurve = *fiberfalCurve_ptr;
            //fiberfalCurve.printMuscleCurveToCSVFile(filePath);


            SimTK::Matrix fiberfalCurveSample 
                            = fiberfalCurve.calcSampledMuscleCurve(6,0,lce3);

        //0. Test that each curve fulfills its contract.
            cout << "   Keypoint Testing" << endl;

            SimTK_TEST_EQ_TOL(fiberfalCurve.calcValue(lce0),shoulderVal,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberfalCurve.calcDerivative(lce0,1),0,tolBig);
            SimTK_TEST_EQ_TOL(fiberfalCurve.calcDerivative(lce0,2),0.0,tolBig);
              
            //lce2 isn't the location of the end of a quintic Bezier curve
            //so I can't actually do any testing on this point.
            //SimTK_TEST_EQ_TOL(fiberfalCurve.calcValue(lce0),shoulderVal,tolSmall);  
            //SimTK_TEST_EQ_TOL(fiberfalCurve.calcDerivative(lce2,1),plateauSlope,tolBig);
            //SimTK_TEST_EQ_TOL(fiberfalCurve.calcDerivative(lce2,2),0.0,tolBig);

            SimTK_TEST_EQ_TOL(fiberfalCurve.calcValue(lce2),1,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberfalCurve.calcDerivative(lce2,1),0,tolBig);
            SimTK_TEST_EQ_TOL(fiberfalCurve.calcDerivative(lce2,2),0.0,tolBig);

            SimTK_TEST_EQ_TOL(fiberfalCurve.calcValue(lce3),shoulderVal,tolSmall);  
            SimTK_TEST_EQ_TOL(fiberfalCurve.calcDerivative(lce3,1),0,tolBig);
            SimTK_TEST_EQ_TOL(fiberfalCurve.calcDerivative(lce3,2),0.0,tolBig);

            cout << "   passed" << endl;
            cout << endl;
        //1. Test each derivative sample for correctness against a numerically
        //   computed version
            testMuscleCurveDerivatives(fiberfalCurve,fiberfalCurveSample,tolDX);

        //2. Test each integral, where computed for correctness.
            testMuscleCurveIntegral(fiberfalCurve,fiberfalCurveSample);

        //3. Test numerically to see if the curve is C2 continuous
            testMuscleCurveC2Continuity(fiberfalCurve,fiberfalCurveSample);

            //fiberfalCurve.MuscleCurveToCSVFile("C:/mjhmilla/Stanford/dev");
       
        //4. Exception Testing
            cout << endl;
            cout << "    Exception Testing" << endl;
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* 
                                    fiberfalCurveEX = */
                                  SmoothSegmentedFunctionFactory::
                createFiberActiveForceLengthCurve(lce0, lce0, lce2, lce3, 
                      shoulderVal, plateauSlope, curviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* 
                                    fiberfalCurveEX = */
                                  SmoothSegmentedFunctionFactory::
                createFiberActiveForceLengthCurve(lce0, lce1, lce1, lce3, 
                      shoulderVal, plateauSlope, curviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* 
                                    fiberfalCurveEX = */
                                  SmoothSegmentedFunctionFactory::
                createFiberActiveForceLengthCurve(lce0, lce1, lce2, lce2, 
                      shoulderVal, plateauSlope, curviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* 
                                    fiberfalCurveEX = */
                                  SmoothSegmentedFunctionFactory::
                createFiberActiveForceLengthCurve(lce0, lce1, lce2, lce3, 
                      -0.01, plateauSlope, curviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* 
                                    fiberfalCurveEX = */
                                  SmoothSegmentedFunctionFactory::
                createFiberActiveForceLengthCurve(lce0, lce1, lce2, lce3, 
                      shoulderVal, -0.01, curviness,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* 
                                    fiberfalCurveEX = */
                                  SmoothSegmentedFunctionFactory::
                createFiberActiveForceLengthCurve(lce0, lce1, lce2, lce3, 
                      shoulderVal, plateauSlope, -0.01,false,"test"));
            SimTK_TEST_MUST_THROW(/*SmoothSegmentedFunction* 
                                    fiberfalCurveEX = */
                                  SmoothSegmentedFunctionFactory::
                createFiberActiveForceLengthCurve(lce0, lce1, lce2, lce3, 
                      shoulderVal, plateauSlope, 1.01,false,"test"));
            cout << "    passed"<<endl;

                    ///////////////////////////////////////
        //FIBER COMPRESSIVE PHI CURVE
        ///////////////////////////////////////
            cout <<"**************************************************"<<endl;
            cout <<"SmoothSegmentedFunction Exception Testing     "<<endl;

            //calcValue doesn't throw an exception in a way a user can trigger
            //calcDerivative ... ditto
            
            //This function does not have an integral curve
            SimTK_TEST_MUST_THROW(/*double tst = */fiberfalCurve.calcIntegral(0.0));

            //isIntegralAvailable doesn't throw an exception
            //isIntegralComputedLeftToRight doesn't throw an exception
            //getName doesn't throw an exception
            //getCurveDomain doesn't throw an exception

            //printMuscleCurveToCSVFile should throw one when given a bad path
            SimTK_TEST_MUST_THROW(
                fiberfalCurve.printMuscleCurveToCSVFile("C:/aBadPath",0,2.0));
            //fiberfalCurve.printMuscleCurveToCSVFile("C:/mjhmilla/Stanford/dev");
            cout << "    passed"<<endl;
        SimTK_END_TEST();

    }
    catch (const std::exception& ex)
    {
        cout << ex.what() << endl;
        cin.get();      
        return 1;
    }
    catch (...)
    {
        cout << "UNRECOGNIZED EXCEPTION" << endl;
        cin.get();
        return 1;
    }

    

    cout << "\ntest of SmoothSegmentedFunctionFactory completed successfully.\n";
    return 0;
}

