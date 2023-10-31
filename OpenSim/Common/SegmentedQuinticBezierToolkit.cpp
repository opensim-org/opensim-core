/* -------------------------------------------------------------------------- *
 *                OpenSim:  SegmentedQuinticBezierToolkit.cpp                 *
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
//=============================================================================
// INCLUDES
//=============================================================================
#include "SegmentedQuinticBezierToolkit.h"
#include <fstream>


//=============================================================================
// STATICS
//=============================================================================
using namespace SimTK;
using namespace OpenSim;
using namespace std;


 //The number of knot points to use to sample each Bezier corner section.
static constexpr int NUM_SAMPLE_PTS = 100;
static_assert(NUM_SAMPLE_PTS>0, "SegmentedQuinticBezierToolkit::NUM_SAMPLE_PTS must be larger than zero.");

/**
* This function will print cvs file of the column vector col0 and the matrix data
*
* @params col0: A vector that must have the same number of rows as the data matrix
*               This column vector is printed as the first column
* @params data: A matrix of data
* @params filename: The name of the file to print
*/
void SegmentedQuinticBezierToolkit::printMatrixToFile(
    const SimTK::Vector& col0,
    const SimTK::Matrix& data,
    const std::string& filename)
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

void SegmentedQuinticBezierToolkit::printBezierSplineFitCurves(
    const SimTK::Function_<double>& curveFit,
    const SimTK::Array_<SimTK::Vec6>& ctrlPtsX,
    const SimTK::Array_<SimTK::Vec6>& ctrlPtsY,
    const SimTK::Vector& xVal,
    const SimTK::Vector& yVal,
    const std::string& filename)
{

    SimTK_ERRCHK_ALWAYS(ctrlPtsX.size() == ctrlPtsY.size(),
        "SegmentedQuinticBezierToolkit::printBezierSplineFitCurves",
        "Error: X and Y control points must have same number of elements");

        const int nbezier = ctrlPtsX.size();
        const int rows = (NUM_SAMPLE_PTS - 1) * nbezier + 1;

        SimTK::Vector y1Val(rows);
        SimTK::Vector y2Val(rows);

        SimTK::Vector ySVal(rows);
        SimTK::Vector y1SVal(rows);
        SimTK::Vector y2SVal(rows);

        SimTK::Matrix printMatrix(rows,6);

        SimTK::Vector tmp(1);
        std::vector<int> deriv1(1);
        std::vector<int> deriv2(2);

        deriv1[0] = 0;
        deriv2[0] = 0;
        deriv2[1] = 0;
        for(int j=0; j < nbezier ; j++)
        {
            const int offset = (j > 0)? 1: 0;

            for(int i=0; i<NUM_SAMPLE_PTS-offset; i++)
            {
              const int oidx = i + j*(NUM_SAMPLE_PTS-offset) + 1;

              const double u = ( (double)(i+offset) )/( (double)(NUM_SAMPLE_PTS-1) );
              y1Val(oidx) = calcQuinticBezierCurveDerivDYDX(u, ctrlPtsX[j], ctrlPtsY[j], 1);
              y2Val(oidx) = calcQuinticBezierCurveDerivDYDX(u, ctrlPtsX[j], ctrlPtsY[j], 2);

              tmp(0) = xVal(oidx);
              ySVal(oidx) = curveFit.calcValue( tmp );

              y1SVal(oidx) = curveFit.calcDerivative(deriv1,tmp);
              y2SVal(oidx) = curveFit.calcDerivative(deriv2,tmp);

              printMatrix(oidx,0) = yVal(oidx);
              printMatrix(oidx,1) = y1Val(oidx);
              printMatrix(oidx,2) = y2Val(oidx);
              printMatrix(oidx,3) = ySVal(oidx);
              printMatrix(oidx,4) = y1SVal(oidx);
              printMatrix(oidx,5) = y2SVal(oidx);
            }
        }
        printMatrixToFile(xVal,printMatrix,filename);

}

//=============================================================================
// Bezier Corner Element Fitting Function
//=============================================================================

/*Detailed Computational Costs
        Divisions   Multiplication  Additions   Assignments
        1           13              9              23
*/
SegmentedQuinticBezierToolkit::ControlPointsXY
    SegmentedQuinticBezierToolkit::calcQuinticBezierCornerControlPoints(
        double x0,
        double y0,
        double dydx0,
        double x1,
        double y1,
        double dydx1,
        double curviness)
{
    SimTK_ERRCHK_ALWAYS( (curviness>=0 && curviness <= 1) , 
        "SegmentedQuinticBezierToolkit::calcQuinticBezierCornerControlPoints", 
        "Error: double argument curviness must be between 0.0 and 1.0.");


    //1. Calculate the location where the two lines intersect
    // (x-x0)*dydx0 + y0 = (x-x1)*dydx1 + y1
    //   x*(dydx0-dydx1) = y1-y0-x1*dydx1+x0*dydx0
    //                 x = (y1-y0-x1*dydx1+x0*dydx0)/(dydx0-dydx1);

    double xC = 0;
    double yC = 0;
    double rootEPS = sqrt(SimTK::Eps);
    if(abs(dydx0-dydx1) > rootEPS){
        xC = (y1-y0-x1*dydx1+x0*dydx0)/(dydx0-dydx1);    
    }else{
        xC = (x1+x0)/2;
    }

    yC = (xC-x1)*dydx1 + y1;
    //Check to make sure that the inputs are consistent with a corner, and will
    //not produce an 's' shaped section. To check this we compute the sides of
    //a triangle that is formed by the two points that the user entered, and 
    //also the intersection of the 2 lines the user entered. If the distance
    //between the two points the user entered is larger than the distance from
    //either point to the intersection location, this function will generate a
    //'C' shaped curve. If this is not true, an 'S' shaped curve will result, 
    //and this function should not be used.

    double xCx0 = (xC-x0);
    double yCy0 = (yC-y0);
    double xCx1 = (xC-x1);
    double yCy1 = (yC-y1);
    double x0x1 = (x1-x0);
    double y0y1 = (y1-y0);

    double a = xCx0*xCx0 + yCy0*yCy0;
    double b = xCx1*xCx1 + yCy1*yCy1;
    double c = x0x1*x0x1 + y0y1*y0y1;

    //This error message needs to be better.
    SimTK_ERRCHK_ALWAYS( ((c > a) && (c > b)), 
        "SegmentedQuinticBezierToolkit::calcQuinticBezierCornerControlPoints", 
        "The intersection point for the two lines defined by the input"
        "parameters must be consistent with a C shaped corner.");

    /*
    //New mid point control code, which spreads the curve out more gradually    
    double deltaX   = (xC-xyPts(0,0));    
    double deltaY   = (yC-xyPts(0,1));
    double sinCPi   = sin(curviness*SimTK::Pi);

    //First two midpoints
    xyPts(1,0) = x0 + (curviness - 0.25*sinCPi)*deltaX;
    xyPts(1,1) = y0 + (curviness - 0.25*sinCPi)*deltaY;
    xyPts(2,0) = x0 + (curviness + 0.25*sinCPi)*deltaX;
    xyPts(2,1) = y0 + (curviness + 0.25*sinCPi)*deltaY;

    //Second two midpoints
    deltaX   = (xC-xyPts(5,0));    
    deltaY   = (yC-xyPts(5,1));

    xyPts(3,0) = xyPts(5,0) + (curviness - 0.25*sinCPi)*deltaX;
    xyPts(3,1) = xyPts(5,1) + (curviness - 0.25*sinCPi)*deltaY;
    xyPts(4,0) = xyPts(5,0) + (curviness + 0.25*sinCPi)*deltaX;
    xyPts(4,1) = xyPts(5,1) + (curviness + 0.25*sinCPi)*deltaY;
    */
    
    //Original code - leads to 2 localized corners
    double x0_mid = x0 + curviness*(xCx0);
    double y0_mid = y0 + curviness*(yCy0);

    //Second two midpoints
    double x1_mid = x1 + curviness*(xCx1);
    double y1_mid = y1 + curviness*(yCy1);
    
    SimTK::Vec6 xPts(x0, x0_mid, x0_mid, x1_mid, x1_mid, x1);
    SimTK::Vec6 yPts(y0, y0_mid, y0_mid, y1_mid, y1_mid, y1);

    return SegmentedQuinticBezierToolkit::ControlPointsXY{xPts, yPts};
}

//=============================================================================
// BASIC QUINTIC BEZIER EVALUATION FUNCTIONS
//=============================================================================

/* 
Multiplications     Additions   Assignments
21                  20          13
*/
double SegmentedQuinticBezierToolkit::calcQuinticBezierCurveVal(
    double u,
    const SimTK::Vec6& pts)
{
    return calcQuinticBezierCurveDerivU(u, pts, 0);
}

/*
Detailed Computational Costs
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

*/
double SegmentedQuinticBezierToolkit::calcQuinticBezierCurveDerivDYDX(
    double u,
    const SimTK::Vec6& xpts,
    const SimTK::Vec6& ypts,
    int order)
{
    double val = SimTK::NaN;
   
    //Bounds checking on the input
    SimTK_ERRCHK_ALWAYS( (u>=0 && u <= 1) , 
        "SegmentedQuinticBezierToolkit::calcQuinticBezierCurveDerivU", 
        "Error: double argument u must be between 0.0 and 1.0.");

    SimTK_ERRCHK_ALWAYS( (order >= 0),
        "SegmentedQuinticBezierToolkit::calcQuinticBezierCurveDerivU", 
        "Error: order must be greater than zero.");

    SimTK_ERRCHK_ALWAYS( (order <= 6),
        "SegmentedQuinticBezierToolkit::calcQuinticBezierCurveDerivU", 
        "Error: order must be less than, or equal to 6.");

    //std::string localCaller = caller;
    //localCaller.append(".calcQuinticBezierCurveDerivDYDX");
    //Compute the derivative d^n y/ dx^n
     switch(order){
        case 0: // Calculate y.
            {
                double y = calcQuinticBezierCurveDerivU(u, ypts, 0);

                val = y;
            }
            break;
        case 1: // Calculate dy/dx
            { 
                double dxdu =calcQuinticBezierCurveDerivU(u,xpts,1);
                double dydu =calcQuinticBezierCurveDerivU(u,ypts,1);
                double dydx = dydu/dxdu;

                val = dydx;
                //Question: 
                //how is a divide by zero treated? Is SimTK::INF returned?
            }
            break;
        case 2: //Calculate d^2y/dx^2
            { 
               double dxdu  =calcQuinticBezierCurveDerivU(u,xpts,1);
               double dydu  =calcQuinticBezierCurveDerivU(u,ypts,1);
               double d2xdu2=calcQuinticBezierCurveDerivU(u,xpts,2);
               double d2ydu2=calcQuinticBezierCurveDerivU(u,ypts,2);
                
                //Optimized code from Maple - 
                //see MuscleCurveCodeOpt_20120210 for details
                double t1 = 0.1e1 / dxdu;
                double t3 = dxdu*dxdu;//dxdu ^ 2;
                double d2ydx2 = (d2ydu2 * t1 - dydu / t3 * d2xdu2) * t1;

                val = d2ydx2;

            }
            break;
        case 3: //Calculate d^3y/dx^3
            { 
               double dxdu  =calcQuinticBezierCurveDerivU(u,xpts,1);
               double dydu  =calcQuinticBezierCurveDerivU(u,ypts,1);
               double d2xdu2=calcQuinticBezierCurveDerivU(u,xpts,2);
               double d2ydu2=calcQuinticBezierCurveDerivU(u,ypts,2);
               double d3xdu3=calcQuinticBezierCurveDerivU(u,xpts,3);
               double d3ydu3=calcQuinticBezierCurveDerivU(u,ypts,3);

               double t1 = 1 / dxdu;
               double t3 = dxdu*dxdu;//(dxdu ^ 2);
               double t4 = 1 / t3;
               double t11 = d2xdu2*d2xdu2;//(d2xdu2 ^ 2);
               double t14 = (dydu * t4);
               double d3ydx3 = ((d3ydu3 * t1 - 2 * d2ydu2 * t4 * d2xdu2 
                   + 2 * dydu / t3 / dxdu * t11 - t14 * d3xdu3) * t1 
                   - (d2ydu2 * t1 - t14 * d2xdu2) * t4 * d2xdu2) * t1;

                val = d3ydx3;
            }
            break;
        case 4: //Calculate d^4y/dx^4
            { 
               double dxdu  =calcQuinticBezierCurveDerivU(u,xpts,1);
               double dydu  =calcQuinticBezierCurveDerivU(u,ypts,1);
               double d2xdu2=calcQuinticBezierCurveDerivU(u,xpts,2);
               double d2ydu2=calcQuinticBezierCurveDerivU(u,ypts,2);
               double d3xdu3=calcQuinticBezierCurveDerivU(u,xpts,3);
               double d3ydu3=calcQuinticBezierCurveDerivU(u,ypts,3);
               double d4xdu4=calcQuinticBezierCurveDerivU(u,xpts,4);
               double d4ydu4=calcQuinticBezierCurveDerivU(u,ypts,4);

               double t1 = 1 / dxdu;
               double t3 = dxdu*dxdu;//dxdu ^ 2;
               double t4 = 1 / t3;
               double t9 = (0.1e1 / t3 / dxdu);
               double t11 = d2xdu2*d2xdu2;//(d2xdu2 ^ 2);
               double t14 = (d2ydu2 * t4);
               double t17 = t3*t3;//(t3 ^ 2);
               double t23 = (dydu * t9);
               double t27 = (dydu * t4);
               double t37 = d3ydu3 * t1 - 2 * t14 * d2xdu2 + 2 * t23 * t11 
                            - t27 * d3xdu3;
               double t43 = d2ydu2 * t1 - t27 * d2xdu2;
               double t47 = t43 * t4;
               double d4ydx4 = (((d4ydu4 * t1 - 3 * d3ydu3 * t4 * d2xdu2 
                       + 6 * d2ydu2 * t9 * t11 - 3 * t14 * d3xdu3 
                       - 6 * dydu / t17 * t11 * d2xdu2 + 6 * t23 * d2xdu2 * d3xdu3
                       - t27 * d4xdu4) * t1 - 2 * t37 * t4 * d2xdu2 
                       + 2 * t43 * t9 * t11 - t47 * d3xdu3) * t1 
                       - (t37 * t1 - t47 * d2xdu2) * t4 * d2xdu2) * t1;

                
                val = d4ydx4;

            }
            break;
        case 5: 
            { 
               double dxdu  =calcQuinticBezierCurveDerivU(u,xpts,1);
               double dydu  =calcQuinticBezierCurveDerivU(u,ypts,1);
               double d2xdu2=calcQuinticBezierCurveDerivU(u,xpts,2);
               double d2ydu2=calcQuinticBezierCurveDerivU(u,ypts,2);
               double d3xdu3=calcQuinticBezierCurveDerivU(u,xpts,3);
               double d3ydu3=calcQuinticBezierCurveDerivU(u,ypts,3);
               double d4xdu4=calcQuinticBezierCurveDerivU(u,xpts,4);
               double d4ydu4=calcQuinticBezierCurveDerivU(u,ypts,4);
               double d5xdu5=calcQuinticBezierCurveDerivU(u,xpts,5);
               double d5ydu5=calcQuinticBezierCurveDerivU(u,ypts,5);

               double t1 = 1 / dxdu;
               double t3 = dxdu*dxdu;//dxdu ^ 2;
               double t4 = 1 / t3;
               double t9 = (0.1e1 / t3 / dxdu);
               double t11 = d2xdu2*d2xdu2;//(d2xdu2 ^ 2);
               double t14 = (d3ydu3 * t4);
               double t17 = t3*t3;//(t3 ^ 2);
               double t18 = 1 / t17;
               double t20 = (t11 * d2xdu2);
               double t23 = (d2ydu2 * t9);
               double t24 = (d2xdu2 * d3xdu3);
               double t27 = (d2ydu2 * t4);
               double t33 = t11*t11;//(t11 ^ 2);
               double t36 = (dydu * t18);
               double t40 = (dydu * t9);
               double t41 = d3xdu3*d3xdu3;//(d3xdu3 ^ 2);
               double t47 = (dydu * t4);
               double t49 = d5ydu5 * t1 - 4 * d4ydu4 * t4 * d2xdu2 + 12 * d3ydu3 * t9 * t11 - 6 * t14 * d3xdu3 - 24 * d2ydu2 * t18 * t20 + 24 * t23 * t24 - 4 * t27 * d4xdu4 + 24 * dydu / t17 / dxdu * t33 - 36 * t36 * t11 * d3xdu3 + 6 * t40 * t41 + 8 * t40 * d2xdu2 * d4xdu4 - t47 * d5xdu5;
               double t63 = d4ydu4 * t1 - 3 * t14 * d2xdu2 + 6 * t23 * t11 - 3 * t27 * d3xdu3 - 6 * t36 * t20 + 6 * t40 * t24 - t47 * d4xdu4;
               double t73 = d3ydu3 * t1 - 2 * t27 * d2xdu2 + 2 * t40 * t11 - t47 * d3xdu3;
               double t77 = t73 * t4;
               double t82 = d2ydu2 * t1 - t47 * d2xdu2;
               double t86 = t82 * t9;
               double t89 = t82 * t4;
               double t99 = t63 * t1 - 2 * t77 * d2xdu2 + 2 * t86 * t11 - t89 * d3xdu3;
               double t105 = t73 * t1 - t89 * d2xdu2;
               double t109 = t105 * t4;
               double d5ydx5 = (((t49 * t1 - 3 * t63 * t4 * d2xdu2 
                   + 6 * t73 * t9 * t11 - 3 * t77 * d3xdu3 - 6 * t82 * t18 * t20 
                   + 6 * t86 * t24 - t89 * d4xdu4) * t1 - 2 * t99 * t4 * d2xdu2
                   + 2 * t105 * t9 * t11 - t109 * d3xdu3) * t1 
                   - (t99 * t1 - t109 * d2xdu2) * t4 * d2xdu2) * t1;


                val = d5ydx5;

            }
            break;
        case 6: 
            {
               double dxdu  =calcQuinticBezierCurveDerivU(u,xpts,1);
               double dydu  =calcQuinticBezierCurveDerivU(u,ypts,1);
               double d2xdu2=calcQuinticBezierCurveDerivU(u,xpts,2);
               double d2ydu2=calcQuinticBezierCurveDerivU(u,ypts,2);
               double d3xdu3=calcQuinticBezierCurveDerivU(u,xpts,3);
               double d3ydu3=calcQuinticBezierCurveDerivU(u,ypts,3);
               double d4xdu4=calcQuinticBezierCurveDerivU(u,xpts,4);
               double d4ydu4=calcQuinticBezierCurveDerivU(u,ypts,4);
               double d5xdu5=calcQuinticBezierCurveDerivU(u,xpts,5);
               double d5ydu5=calcQuinticBezierCurveDerivU(u,ypts,5);
               double d6xdu6=calcQuinticBezierCurveDerivU(u,xpts,6);
               double d6ydu6=calcQuinticBezierCurveDerivU(u,ypts,6);

               double t1 = dxdu*dxdu;//(dxdu ^ 2);
               double t3 = (0.1e1 / t1 / dxdu);
               double t5 = d2xdu2*d2xdu2;//(d2xdu2 ^ 2);
               double t8 = t1*t1;//(t1 ^ 2);
               double t9 = 1 / t8;
               double t11 = (t5 * d2xdu2);
               double t14 = (d3ydu3 * t3);
               double t15 = (d2xdu2 * d3xdu3);
               double t19 = (0.1e1 / t8 / dxdu);
               double t21 = t5*t5;//(t5 ^ 2);
               double t24 = (d2ydu2 * t9);
               double t25 = (t5 * d3xdu3);
               double t28 = (d2ydu2 * t3);
               double t29 = d3xdu3*d3xdu3;//(d3xdu3 ^ 2);
               double t32 = (d2xdu2 * d4xdu4);
               double t41 = (dydu * t19);
               double t45 = (dydu * t9);
               double t49 = (dydu * t3);
               double t56 = 1 / dxdu;
               double t61 = 1 / t1;
               double t62 = (dydu * t61);
               double t67 = (d4ydu4 * t61);
               double t70 = (d2ydu2 * t61);
               double t73 = (d3ydu3 * t61);
               double t76 = 20 * d4ydu4 * t3 * t5 - 60 * d3ydu3 * t9 * t11 
                   + 60 * t14 * t15 + 120 * d2ydu2 * t19 * t21 - 180 * t24 * t25 
                   + 30 * t28 * t29 + 40 * t28 * t32 
                   - 120 * dydu / t8 / t1 * t21 * d2xdu2 + 240 * t41 *t11*d3xdu3 
                   - 60 * t45 * t5 * d4xdu4 + 20 * t49 * d3xdu3 * d4xdu4 
                   + 10 * t49 * d2xdu2 * d5xdu5 + d6ydu6 * t56 
                   - 90 * t45 * d2xdu2 * t29 - t62 * d6xdu6 
                   - 5 * d5ydu5 * t61 * d2xdu2 - 10 * t67 * d3xdu3 
                   - 5 * t70 * d5xdu5 - 10 * t73 * d4xdu4;

               double t100 = d5ydu5 * t56 - 4 * t67 * d2xdu2 + 12 * t14 * t5 
                   - 6 * t73 * d3xdu3 - 24 * t24 * t11 + 24 * t28 * t15 
                   - 4 * t70 * d4xdu4 + 24 * t41 * t21 - 36 * t45 * t25
                   + 6 * t49 * t29 + 8 * t49 * t32 - t62 * d5xdu5;

               double t116 = d4ydu4 * t56 - 3 * t73 * d2xdu2 + 6 * t28 * t5 
                   - 3 * t70 * d3xdu3 - 6 * t45 * t11 + 6 * t49 * t15 
                   - t62 * d4xdu4;

               double t120 = t116 * t61;
               double t129 = d3ydu3 * t56 - 2 * t70 * d2xdu2 + 2 * t49 * t5 
                   - t62 * d3xdu3;
               double t133 = t129 * t3;
               double t136 = t129 * t61;
               double t141 = d2ydu2 * t56 - t62 * d2xdu2;
               double t145 = t141 * t9;
               double t148 = t141 * t3;
               double t153 = t141 * t61;
               double t155 = t76 * t56 - 4 * t100 * t61 * d2xdu2 
                   + 12 * t116 * t3 * t5 - 6 * t120 * d3xdu3 
                   - 24 * t129 * t9 * t11 + 24 * t133 * t15 - 4 * t136 * d4xdu4 
                   + 24 * t141 * t19 * t21 - 36 * t145 * t25 + 6 * t148 * t29 
                   + 8 * t148 * t32 - t153 * d5xdu5;

               double t169 = t100 * t56 - 3 * t120 * d2xdu2 + 6 * t133 * t5 
                   - 3 * t136 * d3xdu3 - 6 * t145 * t11 + 6 * t148 * t15 
                   - t153 * d4xdu4;

               double t179 = t116 * t56 - 2 * t136 * d2xdu2 + 2 * t148 * t5 
                   - t153 * d3xdu3;

               double t183 = t179 * t61;
               double t188 = t129 * t56 - t153 * d2xdu2;
               double t192 = t188 * t3;
               double t195 = t188 * t61;
               double t205 = t169 * t56 - 2 * t183 * d2xdu2 + 2 * t192 * t5 
                   - t195 * d3xdu3;
               double t211 = t179 * t56 - t195 * d2xdu2;
               double t215 = t211 * t61;
               double d6ydx6 = (((t155 * t56 - 3 * t169 * t61 * d2xdu2 
                   + 6 * t179 * t3 * t5 - 3 * t183 * d3xdu3 - 6 * t188 * t9 *t11 
                   + 6 * t192 * t15 - t195 * d4xdu4) * t56 
                   - 2 * t205 * t61 * d2xdu2 + 2 * t211*t3*t5-t215*d3xdu3)*t56 
                   - (t205 * t56 - t215 * d2xdu2) * t61 * d2xdu2) * t56;


                val = d6ydx6;
            }
            break;
        default:
            val = SimTK::NaN;
     }
     return val;
}

/* Computational Cost Details
        Divisions   Multiplications Additions   Assignments
dx/du               20              19          11
d2x/du2             17              17          9
d3y/du3             14              14          6

*/
double SegmentedQuinticBezierToolkit::calcQuinticBezierCurveDerivU(
    double u,
    const SimTK::Vec6& pts,
    int order)
{
    double val = -1;

    SimTK_ERRCHK_ALWAYS( (u>=0 && u <= 1) , 
        "SegmentedQuinticBezierToolkit::calcQuinticBezierCurveDerivU", 
        "Error: double argument u must be between 0.0 and 1.0.");

    SimTK_ERRCHK_ALWAYS( (order >= 0),
        "SegmentedQuinticBezierToolkit::calcQuinticBezierCurveDerivU",
        "Error: order must be greater than, or equal to 0");

    //Compute the Bezier point
    double p0 = pts(0);
    double p1 = pts(1);
    double p2 = pts(2);
    double p3 = pts(3);
    double p4 = pts(4);
    double p5 = pts(5);

    switch(order){
        case 0:
            {
                double u5 = 1;
                double u4 = u;
                double u3 = u4*u;
                double u2 = u3*u;
                double u1 = u2*u;
                double u0 = u1*u;

                //See lines 1-6 of MuscleCurveCodeOpt_20120210
                double t2 = u1 * 0.5e1;
                double t3 = u2 * 0.10e2;
                double t4 = u3 * 0.10e2;
                double t5 = u4 * 0.5e1;
                double t9 = u0 * 0.5e1;
                double t10 = u1 * 0.20e2;
                double t11 = u2 * 0.30e2;
                double t15 = u0 * 0.10e2;
                val = p0 * (u0 * (-0.1e1) + t2 - t3 + t4 - t5 + u5 * 0.1e1)
                    + p1 * (t9 - t10 + t11 + u3 * (-0.20e2) + t5)
                    + p2 * (-t15 + u1 * 0.30e2 - t11 + t4)
                    + p3 * (t15 - t10 + t3)
                    + p4 * (-t9 + t2) + p5 * u0 * 0.1e1;
            }
            break;
        case 1: 
            {  
                double t1 = u*u;//u ^ 2;
                double t2 = t1*t1;//t1 ^ 2;
                double t4 = t1 * u;
                double t5 = t4 * 0.20e2;
                double t6 = t1 * 0.30e2;
                double t7 = u * 0.20e2;
                double t10 = t2 * 0.25e2;
                double t11 = t4 * 0.80e2;
                double t12 = t1 * 0.90e2;
                double t16 = t2 * 0.50e2;
                val = p0 * (t2 * (-0.5e1) + t5 - t6 + t7 - 0.5e1) 
                    + p1 * (t10 - t11 + t12 + u * (-0.40e2) + 0.5e1) 
                    + p2 * (-t16 + t4 * 0.120e3 - t12 + t7) 
                    + p3 * (t16 - t11 + t6) 
                    + p4 * (-t10 + t5) 
                    + p5 * t2 * 0.5e1;

            }
            break;
        case 2:    
            {
                double t1 = u*u;//u ^ 2;
                double t2 = t1 * u;
                double t4 = t1 * 0.60e2;
                double t5 = u * 0.60e2;
                double t8 = t2 * 0.100e3;
                double t9 = t1 * 0.240e3;
                double t10 = u * 0.180e3;
                double t13 = t2 * 0.200e3;
                val = p0 * (t2 * (-0.20e2) + t4 - t5 + 0.20e2) 
                    + p1 * (t8 - t9 + t10 - 0.40e2) 
                    + p2 * (-t13 + t1 * 0.360e3 - t10 + 0.20e2) 
                    + p3 * (t13 - t9 + t5) 
                    + p4 * (-t8 + t4)
                    + p5 * t2 * 0.20e2;

            }
            break;      
        case 3:   
            {
                double t1 = u*u;//u ^ 2;
                double t3 = u * 0.120e3;
                double t6 = t1 * 0.300e3;
                double t7 = u * 0.480e3;
                double t10 = t1 * 0.600e3;
                val = p0 * (t1 * (-0.60e2) + t3 - 0.60e2) 
                    + p1 * (t6 - t7 + 0.180e3) 
                    + p2 * (-t10 + u * 0.720e3 - 0.180e3) 
                    + p3 * (t10 - t7 + 0.60e2) 
                    + p4 * (-t6 + t3) 
                    + p5 * t1 * 0.60e2;

            }
            break;
        case 4:       
            {
                double t4 = u * 0.600e3;
                double t7 = u * 0.1200e4;
                val = p0 * (u * (-0.120e3) + 0.120e3) 
                    + p1 * (t4 - 0.480e3) 
                    + p2 * (-t7 + 0.720e3) 
                    + p3 * (t7 - 0.480e3) 
                    + p4 * (-t4 + 0.120e3) 
                    + p5 * u * 0.120e3;
            }
            break;
        case 5:    
            {
                val = p0 * (-0.120e3) 
                    + p1 * 0.600e3 
                    + p2 * (-0.1200e4) 
                    + p3 * 0.1200e4 
                    + p4 * (-0.600e3) 
                    + p5 * 0.120e3;
            }
            break;     
        default:
            val=0;

    }

    return val;


}

double SegmentedQuinticBezierToolkit::clampU(double u){
    double uC = u;
    if(u<0.0){
        uC=0;
    }
    if(u>1.0){
        uC=1;
    }
    return uC;
}

/*Detailed Computational Costs
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
*/
double SegmentedQuinticBezierToolkit::calcU(
    double ax,
    const SimTK::Vec6& bezierPtsX,
    const SimTK::Spline& splineUX,
    double tol,
    int maxIter)
{
    SimTK_ERRCHK_ALWAYS( ax >= SimTK::min(bezierPtsX) && ax <= SimTK::max(bezierPtsX),
        "SegmentedQuinticBezierToolkit::calcU", 
        "Error: input ax was not in the domain of the Bezier curve specified \n"
        "by the control points in bezierPtsX.");

    double u = splineUX.calcValue(ax);
    u = clampU(u);
    double f = 0;


    f = calcQuinticBezierCurveVal(u,bezierPtsX)-ax;
    double df= 0;
    double du= 0;
    int iter = 0;
    bool pathologic = false;
    
    //Newton iterate to the desired tolerance
    while(abs(f) > tol && iter < maxIter && pathologic == false){
       //Take a Newton step
        df = calcQuinticBezierCurveDerivU(u,bezierPtsX,1);
        if(abs(df) > 0){
            du = -f/df;
            u  = u + du;
            u  = clampU(u); 
            f = calcQuinticBezierCurveVal(u,bezierPtsX)-ax;
        }else{
            pathologic = true;
        }
        iter++;
    }

    //Check for convergence
    SimTK_ERRCHK2_ALWAYS( (f <= tol), 
        "SegmentedQuinticBezierToolkit::calcU", 
        "Error: desired tolerance of %f on U not met by the Newton iteration."
        " A tolerance of %f was reached.",tol, f);

    SimTK_ERRCHK_ALWAYS( (pathologic==false), 
        "SegmentedQuinticBezierToolkit::calcU", 
        "Error: Newton iteration went pathologic: df = 0 to machine precision.");
      
    
    //Return the value
    return u;
}
/*

Cost: n comparisons, for a quintic Bezier curve with n-spline sections

                Comp    Div     Mult        Add      Assignments
Cost            3*n+2                       1*n      3                         
        
*/
int SegmentedQuinticBezierToolkit::calcIndex(double x, 
                                             const SimTK::Matrix& bezierPtsX)
{
    int idx = 0;
    bool flag_found = false;

    for(int i=0; i<bezierPtsX.ncol(); i++){
        if( x >= bezierPtsX(0,i) && x < bezierPtsX(5,i) ){
            idx = i;
            i = bezierPtsX.ncol();
            flag_found = true;
        }
    }

    //Check if the value x is identically the last point
    if(flag_found == false && x == bezierPtsX(5,bezierPtsX.ncol()-1)){
        idx = bezierPtsX.ncol()-1;
        flag_found = true;
    }

    SimTK_ERRCHK_ALWAYS( (flag_found == true), 
        "SegmentedQuinticBezierToolkit::calcIndex", 
        "Error: A value of x was used that is not within the Bezier curve set.");

    return idx;
}

int SegmentedQuinticBezierToolkit::calcIndex(
    double x,
    const SimTK::Array_<SimTK::Vec6>& bezierPtsX)
{
    int idx = 0;
    bool flag_found = false;

    int n = bezierPtsX.size(); 
    for(int i=0; i<n; i++){
        if( x >= bezierPtsX[i](0) && x < bezierPtsX[i](5) ){
            idx = i;
            flag_found = true;
            break; 
        }
    }

    //Check if the value x is identically the last point
    if(!flag_found && x == bezierPtsX[n-1](5)){
        idx = n-1;
        flag_found = true;
    }

    SimTK_ERRCHK_ALWAYS( (flag_found == true), 
        "SegmentedQuinticBezierToolkit::calcIndex", 
        "Error: A value of x was used that is not within the Bezier curve set.");

    return idx;
}



/*
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

*/
SimTK::Matrix SegmentedQuinticBezierToolkit::calcNumIntBezierYfcnX(
                            const SimTK::Vector& vX, 
                            double ic0, double intAcc, 
                            double uTol, int uMaxIter,
                            const SimTK::Array_<SimTK::Vec6>& ctrlPtsX,
                            const SimTK::Array_<SimTK::Vec6>& ctrlPtsY,
                            const SimTK::Array_<SimTK::Spline>& aSplineUX,
                            bool flag_intLeftToRight,
                            const std::string& caller)
{
    SimTK::Matrix intXY(vX.size(),2);
    BezierData bdata;
        bdata._ctrlPtsX       = ctrlPtsX;
        bdata._ctrlPtsY       = ctrlPtsY;
        bdata._initalValue    = ic0;
        bdata._aArraySplineUX = aSplineUX;
        bdata._uMaxIter       = uMaxIter;
        bdata._uTol           = uTol;
        bdata._flag_intLeftToRight = flag_intLeftToRight;
        bdata._name           = caller;

    //These aren't really times, but I'm perpetuating the SimTK language
    //so that I don't make a mistake
    double startTime = vX(0);
    double endTime   = vX(vX.size()-1);
    
    if(flag_intLeftToRight){
        bdata._startValue = startTime;
    }else{
        bdata._startValue = endTime;
    }

    MySystem sys(bdata);
    State initState = sys.realizeTopology();
    initState.setTime(startTime);


    RungeKuttaMersonIntegrator integ(sys);
    integ.setAccuracy(intAcc);
    integ.setFinalTime(endTime);
    integ.setReturnEveryInternalStep(false);
    integ.initialize(initState);

    int idx = 0;    
    double nextTimeInterval = 0;
    Integrator::SuccessfulStepStatus status;

    while (idx < vX.nelt()) {      
        if(idx < vX.nelt()){
            if(flag_intLeftToRight){
                nextTimeInterval = vX(idx);
            }else{
                nextTimeInterval = endTime-vX(vX.size()-idx-1);
            }
        }
        status=integ.stepTo(nextTimeInterval);

        // Use this for variable step output.
        //status = integ.stepTo(Infinity);

        if (status == Integrator::EndOfSimulation)
            break;
        
        const State& state = integ.getState();

        if(flag_intLeftToRight){
            intXY(idx,0) = nextTimeInterval;
            intXY(idx,1) = (double)state.getZ()[0];                        
        }else{
            intXY(vX.size()-idx-1,0) = vX(vX.size()-idx-1);
            intXY(vX.size()-idx-1,1) = (double)state.getZ()[0];
        }
        idx++;

    }
    //intXY.resizeKeep(idx,2);
    return intXY;
}
