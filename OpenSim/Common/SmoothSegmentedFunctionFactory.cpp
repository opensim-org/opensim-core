/* -------------------------------------------------------------------------- *
 *                OpenSim:  SmoothSegmentedFunctionFactory.cpp                *
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

#include "SmoothSegmentedFunctionFactory.h"
//=============================================================================
// STATICS
//=============================================================================
using namespace SimTK;
using namespace OpenSim;
using namespace std;


//static int NUM_SAMPLE_PTS = 100; //The number of knot points to use to sample
                                //each Bezier corner section

//static double SMOOTHING = 0;   //The amount of smoothing to use when fitting 
                                //3rd order splines to the quintic Bezier
                                //functions
//static bool DEBUG = true;    //When this is set to true, each function's debug
                            //routine will be called, which usually results
                            //in a text file of its output being produced

//static double UTOL = (double)SimTK::Eps*1e2;

//static double INTTOL = (double)SimTK::Eps*1e4;

//static int MAXITER = 20;
//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================
double SmoothSegmentedFunctionFactory::scaleCurviness(double curviness)
{
    double c = 0.1 + 0.8*curviness;
    return c;
}
//=============================================================================
// MUSCLE CURVE FITTING FUNCTIONS
//=============================================================================
SmoothSegmentedFunction* SmoothSegmentedFunctionFactory::
    createFiberActiveForceLengthCurve(double x0, double x1, double x2, 
    double x3, double ylow,  double dydx, double curviness,
    bool computeIntegral, const std::string& curveName)
{
    //Ensure that the inputs are within a valid range
    double rootEPS = sqrt(SimTK::Eps);
    SimTK_ERRCHK1_ALWAYS( (x0>=0 && x1>x0+rootEPS  
                        && x2>x1+rootEPS && x3>x2+rootEPS),
        "SmoothSegmentedFunctionFactory::createFiberActiveForceLengthCurve",
        "%s: This must be true: 0 < lce0 < lce1 < lce2 < lce3",
        curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( ylow >= 0,
        "SmoothSegmentedFunctionFactory::createFiberActiveForceLengthCurve",
        "%s: shoulderVal must be greater than, or equal to 0",
        curveName.c_str());
    double dydxUpperBound = (1-ylow)/(x2-x1);
    SimTK_ERRCHK2_ALWAYS(dydx >= 0 && dydx < dydxUpperBound,
        "SmoothSegmentedFunctionFactory::createFiberActiveForceLengthCurve",
        "%s: plateauSlope must be greater than 0 and less than %f",
        curveName.c_str(),dydxUpperBound);
    SimTK_ERRCHK1_ALWAYS( (curviness >= 0 && curviness <= 1),
        "SmoothSegmentedFunctionFactory::createFiberActiveForceLengthCurve",
        "%s: curviness must be between 0 and 1",
        curveName.c_str());

    std::string name = curveName;
    name.append(".createFiberActiveForceLengthCurve");



    //Translate the users parameters into Bezier curves
    double c = scaleCurviness(curviness);

    //The active force length curve is made up of 5 elbow shaped sections. 
    //Compute the locations of the joining point of each elbow section.

    //Calculate the location of the shoulder
       double xDelta = 0.05*x2; //half the width of the sarcomere 0.0259, 
                               //but TM.Winter's data has a wider shoulder than
                               //this

       double xs    = (x2-xDelta);//x1 + 0.75*(x2-x1);
   
   //Calculate the intermediate points located on the ascending limb
       double y0    = 0;   
       double dydx0 = 0;

       double y1    = 1 - dydx*(xs-x1);
       double dydx01= 1.25*(y1-y0)/(x1-x0);//(y1-y0)/(x1-(x0+xDelta));

       double x01   = x0 + 0.5*(x1-x0); //x0 + xDelta + 0.5*(x1-(x0+xDelta));
       double y01   = y0 + 0.5*(y1-y0);
   
   //Calculate the intermediate points of the shallow ascending plateau
       double x1s   = x1 + 0.5*(xs-x1);
       double y1s   = y1 + 0.5*(1-y1);
       double dydx1s= dydx;
   
       //double dydx01c0 = 0.5*(y1s-y01)/(x1s-x01) + 0.5*(y01-y0)/(x01-x0);
       //double dydx01c1 = 2*( (y1-y0)/(x1-x0));
       //double dydx01(1-c)*dydx01c0 + c*dydx01c1; 
       
       //x2 entered
       double y2 = 1;
       double dydx2 = 0;
   
   //Descending limb
       //x3 entered
       double y3 = 0;
       double dydx3 = 0;
       
       double x23 = (x2+xDelta) + 0.5*(x3-(x2+xDelta)); //x2 + 0.5*(x3-x2);
       double y23 = y2 + 0.5*(y3-y2);
             
       //double dydx23c0 = 0.5*((y23-y2)/(x23-x2)) + 0.5*((y3-y23)/(x3-x23));
       //double dydx23c1 = 2*(y3-y2)/(x3-x2);
       double dydx23   = (y3-y2)/((x3-xDelta)-(x2+xDelta)); 
       //(1-c)*dydx23c0 + c*dydx23c1; 
    
    //Compute the locations of the control points
       SimTK::Matrix p0 = SegmentedQuinticBezierToolkit::
           calcQuinticBezierCornerControlPoints(x0,ylow,dydx0,x01,y01,dydx01,c);
       SimTK::Matrix p1 = SegmentedQuinticBezierToolkit::
          calcQuinticBezierCornerControlPoints(x01,y01,dydx01,x1s,y1s,dydx1s,c);
       SimTK::Matrix p2 = SegmentedQuinticBezierToolkit::
          calcQuinticBezierCornerControlPoints(x1s,y1s,dydx1s,x2, y2, dydx2,c);
       SimTK::Matrix p3 = SegmentedQuinticBezierToolkit::
           calcQuinticBezierCornerControlPoints(x2, y2, dydx2,x23,y23,dydx23,c);
       SimTK::Matrix p4 = SegmentedQuinticBezierToolkit::
           calcQuinticBezierCornerControlPoints(x23,y23,dydx23,x3,ylow,dydx3,c);
                                    
        SimTK::Matrix mX(6,5), mY(6,5);
        mX(0) = p0(0);
        mX(1) = p1(0);
        mX(2) = p2(0);
        mX(3) = p3(0);
        mX(4) = p4(0);

        mY(0) = p0(1);
        mY(1) = p1(1);
        mY(2) = p2(1);
        mY(3) = p3(1);
        mY(4) = p4(1);

        //std::string curveName = muscleName;
        //curveName.append("_fiberActiveForceLengthCurve");
        SmoothSegmentedFunction* mclCrvFcn = 
            new SmoothSegmentedFunction(
                mX,mY,x0,x3,ylow,ylow,0,0,computeIntegral,
            true, curveName);    
        return mclCrvFcn;
}

SmoothSegmentedFunction* SmoothSegmentedFunctionFactory::
    createFiberForceVelocityCurve(double fmaxE, 
    double dydxC, double dydxNearC, 
    double dydxIso, 
    double dydxE, double dydxNearE,
    double concCurviness,double eccCurviness,
    bool computeIntegral, const std::string& curveName)
{
    //Ensure that the inputs are within a valid range
    SimTK_ERRCHK1_ALWAYS( fmaxE > 1.0, 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityCurve",
        "%s: fmaxE must be greater than 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( (dydxC >= 0.0 && dydxC < 1), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityCurve",
        "%s: dydxC must be greater than or equal to 0"
        "and less than 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( (dydxNearC > dydxC && dydxNearC <= 1), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityCurve",
        "%s: dydxNearC must be greater than or equal to 0"
        "and less than 1",curveName.c_str());
    SimTK_ERRCHK2_ALWAYS( dydxIso > 1, 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityCurve",
        "%s: dydxIso must be greater than (fmaxE-1)/1 (%f)",curveName.c_str(),
                                                            ((fmaxE-1.0)/1.0));
    SimTK_ERRCHK2_ALWAYS( (dydxE >= 0.0 && dydxE < (fmaxE-1)), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityCurve",
        "%s: dydxE must be greater than or equal to 0"
        "and less than fmaxE-1 (%f)",curveName.c_str(),(fmaxE-1));
    SimTK_ERRCHK2_ALWAYS( (dydxNearE >= dydxE && dydxNearE < (fmaxE-1)), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityCurve",
        "%s: dydxNearE must be greater than or equal to dydxE"
        "and less than fmaxE-1 (%f)",curveName.c_str(),(fmaxE-1));
    SimTK_ERRCHK1_ALWAYS( (concCurviness <= 1.0 && concCurviness >= 0), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityCurve",
        "%s: concCurviness must be between 0 and 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( (eccCurviness <= 1.0 && eccCurviness >= 0), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityCurve",
        "%s: eccCurviness must be between 0 and 1",curveName.c_str());

    std::string name = curveName;
    name.append(".createFiberForceVelocityCurve");

    //Translate the users parameters into Bezier point locations
    double cC = scaleCurviness(concCurviness);
    double cE = scaleCurviness(eccCurviness);
    
    //Compute the concentric control point locations
    double xC   = -1;
    double yC   = 0;
    
    double xNearC = -0.9;
    double yNearC = yC + 0.5*dydxNearC*(xNearC-xC) + 0.5*dydxC*(xNearC-xC);

    double xIso = 0;
    double yIso = 1;

    double xE   = 1;
    double yE   = fmaxE;

    double xNearE = 0.9;
    double yNearE = yE + 0.5*dydxNearE*(xNearE-xE) + 0.5*dydxE*(xNearE-xE);


    SimTK::Matrix concPts1 = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(xC,yC,dydxC, 
                                            xNearC, yNearC,dydxNearC,cC);
    SimTK::Matrix concPts2 = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(xNearC,yNearC,dydxNearC, 
                                             xIso,  yIso,  dydxIso,  cC);
    SimTK::Matrix eccPts1 = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(xIso,      yIso,    dydxIso, 
                                             xNearE,  yNearE,  dydxNearE, cE);

    SimTK::Matrix eccPts2 = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(xNearE, yNearE, dydxNearE, 
                                                 xE,     yE,     dydxE, cE);

    SimTK::Matrix mX(6,4), mY(6,4);
    mX(0) = concPts1(0);
    mX(1) = concPts2(0);
    mX(2) = eccPts1(0);
    mX(3) = eccPts2(0);

    mY(0) = concPts1(1);
    mY(1) = concPts2(1);
    mY(2) = eccPts1(1);
    mY(3) = eccPts2(1);

    //std::string curveName = muscleName;
    //curveName.append("_fiberForceVelocityCurve");
    SmoothSegmentedFunction* mclCrvFcn = 
        new SmoothSegmentedFunction(mX,mY,xC,xE,yC,yE,dydxC,dydxE,
                                        computeIntegral, true, curveName);    
    return mclCrvFcn;
}


SmoothSegmentedFunction* SmoothSegmentedFunctionFactory::
    createFiberForceVelocityInverseCurve(double fmaxE, 
    double dydxC, double dydxNearC, 
    double dydxIso,
    double dydxE, double dydxNearE,
    double concCurviness, double eccCurviness,
    bool computeIntegral, const std::string& curveName)
{
    //Ensure that the inputs are within a valid range
    SimTK_ERRCHK1_ALWAYS( fmaxE > 1.0, 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: fmaxE must be greater than 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( (dydxC > SimTK::SignificantReal && dydxC < 1), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: dydxC must be greater than 0"
        "and less than 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( (dydxNearC > dydxC && dydxNearC < 1), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: dydxNearC must be greater than 0"
        "and less than 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( dydxIso > 1, 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: dydxIso must be greater than or equal to 1",curveName.c_str());
    SimTK_ERRCHK2_ALWAYS( (dydxE > SimTK::SignificantReal && dydxE < (fmaxE-1)), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: dydxE must be greater than or equal to 0"
        "and less than fmaxE-1 (%f)",curveName.c_str(),(fmaxE-1));
    SimTK_ERRCHK2_ALWAYS( (dydxNearE >= dydxE && dydxNearE < (fmaxE-1)), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: dydxNearE must be greater than or equal to dydxE"
        "and less than fmaxE-1 (%f)",curveName.c_str(),(fmaxE-1));
    SimTK_ERRCHK1_ALWAYS( (concCurviness <= 1.0 && concCurviness >= 0), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: concCurviness must be between 0 and 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( (eccCurviness <= 1.0 && eccCurviness >= 0), 
        "SmoothSegmentedFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: eccCurviness must be between 0 and 1",curveName.c_str());

    std::string name = curveName;
    name.append(".createFiberForceVelocityInverseCurve");

    //Translate the users parameters into Bezier point locations
    double cC = scaleCurviness(concCurviness);
    double cE = scaleCurviness(eccCurviness);
    
    //Compute the concentric control point locations
    double xC   = -1;
    double yC   = 0;
    
    double xNearC = -0.9;
    double yNearC = yC + 0.5*dydxNearC*(xNearC-xC) + 0.5*dydxC*(xNearC-xC);

    double xIso = 0;
    double yIso = 1;

    double xE   = 1;
    double yE   = fmaxE;

    double xNearE = 0.9;
    double yNearE = yE + 0.5*dydxNearE*(xNearE-xE) + 0.5*dydxE*(xNearE-xE);


    SimTK::Matrix concPts1 = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(xC,yC,dydxC, 
                                            xNearC, yNearC,dydxNearC,cC);
    SimTK::Matrix concPts2 = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(xNearC,yNearC,dydxNearC, 
                                             xIso,  yIso,  dydxIso,  cC);
    SimTK::Matrix eccPts1 = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(xIso,      yIso,    dydxIso, 
                                             xNearE,  yNearE,  dydxNearE, cE);

    SimTK::Matrix eccPts2 = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(xNearE, yNearE, dydxNearE, 
                                                 xE,     yE,     dydxE, cE);

    SimTK::Matrix mX(6,4), mY(6,4);
    mX(0) = concPts1(0);
    mX(1) = concPts2(0);
    mX(2) = eccPts1(0);
    mX(3) = eccPts2(0);

    mY(0) = concPts1(1);
    mY(1) = concPts2(1);
    mY(2) = eccPts1(1);
    mY(3) = eccPts2(1);
    
    SmoothSegmentedFunction* mclCrvFcn = new 
        SmoothSegmentedFunction(mY,mX,yC,yE,xC,xE,1/dydxC,1/dydxE,
            computeIntegral,true, curveName);    
    return mclCrvFcn;

}

SmoothSegmentedFunction* SmoothSegmentedFunctionFactory::
    createFiberCompressiveForcePennationCurve(double phi0, double k, 
         double curviness, bool computeIntegral, const std::string& curveName)
{
    //Check the input arguments
    SimTK_ERRCHK1_ALWAYS( (phi0>0 && phi0<(SimTK::Pi/2.0)) , 
        "SmoothSegmentedFunctionFactory::createFiberCompressiveForcePennationCurve", 
        "%s: phi0 must be greater than 0, and less than Pi/2",curveName.c_str());

    SimTK_ERRCHK2_ALWAYS( k > (1.0/(SimTK::Pi/2.0-phi0)) , 
        "SmoothSegmentedFunctionFactory::createFiberCompressiveForcePennationCurve", 
        "%s: k must be greater than %f",curveName.c_str(), 
        (1.0/(SimTK::Pi/2.0-phi0)));

    SimTK_ERRCHK1_ALWAYS( (curviness>=0 && curviness <= 1) , 
        "SmoothSegmentedFunctionFactory::createFiberCompressiveForcePennationCurve", 
        "%s: curviness must be between 0.0 and 1.0",curveName.c_str());

    std::string name=curveName;
    name.append(".createFiberCompressiveForcePennationCurve");

    //Translate the user parameters to quintic Bezier points
    double c = scaleCurviness(curviness);
    double x0 = phi0;
    double y0 = 0;
    double dydx0 = 0;
    double x1 = SimTK::Pi/2.0;
    double y1 = 1;
    double dydx1 = k;

    SimTK::Matrix ctrlPts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c);
    
    SimTK::Matrix mX(6,1), mY(6,1);
    mX = ctrlPts(0);
    mY = ctrlPts(1);

    //std::string curveName = muscleName;
    //curveName.append("_fiberCompressiveForcePennationCurve");
    SmoothSegmentedFunction* mclCrvFcn = 
        new SmoothSegmentedFunction(mX,mY,x0,x1,y0,y1,dydx0,dydx1,computeIntegral,
                                                                true,curveName);

    //If in debug, print the function
    return mclCrvFcn;

}

SmoothSegmentedFunction* SmoothSegmentedFunctionFactory::
    createFiberCompressiveForceCosPennationCurve(double cosPhi0, double k, 
         double curviness, bool computeIntegral, const std::string& curveName)
{
    //Check the input arguments
    SimTK_ERRCHK1_ALWAYS( (cosPhi0>0 && cosPhi0 < 1) , 
        "SmoothSegmentedFunctionFactory::createFiberCompressiveForceCosPennationCurve", 
        "%s: cosPhi0 must be greater than 0, and less than 1",curveName.c_str());

    SimTK_ERRCHK1_ALWAYS( k < 1/cosPhi0 , 
        "SmoothSegmentedFunctionFactory::createFiberCompressiveForceCosPennationCurve", 
        "%s: k must be less than 0",curveName.c_str());

    SimTK_ERRCHK1_ALWAYS( (curviness>=0 && curviness <= 1) , 
        "SmoothSegmentedFunctionFactory::createFiberCompressiveForceCosPennationCurve", 
        "%s: curviness must be between 0.0 and 1.0",curveName.c_str());

    std::string name=curveName;
    name.append(".createFiberCompressiveForceCosPennationCurve");

    //Translate the user parameters to quintic Bezier points
    double c = scaleCurviness(curviness);
    double x0 = 0;
    double y0 = 1;
    double dydx0 = k;
    double x1 = cosPhi0;
    double y1 = 0;
    double dydx1 = 0;

    SimTK::Matrix ctrlPts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c);
    
    SimTK::Matrix mX(6,1), mY(6,1);
    mX = ctrlPts(0);
    mY = ctrlPts(1);

    //std::string curveName = muscleName;
    //curveName.append("_fiberCompressiveForceCosPennationCurve");
    SmoothSegmentedFunction* mclCrvFcn = 
        new SmoothSegmentedFunction(mX,mY,x0,x1,y0,y1,dydx0,dydx1,computeIntegral,
                                                              false,curveName);

    //If in debug, print the function
    return mclCrvFcn;

}

SmoothSegmentedFunction* SmoothSegmentedFunctionFactory::
      createFiberCompressiveForceLengthCurve(double lmax, double k, 
               double curviness, bool computeIntegral, 
               const std::string& curveName)
{
    //Check the input arguments
    SimTK_ERRCHK1_ALWAYS( lmax>0 , 
        "SmoothSegmentedFunctionFactory::createFiberCompressiveForceLength", 
        "%s: l0 must be greater than 0",curveName.c_str());

    SimTK_ERRCHK2_ALWAYS( k < -(1.0/lmax) , 
        "SmoothSegmentedFunctionFactory::createFiberCompressiveForceLength", 
        "%s: k must be less than %f",curveName.c_str(),-(1.0/lmax));

    SimTK_ERRCHK1_ALWAYS( (curviness>=0 && curviness <= 1) , 
        "SmoothSegmentedFunctionFactory::createFiberCompressiveForceLength", 
        "%s: curviness must be between 0.0 and 1.0",curveName.c_str());

    std::string caller = curveName;
    caller.append(".createFiberCompressiveForceLength");

    //Translate the user parameters to quintic Bezier points
    double c = scaleCurviness(curviness);
    double x0 = 0.0;
    double y0 = 1;
    double dydx0 = k;
    double x1 = lmax;
    double y1 = 0;
    double dydx1 = 0;

    SimTK::Matrix ctrlPts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c);

    SimTK::Matrix mX(6,1), mY(6,1);
    mX(0) = ctrlPts(0);
    mY(0) = ctrlPts(1);

    // curveName = muscleName;
    //curveName.append("_fiberCompressiveForceLengthCurve");
    SmoothSegmentedFunction* mclCrvFcn = 
        new SmoothSegmentedFunction(mX,mY,x0,x1,y0,y1,dydx0,dydx1,computeIntegral,
                                                               false,curveName);

    return mclCrvFcn;

}


SmoothSegmentedFunction* SmoothSegmentedFunctionFactory::
    createFiberForceLengthCurve(double eZero, double eIso, 
                                double kLow, double kIso, double curviness,
                             bool computeIntegral, const std::string& curveName)
{
    
    //Check the input arguments
    SimTK_ERRCHK1_ALWAYS( eIso > eZero , 
        "SmoothSegmentedFunctionFactory::createFiberForceLength", 
        "%s: The following must hold: eIso  > eZero",curveName.c_str());

    SimTK_ERRCHK2_ALWAYS( kIso > (1.0/(eIso-eZero)) , 
       "SmoothSegmentedFunctionFactory::createFiberForceLength", 
       "%s: kiso must be greater than 1/(eIso-eZero) (%f)",
       curveName.c_str(),(1.0/(eIso-eZero)));

    SimTK_ERRCHK1_ALWAYS(kLow > 0.0 && kLow < 1/(eIso-eZero),
        "SmoothSegmentedFunctionFactory::createFiberForceLength", 
        "%s: kLow must be greater than 0 and less than or equal to 1",
        curveName.c_str());

    SimTK_ERRCHK1_ALWAYS( (curviness>=0 && curviness <= 1) , 
        "SmoothSegmentedFunctionFactory::createFiberForceLength", 
        "%s: curviness must be between 0.0 and 1.0",curveName.c_str());

    std::string callerName = curveName;
    callerName.append(".createFiberForceLength");

    //Translate the user parameters to quintic Bezier points
    /*
    double c = scaleCurviness(curviness);
    double x0 = 1.0 + e0;
    double y0 = 0;
    double dydx0 = 0;
    double x1 = 1.0 + e1;
    double y1 = 1;
    double dydx1 = kiso;

    SimTK::Matrix ctrlPts = SegmentedQuinticBezierToolkit::
        calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c,callerName);

    SimTK::Matrix mX(6,1), mY(6,1);
    mX(0) = ctrlPts(0);
    mY(0) = ctrlPts(1);
    */

        //Translate the user parameters to quintic Bezier points
    double c = scaleCurviness(curviness);
    double xZero = 1+eZero;
    double yZero = 0;
    
    double xIso = 1 + eIso;
    double yIso = 1;
    
    double deltaX = min(0.1*(1.0/kIso), 0.1*(xIso-xZero));

    double xLow     = xZero + deltaX;
    double xfoot    = xZero + 0.5*(xLow-xZero);
    double yfoot    = 0;
    double yLow     = yfoot + kLow*(xLow-xfoot);

    //Compute the Quintic Bezier control points
    SimTK::Matrix p0 = SegmentedQuinticBezierToolkit::
     calcQuinticBezierCornerControlPoints(xZero, yZero, 0,
                                           xLow, yLow,  kLow,c);
    
    SimTK::Matrix p1 = SegmentedQuinticBezierToolkit::
     calcQuinticBezierCornerControlPoints(xLow, yLow, kLow,
                                          xIso, yIso, kIso, c);
    SimTK::Matrix mX(6,2);
    SimTK::Matrix mY(6,2);

    mX(0) = p0(0);
    mY(0) = p0(1);

    mX(1) = p1(0);
    mY(1) = p1(1);
    

    //std::string curveName = muscleName;
    //curveName.append("_tendonForceLengthCurve");
    //Instantiate a muscle curve object
   SmoothSegmentedFunction* mclCrvFcn = 
    new SmoothSegmentedFunction(  mX,    mY,
                                       xZero,    xIso,
                                       yZero,    yIso,
                                         0.0,    kIso,
                                       computeIntegral,
                                       true,curveName);


    return mclCrvFcn;
}




SmoothSegmentedFunction* SmoothSegmentedFunctionFactory::
          createTendonForceLengthCurve( double eIso, double kIso, 
                                        double fToe, double curviness,
                                        bool computeIntegral, 
                                        const std::string& curveName)
{
    //Check the input arguments
    //eIso>0 
    SimTK_ERRCHK2_ALWAYS( eIso>0 , 
        "SmoothSegmentedFunctionFactory::createTendonForceLengthCurve", 
        "%s: eIso must be greater than 0, but %f was entered", 
        curveName.c_str(),eIso);

    SimTK_ERRCHK2_ALWAYS( (fToe>0 && fToe < 1) , 
        "SmoothSegmentedFunctionFactory::createTendonForceLengthCurve", 
        "%s: fToe must be greater than 0 and less than 1, but %f was entered", 
        curveName.c_str(),fToe);

    SimTK_ERRCHK3_ALWAYS( kIso > (1/eIso) , 
       "SmoothSegmentedFunctionFactory::createTendonForceLengthCurve", 
       "%s : kIso must be greater than 1/eIso, (%f), but kIso (%f) was entered", 
        curveName.c_str(), (1/eIso),kIso);

    SimTK_ERRCHK2_ALWAYS( (curviness>=0 && curviness <= 1) , 
        "SmoothSegmentedFunctionFactory::createTendonForceLengthCurve", 
        "%s : curviness must be between 0.0 and 1.0, but %f was entered"
        , curveName.c_str(),curviness);

    std::string callerName = curveName;
    callerName.append(".createTendonForceLengthCurve");

    //Translate the user parameters to quintic Bezier points
    double c = scaleCurviness(curviness);
    double x0 = 1.0;
    double y0 = 0;
    double dydx0 = 0;

    double xIso = 1.0 + eIso;
    double yIso = 1;
    double dydxIso = kIso;

    //Location where the curved section becomes linear
    double yToe = fToe;
    double xToe = (yToe-1)/kIso + xIso;


    //To limit the 2nd derivative of the toe region the line it tends to
    //has to intersect the x axis to the right of the origin
        double xFoot = 1.0+(xToe-1.0)/10.0;
        double yFoot = 0;
        //double dydxToe = (yToe-yFoot)/(xToe-xFoot);

    //Compute the location of the corner formed by the average slope of the
    //toe and the slope of the linear section
    double yToeMid = yToe*0.5;
    double xToeMid = (yToeMid-yIso)/kIso + xIso;
    double dydxToeMid = (yToeMid-yFoot)/(xToeMid-xFoot);

    //Compute the location of the control point to the left of the corner
    double xToeCtrl = xFoot + 0.5*(xToeMid-xFoot); 
    double yToeCtrl = yFoot + dydxToeMid*(xToeCtrl-xFoot);



    //Compute the Quintic Bezier control points
    SimTK::Matrix p0 = SegmentedQuinticBezierToolkit::
     calcQuinticBezierCornerControlPoints(x0,y0,dydx0,
                                        xToeCtrl,yToeCtrl,dydxToeMid,c);
    SimTK::Matrix p1 = SegmentedQuinticBezierToolkit::
     calcQuinticBezierCornerControlPoints(xToeCtrl, yToeCtrl, dydxToeMid,
                                              xToe,     yToe,    dydxIso, c);
    SimTK::Matrix mX(6,2);
    SimTK::Matrix mY(6,2);

    mX(0) = p0(0);
    mY(0) = p0(1);

    mX(1) = p1(0);
    mY(1) = p1(1);

    //std::string curveName = muscleName;
    //curveName.append("_tendonForceLengthCurve");
    //Instantiate a muscle curve object
   SmoothSegmentedFunction* mclCrvFcn = 
         new SmoothSegmentedFunction(  mX,    mY,
                                       x0,    xToe,
                                       y0,    yToe,
                                       dydx0, dydxIso,
                                       computeIntegral,
                                       true,curveName);

    return mclCrvFcn;
}


