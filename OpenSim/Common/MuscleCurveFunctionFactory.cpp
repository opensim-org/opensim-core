// MuscleCurveFunctionFactory.cpp
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
//=============================================================================
// INCLUDES
//=============================================================================

#include "MuscleCurveFunctionFactory.h"
//=============================================================================
// STATICS
//=============================================================================
using namespace SimTK;
using namespace OpenSim;
using namespace std;


static int NUM_SAMPLE_PTS = 100; //The number of knot points to use to sample
                                //each Bezier corner section

static double SMOOTHING = 0;   //The amount of smoothing to use when fitting 
                                //3rd order splines to the quintic Bezier
                                //functions
static bool DEBUG = true;    //When this is set to true, each function's debug
                            //routine will be called, which ususally results
                            //in a text file of its output being produced

static double UTOL = (double)SimTK::Eps*1e2;

static double INTTOL = (double)SimTK::Eps*1e4;

static int MAXITER = 20;
//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================
double MuscleCurveFunctionFactory::scaleCurviness(double curviness)
{
    double c = 0.1 + 0.8*curviness;
    return c;
}
//=============================================================================
// MUSCLE CURVE FITTING FUNCTIONS
//=============================================================================
MuscleCurveFunction MuscleCurveFunctionFactory::
    createFiberActiveForceLengthCurve(double x0, double x1, double x2, 
    double x3, double ylow,  double dydx, double curviness,
    bool computeIntegral, const std::string& curveName)
{
    //Ensure that the inputs are within a valid range
    double rootEPS = sqrt(SimTK::Eps);
    SimTK_ERRCHK1_ALWAYS( (x0>0 && x1>x0+rootEPS  
                        && x2>x1+rootEPS && x3>x2+rootEPS),
        "MuscleCurveFunctionFactory::createFiberActiveForceLengthCurve",
        "%s: This must be true: 0 < lce0 < lce1 < lce2 < lce3",
        curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( ylow >= 0,
        "MuscleCurveFunctionFactory::createFiberActiveForceLengthCurve",
        "%s: shoulderVal must be greater than, or equal to 0",
        curveName.c_str());
    SimTK_ERRCHK1_ALWAYS(dydx >= 0 && dydx < (1/(x2-x1)),
        "MuscleCurveFunctionFactory::createFiberActiveForceLengthCurve",
        "%s: plateauSlope must be greater than 0",
        curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( (curviness >= 0 && curviness <= 1),
        "MuscleCurveFunctionFactory::createFiberActiveForceLengthCurve",
        "%s: curviness must be between 0 and 1",
        curveName.c_str());

    std::string name = curveName;
    name.append(".createFiberActiveForceLengthCurve");



    //Translate the users parameters into Bezier curves
    double c = scaleCurviness(curviness);

    //The active force length curve is made up of 5 elbow shaped sections. 
    //Compute the locations of the joining point of each elbow section.

    //Calculate the location of the shoulder
       double xs    = x1 + 0.75*(x2-x1);
   
   //Calculate the intermediate points located on the ascending limb
       double y0    = ylow;   
       double dydx0 = 0;
       double y1    = 1 - dydx*(xs-x1);

       double x01   = x0 + 0.5*(x1-x0);
       double y01   = y0 + 0.5*(y1-y0);
   
   //Calculate the intermediate points of the plateau
       double x1s   = x1 + 0.5*(xs-x1);
       double y1s   = y1 + 0.5*(1-y1);
       double dydx1s= dydx;
   
       double dydx01c0 = 0.5*(y1s-y01)/(x1s-x01) + 0.5*(y01-y0)/(x01-x0);
       double dydx01c1 = 2*( (y1-y0)/(x1-x0));
       double dydx01   = (1-c)*dydx01c0 + c*dydx01c1; 
       
       //x2 entered
       double y2 = 1;
       double dydx2 = 0;
   
   //Descending limb
       //x3 entered
       double y3 = ylow;
       double dydx3 = 0;
       
       double x23 = x2 + 0.5*(x3-x2);
       double y23 = y2 + 0.5*(y3-y2);
             
       double dydx23c0 = 0.5*((y23-y2)/(x23-x2)) + 0.5*((y3-y23)/(x3-x23));
       double dydx23c1 = 2*(y3-y2)/(x3-x2);
       double dydx23   = (1-c)*dydx23c0 + c*dydx23c1; 
    
    //Compute the locations of the control points
       SimTK::Matrix p0 = QuinticBezierCurveSet::
           calcQuinticBezierCornerControlPoints(x0, y0, dydx0,x01,y01,dydx01,c
                                                                    ,name);
       SimTK::Matrix p1 = QuinticBezierCurveSet::
          calcQuinticBezierCornerControlPoints(x01,y01,dydx01,x1s,y1s,dydx1s,c
                                                                    ,name);
       SimTK::Matrix p2 = QuinticBezierCurveSet::
          calcQuinticBezierCornerControlPoints(x1s,y1s,dydx1s,x2, y2, dydx2,c
                                                                    ,name);
       SimTK::Matrix p3 = QuinticBezierCurveSet::
           calcQuinticBezierCornerControlPoints(x2, y2, dydx2,x23,y23,dydx23,c
                                                                    ,name);
       SimTK::Matrix p4 = QuinticBezierCurveSet::
           calcQuinticBezierCornerControlPoints(x23,y23,dydx23,x3, y3, dydx3,c
                                                                     ,name);
                                    
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
        MuscleCurveFunction mclCrvFcn(mX,mY,x0,x3,ylow,ylow,0,0,computeIntegral,
            true, curveName);    
        return mclCrvFcn;
}

MuscleCurveFunction MuscleCurveFunctionFactory::
    createFiberForceVelocityCurve(double fmaxE, double dydxC, double dydxIso, 
    double dydxE, double concCurviness,double eccCurviness,
    bool computeIntegral, const std::string& curveName)
{
    //Ensure that the inputs are within a valid range
    SimTK_ERRCHK1_ALWAYS( fmaxE > 1.0, 
        "MuscleCurveFunctionFactory::createFiberForceVelocityCurve",
        "%s: fmaxE must be greater than 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( (dydxC >= 0.0 && dydxC < 1), 
        "MuscleCurveFunctionFactory::createFiberForceVelocityCurve",
        "%s: dydxC must be greater than or equal to 0"
        "and less than 1",curveName.c_str());
    SimTK_ERRCHK2_ALWAYS( dydxIso > 1, 
        "MuscleCurveFunctionFactory::createFiberForceVelocityCurve",
        "%s: dydxIso must be greater than (fmaxE-1)/1 (%f)",curveName.c_str(),
                                                            ((fmaxE-1.0)/1.0));
    SimTK_ERRCHK2_ALWAYS( (dydxE >= 0.0 && dydxE < (fmaxE-1)), 
        "MuscleCurveFunctionFactory::createFiberForceVelocityCurve",
        "%s: dydxE must be greater than or equal to 0"
        "and less than fmaxE-1 (%f)",curveName.c_str(),(fmaxE-1));
    SimTK_ERRCHK1_ALWAYS( (concCurviness <= 1.0 && concCurviness >= 0), 
        "MuscleCurveFunctionFactory::createFiberForceVelocityCurve",
        "%s: concCurviness must be between 0 and 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( (eccCurviness <= 1.0 && eccCurviness >= 0), 
        "MuscleCurveFunctionFactory::createFiberForceVelocityCurve",
        "%s: eccCurviness must be between 0 and 1",curveName.c_str());

    std::string name = curveName;
    name.append(".createFiberForceVelocityCurve");

    //Translate the users parameters into Bezier point locations
    double cC = scaleCurviness(concCurviness);
    double cE = scaleCurviness(eccCurviness);
    
    //Compute the concentric control point locations
    double xC   = -1;
    double yC   = 0;
    double xIso = 0;
    double yIso = 1;
    double xE   = 1;
    double yE   = fmaxE;

    SimTK::Matrix concPts = QuinticBezierCurveSet::
        calcQuinticBezierCornerControlPoints(xC,yC,dydxC, xIso,yIso,dydxIso,cC,
        name);
    SimTK::Matrix eccPts = QuinticBezierCurveSet::
        calcQuinticBezierCornerControlPoints(xIso,yIso,dydxIso, xE,yE,dydxE,cE,
        name);

    SimTK::Matrix mX(6,2), mY(6,2);
    mX(0) = concPts(0);
    mX(1) = eccPts(0);
    mY(0) = concPts(1);
    mY(1) = eccPts(1);

    //std::string curveName = muscleName;
    //curveName.append("_fiberForceVelocityCurve");
    MuscleCurveFunction mclCrvFcn(mX,mY,xC,xE,yC,yE,dydxC,dydxE,computeIntegral,
                                                               true, curveName);    
    return mclCrvFcn;
}


MuscleCurveFunction MuscleCurveFunctionFactory::
    createFiberForceVelocityInverseCurve(double fmaxE, double dydxC, 
    double dydxIso, double dydxE, double concCurviness, double eccCurviness,
    bool computeIntegral, const std::string& curveName)
{
    //Ensure that the inputs are within a valid range
    SimTK_ERRCHK1_ALWAYS( fmaxE > 1.0, 
        "MuscleCurveFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: fmaxE must be greater than 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( (dydxC > 0.0 && dydxC < 1), 
        "MuscleCurveFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: dydxC must be greater than 0"
        "and less than 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( dydxIso > 1, 
        "MuscleCurveFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: dydxIso must be greater than or equal to 1",curveName.c_str());
    SimTK_ERRCHK2_ALWAYS( (dydxE > 0.0 && dydxE < (fmaxE-1)), 
        "MuscleCurveFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: dydxE must be greater than or equal to 0"
        "and less than fmaxE-1 (%f)",curveName.c_str(),(fmaxE-1));
    SimTK_ERRCHK1_ALWAYS( (concCurviness <= 1.0 && concCurviness >= 0), 
        "MuscleCurveFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: concCurviness must be between 0 and 1",curveName.c_str());
    SimTK_ERRCHK1_ALWAYS( (eccCurviness <= 1.0 && eccCurviness >= 0), 
        "MuscleCurveFunctionFactory::createFiberForceVelocityInverseCurve",
        "%s: eccCurviness must be between 0 and 1",curveName.c_str());

    std::string name = curveName;
    name.append(".createFiberForceVelocityInverseCurve");

    //Translate the users parameters into Bezier point locations
    double cC = scaleCurviness(concCurviness);
    double cE = scaleCurviness(eccCurviness);
    
    //Compute the concentric control point locations
    double xC   = -1;
    double yC   = 0;
    double xIso = 0;
    double yIso = 1;
    double xE   = 1;
    double yE   = fmaxE;

        SimTK::Matrix concPts = QuinticBezierCurveSet::
    calcQuinticBezierCornerControlPoints(xC,yC,dydxC,xIso,yIso,dydxIso,cC,name);
        SimTK::Matrix eccPts = QuinticBezierCurveSet::
    calcQuinticBezierCornerControlPoints(xIso,yIso,dydxIso,xE,yE,dydxE,cE,name);
    //Sample the curve. There are NUM_SAMPLE_PTS*2 -1 to remove the 1 overlapping
    //point of both curves.
    SimTK::Matrix mX(6,2), mY(6,2);
    mX(0) = concPts(0);
    mX(1) = eccPts(0);
    mY(0) = concPts(1);
    mY(1) = eccPts(1);

    //std::string curveName = muscleName;
    //curveName.append("_fiberForceVelocityInverseCurve");
    MuscleCurveFunction mclCrvFcn(mY,mX,yC,yE,xC,xE,1/dydxC,1/dydxE,
        computeIntegral,true, curveName);    
    return mclCrvFcn;

}

MuscleCurveFunction MuscleCurveFunctionFactory::
    createFiberCompressiveForcePennationCurve(double phi0, double k, 
         double curviness, bool computeIntegral, const std::string& curveName)
{
    //Check the input arguments
    SimTK_ERRCHK1_ALWAYS( (phi0>0 && phi0<(SimTK::Pi/2.0)) , 
        "MuscleCurveFunctionFactory::createFiberCompressiveForcePennationCurve", 
        "%s: phi0 must be greater than 0, and less than Pi/2",curveName.c_str());

    SimTK_ERRCHK2_ALWAYS( k > (1.0/(SimTK::Pi/2.0-phi0)) , 
        "MuscleCurveFunctionFactory::createFiberCompressiveForcePennationCurve", 
        "%s: k must be greater than %f",curveName.c_str(), 
        (1.0/(SimTK::Pi/2.0-phi0)));

    SimTK_ERRCHK1_ALWAYS( (curviness>=0 && curviness <= 1) , 
        "MuscleCurveFunctionFactory::createFiberCompressiveForcePennationCurve", 
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

    SimTK::Matrix ctrlPts = QuinticBezierCurveSet::
        calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c,name);
    
    SimTK::Matrix mX(6,1), mY(6,1);
    mX = ctrlPts(0);
    mY = ctrlPts(1);

    //std::string curveName = muscleName;
    //curveName.append("_fiberCompressiveForcePennationCurve");
    MuscleCurveFunction mclCrvFcn(mX,mY,x0,x1,y0,y1,dydx0,dydx1,computeIntegral,
                                                                true,curveName);

    //If in debug, print the function
    return mclCrvFcn;

}

MuscleCurveFunction MuscleCurveFunctionFactory::
    createFiberCompressiveForceCosPennationCurve(double cosPhi0, double k, 
         double curviness, bool computeIntegral, const std::string& curveName)
{
    //Check the input arguments
    SimTK_ERRCHK1_ALWAYS( (cosPhi0>0 && cosPhi0 < 1) , 
        "MuscleCurveFunctionFactory::createFiberCompressiveForceCosPennationCurve", 
        "%s: cosPhi0 must be greater than 0, and less than 1",curveName.c_str());

    SimTK_ERRCHK1_ALWAYS( k < 1/cosPhi0 , 
        "MuscleCurveFunctionFactory::createFiberCompressiveForceCosPennationCurve", 
        "%s: k must be less than 0",curveName.c_str());

    SimTK_ERRCHK1_ALWAYS( (curviness>=0 && curviness <= 1) , 
        "MuscleCurveFunctionFactory::createFiberCompressiveForceCosPennationCurve", 
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

    SimTK::Matrix ctrlPts = QuinticBezierCurveSet::
        calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c,name);
    
    SimTK::Matrix mX(6,1), mY(6,1);
    mX = ctrlPts(0);
    mY = ctrlPts(1);

    //std::string curveName = muscleName;
    //curveName.append("_fiberCompressiveForceCosPennationCurve");
    MuscleCurveFunction mclCrvFcn(mX,mY,x0,x1,y0,y1,dydx0,dydx1,computeIntegral,
                                                              false,curveName);

    //If in debug, print the function
    return mclCrvFcn;

}

MuscleCurveFunction MuscleCurveFunctionFactory::
      createFiberCompressiveForceLengthCurve(double lmax, double k, 
               double curviness, bool computeIntegral, 
               const std::string& curveName)
{
    //Check the input arguments
    SimTK_ERRCHK1_ALWAYS( lmax>0 , 
        "MuscleCurveFunctionFactory::createFiberCompressiveForceLength", 
        "%s: l0 must be greater than 0",curveName.c_str());

    SimTK_ERRCHK2_ALWAYS( k < -(1.0/lmax) , 
        "MuscleCurveFunctionFactory::createFiberCompressiveForceLength", 
        "%s: k must be less than %f",curveName.c_str(),-(1.0/lmax));

    SimTK_ERRCHK1_ALWAYS( (curviness>=0 && curviness <= 1) , 
        "MuscleCurveFunctionFactory::createFiberCompressiveForceLength", 
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

    SimTK::Matrix ctrlPts = QuinticBezierCurveSet::
        calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c,caller);

    SimTK::Matrix mX(6,1), mY(6,1);
    mX(0) = ctrlPts(0);
    mY(0) = ctrlPts(1);

    // curveName = muscleName;
    //curveName.append("_fiberCompressiveForceLengthCurve");
    MuscleCurveFunction mclCrvFcn(mX,mY,x0,x1,y0,y1,dydx0,dydx1,computeIntegral,
                                                               false,curveName);

    return mclCrvFcn;

}


MuscleCurveFunction MuscleCurveFunctionFactory::
          createFiberForceLengthCurve(double e0, double kiso, double curviness,
          bool computeIntegral, const std::string& curveName)
{
    
    //Check the input arguments
    SimTK_ERRCHK1_ALWAYS( e0>0 , 
        "MuscleCurveFunctionFactory::createFiberForceLength", 
        "%s: e0 must be greater than 0",curveName.c_str());

    SimTK_ERRCHK2_ALWAYS( kiso > (1/e0) , 
        "MuscleCurveFunctionFactory::createFiberForceLength", 
        "%s: kiso must be greater than 1/e0 (%f)",curveName.c_str(),(1/e0));

    SimTK_ERRCHK1_ALWAYS( (curviness>=0 && curviness <= 1) , 
        "MuscleCurveFunctionFactory::createFiberForceLength", 
        "%s: curviness must be between 0.0 and 1.0",curveName.c_str());

    std::string caller = curveName;
    caller.append(".createFiberForceLength");

    //Translate the user parameters to quintic Bezier points
    double c = scaleCurviness(curviness);
    double x0 = 1.0;
    double y0 = 0;
    double dydx0 = 0;
    double x1 = 1.0 + e0;
    double y1 = 1;
    double dydx1 = kiso;

    SimTK::Matrix ctrlPts = QuinticBezierCurveSet::
        calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c,caller);

    SimTK::Matrix mX(6,1), mY(6,1);
    mX(0) = ctrlPts(0);
    mY(0) = ctrlPts(1);

    //std::string curveName = muscleName;
    //curveName.append("_fiberForceLengthCurve");
        //Instantiate a muscle curve object
    MuscleCurveFunction mclCrvFcn(mX,mY,x0,x1,y0,y1,dydx0,dydx1,computeIntegral,
                                                                true,curveName);

    return mclCrvFcn;
}




MuscleCurveFunction MuscleCurveFunctionFactory::
          createTendonForceLengthCurve(double e0, double kiso, double curviness,
                                    bool computeIntegral, 
                                    const std::string& curveName)
{
    //Check the input arguments
    //e0>0 
    SimTK_ERRCHK1_ALWAYS( e0>0 , 
        "MuscleCurveFunctionFactory::createTendonForceLengthCurve", 
        "%s: e0 must be greater than 0, but %f was entered", curveName.c_str());

    SimTK_ERRCHK2_ALWAYS( kiso > (1/e0) , 
        "MuscleCurveFunctionFactory::createTendonForceLengthCurve", 
        "%s : kiso must be greater than 1/e0, (%f)", 
        curveName.c_str(), (1/e0));

    SimTK_ERRCHK1_ALWAYS( (curviness>=0 && curviness <= 1) , 
        "MuscleCurveFunctionFactory::createTendonForceLengthCurve", 
        "%s : curviness must be between 0.0 and 1.0, but %f was entered"
        , curveName.c_str());

    std::string callerName = curveName;
    callerName.append(".createTendonForceLengthCurve");

    //Translate the user parameters to quintic Bezier points
    double c = scaleCurviness(curviness);
    double x0 = 1.0;
    double y0 = 0;
    double dydx0 = 0;
    double x1 = 1.0 + e0;
    double y1 = 1;
    double dydx1 = kiso;

    //Compute the Quintic Bezier control points
    SimTK::Matrix ctrlPts = QuinticBezierCurveSet::
     calcQuinticBezierCornerControlPoints(x0,y0,dydx0,x1,y1,dydx1,c,callerName);
    SimTK::Matrix mX(6,1);
    SimTK::Matrix mY(6,1);

    mX(0) = ctrlPts(0);
    mY(0) = ctrlPts(1);

    //std::string curveName = muscleName;
    //curveName.append("_tendonForceLengthCurve");
    //Instantiate a muscle curve object
   MuscleCurveFunction mclCrvFcn(mX,mY,x0,x1,y0,y1,dydx0,dydx1,computeIntegral,
                                                                true,curveName);

    return mclCrvFcn;
}


