// MuscleCurveFunction.cpp
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
#include "MuscleCurveFunction.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace SimTK;
using namespace OpenSim;
//using namespace OpenSim;
using namespace std;

static bool DEBUG=false;
static double UTOL = (double)SimTK::Eps*1e2;
static double INTTOL = (double)SimTK::Eps*1e2;
static int MAXITER = 20;
static int NUM_SAMPLE_PTS = 100;
//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================


MuscleCurveFunction::
  MuscleCurveFunction(const SimTK::Matrix& mX, const SimTK::Matrix& mY,  
          double x0, double x1, double y0, double y1,double dydx0, double dydx1,
          const bool computeIntegral, const bool intx0x1, const string name):
 _mX(mX),_mY(mY),_x0(x0),_x1(x1),_y0(y0),_y1(y1),_dydx0(dydx0),_dydx1(dydx1),
     _computeIntegral(computeIntegral),_intx0x1(intx0x1),_name(name)
{
    

    _numBezierSections = _mX.ncol();

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

    if(_computeIntegral){
        //////////////////////////////////////////////////
        //Compute the integral of y(x) and spline the result    
        //////////////////////////////////////////////////

        SimTK::Matrix yInt =  QuinticBezierCurveSet::
            calcNumIntBezierYfcnX(xALL,0,INTTOL, UTOL, MAXITER,_mX, _mY,
            _arraySplineUX,_intx0x1,_name);

        //not correct
        //if(_intx0x1==false){
        //    yInt = yInt*-1;
        //    yInt = yInt - yInt(yInt.nelt()-1);
        //}

        _splineYintX = SimTK::SplineFitter<Real>::
                fitForSmoothingParameter(3,yInt(0),yInt(1),0).getSpline();
    }
  
}

double MuscleCurveFunction::calcValue(double x) const
{
    return calcValue(SimTK::Vector(1,x));
}

double MuscleCurveFunction::calcDerivative(double x, int order) const
{
    return calcDerivative( SimTK::Array_<int>(order,0),
                           SimTK::Vector(1,x));
}


double MuscleCurveFunction::calcValue(const SimTK::Vector& ax) const
{
    
    SimTK_ERRCHK2_ALWAYS( ax.nelt() == 1,
        "MuscleCurveFunction::calcValue",
        "%s: Argument x must have only 1 element, as this function is "
        "designed only for 1D functions, but a function with %i elements was"
        "entered",_name.c_str(),ax.nelt());

    double yVal = 0;
    if(ax(0) >= _x0 && ax(0) <= _x1 )
    {
        int idx  = QuinticBezierCurveSet::calcIndex(ax(0),_mX,_name);
        double u = QuinticBezierCurveSet::
                 calcU(ax(0),_mX(idx), _arraySplineUX[idx], UTOL,MAXITER,_name);
        yVal = QuinticBezierCurveSet::
                 calcQuinticBezierCurveVal(u,_mY(idx),_name);
    }else{
        if(ax(0) < _x0){
            yVal = _y0 + _dydx0*(ax(0)-_x0);            
        }else{
            yVal = _y1 + _dydx1*(ax(0)-_x1);                    
        }    
    }

    return yVal;
}

double MuscleCurveFunction::
    calcDerivative(const SimTK::Array_<int>& derivComponents,
                 const SimTK::Vector& ax) const
{
    for(int i=0; i < (signed)derivComponents.size(); i++){
        SimTK_ERRCHK2_ALWAYS( derivComponents[i] == 0,
        "MuscleCurveFunction::calcDerivative",
        "%s: derivComponents can only be populated with 0's because "
        "MuscleCurveFunction is only valid for a 1D function, but "
        "derivComponents had a value of %i in it",
        _name.c_str(), derivComponents[i]);
    }
    SimTK_ERRCHK2_ALWAYS( derivComponents.size() <= 6,
        "MuscleCurveFunction::calcDerivative",
        "%s: calcDerivative is only valid up to a 6th order derivative"
        " but derivComponents had a size of %i",
        _name.c_str(), derivComponents.size());

    SimTK_ERRCHK2_ALWAYS( ax.nelt() == 1,
        "MuscleCurveFunction::calcValue",
        "%s: Argument x must have only 1 element, as this function is "
        "designed only for 1D functions, but ax had a size of %i",
        _name.c_str(), ax.nelt());


    double yVal = 0;

    //QUINTIC SPLINE

    
    if(derivComponents.size()==0){
                yVal = calcValue(ax);
    }else{
    
            if(ax(0) >= _x0 && ax(0) <= _x1){        
                int idx  = QuinticBezierCurveSet::
                                calcIndex(ax(0),_mX,_name);
                double u = QuinticBezierCurveSet::
                                calcU(ax(0),_mX(idx), _arraySplineUX[idx], 
                                UTOL,MAXITER,_name);
                yVal = QuinticBezierCurveSet::
                            calcQuinticBezierCurveDerivDYDX(u, _mX(idx), 
                            _mY(idx), derivComponents.size(),_name);
            }else{
                    if(derivComponents.size() == 1){
                        if(ax(0) < _x0){
                            yVal = _dydx0;
                        }else{
                            yVal = _dydx1;}
                    }else{
                        yVal = 0;}   
                }
        }

    return yVal;
}

bool MuscleCurveFunction::isIntegralAvailable() const
{
    return _computeIntegral;
}

bool MuscleCurveFunction::isIntegralComputedLeftToRight() const
{
    return _intx0x1;
}

double MuscleCurveFunction::calcIntegral(double x) const
{
    SimTK_ERRCHK1_ALWAYS(_computeIntegral,
        "MuscleCurveFunction::calcIntegral",
        "%s: This curve was not constructed with its integral because"
        "computeIntegral was false",_name.c_str());


    double yVal = 0;    
    if(x >= _x0 && x <= _x1){
        yVal = _splineYintX.calcValue(SimTK::Vector(1,x));
    }else{
        //LINEAR EXTRAPOLATION         
        if(x < _x0){
            SimTK::Vector tmp(1);
            tmp(0) = _x0;
            double ic = _splineYintX.calcValue(tmp);
            if(_intx0x1){//Integrating left to right
                yVal = _y0*(x-_x0) 
                    + _dydx0*(x-_x0)*(x-_x0)*0.5 
                    + ic;
            }else{//Integrating right to left
                yVal = -_y0*(x-_x0) 
                    - _dydx0*(x-_x0)*(x-_x0)*0.5 
                    + ic;
            }            
        }else{
            SimTK::Vector tmp(1);
            tmp(0) = _x1;
            double ic = _splineYintX.calcValue(tmp);
            if(_intx0x1){
                yVal = _y1*(x-_x1) 
                    + _dydx1*(x-_x1)*(x-_x1)*0.5 
                    + ic;
            }else{
                yVal = -_y1*(x-_x1) 
                    - _dydx1*(x-_x1)*(x-_x1)*0.5 
                    + ic;
            }
        }
    } 
    
    return yVal;
}

int MuscleCurveFunction::getArgumentSize() const
{
    return 1;
}

int MuscleCurveFunction::getMaxDerivativeOrder() const
{
    return 6;
}

string MuscleCurveFunction::getName() const
{
    return _name;
}

SimTK::Vec2 MuscleCurveFunction::getCurveDomain() const
{
    SimTK::Vec2 xrange;
    xrange(0) = _mX(0,0);
    xrange(1) = _mX( _mX.nrow()-1, _mX.ncol()-1);
    return xrange;
}

///////////////////////////////////////////////////////////////////////////////
// Utility functions
///////////////////////////////////////////////////////////////////////////////
SimTK::Matrix MuscleCurveFunction::calcSampledMuscleCurve() const{
    int pts = 1; //Number of points between each of the spline points used
                  //to fit u(x), and also the integral spline

    double x0,x1,delta;
    //y,dy,d1y,d2y,d3y,d4y,d5y,d6y,iy
   SimTK::Vector midX(NUM_SAMPLE_PTS*_numBezierSections-(_numBezierSections-1));
   SimTK::Vector x(NUM_SAMPLE_PTS);

   //Generate a sample of X values inside of the curve that is denser where 
   //the curve is more curvy.
   double u;
   int idx = 0;
      for(int s=0; s < _numBezierSections; s++){
        //Sample the local set for u and x
        for(int i=0;i<NUM_SAMPLE_PTS;i++){
                u = ( (double)i )/( (double)(NUM_SAMPLE_PTS-1) );
                x(i) = QuinticBezierCurveSet::
                    calcQuinticBezierCurveVal(u,_mX(s),_name);            
                if(_numBezierSections > 1){
                   //Skip the last point of a set that has another set of points
                   //after it. Why? The last point and the starting point of the
                   //next set are identical in value.
                    if(i<(NUM_SAMPLE_PTS-1) || s == (_numBezierSections-1)){
                        midX(idx) = x(i);
                        idx++;
                    }
                }else{
                    midX(idx) = x(i);                
                    idx++;
                }
            }
        }         

        
    SimTK::Vector xsmpl(pts*(midX.nelt()-1)+2*10*pts);
    
    SimTK::Matrix results;

    if(_computeIntegral){
        results.resize(pts*(midX.nelt()-1)+2*10*pts,9);
    }else{
        results.resize(pts*(midX.nelt()-1)+2*10*pts,8);
    }
    //Array initialization is so ugly ...
    SimTK::Array_<int> d1y(1),d2y(2),d3y(3),d4y(4),d5y(5),d6y(6);
    d1y[0]=0;
    d2y[0] = 0;
    d2y[1] = 0;
    for(int i=0;i<3;i++)
        d3y[i]=0;
    for(int i=0;i<4;i++)
        d4y[i]=0;
    for(int i=0;i<5;i++)
        d5y[i]=0;
    for(int i=0;i<6;i++)
        d6y[i]=0;

    //generate some sample points in the extrapolated region
    idx=0;
    x0 = _x0 - 0.1*(_x1-_x0);
    x1 = _x0;
    delta = 0.1*(x1-x0)/pts;

    for(int j=0;j<pts*10;j++){
        xsmpl(idx) = x0 + delta*j;
        idx++;
    }


    //generate some points in the mid region
    for(int i=0; i< midX.nelt()-1;i++){  
        x0 = midX(i);
        x1 = midX(i+1);
        delta = (x1-x0)/pts;
        for(int j=0;j<pts;j++){
            xsmpl(idx) = x0 + delta*j;
            idx++;   
        }        
    }

    //generate some points in the extrapolated region
    x0 = _x1;
    x1 = _x1 + 0.1*(_x1-_x0);
    delta = 0.1*(x1-x0)/pts;

    for(int j=0;j<pts*10;j++){
        xsmpl(idx) = x0 + delta*j;
        idx++;
    }

    //Populate the results matrix at the sample points
    SimTK::Vector ax(1);
    for(int i=0; i < xsmpl.nelt(); i++){
        ax(0) = xsmpl(i);
        results(i,0) = ax(0);
        results(i,1) = calcValue(ax);
        results(i,2) = calcDerivative(d1y,ax);
        results(i,3) = calcDerivative(d2y,ax);
        results(i,4) = calcDerivative(d3y,ax);
        results(i,5) = calcDerivative(d4y,ax);
        results(i,6) = calcDerivative(d5y,ax);
        results(i,7) = calcDerivative(d6y,ax);

        if(_computeIntegral){
            results(i,8) = calcIntegral(ax(0));
        }
    }
   return results;
}

void MuscleCurveFunction::printMuscleCurveToFile(string path) const
{
    SimTK::Matrix results = calcSampledMuscleCurve();
    SimTK::Array_<string> colNames(results.ncol());
    colNames[0] = "x";
    colNames[1] = "y";
    colNames[2] = "dy/dx";
    colNames[3] = "d2y/dx2";
    colNames[4] = "d3y/dx3";
    colNames[5] = "d4y/dx4";
    colNames[6] = "d5y/dx5";
    colNames[7] = "d6y/dx6";

    if(results.ncol() == 9){
        colNames[8] = "int_y(x)";
    }

            string fname = _name;
            fname.append(".csv");
            printMatrixToFile(results,colNames,path,fname);
}
/**
This function will print cvs file of the column vector col0 and the matrix data

@params data: A matrix of data
@params filename: The name of the file to print
*/
void MuscleCurveFunction::
    printMatrixToFile(SimTK::Matrix& data, SimTK::Array_<string> colNames,
    string path, string filename)
{
	
    ofstream datafile;
    string fullpath = path;
    fullpath.append("/");
    fullpath.append(filename);
	datafile.open(fullpath.c_str());

    for(int i = 0; i < (signed)colNames.size(); i++){
        if(i < (signed)colNames.size()-1)
            datafile << colNames[i] << ",";
        else
            datafile << colNames[i] << "\n";
    }

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