/* -------------------------------------------------------------------------- *
 *                   OpenSim:  SmoothSegmentedFunction.cpp                    *
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
#include "SmoothSegmentedFunction.h"
#include <fstream>
#include "simmath/internal/SplineFitter.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace SimTK;
using namespace OpenSim;
using namespace std;

//static bool DEBUG=false;
static double UTOL = (double)SimTK::Eps*1e2;
static double INTTOL = (double)SimTK::Eps*1e2;
static int MAXITER = 20;
static int NUM_SAMPLE_PTS = 100;
//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================
/*
 DETAILED COMPUTATIONAL COSTS:
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

            m*100       SegmentedQuinticBezierToolkit::
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
                                   > 40,000 comparisons 
                                   > 3000 divisions

       =========================================================================
        M: Matrix
        V: Vector

        N.B. These costs are dependent on SegmentedQuinticBezierToolkit
*/
SmoothSegmentedFunction::
  SmoothSegmentedFunction(const SimTK::Matrix& mX, const SimTK::Matrix& mY,  
          double x0, double x1, double y0, double y1,double dydx0, double dydx1,
          bool computeIntegral, bool intx0x1, const std::string& name):
_x0(x0),_x1(x1),_y0(y0),_y1(y1),_dydx0(dydx0),_dydx1(dydx1),
     _computeIntegral(computeIntegral),_intx0x1(intx0x1),_name(name)
{
    

    _numBezierSections = mX.ncol();

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
            x(i) = SegmentedQuinticBezierToolkit::
                calcQuinticBezierCurveVal(u(i),mX(s));            
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

        SimTK::Matrix yInt =  SegmentedQuinticBezierToolkit::
            calcNumIntBezierYfcnX(xALL,0,INTTOL, UTOL, MAXITER,mX, mY,
            _arraySplineUX,_intx0x1,_name);

        //not correct
        //if(_intx0x1==false){
        //    yInt = yInt*-1;
        //    yInt = yInt - yInt(yInt.nelt()-1);
        //}

        _splineYintX = SimTK::SplineFitter<Real>::
                fitForSmoothingParameter(3,yInt(0),yInt(1),0).getSpline();
    }
    
    _mXVec.resize(_numBezierSections);
    _mYVec.resize(_numBezierSections);
    for(int s=0; s < _numBezierSections; s++){
        _mXVec[s] = mX(s); 
        _mYVec[s] = mY(s); 
    }
}

 SmoothSegmentedFunction::SmoothSegmentedFunction():
 _x0(SimTK::NaN),_x1(SimTK::NaN),_y0(SimTK::NaN)
     ,_y1(SimTK::NaN),_dydx0(SimTK::NaN),_dydx1(SimTK::NaN),
     _computeIntegral(false),_intx0x1(false),_name("NOT_YET_SET")
 {
        _arraySplineUX.resize(0);        
        _mXVec.resize(0);
        _mYVec.resize(0);
        _splineYintX = SimTK::Spline();
        _numBezierSections = (int)SimTK::NaN;
       
 }

 /*Detailed Computational Costs
 ________________________________________________________________________
    If x is in the Bezier Curve
                            Name     Comp.   Div.    Mult.   Add.    Assign.
_______________________________________________________________________
        SegmentedQuinticBezierToolkit::
                        calcIndex     3*m+2                   1*m     3   
                            *calcU     15      2      82      42      60  
        calcQuinticBezierCurveVal                     21      20      13
                            total  15+3*m+2    2      103     62+1*m  76

        *Approximate. Uses iteration
________________________________________________________________________
If x is in the linear region

                        Name     Comp.   Div.    Mult.   Add.    Assign.
                                    1               1      2     1
________________________________________________________________________
 
 */

double SmoothSegmentedFunction::calcValue(double x) const
{
    double yVal = 0;
    if(x >= _x0 && x <= _x1 )
    {
        int idx  = SegmentedQuinticBezierToolkit::calcIndex(x,_mXVec);
        double u = SegmentedQuinticBezierToolkit::
                 calcU(x,_mXVec[idx], _arraySplineUX[idx], UTOL,MAXITER);
        yVal = SegmentedQuinticBezierToolkit::
                 calcQuinticBezierCurveVal(u,_mYVec[idx]);
    }else{
        if(x < _x0){
            yVal = _y0 + _dydx0*(x-_x0);            
        }else{
            yVal = _y1 + _dydx1*(x-_x1);                    
        }    
    }

    return yVal;
}

double SmoothSegmentedFunction::calcValue(const SimTK::Vector& ax) const
{
    
    SimTK_ERRCHK2_ALWAYS( ax.nelt() == 1,
        "SmoothSegmentedFunction::calcValue",
        "%s: Argument x must have only 1 element, as this function is "
        "designed only for 1D functions, but a function with %i elements was"
        "entered",_name.c_str(),ax.nelt());

    return calcValue(ax(0)); 
}

/*Detailed Computational Costs
________________________________________________________________________
If x is in the Bezier Curve, and dy/dx is being evaluated
                        Name     Comp.   Div.    Mult.   Add.    Assign.
_______________________________________________________________________
Overhead:
    SegmentedQuinticBezierToolkit::
                    calcIndex     3*m+2                   1*m     3   
                        *calcU     15      2      82      42     60  
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
                                    1                            1
________________________________________________________________________
    */

double SmoothSegmentedFunction::calcDerivative(double x, int order) const
{
    //return calcDerivative( SimTK::Array_<int>(order,0),
      //                     SimTK::Vector(1,x));
    double yVal = 0;

    //QUINTIC SPLINE

    
    if(order==0){
                yVal = calcValue(x);
    }else{
            if(x >= _x0 && x <= _x1){        
                int idx  = SegmentedQuinticBezierToolkit::calcIndex(x,_mXVec);
                double u = SegmentedQuinticBezierToolkit::
                                calcU(x,_mXVec[idx], _arraySplineUX[idx], 
                                UTOL,MAXITER);
                yVal = SegmentedQuinticBezierToolkit::
                            calcQuinticBezierCurveDerivDYDX(u, _mXVec[idx], 
                            _mYVec[idx], order);
/*
                            std::cout << _mX(3, idx) << std::endl;
                            std::cout << _mX(idx) << std::endl;*/
            }else{
                    if(order == 1){
                        if(x < _x0){
                            yVal = _dydx0;
                        }else{
                            yVal = _dydx1;}
                    }else{
                        yVal = 0;}   
                }
        }

    return yVal;
}



double SmoothSegmentedFunction::
    calcDerivative(const SimTK::Array_<int>& derivComponents,
                 const SimTK::Vector& ax) const
{
    for(int i=0; i < (signed)derivComponents.size(); i++){
        SimTK_ERRCHK2_ALWAYS( derivComponents[i] == 0,
        "SmoothSegmentedFunction::calcDerivative",
        "%s: derivComponents can only be populated with 0's because "
        "SmoothSegmentedFunction is only valid for a 1D function, but "
        "derivComponents had a value of %i in it",
        _name.c_str(), derivComponents[i]);
    }
    SimTK_ERRCHK2_ALWAYS( derivComponents.size() <= 6,
        "SmoothSegmentedFunction::calcDerivative",
        "%s: calcDerivative is only valid up to a 6th order derivative"
        " but derivComponents had a size of %i",
        _name.c_str(), derivComponents.size());

    SimTK_ERRCHK2_ALWAYS( ax.nelt() == 1,
        "SmoothSegmentedFunction::calcValue",
        "%s: Argument x must have only 1 element, as this function is "
        "designed only for 1D functions, but ax had a size of %i",
        _name.c_str(), ax.nelt());

    return calcDerivative(ax(0), derivComponents.size());
}

/*Detailed Computational Costs
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

*/
double SmoothSegmentedFunction::calcIntegral(double x) const
{
    SimTK_ERRCHK1_ALWAYS(_computeIntegral,
        "SmoothSegmentedFunction::calcIntegral",
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

bool SmoothSegmentedFunction::isIntegralAvailable() const
{
    return _computeIntegral;
}

bool SmoothSegmentedFunction::isIntegralComputedLeftToRight() const
{
    return _intx0x1;
}

int SmoothSegmentedFunction::getArgumentSize() const
{
    return 1;
}

int SmoothSegmentedFunction::getMaxDerivativeOrder() const
{
    return 6;
}

std::string SmoothSegmentedFunction::getName() const
{
    return _name;
}

void SmoothSegmentedFunction::setName(std::string &name) 
{
    _name = name;
}

SimTK::Vec2 SmoothSegmentedFunction::getCurveDomain() const
{
    SimTK::Vec2 xrange;
    
    xrange(0) = 0; 
    xrange(1) = 0; 
    if (!_mXVec.empty()) {
        xrange(0) = _mXVec[0](0); 
        xrange(1) = _mXVec[_mXVec.size()-1](_mXVec[0].size()-1); 
    }
    return xrange;
}

///////////////////////////////////////////////////////////////////////////////
// Utility functions
///////////////////////////////////////////////////////////////////////////////

/*Detailed Computational Costs

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
*/
SimTK::Matrix SmoothSegmentedFunction::calcSampledMuscleCurve(int maxOrder,
    double domainMin,
    double domainMax) const{
    int pts = 1; //Number of points between each of the spline points used
                  //to fit u(x), and also the integral spline
    SimTK_ERRCHK_ALWAYS(maxOrder <= getMaxDerivativeOrder(),
        "SmoothSegmentedFunction::calcSampledMuscleCurve",
        "Derivative order past the maximum computed order requested");

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
                x(i) = SegmentedQuinticBezierToolkit::
                    calcQuinticBezierCurveVal(u,_mXVec[s]);    
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

        
    SimTK::Vector xsmpl(pts*(midX.size()-1)+2*10*pts);
    
    SimTK::Matrix results;

    if(_computeIntegral){
        results.resize(pts*(midX.size()-1)+2*10*pts,maxOrder+2+1);
    }else{
        results.resize(pts*(midX.size()-1)+2*10*pts,maxOrder+2);
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
    if(domainMin < x0)
        x0 = domainMin;

    x1 = _x0;
    delta = (0.1)*(x1-x0)/(pts);

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
    if(domainMax > x1)
        x1 = domainMax;

    delta = (1.0/9.0)*(x1-x0)/(pts);

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
        if(maxOrder>=1)
        results(i,2) = calcDerivative(d1y,ax);

        if(maxOrder>=2)
        results(i,3) = calcDerivative(d2y,ax);
        
        if(maxOrder>=3)
        results(i,4) = calcDerivative(d3y,ax);
        
        if(maxOrder>=4)
        results(i,5) = calcDerivative(d4y,ax);
        
        if(maxOrder>=5)
        results(i,6) = calcDerivative(d5y,ax);
        
        if(maxOrder>=6)
        results(i,7) = calcDerivative(d6y,ax);

        if(_computeIntegral){
            results(i,maxOrder+2) = calcIntegral(ax(0));
        }
    }
   return results;
}


/*Detailed Computational Costs

_______________________________________________________________________
                        Name     Comp.   Div.    Mult.   Add.    Assign.
_______________________________________________________________________

                 *overhead     (17+3*m     2    82      42+m    63)*3
                                51+9m      6    246     126+3m   189

                calcValue                       21      20      13
    calcDerivative  : dy/dx                1    40      38      22          
                    : d2y/dx2              2    78      73      23

                **calcIntegral    7              2       3       1             

            total per point      58+9m     9    387    260+3m   248
            total per elbow    5.8k+900m   900  38.7k  26k+300m 24.8k
                   
    *Approximate. Overhead associated with finding the correct Bezier
                    spline section, and evaluating u(x). 
                    Assumes 2 Newton iterations in calcU

    **Approximate. Includes estimated cost of evaluating a cubic spline
                    with 100 knots
*/
void SmoothSegmentedFunction::printMuscleCurveToCSVFile(
                                                const std::string& path,
                                                double domainMin,
                                                double domainMax) const
{
    //Only compute up to the 2nd derivative
    SimTK::Matrix results = calcSampledMuscleCurve(2,domainMin,domainMax);
    SimTK::Array_<std::string> colNames(results.ncol());
    colNames[0] = "x";
    colNames[1] = "y";
    colNames[2] = "dy/dx";
    colNames[3] = "d2y/dx2";
    
    if(results.ncol() == 5){
        colNames[4] = "int_y(x)";
    }

            std::string fname = _name;
            SimTK_ERRCHK_ALWAYS(fname.length() > 0,
                "SmoothSegmentedFunction::printMuscleCurveToCSVFile",
                "Muscle Curve name is empty!");
            fname.append(".csv");

            printMatrixToFile(results,colNames,path,fname);
}
/*
This function will print cvs file of the column vector col0 and the matrix data
*/
void SmoothSegmentedFunction::
    printMatrixToFile(SimTK::Matrix& data, SimTK::Array_<std::string>& colNames,
    const std::string& path, const std::string& filename) const
{
    
    ofstream datafile;
    std::string fullpath = path;
    
    if(fullpath.length() > 0)
        fullpath.append("/");
    
    fullpath.append(filename);

    datafile.open(fullpath.c_str(),std::ios::out);

    if(!datafile){
        datafile.close();
        SimTK_ERRCHK2_ALWAYS( false, 
        "SmoothSegmentedFunction::printMatrixToFile",
        "%s: Failed to open the file path: %s", _name.c_str(),fullpath.c_str());
    }


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


