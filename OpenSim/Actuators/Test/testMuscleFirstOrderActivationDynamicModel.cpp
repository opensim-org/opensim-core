/* -------------------------------------------------------------------------- *
 *          OpenSim:  testMuscleFirstOrderActivationDynamicModel.cpp          *
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

#include "Simbody.h"
#include "OpenSim/OpenSim.h"
#include <OpenSim/Actuators/MuscleFirstOrderActivationDynamicModel.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void printMatrixToFile( const SimTK::Matrix& data, const std::string& filename);

SimTK::Vector calcCentralDifference(const SimTK::Vector& x, 
        const SimTK::Vector& y, bool extrap_endpoints);

bool isFunctionContinuous(const SimTK::Vector& xV, const SimTK::Vector& yV,
        const SimTK::Vector& dydxV, const SimTK::Vector& d2ydx2V, double minTol,
        double taylorErrorMult);

SimTK::Matrix calcFunctionTimeIntegral( 
        const SimTK::Vector& timeV, 
        const SimTK::Matrix& xM, 
        const MuscleFirstOrderActivationDynamicModel& yF,
        double ic, 
        int dim, 
        double startTime, 
        double endTime,
        double intAcc);

int main(int argc, char* argv[])
{


    try {
        SimTK_START_TEST("Testing MuscleFirstOrderDynamicModel");

        // Test property bounds.
        {
            MuscleFirstOrderActivationDynamicModel actMdl;
            actMdl.set_activation_time_constant(0.0);
            SimTK_TEST_MUST_THROW_EXC(actMdl.finalizeFromProperties(),
                    InvalidPropertyValue);
        }
        {
            MuscleFirstOrderActivationDynamicModel actMdl;
            actMdl.set_deactivation_time_constant(0.0);
            SimTK_TEST_MUST_THROW_EXC(actMdl.finalizeFromProperties(),
                    InvalidPropertyValue);
        }
        {
            MuscleFirstOrderActivationDynamicModel actMdl;
            actMdl.set_minimum_activation(-SimTK::SignificantReal);
            SimTK_TEST_MUST_THROW_EXC(actMdl.finalizeFromProperties(),
                    InvalidPropertyValue);
        }
        {
            MuscleFirstOrderActivationDynamicModel actMdl;
            actMdl.set_minimum_activation(1.0);
            SimTK_TEST_MUST_THROW_EXC(actMdl.finalizeFromProperties(),
                    InvalidPropertyValue);
        }

        /*
           Test conditions:

           1. Generate the step response and make sure that it stays between
           the minimum activation and the maximum (amin, and 1)
           2. Ensure that the derivative function is continuous
           3. Ensure that the 10%-90% rise time is about 3 time constants
           Ensure that the fall time is about 2.25 time constants.
           */


        int pts = 100;
        double stepStart = 0.2;
        double stepEnd   = 0.5;
        double amin = 0.01;

        double tauA   = 0.01;
        double tauD   = 0.04;



        ////////////////////////////
        //generate a step input
        ////////////////////////////

        SimTK::Vector timeV(pts);
        SimTK::Matrix xM(pts,2);

        for(int i=0; i<pts; i++){
            timeV(i) = ((double)i)/((double)pts-1.0);
            xM(i,0) = amin;
            if( timeV(i)<=stepStart || timeV(i)>=stepEnd ){
                xM(i,1) = amin;
            }else{
                xM(i,1) = 1;
            }                 
        }


        MuscleFirstOrderActivationDynamicModel actMdl;

        MuscleFirstOrderActivationDynamicModel actMdl2;
        actMdl2.set_activation_time_constant(2*tauA);
        actMdl2.set_deactivation_time_constant(2*tauD);
        actMdl2.set_minimum_activation(2*amin);

        cout << endl;
        cout<<"*****************************************************"<<endl;
        cout << "TEST: Serialization"<<endl;
        cout << endl;

        actMdl.print("default_MuscleFirstOrderActivationDynamicModel.xml");

        Object* tmpObj = Object::
            makeObjectFromFile("default_MuscleFirstOrderActivationDynamicModel.xml");
        actMdl2 = *dynamic_cast<MuscleFirstOrderActivationDynamicModel*>(tmpObj);
        delete tmpObj;

        SimTK_TEST(actMdl ==actMdl2);
        remove("default_MuscleFirstOrderActivationDynamicModel.xml");

        cout << endl;
        cout<<"*****************************************************"<<endl;
        printf("TEST: Activation bounds %f and 1 \n",amin);
        cout<<"       respected during step response"<<endl;
        cout << endl;


        SimTK::Matrix stepResponse = calcFunctionTimeIntegral(  timeV, 
                xM, 
                actMdl,
                amin, 
                0, 
                0, 
                1,
                1e-12);

        //printMatrixToFile( stepResponse, "stepResponse.csv");

        //TEST 1: Check bounds on the step response
        bool boundsRespected= true;

        for(int i=0; i<stepResponse.nrow(); i++){
            if(stepResponse(i,1) < amin-SimTK::Eps)
                boundsRespected = false;

            if(stepResponse(i,1) > 1+SimTK::Eps)
                boundsRespected = false;

        }
        SimTK_TEST(boundsRespected);
        printf("PASSED to a tol of %fe-16",SimTK::Eps*1e16);

        cout << endl;

        cout<<"*****************************************************"<<endl;
        cout<<"TEST: 10%-90% Rise time 3*tau_act +/- 10%"<<endl;
        cout<<"      90%-10% Fall time 2.17*tau_dact  +/- 10%" <<endl;
        cout<<"      And causality" << endl;
        cout << endl;

        double rt10 = 0;
        double rt90 = 0;
        double ft90 = 0;
        double ft10 = 0;

        double v10 = (1-amin)*0.10 + amin;
        double v90 = (1-amin)*0.90 + amin;

        for(int i=0; i<stepResponse.nrow()-1; i++){
            if(stepResponse(i,1) <= v10 && stepResponse(i+1,1) > v10)
                rt10 = stepResponse(i+1,0);

            if(stepResponse(i,1) <= v90 && stepResponse(i+1,1) > v90)
                rt90 = stepResponse(i+1,0);

            if(stepResponse(i,1) > v10 && stepResponse(i+1,1) <= v10)
                ft10 = stepResponse(i+1,0);

            if(stepResponse(i,1) > v90 && stepResponse(i+1,1) <= v90)
                ft90 = stepResponse(i+1,0);
        }

        double normRiseTime = (rt90-rt10)/tauA;
        double normFallTime = (ft10-ft90)/tauD;

        //Checking for causality
        SimTK_TEST(  (rt10>stepStart) && (rt90>rt10) 
                && (ft90>rt90) && (ft90>stepEnd) && (ft10>ft90)); 
        SimTK_TEST_EQ_TOL(normRiseTime,3,0.3);
        SimTK_TEST_EQ_TOL(normFallTime,2.17,0.2);

        printf("PASSED: with a normalized 10-90 percent rise time of %f\n"
                "         and a normalized 90-10 percent fall time of %f\n",
                normRiseTime,normFallTime);

        cout<<"*****************************************************"<<endl;
        cout<<"TEST: Continuity of d/dt activation w.r.t. excitation"<<endl;
        cout<<"       and constant activation" <<endl;
        cout << endl;
        //Generate a range of activation values
        SimTK::Vector actV(10);
        for(int i=0; i<actV.size(); i++){
            actV(i) = (1-amin)*((double)i)/((double)actV.size()-1.0) + amin;
        }

        //Generate a detailed range of excitations
        SimTK::Vector uV(100);
        for(int i=0; i<uV.size(); i++){
            uV(i) = ((double)i)/((double)uV.size()-1.0);
        }

        //For each activation value, generate the derivative curve, and
        //check its continuity
        SimTK::Vector dx(uV.size()), d2x(uV.size()), d3x(uV.size());
        dx = 0;
        d2x = 0;
        d3x = 0;
        xM.resize(uV.size(),2);

        /*
           calcCentralDifference(const SimTK::Matrix& xM,
           const SimTK::Function& yF,
           int dim,int order)
           */

        bool continuous = false;
        double tmp = 0;
        double maxDxDiff = 0;
        double minTol = 0.5;
        double taylorMult = 2.0000001;


        for(int i=0;i<actV.size();i++){

            for(int j=0; j<uV.size(); j++){
                xM(j,0) = actV(i);
                xM(j,1) = uV(j);
                dx(j) = actMdl.calcDerivative(actV(i), uV(j));
            }
            d2x = calcCentralDifference(xM(1), dx,true);                  

            for(int j=1;j<uV.size();j++){
                tmp = abs(d2x(j)-d2x(j-1));
                if( tmp > maxDxDiff)
                    maxDxDiff = tmp;
            }

            d3x = calcCentralDifference(xM(1), d2x, true);  

            continuous = isFunctionContinuous(xM(1), dx, d2x, d3x, 
                    minTol,taylorMult);
            SimTK_TEST(continuous);
        }
        printf("PASSED: with an allowable error of minTol of %f\n"
                "    or %f x the next Taylor series term. These values\n"
                "    are reasonable given the elbow in the function and\n"
                "    the maximum difference in slope of %f\n"
                ,minTol,taylorMult,maxDxDiff);

        cout << endl;
        cout<<"*****************************************************"<<endl;
        cout<<"TEST: Continuity of d/dt activation w.r.t. activation"<<endl;
        cout<<"       and constant excitation" <<endl;
        cout << endl;

        //Generate a range of activation values
        actV.resize(100);
        amin = actMdl.get_minimum_activation();
        for(int i=0; i<actV.size(); i++){
            actV(i) = (1-amin)*((double)i)/((double)actV.size()-1.0) + amin;
        }

        //Generate a detailed range of excitations
        uV.resize(10);
        for(int i=0; i<uV.size(); i++){
            uV(i) = ((double)i)/((double)uV.size()-1.0);
        }

        maxDxDiff = 0;
        for(int i=0;i<uV.size();i++){

            for(int j=0; j<actV.size(); j++){
                xM(j,0) = actV(j);
                xM(j,1) = uV(i);
                dx(j) = actMdl.calcDerivative(actV(j), uV(i));
            }                

            d2x = calcCentralDifference(xM(0), dx,true);                  



            for(int j=1;j<uV.size();j++){
                tmp = abs(d2x(j)-d2x(j-1));
                if( tmp > maxDxDiff)
                    maxDxDiff = tmp;
            }

            d3x = calcCentralDifference(xM(0), d2x, true);  

            continuous = isFunctionContinuous(xM(0), dx, d2x, d3x, 
                    minTol,taylorMult);
            SimTK_TEST(continuous);
        }
        printf("PASSED: with an allowable error of minTol of %f\n"
                "    or %f x the next Taylor series term. These values\n"
                "    are reasonable given the elbow in the function and\n"
                "    the maximum difference in slope of %f\n"
                ,minTol,taylorMult,maxDxDiff);

        cout<<"*****************************************************"<<endl;
        cout<<"TEST: Exceptions thrown correctly.                   "<<endl;           
        cout << endl;

        SimTK_END_TEST();
    }
    catch (const OpenSim::Exception& ex)
    {
        cout << ex.getMessage() << endl;
        cin.get();
        return 1;
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



    cout << "\ntestMuscleFirstOrderDynamicModel completed successfully .\n";
    return 0;
}

/**
  This function will print cvs file of the matrix 
  data

  @params data: A matrix of data
  @params filename: The name of the file to print
  */
void printMatrixToFile( const SimTK::Matrix& data, const std::string& filename)
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
  This function tests numerically for continuity of a curve. The test is 
  performed by taking a point on the curve, and then two points (called the 
  shoulder points) to the left and right of the point in question. The value
  of the functions derivative is evaluated at each of the shoulder points and
  used to linearly extrapolate from the shoulder points back to the original 
  point. If the original point and the linear extrapolations of each of the 
  shoulder points agree within tol, then the curve is assumed to be 
  continuous.


  @param xV       Values to test for continuity, note that the first and last
  points cannot be tested

  @param yV       Function values at the test points

  @param dydxV    Function derivative values evaluated at xV

  @param d2ydx2V  Function 2nd derivative values (or estimates) evaluated at
  xV

  @param minTol   The minimum error allowed - this prevents the second order
  error term from going to zero

  @param taylorErrorMult  This scales the error tolerance. The default error
  tolerance is the 2nd-order Taylor series
  term.
  */
bool isFunctionContinuous(const SimTK::Vector& xV, const SimTK::Vector& yV,
        const SimTK::Vector& dydxV, const SimTK::Vector& d2ydx2V, 
        double minTol, double taylorErrorMult)
{
    bool flag_continuous = true;
    //double contErr = 0;

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
        yVal = yV(i);

        xL = xV(i-1);
        xR = xV(i+1);

        yL = yV(i-1);
        yR = yV(i+1);

        dydxL = dydxV(i-1);
        dydxR = dydxV(i+1);


        yValEL = yL + dydxL*(xVal-xL);
        yValER = yR - dydxR*(xR-xVal);

        errL = abs(yValEL-yVal);
        errR = abs(yValER-yVal);

        errLMX = abs(d2ydx2V(i-1)*0.5*(xVal-xL)*(xVal-xL));
        errRMX = abs(d2ydx2V(i+1)*0.5*(xR-xVal)*(xR-xVal));

        errLMX*=taylorErrorMult;
        errRMX*=taylorErrorMult;

        if(errLMX < minTol)
            errLMX = minTol;

        if(errRMX < minTol)
            errRMX = minTol; // to accommodate numerical
        //error in errL

        if(errL > errLMX || errR > errRMX){
            /*if(errL > errR){
              if(errL > contErr)
              contErr = errL;

              }else{
              if(errR > contErr)
              contErr = errR;
              }*/

            flag_continuous = false;
        }
    }

    return flag_continuous;//contErr;//
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
SimTK::Vector calcCentralDifference(const SimTK::Vector& x, 
        const SimTK::Vector& y,                                          
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


///@cond
class FunctionData {
public:
    const MuscleFirstOrderActivationDynamicModel& m_func;
    SimTK::Array_<SimTK::Spline_<double> > m_splinedInput;

    double m_ic;        
    int m_intDim;

    mutable SimTK::Vector m_tmpXV; //A temporary variable   

    FunctionData(const MuscleFirstOrderActivationDynamicModel& func):m_func(func)
    {};
};


//#ifdef DEF



/**
  This is the nice user interface class to MySystemGuts, which creates a System
  object that is required to use SimTK's integrators to integrate the Bezier
  curve sets
  */
class MySystem;
/**
  Class that makes a system so that SimTK's integrators (which require a system)
  can be used to numerically integrate the BezierCurveSet. This class is used
  by the function calcNumIntBezierYfcnX, which evaluates the numerical integral
  of a Bezier curve set.

  A System is actually two classes: System::Guts does the work while System
  provides a pleasant veneer to make usage easier. This is the workhorse
  */
class MySystemGuts : public SimTK::System::Guts {
    friend class MySystem;

    MySystemGuts(const FunctionData afuncData) : funcData(afuncData) {}

    // Implement required System::Guts virtuals.
    MySystemGuts* cloneImpl() const override {return new MySystemGuts(*this);}

    // During realizeTopology() we allocate the needed State.
    int realizeTopologyImpl(State& state) const override {
        // HERE'S WHERE THE IC GETS SET
        Vector zInit(1, funcData.m_ic); // initial value for z
        state.allocateZ(SubsystemIndex(0), zInit);
        return 0;
    }

    // During realizeAcceleration() we calculate the State derivative.
    int realizeAccelerationImpl(const State& state) const override {
        Real x = state.getTime();
        Real z = state.getZ()[0];

        //if(z>funcData.m_ic+SimTK::Eps)
        //    printf("z: %f",z);

        //Sample the input vector at time t
        for(int i=0; i<funcData.m_tmpXV.size(); i++)
        {
            funcData.m_tmpXV(i) = funcData.m_splinedInput[i].
                calcValue(SimTK::Vector(1,x));
        }

        //Update the dimension of the vector that we're integrating along
        funcData.m_tmpXV(funcData.m_intDim) = z;

        //Compute the function derivative at the location of interest
        Real dz = funcData.m_func.calcDerivative(funcData.m_tmpXV[0], funcData.m_tmpXV[1]);
        //if(z>funcData.m_ic+SimTK::Eps)
        //    printf(" dz: %f\n",dz);

        state.updZDot()[0] = dz;
        return 0;
    }

    // Disable prescribe and project since we have no constraints or
    // prescribed state variables to worry about.
    int prescribeImpl(State&, Stage) const {return 0;}
    int projectImpl(State&, Real, const Vector&, const Vector&, 
            Vector&, SimTK::ProjectOptions) const {return 0;}
private:
    /**The Bezier curve data that is being integrated*/
    const FunctionData funcData;
};

/**
  This is the implementation of the nice user interface class to MySystemGuts, 
  which creates a System object that is required to use SimTK's integrators to 
  integrate the Bezier curve sets. Used in function
  SegmentedQuinticBezierToolkit::calcNumIntBezierYfcnX
  */
class MySystem : public System {
public:
    MySystem(const FunctionData& funcData) {
        adoptSystemGuts(new MySystemGuts(funcData));
        DefaultSystemSubsystem defsub(*this);
    }
};

/**
  @param timeV A nx1 time vector to integrate along, which must monotonic 
  and increasing
  @param xM    A nxm matrix of row vectors, each corresponding to the row vector
  that should be applied to yF at time t
  @param yF    A function
  @param ic    The initial condition for the integral
  @param intAcc The accuracy of the integral
  @returns an nx2 matrix, time in column 0, integral of y in column 1
  */
SimTK::Matrix calcFunctionTimeIntegral( 
        const SimTK::Vector& timeV,
        const SimTK::Matrix& xM, 
        const MuscleFirstOrderActivationDynamicModel& yF,
        double ic, 
        int dim, 
        double startTime, 
        double endTime,
        double intAcc)
{
    SimTK::Matrix intXY(timeV.size(),2);

    //Populate FunctionData
    FunctionData fdata(yF);
    fdata.m_ic      = ic;
    fdata.m_intDim  = dim;
    fdata.m_tmpXV      = SimTK::Vector(xM.ncol());
    fdata.m_tmpXV = 0;

    SimTK::Array_< SimTK::Spline_<double> > splinedInput(xM.ncol());

    //Now spline xM over time
    for(int i=0; i<xM.ncol(); i++){        

        splinedInput[i] = SimTK::SplineFitter<Real>::
            fitForSmoothingParameter(1,timeV,xM(i),0).getSpline();
    }
    fdata.m_splinedInput = splinedInput;
    //FunctionData is now completely built


    //Set up system
    //double startTime = timeV(0);
    //double endTime   = timeV(timeV.nelt()-1);   
    MySystem sys(fdata);
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

    while (idx < timeV.nelt()) {      
        nextTimeInterval = timeV(idx);

        status=integ.stepTo(nextTimeInterval);

        // Use this for variable step output.
        //status = integ.stepTo(Infinity);

        if (status == Integrator::EndOfSimulation)
            break;

        const State& state = integ.getState();

        intXY(idx,0) = nextTimeInterval;
        intXY(idx,1) = (double)state.getZ()[0];                        

        idx++;

    }

    //cout << "Integrated Solution"<<endl;
    //cout << intXY << endl;


    //intXY.resizeKeep(idx,2);
    return intXY;
}

//#endif
///@endcond
