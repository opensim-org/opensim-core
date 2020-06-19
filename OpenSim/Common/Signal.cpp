/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Signal.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <math.h>
#include "Signal.h"
#include "Array.h"
#include "SimTKcommon/Constants.h"
#include "SimTKcommon/Orientation.h"
#include "SimTKcommon/Scalar.h"
#include "SimTKcommon/SmallMatrix.h"
#include "simmath/internal/Spline.h"
#include "simmath/internal/SplineFitter.h"

using namespace OpenSim;
using namespace std;

//=============================================================================
// FILTERS
//=============================================================================
//-----------------------------------------------------------------------------
// GCVSPL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Generalized, cross-validatory spline smoothing (Adv. Engng. Softw. 8:104-113)
 *
 *  @param degree Degree of the spline.
 *  @param T Sample interval in seconds.
 *  @param fc Cutoff frequency in Hz.
 *  @param N Number of data points in the signal.
 *  @param times The times for a specified signal samples.
 *  @param sig The sampled signal.
 *  @param sigf The filtered signal.
 *
 * @return 0 on success, and -1 on failure.
 */
int Signal::
SmoothSpline(int degree,double T,double fc,int N,double *times,double *sig,double *sigf)
{
    /* Background for choice of smoothing parameter (by Ton van den Bogert):

    In the GCVSPL software, VAL is the "smoothing parameter" of the spline, 
    which occurs as a weighting factor in a cost function which is 
    minimized.  This produces a compromise between how well the spline 
    fits the measurements and its smoothness (for a more detailed 
    explanation, see Ton van den Bogert's notes in 
    http://isbweb.org/software/sigproc/bogert/filter.pdf).

    Woltring describes the relationship between VAL and frequency domain 
    filter characteristics in his release notes: 
    http://isbweb.org/software/sigproc/gcvspl/gcvspl.memo.  
    This states that a smoothing spline is equivalent to a 

          "double, phase-symmetric Butterworth filter, with transfer function 
          H(w) = [1 + (w/wo)^2M]^-1, where  w  is  the  frequency, 
          wo = (p*T)^(-0.5/M) the filter's cut-off frequency, p the smoothing 
          parameter, T the sampling interval, and  2M the  order  of the spline.  
          If T is expressed in seconds, the frequencies are expressed in 
          radians/second."

    The VAL calculations can be derived from this.  The equation for the 
    transfer function has the property H(w0)= 0.5.  This is because it is 
    a double Butterworth filter (applied twice).  Cut-off frequency is 
    usually defined as the frequency at which H=1/sqrt(2), and this is why 
    Tony Reina and Ton van den Bogert have an extra factor in the equation.

    You can easily verify the correctness of your spline smoothing by 
    processing a sine wave signal at the cut-off frequency.  If it comes 
    out as a sine wave with its amplitude reduced by a factor 1.41 (=sqrt(2)), 
    you have used the correct VAL.

    */

    SimTK::Vector x(N);
    SimTK::Vector_<SimTK::Vec<1> > y(N);
    for (int i = 0; i < N; ++i)
        x[i] = times[i];
    for (int i = 0; i < N; ++i)
        y[i] = SimTK::Vec<1>(sig[i]);

    int M = (degree+1)/2; // Half-order
    SimTK::Real p; // Smoothing parameter
    p = (1.0/T) / 
        (pow( 
            (2.0*SimTK_PI*fc) / 
            (pow(
                (sqrt(2.0)-1),
                (0.5/M)
                )),
            (2*M)));

    // SMOOTH SPLINE FIT TWICE TO SIMULATE DOUBLE, PHASE-SYMMETRIC BUTTERWORTH FILTER
    SimTK::Spline_<SimTK::Vec<1> > smoothSpline1;
    smoothSpline1 = SimTK::SplineFitter<SimTK::Vec<1> >::fitForSmoothingParameter(degree,x,y,p).getSpline();
    SimTK::Spline_<SimTK::Vec<1> > smoothSpline2;
    smoothSpline2 = SimTK::SplineFitter<SimTK::Vec<1> >::fitForSmoothingParameter(degree,x,smoothSpline1.getControlPointValues(),p).getSpline();

    for (int i = 0; i < N; ++i)
        sigf[i] = smoothSpline2.getControlPointValues()[i][0];

    // CHECK THAT P IS NOT BEYOND BOUND
    SimTK::Real pActual;
    pActual = SimTK::SplineFitter<SimTK::Vec<1> >::fitForSmoothingParameter(degree,x,smoothSpline2.getControlPointValues(),p).getSmoothingParameter();
    //cout << "Requested smoothing parameter = " << p << endl;
    //cout << "Actual smoothing parameter    = " << pActual << endl;
    if(p!=pActual) {
        log_error("Signal.SmoothSpline: The cutoff frequency ({}) produced a "
                  "smoothing parameter ({}) beyond its bound ({}).",
                  fc, p, pActual);
        return(-1);
    }

  return(0);
}

//-----------------------------------------------------------------------------
// IIR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * 3rd ORDER LOWPASS IIR BUTTERWORTH DIGITAL FILTER
 *
 * It is assumed that enough memory is allocated at sigf.
 * Note also that the first and last three data points are not filtered.
 *
 *  @param T Sample interval in seconds.
 *  @param fc Cutoff frequency in Hz.
 *  @param N Number of data points in the signal.
 *  @param sig The sampled signal.
 *  @param sigf The filtered signal.
 *
 * @return 0 on success, and -1 on failure.
 */
int Signal::
LowpassIIR(double T,double fc,int N,const double *sig,double *sigf)
{
int i,j;
double fs/*,ws*/,wc,wa,wa2,wa3;
double a[4],b[4],denom;
double *sigr;

    // ERROR CHECK
    if(T==0) return(-1);
    if(N==0) return(-1);
    if(sig==NULL) return(-1);
    if(sigf==NULL) return(-1);

    // CHECK THAT THE CUTOFF FREQUENCY IS LESS THAN HALF THE SAMPLE FREQUENCY
    fs = 1 / T;
    if (fc >= 0.5 * fs) {
        fc = 0.49 * fs;
        log_warn("Cutoff frequency should be less than half sample frequency. "
                 "Changing the cutoff frequency to 0.49*(Sample Frequency)..."
                 "cutoff = {}", fc);
    }

    // INITIALIZE SOME VARIABLES
    //ws = 2*SimTK_PI*fs;
    wc = 2*SimTK_PI*fc;

    // CALCULATE THE FREQUENCY WARPING
    wa = tan(wc*T/2.0);
    wa2 = wa*wa;
    wa3 = wa*wa*wa;

    // GET COEFFICIENTS FOR THE FILTER
    denom = (wa+1) * (wa*wa + wa + 1.0);
    a[0] = wa3 / denom;
    a[1] = 3*wa3 / denom;
    a[2] = 3*wa3 / denom;
    a[3] = wa3 / denom;
    b[0] = 1;
    b[1] = (3*wa3 + 2*wa2 - 2*wa - 3) / denom; 
    b[2] = (3*wa3 - 2*wa2 - 2*wa + 3) / denom; 
    b[3] = (wa - 1) * (wa2 - wa + 1) / denom;

    // ALLOCATE MEMORY FOR sigr[]
    sigr = new double[N];
    if(sigr==NULL) {
        log_error("Signal.lowpassIIR: Not enough memory.");
        return(-1);
    }

    // FILTER THE DATA
    // FILL THE 1ST THREE TERMS OF sigf
    for (i=0;i<=3;i++) sigf[i] = sig[i];

    // IMPLEMENT THE FORMULA
    for (i=3;i<N;i++) {
        sigf[i] = a[0]*sig[i] + a[1]*sig[i-1] +  a[2]*sig[i-2] +  a[3]*sig[i-3]
                            - b[1]*sigf[i-1] - b[2]*sigf[i-2] - b[3]*sigf[i-3];
    }

    // REVERSE THE FILTERED ARRAY
    for (i=0,j=N-1;i<N;i++,j--)  sigr[i] = sigf[j]; 

    // FILL THE 1ST THREE TERMS OF sigf
    for (i=0;i<=3;i++) sigf[i] = sigr[i];

    // IMPLEMENT THE FORMULA AGAIN
    for (i=3;i<N;i++) {
        sigf[i] = a[0]*sigr[i] + a[1]*sigr[i-1] +  a[2]*sigr[i-2] +  a[3]*sigr[i-3]
                             - b[1]*sigf[i-1] - b[2]*sigf[i-2] - b[3]*sigf[i-3];
    }

    // REVERSE THE FILTERED ARRAY AGAIN
    for (i=0,j=N-1;i<N;i++,j--)  sigr[i] = sigf[j]; 

    // ASSIGN sigf TO sigr
    for (i=0;i<N;i++)  sigf[i] = sigr[i];

    // CLEANUP
    if(sigr!=NULL)  delete[] sigr;

  return(0);
}

//-----------------------------------------------------------------------------
// FIR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * LOWPASS FIR NON-RECURSIVE DIGITAL FILTER
 *
 * It is permissible for sig and sigf to be the same array or overlap.
 *
 * PARAMETERS
 *  @param M Order of filter (should be 30 or greater).
 *  @param T Sample interval in seconds.
 *  @param f Cutoff frequency in Hz.
 *  @param N Number of data points in the signal.
 *  @param sig The sampled signal.
 * @param sigf The filtered signal.
 *
 * @return 0 on success, and -1 on failure.
 */
int Signal::
LowpassFIR(int M,double T,double f,int N,double *sig,double *sigf)
{
    int n,k;
    double w,x;

    // CHECK THAT M IS NOT TOO LARGE RELATIVE TO N
    if((M+M)>N) {
        log_error("rdSingal.lowpassFIR: The number of data points ({}) "
                  "should be at least twice the order of the filter ({}).",
                N, M);
        return(-1);
    }

    // PAD THE SIGNAL SO FILTERING CAN BEGIN AT THE FIRST DATA POINT
    std::vector<double> s = Pad(M,N,sig);

    // CALCULATE THE ANGULAR CUTOFF FREQUENCY
    w = 2.0*SimTK_PI*f;

    // FILTER THE DATA
    double sum_coef,coef;
    for(n=0;n<N;n++) {
        sum_coef = 0.0;
        sigf[n] = 0.0;
        for(k=-M;k<=M;k++) {   
            x = (double)k*w*T; // k*T = time (seconds) and w scales sinc input argument using filter cutoff
            coef = (sinc(x)*T*w/SimTK_PI)*hamming(k,M); // scale lowpass sinc amplitude by 2*f*T = T*w/pi
            sigf[n] = sigf[n] + coef*s[M+n-k]; 
            sum_coef = sum_coef + coef;
        }
        sigf[n] = sigf[n] / sum_coef; // normalize for unity gain at DC
    }

    // Filter check derived from http://www.dspguide.com/CH16.PDF
    //double sum_coef,coef;
    //for(n=0;n<N;n++) {
    //  sum_coef = 0.0;
    //  sigf[n] = 0.0;
    //  for(k=-M;k<=M;k++) {
    //      if(k==0) {
    //          coef = 2.0*SimTK_PI*f;
    //      } else {
    //          coef = sin((double)k*2.0*SimTK_PI*f) / (double)k;
    //      }
    //      coef = coef*hamming(k,M);
    //      sigf[n] = sigf[n] + coef*s[M+n-k];
    //      sum_coef = sum_coef + coef;
    //  }
    //  sigf[n] = sigf[n] / sum_coef;
    //}

  return 0;
}



//_____________________________________________________________________________
/**
 * BANDPASS FIR NON-RECURSIVE DIGITAL FILTER
 *
 * Note that sig and sigf must point to distinct locations in memory which do
 * not overlap.
 *
 * PARAMETERS
 *  @param M Order of filter (should be 30 or greater).
 *  @param T Sample interval in seconds.
 *  @paramf1 lowend cutoff frequency in Hz
 *  @paramf2 highend cutoff frequency in Hz
 *  @param N Number of data points in the signal.
 *  @param sig The sampled signal.
 * @param sigf The filtered signal.
 *
 * @return 0 on success, and -1 on failure.
 */
int Signal::
BandpassFIR(int M,double T,double f1,double f2,int N,double *sig,
    double *sigf)
{
size_t size;
int i,j;
int n,k;
double w1,w2,x1,x2;
double *s;


    // CHECK THAT M IS NOT TOO LARGE RELATIVE TO N
    if((M+M)>N) {
        log_error("The number of data points ({}) "
                  "should be at least twice the order of the filter ({}).",
                N, M);
        return(-1);
    }

    // ALLOCATE MEMORY FOR s
    size = N + M + M;
    s = (double *) calloc(size,sizeof(double));
    if (s == NULL) {
        log_error(
                "lowpass() -> Not enough memory to process your sampled data.");
        return(-1);
    }

    // CALCULATE THE ANGULAR CUTOFF FREQUENCY
    w1 = 2*SimTK_PI*f1;
    w2 = 2*SimTK_PI*f2;

    // PAD THE SIGNAL SO FILTERING CAN BEGIN AT THE FIRST DATA POINT
    for (i=0,j=M;i<M;i++,j--)          s[i] = sig[j];
    for (i=M,j=0;i<M+N;i++,j++)        s[i] = sig[j];
    for (i=M+N,j=N-2;i<M+M+N;i++,j--)  s[i] = sig[j];
  

    // FILTER THE DATA
    double sum_coef,coef;
    for (n=0;n<N;n++) {
        sum_coef = 0.0;
        sigf[n] = 0.0;
        for (k=-M;k<=M;k++) {   
            x1 = (double)k*w1*T;  // k*T = time (seconds) and w scales sinc input argument using filter cutoff
            x2 = (double)k*w2*T;  // k*T = time (seconds) and w scales sinc input argument using filter cutoff
            coef = (sinc(x2)*T*w2/SimTK_PI - sinc(x1)*T*w1/SimTK_PI)*hamming(k,M); // scale lowpass sinc amplitude by 2*f*T = T*w/pi
            sigf[n] = sigf[n] + coef*s[M+n-k];
            sum_coef = sum_coef + coef;
        }
        sigf[n] = sigf[n] / sum_coef; // normalize for unity gain at DC
    }

    // CLEANUP
    if(s!=NULL) delete s;

  return(0);
}

std::vector<double> Signal::
Pad(int aPad,int aN,const double aSignal[])
{
    if (aPad == 0) return std::vector<double>(aSignal, aSignal + aN);
    OPENSIM_THROW_IF(aPad < 0, Exception,
            "Expected aPad to be non-negative, but got {}.",
            aPad);

    // COMPUTE FINAL SIZE
    int size = aN + 2*aPad;
    OPENSIM_THROW_IF(aPad > aN, Exception,
            "Signal.Pad: requested pad size ({}) is greater than the "
            "number of points ({}).",
            aPad, aN);

    // ALLOCATE
    std::vector<double> s(size);

    // PREPEND
    int i,j;
    for(i=0,j=aPad;i<aPad;i++,j--)  s[i] = aSignal[j];
    for(i=0;i<aPad;i++)  s[i] = 2.0*aSignal[0] - s[i];

    // SIGNAL
    for(i=aPad,j=0;i<aPad+aN;i++,j++)  s[i] = aSignal[j];

    // APPEND
    for(i=aPad+aN,j=aN-2;i<aPad+aPad+aN;i++,j--)  s[i] = aSignal[j];
    for(i=aPad+aN;i<aPad+aPad+aN;i++)  s[i] = 2.0*aSignal[aN-1] - s[i];
 
    return s;
}
//_____________________________________________________________________________
/**
 * Pad a signal with a specified number of data points.
 *
 * The signal is prepended and appended with a reflected and negated
 * portion of the signal of the appropriate size so as to preserve the value
 * and slope of the signal.
 *
 * PARAMETERS
 *  @param aPad Size of the pad-- number of points to prepend and append.
 *  @param rSignal Signal to be padded.
 */
void Signal::
Pad(int aPad,Array<double> &rSignal)
{
    if(aPad<=0) return;

    // COMPUTE NEW SIZE
    int size = rSignal.getSize();
    int newSize = size + 2*aPad;

    // HANDLE PAD GREATER THAN SIZE
    if(aPad>=size) {
        int pad = size - 1;
        while(size<newSize) {
            Pad(pad,rSignal);
            size = rSignal.getSize();
            pad = (newSize - size) / 2;
            if(pad>=size) pad = size - 1;
        }
        return;
    }

    // ALLOCATE
    Array<double> s(0.0,newSize);

    // PREPEND
    int i,j;
    for(i=0,j=aPad;i<aPad;i++,j--)  s[i] = rSignal[j];
    for(i=0;i<aPad;i++)  s[i] = 2.0*rSignal[0] - s[i];

    // SIGNAL
    for(i=aPad,j=0;i<aPad+size;i++,j++)  s[i] = rSignal[j];

    // APPEND
    for(i=aPad+size,j=size-2;i<aPad+aPad+size;i++,j--)  s[i] = rSignal[j];
    for(i=aPad+size;i<aPad+aPad+size;i++)  s[i] = 2.0*rSignal[size-1] - s[i];

    // ALTER SIGNAL
    rSignal = s;
}


//-----------------------------------------------------------------------------
// POINT REDUCTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Remove points in a signal based on the angle between adjacent segments in
 * the signal.  The end points of the signal are always retained.
 * 
 * @param aAngle If the angle between two adjacent segments is less than
 * aAngle the point in common between the two segments is removed.  This
 * is evaluate for each point in the signal.
 * @param rTime Array of time values.  This array is altered.
 * @param rSignal Array of signal values.  This array is altered.
 * @return Number of points removed.
 */
int Signal::
ReduceNumberOfPoints(double aDistance,
                            Array<double> &rTime,Array<double> &rSignal)
{
    // CHECK SIZES
    int size = rTime.getSize();
    int sizeSignal = rSignal.getSize();
    if(size!=sizeSignal) {
        log_error("Signal.ReduceNumberOfPoints:: quitting.  The time and "
                  "signal arrays have different numbers of points.");
        return(0);
    }

    // CHECK ANGLE
    if(aDistance<SimTK::Zero) aDistance = SimTK::Zero;

    // APPEND FIRST POINT
    Array<double> t(0.0,0,size),s(0.0,0,size);
    t.append(rSignal[0]);
    s.append(rSignal[0]);
    int iLast=0;

    // REMOVE POINTS
    int i,imid;
    SimTK::Vec3 p1(0.0,0.0,0.0);
    SimTK::Vec3 p2(0.0,0.0,0.0);
    SimTK::Vec3 p3(0.0,0.0,0.0);
    SimTK::Vec3 v1(0.0,0.0,0.0);
    SimTK::Vec3 v2(0.0,0.0,0.0);

    double tmid,mv1,mv2,cos,dsq;
    for(i=1;i<(size-1);i++) {

        // FIRST POINT
        // LAST POINT IN SIMPLIFIED ARRAY
        p1[0] = t.getLast();
        p1[1] = s.getLast();

        // THIRD POINT
        p3[0] = rTime[i+1];
        p3[1] = rSignal[i+1];

        // SECOND POINT
        // MIDPOINT
        tmid = 0.5*(p3[0]-p1[0]) + p1[0];
        imid = rTime.searchBinary(tmid,false,iLast,i+1);
        if(imid<=iLast) imid++;
        p2[0] = rTime[imid];
        p2[1] = rSignal[imid];

        v1 = p2 - p1; //Mtx::Subtract(1,3,p2,p1,v1);
        v2 = p3 - p1; //Mtx::Subtract(1,3,p3,p1,v2);

        mv1 = v1.norm(); //Mtx::Magnitude(3,v1);
        mv2 = v2.norm(); //Mtx::Magnitude(3,v2);
        cos = (~v1*v2)/(mv1*mv2); //Mtx::DotProduct(3,v1,v2) / (mv1*mv2); 

        dsq = mv1 * mv1 * (1.0 - cos*cos);

        if(dsq < (aDistance*aDistance)) continue;

        iLast = i;
        t.append(rTime[i]);
        s.append(rSignal[i]);
    }

    // APPEND ENDPOINT
    t.append(rTime.getLast());
    s.append(rSignal.getLast());

    // NUMBER REMOVED
    int numberRemoved = rTime.getSize() - t.getSize();

    // ALTER ARRAYS
    rTime = t;
    rSignal = s;

    return(numberRemoved);
}



//=============================================================================
// CORE MATH
//=============================================================================
//_____________________________________________________________________________
/**
 * SINC(X) = SIN(X) / X
 *
 * @return sin(x)/x.
 */
double Signal::
sinc(double x)
{
  if ((x<1.0e-8) && (x>-1.0e-8)) return(1.0);
  return(sin(x)/x);
}
//_____________________________________________________________________________
/**
 * Hamming Window- dampens Gibbs phenomenon in filtering functions.
 */
double Signal::
hamming(int k,int M)
{
  double x = (double)k * SimTK_PI / (double)M;
  double d = 0.54 + 0.46*cos(x);
  return(d); 
}


