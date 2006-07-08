// Mtx.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <iostream>
#include <string>
#include "Signal.h"
#include "rdMath.h"
#include "Mtx.h"
#include "Array.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// FILTERS
//=============================================================================
//-----------------------------------------------------------------------------
// IIR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * 3rd ORDER LOWPASS IIR BUTTERWORTH DIGITAL DIGITAL FILTER
 *
 * It is assumed that enough memory is allocated at sigf.
 * Note also that the first and last three data points are not filtered.
 *
 *	@param T Sample interval in seconds.
 *	@param fc Cutoff frequency in Hz.
 *	@param N Number of data points in the signal.
 *	@param sig The sampled signal.
 *	@param sigf The filtered signal.
 *
 * @return 0 on success, and -1 on failure.
 */
int Signal::
LowpassIIR(double T,double fc,int N,double *sig,double *sigf)
{
int i,j;
double fs,ws,wc,wa,wa2,wa3;
double a[4],b[4],denom;
double *sigr;

	// ERROR CHECK
	if(T==0) return(-1);
	if(N==0) return(-1);
	if(sig==NULL) return(-1);
	if(sigf==NULL) return(-1);

	// CHECK THAT THE CUTOFF FREQUENCY IS LESS THAN HALF THE SAMPLE FREQUENCE
	fs = 1 / T;
	if (fc >= 0.5 * fs) {
		printf("\nCutoff frequency should be less than half sample frequency.");
		printf("\nchanging the cutoff frequency to 0.49*(Sample Frequency)...");
		fc = 0.49 * fs;
		printf("\ncutoff = %lf\n\n",fc);
	}

	// INITIALIZE SOME VARIABLES
	ws = 2*rdMath::PI*fs;
	wc = 2*rdMath::PI*fc;

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
		printf("\nSignal.lowpassIIR: ERROR- Not enough memory.\n");
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
	if(sigr!=NULL)  delete sigr;

  return(0);
}

//-----------------------------------------------------------------------------
// FIR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * LOWPASS FIR NONRECURSIVE DIGITAL FILTER
 *
 * It is permissible for sig and sigf to be the same array or overlap.
 *
 * PARAMETERS
 *	@param M Order of filter (should be 30 or greater).
 *	@param T Sample interval in seconds.
 *	@param f Cutoff frequency in Hz.
 *	@param N Number of data points in the signal.
 *	@param sig The sampled signal.
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
		printf("rdSingal.lowpassFIR:  ERROR- The number of data points (%d)",N);
		printf(" should be at least twice the order of the filter (%d).\n",M);
		return(-1);
	}

	// PAD THE SIGNAL SO FILTERING CAN BEGIN AT THE FIRST DATA POINT
	double *s = Pad(M,N,sig);
	if(s==NULL) return(-1);

	// CALCULATE THE ANGULAR CUTOFF FREQUENCY
	w = 2.0*rdMath::PI*f;

	// FILTER THE DATA
	for(n=0;n<N;n++) {
		sigf[n] = 0.0;
		for(k=-M;k<=M;k++) {   
			x = (double)k*w*T;
			sigf[n] = sigf[n] + (sinc(x)*T*w/rdMath::PI)*hamming(k,M)*s[M+n-k];
		}
	}

	// CLEANUP
  delete[] s;

  return(0);
}



//_____________________________________________________________________________
/**
 * BANDPASS FIR NONRECURSIVE DIGITAL FILTER
 *
 * Note that sig and sigf must point to distinct locations in memory which do
 * not overlap.
 *
 * PARAMETERS
 *	@param M Order of filter (should be 30 or greater).
 *	@param T Sample interval in seconds.
 *	@paramf1 lowend cutoff frequency in Hz
 *	@paramf2 highend cutoff frequency in Hz
 *	@param N Number of data points in the signal.
 *	@param sig The sampled signal.
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
    printf("\n\nThe number of data points (%d) should be at least twice\n",N);
    printf("the order of the filter (%d).\n\n",M);
    return(-1);
	}

	// ALLOCATE MEMORY FOR s
	size = N + M + M;
	s = (double *) calloc(size,sizeof(double));
	if (s == NULL) {
		printf("\n\nlowpass() -> Not enough memory to process your sampled data.");
		return(-1);
	}

	// CALCULATE THE ANGULAR CUTOFF FREQUENCY
	w1 = 2*rdMath::PI*f1;
	w2 = 2*rdMath::PI*f2;

	// PAD THE SIGNAL SO FILTERING CAN BEGIN AT THE FIRST DATA POINT
	for (i=0,j=M;i<M;i++,j--)          s[i] = sig[j];
	for (i=M,j=0;i<M+N;i++,j++)        s[i] = sig[j];
	for (i=M+N,j=N-2;i<M+M+N;i++,j--)  s[i] = sig[j];
  

	// FILTER THE DATA
	for (n=0;n<N;n++) {
		sigf[n] = 0.0;
		for (k=-M;k<=M;k++) {   
			x1 = (double)k*w1*T;
			x2 = (double)k*w2*T;
			sigf[n] = sigf[n] +
			 (sinc(x2)*T*w2/rdMath::PI - sinc(x1)*T*w1/rdMath::PI)*
			 hamming(k,M)*s[M+n-k];
		}
	}

	// CLEANUP
	if(s!=NULL) delete s;

  return(0);
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
 *	@param aPad Size of the pad-- number of points to prepend and append.
 *	@param aN Number of data points in the signal.
 *	@param aSignal Signal to be padded.
 *	@return Padded signal.  The size is aN + 2*aPad.  NULL is returned
 * on an error.  The caller is responsible for deleting the returned
 * array.
 */
double* Signal::
Pad(int aPad,int aN,const double aSignal[])
{
	if(aPad<=0) return(NULL);

	// COMPUTE FINAL SIZE
	int size = aN + 2*aPad;
	if(aPad>aN) {
		cout<<"\nSignal.Pad(double[]): ERROR- requested pad size ("<<aPad<<") is greater than the number of points ("<<aN<<").\n";
		return(NULL);
	}

	// ALLOCATE
	double *s = new double[size];
	if (s == NULL) {
		printf("\n\nSignal.Pad: Failed to allocate memory.\n");
		return(NULL);
	}

	// PREPEND
	int i,j;
	for(i=0,j=aPad;i<aPad;i++,j--)  s[i] = aSignal[j];
	for(i=0;i<aPad;i++)  s[i] = 2.0*aSignal[0] - s[i];

	// SIGNAL
	for(i=aPad,j=0;i<aPad+aN;i++,j++)  s[i] = aSignal[j];

	// APPEND
	for(i=aPad+aN,j=aN-2;i<aPad+aPad+aN;i++,j--)  s[i] = aSignal[j];
	for(i=aPad+aN;i<aPad+aPad+aN;i++)  s[i] = 2.0*aSignal[aN-1] - s[i];
 
	return(s);
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
 *	@param aPad Size of the pad-- number of points to prepend and append.
 *	@param rSignal Signal to be padded.
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
		cout<<"\n\nSignal.ReduceNumberOfPoints:: quitting.  The time and signal ";
		cout<<"arrays have different numbers of points.\n\n";
		return(0);
	}

	// CHECK ANGLE
	if(aDistance<rdMath::ZERO) aDistance = rdMath::ZERO;

	// APPEND FIRST POINT
	Array<double> t(0.0,0,size),s(0.0,0,size);
	t.append(rSignal[0]);
	s.append(rSignal[0]);
	int iLast=0;

	// REMOVE POINTS
	int i,imid;
	double p1[3] = {0.0,0.0,0.0};
	double p2[3] = {0.0,0.0,0.0};
	double p3[3] = {0.0,0.0,0.0};
	double v1[3] = {0.0,0.0,0.0};
	double v2[3] = {0.0,0.0,0.0};
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

		Mtx::Subtract(1,3,p2,p1,v1);
		Mtx::Subtract(1,3,p3,p1,v2);

		mv1 = Mtx::Magnitude(3,v1);
		mv2 = Mtx::Magnitude(3,v2);
		cos = Mtx::DotProduct(3,v1,v2) / (mv1*mv2); 

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
 * Hamming Window- dampens gibs phenominon infiltering functions.
 */
double Signal::
hamming(int k,int M)
{
  double x = (double)k * rdMath::PI / (double)M;
  double d = 0.54 + 0.46*cos(x);
  return(d); 
}


