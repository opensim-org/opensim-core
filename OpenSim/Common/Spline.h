#ifndef _Spline_h_
#define _Spline_h_
// Spline.cpp
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
#include "osimCommon.h"

// DEFINES
#define RDSPLINE_MAXSIZE 2048


//=============================================================================
//=============================================================================
/**
 * A class for representing smooth functions with b-splines.
 */
namespace OpenSim { 

class OSIMCOMMON_API Spline
{
//=============================================================================
// DATA
//=============================================================================
private:
	int _status;
	char _name[RDSPLINE_MAXSIZE];
	double _ti,_tf;
	int _order;
	int _nknots;
	int _ncoefs;
	double *_knots;
	double *_coefs;
	double *_tx;
	double *_b;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	//Spline(const char *aName,double aTI,double aTF,int aOrder,
	// int aNKnots,double *aKnots,int aNCoefs,double *aCoefs);
	virtual ~Spline();
	Spline(const char* aFileName);
	Spline(FILE *aFP);
private:
	int initialize(FILE *aFP);
	void null();
	int checkFileStatus(int aStatus);

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
public:
	int getKnotIndex(double x);
	double evaluate(double x);

	//--------------------------------------------------------------------------
	// PRINTING
	//--------------------------------------------------------------------------
	void print();

//=============================================================================
};	// END class Spline

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Spline_h__
