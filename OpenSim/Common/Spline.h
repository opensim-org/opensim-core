#ifndef _Spline_h_
#define _Spline_h_
// Spline.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */



// INCLUDES
#include <stdio.h>
#include "osimCommonDLL.h"

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
