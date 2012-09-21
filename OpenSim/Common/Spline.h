#ifndef _Spline_h_
#define _Spline_h_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Spline.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <stdio.h>
#include "osimCommonDLL.h"
#include "SimTKmath.h"

// DEFINES
#define SPLINE_MAXSIZE 2048

using SimTK::Vector;
//=============================================================================
//=============================================================================
/**
 * A class for representing smooth functions with b-splines.
 */
namespace OpenSim { 

// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond
class OSIMCOMMON_API Spline
{
//=============================================================================
// DATA
//=============================================================================
private:
	int _status;
	char _name[SPLINE_MAXSIZE];
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
	int getKnotIndex(double x) const;
    virtual double calcValue(const SimTK::Vector& x) const;

	//--------------------------------------------------------------------------
	// PRINTING
	//--------------------------------------------------------------------------
	void print();

//=============================================================================
};	// END class Spline
/// @endcond
}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Spline_h__
