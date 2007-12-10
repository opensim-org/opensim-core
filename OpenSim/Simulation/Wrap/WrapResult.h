#ifndef __WrapResult_h__
#define __WrapResult_h__

// WrapResult.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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


// INCLUDE
#include <iostream>
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/SimmPoint.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for holding the results of a wrapping calculation.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API WrapResult
{

//=============================================================================
// DATA
//=============================================================================
public:
	int startPoint;            // first point in muscle line that is wrapped
	int endPoint;              // second point in muscle line that is wrapped
	Array<SimmPoint> wrap_pts; // array of wrapping path points
   double wrap_path_length;   // distance along curved r1->r2 path
   double r1[3];              // wrap tangent point nearest to p1
   double r2[3];              // wrap tangent point nearest to p2
	double c1[3];              // intermediate point used by some wrap objects
	double sv[3];              // intermediate point used by some wrap objects
	double factor;             // scale factor used to normalize parameters

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	WrapResult();
	virtual ~WrapResult();
	void copyData(const WrapResult& aWrapResult);
	WrapResult& operator=(const WrapResult& aWrapResult);

//=============================================================================
};	// END of class WrapResult
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapResult_h__


