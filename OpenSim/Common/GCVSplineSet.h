#ifndef _GCVSplineSet_h_
#define _GCVSplineSet_h_
// GCVSplineSet.cpp
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
#include "osimCommonDLL.h"
#include "Object.h"
#include "FunctionSet.h"
#include "GCVSpline.h"
#include "Storage.h"


//=============================================================================
//=============================================================================
/**
 * A class for holding a set of generalized cross-validated splines.
 *
 * @see GCVSpline
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API GCVSplineSet : public FunctionSet
{
//=============================================================================
// DATA
//=============================================================================
protected:

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	GCVSplineSet();
	GCVSplineSet(const char *aFileName);
	GCVSplineSet(int aDegree,const Storage *aStore,double aErrorVariance=0.0);
	virtual ~GCVSplineSet();
private:
	void setNull();
	void construct(int aDegree,const Storage *aStore,double aErrorVariance);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	GCVSpline* getGCVSpline(int aIndex);

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	Storage* constructStorage(int aDerivOrder,double aDX=-1);

//=============================================================================
};	// END class GCVSplineSet

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __GCVSplineSet_h__
