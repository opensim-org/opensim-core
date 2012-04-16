// FunctionSet.cpp
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
#include "PropertyDbl.h"
#include "PropertyObjArray.h"
#include "FunctionSet.h"
#include "GCVSplineSet.h"
#include "SimTKcommon.h"




using namespace OpenSim;
using namespace std;

template class OSIMCOMMON_API OpenSim::Set<OpenSim::Function>;

//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
FunctionSet::~FunctionSet()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aName Name of the function set.
 */
FunctionSet::FunctionSet() :
	Set<Function>()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Construct a function set from file.
 *
 * @param aFileName Name of the file.
 */
FunctionSet::FunctionSet(const string &aFileName) :
	Set<Function>(aFileName, false)
{
	setNull();
	updateFromXMLDocument();
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void FunctionSet::
setNull()
{
}

//=============================================================================
// EVALUATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Evaluate a function or one of its derivatives.
 *
 * @param aIndex Index of the function to evaluate.
 * @param aDerivOrder Order of the derivative to evaluate.
 * @param aX Value of the x independent variable.
 * @param aY Value of the y independent variable.
 * @param aZ Value of the z independent variable.
 * @return Value of the function.  If the function is NULL or undefined,
 * SimTK::NaN is returned.
 * @see Function
 */
double FunctionSet::
evaluate(int aIndex,int aDerivOrder,double aX) const
{
	Function& func = get(aIndex);

	SimTK::Vector arg = SimTK::Vector(1, aX);
	if (aDerivOrder==0)
		return (func.calcValue(arg));

	std::vector<int> derivComponents;
	for(int i=0; i<aDerivOrder; i++)
		derivComponents.push_back(0);

	return( func.calcDerivative(derivComponents, arg) );
}

//_____________________________________________________________________________
/**
 * Evaluate all the functions in the function set or their derivatives.
 *
 * @param rValues Array containing the values of the functions.
 * @param aDerivOrder Order of the derivative to evaluate.
 * @param aX Value of the x independent variable.
 * @param aY Value of the y independent variable.
 * @param aZ Value of the z independent variable.
 * @return Value of the function.  If the function is NULL or undefined,
 * SimTK::NaN is returned.
 * @see Function
 */
void FunctionSet::
evaluate(Array<double> &rValues,int aDerivOrder,double aX) const
{
	int size = getSize();
	rValues.setSize(size);

	int i;
	for(i=0;i<size;i++) {
		Function& func = get(i);
		if (aDerivOrder==0)
			rValues[i] = func.calcValue(SimTK::Vector(1,aX));
		else {
			std::vector<int> derivComponents;
			for(int j=0; j<aDerivOrder; j++)
				derivComponents.push_back(0);
			rValues[i] = func.calcDerivative(derivComponents, SimTK::Vector(1,aX));

		}
	}
}
