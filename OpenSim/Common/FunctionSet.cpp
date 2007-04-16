// FunctionSet.cpp
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
#include "rdMath.h"
#include "PropertyDbl.h"
#include "PropertyObjArray.h"
#include "FunctionSet.h"
#include "GCVSplineSet.h"




using namespace OpenSim;
using namespace std;


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
	Set<Function>(),
	_minX(_propMinX.getValueDbl()),
	_maxX(_propMaxX.getValueDbl()),
	_minY(_propMinX.getValueDbl()),
	_maxY(_propMaxX.getValueDbl()),
	_minZ(_propMinX.getValueDbl()),
	_maxZ(_propMaxX.getValueDbl())
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
	Set<Function>(aFileName, false),
	_minX(_propMinX.getValueDbl()),
	_maxX(_propMaxX.getValueDbl()),
	_minY(_propMinX.getValueDbl()),
	_maxY(_propMaxX.getValueDbl()),
	_minZ(_propMinX.getValueDbl()),
	_maxZ(_propMaxX.getValueDbl())
{
	setNull();
	updateFromXMLNode();
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
	setType("FunctionSet");
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Setup serialized member variables.
 */
void FunctionSet::
setupProperties()
{
	// X
	_propMinX.setName("min_x");
	_propMinX.setValue(0.0);
	_propertySet.append( &_propMinX );

	_propMaxX.setName("max_x");
	_propMaxX.setValue(0.0);
	_propertySet.append( &_propMaxX );

	// Y
	_propMinY.setName("min_y");
	_propMinY.setValue(0.0);
	_propertySet.append( &_propMinY );

	_propMaxY.setName("max_y");
	_propMaxY.setValue(0.0);
	_propertySet.append( &_propMaxY );

	// Z
	_propMinZ.setName("min_z");
	_propMinZ.setValue(0.0);
	_propertySet.append( &_propMinZ );

	_propMaxZ.setName("max_z");
	_propMaxZ.setValue(0.0);
	_propertySet.append( &_propMaxZ );


	// ARRAY OF FUNCTIONS
	//_propertySet.append(	new PropertyObjArray("functions") );
	//_functions = (ArrayPtrs<Function>*)&
	//	_propertySet.get("functions")->getValueObjArray();
}


//=============================================================================
// SET AND GET
//=============================================================================
//-----------------------------------------------------------------------------
// MIN AND MAX X
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum x independent variable.
 *
 * @param aMinX Minimum x.
 */
void FunctionSet::
setMinX(double aMinX)
{
	_minX = aMinX;
}
//_____________________________________________________________________________
/**
 * Get the minimum x independent variable.
 *
 * @return Minimum x.
 */
double FunctionSet::
getMinX()
{
	return(_minX);
}

//_____________________________________________________________________________
/**
 * Set the maximum x independent variable.
 *
 * @param aMaxX Maximum x.
 */
void FunctionSet::
setMaxX(double aMaxX)
{
	_maxX = aMaxX;
}
//_____________________________________________________________________________
/**
 * Get the maximum x independent variable.
 *
 * @return Maximum x.
 */
double FunctionSet::
getMaxX()
{
	return(_maxX);
}

//-----------------------------------------------------------------------------
// MIN AND MAX Y
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum y independent variable.
 *
 * @param aMinY Minimum y.
 */
void FunctionSet::
setMinY(double aMinY)
{
	_minY = aMinY;
}
//_____________________________________________________________________________
/**
 * Get the minimum y independent variable.
 *
 * @return Minimum y.
 */
double FunctionSet::
getMinY()
{
	return(_minY);
}

//_____________________________________________________________________________
/**
 * Set the maximum y independent variable.
 *
 * @param aMaxY Maximum y.
 */
void FunctionSet::
setMaxY(double aMaxY)
{
	_maxY = aMaxY;
}
//_____________________________________________________________________________
/**
 * Get the maximum y independent variable.
 *
 * @return Maximum y.
 */
double FunctionSet::
getMaxY()
{
	return(_maxY);
}

//-----------------------------------------------------------------------------
// MIN AND MAX Z
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum z independent variable.
 *
 * @param aMinZ Minimum z.
 */
void FunctionSet::
setMinZ(double aMinZ)
{
	_minZ = aMinZ;
}
//_____________________________________________________________________________
/**
 * Get the minimum z independent variable.
 *
 * @return Minimum z.
 */
double FunctionSet::
getMinZ()
{
	return(_minZ);
}

//_____________________________________________________________________________
/**
 * Set the maximum z independent variable.
 *
 * @param aMaxZ Maximum z.
 */
void FunctionSet::
setMaxZ(double aMaxZ)
{
	_maxZ = aMaxZ;
}
//_____________________________________________________________________________
/**
 * Get the maximum z independent variable.
 *
 * @return Maximum z.
 */
double FunctionSet::
getMaxZ()
{
	return(_maxZ);
}


//=============================================================================
// EVALUATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Update the bounding boxes of all functions in the set.
 *
 * @see GCVSplineSet::UpdateBoundingBox()
 */
void FunctionSet::
updateBoundingBox()
{
	int i,n = _objects.getSize();
	for(i=0;i<n;i++) updateBoundingBox(i);
}
//_____________________________________________________________________________
/**
 * Update the bounding box of a specified function.
 *
 * @param aIndex Index of the function for which to update the bounding box.
 * @see GCVSplineSet::UpdateBoundingBox()
 */
void FunctionSet::
updateBoundingBox(int aIndex)
{
	Function *func = get(aIndex);
	if(func==NULL) return;
	func->updateBoundingBox();
}
//_____________________________________________________________________________
/**
 * Update the bounding box for this function set.
 * The bounding box is bounding box for all functions.  The bounding box
 * is determined by looping through all the functions in this set and
 * querying each for its bounding box. Note that none of the bounding boxes
 * of the functions in this set are changed by this method.
 *
 * @see Function
 */
void FunctionSet::
updateSetBoundingBox()
{
	int i;
	bool initialized=false;
	Function *func;
	for(i=0;i<getSize();i++) {

		// GET FUNCTION
		func = get(i);
		if(func==NULL) continue;

		// INITIALIZE
		if(!initialized) {
			setMinX(func->getMinX());
			setMaxX(func->getMaxX());
			setMinY(func->getMinY());
			setMaxY(func->getMaxY());
			setMinZ(func->getMinZ());
			setMaxZ(func->getMaxZ());
			initialized = true;

		// CHECK BOUNDS
		} else {

			if(func->getMinX()<getMinX()) setMinX(func->getMinX());
			if(func->getMinY()<getMinY()) setMinY(func->getMinY());
			if(func->getMinZ()<getMinZ()) setMinZ(func->getMinZ());

			if(func->getMaxX()>getMaxX()) setMaxX(func->getMaxX());
			if(func->getMaxY()>getMaxY()) setMaxY(func->getMaxY());
			if(func->getMaxZ()>getMaxZ()) setMaxZ(func->getMaxZ());

		}
	}
}

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
 * rdMath::NAN is returned.
 * @see Function
 */
double FunctionSet::
evaluate(int aIndex,int aDerivOrder,double aX,double aY,double aZ) const
{
	Function *func = get(aIndex);
	if(func==NULL) return(rdMath::NAN);

	return( func->evaluate(aDerivOrder,aX,aY,aZ) );
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
 * rdMath::NAN is returned.
 * @see Function
 */
void FunctionSet::
evaluate(Array<double> &rValues,int aDerivOrder,double aX,double aY,double aZ) const
{
	int size = getSize();
	rValues.setSize(size);

	int i;
	Function *func;
	for(i=0;i<size;i++) {
		func = get(i);
		if(func==NULL) {
			rValues[i] = rdMath::NAN;
		} else {
			rValues[i] = func->evaluate(aDerivOrder,aX,aY,aZ);
		}
	}
}
