// PathWrapPoint.cpp
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "PathWrapPoint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PathWrapPoint::PathWrapPoint()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
PathWrapPoint::~PathWrapPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint PathWrapPoint to be copied.
 */
PathWrapPoint::PathWrapPoint(const PathWrapPoint &aPoint) :
   PathPoint(aPoint)
{
	setNull();
	copyData(aPoint);
}

//_____________________________________________________________________________
/**
 * Copy this muscle via point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this PathWrapPoint.
 */
Object* PathWrapPoint::copy() const
{
	PathWrapPoint *pt = new PathWrapPoint(*this);
	return(pt);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one PathWrapPoint to another.
 *
 * @param aPoint PathWrapPoint to be copied.
 */
void PathWrapPoint::copyData(const PathWrapPoint &aPoint)
{
	_wrapPath = aPoint._wrapPath;
	_wrapPathLength = aPoint._wrapPathLength;
	_wrapObject = aPoint._wrapObject;
}

//_____________________________________________________________________________
/**
 * Set the data members of this PathWrapPoint to their null values.
 */
void PathWrapPoint::setNull()
{
	setType("PathWrapPoint");

	_wrapPath.setSize(0);
	_wrapPathLength = 0.0;
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this PathWrapPoint.
 */
void PathWrapPoint::setup(const Model& aModel, GeometryPath& aPath)
{
	// base class
	PathPoint::setup(aModel, aPath);
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
PathWrapPoint& PathWrapPoint::operator=(const PathWrapPoint &aPoint)
{
	// BASE CLASS
	PathPoint::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}
