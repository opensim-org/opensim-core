// rdCMC_Joint.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS,
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
// THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef rdCMCJoint_h__
#define rdCMCJoint_h__

//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/AbstractModel.h>
#include "rdCMC_Task.h"

namespace OpenSim {

class AbstractCoordinate;

//=============================================================================
//=============================================================================
/**
 * A class for specifying the tracking task for a joint.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API rdCMC_Joint : public rdCMC_Task
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Name of the generalized coordinate to be tracked. */
	PropertyStr _propCoordinateName;
	/** Error limit on the coordinate. */
	PropertyDbl _propLimit;

	// REFERENCES
	std::string &_coordinateName;
	double &_limit;

	// Work Variables
	AbstractCoordinate *_q;
	AbstractSpeed *_u;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	rdCMC_Joint(const std::string &aCoordinateName = "");
	rdCMC_Joint(const rdCMC_Joint &aTask);
	virtual ~rdCMC_Joint();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	void copyData(const rdCMC_Joint &aTask);
	void updateWorkVariables();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	rdCMC_Joint& operator=(const rdCMC_Joint &aTask);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setModel(AbstractModel *aModel);
	void setCoordinateName(const std::string &aName);
	std::string getCoordinateName() const;
	double getLimit() const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeErrors(double aT);
	virtual void computeDesiredAccelerations(double aT);
	virtual void computeDesiredAccelerations(double aTI,double aTF);
	virtual void computeAccelerations();

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();


//=============================================================================
};	// END of class rdCMC_Joint
//=============================================================================
//=============================================================================

}; // end namespace

#endif // rdCMCJoint_h__


