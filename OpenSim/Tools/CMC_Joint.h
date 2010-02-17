#ifndef CMCJoint_h__
#define CMCJoint_h__
// CMC_Joint.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contributors: Frank C. Anderson
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
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "CMC_Task.h"

namespace OpenSim {

class Coordinate;

//=============================================================================
//=============================================================================
/**
 * A class for specifying the tracking task for a joint.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API CMC_Joint : public CMC_Task
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
	Coordinate *_q;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	CMC_Joint(const std::string &aCoordinateName = "");
	CMC_Joint(const CMC_Joint &aTask);
	virtual ~CMC_Joint();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	void copyData(const CMC_Joint &aTask);
	void updateWorkVariables();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	CMC_Joint& operator=(const CMC_Joint &aTask);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setModel(Model& aModel);
	void setCoordinateName(const std::string &aName);
	std::string getCoordinateName() const;
	double getLimit() const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeErrors(const SimTK::State& s, double aT);
	virtual void computeDesiredAccelerations(const SimTK::State& s, double aT);
	virtual void computeDesiredAccelerations(const SimTK::State& s, double aTI,double aTF);
	virtual void computeAccelerations(const SimTK::State& s );

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();


//=============================================================================
};	// END of class CMC_Joint
//=============================================================================
//=============================================================================

}; // end namespace

#endif // CMCJoint_h__


