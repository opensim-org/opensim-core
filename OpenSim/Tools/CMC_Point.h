#ifndef CMCPoint_h__
#define CMCPoint_h__
// CMC_Point.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contributors: Frank C. Anderson
//
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include "CMC_Task.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for specifying and computing parameters for tracking a point.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API CMC_Point : public CMC_Task
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Location of the tracked point on the body expressed in the body-local
	coordinate frame. */
	PropertyDblVec3 _propPoint;

	// REFERENCES
	SimTK::Vec3 &_point;

	// Work Variables
	SimTK::Vec3 _p,_v,_inertialPTrk,_inertialVTrk;
	Body *_wrtBody,*_expressBody;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	CMC_Point(const SimTK::Vec3 &aPoint = SimTK::Vec3(0));
	CMC_Point(const CMC_Point &aTask);
	virtual ~CMC_Point();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	void copyData(const CMC_Point &aTask);
	void updateWorkVariables(const SimTK::State& s);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	CMC_Point& operator=(const CMC_Point &aTask);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setModel(Model& aModel);
	void setPoint(const SimTK::Vec3 &aPoint);
	SimTK::Vec3 getPoint() const;

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
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

//=============================================================================
};	// END of class CMC_Point
//=============================================================================
//=============================================================================

}; // end namespace

#endif // CMCPoint_h__


