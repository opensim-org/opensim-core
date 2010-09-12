#ifndef __TransformAxis_h__
#define __TransformAxis_h__

// TransformAxis.h
// Author: Peter Loan, Frank C. Anderson, Jeffrey A. Reinbolt, Ajay Seth
/*
 * Copyright (c)  2006-2007, Stanford University. All rights reserved. 
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
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/Function.h>


namespace OpenSim {

class Joint;
class CustomJoint;
class Coordinate;
class SimbodyEngine;

//=============================================================================
//=============================================================================
/**
 * A class expressing a tranformation of a child body in relation to a parent
 * body along either a translation or rotation axis.
 *
 * @author Peter Loan, Frank C. Anderson, Jeffrey A. Reinbolt, Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API TransformAxis : public Object  
{

//=============================================================================
// DATA
//=============================================================================
protected:

	/** Rotation or translation axis for the transform. */
	PropertyDblVec3 _axisProp;
	SimTK::Vec3 &_axis;

	/** Transform function of the generalized coordinate used to represent
	the amount of transformation along a specified axis. */
	PropertyObjPtr<Function> _functionProp;
	Function *&_function;

	/** Names of the coordinate that serves as the indepenent variables of the
	tranform function. */
	PropertyStrArray  _coordinateNamesProp;
	Array<std::string>& _coordinateNames;

	// Pointer to the joint to which the coordinates belong.
	Joint *_joint;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	TransformAxis();
	TransformAxis(const Array<std::string> &coordNames, const SimTK::Vec3& aAxis);
	TransformAxis(DOMElement *aNode);
	//TransformAxis(const std::string &aName, const Array<std::string> &coordNames, const SimTK::Vec3& aAxis, bool isRotation=false);
	TransformAxis(const TransformAxis &anAxis);
	virtual ~TransformAxis();
	virtual Object* copy() const;

#ifndef SWIG
	TransformAxis& operator=(const TransformAxis &anAxis);
#endif
	void copyData(const TransformAxis &anAxis);

	// GET
	virtual void setCoordinateNames(const Array<std::string> &coordNames);
	virtual const Array<std::string> getCoordinateNames() const { return _coordinateNames; }
    /**
     * Determine whether a custom function has been specified to map between the generalized coordinate
     * and the amount of transformation along the specified axis.
     */
    virtual bool hasFunction() const;
    /**
     * Get the custom function that maps between the generalized coordinate and the amount of
     * transformation along the specified axis.  If no function has been specified, this throws an
     * exception.
     */
    virtual Function& getFunction() const;
    /**
     * Set the custom function that maps between the generalized coordinate and the amount of
     * transformation along the specified axis.  This object adopts ownership of the Function
     * object, and deletes it when this object is itself deleted.
     */
    virtual void setFunction(Function* aFunction);
    /**
     * Set the custom function that maps between the generalized coordinate and the amount of
     * transformation along the specified axis.  This method creates a copy of the original Function
     * object, which is unaffected.
     */
	virtual void setFunction(const Function& aFunction);
    Joint& getJoint() { return *_joint; }

	virtual double getValue(const SimTK::State& s );
	virtual void setAxis(const SimTK::Vec3& aAxis);
	virtual void getAxis(SimTK::Vec3& rAxis) const;
	virtual void getAxis(double rAxis[]) const { SimTK::Vec3 temp; temp.updAs(rAxis)=_axis; }
	const SimTK::Vec3& getAxis() const { return _axis; }	
	const double& getAxis(int aXYZ) const { assert(aXYZ>=0 && aXYZ<=2); return _axis[aXYZ]; }

	// Setup the Transform Axis for to be used by its owning Joint
    virtual void setup(Joint& aJoint);

	virtual void updateFromXMLNode();
	OPENSIM_DECLARE_DERIVED(TransformAxis, Object);

protected:

private:
	void setNull();
	void setupProperties();
	friend class SimbodyEngine;
//=============================================================================
};	// END of class TransformAxis
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __TransformAxis_h__


