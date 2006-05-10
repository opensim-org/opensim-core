#ifndef _SimmCoordinate_h_
#define _SimmCoordinate_h_

// SimmCoordinate.h
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyStrArray.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/Function.h>
#include <OpenSim/Simulation/SIMM/Coordinate.h>
#include "SimmPath.h"

namespace OpenSim { 

class SimmJoint;
class SimmKinematicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM generalized coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */


class RDSIMULATION_API SimmCoordinate : public OpenSim::Coordinate  
{
//=============================================================================
// DATA
//=============================================================================
public:
	typedef struct
	{
		int restraintFuncNum;
		int minRestraintFuncNum;
		int maxRestraintFuncNum;
	} sdfastCoordinateInfo;

	sdfastCoordinateInfo _sdfastInfo;

protected:
	PropertyDbl _defaultValueProp;
	double &_defaultValue;

	/* value is specified as a string, so the user can specify
	 * options like "fromFile" which means to read the value
	 * from a coordinate file.
	 */
	PropertyStr _valueStrProp;
	std::string &_valueStr;
	double _value;

	PropertyDbl _toleranceProp;
	double &_tolerance;

	PropertyDbl _PDStiffnessProp;
	double &_PDStiffness;

	PropertyDbl _IKweightProp;
	double &_IKweight;

	PropertyDblArray _rangeProp;
	Array<double>& _range;

	PropertyStrArray _keysProp;
	Array<std::string>& _keys;

	PropertyBool _clampedProp;
	bool &_clamped;

	PropertyBool _lockedProp;
	bool &_locked;

	PropertyObjArray _restraintFunctionProp;
	ArrayPtrs<Function> &_restraintFunction;

	PropertyObjArray _minRestraintFunctionProp;
	ArrayPtrs<Function> &_minRestraintFunction;

	PropertyObjArray _maxRestraintFunctionProp;
	ArrayPtrs<Function> &_maxRestraintFunction;

	PropertyBool _restraintActiveProp;
	bool &_restraintActive;

	Array<SimmJoint*> _jointList; // list of joints that use this coordinate
	Array<SimmPath*> _pathList; // list of paths that use this coordinate

	int _RTtype; // rotational or translational (based on the DOFs it's used in)

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmCoordinate();
	SimmCoordinate(DOMElement *aElement);
	SimmCoordinate(const SimmCoordinate &aCoordinate);
	virtual ~SimmCoordinate();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmCoordinate& operator=(const SimmCoordinate &aCoordinate);
#endif
   void SimmCoordinate::copyData(const SimmCoordinate &aCoordinate);

   void addJointToList(SimmJoint* aJoint) { _jointList.append(aJoint); }
   void addPathToList(SimmPath* aPath) { _pathList.append(aPath); }
   void setup(SimmKinematicsEngine* aEngine);
	bool setValue(double value);
	bool setValue(std::string& aValueStr);
	double getValue() const { return _value; }
	const std::string& getValueStr() const { return _valueStr; }
	double getTolerance() const { return _tolerance; }
	double getDefaultValue() const { return _defaultValue; }
	double getPDStiffness() const { return _PDStiffness; }
	double getIKweight() const { return _IKweight; }
	void getRange(double range[2]) const { range[0] = _range[0]; range[1] = _range[1]; }
	virtual double getRangeMin() const { return _range[0]; }
	virtual double getRangeMax() const { return _range[1]; }
	void getKeys(std::string keys[]) const;
	const Array<SimmJoint*>& getJointList() const { return _jointList; }
	const Array<SimmPath*>& getPathList() const { return _pathList; }
	bool isUsedInModel(void) { if (getJointList().getSize() > 0) return true; else return false; }
	bool isClamped(void) const { return _clamped; }
	bool isLocked(void) const { return _locked; }
	void setLocked(bool aState) { _locked = aState; }
	bool isRestraintActive(void) const { return _restraintActive; }
	Function* getRestraintFunction(void) const;
	Function* getMinRestraintFunction(void) const;
	Function* getMaxRestraintFunction(void) const;
	void updateFromCoordinate(const SimmCoordinate &aCoordinate);

	void writeSIMM(std::ofstream& out, int& aFunctionIndex) const;

	void peteTest(void) const;

protected:

private:
	void setNull(void);
	void setupProperties(void);
	void determineType(SimmKinematicsEngine* aEngine);

//=============================================================================
};	// END of class SimmCoordinate

//=============================================================================
//=============================================================================

typedef RDSIMULATION_API OpenSim::ArrayPtrs<OpenSim::SimmCoordinate> SimmCoordinateArray;
}; //namespace

#endif // __SimmCoordinate_h__


