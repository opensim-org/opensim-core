#ifndef _SimmBone_h_
#define _SimmBone_h_

// SimmBone.h
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
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyStrArray.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/Object.h>

namespace OpenSim { 

class SimmKinematicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM bone. Because the user can interact with the
 * geometry of a bone while manipulating the model, it needs to be accesible
 * within the simm/rd framework (not just on the Java side). SimmBone is
 * a type of Object, with additional private data members for holding
 * the bone geometry.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmBone : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:
protected:
	PropertyStrArray _geometryFilesProp;
	Array<std::string>& _geometryFiles;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmBone();
	SimmBone(DOMElement *aElement);
	SimmBone(const SimmBone &aBone);
	virtual ~SimmBone();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
	std::string& getGeometryFileName(int i) const
	{
		return _geometryFiles.get(i);
	}
	int getNumGeometryFiles() const
	{
		return _geometryFiles.getSize();
	}
	void addGeometryFile(std::string& geomFile)
	{
		_geometryFiles.append(geomFile);
	}
#ifndef SWIG
	SimmBone& operator=(const SimmBone &aBone);
#endif
   void SimmBone::copyData(const SimmBone &aBone);

   void setup(SimmKinematicsEngine* aEngine);
	void scale(Array<double>& aScaleFactors);

	void writeSIMM(std::ofstream& out) const;

	void peteTest() const;

protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmBone

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimmBone_h__


