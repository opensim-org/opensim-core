#ifndef __PolyObject_h__
#define __PolyObject_h__

// PolyObject.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/VisibleObject.h>
#ifdef BUILD_GUI
	#include <vtkXMLPolyDataReader.h>
	#include <vtkPolyData.h>
#endif

namespace OpenSim {

class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a visible object comprised of one or more polygonal
 * surfaces whose vertices and polygons are specified in a VTK file.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API PolyObject : public VisibleObject  
{

//=============================================================================
// DATA
//=============================================================================
private:
#ifdef BUILD_GUI
	Array<vtkPolyData*> _vtkBones;
	Array<vtkXMLPolyDataReader*> _vtkReaders;
#endif
protected:
	/** Name of geometry file name(s) */
	PropertyStrArray	_propGeometryFileNames;
	Array<std::string>&	_geometryFileNames;	

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PolyObject();
	PolyObject(const std::string &aFileName);
	PolyObject(const PolyObject &aPoly);
	virtual ~PolyObject();
	virtual Object* copy() const;

	PolyObject& operator=(const PolyObject &aPoly);
   void copyData(const PolyObject &aPoly);

	void setNumGeometryFiles(int n);
	const int getNumGeometryFiles() const;
	void setGeometryFileName(int i, const std::string &aGeometryFileName);
	const char* getGeometryFileName(int i) const;

   virtual void setup(AbstractDynamicsEngine* aEngine);
	virtual void scale(Array<double>& aScaleFactors);

	void peteTest() const;

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class PolyObject
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __PolyObject_h__


