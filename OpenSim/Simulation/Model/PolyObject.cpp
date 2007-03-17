// PolyObject.cpp
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "PolyObject.h"
#include "AbstractDynamicsEngine.h"
#ifdef BUILD_GUI
#include <vtkPointData.h>
#include <vtkDataArray.h>
#endif

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
PolyObject::PolyObject() :
	_geometryFileNames(_propGeometryFileNames.getValueStrArray())
#ifdef BUILD_GUI
	,
   _vtkBones(0),
	_vtkReaders(0)
#endif
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
PolyObject::PolyObject(const string &aFileName):
	_geometryFileNames(_propGeometryFileNames.getValueStrArray())
#ifdef BUILD_GUI
	,
   _vtkBones(0),
	_vtkReaders(0)
#endif
{
	// NULL STATES
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
PolyObject::~PolyObject()
{
#ifdef BUILD_GUI
	for (int i = 0; i < _vtkReaders.getSize(); i++)
		_vtkReaders[i]->Delete();
#endif
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoly PolyObject to be copied.
 */
PolyObject::PolyObject(const PolyObject &aPoly) :
   VisibleObject(aPoly),
	_geometryFileNames(_propGeometryFileNames.getValueStrArray())
#ifdef BUILD_GUI
	,
	_vtkBones(0),
	_vtkReaders(0)
#endif
{
	setNull();
	setupProperties();
	copyData(aPoly);
}

//_____________________________________________________________________________
/**
 * Copy this PolyObject and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this PolyObject.
 */
Object* PolyObject::copy() const
{
	PolyObject *poly = new PolyObject(*this);
	return(poly);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one PolyObject to another.
 *
 * @param aPoly PolyObject to be copied.
 */
void PolyObject::copyData(const PolyObject &aPoly)
{
	setNumGeometryFiles(aPoly.getNumGeometryFiles());
	for (int i = 0; i < aPoly.getNumGeometryFiles(); i++)
		setGeometryFileName(i, aPoly.getGeometryFileName(i));

	// TODO: should probably copy the vtkPolyData/vtkXMLPolyDataReader
	// objects themselves, not just the pointers, but can't figure out
	// how to do that.
#ifdef BUILD_GUI
	_vtkBones = aPoly._vtkBones;
	_vtkReaders = aPoly._vtkReaders;
#endif
}

//_____________________________________________________________________________
/**
 * Set the data members of this PolyObject to their null values.
 */
void PolyObject::setNull()
{
	setType("PolyObject");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PolyObject::setupProperties()
{
	_propertySet.append(&_propGeometryFileNames);
	_propGeometryFileNames.setName("GeometryFiles");
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this PolyObject.
 */
void PolyObject::setup(AbstractDynamicsEngine* aEngine)
{
#ifdef BUILD_GUI
	int index;

	/* Each VTK file needs its own reader, which owns the
	 * poly data in the VTK file. The PolyObject destructor
	 * calls vtkXMLPolyDataReader::Delete() to free this
	 * memory.
	 */
	for (int i = 0; i < getNumGeometryFiles(); i++)
	{
		index = _vtkReaders.append(vtkXMLPolyDataReader::New()) - 1;
		_vtkReaders[index]->SetFileName(getGeometryFileName(i));
		_vtkReaders[index]->Update();
		vtkPolyData* data = _vtkReaders[index]->GetOutput();
		/* vtkXMLPolyDataReader::GetOutput() returns a non-NULL
		 * pointer in all cases, even if the file is not found.
		 * So check for # polygons and # vertices before adding
		 * the vtkPolyData object to the array. If the data is
		 * bad, add a NULL pointer so the _vtkBones array still
		 * lines up with the _vtkReaders array.
		 */
		if (data && (data->GetNumberOfPolys() > 0 || data->GetNumberOfPoints() > 0))
			_vtkBones.append(_vtkReaders[index]->GetOutput());
		else
			_vtkBones.append(NULL);
	}
#endif
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
PolyObject& PolyObject::operator=(const PolyObject &aPoly)
{
	// BASE CLASS
	VisibleObject::operator=(aPoly);

	copyData(aPoly);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the number of geometry files associated with the PolyObject
 */
void PolyObject::setNumGeometryFiles(int n)
{
	_geometryFileNames.setSize(n);
}

//_____________________________________________________________________________
/**
 * Get the number of geometry files associated with the PolyObject
 */
const int PolyObject::getNumGeometryFiles() const
{
	return (_geometryFileNames.getSize());
}

//_____________________________________________________________________________
/**
 * Set the name of the ith geometry file associated with the PolyObject
 */
void PolyObject::setGeometryFileName(int i, const string &aGeometryFileName)
{
	_geometryFileNames.set(i,aGeometryFileName);
}

//_____________________________________________________________________________
/**
 * Get the name of the ith geometry file associated with the PolyObject
 */
const char *PolyObject::getGeometryFileName(int i) const
{
	return _geometryFileNames[i].c_str();
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the vertices in a PolyObject (vtkPolyData object). You'd think
 * there'd be a more efficient way to get and set the points
 * (e.g., a GetPoints() method that returned an array of doubles,
 * and a SetPoints() method to put them back), but I could not
 * find one. I also don't know if the bounding box, normals,
 * and other derivative information is automatically updated.
 *
 * @param aScaleFactors XYZ scale factors.
 */
void PolyObject::scale(Array<double>& aScaleFactors)
{
#ifdef BUILD_GUI
	for (int i = 0; i < _vtkBones.getSize(); i++)
	{
		if (_vtkBones[i])
		{
			vtkPoints* pts = _vtkBones[i]->GetPoints();
			for (int j = 0; j < pts->GetNumberOfPoints(); j++)
			{
				double* pt = pts->GetPoint(j);
				for (int k = 0; k < 3; k++)
					pt[k] *= aScaleFactors[k];
				pts->SetPoint(j, pt);
			}
		}
	}
#endif
}

void PolyObject::peteTest() const
{
	cout << "   PolyObject: " << getName() << endl;
#ifdef BUILD_GUI
	for (int i = 0; i < _vtkBones.getSize(); i++)
	{
		if (_vtkBones[i] == NULL)
			cout << "      file " << getGeometryFileName(i) << " is empty or missing." << endl;
		else
			cout << "      file " << getGeometryFileName(i) << " has " << _vtkBones[i]->GetNumberOfPolys() <<
			     " polygons and " << _vtkBones[i]->GetNumberOfPoints() << " vertices." << endl;
	}
#endif
}

