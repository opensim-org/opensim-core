// SimmMuscleGroup.cpp
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "SimmMuscleGroup.h"
#include "SimmMuscle.h"
#include "SimmModel.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmMuscleGroup::SimmMuscleGroup() :
	_muscles(NULL)
{
	setNull();

}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmMuscleGroup::SimmMuscleGroup(DOMElement *aElement) :
   Object(aElement),
	_muscles(NULL)
{
	setNull();
	//updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMuscleGroup::~SimmMuscleGroup()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aGroup Group to be copied.
 */
SimmMuscleGroup::SimmMuscleGroup(const SimmMuscleGroup &aGroup) :
   Object(aGroup),
	_muscles(NULL)
{
	copyData(aGroup);
}
//_____________________________________________________________________________
/**
 * Copy this muscle group and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this group.
 */
Object* SimmMuscleGroup::copy() const
{
	SimmMuscleGroup *grp = new SimmMuscleGroup(*this);
	return(grp);
}
//_____________________________________________________________________________
/**
 * Copy this SimmMuscleGroup and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmMuscleGroup::SimmMuscleGroup(DOMElement*) in order to establish the
 * relationship of the SimmMuscleGroup object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmMuscleGroup object. Finally, the data members of the
 * copy are updated using SimmMuscleGroup::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmMuscleGroup.
 */
Object* SimmMuscleGroup::copy(DOMElement *aElement) const
{
	SimmMuscleGroup *grp = new SimmMuscleGroup(aElement);
	*grp = *this;
	//grp->updateFromXMLNode();
	return(grp);
}

void SimmMuscleGroup::copyData(const SimmMuscleGroup &aGroup)
{
	_muscles = aGroup._muscles;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmMuscleGroup to their null values.
 */
void SimmMuscleGroup::setNull()
{
	setType("SimmMuscleGroup");
}

SimmMuscleGroup& SimmMuscleGroup::operator=(const SimmMuscleGroup &aGroup)
{
	// BASE CLASS
	Object::operator=(aGroup);

	copyData(aGroup);

	return(*this);
}

/* Perform some set up functions that happen after the
 * object has been deserialized or copied.
 */
void SimmMuscleGroup::setup(SimmModel* aModel)
{
	_muscles.setSize(0);

	SimmMuscle* sm;
	for (int i = 0; i < aModel->getNumberOfMuscles(); i++)
	{
		if (sm = dynamic_cast<SimmMuscle*>(aModel->getMuscle(i)))
		{
			const Array<string>& groupNames = sm->getGroupNames();
			for (int j = 0; j < groupNames.getSize(); j++)
			{
				if (groupNames[j] == getName())
					_muscles.append(sm);
			}
		}
	}
}

bool SimmMuscleGroup::contains(const std::string& aName) const
{
	for (int i = 0; i < _muscles.getSize(); i++)
		if (_muscles[i]->getName() == aName)
			return true;

	return false;
}

void SimmMuscleGroup::peteTest() const
{
	cout << "Muscle Group: " << getName() << endl;
	cout << "   muscles: ";
	for (int i = 0; i < _muscles.getSize(); i++)
		cout << _muscles[i]->getName() << " ";
	cout << endl;
}

