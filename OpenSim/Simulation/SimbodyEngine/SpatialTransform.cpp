// SpatialTransform.cpp
// Author: Ajay Seth
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

#include "SpatialTransform.h"
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Common/LinearFunction.h>

#define ASSERT(cond) {if (!(cond)) throw(exception());}

using namespace std;
using namespace OpenSim;
using namespace SimTK;


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SpatialTransform::~SpatialTransform(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a SpatialTransform.
 */
SpatialTransform::SpatialTransform() :
	Object(),
	_rotation1Prop(PropertyObj("rotation1", TransformAxis(Array<string>(), Vec3(1,0,0)))),
	_rotation1((TransformAxis &)_rotation1Prop.getValueObj()),
	_rotation2Prop(PropertyObj("rotation2", TransformAxis(Array<string>(), Vec3(0,1,0)))),
	_rotation2((TransformAxis &)_rotation2Prop.getValueObj()),
	_rotation3Prop(PropertyObj("rotation3", TransformAxis(Array<string>(), Vec3(0,0,1)))),
    _rotation3((TransformAxis &)_rotation3Prop.getValueObj()),
	_translation1Prop(PropertyObj("translation1", TransformAxis(Array<string>(), Vec3(1,0,0)))),
	_translation1((TransformAxis &)_translation1Prop.getValueObj()),
	_translation2Prop(PropertyObj("translation2", TransformAxis(Array<string>(), Vec3(0,1,0)))),
	_translation2((TransformAxis &)_translation2Prop.getValueObj()),
	_translation3Prop(PropertyObj("translation3", TransformAxis(Array<string>(), Vec3(0,0,1)))),
	_translation3((TransformAxis &)_translation3Prop.getValueObj())
{
	setNull();
	setupProperties();
	constructTransformAxes();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a SpatialTransform.
 */
SpatialTransform::SpatialTransform(const SpatialTransform& aSpatialTransform):
	Object(aSpatialTransform),
	_rotation1Prop(PropertyObj("rotation1", TransformAxis(Array<string>(), Vec3(1,0,0)))),
	_rotation1((TransformAxis &)_rotation1Prop.getValueObj()),
	_rotation2Prop(PropertyObj("rotation2", TransformAxis(Array<string>(), Vec3(0,1,0)))),
	_rotation2((TransformAxis &)_rotation2Prop.getValueObj()),
	_rotation3Prop(PropertyObj("rotation3", TransformAxis(Array<string>(), Vec3(0,0,1)))),
    _rotation3((TransformAxis &)_rotation3Prop.getValueObj()),
	_translation1Prop(PropertyObj("translation1", TransformAxis(Array<string>(), Vec3(1,0,0)))),
	_translation1((TransformAxis &)_translation1Prop.getValueObj()),
	_translation2Prop(PropertyObj("translation2", TransformAxis(Array<string>(), Vec3(0,1,0)))),
	_translation2((TransformAxis &)_translation2Prop.getValueObj()),
	_translation3Prop(PropertyObj("translation3", TransformAxis(Array<string>(), Vec3(0,0,1)))),
	_translation3((TransformAxis &)_translation3Prop.getValueObj())
{
	setNull();
	setupProperties();
	constructTransformAxes();
	copyData(aSpatialTransform);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this SpatialTransform to their null values.
 */
void SpatialTransform::setNull()
{
	setType("SpatialTransform");
	_owningJoint = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SpatialTransform::setupProperties()
{
	_rotation1Prop.setComment("3 Axes for rotations are listed first.");
	_rotation1Prop.setName("rotation1");
	_rotation1Prop.setMatchName(true);
	_propertySet.append(&_rotation1Prop);

	_rotation2Prop.setName("rotation2");
	_rotation2Prop.setMatchName(true);
	_propertySet.append(&_rotation2Prop);

	_rotation3Prop.setName("rotation3");
	_rotation3Prop.setMatchName(true);
	_propertySet.append(&_rotation3Prop);

	_translation1Prop.setComment("3 Axes for translations are listed next.");
	_translation1Prop.setName("translation1");
	_translation1Prop.setMatchName(true);
	_propertySet.append(&_translation1Prop);

	_translation2Prop.setName("translation2");
	_translation2Prop.setMatchName(true);
	_propertySet.append(&_translation2Prop);

	_translation3Prop.setName("translation3");
	_translation3Prop.setMatchName(true);
	_propertySet.append(&_translation3Prop);
}


/**
 * Construct the individual TransformAxes of this SpatialTransform 
 */
void SpatialTransform::constructTransformAxes()
{
	for(int i =0; i < _numTransformAxes; i++){
		if( &operator[](i) == NULL){
			throw Exception("SpatialTransform: NULL axis found");
		}
		//operator[](i).setFunction(new OpenSim::Constant(0));
	}
}


//_____________________________________________________________________________
/**
 * Copy this SpatialTransform and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this OpenSim::SpatialTransform.
 */
Object* SpatialTransform::copy() const
{
	SpatialTransform *spatialTransform = new SpatialTransform(*this);
	return(spatialTransform);
}

//_____________________________________________________________________________
/**
 * Copy the contents of the one SpatialTransform (i.e. the axes) to another.
 *
 * @param aSpatialTransform SpatialTransform to be copied.
 */
void SpatialTransform::copyData(const SpatialTransform &aSpatialTransform)
{
	//for(int i=0; i<_numTransformAxes; i++)
	//	operator[](i) = aSpatialTransform[i];
	_rotation1 = aSpatialTransform._rotation1;
	_rotation2 = aSpatialTransform._rotation2;
	_rotation3 = aSpatialTransform._rotation3;
	_translation1 = aSpatialTransform._translation1;
	_translation2 = aSpatialTransform._translation2;
	_translation3 = aSpatialTransform._translation3;

	/*_rotation1Prop = aSpatialTransform._rotation1Prop;
	_rotation2Prop = aSpatialTransform._rotation2Prop;
	_rotation3Prop = aSpatialTransform._rotation3Prop;
	_translation1Prop = aSpatialTransform._translation1Prop;
	_translation2Prop = aSpatialTransform._translation2Prop;
	_translation3Prop = aSpatialTransform._translation3Prop;
	*/
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
SpatialTransform& SpatialTransform::operator=(const SpatialTransform &aSpatialTransform)
{
	Object::operator=(aSpatialTransform);
	copyData(aSpatialTransform);

	return (*this);
}

TransformAxis& SpatialTransform::getTransformAxis(int aIndex) const
{
	return operator[](aIndex);
}

TransformAxis& SpatialTransform::operator[](int aIndex) const
{
	 switch(aIndex){
		case(0) : 	
			return _rotation1;
			break;
		case(1) :
			return _rotation2;
			break;
		case(2) :
			return _rotation3;
			break;
		case(3) :
			return _translation1;
			break;
		case(4) :
			return _translation2;
			break;
		case(5) :
			return _translation3;
			break;
		default :
			throw(Exception("SpatialTransform: Attempting to access beyond 6 axes."));
	 }
}


// SETUP
void SpatialTransform::setup(CustomJoint &aJoint)
{
	_owningJoint = &aJoint;
	
	// define default function for TransformAxes that have none specified
	for(int i=0; i<_numTransformAxes; i++){
		// Exception if a transform does not exist
		if(&operator[](i) == NULL){
			throw Exception("SpatialTransform: 6 TransformAxes were not specified.");
		}

		// Call the transform axis setup function.
		operator[](i).setup(*((Joint*)(&aJoint)));

		// check if it has a function
		if(!operator[](i).hasFunction()){
			// does it have a coordinate?
			if(operator[](i).getCoordinateNames().getSize() == 1)
				operator[](i).setFunction(new LinearFunction());
			else if(operator[](i).getCoordinateNames().getSize() > 1)
				throw Exception("TransformAxis: an appropriate multicoordinate function was not supplied");
			else
				operator[](i).setFunction(new Constant());
		}
	}
}

// Spatial Transform specific methods
OpenSim::Array<string> SpatialTransform::getCoordinateNames()
{
	OpenSim::Array<string> coordinateNames;

	for(int i=0; i<_numTransformAxes; i++){
		TransformAxis *transform = &operator[](i);
		for(int j = 0; j < transform->getCoordinateNames().getSize(); j++){
			string name = transform->getCoordinateNames()[j];
			if(coordinateNames.findIndex(name) < 0)
				coordinateNames.append(name);
		}
	}
	return coordinateNames;
}

std::vector<std::vector<int> > SpatialTransform::getCooridinateIndices()
{
	std::vector<std::vector<int> > coordIndices(6);
	Array<string> coordinateNames = getCoordinateNames();

	for(int i=0; i<_numTransformAxes; i++){
		TransformAxis *transform = &operator[](i);
		// Get the number of coordinates that dictate motion along this axis
		int ncoords = transform->getCoordinateNames().getSize();
		std::vector<int> findex(ncoords);
		for(int j=0; j< ncoords; j++){
			int ind = coordinateNames.findIndex(transform->getCoordinateNames()[j]);
			if (ind > -1)
				findex[j] = ind;
		}
		coordIndices[i] = findex;	
	}
	
	return coordIndices;
}
std::vector<const SimTK::Function*> SpatialTransform::getFunctions()
{
	std::vector<const SimTK::Function*> functions(_numTransformAxes);
	for(int i=0; i<_numTransformAxes; i++){
		functions[i] = operator[](i).getFunction().createSimTKFunction();
	}
	return functions;
}
std::vector<SimTK::Vec3> SpatialTransform::getAxes()
{
	std::vector<SimTK::Vec3> axes(_numTransformAxes);
	for(int i=0; i<_numTransformAxes; i++){
		axes[i] = operator[](i).getAxis();
	}

	return axes;
}


void SpatialTransform::scale(const SimTK::Vec3 scaleFactors)
{
	// Scale the spatial transform functions of translations only
	for (int i = 3; i < _numTransformAxes; i++) {
		TransformAxis *transform = &operator[](i);
        if (transform->hasFunction()) {
			Function& function = transform->getFunction();
			SimTK::Vec3 axis;
            transform->getAxis(axis);
            double scaleFactor = ~axis * scaleFactors;
			// If the function is already a MultiplierFunction, just update its scale factor.
			// Otherwise, make a MultiplierFunction from it and make the transform axis use
			// the new MultiplierFunction.
			MultiplierFunction* mf = dynamic_cast<MultiplierFunction*>(&function);
			if (mf) {
				mf->setScale(mf->getScale() * scaleFactor);
			} 
			else {
				mf = new MultiplierFunction();
				mf->setScale(scaleFactor);
				// Make a copy of the original function and delete the original
				// (so its node will be removed from the XML document).
				mf->setFunction((Function*)function.copy());
				transform->setFunction(mf);
			}
		}
	}
}
/**
 * constructIndepndentAxes checks if the TransformAxis at indices startIndex, startIndex+1, startIndex+2 
 * are independent and fixes them otherwise. It assumes that the first nAxes are ok
 */
void SpatialTransform::constructIndepndentAxes(int nAxes, int startIndex)
{
	if (nAxes == 3 || nAxes==0)	return;		// Nothing to do
	Vec3 v1 = operator[](0+startIndex).getAxis();
	Vec3 v2 = operator[](1+startIndex).getAxis();
	Vec3 v3 = operator[](2+startIndex).getAxis();
	if (nAxes ==2){ // Easy, make third axis the cross of the first 2.
		SimTK::Vec3 cross= (v1 % v2);
		cross.normalize();
		operator[](2+startIndex).setAxis(cross);
	}
	else {	// only v1 was specified, check if v2 is collinear if so, exchange v2, v3
		if (fabs(fabs(~v1 * v2) -1) < 1e-4){
			operator[](1+startIndex).setAxis(v3);
		}
		v2 = operator[](1+startIndex).getAxis();
		SimTK::Vec3 cross= (v2 % v1);
		cross.normalize();
		operator[](2+startIndex).setAxis(cross);	
	}
}
