/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Body.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include "Body.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/SimmMacros.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
//using namespace SimTK;
using namespace OpenSim;
using SimTK::Mat33;
using SimTK::Vec3;
using SimTK::DecorativeGeometry;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Body::~Body()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
Body::Body() :
    ModelComponent(),
	_mass(_massProp.getValueDbl()),
	_massCenter(_massCenterProp.getValueDblVec()),
	_inertiaXX(_inertiaXXProp.getValueDbl()),
	_inertiaYY(_inertiaYYProp.getValueDbl()),
	_inertiaZZ(_inertiaZZProp.getValueDbl()),
	_inertiaXY(_inertiaXYProp.getValueDbl()),
	_inertiaXZ(_inertiaXZProp.getValueDbl()),
	_inertiaYZ(_inertiaYZProp.getValueDbl()),
	_joint(_jointProp.getValueObjPtrRef()),
	_displayerProp(PropertyObj("", VisibleObject())),
	_displayer((VisibleObject&)_displayerProp.getValueObj()),
	_wrapObjectSetProp(PropertyObj("", WrapObjectSet())),
	_wrapObjectSet((WrapObjectSet&)_wrapObjectSetProp.getValueObj())
{
	setNull();
	setupProperties();
	//cout<<_mass<<endl;
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
Body::Body(const std::string &aName,double aMass,const SimTK::Vec3& aMassCenter,const SimTK::Inertia& aInertia) :
   ModelComponent(),
	_mass(_massProp.getValueDbl()),
	_massCenter(_massCenterProp.getValueDblVec()),
	_inertiaXX(_inertiaXXProp.getValueDbl()),
	_inertiaYY(_inertiaYYProp.getValueDbl()),
	_inertiaZZ(_inertiaZZProp.getValueDbl()),
	_inertiaXY(_inertiaXYProp.getValueDbl()),
	_inertiaXZ(_inertiaXZProp.getValueDbl()),
	_inertiaYZ(_inertiaYZProp.getValueDbl()),
	_joint(_jointProp.getValueObjPtrRef()),
	_displayerProp(PropertyObj("", VisibleObject())),
	_displayer((VisibleObject&)_displayerProp.getValueObj()),
	_wrapObjectSetProp(PropertyObj("", WrapObjectSet())),
	_wrapObjectSet((WrapObjectSet&)_wrapObjectSetProp.getValueObj())
{
	setNull();
	setupProperties();
	setName(aName);
	setMass(aMass);
	setMassCenter(aMassCenter);
	setInertia(aInertia);
	//cout<<_mass<<endl;
}

   
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBody Body to be copied.
 */
Body::Body(const Body &aBody) :
   ModelComponent(aBody),
	_mass(_massProp.getValueDbl()),
	_massCenter(_massCenterProp.getValueDblVec()),
	_inertiaXX(_inertiaXXProp.getValueDbl()),
	_inertiaYY(_inertiaYYProp.getValueDbl()),
	_inertiaZZ(_inertiaZZProp.getValueDbl()),
	_inertiaXY(_inertiaXYProp.getValueDbl()),
	_inertiaXZ(_inertiaXZProp.getValueDbl()),
	_inertiaYZ(_inertiaYZProp.getValueDbl()),
	_joint(_jointProp.getValueObjPtrRef()),
	_displayerProp(PropertyObj("", VisibleObject())),
	_displayer((VisibleObject&)_displayerProp.getValueObj()),
	_wrapObjectSetProp(PropertyObj("", WrapObjectSet())),
	_wrapObjectSet((WrapObjectSet&)_wrapObjectSetProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aBody);
	//cout<<_mass<<endl;
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Body to another.
 *
 * @param aBody Body to be copied.
 */
void Body::copyData(const Body &aBody)
{
	_mass = aBody._mass;
	_massCenter = aBody._massCenter;
	_inertiaXX = aBody._inertiaXX;
	_inertiaYY = aBody._inertiaYY;
	_inertiaZZ = aBody._inertiaZZ;
	_inertiaXY = aBody._inertiaXY;
	_inertiaXZ = aBody._inertiaXZ;
	_inertiaYZ = aBody._inertiaYZ;
	_displayer = aBody._displayer;
	//_index = aBody._index;
	_joint = dynamic_cast<Joint*>(Object::SafeCopy(aBody._joint));
	//bool check = (_joint==NULL && aBody._joint==NULL) || (*_joint == *(aBody._joint));
	if (_joint) _joint->setBody(*this);
	_model = NULL;
	_wrapObjectSet = aBody._wrapObjectSet;
}

//_____________________________________________________________________________
/**
 * Copy data members from an AbstractBody to an Body.
 *
 * @param aBody AbstractBody to be copied.
 *
void Body::copyData(const AbstractBody &aBody)
{
	// Mass
	_mass = aBody.getMass();

	// Mass center
	aBody.getMassCenter(_massCenter);

	// Inertia tensor
	SimTK::Mat33 inertia;
	aBody.getInertia(inertia);
	setInertia(inertia);

	// Joint
	//_joint = aBody._joint;  Problem? Abstract bodies do not have a joint?

	// Displayer
	_displayer = *aBody.getDisplayer();
}
*/

//_____________________________________________________________________________
/**
 * Set the data members of this Body to their null values.
 */
void Body::setNull()
{
	setAuthors("Frank C. Anderson, Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Body::setupProperties()
{
	double mass = 1.0;
	_massProp.setName("mass");
	_massProp.setValue(mass);
	_propertySet.append(&_massProp);

	const SimTK::Vec3 defaultMC(0.0, 0.0, 0.0);
	_massCenterProp.setName("mass_center");
	_massCenterProp.setValue(defaultMC);
	//_massCenterProp.setAllowableListSize(3);
	_propertySet.append(&_massCenterProp);

	// Ixx
	_inertiaXXProp.setName("inertia_xx");
	_inertiaXXProp.setValue(1.0);
	_propertySet.append(&_inertiaXXProp);

	// Iyy
	_inertiaYYProp.setName("inertia_yy");
	_inertiaYYProp.setValue(1.0);
	_propertySet.append(&_inertiaYYProp);

	// Izz
	_inertiaZZProp.setName("inertia_zz");
	_inertiaZZProp.setValue(1.0);
	_propertySet.append(&_inertiaZZProp);

	// Ixy
	_inertiaXYProp.setName("inertia_xy");
	_inertiaXYProp.setValue(0.0);
	_propertySet.append(&_inertiaXYProp);

	// Ixz
	_inertiaXZProp.setName("inertia_xz");
	_inertiaXZProp.setValue(0.0);
	_propertySet.append(&_inertiaXZProp);

	// Iyz
	_inertiaYZProp.setName("inertia_yz");
	_inertiaYZProp.setValue(0.0);
	_propertySet.append(&_inertiaYZProp);

	// Joint
	_jointProp.setComment("Joint that connects this body with the parent body.");
	_jointProp.setName("Joint");
	_propertySet.append(&_jointProp);

	_displayerProp.setName("Displayer");
	_propertySet.append(&_displayerProp);

	_wrapObjectSetProp.setName("WrapObjectSet");
	_propertySet.append(&_wrapObjectSetProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this Body.
 */
void Body::connectToModel(Model& aModel)
{
	Super::connectToModel(aModel);

	for(int i=0; i<_wrapObjectSet.getSize(); i++)
		_wrapObjectSet.get(i).connectToModelAndBody(aModel, *this);
	if(this->getName() != "ground") {
		if(_joint) _joint->connectToModel(aModel);
    }
}

void Body::addToSystem(SimTK::MultibodySystem& system) const
{
	Super::addToSystem(system);

	if(getName() == "ground"){
		Body * mutableThis = const_cast<Body *>(this);
		mutableThis->_index = SimTK::GroundIndex;
	}

}

void Body::generateDecorations(bool fixed, 
                               const ModelDisplayHints& hints,
                               const SimTK::State& state,
                               SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const
{
    const SimTK::SimbodyMatterSubsystem& matter = getModel().getMatterSubsystem();
    // Display wrap objects.
    if (hints.getShowWrapGeometry() && fixed) { // Wrap Geometry is Fixed
        /*const Vec3 color(SimTK::Cyan);
        Transform ztoy;
        ztoy.updR().setRotationFromAngleAboutX(SimTK_PI/2);
        const Transform& X_GB = 
            matter.getMobilizedBody(getIndex()).getBodyTransform(state);*/
        const WrapObjectSet& wrapObjects = getWrapObjectSet();
        for (int j = 0; j < wrapObjects.getSize(); j++) {
            wrapObjects[j].generateDecorations(getModel(), hints, state, geometry);
            /*
            const string type = wrapObjects[j].getConcreteClassName();
            if (type == "WrapCylinder") {
                const WrapCylinder* cylinder = 
                    dynamic_cast<const WrapCylinder*>(&wrapObjects[j]);
                if (cylinder != NULL) {
                    Transform X_GW = X_GB*cylinder->getTransform()*ztoy;
                    geometry.push_back(
                        DecorativeCylinder(cylinder->getRadius(), 
                        cylinder->getLength()/2)
                        .setTransform(X_GW).setResolution(_dispWrapResolution)
                        .setColor(color).setOpacity(_dispWrapOpacity));
                }
            }
            else if (type == "WrapEllipsoid") {
                const WrapEllipsoid* ellipsoid = 
                    dynamic_cast<const WrapEllipsoid*>(&wrapObjects[j]);
                if (ellipsoid != NULL) {
                    Transform X_GW = X_GB*ellipsoid->getTransform();
                    geometry.push_back(
                        DecorativeEllipsoid(ellipsoid->getRadii())
                        .setTransform(X_GW).setResolution(_dispWrapResolution)
                        .setColor(color).setOpacity(_dispWrapOpacity));
                }
            }
            else if (type == "WrapSphere") {
                const WrapSphere* sphere = 
                    dynamic_cast<const WrapSphere*>(&wrapObjects[j]);
                if (sphere != NULL) {
                    Transform X_GW = X_GB*sphere->getTransform();
                    geometry.push_back(
                        DecorativeSphere(sphere->getRadius())
                        .setTransform(X_GW).setResolution(_dispWrapResolution)
                        .setColor(color).setOpacity(_dispWrapOpacity));
                }
            }*/
        }
    }


    /*
    // Display contact geometry objects.
    if (hints.getShowContactGeometry()) {
        const Vec3 color(SimTK::Green);
        Transform ztoy;
        ztoy.updR().setRotationFromAngleAboutX(SimTK_PI/2);
        const ContactGeometrySet& contactGeometries = _model.getContactGeometrySet();

        for (int i = 0; i < contactGeometries.getSize(); i++) {
            const OpenSim::Body& body = contactGeometries.get(i).getBody();
            const Transform& X_GB = 
                matter.getMobilizedBody(body.getIndex()).getBodyTransform(state);
            const string type = contactGeometries.get(i).getConcreteClassName();
            const int displayPref = contactGeometries.get(i).getDisplayPreference();
            //cout << type << ": " << contactGeometries.get(i).getName() << ": disp pref = " << displayPref << endl;

            if (type == "ContactSphere" && displayPref == 4) {
                ContactSphere* sphere = 
                    dynamic_cast<ContactSphere*>(&contactGeometries.get(i));
                if (sphere != NULL) {
                    Transform X_GW = X_GB*sphere->getTransform();
                    geometry.push_back(
                        DecorativeSphere(sphere->getRadius())
                        .setTransform(X_GW).setResolution(_dispContactResolution)
                        .setColor(color).setOpacity(_dispContactOpacity));
                }
            }
        }
    } */
    // Now the meshes
        if (fixed){
            Vec3 scale; 
            _displayer.getScaleFactors(scale);
            const Transform X_BV = _displayer.getTransform();
            const OpenSim::GeometrySet& geom = _displayer.getGeometrySet();
            for (int i = 0; i < geom.getSize(); i++){
                DisplayGeometry& dGeom = geom.get(i);
                const DisplayGeometry::DisplayPreference pref = dGeom.getDisplayPreference();
                DecorativeGeometry::Representation rep;
                switch(pref) {
                    case DisplayGeometry::None: 
                        continue; // don't bother with this one (TODO: is that right)
                    case DisplayGeometry::WireFrame: 
                        rep=DecorativeGeometry::DrawWireframe; 
                        break;
                    case DisplayGeometry::SolidFill:
                    case DisplayGeometry::FlatShaded:
                    case DisplayGeometry::GouraudShaded:
                        rep = DecorativeGeometry::DrawSurface;
                        break;
                    default: assert(!"bad DisplayPreference");
                };
                Transform bodyToMesh = dGeom.getTransform();
                SimTK::DecorativeMeshFile mesh(dGeom.getGeometryFile());
                mesh.setBodyId(getIndex());
                const Vec3 netScale = dGeom.getScaleFactors()
                                        .elementwiseMultiply(scale);
                mesh.setScaleFactors(netScale);
                mesh.setTransform(bodyToMesh);
                mesh.setColor(dGeom.getColor());
                mesh.setOpacity(dGeom.getOpacity());
                mesh.setUserRef(&geom.get(i));
                mesh.setRepresentation(rep);
                geometry.push_back(mesh);
            }
        }

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
Body& Body::operator=(const Body &aBody)
{
	// BASE CLASS
	Object::operator=(aBody);

	copyData(aBody);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Add display geometry to body.
 *
 * @param aGeometryFileName Geometry filename.
 */
void Body::addDisplayGeometry(const std::string &aGeometryFileName)
{
	updDisplayer()->setGeometryFileName(updDisplayer()->getNumGeometryFiles(), aGeometryFileName);
}
//_____________________________________________________________________________
/**
 * Get the mass of the body.
 *
 * @return Mass of body from Simbody code.
 */
double Body::getMass() const
{
	return _mass;
}
//_____________________________________________________________________________
/**
 * Set the mass of the body.
 *
 * @param aMass mass of body.
 * @return Whether mass was successfully changed.
 */
bool Body::setMass(double aMass)
{
	if(aMass<0.0) {
		cerr<<"Body.setMass(): ERROR- zero or negative mass not allowed.\n";
		return false;
	}
	_mass = aMass;
    if (_model != NULL)
        _model->invalidateSystem();
	return true;
}

//done_____________________________________________________________________________
/**
 * Get the mass center of the body.
 *
 * @param rVec XYZ coordinates of mass center are returned here.
 */
void Body::getMassCenter(SimTK::Vec3& rVec) const
{
	rVec=_massCenter;
}
//_____________________________________________________________________________
/**
 * Set the mass center of the body.
 *
 * @param aVec XYZ coordinates of mass center.
 * @return Whether mass center was successfully changed.
 */
bool Body::setMassCenter(const SimTK::Vec3& aVec)
{
	_massCenter=aVec;
    if (_model != NULL)
        _model->invalidateSystem();
	return true;
}

//_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param 3x3 inertia matrix.
 */
void Body::getInertia(SimTK::Mat33 &rInertia) const
{
	rInertia[0][0] = _inertiaXX;
	rInertia[0][1] = _inertiaXY;
	rInertia[0][2] = _inertiaXZ;
	rInertia[1][0] = _inertiaXY;
	rInertia[1][1] = _inertiaYY;
	rInertia[1][2] = _inertiaYZ;
	rInertia[2][0] = _inertiaXZ;
	rInertia[2][1] = _inertiaYZ;
	rInertia[2][2] = _inertiaZZ;
}
//_____________________________________________________________________________
/**
 * Set the inertia matrix of the body.
 *
 * @param aInertia 9-element inertia matrix.
 * @return Whether inertia matrix was successfully changed.
 */
bool Body::setInertia(const SimTK::Inertia& aInertia)
{
	Mat33 inertiaMat = aInertia.toMat33();
	_inertiaXX = inertiaMat[0][0];
	_inertiaXY = inertiaMat[0][1];
	_inertiaXZ = inertiaMat[0][2];
	_inertiaYY = inertiaMat[1][1];
	_inertiaYZ = inertiaMat[1][2];
	_inertiaZZ = inertiaMat[2][2];
    if (_model != NULL)
        _model->invalidateSystem();
	return true;
}

//_____________________________________________________________________________
/**
 * Set the joint for this body.
 *
 * @param aJoint Joint connecting this body to the parent body.
 */
void Body::
setJoint(Joint& aJoint)
{
	_joint = &aJoint;
}

Joint& Body::getJoint() const {
    if (_joint == NULL)
        throw Exception("Body::getJoint(): This Body does not have a Joint");
    return *_joint;
}



//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the body.
 *
 * @param aScaleFactors XYZ scale factors.
 * @param aScaleMass whether or not to scale mass properties
 */
void Body::scale(const SimTK::Vec3& aScaleFactors, bool aScaleMass)
{
   // Base class, to scale wrap objects
	for (int i=0; i<_wrapObjectSet.getSize(); i++)
		_wrapObjectSet.get(i).scale(aScaleFactors);

	SimTK::Vec3 oldScaleFactors;
	getDisplayer()->getScaleFactors(oldScaleFactors);

	for(int i=0; i<3; i++) {
		_massCenter[i] *= aScaleFactors[i];
		oldScaleFactors[i] *= aScaleFactors[i];
	}
	// Update scale factors for displayer
	updDisplayer()->setScaleFactors(oldScaleFactors);

	if (getName() != "ground")	// The following throws an exception if applied to ground.
		scaleInertialProperties(aScaleFactors, aScaleMass);
}

//_____________________________________________________________________________
/**
 * Scale the body's mass and inertia tensor.
 *
 * @param aScaleFactors XYZ scale factors.
 * @param aScaleMass Whether or not to scale the mass
 */
void Body::scaleInertialProperties(const SimTK::Vec3& aScaleFactors, bool aScaleMass)
{
	// Save the unscaled mass for possible use later.
	double unscaledMass = _mass;

	// Calculate and store the product of the scale factors.
	double massScaleFactor = fabs(aScaleFactors[0] * aScaleFactors[1] * aScaleFactors[2]);

	// Scale the mass.
	if (aScaleMass)
		_mass *= massScaleFactor;

	// Scale the inertia tensor.
	SimTK::Mat33 inertia;
	getInertia(inertia);

	// If the mass is zero, then make the inertia tensor zero as well.
	// If the X, Y, Z scale factors are equal, then you can scale the
	// inertia tensor exactly by the square of the scale factor (and
	// possibly by massScaleFactor), since each element in the tensor
	// is proportional to the square of one or more dimensional
	// measurements. For determining if the scale factors are equal,
	// ignore reflections-- look only at the absolute value of the factors.
	if (_mass <= ROUNDOFF_ERROR) {
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				inertia[i][j]=0.0;

	} else if (EQUAL_WITHIN_ERROR(DABS(aScaleFactors[0]), DABS(aScaleFactors[1])) &&
		        EQUAL_WITHIN_ERROR(DABS(aScaleFactors[1]), DABS(aScaleFactors[2]))) {
		// If the mass is also being scaled, scale the inertia terms by massScaleFactor.
		if (aScaleMass) {
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					inertia[i][j] *= massScaleFactor;
		}

		// Now scale by the length-squared component.
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				inertia[i][j] *= (aScaleFactors[0] * aScaleFactors[0]);

	} else {
		// If the scale factors are not equal, then assume that the segment
		// is a cylinder and the inertia is calculated about one end of it.
		int axis;

		// 1. Find the smallest diagonal component. This dimension is the axis
		//    of the cylinder.
		if (inertia[0][0] <= inertia[1][1]){
			if (inertia[0][0] <= inertia[2][2])
				axis = 0;
			else
				axis = 2;

		} else if (inertia[1][1] <= inertia[2][2]) {
			axis = 1;

		} else {
			axis = 2;
		}

		// 2. The smallest inertia component is equal to 0.5 * mass * radius * radius,
		//    so you can rearrange and solve for the radius.
		int oa;
		double radius, rad_sqr, length;
		double term = 2.0 * inertia[axis][axis] / unscaledMass;
		if (term < 0.0)
			radius = 0.0;
		else
			radius = sqrt(term);

		// 3. Choose either of the other diagonal components and use it to solve for the
		//    length of the cylinder. This component is equal to:
		//    0.083 * mass * length * length  +  0.25 * mass * radius * radius
		if (axis == 0)
			oa = 1;
		else
			oa = 0;
		term = 12.0 * (inertia[oa][oa] - 0.25 * unscaledMass * radius * radius) / unscaledMass;
		if (term < 0.0)
			length = 0.0;
		else
			length = sqrt(term);

		// 4. Scale the radius and length, and recalculate the diagonal inertia terms.
		length *= DABS(aScaleFactors[axis]);

		if (axis == 0) {
			rad_sqr = radius * DABS(aScaleFactors[1]) * radius * DABS(aScaleFactors[2]);
			inertia[0][0] = 0.5 * _mass * rad_sqr;
			inertia[1][1] = _mass * ((length * length / 12.0) + 0.25 * rad_sqr);
			inertia[2][2] = _mass * ((length * length / 12.0) + 0.25 * rad_sqr);

		} else if (axis == 1) {
			rad_sqr = radius * DABS(aScaleFactors[0]) * radius * DABS(aScaleFactors[2]);
			inertia[0][0] = _mass * ((length * length / 12.0) + 0.25 * rad_sqr);
			inertia[1][1] = 0.5 * _mass * rad_sqr;
			inertia[2][2] = _mass * ((length * length / 12.0) + 0.25 * rad_sqr);

		} else {
			rad_sqr = radius * DABS(aScaleFactors[0]) * radius * DABS(aScaleFactors[1]);
			inertia[0][0] = _mass * ((length * length / 12.0) + 0.25 * rad_sqr);
			inertia[1][1] = _mass * ((length * length / 12.0) + 0.25 * rad_sqr);
			inertia[2][2] = 0.5 * _mass * rad_sqr;
		}

		// 5. Scale the inertia products, in case some are non-zero. These are scaled by
		//    two scale factors for the length term (which two depend on the inertia term
		//    being scaled), and, if the mass is also scaled, by massScaleFactor.
		inertia[0][1] *= DABS((aScaleFactors[0] * aScaleFactors[1]));
		inertia[0][2] *= DABS((aScaleFactors[0] * aScaleFactors[2]));
		inertia[1][0] *= DABS((aScaleFactors[1] * aScaleFactors[0]));
		inertia[1][2] *= DABS((aScaleFactors[1] * aScaleFactors[2]));
		inertia[2][0] *= DABS((aScaleFactors[2] * aScaleFactors[0]));
		inertia[2][1] *= DABS((aScaleFactors[2] * aScaleFactors[1]));

		if (aScaleMass) {
			inertia[0][1] *= massScaleFactor;
			inertia[0][2] *= massScaleFactor;
			inertia[1][0] *= massScaleFactor;
			inertia[1][2] *= massScaleFactor;
			inertia[2][0] *= massScaleFactor;
			inertia[2][1] *= massScaleFactor;
		}
	}

	setInertia(SimTK::Inertia(inertia));
}

//_____________________________________________________________________________
/**
 * Scale the body's mass and inertia tensor (represents a scaling of the
 * body's density).
 *
 * @param aScaleFactors XYZ scale factors.
 */
void Body::scaleMass(double aScaleFactor)
{
	if (_index==0)	// The following throws an exception if applied to ground.
		return;

	_mass *= aScaleFactor;
	_inertiaXX *= aScaleFactor;
	_inertiaYY *= aScaleFactor;
	_inertiaZZ *= aScaleFactor;
	_inertiaXY *= aScaleFactor;
	_inertiaXZ *= aScaleFactor;
	_inertiaYZ *= aScaleFactor;
    if (_model != NULL)
        _model->invalidateSystem();
}

//=============================================================================
// UTILITY
//=============================================================================
SimTK::MassProperties Body::getMassProperties()
{
	SimTK::Inertia inertiaAboutCOM = SimTK::Inertia(_inertiaXX, _inertiaYY, _inertiaZZ,
													_inertiaXY, _inertiaXZ, _inertiaYZ);
	SimTK::Inertia inertiaAboutOrigin = inertiaAboutCOM.shiftFromMassCenter(_massCenter, _mass);
	return SimTK::MassProperties(_mass, _massCenter, inertiaAboutOrigin);
}

//_____________________________________________________________________________
/**
 * Get the named wrap object, if it exists.
 *
 * @param aName Name of the wrap object.
 * @return Pointer to the wrap object.
 */
WrapObject* Body::getWrapObject(const string& aName) const
{
	int i;

	for (i = 0; i < _wrapObjectSet.getSize(); i++) {
		if (aName == _wrapObjectSet.get(i).getName())
			return &_wrapObjectSet.get(i);
	}

	return NULL;
}

void Body::addWrapObject(WrapObject* wrap) {
	_wrapObjectSet.adoptAndAppend(wrap);
}

//=============================================================================
// I/O
//=============================================================================
void Body::getScaleFactors(SimTK::Vec3& scales) const
{

	SimTK::Vec3 scaleFactors;
	_displayer.getScaleFactors(scaleFactors);

	scales = scaleFactors;

}
