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
#include "Body.h"
#include "WeldConstraint.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
//using namespace SimTK;
using namespace OpenSim;
using SimTK::Mat33;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Body::Body() : ModelComponent()
{
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
Body::Body(const std::string &aName,double aMass,const SimTK::Vec3& aMassCenter,const SimTK::Inertia& aInertia) :
    ModelComponent()
{
    constructProperties();
    setName(aName);
    set_mass(aMass);
    set_mass_center(aMassCenter);
    setInertia(aInertia);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Body::constructProperties()
{
    constructProperty_mass(SimTK::NaN);
    constructProperty_mass_center(SimTK::Vec3(0));
    constructProperty_inertia(SimTK::Vec6(0));
    constructProperty_WrapObjectSet(WrapObjectSet());
    constructProperty_VisibleObject(VisibleObject());
}


void Body::finalizeFromProperties()
{
    const SimTK::MassProperties& massProps = getMassProperties();
    _internalRigidBody = SimTK::Body::Rigid(massProps);

    _index.invalidate();
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

    for(int i=0; i< get_WrapObjectSet().getSize(); i++)
        get_WrapObjectSet().get(i).connectToModelAndBody(aModel, *this);

    int nslaves = (int)_slaves.size();

    if (nslaves) {
        int nbods = nslaves + 1; // include the master
        const SimTK::MassProperties& massProps = getMassProperties();
        SimTK::MassProperties slaveMassProps(massProps.getMass() / nbods,
                                             massProps.getMassCenter(), massProps.getUnitInertia());

        // update the portion taken on by the master
        _internalRigidBody = SimTK::Body::Rigid(slaveMassProps);

        // and the slaves
        for (int i = 0; i < nslaves; ++i) {
            _slaves[i]->_internalRigidBody = SimTK::Body::Rigid(slaveMassProps);
        }
    }

}

void Body::addToSystem(SimTK::MultibodySystem& system) const
{
    if(getName() == "ground") {
        Body * mutableThis = const_cast<Body *>(this);
        mutableThis->_index = SimTK::GroundIndex;
    }

    // Add subcomponents of the Body (namely its Joint)
    Super::addToSystem(system);
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
 * Get the inertia matrix of the body.
 *
 * @return 3x3 inertia matrix.
 */
const SimTK::Inertia& Body::getInertia() const
{
    // Has not been set programmatically
    if (_inertia.isNaN()) {
        // initialize from properties
        const SimTK::Vec6& Ivec = get_inertia();
        _inertia = SimTK::Inertia(Ivec.getSubVec<3>(0), Ivec.getSubVec<3>(3));
    }
    return _inertia;
}
//_____________________________________________________________________________
/**
 * Set the inertia matrix of the body.
 *
 * @param aInertia 9-element inertia matrix.
 */
void Body::setInertia(const SimTK::Inertia& inertia)
{
    _inertia = inertia;
    const SimTK::SymMat33& I = _inertia.asSymMat33();
    upd_inertia()[0] = I[0][0];
    upd_inertia()[1] = I[1][1];
    upd_inertia()[2] = I[2][2];
    upd_inertia()[3] = I[0][1];
    upd_inertia()[4] = I[0][2];
    upd_inertia()[5] = I[1][2];
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
    for (int i=0; i< get_WrapObjectSet().getSize(); i++)
        upd_WrapObjectSet().get(i).scale(aScaleFactors);

    SimTK::Vec3 oldScaleFactors;
    getDisplayer()->getScaleFactors(oldScaleFactors);

    for(int i=0; i<3; i++) {
        upd_mass_center()[i] *= aScaleFactors[i];
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
    double unscaledMass = get_mass();

    // Calculate and store the product of the scale factors.
    double massScaleFactor = fabs(aScaleFactors[0] * aScaleFactors[1] * aScaleFactors[2]);

    // Scale the mass.
    if (aScaleMass)
        upd_mass() *= massScaleFactor;

    SimTK::SymMat33 inertia = _inertia.asSymMat33();

    // If the mass is zero, then make the inertia tensor zero as well.
    // If the X, Y, Z scale factors are equal, then you can scale the
    // inertia tensor exactly by the square of the scale factor (and
    // possibly by massScaleFactor), since each element in the tensor
    // is proportional to the square of one or more dimensional
    // measurements. For determining if the scale factors are equal,
    // ignore reflections-- look only at the absolute value of the factors.
    if (get_mass() <= SimTK::Eps) {
        inertia *= 0.0;
    }
    else if (SimTK::isNumericallyEqual(aScaleFactors[0], aScaleFactors[1])
             && SimTK::isNumericallyEqual(aScaleFactors[1], aScaleFactors[2])) {
        // If the mass is also being scaled, scale the inertia terms by massScaleFactor.
        if (aScaleMass) {
            inertia *= massScaleFactor;
        }

        // Now scale by the length-squared component.
        inertia *= (aScaleFactors[0] * aScaleFactors[0]);

    } else {
        // If the scale factors are not equal, then assume that the segment
        // is a cylinder and the inertia is calculated about one end of it.
        int axis;

        // 1. Find the smallest diagonal component. This dimension is the axis
        //    of the cylinder.
        if (inertia[0][0] <= inertia[1][1]) {
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
        length *= aScaleFactors[axis];

        if (axis == 0) {
            rad_sqr = radius * (aScaleFactors[1]) * radius * (aScaleFactors[2]);
            inertia[0][0] = 0.5 * get_mass() * rad_sqr;
            inertia[1][1] = get_mass() * ((length * length / 12.0) + 0.25 * rad_sqr);
            inertia[2][2] = get_mass() * ((length * length / 12.0) + 0.25 * rad_sqr);

        } else if (axis == 1) {
            rad_sqr = radius * (aScaleFactors[0]) * radius * (aScaleFactors[2]);
            inertia[0][0] = get_mass() * ((length * length / 12.0) + 0.25 * rad_sqr);
            inertia[1][1] = 0.5 * get_mass() * rad_sqr;
            inertia[2][2] = get_mass() * ((length * length / 12.0) + 0.25 * rad_sqr);

        } else {
            rad_sqr = radius * (aScaleFactors[0]) * radius * (aScaleFactors[1]);
            inertia[0][0] = get_mass() * ((length * length / 12.0) + 0.25 * rad_sqr);
            inertia[1][1] = get_mass() * ((length * length / 12.0) + 0.25 * rad_sqr);
            inertia[2][2] = 0.5 * get_mass() * rad_sqr;
        }

        // 5. Scale the inertia products, in case some are non-zero. These are scaled by
        //    two scale factors for the length term (which two depend on the inertia term
        //    being scaled), and, if the mass is also scaled, by massScaleFactor.
        inertia[0][1] *= ((aScaleFactors[0] * aScaleFactors[1]));
        inertia[0][2] *= ((aScaleFactors[0] * aScaleFactors[2]));
        inertia[1][0] *= ((aScaleFactors[1] * aScaleFactors[0]));
        inertia[1][2] *= ((aScaleFactors[1] * aScaleFactors[2]));
        inertia[2][0] *= ((aScaleFactors[2] * aScaleFactors[0]));
        inertia[2][1] *= ((aScaleFactors[2] * aScaleFactors[1]));

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

    upd_mass() *= aScaleFactor;
    _inertia *= aScaleFactor;
    upd_inertia() *= aScaleFactor;
}

//=============================================================================
// UTILITY
//=============================================================================
SimTK::MassProperties Body::getMassProperties() const
{
    const double& m = get_mass();
    const Vec3& com = get_mass_center();
    const SimTK::Inertia& Icom = getInertia();

    return SimTK::MassProperties(m, com, Icom.shiftFromMassCenter(com, m));
}

//_____________________________________________________________________________
/**
 * Get the named wrap object, if it exists.
 *
 * @param aName Name of the wrap object.
 * @return Pointer to the wrap object.
 */
const WrapObject* Body::getWrapObject(const string& aName) const
{
    int i;

    for (i = 0; i < get_WrapObjectSet().getSize(); i++) {
        if (aName == get_WrapObjectSet()[i].getName())
            return &get_WrapObjectSet()[i];
    }
    return nullptr;
}

void Body::addWrapObject(WrapObject* wrap) {
    upd_WrapObjectSet().adoptAndAppend(wrap);
}

//=============================================================================
// I/O
//=============================================================================
void Body::getScaleFactors(SimTK::Vec3& scales) const
{

    SimTK::Vec3 scaleFactors;
    get_VisibleObject().getScaleFactors(scaleFactors);

    scales = scaleFactors;

}

void Body::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    if (versionNumber < XMLDocument::getLatestVersion()) {
        if (versionNumber < 30500) {
            SimTK::Vec6 newInertia(1.0, 1.0, 1.0, 0., 0., 0.);
            std::string inertiaComponents[] = { "inertia_xx", "inertia_yy", "inertia_zz", "inertia_xy", "inertia_xz", "inertia_yz" };
            for (int i = 0; i<6; ++i) {
                SimTK::Xml::element_iterator iIter = aNode.element_begin(inertiaComponents[i]);
                if (iIter != aNode.element_end()) {
                    newInertia[i] = iIter->getValueAs<double>();
                    aNode.removeNode(iIter);
                }
            }
            std::ostringstream strs;
            for (int i = 0; i < 6; ++i) {
                strs << newInertia[i];
                if (i < 5) strs << " ";
            }
            std::string strInertia = strs.str();
            SimTK::Xml::Element inertiaNode("inertia", strInertia);
            aNode.insertNodeAfter(aNode.element_end(), inertiaNode);
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
}

Body* Body::addSlave()
{
    Body* slave = new Body();
    int count = (int)_slaves.size();

    stringstream name;
    name << getName() << "_slave_" << count;
    slave->setName(name.str());

    //add to internal list as memory owner
    _slaves.push_back(slave);

    //add to list of subcomponents to autotically add to system and initialize
    addComponent(slave);

    return slave;
}
