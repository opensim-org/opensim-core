/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Body.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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
#include "Simbody.h"
#include "Body.h"
#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelVisualizer.h>

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
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Body::Body() : PhysicalFrame()
{
    //_body = this;
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
Body::Body(const std::string &aName,double aMass,const SimTK::Vec3& aMassCenter,const SimTK::Inertia& aInertia) :
    PhysicalFrame()
{
    constructProperties();
    setName(aName);
    set_mass(aMass);
    set_mass_center(aMassCenter);
    setInertia(aInertia);
    // Better use name search or more robust method
    upd_geometry(0).setFrameName(aName);
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
}


void Body::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
    const SimTK::MassProperties& massProps = getMassProperties();
    _internalRigidBody = SimTK::Body::Rigid(massProps);
}

//_____________________________________________________________________________
/* Connect this Body to the Model it belongs to
 *
 * @param aModel OpenSim model containing this Body.
 */
void Body::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);
    
    int nslaves = (int)_slaves.size();

    if (nslaves){
        int nbods = nslaves + 1; // include the master
        const SimTK::MassProperties& massProps = getMassProperties();
        SimTK::MassProperties slaveMassProps(massProps.getMass() / nbods,
            massProps.getMassCenter(), massProps.getUnitInertia());

        // update the portion taken on by the master
        _internalRigidBody = SimTK::Body::Rigid(slaveMassProps);

        // and the slaves
        for (int i = 0; i < nslaves; ++i){
            _slaves[i]->_internalRigidBody = SimTK::Body::Rigid(slaveMassProps);
            _slaves[i]->setInertia(slaveMassProps.getUnitInertia());
            _slaves[i]->setMass(slaveMassProps.getMass());
            _slaves[i]->setMassCenter(slaveMassProps.getMassCenter());
        }
    }
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
void Body::addMeshGeometry(const std::string& aGeometryFileName, const SimTK::Vec3 scale)
{
    Mesh geom(aGeometryFileName);
    geom.set_scale_factors(scale);
    if (geom.getFrameName() == "")
        geom.setFrameName(getName());
    addGeometry(geom);
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
    if (_inertia.isNaN()){
        // initialize from properties
        const double& m = getMass();
        // if mass is zero, non-zero inertia makes no sense
        if (-SimTK::SignificantReal <= m && m <= SimTK::SignificantReal){
            // force zero intertia
            cout<<"Body '"<<getName()<<"' is massless but nonzero inertia provided.";
            cout<<" Inertia reset to zero. "<<"Otherwise provide nonzero mass."<< endl;
            _inertia = SimTK::Inertia(0);
        }
        else{
            const SimTK::Vec6& Ivec = get_inertia();
            try {
                _inertia = SimTK::Inertia(Ivec.getSubVec<3>(0), Ivec.getSubVec<3>(3));
            } 
            catch (const std::exception& ex){
                // Should throw an Exception but we have models we have released with
                // bad intertias. E.g. early gait23 models had an error in the inertia
                // of the toes Body. We cannot allow failures with our models so 
                // raise a warning and do something sensible with the values at hand.
                cout << "WARNING: Body " + getName() + " has invalid inertia. " << endl;
                cout << ex.what() << endl;

                // get some aggregate value for the inertia based on exsiting values
                double diag = Ivec.getSubVec<3>(0).norm()/sqrt(3.0);

                // and then assume a spherical shape.
                _inertia = SimTK::Inertia(Vec3(diag), Vec3(0));
                
                cout << getName() << " Body's inertia being reset to:" << endl;
                cout << _inertia << endl;
            }
        }
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
    Super::scale(aScaleFactors);
    for(int i=0; i<3; i++) {
        upd_mass_center()[i] *= aScaleFactors[i];
    }

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
    double massScaleFactor = abs(aScaleFactors[0] * aScaleFactors[1] * aScaleFactors[2]);

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

    try{
        const SimTK::Inertia& Icom = getInertia();

        SimTK::Inertia Ib = Icom;
        // If com and body, b, frame are coincident then don't bother shifting
        if (com.norm() > SimTK::Eps) {
            // shift if com has nonzero distance from b
            Ib = Icom.shiftFromMassCenter(com, m);
        }
    
        return SimTK::MassProperties(m, com, Ib);
    }
    catch (const std::exception& ex) {
        string msg = "Body " + getName() + " has invalid mass properties. ";
        msg += ex.what();
        throw Exception(msg, __FILE__, __LINE__);
    }
}

//=============================================================================
// I/O
//=============================================================================

void Body::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    if (versionNumber < XMLDocument::getLatestVersion()){
        if (versionNumber < 30500) {
            SimTK::Vec6 newInertia(1.0, 1.0, 1.0, 0., 0., 0.);
            std::string inertiaComponents[] = { "inertia_xx", "inertia_yy", "inertia_zz", "inertia_xy", "inertia_xz", "inertia_yz" };
            for (int i = 0; i<6; ++i){
                SimTK::Xml::element_iterator iIter = aNode.element_begin(inertiaComponents[i]);
                if (iIter != aNode.element_end()){
                    newInertia[i] = iIter->getValueAs<double>();
                    aNode.removeNode(iIter);
                }
            }
            std::ostringstream strs;
            for (int i = 0; i < 6; ++i){
                strs << newInertia[i];
                if (i < 5) strs << " ";
            }
            std::string strInertia = strs.str();
            SimTK::Xml::Element inertiaNode("inertia", strInertia);
            aNode.insertNodeAfter(aNode.element_end(), inertiaNode);
        }
        if (versionNumber < 30502){
            // Find node for <VisibleObject> remove it then create Geometry for the 
            SimTK::Xml::element_iterator iIter = aNode.element_begin("VisibleObject");
            if (iIter != aNode.element_end()){
                SimTK::Xml::Element visObjElement = SimTK::Xml::Element::getAs(aNode.removeNode(iIter));
                // Scale factors
                SimTK::Vec3 outerScaleFactors(1.0);
                SimTK::Xml::element_iterator outerScaleFactortIter = visObjElement.element_begin("scale_factors");
                if (outerScaleFactortIter != visObjElement.element_end()){
                    outerScaleFactors = outerScaleFactortIter->getValueAs<SimTK::Vec3>();
                }
                SimTK::Vec6 outerTransform(0.0);
                SimTK::Xml::element_iterator outerTransformIter = visObjElement.element_begin("transform");
                if (outerTransformIter != visObjElement.element_end()){
                    outerTransform = outerTransformIter->getValueAs<SimTK::Vec6>();
                }
                
                if (versionNumber < 20101) {
                    SimTK::Xml::element_iterator geometryIter = visObjElement.element_begin("geometry_files");
                    SimTK::Array_<SimTK::String> oldGeometryFiles;
                    if (geometryIter != aNode.element_end()){
                        geometryIter->getValueAs(oldGeometryFiles);
                    }
                    std::string bodyName = aNode.getRequiredAttribute("name").getValue();
                    SimTK::Xml::Element geometrySetNode("geometry");
                    aNode.insertNodeAfter(aNode.element_end(), geometrySetNode);
                    // Create Mesh node for each item in oldGeometryFiles
                    for (unsigned ng = 0; ng < oldGeometryFiles.size(); ng++) {
                        SimTK::Xml::Element meshNode("Mesh");
                        std::string geomName = bodyName + "_geom_" + to_string(ng);
                        meshNode.setAttributeValue("name", geomName);
                        SimTK::Xml::Element meshFileNode("mesh_file", oldGeometryFiles[ng]);
                        std::stringstream localScaleStr;
                        localScaleStr << outerScaleFactors[0] << " " << outerScaleFactors[1]
                            << " " << outerScaleFactors[2];
                        SimTK::Xml::Element scaleFactorsNode("scale_factors", localScaleStr.str());
                        meshNode.insertNodeAfter(meshNode.element_end(), scaleFactorsNode);
                        meshNode.insertNodeAfter(meshNode.element_end(), meshFileNode);
                        XMLDocument::addConnector(meshNode, "Connector_Frame_", "frame", bodyName);
                        geometrySetNode.insertNodeAfter(geometrySetNode.element_end(), meshNode);
                    }
                }
                else {
                    SimTK::Xml::element_iterator geomSetIter = visObjElement.element_begin("GeometrySet");
                    if (geomSetIter != visObjElement.element_end()){
                        convertDisplayGeometryToGeometryXML(aNode, outerScaleFactors, outerTransform, *geomSetIter);
                        geomSetIter->setElementTag("geometry");
                    }
                }
                // Regardless add a node for FrameGeometry to control the display of BodyFrame
                std::string bodyName = aNode.getRequiredAttribute("name").getValue();
                SimTK::Xml::Element bodyFrameNode("FrameGeometry");
                bodyFrameNode.setAttributeValue("name", bodyName + "_body_frame");
                XMLDocument::addConnector(bodyFrameNode, "Connector_Frame_", "frame", bodyName);
                SimTK::Xml::Element appearanceNode("Appearance");
                SimTK::Xml::Element frameRepresentation("representation");
                frameRepresentation.setValue("0");
                appearanceNode.insertNodeAfter(appearanceNode.element_end(), frameRepresentation);
                bodyFrameNode.insertNodeAfter(bodyFrameNode.element_end(), appearanceNode);
                SimTK::Xml::element_iterator geomSetIter = aNode.element_begin("geometry");
                if (geomSetIter != aNode.element_end()){
                    geomSetIter->insertNodeAfter(geomSetIter->node_end(), bodyFrameNode);
                 }
            }
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
}

void Body::convertDisplayGeometryToGeometryXML(SimTK::Xml::Element& bodyNode,
    const SimTK::Vec3& outerScaleFactors, const SimTK::Vec6& outerTransform, 
    SimTK::Xml::Element& geomSetElement) const
{
    std::string bodyName = bodyNode.getRequiredAttribute("name").getValue();

    SimTK::Xml::element_iterator objectsIter = geomSetElement.element_begin("objects");

    if (objectsIter != geomSetElement.element_end()){
        SimTK::Xml::Element geometrySetNode("geometry");
        bodyNode.insertNodeAfter(bodyNode.element_end(), geometrySetNode);

        SimTK::Xml::element_iterator displayGeomIter = objectsIter->element_begin("DisplayGeometry");
        int counter = 1;
        while (displayGeomIter != objectsIter->element_end()){
            // Create a <Mesh> Element and populate it
            SimTK::Xml::Element meshNode("Mesh");
            std::string geomName = bodyName + "_geom_" + to_string(counter);
            meshNode.setAttributeValue("name", geomName);
            // geometry_file
            std::string geomFile = "";
            SimTK::Xml::element_iterator geomFileIter = displayGeomIter->element_begin("geometry_file");
            if (geomFileIter != displayGeomIter->element_end()){
                geomFile = geomFileIter->getValueAs<SimTK::String>();
            }
            // transform
            SimTK::Vec6 localXform(0.);
            SimTK::Xml::element_iterator localXformIter = displayGeomIter->element_begin("transform");
            if (localXformIter != displayGeomIter->element_end()){
                localXform = localXformIter->getValueAs<SimTK::Vec6>();
            }
            if (localXform.norm() > SimTK::Eps){
                // Create a Frame
                std::string frameName = bodyName + "_Frame_" + to_string(counter);
                SimTK::Xml::Element frameNode("frame_name", frameName);
                meshNode.insertNodeAfter(meshNode.element_end(), frameNode);

                SimTK::Xml::Element modelNode = bodyNode;
                do {
                modelNode = modelNode.getParentElement();
                SimTK::String edump;
                modelNode.writeToString(edump);
                } while (modelNode.getElementTag() != "Model" && !modelNode.isTopLevelNode());

                SimTK::Xml::element_iterator frameSetIter = modelNode.element_begin("FrameSet");
                SimTK::Xml::element_iterator frameSetObjectsIter;
                if (frameSetIter != modelNode.element_end()){
                    frameSetObjectsIter = frameSetIter->element_begin("objects");
                }
                else {
                    SimTK::Xml::Element frameSetNode("FrameSet");
                    modelNode.insertNodeAfter(modelNode.element_end(), frameSetNode);
                    SimTK::Xml::Element frameSetObjectsNode("objects");
                    frameSetNode.insertNodeAfter(frameSetNode.element_end(), frameSetObjectsNode);
                    frameSetObjectsIter = frameSetNode.element_begin("objects");
                }
                createFrameForXform(frameSetObjectsIter, frameName, localXform, bodyName);
                                
                XMLDocument::addConnector(meshNode, "Connector_Frame_", "frame", frameName);
            }
            else
                XMLDocument::addConnector(meshNode, "Connector_Frame_", "frame", bodyName);
            // scale_factor
            SimTK::Vec3 localScale(1.);
            SimTK::Xml::element_iterator localScaleIter = displayGeomIter->element_begin("scale_factors");
            if (localScaleIter != displayGeomIter->element_end()){
                localScale = localScaleIter->getValueAs<SimTK::Vec3>();
            }
            // Now compose scale factors and xforms and create new node to insert into bodyNode
             SimTK::Xml::Element meshFileNode("mesh_file", geomFile);
             std::stringstream localScaleStr;
             localScaleStr << localScale[0] * outerScaleFactors[0] << " " << localScale[1] * outerScaleFactors[1] 
                 << " " << localScale[2] * outerScaleFactors[2];
             SimTK::Xml::Element scaleFactorsNode("scale_factors", localScaleStr.str());
             meshNode.insertNodeAfter(meshNode.element_end(), scaleFactorsNode);
             meshNode.insertNodeAfter(meshNode.element_end(), meshFileNode);
             SimTK::Xml::Element appearanceNode("Appearance");
             // Move color and opacity under Appearance
             SimTK::Xml::element_iterator colorIter = displayGeomIter->element_begin("color");
             if (colorIter != displayGeomIter->element_end()){
                 appearanceNode.insertNodeAfter(appearanceNode.element_end(), displayGeomIter->removeNode(colorIter));
             }
             SimTK::Xml::element_iterator opacityIter = displayGeomIter->element_begin("opacity");
             if (opacityIter != displayGeomIter->element_end()){
                 appearanceNode.insertNodeAfter(appearanceNode.element_end(), displayGeomIter->removeNode(opacityIter));
             }
             SimTK::Xml::element_iterator reprIter = displayGeomIter->element_begin("display_preference");
             if (reprIter != displayGeomIter->element_end()){
                 reprIter->setElementTag("representation");
                 if (reprIter->getValue() == "4"){
                     // Enum changed to go 0-3 instead of 0-4
                     SimTK::String rep = "3";
                     reprIter->setValue(rep);
                 }
                 appearanceNode.insertNodeAfter(appearanceNode.element_end(), displayGeomIter->removeNode(reprIter));
             }
             meshNode.insertNodeAfter(meshNode.element_end(), appearanceNode);
             // Insert Mesh into parent
             geometrySetNode.insertNodeAfter(geometrySetNode.element_end(), meshNode);
             displayGeomIter++;
             counter++;
        }
    }
}
// This private method creates a frame in the owner model document with passed in name and content relative to bodyName
void Body::createFrameForXform(const SimTK::Xml::element_iterator& frameSetIter, const std::string& frameName, const SimTK::Vec6& localXform, const std::string& bodyName) const
{
    SimTK::Xml::Element frameNode("PhysicalOffsetFrame");
    frameNode.setAttributeValue("name", frameName);
    stringstream ss;
    ss << localXform[3] << " " << localXform[4] << " " << localXform[5];
    SimTK::Xml::Element translationNode("translation", ss.str());
    ss.clear();
    ss << localXform[0] << " " << localXform[1] << " " << localXform[2];
    SimTK::Xml::Element orientationNode("rotation", ss.str());
    frameNode.insertNodeAfter(frameNode.element_end(), translationNode);
    frameNode.insertNodeAfter(frameNode.element_end(), orientationNode);
    frameSetIter->insertNodeAfter(frameSetIter->element_end(), frameNode);
    XMLDocument::addConnector(frameNode, "Connector_Frame_", "parent", bodyName);

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
