/* -------------------------------------------------------------------------- *
 *                   OpenSim:  StatesDocument.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2022-2024 Stanford University and the Authors               *
 * Author(s):  F. C. Anderson                                                 *
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
#include "StatesDocument.h"

using namespace SimTK;
using namespace SimTK::Xml;
using namespace std;
using namespace OpenSim;
using std::cout;

namespace OpenSim {

// Anonymous namespace to ensure local linkage
namespace {

//-----------------------------------------------------------------------------
// Local utility methods for use with class StatesDocument
//-----------------------------------------------------------------------------
struct SDocUtil {

    //_________________________________________________________________________
    template<class T>
    static
    void
    appendVarElt(const string& path, const string& tag, const string& type,
        const Array_<T>& valArr, Element& parent, int precision)
    {
        // Create the variable element.
        Element varElt(tag);
        varElt.setAttributeValue("path", path);
        varElt.setAttributeValue("type", type);

        // Append the variable element
        varElt.setValueAs<Array_<T>>(valArr, precision);
        parent.appendNode(varElt);
    }
    //_________________________________________________________________________
    template<class T>
    inline
    static
    void
    getEltValue(const string& path, size_t expectedSize,
        Element& varElt, Array_<T>& vArr)
    {
        // Interpret the element value
        varElt.getValueAs<Array_<T>>(vArr);

        // Check the size
        size_t n = vArr.size();
        SimTK_ASSERT3_ALWAYS(n == expectedSize,
            "Found %d values in the element for %s, but there should be %d",
            n, path.c_str(), expectedSize);
    }
    //_________________________________________________________________________
    template<class T>
    inline
    static
    void
    initializeStatesForStateVariable(Element& varElt, const Model& model,
        const string& path, Array_<State> & traj)
    {
        // Interpret the element an array of type T
        Array_<T> vArr;
        getEltValue(path, traj.size(), varElt, vArr);

        // Set variable in the States trajectory
        model.setStateVariableTrajectory<T>(path, vArr, traj);
    }
    //_________________________________________________________________________
    template<class T>
    inline
    static
    void
    initializeStatesForDiscreteVariable(Element& varElt, const Model& model,
        const string& path, Array_<State> & traj)
    {
        // Interpret the element an array of type T
        Array_<T> vArr;
        getEltValue(path, traj.size(), varElt, vArr);

        // Set variable in the States trajectory
        model.setDiscreteVariableTrajectory<T>(path, vArr, traj);
    }
    //_________________________________________________________________________
    template<class T>
    inline
    static
    void
    initializeStatesForModelingOption(Element& varElt, const Model& model,
        const string& path, Array_<State> & traj)
    {
        // Interpret the Element value
        Array_<T> vArr;
        varElt.getValueAs<Array_<T>>(vArr);

        // Check the sizes.
        size_t n = vArr.size();
        SimTK_ASSERT2_ALWAYS(n == traj.size(),
            "Found %d values. Should match nTime = %d values.",
            n, traj.size());

        // Set variable in the States trajectory
        model.setModelingOptionTrajectory<T>(path, vArr, traj);
    }
};

} // End anonymous namespace
} // End OpenSim namespace

// Note that the methods below are still in the OpenSim namespace.
// That namespace declaration is taken care of in the .h file.

//-----------------------------------------------------------------------------
// Construction
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
StatesDocument::
StatesDocument(const Model& model, const Array_<State>& trajectory,
    const String& note, int p)
{
    this->note = note;
    this->precision = clamp(1, p, SimTK::LosslessNumDigitsReal);
    formDoc(model, trajectory);
}

//-----------------------------------------------------------------------------
// Serialize
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
StatesDocument::
serialize(const SimTK::String& filename) {
    doc.writeToFile(filename);
}
//_____________________________________________________________________________
void
StatesDocument::
formDoc(const Model& model, const Array_<State>& traj) {
    formRootElement(model, traj);
    formNoteElement(model, traj);
    formTimeElement(model, traj);
    formContinuousElement(model, traj);
    formDiscreteElement(model, traj);
    formModelingElement(model, traj);
}
//_____________________________________________________________________________
void
StatesDocument::
formRootElement(const Model& model, const Array_<State>& traj) {
    // Set the tag of the root element and get an iterator to it.
    doc.setRootTag("ostates");
    Element rootElt = doc.getRootElement();

    // Insert a comment at the top level, just before the root node.
    string info = "OpenSim States Document (Version: ";
    info += std::to_string(model.getDocumentFileVersion());
    info += ")";
    Xml::Comment comment(info);
    Xml::node_iterator root_it = doc.node_begin(Xml::ElementNode);
    doc.insertTopLevelNodeBefore(root_it, comment);

    // Date and time
    const std::time_t now = std::time(nullptr);
    const char *localeName = "C";
    std::locale::global(std::locale(localeName));
    char buf[64];
    strftime(buf, sizeof buf, "%a %b %e %Y %H:%M:%S %Z", std::localtime(&now));

    // Add attributes to the root node
    rootElt.setAttributeValue("model", model.getName());
    rootElt.setAttributeValue("nTime", std::to_string(traj.size()));
    rootElt.setAttributeValue("precision", std::to_string(precision));
    rootElt.setAttributeValue("date", buf);
}
//_____________________________________________________________________________
void
StatesDocument::
formNoteElement(const Model& model, const Array_<State>& traj) {
    Element noteElt = Element("note");
    Element rootElt = doc.getRootElement();
    rootElt.appendNode(noteElt);
    noteElt.setValue(note);
}
//_____________________________________________________________________________
void
StatesDocument::
formTimeElement(const Model& model, const Array_<State>& traj) {
    // Form time element.
    Element timeElt = Element("time");
    Element rootElt = doc.getRootElement();
    rootElt.appendNode(timeElt);

    // Get time values from the StatesTrajectory
    int n = (int)traj.size();
    SimTK::Array_<double> time(n);
    for (int i = 0; i < n; ++i) {
        time[i] = traj[i].getTime();
    }

    // Set the text value on the element
    timeElt.setValueAs<Array_<double>>(time, precision);
}
//_____________________________________________________________________________
void
StatesDocument::
formContinuousElement(const Model& model, const Array_<State>& traj) {
    // Form continuous element.
    Element contElt = Element("continuous");
    Element rootElt = doc.getRootElement();
    rootElt.appendNode(contElt);

    // Get a list of all state variables names from the model.
    OpenSim::Array<std::string> paths = model.getStateVariableNames();

    // Loop over the names.
    // Get the vector of values of each and append as a child element.
    int n = paths.getSize();
    for (int i = 0; i < n; ++i) {
        Array_<double> val;
        model.getStateVariableTrajectory<double>(paths[i], traj, val);
        SDocUtil::appendVarElt<double>(paths[i], "variable", "double",
            val, contElt, precision);
    }
}
//_____________________________________________________________________________
void
StatesDocument::
formDiscreteElement(const Model& model, const Array_<State>& traj) {
    // Form discrete element.
    Element discreteElt = Element("discrete");
    Element rootElt = doc.getRootElement();
    rootElt.appendNode(discreteElt);

    // Get a list of all discrete variable names from the model.
    OpenSim::Array<std::string> paths = model.getDiscreteVariableNames();

    // Loop over the names.
    // Get the vector of values for each and append as a child element.
    int n = paths.getSize();
    for (int i = 0; i < n; ++i) {
        // Get a single discrete variable so that its type can be discerned
        const AbstractValue &v =
            model.getDiscreteVariableAbstractValue(traj[0], paths[i]);

        // Append the vector according to type
        if (SimTK::Value<bool>::isA(v)) {
            Array_<bool> vArr;
            model.getDiscreteVariableTrajectory<bool>(
                paths[i], traj, vArr);
            SDocUtil::appendVarElt<bool>(paths[i], "variable", "bool",
                vArr, discreteElt, precision);
        }
        else if(SimTK::Value<int>::isA(v)) {
            Array_<int> vArr;
            model.getDiscreteVariableTrajectory<int>(
                paths[i], traj, vArr);
            SDocUtil::appendVarElt<int>(paths[i], "variable", "int",
                vArr, discreteElt, precision);
        }
        else if(SimTK::Value<float>::isA(v)) {
            Array_<float> vArr;
            model.getDiscreteVariableTrajectory<float>(
                paths[i], traj, vArr);
            SDocUtil::appendVarElt<float>(paths[i], "variable", "float",
                vArr, discreteElt, precision);
        }
        else if(SimTK::Value<double>::isA(v)) {
            Array_<double> vArr;
            model.getDiscreteVariableTrajectory<double>(
                paths[i], traj, vArr);
            SDocUtil::appendVarElt<double>(paths[i], "variable", "double",
                vArr, discreteElt, precision);
        }
        else if(SimTK::Value<Vec2>::isA(v)) {
            Array_<Vec2> vArr;
            model.getDiscreteVariableTrajectory<Vec2>(
                paths[i], traj, vArr);
            SDocUtil::appendVarElt<Vec2>(paths[i], "variable", "Vec2",
                vArr, discreteElt, precision);
        }
        else if(SimTK::Value<Vec3>::isA(v)) {
            Array_<Vec3> vArr;
            model.getDiscreteVariableTrajectory<Vec3>(
                paths[i], traj, vArr);
            SDocUtil::appendVarElt<Vec3>(paths[i], "variable", "Vec3",
                vArr, discreteElt, precision);
        }
        else if(SimTK::Value<Vec4>::isA(v)) {
            Array_<Vec4> vArr;
            model.getDiscreteVariableTrajectory<Vec4>(
                paths[i], traj, vArr);
            SDocUtil::appendVarElt<Vec4>(paths[i], "variable", "Vec4",
                vArr, discreteElt, precision);
        }
        else if(SimTK::Value<Vec5>::isA(v)) {
            Array_<Vec5> vArr;
            model.getDiscreteVariableTrajectory<Vec5>(
                paths[i], traj, vArr);
            SDocUtil::appendVarElt<Vec5>(paths[i], "variable", "Vec5",
                vArr, discreteElt, precision);
        }
        else if(SimTK::Value<Vec6>::isA(v)) {
            Array_<Vec6> vArr;
            model.getDiscreteVariableTrajectory<Vec6>(
                paths[i], traj, vArr);
            SDocUtil::appendVarElt<Vec6>(paths[i], "variable", "Vec6",
                vArr, discreteElt, precision);
        }
        else {
            string msg = "Unrecognized type: " + v.getTypeName();
            SimTK_ASSERT(false, msg.c_str());
        }
    }

}
//_____________________________________________________________________________
void
StatesDocument::
formModelingElement(const Model& model, const Array_<State>& traj) {
    // Form continuous element.
    Element modelingElt = Element("modeling");
    Element rootElt = doc.getRootElement();
    rootElt.appendNode(modelingElt);

    // Get a list of all modeling option names from the model.
    OpenSim::Array<std::string> paths = model.getModelingOptionNames();

    // Loop over the names.
    // Get the vector of values of each and append as a child element.
    int n = paths.getSize();
    for (int i = 0; i < n; ++i) {
        Array_<int> val;
        model.getModelingOptionTrajectory<int>(paths[i], traj, val);
        SDocUtil::appendVarElt<int>(paths[i], "option", "int",
            val, modelingElt, precision);
    }
}


//-----------------------------------------------------------------------------
// Deserialize
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
StatesDocument::
deserialize(const Model& model, Array_<State>& traj) {
    checkDocConsistencyWithModel(model);
    prepareStatesTrajectory(model, traj);
    initializeTime(traj);
    initializeContinuousVariables(model, traj);
    initializeDiscreteVariables(model, traj);
    initializeModelingOptions(model, traj);
}
//_____________________________________________________________________________
void
StatesDocument::
checkDocConsistencyWithModel(const Model& model) {
    // At this point, only the model name is checked here.
    // Many other aspects are checked for consistency than just the model
    // name. Those are more easily checked as the doc is parced.

    // Check that name of the model in the doc matches the name of the model'.
    Element rootElt = doc.getRootElement();
    Attribute modelNameAttr = rootElt.getOptionalAttribute("model");
    SimTK_ASSERT1(modelNameAttr.isValid(),
        "The 'model' attribute of the root element was not found in file %s.",
        filename.c_str());
    const SimTK::String& modelName = modelNameAttr.getValue();
    if (modelName != model.getName()) {
        SimTK::String msg = "The model name (" + modelName + ")";
        msg += " in states document " + filename + " does not match";
        msg += " the name of the OpenSim model (" + model.getName() + ")";
        msg += " for which the states are being deserialized.";
        SimTK_ASSERT_ALWAYS(false, msg.c_str());
    }

}
//_____________________________________________________________________________
void
StatesDocument::
prepareStatesTrajectory(const Model& model, Array_<State>& traj) {
    // Create a local copy of the Model and get a default State.
    Model localModel(model);
    SimTK::State state = localModel.initSystem();

    // How many State objects should there be?
    // The number of objects needs to be the same as the number of time stamps.
    // Each State object has a time field, which will be set in
    // initializeTime().
    Element rootElt = doc.getRootElement();
    Attribute nTimeAttr = rootElt.getOptionalAttribute("nTime");
    int nTime;
    bool success = nTimeAttr.getValue().tryConvertTo<int>(nTime);
    SimTK_ASSERT_ALWAYS(success,
        "Unable to acquire nTime from root element.");
    SimTK_ASSERT1_ALWAYS(nTime > 0,
        "Root element attribute numStateObjects=%d; should be > 0.", nTime);

    // Append State objects
    for (int i=0; i < nTime; ++i) traj.emplace_back(state);
}
//_____________________________________________________________________________
void
StatesDocument::
initializeNote() {
    // Find the element
    Element rootElt = doc.getRootElement();
    Array_<Element> noteElts = rootElt.getAllElements("note");

    // Check the number of note elements found. Should be 1.
    if (noteElts.size() == 0) {
        this->note = "";
    }
    else if (noteElts.size() > 1) {
        cout << "StatesDocument: More than 1 `note` element found; ";
        cout << "using just the first one." << endl;
    }

    // Get the value
    this->note = noteElts[0].getValue();
}
//_____________________________________________________________________________
void
StatesDocument::
initializePrecision() {
    // Find the element
    Element rootElt = doc.getRootElement();
    Attribute precisionAttr = rootElt.getOptionalAttribute("precision");
    int p;
    bool success = precisionAttr.getValue().tryConvertTo<int>(p);
    SimTK_ASSERT_ALWAYS(success,
        "Unable to acquire the precision from the root element.");
    this->precision = clamp(1, p, SimTK::LosslessNumDigitsReal);
}
//_____________________________________________________________________________
void
StatesDocument::
initializeTime(Array_<State>& traj) {
    // Find the element
    Element rootElt = doc.getRootElement();
    Array_<Element> timeElts = rootElt.getAllElements("time");

    // Check the number of time elements found. Should be 1.
    SimTK_ASSERT1_ALWAYS(timeElts.size() == 1,
        "%d time elements found. Only 1 should be found.", timeElts.size());

    // Get the values
    Array_<double> timeArr;
    timeElts[0].getValueAs<Array_<double>>(timeArr);

    // Check the size of the time array.
    size_t n = traj.size();
    SimTK_ASSERT2_ALWAYS(n == traj.size(),
        "Found %d time values. Should match numStateObjects = %d",
        n, traj.size());

    // Initialize the State objects
    for (size_t i = 0; i < n; ++i) traj[i].setTime(timeArr[i]);
}
//_____________________________________________________________________________
void
StatesDocument::
initializeContinuousVariables(const Model& model, SimTK::Array_<State>& traj) {
    // Find the 'continuous' element
    SimTK::String tag = "continuous";
    Element rootElt = doc.getRootElement();
    Array_<Element> contElts = rootElt.getAllElements(tag);
    SimTK_ASSERT1_ALWAYS(contElts.size() == 1,
        "Found %d elements with tag 'continuous'. Should only be 1.",
        contElts.size());

    // Find all the child 'variable' elements
    SimTK::String childTag = "variable";
    Array_<Element> varElts = contElts[0].getAllElements(childTag);

    // Check that the number matches the number of continous variables.
    // In OpenSim, a continuous variable is referred to as a StateVariable.
    OpenSim::Array<std::string> varNames = model.getStateVariableNames();
    int n = varElts.size();
    int m = varNames.size();
    SimTK_ASSERT2_ALWAYS(n == m,
        "Found %d continuous variable elements. Should be %d.", n, m);

    // Loop over the variable elements
    SimTK::Array_<double> varArr;
    for (int i = 0; i < n; ++i) {
        // type
        Attribute typeAttr = varElts[i].getOptionalAttribute("type");
        const SimTK::String &type = typeAttr.getValue();

        // path
        Attribute pathAttr = varElts[i].getOptionalAttribute("path");
        const SimTK::String path = pathAttr.getValue();

        // Switch based on the type.
        // Type double is expected for continuous variable elements.
        if (type == "double") {
            SDocUtil::initializeStatesForStateVariable<double>(varElts[i],
                model, path, traj);
        }
        else {
            string msg = "Unrecognized type: " + type;
            SimTK_ASSERT(false, msg.c_str());
        }
    }
}
//_____________________________________________________________________________
void
StatesDocument::
initializeDiscreteVariables(const Model& model, SimTK::Array_<State>& traj) {
    Element rootElt = doc.getRootElement();
    Array_<Element> discElts = rootElt.getAllElements("discrete");
    SimTK_ASSERT1_ALWAYS(discElts.size() == 1,
        "Found %d elements with tag 'discrete'. Only 1 should be found.",
        discElts.size());

    // Find all the child 'variable' elements
    SimTK::String childTag = "variable";
    Array_<Element> varElts = discElts[0].getAllElements(childTag);

    // Check that # children matches the number of discrete variables.
    OpenSim::Array<std::string> varNames = model.getDiscreteVariableNames();
    int n = varElts.size();
    int m = varNames.size();
    SimTK_ASSERT2_ALWAYS(n == m,
        "Found %d discrete variable elements. Should be %d.", n, m);

    // Loop over the variable elements
    for (int i = 0; i < n; ++i) {
        // type
        Attribute typeAttr = varElts[i].getOptionalAttribute("type");
        const SimTK::String &type = typeAttr.getValue();

        // path
        Attribute pathAttr = varElts[i].getOptionalAttribute("path");
        const SimTK::String path = pathAttr.getValue();

        // Switch based on the type
        // Append the vector according to type
        if (type == "bool") {
            SDocUtil::initializeStatesForDiscreteVariable<bool>(varElts[i],
                model, path, traj);
        }
        else if(type == "int") {
            SDocUtil::initializeStatesForDiscreteVariable<int>(varElts[i],
                model, path, traj);
        }
        else if(type == "float") {
            SDocUtil::initializeStatesForDiscreteVariable<float>(varElts[i],
                model, path, traj);
        }
        else if(type == "double") {
            SDocUtil::initializeStatesForDiscreteVariable<double>(varElts[i],
                model, path, traj);
        }
        else if(type == "Vec2") {
            SDocUtil::initializeStatesForDiscreteVariable<Vec2>(varElts[i],
                model, path, traj);
        }
        else if(type == "Vec3") {
            SDocUtil::initializeStatesForDiscreteVariable<Vec3>(varElts[i],
                model, path, traj);
        }
        else if(type == "Vec4") {
            SDocUtil::initializeStatesForDiscreteVariable<Vec4>(varElts[i],
                model, path, traj);
        }
        else if(type == "Vec5") {
            SDocUtil::initializeStatesForDiscreteVariable<Vec5>(varElts[i],
                model, path, traj);
        }
        else if(type == "Vec6") {
            SDocUtil::initializeStatesForDiscreteVariable<Vec6>(varElts[i],
                model, path, traj);
        }
        else {
            string msg = "Unrecognized type: " + type;
            SimTK_ASSERT(false, msg.c_str());
        }
    }
}
//_____________________________________________________________________________
void
StatesDocument::
initializeModelingOptions(const Model& model, SimTK::Array_<State>& traj) {
    // Find the element
    Element rootElt = doc.getRootElement();
    Array_<Element> modlElts = rootElt.getAllElements("modeling");
    SimTK_ASSERT1_ALWAYS(modlElts.size() == 1,
        "%d modeling elements found. Only 1 should be found.",
        modlElts.size());
    Element modlElt = modlElts[0];

    // Find all the child 'variable' elements.
    SimTK::String childTag = "option";
    Array_<Element> varElts = modlElts[0].getAllElements(childTag);

    // Check that the number matches the number of continous variables.
    // In OpenSim, a continuous variable is referred to as a StateVariable.
    OpenSim::Array<std::string> varNames = model.getModelingOptionNames();
    int n = varElts.size();
    int m = varNames.size();
    SimTK_ASSERT2_ALWAYS(n == m,
        "Found %d modeling option elements. Should be %d.", n, m);

    // Loop over the modeling options
    SimTK::Array_<double> varArr;
    for (int i = 0; i < n; ++i) {
        // type
        Attribute typeAttr = varElts[i].getOptionalAttribute("type");
        const SimTK::String &type = typeAttr.getValue();

        // path
        Attribute pathAttr = varElts[i].getOptionalAttribute("path");
        const SimTK::String path = pathAttr.getValue();

        // Switch based on the type.
        // Type int is expected for modeling option elements.
        if (type == "int") {
            SDocUtil::initializeStatesForModelingOption<int>(varElts[i],
                model, path, traj);
        }
        else {
            string msg = "Unrecognized type: " + type;
            SimTK_ASSERT(false, msg.c_str());
        }
    }
}
