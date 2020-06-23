/* -------------------------------------------------------------------------- *
 *                   OpenSim:  AnalysisPlugin_Template.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth, Tim Dorn                          *
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "AnalysisPlugin_Template.h"

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND SETUP
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor
 */
AnalysisPlugin_Template::AnalysisPlugin_Template() : Analysis()
{
    setNull();
    constructProperties();
}



//_____________________________________________________________________________
/**
 * SetNull()
 */
void AnalysisPlugin_Template::
setNull()
{

    _bodyIndices = Array<int>();
    _bodypos = Array<double>();
}


//_____________________________________________________________________________
/*
 * Connect properties to local pointers.
 */
void AnalysisPlugin_Template::
constructProperties()
{
    Array<string> defaultBodyNames;
    defaultBodyNames.append("all");
    constructProperty_body_names(defaultBodyNames);

    // Here are some examples of other constructing other scalar property types.
    // Uncomment them as you need them.
    // ------------------------------------------------------
    //constructProperty_string_property("defaultString");
    //constructProperty_int_property(10);
    //constructProperty_bool_property(true);
    //constructProperty_double_property(1.5);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void AnalysisPlugin_Template::
constructDescription()
{
    string descrip;

    descrip = "\nThis file contains body positions (of origin) and orientations.\n";
    descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";
    descrip += "\nIf the header above contains a line with ";
    descrip += "'inDegrees', this indicates whether rotational values ";
    descrip += "are in degrees (yes) or radians (no).";
    descrip += "\n\n";

    setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the output results.
 *
 * For analyses that run during a simulation, the first column is almost
 * always time.  For the purpose of example, the code below adds labels
 * appropriate for recording the translation and orientation of each
 * body in the model.
 *
 * This method needs to be called as necessary to update the column labels.
 */
void AnalysisPlugin_Template::
constructColumnLabels()
{
    if(_model==NULL) return;

    Array<string> labels;
    labels.append("time");

    const BodySet& bodySet = _model->getBodySet();

    if(get_body_names(0) == "all"){
        _bodyIndices.setSize(bodySet.getSize());
        // Get indices of all the bodies.
        for(int j=0;j<bodySet.getSize();j++)
            _bodyIndices[j]=j;
    }
    else{
        _bodyIndices.setSize(getProperty_body_names().size());
        // Get indices of just the bodies listed.
        for(int j=0;j<getProperty_body_names().size();j++)
            _bodyIndices[j]=bodySet.getIndex(get_body_names(j));
    }

    //Do the analysis on the bodies that are in the indices list
    for(int i=0; i<_bodyIndices.getSize(); i++) {
        const Body& body = bodySet.get(_bodyIndices[i]);
        labels.append(body.getName() + "_X");
        labels.append(body.getName() + "_Y");
        labels.append(body.getName() + "_Z");
        labels.append(body.getName() + "_Ox");
        labels.append(body.getName() + "_Oy");
        labels.append(body.getName() + "_Oz");
    }

    setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Set up storage objects.
 *
 * In general, the storage objects in your analysis are used to record
 * the results of your analysis and write them to file.  You will often
 * have a number of storage objects, each for recording a different
 * kind of result.
 */
void AnalysisPlugin_Template::
setupStorage()
{
    // Positions
    _storePos.reset(0);
    _storePos.setName("Positions");
    _storePos.setDescription(getDescription());
    _storePos.setColumnLabels(getColumnLabels());
}


//_____________________________________________________________________________
/**
 * Set the model for which this analysis is to be run.
 *
 * Sometimes the model on which an analysis should be run is not available
 * at the time an analysis is created.  Or, you might want to change the
 * model.  This method is used to set the model on which the analysis is
 * to be run.
 *
 * @param aModel Model pointer
 */
void AnalysisPlugin_Template::
setModel(Model& aModel)
{
    // SET THE MODEL IN THE BASE CLASS
    Super::setModel(aModel);

    // UPDATE VARIABLES IN THIS CLASS
    constructDescription();
    constructColumnLabels();
    setupStorage();

    //Setup size of work array to hold body positions
    int numBodies = _bodyIndices.getSize();
    _bodypos.setSize(6*numBodies);
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute and record the results.
 *
 * This method, for the purpose of example, records the position and
 * orientation of each body in the model.  You will need to customize it
 * to perform your analysis.
 *
 * @param aT Current time in the simulation.
 * @param aX Current values of the controls.
 * @param aY Current values of the states: includes generalized coords and speeds
 */
int AnalysisPlugin_Template::
record(const SimTK::State& s)
{
    // VARIABLES
    SimTK::Vec3 vec,angVec;
    double Mass = 0.0;

    // GROUND BODY
    const Ground& ground = _model->getGround();

    // POSITION
    const BodySet& bodySet = _model->getBodySet();

    for(int i=0;i<_bodyIndices.getSize();i++) {

        const Body& body = bodySet.get(_bodyIndices[i]);

        // GET POSITIONS AND EULER ANGLES
        vec = body.getPositionInGround(s);
        angVec = body.getTransformInGround(s).R()
            .convertThreeAxesRotationToThreeAngles(SimTK::BodyRotationSequence,
                                                   SimTK::XAxis,
                                                   SimTK::YAxis,
                                                   SimTK::ZAxis);

        // CONVERT TO DEGREES?
        if(getInDegrees()) {
            angVec *= SimTK_RADIAN_TO_DEGREE;
        }           

        // FILL KINEMATICS ARRAY
        int I=6*i;
        memcpy(&_bodypos[I],&vec[0],3*sizeof(double));
        memcpy(&_bodypos[I+3],&angVec[0],3*sizeof(double));
    }
    _storePos.append(s.getTime(),_bodypos.getSize(),&_bodypos[0]);

    // VELOCITY 

    // ACCELERATIONS


    return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration in
 * Model::integBeginCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int AnalysisPlugin_Template::begin(const SimTK::State& s)
{
    if(!proceed()) return(0);

    // RESET STORAGE
    _storePos.reset(s.getTime());  //->reset(s.getTime());

    // RECORD
    int status = 0;
    if(_storePos.getSize()<=0) {
        status = record(s);
    }

    return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called in
 * Model::integStepCallback(), which has the same argument list.
 *
 * @param aXPrev Controls at the beginining of the current time step.
 * @param aYPrev States at the beginning of the current time step.
 * @param aYPPrev Pseudo states at the beginning of the current time step.
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int AnalysisPlugin_Template::
step(const SimTK::State& s, int stepNumber)
{
    if(!proceed(stepNumber)) return(0);

    record(s);

    return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * Model::integEndCallback() and has the same argument list.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 *
 * @return -1 on error, 0 otherwise.
 */
int AnalysisPlugin_Template::
end(const SimTK::State&s)
{
    if(!proceed()) return(0);

    record(s);

    return(0);
}




//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int AnalysisPlugin_Template::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    // POSITIONS
    //_storePos.scaleTime(_model->getTimeNormConstant());
    Storage::printResult(&_storePos,aBaseName+"_"+getName()+"_pos",aDir,aDT,aExtension);

    // VELOCITIES


    // ACCELERATIONS

    return(0);
}


