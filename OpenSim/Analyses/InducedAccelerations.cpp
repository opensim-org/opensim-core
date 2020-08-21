/* -------------------------------------------------------------------------- *
 *                     OpenSim:  InducedAccelerations.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ExternalForce.h>
#include "InducedAccelerations.h"

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTANTS
//=============================================================================
#define CENTER_OF_MASS_NAME string("center_of_mass")

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 * Delete any variables allocated using the "new" operator.  You will not
 * necessarily have any of these.
 */
InducedAccelerations::~InducedAccelerations()
{
    delete &_coordSet;
    delete &_bodySet;
    delete _storeConstraintReactions;
}
//_____________________________________________________________________________
/*
 * Construct an InducedAccelerations instance.
 *
 * @param aModel Model for which the analysis is to be run.
 */
InducedAccelerations::InducedAccelerations(Model *aModel) :
    Analysis(aModel),
    _coordSet(*new CoordinateSet()),
    _bodySet(*new BodySet()),
    _coordNames(_coordNamesProp.getValueStrArray()),
    _bodyNames(_bodyNamesProp.getValueStrArray()),
    _constraintSetProp(PropertyObj("", ConstraintSet())),
    _constraintSet((ConstraintSet&)_constraintSetProp.getValueObj()),
    _forceThreshold(_forceThresholdProp.getValueDbl()),
    _computePotentialsOnly(_computePotentialsOnlyProp.getValueBool()),
    _reportConstraintReactions(_reportConstraintReactionsProp.getValueBool())
{
    // make sure members point to NULL if not valid. 
    setNull();
    if(_model==NULL) return;

    // DESCRIPTION
    constructDescription();
}
//_____________________________________________________________________________
/*
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
InducedAccelerations::InducedAccelerations(const std::string &aFileName):
    Analysis(aFileName, false),
    _coordSet(*new CoordinateSet()),
    _bodySet(*new BodySet()),
    _coordNames(_coordNamesProp.getValueStrArray()),
    _bodyNames(_bodyNamesProp.getValueStrArray()),
    _constraintSetProp(PropertyObj("", ConstraintSet())),
    _constraintSet((ConstraintSet&)_constraintSetProp.getValueObj()),
    _forceThreshold(_forceThresholdProp.getValueDbl()),
    _computePotentialsOnly(_computePotentialsOnlyProp.getValueBool()),
    _reportConstraintReactions(_reportConstraintReactionsProp.getValueBool())
{
    setNull();

    // Read properties from XML
    updateFromXMLDocument();
}

// Copy constructor and virtual copy 
//_____________________________________________________________________________
/*
 * Copy constructor.
 *
 */
InducedAccelerations::InducedAccelerations(const InducedAccelerations &aInducedAccelerations):
    Analysis(aInducedAccelerations),
    _coordSet(*new CoordinateSet()),
    _bodySet(*new BodySet()),
    _coordNames(_coordNamesProp.getValueStrArray()),
    _bodyNames(_bodyNamesProp.getValueStrArray()),
    _constraintSetProp(PropertyObj("", ConstraintSet())),
    _constraintSet((ConstraintSet&)_constraintSetProp.getValueObj()),
    _forceThreshold(_forceThresholdProp.getValueDbl()),
    _computePotentialsOnly(_computePotentialsOnlyProp.getValueBool()),
    _reportConstraintReactions(_reportConstraintReactionsProp.getValueBool())
{
    setNull();
    // COPY TYPE AND NAME
    *this = aInducedAccelerations;
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/*
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
InducedAccelerations& InducedAccelerations::
operator=(const InducedAccelerations &aInducedAccelerations)
{
    // Base Class
    Analysis::operator=(aInducedAccelerations);

    // Member Variables
    _coordNames = aInducedAccelerations._coordNames;
    _bodyNames = aInducedAccelerations._bodyNames;
    _constraintSet = aInducedAccelerations._constraintSet;
    _forceThreshold = aInducedAccelerations._forceThreshold;
    _computePotentialsOnly = aInducedAccelerations._computePotentialsOnly;
    _reportConstraintReactions = aInducedAccelerations._reportConstraintReactions;
    _includeCOM = aInducedAccelerations._includeCOM;
    return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void InducedAccelerations::setNull()
{
    setAuthors("Ajay Seth");
    setupProperties();

    _forceThreshold = 6.00;
    _coordNames.setSize(0);
    _bodyNames.setSize(1);
    _bodyNames[0] = CENTER_OF_MASS_NAME;
    _computePotentialsOnly = false;
    _reportConstraintReactions = false;
    // Analysis does not own contents of these sets
    _coordSet.setMemoryOwner(false);
    _bodySet.setMemoryOwner(false);

    _storeConstraintReactions = NULL;
}
//_____________________________________________________________________________
/*
 * Set up the properties the analysis.
 */
void InducedAccelerations::setupProperties()
{
    _coordNamesProp.setName("coordinate_names");
    _coordNamesProp.setComment("Names of the coordinates for which to compute induced accelerations."
        "The key word 'All' indicates that the analysis should be performed for all coordinates.");
    _propertySet.append(&_coordNamesProp);

    _bodyNamesProp.setName("body_names");
    _bodyNamesProp.setComment("Names of the bodies for which to compute induced accelerations."
        "The key word 'All' indicates that the analysis should be performed for all bodies."
        "Use 'center_of_mass' to indicate the induced accelerations of the system center of mass.");
    _propertySet.append(&_bodyNamesProp);

    _constraintSetProp.setName("ConstraintSet");
    _constraintSetProp.setComment("Specify the constraints used to replace external forces applied "
        "to run the forward simulation. Currently, RollingOnSurfaceConstraint, PointConstraint "
        "and WeldConstraint are supported. There must be as many constraints listed as external "
        "forces applied to the model. Constraints must be between the body to which the external "
        "force is applied and ground.");
    _propertySet.append(&_constraintSetProp);

    _forceThresholdProp.setName("force_threshold");
    _forceThresholdProp.setComment("The minimum amount of external force (N) that is necessary to be replaced with a constraint.");
    _propertySet.append(&_forceThresholdProp);

    _computePotentialsOnlyProp.setName("compute_potentials_only");
    _computePotentialsOnlyProp.setComment("Only compute the potential (acceleration/force) of an actuator to accelerate the model.");
    _propertySet.append(&_computePotentialsOnlyProp);

    _reportConstraintReactionsProp.setName("report_constraint_reactions");
    _reportConstraintReactionsProp.setComment("Report individual contributions to constraint reactions in addition to accelerations.");
    _propertySet.append(&_reportConstraintReactionsProp);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the output files.
 */
void InducedAccelerations::constructDescription()
{
    string descrip;

    descrip = "\nThis file contains accelerations of coordinates or bodies.\n";
    descrip += "\nUnits are S.I. units (seconds, meters, Newtons, ...)";
    descrip += "\nIf the header above contains a line with ";
    descrip += "'inDegrees', this indicates whether rotational values ";
    descrip +=  "are in degrees (yes) or radians (no).";

    descrip += "\n\n";

    setDescription(descrip);
    assembleContributors();
}

//_____________________________________________________________________________
/**
 * Assemble the list of contributors for induced acceleration analysis
 */
void InducedAccelerations:: assembleContributors()
{
    Array<string> contribs;
    if (!_computePotentialsOnly)
        contribs.append("total");

    const Set<Actuator> &actuatorSet = _model->getActuators();

    //Do the analysis on the bodies that are in the indices list
    for(int i=0; i< actuatorSet.getSize(); i++) {
        contribs.append(actuatorSet.get(i).getName()) ;
    }
 
    contribs.append("gravity");
    contribs.append("velocity");

    _contributors = contribs;
}

/**
 * Construct column labels for the output results.
 *
 * For analyses that run during a simulation, the first column is 
 * always time.  For the purpose of example, the code below adds labels
 * for each contributor
 * This method needs to be called as necessary to update the column labels.
 */
Array<string> InducedAccelerations:: constructColumnLabelsForCoordinate()
{
    Array<string> labels;
    labels.append("time");
    labels.append(_contributors);

    return labels;
}

/**
 * Construct column labels for the body acceleration results.
 *
 * For analyses that run during a simulation, the first column is 
 * always time.  For the purpose of example, the code below adds labels
 * appropriate for recording the translation and orientation of the 
 * desired body.
 *
 * This method needs to be called as necessary to update the column labels.
 */
Array<string> InducedAccelerations:: constructColumnLabelsForBody()
{
    // Get the main headings for all the contributors
    Array<string> contributors = constructColumnLabelsForCoordinate();
    Array<string> labels;

    // first label is time not a contributor
    labels.append(contributors[0]);
    for(int i=1; i<contributors.getSize(); i++) {
        labels.append(contributors[i] + "_X");
        labels.append(contributors[i] + "_Y");
        labels.append(contributors[i] + "_Z");
        labels.append(contributors[i] + "_Ox");
        labels.append(contributors[i] + "_Oy");
        labels.append(contributors[i] + "_Oz");
    }
    return labels;
}

Array<string> InducedAccelerations:: constructColumnLabelsForCOM()
{
    // Get the main headings for all the contributors
    Array<string> contributors = constructColumnLabelsForCoordinate();
    Array<string> labels;

    // first label is time not a contributor
    labels.append(contributors[0]);
    for(int i=1; i<contributors.getSize(); i++) {
        // ADD CONTRIBUTORS TO THE ACCELERATION OF THE WHOLE BODY
        labels.append(contributors[i] + "_X");
        labels.append(contributors[i] + "_Y");
        labels.append(contributors[i] + "_Z");
    }
    return labels;
}

/**
 * Construct column labels for constraint reaction forces that correspond
 * to the induced accelerations for each contributor.
 *
 * For analyses that run during a simulation, the first column is 
 * always time.  For the purpose of example, the code below adds labels
 * appropriate for recording the translation and orientation of the 
 * desired body.
 *
 * This method needs to be called as necessary to update the column labels.
 */
Array<string> InducedAccelerations:: constructColumnLabelsForConstraintReactions()
{
    int nc = _constraintSet.getSize();
    // Get the main headings for all the contributors
    Array<string> contributors = constructColumnLabelsForCoordinate();
    Array<string> labels;
    Array< Array<std::string> > constraint_reaction_labels;

    for(int j=0; j<nc; j++){
        constraint_reaction_labels.append(_constraintSet[j].getRecordLabels());
    }

    // first label is time not a contributor
    labels.append(contributors[0]);
    for(int i=1; i<contributors.getSize(); i++) {
        for(int j=0; j<nc; j++){
            for(int k=0; k<constraint_reaction_labels[j].getSize(); k++){
                labels.append(contributors[i] + "_" + (constraint_reaction_labels[j])[k]);
            }
        }
    }
    return labels;
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
void InducedAccelerations::setupStorage()
{
    const CoordinateSet& modelCoordSet = _model->getCoordinateSet();
    int nc = _coordNames.getSize();

    // Get the indices of the coordinates we are interested in
    _storeInducedAccelerations.setSize(0);
    _coordSet.setSize(0);
    if(nc && (IO::Uppercase(_coordNames.get(0)) == "ALL")) {
        nc = modelCoordSet.getSize();
        for(int i=0; i<nc; i++)
            _coordSet.adoptAndAppend(&modelCoordSet.get(i));
    }
    else{
        for(int i=0; i<nc; i++){
            int index = modelCoordSet.getIndex(_coordNames[i]);
            if(index<0) throw Exception("InducedAcceleration: ERR- Could not find coordinate '"+_coordNames[i],__FILE__,__LINE__);
            _coordSet.adoptAndAppend(&modelCoordSet.get(index));
        }
    }

    // Setup storage object or each coordinate
    nc = _coordSet.getSize();
    _coordIndAccs.setSize(0);
    Array<string> coordAccLabels = constructColumnLabelsForCoordinate();
    for(int i=0; i<nc; i++){
        _storeInducedAccelerations.append(new Storage(1000));
        _storeInducedAccelerations[i]->setName(_coordSet.get(i).getName());
        _storeInducedAccelerations[i]->setDescription(getDescription());
        _storeInducedAccelerations[i]->setColumnLabels(coordAccLabels);
        _storeInducedAccelerations[i]->setInDegrees(getInDegrees());
        _coordIndAccs.append(new Array<double>(0, coordAccLabels.getSize()));
    }

    // Now get the bodies that we are interested, including system center of mass
    const BodySet& modelBodySet = _model->getBodySet();
    int nb = _bodyNames.getSize();

    _bodySet.setSize(0);

    if(nb && (IO::Uppercase(_bodyNames.get(0)) == "ALL")) {
        nb = modelBodySet.getSize();
        for(int i=0; i<nb; i++)
            _bodySet.adoptAndAppend(&modelBodySet.get(i));
    }
    else{
        for(int i=0; i<nb; i++){
            if(IO::Lowercase(_bodyNames.get(i)) == CENTER_OF_MASS_NAME)
                _includeCOM = true;
            else{
                int bi =  modelBodySet.getIndex(_bodyNames[i]);
                if(bi<0) throw Exception("InducedAcceleration: ERR- Could not find body '"+_bodyNames[i],__FILE__,__LINE__);
                _bodySet.adoptAndAppend(&modelBodySet.get(bi));
            }
        }
    }

    // Setup storage object for bodies
    nb = _bodySet.getSize();
    _bodyIndAccs.setSize(0);
    Array<string> bodyAccLabels = constructColumnLabelsForBody();
    for(int i=0; i<nb; i++){
        _storeInducedAccelerations.append(new Storage(1000));
        // cout << "Body " << i << " in _bodySet: " << _bodySet.get(i).getName() << endl;
        _storeInducedAccelerations[nc+i]->setName(_bodySet.get(i).getName());
        _storeInducedAccelerations[nc+i]->setDescription(getDescription());
        _storeInducedAccelerations[nc+i]->setColumnLabels(bodyAccLabels);
        _bodyIndAccs.append(new Array<double>(0, bodyAccLabels.getSize()));
    }

    if(_includeCOM){
        Array<string> comAccLabels = constructColumnLabelsForCOM();
        _comIndAccs.setSize(0);
        _storeInducedAccelerations.append(new Storage(1000, CENTER_OF_MASS_NAME));
        _storeInducedAccelerations[nc+nb]->setDescription(getDescription());
        _storeInducedAccelerations[nc+nb]->setColumnLabels(comAccLabels);
    }

    delete _storeConstraintReactions;
    if(_reportConstraintReactions){
        Array<string> constReactionLabels = constructColumnLabelsForConstraintReactions();
        _constraintReactions.setSize(0);
        _storeConstraintReactions = new Storage(1000, "induced_constraint_reactions");
        _storeConstraintReactions->setDescription(getDescription());
        _storeConstraintReactions->setColumnLabels(constReactionLabels);
    }

    _coordSet.setMemoryOwner(false);
    _bodySet.setMemoryOwner(false);
}


//=============================================================================
// GET AND SET
//=============================================================================
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
void InducedAccelerations::setModel(Model &aModel)
{
    // SET THE MODEL for this analysis to be a copy
    Analysis::setModel(aModel); //*aModel.clone());
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
int InducedAccelerations::record(const SimTK::State& s)
{
    int nu = _model->getNumSpeeds();
    double aT = s.getTime();
    log_info("time = {}", aT);

    SimTK::Vector Q = s.getQ();

    // Reset Accelerations for coordinates at this time step
    for(int i=0;i<_coordSet.getSize();i++) {
        _coordIndAccs[i]->setSize(0);
    }

    // Reset Accelerations for bodies at this time step
    for(int i=0;i<_bodySet.getSize();i++) {
        _bodyIndAccs[i]->setSize(0);
    }

    // Reset Accelerations for system center of mass at this time step
    _comIndAccs.setSize(0);
    _constraintReactions.setSize(0);

    SimTK::State s_analysis = _model->getWorkingState();

    _model->initStateWithoutRecreatingSystem(s_analysis);
    // Just need to set current time and position to determine state of constraints
    s_analysis.setTime(aT);
    s_analysis.setQ(Q);

    // Check the external forces and determine if contact constraints should be applied at this time
    // and turn constraint on if it should be.
    Array<bool> constraintOn = applyContactConstraintAccordingToExternalForces(s_analysis);

    // Hang on to a state that has the right flags for contact constraints turned on/off
    _model->setPropertiesFromState(s_analysis);
    // Use this state for the remainder of this step (record)
    s_analysis = _model->getMultibodySystem().realizeTopology();
    // DO NOT recreate the system, will lose location of constraint
    _model->initStateWithoutRecreatingSystem(s_analysis);

    //Use same conditions on constraints
    s_analysis.setTime(aT);

    // Cycle through the force contributors to the system acceleration
    for(int c=0; c< _contributors.getSize(); c++){          
        //cout << "Solving for contributor: " << _contributors[c] << endl;
        // Need to be at the dynamics stage to disable a force
        _model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Dynamics);
        
        if(_contributors[c] == "total"){
            // Set gravity ON
            _model->getGravityForce().enable(s_analysis);

            // Set the configuration (gen. coords and speeds) of the model.
            s_analysis.setQ(Q);
            s_analysis.setU(s.getU());
            s_analysis.setZ(s.getZ());

            //Make sure all the actuators are on!
            for(int f=0; f<_model->getActuators().getSize(); f++){
                _model->updActuators().get(f).setAppliesForce(s_analysis, true);
            }

            // Get to  the point where we can evaluate unilateral constraint conditions
             _model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Acceleration);

            /* *********************************** ERROR CHECKING *******************************
            SimTK::Vec3 pcom =_model->getMultibodySystem().getMatterSubsystem().calcSystemMassCenterLocationInGround(s_analysis);
            SimTK::Vec3 vcom =_model->getMultibodySystem().getMatterSubsystem().calcSystemMassCenterVelocityInGround(s_analysis);
            SimTK::Vec3 acom =_model->getMultibodySystem().getMatterSubsystem().calcSystemMassCenterAccelerationInGround(s_analysis);

            SimTK::Matrix M;
            _model->getMultibodySystem().getMatterSubsystem().calcM(s_analysis, M);
            cout << "mass matrix: " << M << endl;

            SimTK::Inertia sysInertia = _model->getMultibodySystem().getMatterSubsystem().calcSystemCentralInertiaInGround(s_analysis);
            cout << "system inertia: " << sysInertia << endl;

            SimTK::SpatialVec sysMomentum =_model->getMultibodySystem().getMatterSubsystem().calcSystemMomentumAboutGroundOrigin(s_analysis);
            cout << "system momentum: " << sysMomentum << endl;

            const SimTK::Vector &appliedMobilityForces = _model->getMultibodySystem().getMobilityForces(s_analysis, SimTK::Stage::Dynamics);
            appliedMobilityForces.dump("All Applied Mobility Forces");
        
            // Get all applied body forces like those from contact
            const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces = _model->getMultibodySystem().getRigidBodyForces(s_analysis, SimTK::Stage::Dynamics);
            appliedBodyForces.dump("All Applied Body Forces");

            SimTK::Vector ucUdot;
            SimTK::Vector_<SimTK::SpatialVec> ucA_GB;
            _model->getMultibodySystem().getMatterSubsystem().calcAccelerationIgnoringConstraints(s_analysis, appliedMobilityForces, appliedBodyForces, ucUdot, ucA_GB) ;
            ucUdot.dump("Udots Ignoring Constraints");
            ucA_GB.dump("Body Accelerations");

            SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces(_constraintSet.getSize(), SimTK::SpatialVec(SimTK::Vec3(0)));
            SimTK::Vector constraintMobilityForces(0);

            int nc = _model->getMultibodySystem().getMatterSubsystem().getNumConstraints();
            for (SimTK::ConstraintIndex cx(0); cx < nc; ++cx) {
                if (!_model->getMultibodySystem().getMatterSubsystem().isConstraintDisabled(s_analysis, cx)){
                    cout << "Constraint " << cx << " enabled!" << endl;
                }
            }
            //int nMults = _model->getMultibodySystem().getMatterSubsystem().getTotalMultAlloc();

            for(int i=0; i<constraintOn.getSize(); i++) {
                if(constraintOn[i])
                    _constraintSet[i].calcConstraintForces(s_analysis, constraintBodyForces, constraintMobilityForces);
            }
            constraintBodyForces.dump("Constraint Body Forces");
            constraintMobilityForces.dump("Constraint Mobility Forces");
            // ******************************* end ERROR CHECKING *******************************/
    
            for(int i=0; i<constraintOn.getSize(); i++) {
                _constraintSet.get(i).setIsEnforced(s_analysis,
                                                    constraintOn[i]);
                // Make sure we stay at Dynamics so each constraint can evaluate its conditions
                _model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Acceleration);
            }

            // This should also push changes to defaults for unilateral conditions
            _model->setPropertiesFromState(s_analysis);

        }
        else if(_contributors[c] == "gravity"){
            // Set gravity ON
            _model->updForceSubsystem().setForceIsDisabled(s_analysis, _model->getGravityForce().getForceIndex(), false);

            s_analysis.setQ(Q);

            // zero velocity
            s_analysis.setU(SimTK::Vector(nu,0.0));
            s_analysis.setZ(s.getZ());

            // disable actuator forces
            for(int f=0; f<_model->getActuators().getSize(); f++){
                _model->updActuators().get(f).setAppliesForce(s_analysis,
                                                              false);
            }
        }
        else if(_contributors[c] == "velocity"){        
            // Set gravity off
            _model->updForceSubsystem().setForceIsDisabled(s_analysis, _model->getGravityForce().getForceIndex(), true);

            s_analysis.setQ(Q);

            // non-zero velocity
            s_analysis.setU(s.getU());
            s_analysis.setZ(s.getZ());
            
            // zero actuator forces
            for(int f=0; f<_model->getActuators().getSize(); f++){
                _model->updActuators().get(f).setAppliesForce(s_analysis,
                                                              false);
            }
            // Set the configuration (gen. coords and speeds) of the model.
            _model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Velocity);
        }
        else{ //The rest are actuators      
            // Set gravity OFF
            _model->updForceSubsystem().setForceIsDisabled(s_analysis, _model->getGravityForce().getForceIndex(), true);

            // zero actuator forces
            for(int f=0; f<_model->getActuators().getSize(); f++){
                _model->updActuators().get(f).setAppliesForce(s_analysis,
                                                              false);
            }

            s_analysis.setQ(Q);

            // zero velocity
            SimTK::Vector U(nu,0.0);
            s_analysis.setU(U);
            s_analysis.setZ(s.getZ());
            // light up the one actuator who's contribution we are looking for
            int ai = _model->getActuators().getIndex(_contributors[c]);
            if(ai<0)
                throw Exception("InducedAcceleration: ERR- Could not find actuator '"+_contributors[c],__FILE__,__LINE__);
            
            Actuator &actuator = _model->getActuators().get(ai);
            ScalarActuator* act = dynamic_cast<ScalarActuator*>(&actuator);
            act->setAppliesForce(s_analysis, true);
            act->overrideActuation(s_analysis, false);
            Muscle *muscle = dynamic_cast<Muscle *>(&actuator);
            if(muscle){
                if(_computePotentialsOnly){
                    muscle->overrideActuation(s_analysis, true);
                    muscle->setOverrideActuation(s_analysis, 1.0);
                }
            }

            // Set the configuration (gen. coords and speeds) of the model.
            _model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Model);
            _model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Velocity);

        }// End of if to select contributor 

        // cout << "Constraint 0 is of "<< _constraintSet[0].getConcreteClassName() << " and should be " << constraintOn[0] << " and is actually " <<  (_constraintSet[0].isDisabled(s_analysis) ? "off" : "on") << endl;
        // cout << "Constraint 1 is of "<< _constraintSet[1].getConcreteClassName() << " and should be " << constraintOn[1] << " and is actually " <<  (_constraintSet[1].isDisabled(s_analysis) ? "off" : "on") << endl;

        // After setting the state of the model and applying forces
        // Compute the derivative of the multibody system (speeds and accelerations)
        _model->getMultibodySystem().realize(s_analysis, SimTK::Stage::Acceleration);

        // Sanity check that constraints hasn't totally changed the configuration of the model
        // double error = (Q-s_analysis.getQ()).norm();

        // Report reaction forces for debugging
        /*
        SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces(_constraintSet.getSize());
        SimTK::Vector mobilityForces(0);

        for(int i=0; i<constraintOn.getSize(); i++) {
            if(constraintOn[i])
                _constraintSet.get(i).calcConstraintForces(s_analysis, constraintBodyForces, mobilityForces);
        }*/

        // VARIABLES
        SimTK::Vec3 vec,angVec;

        // Get Accelerations for kinematics of bodies
        for(int i=0;i<_coordSet.getSize();i++) {
            double acc = _coordSet.get(i).getAccelerationValue(s_analysis);

            if(getInDegrees()) 
                acc *= SimTK_RADIAN_TO_DEGREE;  
            _coordIndAccs[i]->append(1, &acc);
        }

        // cout << "Input Body Names: "<< _bodyNames << endl;

        // Get Accelerations for kinematics of bodies
        for(int i=0;i<_bodySet.getSize();i++) {
            Body &body = _bodySet.get(i);
            // cout << "Body Name: "<< body->getName() << endl;
            const SimTK::Vec3& com = body.get_mass_center();
            
            // Get the body acceleration
            vec = body.findStationAccelerationInGround(s_analysis, com);
            angVec = body.getAccelerationInGround(s_analysis)[0];

            // CONVERT TO DEGREES?
            if(getInDegrees()) 
                angVec *= SimTK_RADIAN_TO_DEGREE;   

            // FILL KINEMATICS ARRAY
            _bodyIndAccs[i]->append(3, &vec[0]);
            _bodyIndAccs[i]->append(3, &angVec[0]);
        }

        // Get Accelerations for kinematics of COM
        if(_includeCOM){
            // Get the body acceleration in ground
            vec = _model->getMultibodySystem().getMatterSubsystem().calcSystemMassCenterAccelerationInGround(s_analysis);

            // FILL KINEMATICS ARRAY
            _comIndAccs.append(3, &vec[0]);
        }

        // Get induced constraint reactions for contributor
        if(_reportConstraintReactions){
            for(int j=0; j<_constraintSet.getSize(); j++){
                _constraintReactions.append(_constraintSet[j].getRecordValues(s_analysis));
            }
        }

    } // End cycling through contributors at this time step

    // Set the accelerations of coordinates into their storages
    int nc = _coordSet.getSize();
    for(int i=0; i<nc; i++) {
        _storeInducedAccelerations[i]->append(aT, _coordIndAccs[i]->getSize(),&(_coordIndAccs[i]->get(0)));
    }

    // Set the accelerations of bodies into their storages
    int nb = _bodySet.getSize();
    for(int i=0; i<nb; i++) {
        _storeInducedAccelerations[nc+i]->append(aT, _bodyIndAccs[i]->getSize(),&(_bodyIndAccs[i]->get(0)));
    }

    // Set the accelerations of system center of mass into a storage
    if(_includeCOM){
        _storeInducedAccelerations[nc+nb]->append(aT, _comIndAccs.getSize(), &_comIndAccs[0]);
    }
    if(_reportConstraintReactions){
        _storeConstraintReactions->append(aT, _constraintReactions.getSize(), &_constraintReactions[0]);
    }

    return(0);
}

/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * @param s SimTK:State
 */
void InducedAccelerations::initialize(const SimTK::State& s)
{   
    // Go forward with a copy of the model so Analysis can add to model if necessary
    _model = _model->clone();

    SimTK::State s_copy = s;
    // double time = s_copy.getTime();

    _externalForces.setSize(0);

    //add constraint to set 
    for(int i=0; i<_constraintSet.getSize(); i++){
        Constraint* contactConstraint = &_constraintSet.get(i);
        if(contactConstraint)
            _model->updConstraintSet().adoptAndAppend(contactConstraint);
    }

    // Create a set of constraints used to model contact with the ground
    // based on external forces (ExternalForces) applied to the model
    auto externalForces = _model->updComponentList<ExternalForce>();
    for(auto& exf : externalForces){
        addContactConstraintFromExternalForce(&exf);
        exf.setAppliesForce(s_copy, false);
        exf.set_appliesForce(false);
    }

    // Get value for gravity
    _gravity = _model->getGravity();

    /*SimTK::State &s_analysis =*/_model->initSystem();

    // UPDATE VARIABLES IN THIS CLASS
    constructDescription();
    setupStorage();
}

//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the beginning of an integration in
 * Model::integBeginCallback() and has the same argument list.
 *
 * @param s SimTK:State
 *
 * @return -1 on error, 0 otherwise.
 */
int InducedAccelerations::begin(const SimTK::State &s)
{
    if(!proceed()) return(0);

    initialize(s);

    // RESET STORAGES
    for(int i = 0; i<_storeInducedAccelerations.getSize(); i++){
        _storeInducedAccelerations[i]->reset(s.getTime());
    }

    log_info("Performing Induced Accelerations Analysis");

    // RECORD
    int status = 0;
    status = record(s);

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
 * @param s, state
 * @param stepNumber
 *
 * @return -1 on error, 0 otherwise.
 */
int InducedAccelerations::step(const SimTK::State &s, int stepNumber)
{
    if(proceed(stepNumber) && getOn())
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
 * @param State
 *
 * @return -1 on error, 0 otherwise.
 */
int InducedAccelerations::end(const SimTK::State &s)
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
int InducedAccelerations::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    // Write out induced accelerations for all kinematic variables
    for(int i = 0; i < _storeInducedAccelerations.getSize(); i++){
        Storage::printResult(_storeInducedAccelerations[i],aBaseName+"_"
                               +getName()+"_"+_storeInducedAccelerations[i]->getName(),aDir,aDT,aExtension);
    }

    if(_reportConstraintReactions){
        Storage::printResult(_storeConstraintReactions, aBaseName+"_"
                               +getName()+"_"+_storeConstraintReactions->getName(),aDir,aDT,aExtension);
    }
    return(0);
}


void InducedAccelerations::addContactConstraintFromExternalForce(ExternalForce *externalForce)
{
    _externalForces.append(externalForce);
}

Array<bool> InducedAccelerations::applyContactConstraintAccordingToExternalForces(SimTK::State &s)
{
    Array<bool> constraintOn(false, _constraintSet.getSize());
    double t = s.getTime();

    for(int i=0; i<_externalForces.getSize(); i++){
        ExternalForce *exf = _externalForces[i];
        SimTK::Vec3 point, force, gpoint;

        force = exf->getForceAtTime(t);
        
        // If the applied force is "significant" replace it with a constraint
        if (force.norm() > _forceThreshold){
            // get the point of contact from applied external force
            point = exf->getPointAtTime(t);
            // point should be expressed in the "applied to" body for consistency across all constraints
            if(exf->getPointExpressedInBodyName() != exf->getAppliedToBodyName()){
                const PhysicalFrame* appliedToBody =
                        _model->findComponent<PhysicalFrame>(
                                exf->getAppliedToBodyName());
                const PhysicalFrame* expressedInBody =
                        _model->findComponent<PhysicalFrame>(
                                exf->getPointExpressedInBodyName());
                OPENSIM_THROW_IF_FRMOBJ(appliedToBody == nullptr, Exception,
                        "ExternalForce's appliedToBody " +
                                exf->getAppliedToBodyName() + " not found.");
                OPENSIM_THROW_IF_FRMOBJ(expressedInBody == nullptr, Exception,
                        "ExternalForce's pointExpressedInBodyName " +
                                exf->getPointExpressedInBodyName() +
                                " not found.");
                OPENSIM_THROW_IF_FRMOBJ(appliedToBody == nullptr, Exception, "ExternalForce's appliedToBody " + exf->getAppliedToBodyName() + " not found.");
                OPENSIM_THROW_IF_FRMOBJ(expressedInBody == nullptr, Exception, "ExternalForce's pointExpressedInBodyName " + exf->getPointExpressedInBodyName() + " not found.");

                _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
                point = expressedInBody->findStationLocationInAnotherFrame(
                        s, point, *appliedToBody);
            }

            _constraintSet.get(i).setContactPointForInducedAccelerations(s, point);

            // turn on the constraint
            _constraintSet.get(i).setIsEnforced(s, true);
            // return the state of the constraint
            constraintOn[i] = true;

        }
        else{
            // turn off the constraint
            _constraintSet.get(i).setIsEnforced(s, false);
            // return the state of the constraint
            constraintOn[i] = false;
        }
    }

    return constraintOn;
}
