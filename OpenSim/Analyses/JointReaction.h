#ifndef OPENSIM_JOINT_REACTION_H_
#define OPENSIM_JOINT_REACTION_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  JointReaction.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matt S. DeMers, Ajay Seth                                       *
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
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

class Model;
class Joint;


/**
 * An analysis for reporting the joint reaction loads from a model. For a given
 * joint, the reaction load is calculated as the forces and moments required to
 * constrain the body motions to satisfy the joint as if the joint did not 
 * exist.
 * 
 * The reaction load acts at the joint center (mobilizer frame) of both the 
 * parent and child bodies and either force can be reported and expressed in 
 * any specified frame. The default behavior is the force on the child 
 * expressed in the ground frame.
 *
 * @author Matt DeMers, Ajay Seth
 * @version 1.0
 */
class OSIMANALYSES_API JointReaction : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(JointReaction, Analysis);

//=============================================================================
// DATA
//=============================================================================
private:

    /* Private struct containing reference indices used to set up the output
     *  of each analyzed joint.*/
    struct JointReactionKey
    {
        /* Joint reference*/
        const Joint* joint;
        /* Is the reaction force applied on the Body the child or the parent
           of the joint?*/
        bool isAppliedOnChild;
        /* The body upon which the force is applied. Specifically, this is
           the base frame of the body specified (i.e. child or parent) from
           isAppliedOnChild. Note that this is only used for printing the
           onBodyName when constructing column labels. */
        const Frame* appliedOnBody;
        /* The reference Frame in which the force should be expressed. */
        const Frame* expressedInFrame;
    };

protected:
    //--------------------------------------------------------------------
    // Properties are the user-specified quantities that are read in from
    // file 
    //--------------------------------------------------------------------

    /** String containing the name of the optional forces storage file*/
    PropertyStr _forcesFileNameProp;
    std::string &_forcesFileName;

    /** String Array containing the names of the joints to be analyzed*/
    PropertyStrArray _jointNamesProp;
    Array<std::string> &_jointNames;

    /** String Array indicating which body of a joint (parent or child)
    *   the reaction load is applied to*/
    PropertyStrArray _onBodyProp;
    Array<std::string> &_onBody;

    /** String Array indicating which frame (ground, parent child)
    *   the reaction load is expressed in*/
    PropertyStrArray _inFrameProp;
    Array<std::string> &_inFrame;

    //-----------------------------------------------------------------------
    // STORAGE
    //-----------------------------------------------------------------------

    /** Storage for holding actuator forces IF SPECIFIED by user.*/
    Storage *_storeActuation;

    /** Storage for recording joint Reaction loads.*/
    Storage _storeReactionLoads;

    //-----------------------------------------------------------------------
    // Additional storage and internal working variable
    //-----------------------------------------------------------------------


    /** Internal work array for holding the computed accelerations. */
    Array<double> _dydt;

    /** Internal work array for holding the computed joint loads for all
    *   joints in the model*/
    Array<double> _allLoads;

    /** Internal work array for holding the computed joint loads of all 
    *   joints specified in _jointNames*/
    Array<double> _Loads;

    /** Internal work array for holding the JointReactionKeys to identify the 
    *   desired joints, onBody, and inFrame to be output*/
    Array<JointReactionKey> _reactionList;

    bool _useForceStorage;

//=============================================================================
// METHODS
//=============================================================================
public:
    //-------------------------------------------------------------------------
    // CONSTRUCTION
    //-------------------------------------------------------------------------
    JointReaction(Model *aModel=0);
    JointReaction(const std::string &aFileName);
    JointReaction(const JointReaction &aObject);
    virtual ~JointReaction();

private:
    void setNull();
    void setupProperties();

    //-------------------------------------------------------------------------
    // OPERATORS
    //-------------------------------------------------------------------------
public:
#ifndef SWIG
    JointReaction& operator=(const JointReaction &aJointReaction);
#endif

    //========================== Required Methods =============================
    //-------------------------------------------------------------------------
    // GET AND SET
    //-------------------------------------------------------------------------
    void setModel(Model& aModel) override;

    // Property accessors
    /** Public accessors for the forcesFileName property */
    const std::string& getForcesFileName() { return _forcesFileName; }
    void setForcesFileName(const std::string newForcesFile) { _forcesFileName = newForcesFile; }
    /** Public accessors for the JointNames property */
    const Array<std::string>& getJointNames() const { return _jointNames; }
    void setJointNames( Array<std::string>& jointNames) { _jointNames = jointNames; }
     /** Public accessors for the onBody property */
    const Array<std::string>& getOnBody() const { return _onBody; }
    void setOnBody( Array<std::string>& onBody) { _onBody = onBody; }
     /** Public accessors for the inFrame property */
    const Array<std::string>& getInFrame() const { return _inFrame; }
    void setInFrame( Array<std::string>& inFrame) { _inFrame = inFrame; }

    //-------------------------------------------------------------------------
    // INTEGRATION
    //----------------------------------------------------------------------
    int
        begin( const SimTK::State& s ) override;
    int
        step( const SimTK::State& s, int setNumber ) override;
    int
        end( const SimTK::State& s ) override;


    //-------------------------------------------------------------------------
    // IO
    //-------------------------------------------------------------------------
    int
        printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto") override;


protected:
    //========================== Internal Methods =============================
    int record(const SimTK::State& s );
    void setupReactionList();
    void constructDescription();
    void constructColumnLabels();
    void setupStorage();
    void loadForcesFromFile();

//=============================================================================
}; // END of class JointReaction
}; //namespace
//=============================================================================
//=============================================================================

#endif // #ifndef OPENSIM_JOINT_REACTION_H_
