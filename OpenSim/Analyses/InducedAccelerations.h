#ifndef _InducedAccelerations_h_
#define _InducedAccelerations_h_
// InducedAccelerations.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Ajay Seth
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2008, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
// Headers define the various property types that OpenSim objects can read 
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyBoolArray.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Simulation/Model/Analysis.h>
// Header to define analysis (DLL) interface
#include "osimAnalysesDLL.h"
#include <SimTKsimbody.h>
#include <map>
#include <string>
// #include <iostream.h>


//=============================================================================
//=============================================================================
/*
 * A class to perform Induced Accelerations using the hard constraint method
 * Reports the contributions of all forces. 
 *
 * @author Ajay Seth
 * @version 1.0
 */
namespace OpenSim { 

class Model;
class BodySet;
class CoordinateSet;
class ConstraintSet;
class PrescribedForce;

class OSIMANALYSES_API InducedAccelerations : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:

	/* Convenience sets of coordinates and bodies whose accelerations we want to analyze. */
	CoordinateSet &_coordSet;
	BodySet &_bodySet;

protected:
	// Properties are the user-specified quantities that are read in from file
	/** Specifies the list of coordinates for which induced accelerations are reported */
	PropertyStrArray _coordNamesProp;
	Array<std::string> &_coordNames;

	/** Specifies the list of bodies which includes center_of_mass 
	    for which induced accelerations are reported */
	PropertyStrArray _bodyNamesProp;
	Array<std::string> &_bodyNames;

	/** Set containing the constraints used to represent contact in the analysis */
	PropertyObj _constraintSetProp;
	ConstraintSet &_constraintSet;

	/** Threshhold of force necessary for a constraint to be active */
	PropertyDbl _forceThresholdProp;
	double &_forceThreshold;

	/** Flag to only compute the potential (acceleration/force) of a muscle to accelerate the model. */
	PropertyBool _computePotentialsOnlyProp;
	bool &_computePotentialsOnly;

	/** Flag to report the constraint reaction forces (Lagrange multipliers). */
	PropertyBool _reportConstraintReactionsProp;
	bool &_reportConstraintReactions;

	/** Storages for recording induced accelerations for specified coordinates and/or bodies. */
	Array<Storage *> _storeInducedAccelerations;
	Storage* _storeConstraintReactions;

	/** List of all the contributors to the model acceleration */
	Array<std::string> _contributors;

	bool _includeCOM;

	// Internal work arrays to hold the induced accelerations at a given instant
	Array<Array<double> *> _coordIndAccs;
	Array<Array<double> *> _bodyIndAccs;
	Array<double> _comIndAccs;
	Array<double> _constraintReactions;

	// Array to hold external forces (appliers) we want to replace
	Array<PrescribedForce *> _externalForces;

	// Hold the actual model gravity since we will be changing it back and forth from 0
	SimTK::Vec3 _gravity;

	// Mapping of constraint type for switch case handling! *Added by Sam Hamner 07-Mar-2011
	// enum constraintValue { ROLL, POINT, PIN, WELD };
	// std::map <std::string, constraintValue> constraintMap;
	

//=============================================================================
// METHODS
//=============================================================================
public:
	/**
	 * Construct an InducedAccelerations instance with a Model.
	 *
	 * @param aModel Model for which the analysis is to be run.
	 */
	InducedAccelerations(Model *aModel=0);


	/**
	 * Construct an object from file.
	 *
	 * @param aFileName File name of the document.
	 */
	InducedAccelerations(const std::string &aFileName);

	/**
	 * Copy constructor.
	 */
	InducedAccelerations(const InducedAccelerations &aObject);

	//-------------------------------------------------------------------------
	// DESTRUCTOR
	//-------------------------------------------------------------------------
	virtual ~InducedAccelerations();

	/** Clone of object */
	virtual Object* copy() const;
private:
	/** Zero data and set pointers to Null */
	void setNull();

	/**
	 * Set up the properties for th analysis.
	 * Each property should have meaningful name and an informative comment.
	 * The name you give each property is the tag that will be used in the XML
	 * file.  The comment will appear before the property in the XML file.
	 * In addition, the comments are used for tool tips in the OpenSim GUI.
	 *
	 * All properties are added to the property set.  Once added, they can be
	 * read in and written to file.
	 */
	void setupProperties();

public:

#ifndef SWIG
	/**
	 * Assign this object to the values of another.
	 *
	 * @return Reference to this object.
	 */
	InducedAccelerations& operator=(const InducedAccelerations &aInducedAccelerations);
#endif

	//========================== Required Methods =============================
	//-------------------------------------------------------------------------
	// GET AND SET
	//-------------------------------------------------------------------------
	virtual void setModel(Model &aModel);

	//-------------------------------------------------------------------------
	// INTEGRATION
	//-------------------------------------------------------------------------
	virtual void initialize(const SimTK::State& s); 
	virtual int begin( SimTK::State& s);
    virtual int step( const SimTK::State& s, int stepNumber);
    virtual int end( SimTK::State& s);

	//-------------------------------------------------------------------------
	// IO
	//-------------------------------------------------------------------------
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");


	void addContactConstraintFromExternalForce(PrescribedForce *externalForce);
	Array<bool> applyContactConstraintAccordingToExternalForces(SimTK::State &s);

protected:
	//========================== Internal Methods =============================
	int record(const SimTK::State& s);
	void constructDescription();
	void assembleContributors();
	Array<std::string> constructColumnLabelsForCoordinate();
	Array<std::string> constructColumnLabelsForBody();
	Array<std::string> constructColumnLabelsForCOM();
	Array<std::string> constructColumnLabelsForConstraintReactions();
	void setupStorage();

	Array<bool> applyConstraintsAccordingToExternalForces(SimTK::State &s);

//=============================================================================
}; // END of class InducedAccelerations
}; //namespace
//=============================================================================
//=============================================================================

#endif // #ifndef __InducedAccelerations_h__
