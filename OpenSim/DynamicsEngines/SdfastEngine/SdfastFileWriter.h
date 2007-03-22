#ifndef __SdfastFileWriter_h__
#define __SdfastFileWriter_h__

// SdfastFileWriter.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
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

// INCLUDE
#include <iostream>
#include <fstream>
#include <string>
#include "osimSdfastEngineDLL.h"
#include <OpenSim/Common/Array.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmStep.h>

namespace OpenSim {

class SdfastEngine;
class Model;
class AbstractBody;
class AbstractJoint;
class AbstractCoordinate;
class AbstractSpeed;
class AbstractDof;
class DofSet;

//=============================================================================
//=============================================================================
/**
 * A class for writing SIMM joint and muscle files for any Model.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSDFASTENGINE_API SdfastFileWriter
{
//=============================================================================
// STRUCTURES
//=============================================================================
public:
	typedef enum
	{
		dpPin,
		dpReversePin,
		dpUniversal,
		dpReverseUniversal,
		dpGimbal,
		dpReverseGimbal,
		dpBall,
		dpReverseBall,
		dpSlider,
		dpReverseSlider,
		dpCylindrical,
		dpReverseCylindrical,
		dpPlanar,
		dpReversePlanar,
		dpBushing,
		dpReverseBushing,
		dpBearing,
		dpReverseBearing,
		dpFree,
		dpReverseFree,
		dpWeld,
		dpSkippable,
		dpLoop,
		dpUnknownJoint
	} dpJointType;

	typedef struct
	{
		std::string name;           /* name as dof appears in SD/FAST code */
		std::string constraintName; /* if constrained, additional name */
		double initialValue;        /* value of dof at start of simulation */
		bool constrained;           /* is this dof constrained by a kinematic function? */
		bool fixed;                 /* is this dof fixed (prescribed in SD/FAST)? */
		int stateNumber;            /* element of state vector that holds this dof */
		int errorNumber;            /* if constrained, index into error array */
		int joint;                  /* SD/FAST joint which contains this dof */
		int axis;                   /* axis in SD/FAST joint which corresponds to dof */
		AbstractDof* modelDof;      /* pointer to model Dof that this info is for */
	} DofInfo;

	typedef struct
	{
		bool used;
		std::string name;
		dpJointType type;
		int index;
		SimmStep::Direction direction;
		std::string inbname;
		std::string outbname;
		bool closesLoop;
		int numDofs;
		DofInfo *dofs;
		AbstractJoint* modelJoint;
		int parentBodyIndex;
		int childBodyIndex;
		double locationInParent[3];
		double locationInChild[3];
		std::string prescribedString;
		std::string pinString;
	} JointInfo;

	typedef struct
	{
		int restraintFuncNum;
		int minRestraintFuncNum;
		int maxRestraintFuncNum;
		AbstractCoordinate* modelCoordinate;
	} CoordinateInfo;

	typedef struct
	{
		bool used;
		int timesSplit;
		double massFactor;
		bool skippable;
		AbstractBody* modelBody;
	} ModelBodyInfo;

	typedef struct
	{
		std::string name;
		double mass;
		double massCenter[3];
		double inertia[3][3];
		double bodyToJoint[3];
		double inboardToJoint[3];
		ModelBodyInfo* bodyInfo;
	} SdfastBodyInfo;

//=============================================================================
// DATA
//=============================================================================
protected:
	// Model to save dynamics for
	Model *_model;

	// Equivalent OpenSim model with SdfastEngine, for use in simulations.
	Model *_simulationModel;
	SdfastEngine *_simulationEngine;

	// Folder to write files to
	std::string _folderName;

	// Whether or not object has been initialized
	bool _initialized;

	// Number of SD/FAST Qs
	int _numQs;

	// Number of SD/FAST constraints
	int _numConstraints;

	// Number of joint restraint functions
	int _numRestraintFunctions;

	// Order that model's joints are written to the SD/FAST input file
	// the ints are indices into the _joints array.
	Array<int> _jointOrder;

	// SD/FAST info for each joint. Maps directly to model's joint set.
	Array<JointInfo> _joints;

	// SD/FAST info for each coordinate. Maps directly to model's coordinate set.
	Array<CoordinateInfo> _coordinates;

	// SD/FAST info for each model body. Maps directly to model's body set.
	Array<ModelBodyInfo> _modelBodies;

	// Info for each SD/FAST body. Maps directly to list of bodies in SD/FAST,
	// not the model's body set.
	Array<SdfastBodyInfo> _sdfastBodies;

	// Pointer to model body that is used as ground.
	AbstractBody* _groundBody;

	// Index into _modelBodies of body that is used as ground.
	int _groundBodyIndex;

	// Name of folder to write output files to
	std::string _outputFolder;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SdfastFileWriter();
	SdfastFileWriter(Model* aModel, const std::string& aFolderName);
	virtual ~SdfastFileWriter();
	bool isValidSdfastModel();
	void makeSdfastModel();
	void writeSdfastFile(const std::string& aFileName);
	void writeModelHeaderFile(const std::string& aFileName);
	void writeModelSourceFile(const std::string& aFileName, const std::string& aModelHeaderFileName);
	void writeSimulationParametersFile(const std::string& aFileName,
		const std::string& aMuscleFileName = "no_muscles",
		const std::string& aBonePath = "no_path",
		const std::string& aKineticsFile = "no_kinetics",
		const std::string& aOutputMotionFile = "results.mot");
	void writeSimulationModelFile(const std::string& aFileName, const std::string& aModelLibraryName);
	void identifySdfastType(AbstractJoint& aJoint, JointInfo& aInfo);
	void initialize();

private:
	DofInfo* findNthSdfastQ(int n, JointInfo*& aJoint) const;
	DofInfo* findUnconstrainedSdfastDof(const AbstractCoordinate* aCoord) const;
	void countSdfastQsAndConstraints(void);
	int getBodyIndex(AbstractBody* aBody);
	void initInfoStructs(void);
	void makeDofSdfastNames();
	std::string makeSdfastBodyName(const std::string& bodyName, int timesSplit) const;
	void makeSdfastJointOrder();
	AbstractDof* getTranslationDof(int aAxis, DofSet* aDofSet) const;
	void makeSdfastJoint(int aJointIndex, int& rDofCount, int& rConstrainedCount);
	void writeJoint(JointInfo& aJointInfo, SdfastBodyInfo& aSdfastBody, std::ofstream& aStream);
	void writeSdfastConstraintData(std::ofstream& out);
	void writeSdfastQRestraintData(std::ofstream& out);
	void writeSdfastQRestraintFunctions(std::ofstream& out);
	void writeSdfastQInitCode(std::ofstream& out);
	void writeSdfastInitCode(std::ofstream& out);
	void writeSdfastConstraintCode(std::ofstream& out);
	void writeSdfastWrapObjects(std::ofstream& out);
	void writeSdfastConstraintObjects(std::ofstream& out);
	void makeSdfastWeld(int aJointIndex, int& rDofCount, int& rConstrainedCount);
	void makeSdfastPin(int aJointIndex, int& rDofCount, int& rConstrainedCount);
	void makeSdfastSlider(int aJointIndex, int& rDofCount, int& rConstrainedCount);
	void makeSdfastPlanar(int aJointIndex, int& rDofCount, int& rConstrainedCount);
	void makeSdfastUniversal(int aJointIndex, int& rDofCount, int& rConstrainedCount);
	void makeSdfastCylindrical(int aJointIndex, int& rDofCount, int& rConstrainedCount);
	void makeSdfastGimbal(int aJointIndex, int& rDofCount, int& rConstrainedCount);
	void makeSdfastBushing(int aJointIndex, int& rDofCount, int& rConstrainedCount);
	AbstractDof* findNthFunctionRotation(JointInfo& aJointInfo, int aN, int& rIndex) const;
	AbstractDof* findNthFunctionTranslation(JointInfo& aJointInfo, int aN, int& rIndex) const;
	AbstractDof* findMatchingTranslationDof(JointInfo& aJointInfo, AbstractDof* aRotDof, int& rIndex) const;
	bool isJointSdfastCompatible(const AbstractJoint* aJoint) const;
	bool axesAreParallel(const double* aAxis1, const double* aAxis2, bool aOppositeDirAllowed) const;
	char* getDpJointName(dpJointType type, SimmStep::Direction direction);
	bool checkDynamicParameters() const;
	void addBodyToSimulationModel(SdfastBodyInfo& aSdfastBody);
	void addJointToSimulationModel(JointInfo& aJointInfo);
	AbstractCoordinate* addCoordinateToSimulationModel(DofInfo& aDofInfo);
	AbstractSpeed* addSpeedToSimulationModel(const std::string& aName, double aDefaultValue, const std::string& aCoordName);
	CoordinateInfo* getCoordinateInfo(const AbstractCoordinate* aCoord) const;

//=============================================================================
};	// END of class SdfastFileWriter
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SdfastFileWriter_h__


