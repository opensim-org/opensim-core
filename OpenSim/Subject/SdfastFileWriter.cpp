// SdfastFileWriter.cpp
// Authors: Peter Loan
/* Copyright (c) 2006, Stanford University.
 * 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include "SdfastFileWriter.h"
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/SIMM/BodySet.h>
#include <OpenSim/Simulation/SIMM/JointSet.h>
#include <OpenSim/Simulation/SIMM/CoordinateSet.h>
#include <OpenSim/Simulation/SIMM/SpeedSet.h>
#include <OpenSim/Simulation/SIMM/DofSet.h>
#include <OpenSim/Simulation/SIMM/DofSet.h>
#include <OpenSim/Simulation/SIMM/AbstractSimmMuscle.h>
#include <OpenSim/Simulation/SIMM/SimmCoordinate.h>
#include <OpenSim/Simulation/SIMM/SimmIO.h>
#include <OpenSim/Simulation/SIMM/SimmMacros.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/NatCubicSpline.h>
#include <OpenSim/Models/SdfastEngine/SdfastEngine.h>

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTANTS
//=============================================================================
static char sdfastGroundName[] = "$ground";
static char sdfastCodeGroundName[] = "ground";
static char fixedString[] = "fixed";
static SdfastFileWriter::JointInfo defJoint = {false, "", SdfastFileWriter::dpUnknownJoint, -1, SimmStep::forward, "", "", false, 0, NULL, NULL, -1, -1,{0.0,0.0,0.0},{0.0,0.0,0.0},""};
static SdfastFileWriter::CoordinateInfo defCoord = {-1, -1, -1, NULL};
static SdfastFileWriter::ModelBodyInfo defBody = {false, 0, 0.0, false, NULL};
static SdfastFileWriter::SdfastBodyInfo defSdfastBody = {"", 0.0, {0.0, 0.0, 0.0}};


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */

SdfastFileWriter::~SdfastFileWriter()
{
	for (int i = 0; i < _joints.getSize(); i++)
		if (_joints[i].dofs)
			delete [] _joints[i].dofs;
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
SdfastFileWriter::SdfastFileWriter(AbstractModel* aModel, const string& aFolderName = string(".")) :
	_model(NULL),
	_simulationModel(NULL),
	_simulationEngine(NULL),
	_initialized(false),
	_jointOrder(-1),
	_joints(defJoint),
	_coordinates(defCoord),
	_modelBodies(defBody),
	_sdfastBodies(defSdfastBody)
{
	_model = aModel;
	_outputFolder = aFolderName;
}


/* Returns the DofInfo which corresponds to the nth Q in the SD/FAST model. */
SdfastFileWriter::DofInfo* SdfastFileWriter::findNthSdfastQ(int n, SdfastFileWriter::JointInfo*& aJoint) const
{
	aJoint = NULL;

	const JointSet* jointSet = _model->getDynamicsEngine().getJointSet();
	for(int index=0; index<jointSet->getSize(); index++) 
	{
		DofSet* dofs = jointSet->get(index)->getDofSet();
		for (int i = 0; i < dofs->getSize(); i++)
		{
			if ((dofs->get(i)->getCoordinate() != NULL || _joints[index].dofs[i].fixed) &&
				 _joints[index].dofs[i].stateNumber == n)
			{
				aJoint = &_joints[index];
				return &_joints[index].dofs[i];
			}
		}
	}

   return NULL;
}

SdfastFileWriter::DofInfo* SdfastFileWriter::findUnconstrainedSdfastDof(const AbstractCoordinate* aCoord) const
{
	const JointSet* jointSet = _model->getDynamicsEngine().getJointSet();
	for(int index=0; index<jointSet->getSize(); index++) 
	{
		DofSet* dofs = jointSet->get(index)->getDofSet();
		for (int i = 0; i < dofs->getSize(); i++)
		{
			if (!_joints[index].dofs[i].constrained && dofs->get(i)->getCoordinate() == aCoord)
			{
				return &_joints[index].dofs[i];
			}
		}
   }

	return NULL;
}


void SdfastFileWriter::countSdfastQsAndConstraints(void)
{
	_numQs = 0;

	const JointSet* jointSet = _model->getDynamicsEngine().getJointSet();
	for(int index=0; index<jointSet->getSize(); index++) 
	{
		DofSet* dofs = jointSet->get(index)->getDofSet();
		for (int i = 0; i < dofs->getSize(); i++)
		{
			if ((dofs->get(i)->getCoordinate() != NULL || _joints[index].dofs[i].fixed))
			{
				_numQs++;
			}
		}
   }

   _numConstraints = _numQs - _model->getDynamicsEngine().getNumCoordinates();

#if 0
   for (i = 0; i < ms->num_constraint_objects; i++)
      _numConstraints += ms->constraintobj[i].numPoints;
#endif
}

/* Get the integer index of a body in the model. These
 * indices are needed because there is a one-to-one
 * mapping between the model's bodies and the _modelBodies
 * array.
 */
int SdfastFileWriter::getBodyIndex(AbstractBody* aBody)
{
	const BodySet* bodySet = _model->getDynamicsEngine().getBodySet();
	for(int i=0; i<bodySet->getSize(); i++)
		if(bodySet->get(i) == aBody)
			return i;

	return -1;
}

/* This function initializes all of the SdfastInfo structures
 * in the joints, bodies, and dofs. It needs to be called at
 * the beginning of every "save dynamics" call in order to
 * clear the values from the last call.
 */
void SdfastFileWriter::initInfoStructs(void)
{
	if (!_model)
		return;

	AbstractDynamicsEngine& engine = _model->getDynamicsEngine();

	_numQs = 0;
	_numConstraints = 0;
	_numRestraintFunctions = 0;

	// Initialize the joint info. Store a pointer to the model's joint
	// in each joint info struct.
	_joints.setSize(engine.getNumJoints());
	JointSet* jointSet = engine.getJointSet();
	for(int index=0; index<jointSet->getSize(); index++) {
		AbstractJoint* joint = jointSet->get(index);
		_joints[index].used = false;
		_joints[index].name.erase(_joints[index].name.begin(), _joints[index].name.end());
		_joints[index].type = dpUnknownJoint;
		_joints[index].index = -1;
		_joints[index].direction = SimmStep::forward;
		_joints[index].inbname.erase(_joints[index].inbname.begin(), _joints[index].inbname.end());
		_joints[index].outbname.erase(_joints[index].outbname.begin(), _joints[index].outbname.end());
		_joints[index].closesLoop = false;
		_joints[index].modelJoint = joint;
		_joints[index].parentBodyIndex = getBodyIndex(joint->getParentBody());
		_joints[index].childBodyIndex = getBodyIndex(joint->getChildBody());
		_joints[index].prescribedString.erase(_joints[index].prescribedString.begin(), _joints[index].prescribedString.end());
		_joints[index].pinString.erase(_joints[index].pinString.begin(), _joints[index].pinString.end());
	}

	/* Initialize the body info. Store a pointer to the model's body
	 * in each body info struct. Also, find the ground body, and its
	 * index in the model's collection of bodies, because this
	 * index is also the index into the _modelBodies array.
	 */
	_modelBodies.setSize(engine.getNumBodies());
	_groundBody = &engine.getGroundBody();
	BodySet* bodySet = engine.getBodySet();
	for(int index=0; index<bodySet->getSize(); index++) {
		_modelBodies[index].used = false;
		_modelBodies[index].timesSplit = 0;
		_modelBodies[index].massFactor = 1.0;
		_modelBodies[index].skippable = false;
		_modelBodies[index].modelBody = bodySet->get(index);
		if (bodySet->get(index) == _groundBody)
			_groundBodyIndex = index;
	}

	/* Initialize the coordinate info. Store a pointer to the model's
	 * coordinate in each coordinate info struct.
	 */
	_coordinates.setSize(engine.getNumCoordinates());
	CoordinateSet* coordinateSet = engine.getCoordinateSet();
	for(int index=0; index<coordinateSet->getSize(); index++) {
		_coordinates[index].restraintFuncNum = -1;
		_coordinates[index].minRestraintFuncNum = -1;
		_coordinates[index].maxRestraintFuncNum = -1;
		_coordinates[index].modelCoordinate = coordinateSet->get(index);
	}

	/* All dofs start as constrained. The function markUnconstrainedDof
	 * unconstains those dofs that correspond to a "true" degree of
	 * freedom in the model.
	 */
	for(int index=0; index<jointSet->getSize(); index++)
	{
		DofSet* dofs = jointSet->get(index)->getDofSet();
		_joints[index].numDofs = dofs->getSize();
		_joints[index].dofs = new DofInfo[_joints[index].numDofs];

		for (int i = 0; i < dofs->getSize(); i++)
		{
			_joints[index].dofs[i].name.erase(_joints[index].dofs[i].name.begin(), _joints[index].dofs[i].name.end());
			_joints[index].dofs[i].constraintName.erase(_joints[index].dofs[i].constraintName.begin(), _joints[index].dofs[i].constraintName.end());
			_joints[index].dofs[i].initialValue = 0.0;
			_joints[index].dofs[i].constrained = true;
			_joints[index].dofs[i].fixed = false;
			_joints[index].dofs[i].stateNumber = -1;
			_joints[index].dofs[i].errorNumber = -1;
			_joints[index].dofs[i].joint = -1;
			_joints[index].dofs[i].axis = -1;
			_joints[index].dofs[i].modelDof = dofs->get(i);
		}
	}
}

bool SdfastFileWriter::isValidSdfastModel()
{
	for (int i = 0; i < _joints.getSize(); i++)
	{
		dpJointType type = _joints[i].type;

		if (type != dpPlanar &&
		    type != dpReversePlanar &&
		    type != dpPin &&
		    type != dpReversePin &&
		    type != dpUniversal &&
		    type != dpReverseUniversal &&
		    type != dpWeld &&
		    type != dpSlider &&
		    type != dpCylindrical &&
		    type != dpReverseCylindrical &&
		    type != dpGimbal &&
		    type != dpReverseGimbal &&
		    type != dpBushing &&
		    type != dpReverseBushing &&
		    type != dpSkippable)
		{
			cerr << "===ERROR===: Joint " << _joints[i].name << " cannot be converted to an SD/FAST joint." << endl;
			return false;
		}
	}

	/* For each coordinate that is used in the model, there must be at least
	 * one DOF which uses it with a 'simple' function. This DOF will be
	 * treated as the Q in the SD/FAST code.
	 */
	AbstractDynamicsEngine& engine = _model->getDynamicsEngine();
	CoordinateSet* coordinateSet = engine.getCoordinateSet();
	for(int index=0; index<coordinateSet->getSize(); index++)
	{
		AbstractCoordinate* coord = coordinateSet->get(index);
		if (coord->isUsedInModel())
		{
			AbstractJoint* jnt = NULL;
			AbstractDof* dof = engine.findUnconstrainedDof(*coord, jnt);
			if (dof && jnt)
			{
				/* Find the DofInfo structure that corresponds to the
				 * unconstrained dof in the model and set it to unconstrained.
				 */
				for (int i = 0; i < _joints.getSize(); i++)
				{
					if (_joints[i].modelJoint == jnt)
					{
						for (int j = 0; j < _joints[i].numDofs; j++)
						{
							if (_joints[i].dofs[j].modelDof == dof)
							{
								_joints[i].dofs[j].constrained = false;
								i = _joints.getSize();
								break;
							}
						}
					}
				}
			}
			else
			{
				cerr << "===ERROR===: At least one DOF must be a \"simple\" function of coordinate " <<
					coord->getName() << " (2 points, slope=1, passes thru zero)." << endl;
				return false;
			}
		}
	}

	return true;
}

/* Names of unconstrained dofs will simply be the names of
 * the gencoords to which they correspond. Names of
 * constrained dofs will be formed from the joint name
 * and the dof keyword (e.g. "hip_tx" and "knee_r1").
 */
void SdfastFileWriter::makeDofSdfastNames()
{
	static string constraintSuffix("_con");

	JointSet* jointSet = _model->getDynamicsEngine().getJointSet();
	for(int index=0; index<jointSet->getSize(); index++)
	{
		AbstractJoint* joint = jointSet->get(index);
		DofSet* dofs = joint->getDofSet();

		for (int i = 0; i < dofs->getSize(); i++)
		{
			if (!_joints[index].dofs[i].constrained)
			{
				const AbstractCoordinate* coord = dofs->get(i)->getCoordinate();
				_joints[index].dofs[i].name = coord->getName();
				convertString(_joints[index].dofs[i].name, true);
				_joints[index].dofs[i].constraintName.erase(_joints[index].dofs[i].constraintName.begin(),
						_joints[index].dofs[i].constraintName.end());
			 }
			 else
			 {
				_joints[index].dofs[i].name = joint->getName();
				_joints[index].dofs[i].name.append("_");
				_joints[index].dofs[i].name.append(_joints[index].dofs[i].modelDof->getName());
				convertString(_joints[index].dofs[i].name, true);
				_joints[index].dofs[i].constraintName = _joints[index].dofs[i].name;
				_joints[index].dofs[i].constraintName.append(constraintSuffix);
			 }
		}
	}
}

string SdfastFileWriter::makeSdfastBodyName(const string& bodyName, int timesSplit) const
{
	string sdfastName = bodyName;

	if (sdfastName[0] >= '0' && sdfastName[0] <= '9')
		sdfastName.insert(0, "_");

	for (int i = 0; i < timesSplit; i++)
		sdfastName.append("p");

	return sdfastName;
}

void SdfastFileWriter::makeSdfastJointOrder()
{
	int jointsDone = 0, numSkippable = 0, jointsUsed = 0;

	_modelBodies[_groundBodyIndex].used = true;

	for (int i = 0; i < _joints.getSize(); i++)
	{
		if (_joints[i].modelJoint->getParentBody() == _groundBody)
		{
			_jointOrder.append(i);
			_joints[i].index = jointsDone++;
			_modelBodies[_joints[i].childBodyIndex].used = true;
			_joints[i].used = true;
			_joints[i].direction = SimmStep::forward;
			_joints[i].inbname = sdfastGroundName;
			_joints[i].outbname = _joints[i].modelJoint->getChildBody()->getName();
			_joints[i].closesLoop = false;
		}
		else if (_joints[i].modelJoint->getChildBody() == _groundBody)
		{
			_jointOrder.append(i);
			_joints[i].index = jointsDone++;
			_modelBodies[_joints[i].parentBodyIndex].used = true;
			_joints[i].used = true;
			_joints[i].direction = SimmStep::inverse;
			_joints[i].inbname = sdfastGroundName;
			_joints[i].outbname = _joints[i].modelJoint->getParentBody()->getName();
			_joints[i].closesLoop = false;
		}
	}

	jointsUsed = jointsDone;
	while (jointsDone + numSkippable < _joints.getSize())
	{
		for (int i = 0; i < _joints.getSize(); i++)
		{
			if (_joints[i].used) continue;

			AbstractBody* parentBody = _joints[i].modelJoint->getParentBody();
			AbstractBody* childBody = _joints[i].modelJoint->getChildBody();

			if (!_modelBodies[_joints[i].parentBodyIndex].used && !_modelBodies[_joints[i].childBodyIndex].used)
			{
				continue;
			}
			else if (_joints[i].type == dpSkippable)
			{
				_joints[i].index = -1;
				numSkippable++;
				_joints[i].used = false;
				continue;
			}
			else if (!_modelBodies[_joints[i].parentBodyIndex].used)
			{
				_jointOrder.append(i);
				_joints[i].index = jointsUsed++;
				_modelBodies[_joints[i].parentBodyIndex].used = true;
				_joints[i].used = true;
				_joints[i].direction = SimmStep::inverse;
				_joints[i].inbname = childBody->getName();
				_joints[i].outbname = parentBody->getName();
				_joints[i].closesLoop = false;
				jointsDone++;
			}
			else if (!_modelBodies[_joints[i].childBodyIndex].used)
			{
				_jointOrder.append(i);
				_joints[i].index = jointsUsed++;
				_modelBodies[_joints[i].childBodyIndex].used = true;
				_joints[i].used = true;
				_joints[i].direction = SimmStep::forward;
				_joints[i].inbname = parentBody->getName();
				_joints[i].outbname = childBody->getName();
				_joints[i].closesLoop = false;
				jointsDone++;
			}
			else
			{
				_jointOrder.append(i);
				_joints[i].index = jointsUsed++;
				_joints[i].used = true;
				_joints[i].direction = SimmStep::forward;
				_modelBodies[_joints[i].childBodyIndex].massFactor += 1.0;
				_modelBodies[_joints[i].childBodyIndex].timesSplit++;
				_joints[i].inbname = parentBody->getName();
				_joints[i].outbname = makeSdfastBodyName(childBody->getName(), _modelBodies[_joints[i].childBodyIndex].timesSplit);
				_joints[i].closesLoop = true;
				jointsDone++;
			}
		}
	}

	/* Remove all special characters in the body names, so that the strings
	 * are valid one-token C strings. Do not touch the ground segment name ($ground)
	 * because SD/FAST requires exactly that name.
	 */
	for (int i = 0; i < _joints.getSize(); i++)
	{
		if (_joints[i].type != dpSkippable)
		{
			if (_joints[i].inbname != sdfastGroundName)
				convertString(_joints[i].inbname, true);
			if (_joints[i].outbname != sdfastGroundName)
				convertString(_joints[i].outbname, true);
		}
   }
}

void SdfastFileWriter::makeSdfastModel()
{
	// Copy gravity from SimmKinematicsEngine
	double gravity[3];
	_model->getDynamicsEngine().getGravity(gravity);
	_simulationModel->getDynamicsEngine().setGravity(gravity);

	int i, j, rDofCount = 0, rConstrainedCount = 0;

	makeDofSdfastNames();
	makeSdfastJointOrder();

	/* Fill in the first body with information about ground. */
	SdfastBodyInfo* sdfastGround = new SdfastBodyInfo;
	sdfastGround->name = _groundBody->getName();
	convertString(sdfastGround->name, true);
	sdfastGround->bodyInfo = &_modelBodies[0]; // TODO always 0?
	_groundBody->getMassCenter(sdfastGround->massCenter);
	sdfastGround->mass = 0.0;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			sdfastGround->inertia[i][j] = 0.0;
	_sdfastBodies.append(*sdfastGround);
	addBodyToSimulationModel(*sdfastGround);

	/* Create the tree joints and bodies. */
	for (i = 0; i < _jointOrder.getSize(); i++)
		if (_joints[_jointOrder[i]].type != dpSkippable)
			makeSdfastJoint(_jointOrder[i], rDofCount, rConstrainedCount);

	/* Create the loop joints and bodies. */
	for (i = 0; i < _modelBodies.getSize(); i++)
	{
		for (j = 0; j < _modelBodies[i].timesSplit; j++)
		{
			/* Make a new SdfastBodyInfo and add it to the array. */
			SdfastBodyInfo* sdfastBody = new SdfastBodyInfo;
			sdfastBody->name = _modelBodies[i].modelBody->getName();
			sdfastBody->mass = 0.0;
			sdfastBody->massCenter[0] = 0.0;
			sdfastBody->massCenter[1] = 0.0;
			sdfastBody->massCenter[2] = 0.0;
			sdfastBody->bodyToJoint[0] = 0.0;
			sdfastBody->bodyToJoint[1] = 0.0;
			sdfastBody->bodyToJoint[2] = 0.0;
			sdfastBody->inboardToJoint[0] = 0.0;
			sdfastBody->inboardToJoint[1] = 0.0;
			sdfastBody->inboardToJoint[2] = 0.0;
			sdfastBody->bodyInfo = NULL;
			_sdfastBodies.append(*sdfastBody);

			/* Make an SdfastBody object and add it to the simulation model's engine.
			 * The engine will take over ownership of the body.
			 */
			addBodyToSimulationModel(*sdfastBody);

			/* Make a new JointInfo and add it to the array. */
			JointInfo* sdfastJoint = new JointInfo;
			sdfastJoint->type = dpLoop;
			sdfastJoint->index = _jointOrder.getSize();
			sdfastJoint->used = true;
			sdfastJoint->direction = SimmStep::forward;
			sdfastJoint->inbname = makeSdfastBodyName(_modelBodies[i].modelBody->getName(), j + 1);
			sdfastJoint->outbname = _modelBodies[i].modelBody->getName();
			sdfastJoint->name = "loop_" + sdfastJoint->inbname + "_" + sdfastJoint->outbname;
			sdfastJoint->closesLoop = true;
			sdfastJoint->numDofs = 0;
			sdfastJoint->dofs = NULL;
			sdfastJoint->modelJoint = NULL;
			_joints.append(*sdfastJoint);
			_jointOrder.append(_jointOrder.getSize());

			/* Make an SdfastJoint object and add it to the simulation model's engine.
			 * The engine will take over ownership of the joint.
			 */
			addJointToSimulationModel(*sdfastJoint);
		}
	}

	countSdfastQsAndConstraints();
}

AbstractDof* SdfastFileWriter::getTranslationDof(int aAxis, DofSet* aDofSet) const
{
	for (int i = 0; i < aDofSet->getSize(); i++)
	{
		if (aDofSet->get(i)->getMotionType() == AbstractDof::Translational)
		{
			double vec[3];
			aDofSet->get(i)->getAxis(vec);
			if (EQUAL_WITHIN_ERROR(vec[aAxis], 1.0))
				return aDofSet->get(i);
		}
	}

	return NULL;
}

void SdfastFileWriter::makeSdfastJoint(int aJointIndex, int& rDofCount, int& rConstrainedCount)
{
	int i, body2Index;
	double body1MassCenter[3], body2MassCenter[3];
	AbstractDof* transDof;
	AbstractBody *body1, *body2;
	ModelBodyInfo* body2Info;

	if (_joints[aJointIndex].direction == SimmStep::forward)
	{
		body1 = _joints[aJointIndex].modelJoint->getParentBody();
		body2 = _joints[aJointIndex].modelJoint->getChildBody();
		body2Index = _joints[aJointIndex].childBodyIndex;
	}
	else
	{
		body1 = _joints[aJointIndex].modelJoint->getChildBody();
		body2 = _joints[aJointIndex].modelJoint->getParentBody();
		body2Index = _joints[aJointIndex].parentBodyIndex;
	}
	body2Info = &_modelBodies[body2Index];
	body1->getMassCenter(body1MassCenter);
	body2->getMassCenter(body2MassCenter);

	/* Make a new SimmSdfastBody, which will later be added to the array of them. */
	SdfastBodyInfo* sdfastBody = new SdfastBodyInfo;
	sdfastBody->name = _joints[aJointIndex].outbname;

	/* If there are loops in the model, then SimmBodys get split and there
	 * will be more sdfastBodys than SimmBodys. So that unsplittable body
	 * parameters (like contact objects) can be assigned to the sdfastBodys,
	 * each sdfastBody has an index of its corresponding SimmBody. But for
	 * SimmBodys that were split, only one piece will have a valid index.
	 */
	if (!_joints[aJointIndex].closesLoop)
	{
		if (_joints[aJointIndex].direction == SimmStep::forward)
			sdfastBody->bodyInfo = &_modelBodies[_joints[aJointIndex].childBodyIndex];
		else
			sdfastBody->bodyInfo = &_modelBodies[_joints[aJointIndex].parentBodyIndex];
	}
	else
	{
		sdfastBody->bodyInfo = NULL;
	}

	/* Copy the mass parameters to the sdfastBody, using the massFactor
	 * parameter to split the values (massFactor = 1.0 for unsplit
	 * bodies).
	 */
	sdfastBody->mass = body2->getMass() / body2Info->massFactor;
	double inertia[3][3];
	body2->getInertia(inertia);
	for (i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			sdfastBody->inertia[i][j] = inertia[i][j] / body2Info->massFactor;
	body2->getMassCenter(sdfastBody->massCenter);

	DofSet* dofs = _joints[aJointIndex].modelJoint->getDofSet();

	/* The bodytojoint vector (inbtojoint vector for INVERSE joints) is always
    * defined by the negative of the mass center vector. The inbtojoint vector
    * (bodytojoint for INVERSE joints) can change depending on whether or not
    * some of the translations in the joint are functions. For all translation
    * components that are functions, you want to use just the mass center
    * vector component so that the DOF value is the same as the SIMM gencoord
    * or SIMM constraint function. For components that are constants, you want
    * to add the DOF value to the mass center vector so that the origin ends
    * up in the right place. 4/3/97.
    * In general, the above method works only for joints in which the translations
    * occur before the rotations. For cases in which the translations occur
    * between two or more rotations, the joint cannot be modeled easily in
    * SD/FAST. For cases in which the translations are after the rotations,
    * SD/FAST has "reverse" joints (e.g., rbushing) that automatically handle
    * the translations properly. Rplanar correctly handles this case because
    * the rplanar joint itself handles two of the translations, and the third
    * one is along the axis of rotation, so it does not need to be handled
    * separately. For joints which do not have a reverse (e.g., pin, cylinder,
    * universal, gimbal), you need to attach the translations to the "other"
    * body segment than you normally would. 4/10/97.
    */
   if (_joints[aJointIndex].type == dpReversePin || _joints[aJointIndex].type == dpReverseGimbal ||
       _joints[aJointIndex].type == dpReverseUniversal || _joints[aJointIndex].type == dpReverseCylindrical)
   {
		if (_joints[aJointIndex].direction == SimmStep::forward)
      {
			transDof = getTranslationDof(0, dofs);
			if (transDof->getCoordinate() == NULL)
				sdfastBody->bodyToJoint[0] = -transDof->getValue() - body2MassCenter[0];
         else
            sdfastBody->bodyToJoint[0] = -body2MassCenter[0];

			transDof = getTranslationDof(1, dofs);
			if (transDof->getCoordinate() == NULL)
            sdfastBody->bodyToJoint[1] = -transDof->getValue() - body2MassCenter[1];
         else
            sdfastBody->bodyToJoint[1] = -body2MassCenter[1];

			transDof = getTranslationDof(2, dofs);
			if (transDof->getCoordinate() == NULL)
            sdfastBody->bodyToJoint[2] = -transDof->getValue() - body2MassCenter[2];
         else
            sdfastBody->bodyToJoint[2] = -body2MassCenter[2];

			sdfastBody->inboardToJoint[0] = -body1MassCenter[0];
         sdfastBody->inboardToJoint[1] = -body1MassCenter[1];
         sdfastBody->inboardToJoint[2] = -body1MassCenter[2];
      }
      else
      {
         sdfastBody->bodyToJoint[0] = -body2MassCenter[0];
         sdfastBody->bodyToJoint[1] = -body2MassCenter[1];
         sdfastBody->bodyToJoint[2] = -body2MassCenter[2];

			transDof = getTranslationDof(0, dofs);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->inboardToJoint[0] = -transDof->getValue() - body1MassCenter[0];
         else
            sdfastBody->inboardToJoint[0] = -body1MassCenter[0];

			transDof = getTranslationDof(1, dofs);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->inboardToJoint[1] = -transDof->getValue() - body1MassCenter[1];
         else
            sdfastBody->inboardToJoint[1] = -body1MassCenter[1];

			transDof = getTranslationDof(2, dofs);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->inboardToJoint[2] = -transDof->getValue() - body1MassCenter[2];
         else
            sdfastBody->inboardToJoint[2] = -body1MassCenter[2];
      }
   }
   else
   {
      if (_joints[aJointIndex].direction == SimmStep::forward)
      {
         sdfastBody->bodyToJoint[0] = -body2MassCenter[0];
         sdfastBody->bodyToJoint[1] = -body2MassCenter[1];
         sdfastBody->bodyToJoint[2] = -body2MassCenter[2];

			transDof = getTranslationDof(0, dofs);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->inboardToJoint[0] = transDof->getValue() - body1MassCenter[0];
         else
            sdfastBody->inboardToJoint[0] = -body1MassCenter[0];

			transDof = getTranslationDof(1, dofs);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->inboardToJoint[1] = transDof->getValue() - body1MassCenter[1];
         else
            sdfastBody->inboardToJoint[1] = -body1MassCenter[1];

			transDof = getTranslationDof(2, dofs);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->inboardToJoint[2] = transDof->getValue() - body1MassCenter[2];
         else
            sdfastBody->inboardToJoint[2] = -body1MassCenter[2];
      }
      else
      {
			transDof = getTranslationDof(0, dofs);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->bodyToJoint[0] = transDof->getValue() - body2MassCenter[0];
         else
            sdfastBody->bodyToJoint[0] = -body2MassCenter[0];

			transDof = getTranslationDof(1, dofs);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->bodyToJoint[1] = transDof->getValue() - body2MassCenter[1];
         else
            sdfastBody->bodyToJoint[1] = -body2MassCenter[1];

			transDof = getTranslationDof(2, dofs);
         if (transDof->getCoordinate() == NULL)
            sdfastBody->bodyToJoint[2] = transDof->getValue() - body2MassCenter[2];
         else
            sdfastBody->bodyToJoint[2] = -body2MassCenter[2];

         sdfastBody->inboardToJoint[0] = -body1MassCenter[0];
         sdfastBody->inboardToJoint[1] = -body1MassCenter[1];
         sdfastBody->inboardToJoint[2] = -body1MassCenter[2];
      }
   }

	switch (_joints[aJointIndex].type)
	{
	   case dpWeld:
		   makeSdfastWeld(aJointIndex, rDofCount, rConstrainedCount);
		   break;
		case dpPin:
		case dpReversePin:
			makeSdfastPin(aJointIndex, rDofCount, rConstrainedCount);
			break;
		case dpSlider:
			makeSdfastSlider(aJointIndex, rDofCount, rConstrainedCount);
			break;
		case dpPlanar:
		case dpReversePlanar:
			makeSdfastPlanar(aJointIndex, rDofCount, rConstrainedCount);
			break;
		case dpUniversal:
		case dpReverseUniversal:
			makeSdfastUniversal(aJointIndex, rDofCount, rConstrainedCount);
			break;
		case dpCylindrical:
		case dpReverseCylindrical:
			makeSdfastCylindrical(aJointIndex, rDofCount, rConstrainedCount);
			break;
		case dpGimbal:
		case dpReverseGimbal:
			makeSdfastGimbal(aJointIndex, rDofCount, rConstrainedCount);
			break;
		case dpBushing:
		case dpReverseBushing:
			makeSdfastBushing(aJointIndex, rDofCount, rConstrainedCount);
			break;
		default:
			throw Exception("Unhandled SD/Fast joint type",__FILE__,__LINE__);
			break;
   }

	// SET THE LOCATION OF THE JOINT IN THE PARENT AND CHILD
	// The location in the child should always be the zero vector.
	Mtx::Add(1,3,body1MassCenter,sdfastBody->inboardToJoint,_joints[aJointIndex].locationInParent);
	Mtx::Add(1,3,body2MassCenter,sdfastBody->bodyToJoint,_joints[aJointIndex].locationInChild);

	/* Now add the sdfastBody to the array of others. */
   _sdfastBodies.append(*sdfastBody);

	/* Make an SdfastBody object and add it to the simulation model's engine.
	 * The engine will take over ownership of the body.
	 */
	addBodyToSimulationModel(*sdfastBody);

	/* Make an SdfastJoint object and add it to the simulation model's engine.
	 * The engine will take over ownership of the joint.
	 */
	addJointToSimulationModel(_joints[aJointIndex]);
}

void SdfastFileWriter::addBodyToSimulationModel(SdfastBodyInfo& aSdfastBody)
{
	SdfastBody *body;
	if (aSdfastBody.bodyInfo && aSdfastBody.bodyInfo->modelBody)
		body = new SdfastBody(*(aSdfastBody.bodyInfo->modelBody));
	else
		body = new SdfastBody();

	body->setName(aSdfastBody.name);
	body->setMassCenter(aSdfastBody.massCenter);
	_simulationEngine->addBody(body);
}

void SdfastFileWriter::addJointToSimulationModel(JointInfo& aJointInfo)
{
	SdfastJoint *joint = new SdfastJoint();
	joint->setName(aJointInfo.name);

	// Inboard body name
	if (aJointInfo.inbname == sdfastGroundName)
		joint->setParentBodyName(sdfastCodeGroundName);
	else
		joint->setParentBodyName(aJointInfo.inbname);

	// Outboard body name
	if (aJointInfo.outbname == sdfastGroundName)
		joint->setChildBodyName(sdfastCodeGroundName);
	else
		joint->setChildBodyName(aJointInfo.outbname);

	// ------------------------------------
	// BEGIN  Added by Clay 2007_01_18
	// To made an SdfastEngine model scalable, two additional properties were
	// added to an SdfastJoint:  the location of the joint in the parent
	// (outboard) body and the location of the joint in the child (inboard)
	// body.  These properties are initialized here.
	// ------------------------------------

	// Location in parent (inboard) body
	// This is the position of the center of mass
	joint->setLocationInParent(aJointInfo.locationInParent);

	// Location in child (outboard) body
	// The location of the joint is always conincident with the body reference frame
	joint->setLocationInChild(aJointInfo.locationInChild);

	// ------------------------------------
	// END  Added by Clay 2007_01_18
	// ------------------------------------

	// Joint type
	joint->setSdfastType(getDpJointName(aJointInfo.type, aJointInfo.direction));
	_simulationEngine->addJoint(joint);
}

AbstractCoordinate* SdfastFileWriter::addCoordinateToSimulationModel(DofInfo& aDofInfo)
{
	SdfastCoordinate *coord;
	if (aDofInfo.fixed == false && aDofInfo.constrained == false)
	{
		const AbstractCoordinate *modelCoord = aDofInfo.modelDof->getCoordinate();
		coord = new SdfastCoordinate(*modelCoord);
		if (modelCoord->getLocked())
			coord->setSdfastType(SdfastCoordinate::dpFixed);
		else
			coord->setSdfastType(SdfastCoordinate::dpUnconstrained);
	}
	else
	{
		coord = new SdfastCoordinate();
		if (aDofInfo.fixed == true)
			coord->setSdfastType(SdfastCoordinate::dpFixed);
		else if (aDofInfo.constrained == true)
			coord->setSdfastType(SdfastCoordinate::dpConstrained);
		else
			coord->setSdfastType(SdfastCoordinate::dpUnconstrained);
	}

	if(aDofInfo.modelDof->getCoordinate() && aDofInfo.constrained) {
		const Function *func = aDofInfo.modelDof->getFunction();
		if(func) coord->setConstraintFunction(func);
	}
	coord->setName(aDofInfo.name);
	coord->setJointIndex(aDofInfo.joint);
	coord->setAxisIndex(aDofInfo.axis);
	// Not clear to me what differences (if any) we may get between aDofInfo.initialValue and aDofInfo.modelDof->getValue(),
	// so just in case they ever differ I added an initialValue property to SdfastCoordinate to store the value that 
	// the initial_value field needs to be set to. 
	coord->setInitialValue(aDofInfo.initialValue);
	coord->setDefaultValue(aDofInfo.modelDof->getValue());
	_simulationEngine->addCoordinate(coord);

	return coord;
}

AbstractSpeed* SdfastFileWriter::addSpeedToSimulationModel(const string& aName, double aDefaultValue, const string& aCoordName)
{
	SdfastSpeed *speed = new SdfastSpeed();

	speed->setName(aName);
	speed->setDefaultValue(aDefaultValue);
	speed->setCoordinateName(aCoordName);

	_simulationEngine->addSpeed(speed);

	return speed;
}

void SdfastFileWriter::writeSdfastFile(const string& aFileName)
{
	ofstream out;
	int i;

	if (!_model)
		return;

	if (!_initialized)
		initialize();

	string fullPath = _outputFolder + '/' + aFileName;

   /* Open the SD/FAST input file and write the header information. */
	out.open(fullPath.c_str());
	out.setf(ios::fixed);
	out.precision(10);

	out << "# SD/FAST input file generated by OpenSim" << endl;
	out << "# Created " << getCurrentTimeString() << endl;
	out << "# Name of original SIMM joints file: dynamicFull.xml" << endl;

	out << "language = c" << endl << endl;

	double gravity[3];
	_model->getGravity(gravity);
	out << "gravity = " << gravity[0] << "? " << gravity[1] << "? " << gravity[2] << "?" << endl << endl;

	for (i = 0; i < _jointOrder.getSize(); i++)
		writeJoint(_joints[_jointOrder[i]], _sdfastBodies[i+1], out); // TODO i+1 ??

   /* Write out the constraints. The number of constraints is the number of q's
    * minus the number of SIMM gencoords, plus the number of constraint object
	 * points.
    */
	DofInfo* dof = NULL;
	JointInfo* joint = NULL;

	countSdfastQsAndConstraints();

	if (_numConstraints > 0)
	{
		out << endl << "#constraints = " << _numConstraints << endl << endl;

		for (i = 0; i < _numQs; i++)
		{
			dof = findNthSdfastQ(i, joint);
			if (dof != NULL && dof->constrained)
				out << "constraint = " << dof->constraintName << endl;
		}
#if 0
		for (i = 0; i < model[mod]->num_constraint_objects; i++)
		{
			for (j = 0; j < model[mod]->constraintobj[i].numPoints; j++)
			{
				out <<  "constraint = %s_", model[mod]->constraintobj[i].name);
				out <<  "%s\n", model[mod]->constraintobj[i].points[j].name);
			}
		}
#endif
	}
	out.close();
}

void SdfastFileWriter::writeJoint(JointInfo& aJointInfo, SdfastBodyInfo& aSdfastBody, ofstream& aStream)
{
	if (aJointInfo.type != dpLoop)
	{
		if (aJointInfo.type == dpWeld)
			aStream << "# the following joint is really a weld joint implemented as a prescribed pin" << endl;

		aStream << "body = " << aJointInfo.outbname << " inb = " << aJointInfo.inbname << " joint = " <<
			getDpJointName(aJointInfo.type, aJointInfo.direction) << endl;

		aStream << "mass = " << aSdfastBody.mass << "? inertia = " << aSdfastBody.inertia[0][0] << "? " <<
			aSdfastBody.inertia[0][1] << "? " << aSdfastBody.inertia[0][2] << "?" << endl;
		aStream << "                                 " << aSdfastBody.inertia[1][0] << "? " <<
			aSdfastBody.inertia[1][1] << "? " << aSdfastBody.inertia[1][2] << "?" << endl;
		aStream << "                                 " << aSdfastBody.inertia[2][0] << "? " <<
			aSdfastBody.inertia[2][1] << "? " << aSdfastBody.inertia[2][2] << "?" << endl;

		aStream << "bodytojoint = " << aSdfastBody.bodyToJoint[0] << "? " << aSdfastBody.bodyToJoint[1] <<
			"? " << aSdfastBody.bodyToJoint[2] << "? " << "   inbtojoint = " << aSdfastBody.inboardToJoint[0] <<
			"? " << aSdfastBody.inboardToJoint[1] << "? " << aSdfastBody.inboardToJoint[2] << "?" << endl;

		aStream << aJointInfo.pinString;
		aStream << "prescribed =" << aJointInfo.prescribedString;
	}
	else
	{
		aStream << "# loop joint" << endl;
		aStream << "body = " << aJointInfo.outbname << " inb = " << aJointInfo.inbname << " joint = " <<
			getDpJointName(aJointInfo.type, aJointInfo.direction) << endl;

		aStream << "bodytojoint = " << aSdfastBody.bodyToJoint[0] << " " << aSdfastBody.bodyToJoint[1] << " " <<
			aSdfastBody.bodyToJoint[2] << "   inbtojoint = " << aSdfastBody.inboardToJoint[0] << " " <<
			aSdfastBody.inboardToJoint[1] << " " << aSdfastBody.inboardToJoint[2] << endl;
	}

	aStream << endl << endl;
}

void SdfastFileWriter::writeModelHeaderFile(const string& aFileName)
{
   int i;
	ofstream out;
	DofInfo* dof = NULL;
	JointInfo* joint = NULL;

	if (!_initialized)
		initialize();

	string fullPath = _outputFolder + '/' + aFileName;

	out.open(fullPath.c_str());
	out.setf(ios::fixed);
	out.precision(10);

	out << "/*******************************************************************************" << endl << endl;
	out << "  " << aFileName << endl << endl;
	out << "  " << getCurrentTimeString() << endl << endl;
	out << "*******************************************************************************/" << endl << endl;

	out << "/*********** Joints ************/" << endl;
	for (i = 0; i < _jointOrder.getSize(); i++)
	{
		out << "#define ";
		out.width(22);
		out << _joints[_jointOrder[i]].name << " ";
		out.width(2);
		out << i << endl;
	}

   out << endl << "/************* Qs **************/" << endl;
	for (i = 0; i < _numQs; i++)
	{
		dof = findNthSdfastQ(i, joint);
		if (dof && joint)
		{
			out << "#define ";
			out.width(22);
			out << dof->name << " ";
			out.width(2);
			out << i << "   /* = sdindx(" << joint->name << "," << dof->axis << ") */" << endl;
		}
	}

   out << endl << "/******* Constrained Qs ********/" << endl;
	for (i = 0; i < _numQs; i++)
	{
		dof = findNthSdfastQ(i, joint);
      if (dof && dof->constrained)
		{
			out << "#define ";
			out.width(22);
			out << dof->constraintName << " ";
			out.width(2);
			out << dof->errorNumber << endl;
		}
   }

   out << endl << "/******** Body Segments ********/" << endl;
	for (i = 0; i < _sdfastBodies.getSize(); i++)
	{
		out << "#define ";
		out.width(22);
		out << _sdfastBodies[i].name << " ";
		out.width(2);
		out << i - 1 << endl;
	}

	out.close();

}

/* This function writes the user-defined kinematic constraint functions to
 * the top of the SD/FAST C file (sdfor.c).
*/

void SdfastFileWriter::writeSdfastConstraintData(ofstream& out)
{
   int i, j;
   bool constraintsExist = false;

   /* First see if there are any user-defined constraint functions.
    * If not, just return.
   */
	for (i = 0; i < _jointOrder.getSize(); i++)
	{
		JointInfo* jointInfo = &_joints[_jointOrder[i]];
		for (j = 0; j < jointInfo->numDofs; j++)
      {
			if (jointInfo->dofs[j].modelDof->getCoordinate() != NULL && jointInfo->dofs[j].constrained)
			{
				constraintsExist = true;
				break;
			}
      }
      if (constraintsExist)
         break;
   }

   if (!constraintsExist)
      return;

#if 0
	// NO LONGER NEED TO HARDCODE VALUES INTO sdfor.c FILE BECAUSE VALUES WILL BE SET USING setJointConstraintFunctions
	out << "/* The following spline-function data points are copied directly from the" << endl;
   out << " * SIMM joints file." << endl << " */" << endl << endl;

	for (i = 0; i < _jointOrder.getSize(); i++)
	{
		JointInfo* jointInfo = &_joints[_jointOrder[i]];
		for (j = 0; j < jointInfo->numDofs; j++)
      {
			if (jointInfo->dofs[j].modelDof->getCoordinate() != NULL && jointInfo->dofs[j].constrained)
			{
				Function* func = jointInfo->dofs[j].modelDof->getFunction();
				NatCubicSpline* cubicSpline = dynamic_cast<NatCubicSpline*>(func);
				if (cubicSpline)
				{
					out << "static double " << jointInfo->dofs[j].name << "_data[][2] =" << endl << "{" << endl;
					for (int k = 0; k < cubicSpline->getNumberOfPoints(); k++)
					{
						out << "{" << cubicSpline->getX()[k] << ", " << cubicSpline->getY()[k] << "}";
						if (k == cubicSpline->getNumberOfPoints() - 1)
							out << endl << "};" << endl << endl;
						else
							out << "," << endl;
					}
				}
			}
		}
   }
#endif

	for (i = 0; i < _jointOrder.getSize(); i++)
	{
		JointInfo* jointInfo = &_joints[_jointOrder[i]];
		for (j = 0; j < jointInfo->numDofs; j++)
      {
			if (jointInfo->dofs[j].modelDof->getCoordinate() != NULL && jointInfo->dofs[j].constrained)
				out << "static dpSplineFunction " << jointInfo->dofs[j].name << "_func;" << endl;
      }
   }
}

void SdfastFileWriter::writeSdfastQRestraintData(ofstream& out)
{
	Function* func;

   for (int i = 0; i < _coordinates.getSize(); i++)
   {
		SimmCoordinate* sc = dynamic_cast<SimmCoordinate*>(_coordinates[i].modelCoordinate);
		if (sc)
		{
			if ((func = sc->getRestraintFunction()) && sc->isRestraintActive())
			{
				NatCubicSpline* cubicSpline = dynamic_cast<NatCubicSpline*>(func);
				if (cubicSpline)
				{
					out << endl << "static double q_restraint_func" << _numRestraintFunctions + 1 << "_data[][2] = {" << endl;
					for (int k = 0; k < cubicSpline->getNumberOfPoints(); k++)
					{
						out << "{" << cubicSpline->getX()[k] << ", " << cubicSpline->getY()[k] << "}";
						if (k == cubicSpline->getNumberOfPoints() - 1)
							out << endl << "};" << endl << endl;
						else
							out << "," << endl;
					}
					_coordinates[i].restraintFuncNum = _numRestraintFunctions++;
				}
			}
			else
			{
				if ((func = sc->getMinRestraintFunction()))
				{
					NatCubicSpline* cubicSpline = dynamic_cast<NatCubicSpline*>(func);
					if (cubicSpline)
					{
						out << endl << "static double q_restraint_func" << _numRestraintFunctions + 1 << "_data[][2] = {" << endl;
						for (int k = 0; k < cubicSpline->getNumberOfPoints(); k++)
						{
							out << "{" << cubicSpline->getX()[k] << ", " << cubicSpline->getY()[k] << "}";
							if (k == cubicSpline->getNumberOfPoints() - 1)
								out << endl << "};" << endl << endl;
							else
								out << "," << endl;
						}
						_coordinates[i].minRestraintFuncNum = _numRestraintFunctions++;
					}
				}

				if ((func = sc->getMaxRestraintFunction()))
				{
					NatCubicSpline* cubicSpline = dynamic_cast<NatCubicSpline*>(func);
					if (cubicSpline)
					{
						out << endl << "static double q_restraint_func" << _numRestraintFunctions + 1 << "_data[][2] = {" << endl;
						for (int k = 0; k < cubicSpline->getNumberOfPoints(); k++)
						{
							out << "{" << cubicSpline->getX()[k] << ", " << cubicSpline->getY()[k] << "}";
							if (k == cubicSpline->getNumberOfPoints() - 1)
								out << endl << "};" << endl << endl;
							else
								out << "," << endl;
						}
						_coordinates[i].maxRestraintFuncNum = _numRestraintFunctions++;
					}
				}
			}
		}
	}

   if (_numRestraintFunctions > 0)
      out << endl << "static dpSplineFunction q_restraint_func[" << _numRestraintFunctions << "];" << endl << endl;

}

void SdfastFileWriter::writeSdfastQRestraintFunctions(ofstream& out)
{
   out << "/* INIT_Q_RESTRAINT_FUNCTIONS: this routine initializes the restraint" << endl;
   out << " * functions which are used to keep the Qs from exceeding their ranges of motion.\n */" << endl << endl;

   out << "void init_q_restraint_functions(void)" << endl << "{" << endl << "   int i, numpts;" << endl << endl;

   if (_numRestraintFunctions < 1)
   {
      out << "   /* There are no user-defined restraints in this model */" << endl << endl;
   }
   else
   {
      for (int i = 0; i < _numRestraintFunctions; i++)
      {
         out << "   numpts = sizeof(q_restraint_func" << i + 1 << "_data)/(sizeof(double)*2);" << endl;
         out << "   (void)malloc_function(&q_restraint_func[" << i << "],numpts);" << endl;
         out << "   q_restraint_func[" << i << "].numpoints = numpts;" << endl;
         out << "   for (i=0; i<numpts; i++)\n   {" << endl;
         out << "      q_restraint_func[" << i << "].x[i] = q_restraint_func" << i + 1 << "_data[i][0];" << endl;
         out << "      q_restraint_func[" << i << "].y[i] = q_restraint_func" << i + 1 << "_data[i][1];" << endl;
         out << "   }" << endl;
         out << "   calc_spline_coefficients(&q_restraint_func[" << i << "]);" << endl << endl;
      }
   }

   out << "}" << endl << endl << endl;
}

SdfastFileWriter::CoordinateInfo* SdfastFileWriter::getCoordinateInfo(const AbstractCoordinate* aCoord) const
{
	for (int i = 0; i < _coordinates.getSize(); i++)
		if (_coordinates.get(i).modelCoordinate == aCoord)
			return &_coordinates.get(i);

	return NULL;
}

void SdfastFileWriter::writeSdfastQInitCode(ofstream& out)
{
	DofInfo* dof;
	JointInfo* joint;

   out << "/* INIT_QS: this routine initializes the array of structures" << endl;
   out << " * that hold information about the Qs (gencoords)." << endl;
   out << " */" << endl << endl;
   
   out << "void init_qs(void)\n{\n\n   int i;" << endl << endl;
   
   out << "   sdm.q = (dpQStruct*)simm_malloc(sdm.nq*sizeof(dpQStruct));" << endl;
   
	for (int i = 0; i < _numQs; i++)
	{
		dof = findNthSdfastQ(i, joint);
		if (dof && joint)
		{
			const AbstractCoordinate* coord = dof->modelDof->getCoordinate();
			CoordinateInfo* coordInfo = getCoordinateInfo(coord);

			out << "   mstrcpy(&sdm.q[" << dof->name << "].name,\"" << dof->name << "\");" << endl;
			if (dof->fixed)
			{
				out << "   sdm.q[" << dof->name << "].type = dpFixedQ;" << endl;
			}
			else if (!dof->constrained)
			{
				/* Locked gencoords are modeled as fixed Qs (as of SIMM 4.1.1). */
				if (coord && coord->getLocked())
					out << "   sdm.q[" << dof->name << "].type = dpFixedQ;" << endl;
				else
					out << "   sdm.q[" << dof->name << "].type = dpUnconstrainedQ;" << endl;
			}
			else
			{
				out << "   sdm.q[" << dof->name << "].type = dpConstrainedQ;" << endl;
			}
			out << "   sdm.q[" << dof->name << "].joint = " << joint->name << ";" << endl;
			out << "   sdm.q[" << dof->name << "].axis = " << dof->axis << ";" << endl;
			out << "   // initial_value will now be set using setCoordinateInitialValues" << endl;
			out << "   // sdm.q[" << dof->name << "].initial_value = " << dof->initialValue << ";" << endl;
			out << "   sdm.q[" << dof->name << "].initial_velocity = 0.0;" << endl;
			if (!dof->constrained && !dof->fixed)
			{
				out << "   sdm.q[" << dof->name << "].range_start = " << dof->modelDof->getCoordinate()->getRangeMin() << ";" << endl;
				out << "   sdm.q[" << dof->name << "].range_end = " << dof->modelDof->getCoordinate()->getRangeMax() << ";" << endl;
			}
			else
			{
				out << "   sdm.q[" << dof->name << "].range_start = -99999.9;" << endl;
				out << "   sdm.q[" << dof->name << "].range_end = 99999.9;" << endl;
			}
			if (dof->constrained || dof->fixed)
			{
				out << "   sdm.q[" << dof->name << "].restraint_func = NULL;" << endl;
				out << "   sdm.q[" << dof->name << "].min_restraint_func = NULL;" << endl;
				out << "   sdm.q[" << dof->name << "].max_restraint_func = NULL;" << endl;
				out << "   sdm.q[" << dof->name << "].function_active = dpNo;" << endl;
			}
			else if (coordInfo)
			{
				if (coordInfo->restraintFuncNum != -1)
				{
					out << "   sdm.q[" << dof->name << "].restraint_func = &q_restraint_func[" <<
						    coordInfo->restraintFuncNum << "];" << endl;
					out << "   sdm.q[" << dof->name << "].min_restraint_func = NULL;" << endl;
					out << "   sdm.q[" << dof->name << "].max_restraint_func = NULL;" << endl;
					if (coord->isRestraintActive())
						out << "   sdm.q[" << dof->name << "].function_active = dpYes;" << endl;
					else
						out << "   sdm.q[" << dof->name << "].function_active = dpNo;" << endl;
				}
				else
				{
					out << "   sdm.q[" << dof->name << "].restraint_func = NULL;" << endl;
					if (coordInfo->minRestraintFuncNum == -1)
						out << "   sdm.q[" << dof->name << "].min_restraint_func = NULL;" << endl;
					else
						out << "   sdm.q[" << dof->name << "].min_restraint_func = &q_restraint_func[" <<
						       coordInfo->minRestraintFuncNum << "];" << endl;
					if (coordInfo->maxRestraintFuncNum == -1)
						out << "   sdm.q[" << dof->name << "].max_restraint_func = NULL;" << endl;
					else
						out << "   sdm.q[" << dof->name << "].max_restraint_func = &q_restraint_func[" <<
						       coordInfo->maxRestraintFuncNum << "];" << endl;
					out << "   sdm.q[" << dof->name << "].function_active = dpNo;" << endl;
				}
			}
			if (dof->constrained)
			{
				DofInfo* indDof = findUnconstrainedSdfastDof(coord);
				if (indDof)
				{
					out << "   sdm.q[" << dof->name << "].constraint_func = &" << dof->name << "_func;" << endl;
					out << "   sdm.q[" << dof->name << "].constraint_num = " << dof->constraintName << ";" << endl;
					out << "   sdm.q[" << dof->name << "].q_ind = " << indDof->name << ";" << endl;
				}
			}
			else
			{
				out << "   sdm.q[" << dof->name << "].constraint_func = NULL;" << endl;
				out << "   sdm.q[" << dof->name << "].constraint_num = -1;" << endl;
				out << "   sdm.q[" << dof->name << "].q_ind = -1;" << endl;
			}
			if (dof->fixed || dof->constrained)
			{
				out << "   sdm.q[" << dof->name << "].output = dpNo;" << endl;
				out << "   sdm.q[" << dof->name << "].pd_stiffness = 0.0;" << endl;
			}
			else
			{
				out << "   sdm.q[" << dof->name << "].output = dpYes;" << endl;
				out << "   sdm.q[" << dof->name << "].pd_stiffness = " << "0.0" /*coord->getPDStiffness() TODO */ << ";" << endl;
			}
			out << "   sdm.q[" << dof->name << "].torque = 0.0;" << endl;
			out << "" << endl;
		}
   }
   
   out << "   for (i=0, sdm.num_gencoords=0; i<sdm.nq; i++)" << endl;
   out << "      if (sdm.q[i].type == dpUnconstrainedQ)" << endl;
   out << "         sdm.num_gencoords++;" << endl << endl;
   
   out << "   check_for_sderror(\"INIT_QS\");" << endl;
   
   out << "}" << endl << endl << endl;
}

void SdfastFileWriter::writeSdfastConstraintCode(ofstream& out)
{
#if 0
	// VALUES WILL BE SET USING setJointConstraintFunctions INSTEAD OF USING THIS FUNCTION
	
	out << "/* INIT_JOINT_FUNCTIONS: this routine initializes the constraint functions" << endl;
	out << " * for the joints which have user-defined constraints.\n */" << endl << endl;

	out << "void init_joint_functions(void)\n{\n\n   int i, numpts;" << endl << endl;

	int count = 0;
	for (int i = 0; i < _joints.getSize(); i++)
	{
		JointInfo &joint = _joints.get(i);
		DofInfo *dofs = joint.dofs;

		for (int j = 0; j < joint.numDofs; j++)
		{
			if (dofs[j].modelDof->getCoordinate() != NULL && dofs[j].constrained)
			{
				out << "   numpts = sizeof(" << dofs[j].name << "_data)/(sizeof(double)*2);" << endl;
				out << "   (void)malloc_function(&" << dofs[j].name << "_func,numpts);" << endl;
				out << "   " << dofs[j].name << "_func.numpoints = numpts;" << endl;
				out << "   for (i=0; i<numpts; i++)\n   {" << endl;
				out << "      " << dofs[j].name << "_func.x[i] = " << dofs[j].name << "_data[i][0];" << endl;
				out << "      " << dofs[j].name << "_func.y[i] = " << dofs[j].name << "_data[i][1];" << endl;
				out << "   }" << endl;
				out << "   calc_spline_coefficients(&" << dofs[j].name << "_func);" << endl << endl;
				count++;
			}
		}
	}
   
	if (count == 0)
		out << "   /* There are no user-defined constraints in this model */" << endl << endl;

	out << "}" << endl << endl << endl;
#endif
}

void SdfastFileWriter::writeSdfastWrapObjects(ofstream& out)
{
	//int i, j, k;
	//WrapObject* wo;

	out << "void init_wrap_objects(void)" << endl;
	out << "{" << endl << endl;


	//if (ms->num_wrap_objects < 1)
	{
		out << "   /* Wrap objects are handled by the native OpenSim code, so */" << endl;
		out << "   /* they are not exported to the Pipeline source code. */" << endl << endl;
		out << "  sdm.num_wrap_objects = 0;" << endl;
		out << "  sdm.wrap_object = NULL;" << endl << endl;
		out << "}" << endl << endl;
	}
#if 0
	else
	{
		fprintf(*fp, "   int i;\n\n");

		fprintf(*fp, "   /* The from_local_xforms in this array of wrap objects are expressed relative to\n");
		fprintf(*fp, "    * the origin of the SIMM segment reference frame. The to_local_xforms are filled in with\n");
		fprintf(*fp, "    * zeros-- those transforms are calculated later, directly from the to_local_xforms.\n");
		fprintf(*fp, "    */\n");

		fprintf(*fp,"   dpWrapObject wrap_object[] = {\n");
		for (i = 0; i < ms->num_wrap_objects; i++)
		{
			wo = &ms->wrapobj[i];

			strcpy(buffer, wo->name);
			convert_string(buffer, yes);

			fprintf(*fp,"   {\"%s\", %s, %s, %d, %d, %.10lf, %.10lf, %.10lf, %.10lf, %d, %d,\n",
				buffer, get_wrap_name(wo->wrap_type), (wo->active == yes) ? "dpYes" : "dpNo",
				wo->wrap_algorithm, get_sd_seg_num(ms->segment[wo->segment].name),
				wo->radius.xyz[0], wo->radius.xyz[1], wo->radius.xyz[2], wo->height,
				wo->wrap_axis, wo->wrap_sign);

			fprintf(*fp,"   {%.10lf, %.10lf, %.10lf, %.10lf,\n", wo->from_local_xform[0][0], wo->from_local_xform[0][1],
				wo->from_local_xform[0][2], wo->from_local_xform[0][3]);
			fprintf(*fp,"    %.10lf, %.10lf, %.10lf, %.10lf,\n", wo->from_local_xform[1][0], wo->from_local_xform[1][1],
				wo->from_local_xform[1][2], wo->from_local_xform[1][3]);
			fprintf(*fp,"    %.10lf, %.10lf, %.10lf, %.10lf,\n", wo->from_local_xform[2][0], wo->from_local_xform[2][1],
				wo->from_local_xform[2][2], wo->from_local_xform[2][3]);
			fprintf(*fp,"    %.10lf, %.10lf, %.10lf, %.10lf},\n", wo->from_local_xform[3][0], wo->from_local_xform[3][1],
				wo->from_local_xform[3][2], wo->from_local_xform[3][3]);

			if (i == ms->num_wrap_objects - 1)
				fprintf(*fp,"   }\n");
			else
				fprintf(*fp,"   },\n");
		}
		fprintf(*fp,"   };\n\n");

		fprintf(*fp, "   sdm.num_wrap_objects = %d;\n\n", ms->num_wrap_objects);

		fprintf(*fp, "   sdm.wrap_object = (dpWrapObject*)simm_malloc(sdm.num_wrap_objects * sizeof(dpWrapObject));\n\n");

		fprintf(*fp, "   /* Copy the wrap objects into the sdm structure, and adjust the transforms so that they\n");
		fprintf(*fp, "    * are relative to the mass center of the segment. Then compute the to_local_xform as the\n");
		fprintf(*fp, "    * inverse of the from_local_xform.\n");
		fprintf(*fp, "    */\n");

		fprintf(*fp, "   for (i = 0; i < sdm.num_wrap_objects; i++)\n");
		fprintf(*fp, "   {\n");
		fprintf(*fp, "      sdm.wrap_object[i] = wrap_object[i];\n");
		fprintf(*fp, "      mstrcpy(&sdm.wrap_object[i].name, wrap_object[i].name);\n");
		fprintf(*fp, "      sdm.wrap_object[i].from_local_xform[3][XX] -= sdm.body_segment[sdm.wrap_object[i].segment+1].mass_center[XX];\n");
		fprintf(*fp, "      sdm.wrap_object[i].from_local_xform[3][YY] -= sdm.body_segment[sdm.wrap_object[i].segment+1].mass_center[YY];\n");
		fprintf(*fp, "      sdm.wrap_object[i].from_local_xform[3][ZZ] -= sdm.body_segment[sdm.wrap_object[i].segment+1].mass_center[ZZ];\n");
		fprintf(*fp, "      invert_4x4transform(sdm.wrap_object[i].from_local_xform, sdm.wrap_object[i].to_local_xform);\n");
		fprintf(*fp, "   }\n");

		fprintf(*fp,"}\n\n");
	}
#endif
}

void SdfastFileWriter::writeSdfastConstraintObjects(ofstream& out)
{
	//int i, j, k, totalNumPts = 0;
	//ConstraintObject* co;
	//ConstraintPoint* pt;

	out << "void init_constraint_objects(void)" << endl;
	out << "{" << endl << endl;

	//if (ms->num_constraint_objects < 1)
	{
		out << "   /* There are no constraint objects in this model. */" << endl << endl;
		out << "  sdm.num_constraint_objects = 0;" << endl;
		out << "  sdm.constraint_object = NULL;" << endl;
		out << "}" << endl << endl;
	}
#if 0
	else
	{
		fprintf(*fp, "   int i, j, index, constraint_num, num_points, sdm_seg_index;\n\n");

		fprintf(*fp, "   /* The from_local_xforms in this array of constraint objects are expressed relative to\n");
		fprintf(*fp, "    * the origin of the SIMM segment reference frame. The to_local_xforms are filled in with\n");
		fprintf(*fp, "    * zeros-- those transforms are calculated later, directly from the to_local_xforms.\n");
		fprintf(*fp, "    */\n");

		fprintf(*fp,"   dpConstraintObject constraint_object[] = {\n");
		for (i = 0; i < ms->num_constraint_objects; i++)
		{
			co = &ms->constraintobj[i];

			strcpy(buffer, co->name);
			convert_string(buffer, yes);

			fprintf(*fp, "   {\"%s\", %s, ", buffer, get_constraint_name(co->constraintType));
			fprintf(*fp, "    %s, %d, \n", (co->active == yes) ? "dpYes" : "dpNo", get_sd_seg_num(ms->segment[co->segment].name));
			fprintf(*fp, "     %.10lf, %.10lf, %.10lf, %.10lf, ",
				co->radius.xyz[0], co->radius.xyz[1], co->radius.xyz[2], co->height);
			fprintf(*fp, "    %d, %d, \n", co->constraintAxis, co->constraintSign);

			fprintf(*fp, "    %d, %d,", co->numPoints, totalNumPts);

			fprintf(*fp,"   {%.10lf, %.10lf, %.10lf, %.10lf,\n", co->from_local_xform[0][0], co->from_local_xform[0][1],
				co->from_local_xform[0][2], co->from_local_xform[0][3]);
			fprintf(*fp,"    %.10lf, %.10lf, %.10lf, %.10lf,\n", co->from_local_xform[1][0], co->from_local_xform[1][1],
				co->from_local_xform[1][2], co->from_local_xform[1][3]);
			fprintf(*fp,"    %.10lf, %.10lf, %.10lf, %.10lf,\n", co->from_local_xform[2][0], co->from_local_xform[2][1],
				co->from_local_xform[2][2], co->from_local_xform[2][3]);
			fprintf(*fp,"    %.10lf, %.10lf, %.10lf, %.10lf}", co->from_local_xform[3][0], co->from_local_xform[3][1],
				co->from_local_xform[3][2], co->from_local_xform[3][3]);

			if (co->constraintType == constraint_plane)
			{
				fprintf(*fp, ",\n   {%.8lf, %.8lf, %.8lf, %.8lf}", co->plane.a,
					co->plane.b, co->plane.c, co->plane.d);
			}
			fprintf(*fp, "\n");

			if (i == ms->num_constraint_objects - 1)
				fprintf(*fp,"   }\n");
			else
				fprintf(*fp,"   },\n");
			totalNumPts += ms->constraintobj[i].numPoints;
		}
		fprintf(*fp,"   };\n\n");

		fprintf(*fp,"   dpConstraintPoint constraint_points[] = {\n");
		for (i = 0, k = 0; i < ms->num_constraint_objects; i++)
		{
			co = &ms->constraintobj[i];

			for (j = 0; j < co->numPoints; j++)
			{
				pt = &ms->constraintobj[i].points[j];
				strcpy(buffer, pt->name);
				convert_string(buffer, yes);

				/* print name and constrainttype */
				fprintf(*fp, "   {\"%s\", ", buffer);
				fprintf(*fp, "    %d, ", get_sd_seg_num(ms->segment[pt->segment].name));
				fprintf(*fp, "    %.10lf, %.10lf, %.10lf, %.10lf",
					pt->offset[0], pt->offset[1], pt->offset[2], pt->weight);

				if (k == totalNumPts - 1)
					fprintf(*fp," }\n");
				else
					fprintf(*fp," },\n");
				k++;
			}
		}
		fprintf(*fp,"   };\n\n");

		fprintf(*fp, "   sdm.num_constraint_objects = %d;\n\n", ms->num_constraint_objects);

		fprintf(*fp, "   sdm.constraint_object = (dpConstraintObject*)simm_malloc(sdm.num_constraint_objects * sizeof(dpConstraintObject));\n\n");

		fprintf(*fp, "   num_points = sizeof(constraint_points) / sizeof(dpConstraintPoint);\n");
		fprintf(*fp, "   constraint_num = sdm.num_user_constraints - num_points;\n\n");

		fprintf(*fp, "   /* Copy the constraint objects into the sdm structure, and adjust the point offsets so that they\n");
		fprintf(*fp, "    * are relative to the mass center of the segment. Then compute the to_local_xform as the\n");
		fprintf(*fp, "    * inverse of the from_local_xform.\n");
		fprintf(*fp, "    */\n");

		fprintf(*fp, "   for (i = 0; i < sdm.num_constraint_objects; i++)\n");
		fprintf(*fp, "   {\n");
		fprintf(*fp, "      sdm.constraint_object[i] = constraint_object[i];\n");
		fprintf(*fp, "      mstrcpy(&sdm.constraint_object[i].name, constraint_object[i].name);\n");
		fprintf(*fp, "      sdm.constraint_object[i].from_local_xform[3][XX] -= sdm.body_segment[sdm.constraint_object[i].segment+1].mass_center[XX];\n");
		fprintf(*fp, "      sdm.constraint_object[i].from_local_xform[3][YY] -= sdm.body_segment[sdm.constraint_object[i].segment+1].mass_center[YY];\n");
		fprintf(*fp, "      sdm.constraint_object[i].from_local_xform[3][ZZ] -= sdm.body_segment[sdm.constraint_object[i].segment+1].mass_center[ZZ];\n");
		fprintf(*fp, "      invert_4x4transform(sdm.constraint_object[i].from_local_xform, sdm.constraint_object[i].to_local_xform);\n");
		fprintf(*fp, "      sdm.constraint_object[i].points = (dpConstraintPoint*)simm_malloc(sdm.constraint_object[i].numPoints * sizeof(dpConstraintPoint));\n");
		fprintf(*fp, "      index = sdm.constraint_object[i].ptIndex;\n");
		fprintf(*fp, "      for (j = 0; j < sdm.constraint_object[i].numPoints; j++)\n");
		fprintf(*fp, "      {\n");
		fprintf(*fp, "         /* adjust the offset to be w.r.t. the mass center, rather than the origin. */\n");
		fprintf(*fp, "         sdm_seg_index = constraint_points[index].segment + 1;\n");
		fprintf(*fp, "         sdvsub(constraint_points[index].offset, sdm.body_segment[sdm_seg_index].mass_center,\n");
		fprintf(*fp, "            sdm.constraint_object[i].points[j].offset);\n");
		fprintf(*fp, "         sdm.constraint_object[i].points[j].segment = constraint_points[index].segment;\n");
		fprintf(*fp, "         sdm.constraint_object[i].points[j].weight = constraint_points[index].weight;\n");
		fprintf(*fp, "         mstrcpy(&sdm.constraint_object[i].points[j].name, constraint_points[index].name);\n");
		fprintf(*fp, "         sdm.constraint_object[i].points[j].constraint_num = constraint_num++;\n");
		fprintf(*fp, "         index++;\n");
		fprintf(*fp, "      }\n");
		fprintf(*fp, "   }\n");

		fprintf(*fp, "}\n\n");
	}
#endif
}

void SdfastFileWriter::writeModelSourceFile(const string& aFileName, const string& aModelHeaderFileName)
{
	ofstream out;
	string fullPath = _outputFolder + '/' + aFileName;

	if (!_initialized)
		initialize();

	out.open(fullPath.c_str());
	out.setf(ios::fixed);
	out.precision(10);

	out << "/*******************************************************************************" << endl << endl;
	out << "   " << aFileName << endl << endl;
	out << "   Created by NMBLTS (from model " << _model->getName() << ")" << endl << endl;
	out << "   Time of creation: " << getCurrentTimeString() << endl << endl;
	out << "   Description: This file contains the routines needed to perform a forward" << endl;
	out << "      dynamics simulation of an SD/FAST model. The specific routines that it" << endl;
	out << "      contains depend on the SIMM model from which this code was generated." << endl << endl;
	out << "*******************************************************************************/" << endl << endl;
	out << "#include \"universal.h\"" << endl;
	out << "#include \"" << aModelHeaderFileName << "\"" << endl << endl;

	out << "/*************** DEFINES (for this file only) *********************************/" << endl;

	out << "#define BAUMGARTE_STAB 20" << endl << endl;
	out << "/* Defines for the joints, Qs, and body segments are now found in " << aModelHeaderFileName << " */" << endl << endl;

	out << "\n\n/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/" << endl << endl;

	writeSdfastConstraintData(out);

	writeSdfastQRestraintData(out);

	out << "\n\n/**************** GLOBAL VARIABLES (used in only a few files) *****************/" << endl;
	out << "extern dpModelStruct sdm;" << endl << endl << endl;

	out << "/*************** EXTERNED VARIABLES (declared in another file) ****************/" << endl << endl << endl << endl;

	out << "/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/" << endl << endl << endl;

	writeSdfastQInitCode(out);

	writeSdfastQRestraintFunctions(out);

	writeSdfastConstraintCode(out);

	writeSdfastWrapObjects(out);

	writeSdfastConstraintObjects(out);

	out.close();
}

void SdfastFileWriter::writeSimulationModelFile(const string& aFileName)
{
	IO::SetPrecision(8);

	if (!_initialized)
		initialize();

	if (_simulationModel)
		_simulationModel->print(aFileName);
}

void SdfastFileWriter::identifySdfastType(AbstractJoint& aJoint, JointInfo& aInfo)
{
   int i, zero = 0;
   int numConstantRotations = 0, numFunctionRotations = 0;
	int numRealConstantRotations = 0, numRealFunctionRotations = 0;
   int numConstantTranslations = 0, numFunctionTranslations = 0;

   /* Give the joint a one-token name. In most cases, this is just
    * the user-given name of the joint. However, if that name has special
    * characters in it (e.g. -), those characters must be removed. Also,
    * if the name starts with a number, then an underscore is prepended.
    */
	aInfo.name = aJoint.getName();
   convertString(aInfo.name, true);

	DofSet* dofs = aJoint.getDofSet();

	/* If the joint has no DOFs it cannot be converted to an SD/FAST joint
	 * (even a weld joint needs a dof to hold gencoord information because
	 * tree welds are modeled as fixed pins).
	 */
	if (dofs->getSize() < 1)
	{
		aInfo.type = dpUnknownJoint;
		return;
	}

   /* Constant, non-zero rotations are not supported in SD/FAST. So to
    * implement them, you treat them as gencoords, and then prescribe
    * their motion to be constant (like weld joints).
	 *
    * For the purposes of optimizing the pipeline code, you want to remove
    * from the model segments that:
    *   (1) are leaf nodes (are used in only one joint and are not ground),
    *   (2) have no gencoords in their joint,
    *   (3) have zero mass, and
    *   (4) have no muscle points, wrap objects, or constraint objects.
    * Such segments are often used for holding markers, and so are not needed
    * in the dynamic simulation. They are removed by setting their joint's
	 * type to dpSkippable. Because constant, non-zero rotations are modeled
	 * as functions in SD/FAST, the numRealConstantRotations holds the number
	 * of "real" constant rotations so it can be determined if the joint is
	 * dpSkippable or not.
    */
	for (i = 0; i < dofs->getSize(); i++)
	{
		if (dofs->get(i)->getMotionType() == AbstractDof::Translational)
		{
			if (dofs->get(i)->getCoordinate() == NULL)
				numConstantTranslations++;
			else
				numFunctionTranslations++;
		}
		else
		{
			if (dofs->get(i)->getCoordinate() == NULL)
			{
				numRealConstantRotations++;
				if (EQUAL_WITHIN_ERROR(dofs->get(i)->getValue(), 0.0))
					numConstantRotations++;
				else
					numFunctionRotations++;
			}
			else
			{
				numFunctionRotations++;
				numRealFunctionRotations++;
			}
		}
   }

	/* An SD/FAST-compatible joint is one with:
	 *  1. no more than 3 translation DOFs
	 *  2. no 2 translation DOFs along the same axis (X, Y, Z)
	 *  3. no more than 3 rotation DOFs
	 *  4. no rotations between the translation DOFs
	 */
	if (!isJointSdfastCompatible(aInfo.modelJoint))
	{
		aInfo.type = dpUnknownJoint;
		return;
	}

   /* If the joint has no degrees of freedom, check to see if one of
    * the bodies is a leaf node. If it is, and the body does not have
	 * any elements needed for dynamics, mark the joint as dpSkippable
	 * so it will not be included in the dynamics.
    */
   if (numRealFunctionRotations == 0 && numFunctionTranslations == 0)
   {
		AbstractBody* leafBody = _model->getDynamicsEngine().getLeafBody(&aJoint);

		if (leafBody && leafBody->getMass() < TINY_NUMBER
			// && leafBody->getNumWrapObjects() == 0  TODO
			)
		{
			aInfo.type = dpSkippable;
			// Figure out if the leaf body is the parent or child body in this
			// joint, and set it to skippable.
			if (aJoint.getParentBody() == leafBody)
				_modelBodies[aInfo.parentBodyIndex].skippable = true;
			else if (aJoint.getChildBody() == leafBody)
				_modelBodies[aInfo.childBodyIndex].skippable = true;
			return;
		}
   }

   if (numFunctionRotations == 0 && numFunctionTranslations == 0)
	{
      aInfo.type = dpWeld;
		return;
	}

	if (numFunctionRotations == 3 && numFunctionTranslations == 3)
   {
      /* In SD/FAST, bushing joints use the axes of rotation as
       * the axes of translation as well. Thus for a SIMM joint
       * to properly convert to an SD/FAST bushing, the rotation
       * axes must be X, Y, and Z, since the translations are
       * always w.r.t. these axes.
       */
      if (!aJoint.hasXYZAxes())
         aInfo.type = dpUnknownJoint;
      else if (dofs->get(zero)->getMotionType() == AbstractDof::Translational) // translation first
         aInfo.type = dpBushing;
      else if (dofs->get(zero)->getMotionType() == AbstractDof::Rotational) // rotation first
         aInfo.type = dpReverseBushing;
      else
         aInfo.type = dpUnknownJoint;
		return;
   }

   if (numFunctionRotations == 1 && numFunctionTranslations == 0)
   {
      /* If the one rotation happens after the translations,
       * then this is a [normal] pin joint. If it happens
       * before, then this is a reverse pin joint, and the
       * translations have to be added to the other body segment.
       */
		for (i = 0; i < dofs->getSize(); i++)
		{
			if (dofs->get(i)->getMotionType() == AbstractDof::Rotational &&
				 (dofs->get(i)->getCoordinate() != NULL || NOT_EQUAL_WITHIN_ERROR(dofs->get(i)->getValue(), 0.0)))
			{
				if (dofs->get(zero)->getMotionType() == AbstractDof::Translational) // translation first
					aInfo.type = dpPin;
				else if (dofs->get(zero)->getMotionType() == AbstractDof::Rotational) // rotation first
					aInfo.type = dpReversePin;
				else
					aInfo.type = dpUnknownJoint;
				return;
			}
		}

		// Couldn't find the appropriate rotation
		aInfo.type = dpUnknownJoint;
		return;
   }

   if (numFunctionRotations == 0 && numFunctionTranslations == 1)
	{
      aInfo.type = dpSlider;
		return;
	}

   if (numFunctionRotations == 3 && numFunctionTranslations == 0)
   {
      if (dofs->get(zero)->getMotionType() == AbstractDof::Translational) // translation first
         aInfo.type = dpGimbal;
      else if (dofs->get(zero)->getMotionType() == AbstractDof::Rotational) // rotation first
         aInfo.type = dpReverseGimbal;
      else
         aInfo.type = dpUnknownJoint;
		return;
   }

   if (numFunctionRotations == 2 && numFunctionTranslations == 0)
   {
      if (dofs->get(zero)->getMotionType() == AbstractDof::Translational) // translation first
         aInfo.type = dpUniversal;
      else if (dofs->get(zero)->getMotionType() == AbstractDof::Rotational) // rotation first
         aInfo.type = dpReverseUniversal;
      else
         aInfo.type = dpUnknownJoint;
		return;
   }

   if (numFunctionRotations == 1 && numFunctionTranslations == 1)
   {
		int rdIndex = -1, tdIndex = -1;
		AbstractDof* rd = findNthFunctionRotation(aInfo, 0, rdIndex);
		AbstractDof* td = findNthFunctionTranslation(aInfo, 0, tdIndex);

		if (rd && td)
		{
			const double* axis1 = rd->getAxisPtr();
			const double* axis2 = td->getAxisPtr();

			if (axesAreParallel(axis1, axis2, false))
			{
				/* If the [one] rotation happens after the translations,
				 * then this is a normal cylinder joint. If it happens
				 * before, then this is a reverse cylinder joint, and the
				 * translations have to be added to the other body segment.
				 */
				if (dofs->get(zero)->getMotionType() == AbstractDof::Translational) // translation first
					aInfo.type = dpCylindrical;
				else if (dofs->get(zero)->getMotionType() == AbstractDof::Rotational) // rotation first
					aInfo.type = dpReverseCylindrical;
				else
					aInfo.type = dpUnknownJoint;
				return;
			}
		}

		aInfo.type = dpUnknownJoint;
		return;
   }

   if (numFunctionRotations == 1 && numFunctionTranslations == 2)
   {
		int rdIndex = -1, td1Index = -1, td2Index = -1;
		AbstractDof* rd = findNthFunctionRotation(aInfo, 0, rdIndex);
		AbstractDof* td1 = findNthFunctionTranslation(aInfo, 0, td1Index);
		AbstractDof* td2 = findNthFunctionTranslation(aInfo, 1, td2Index);

		if (rd && td1 && td2)
		{
			const double* axis1 = rd->getAxisPtr();
			const double* axis2 = td1->getAxisPtr();
			const double* axis3 = td2->getAxisPtr();

		   /* As long as the rotation axis is not parallel to either of
          * the translation axes, and the translation is not in the
          * middle of the transformation order, this is a valid planar joint.
          */
			if (axesAreParallel(axis1, axis2, true) || axesAreParallel(axis1, axis3, true))
			{
				aInfo.type = dpUnknownJoint;
				return;
			}
			else
			{
				if (dofs->get(zero)->getMotionType() == AbstractDof::Translational) // translation first
					aInfo.type = dpPlanar;
				else if (dofs->get(zero)->getMotionType() == AbstractDof::Rotational) // rotation first
					aInfo.type = dpReversePlanar;
				else
					aInfo.type = dpUnknownJoint;
				return;
			}
		}

		aInfo.type = dpUnknownJoint;
		return;
	}

	aInfo.type = dpUnknownJoint;
}

void SdfastFileWriter::makeSdfastWeld(int aJointIndex, int& rDofCount, int& rConstrainedCount)
{
	/* A [tree] weld joint is implemented as a fixed pin. You don't
	 * know how many DOFs a weld joint has, but you know it has at
	 * least one and that none of them is a function. Use the first
	 * one to hold the fixed pin information.
	 */
	char numString[10];
	sprintf(numString, "%d", rDofCount);
	_joints[aJointIndex].dofs[0].name = fixedString;
	_joints[aJointIndex].dofs[0].name.append(numString);
	_joints[aJointIndex].dofs[0].stateNumber = rDofCount++;
	_joints[aJointIndex].dofs[0].constrained = false;
	_joints[aJointIndex].dofs[0].fixed = true;
	_joints[aJointIndex].dofs[0].joint = aJointIndex;
	_joints[aJointIndex].dofs[0].axis = 0;
	_joints[aJointIndex].dofs[0].initialValue = 0.0;
	_joints[aJointIndex].pinString.append("pin = 1.0? 0.0? 0.0?\n");
	_joints[aJointIndex].prescribedString.append(" 1");
	AbstractCoordinate *newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[0]);

	/* Now create a speed that corresponds to the coordinate. */
	string speedName=AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[0].name);
	addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
}

void SdfastFileWriter::makeSdfastPin(int aJointIndex, int& rDofCount, int& rConstrainedCount)
{
	char buffer[200];
	int rdIndex;
	double axis[3];
	AbstractDof* rotDof = findNthFunctionRotation(_joints[aJointIndex], 0, rdIndex);
	const AbstractCoordinate* coord = rotDof->getCoordinate();

	rotDof->getAxis(axis);

	if (_joints[aJointIndex].direction == SimmStep::inverse)
	{
		axis[0] = -axis[0];
		axis[1] = -axis[1];
		axis[2] = -axis[2];
	}

	sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", axis[0], axis[1], axis[2]);
	_joints[aJointIndex].pinString = buffer;

	if (coord == NULL)
	{
		char numString[10];
		sprintf(numString, "%d", rDofCount); // TODO
		_joints[aJointIndex].dofs[rdIndex].name = fixedString;
		_joints[aJointIndex].dofs[rdIndex].name.append(numString);
		_joints[aJointIndex].dofs[rdIndex].constrained = false;
		_joints[aJointIndex].dofs[rdIndex].fixed = true;
		_joints[aJointIndex].dofs[rdIndex].initialValue = rotDof->getValue();
		_joints[aJointIndex].prescribedString.append(" 1");
	}
	else
	{
		_joints[aJointIndex].dofs[rdIndex].fixed = false;
		if (_joints[aJointIndex].dofs[rdIndex].constrained)
			_joints[aJointIndex].dofs[rdIndex].errorNumber = rConstrainedCount++;
		if (!_joints[aJointIndex].dofs[rdIndex].constrained)
		{
			_joints[aJointIndex].dofs[rdIndex].initialValue = coord->getValue();
			_joints[aJointIndex].prescribedString.append(" 0?");
		}
		else
		{
			_joints[aJointIndex].dofs[rdIndex].initialValue = rotDof->getValue();
			_joints[aJointIndex].prescribedString.append(" 0");
		}
	}

	_joints[aJointIndex].dofs[rdIndex].stateNumber = rDofCount++;
	_joints[aJointIndex].dofs[rdIndex].joint = _joints[aJointIndex].index;
	_joints[aJointIndex].dofs[rdIndex].axis = 0;
	AbstractCoordinate *newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[rdIndex]);

	/* Now create a speed that corresponds to the coordinate. */
	string speedName=AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[rdIndex].name);
	addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
}

void SdfastFileWriter::makeSdfastSlider(int aJointIndex, int& rDofCount, int& rConstrainedCount)
{
	char buffer[200];
	int tdIndex = -1;
	double axis[3];
	AbstractDof* transDof = findNthFunctionTranslation(_joints[aJointIndex], 0, tdIndex);
	const AbstractCoordinate* coord = transDof->getCoordinate();

	transDof->getAxis(axis);

	if (_joints[aJointIndex].direction == SimmStep::inverse)
	{
		axis[0] = -axis[0];
		axis[1] = -axis[1];
		axis[2] = -axis[2];
	}

	sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", axis[0], axis[1], axis[2]);
	_joints[aJointIndex].pinString = buffer;

	_joints[aJointIndex].dofs[tdIndex].stateNumber = rDofCount++;
	_joints[aJointIndex].dofs[tdIndex].fixed = false;
	if (_joints[aJointIndex].dofs[tdIndex].constrained)
		_joints[aJointIndex].dofs[tdIndex].errorNumber = rConstrainedCount++;
	_joints[aJointIndex].dofs[tdIndex].joint = _joints[aJointIndex].index;
	_joints[aJointIndex].dofs[tdIndex].axis = 0;
	if (!_joints[aJointIndex].dofs[tdIndex].constrained)
	{
		_joints[aJointIndex].dofs[tdIndex].initialValue = coord->getValue();
		_joints[aJointIndex].prescribedString.append(" 0?");
	}
	else
	{
		_joints[aJointIndex].dofs[tdIndex].initialValue = transDof->getValue();
		_joints[aJointIndex].prescribedString.append(" 0");
	}

	AbstractCoordinate *newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[tdIndex]);

	/* Now create a speed that corresponds to the coordinate. */
	string speedName=AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[tdIndex].name);
	addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
}

void SdfastFileWriter::makeSdfastPlanar(int aJointIndex, int& rDofCount, int& rConstrainedCount)
{
	char buffer[200];
	int i, axisNum = 0;
	double axis[3];
	AbstractCoordinate *newCoord;
	string speedName;

	if ((_joints[aJointIndex].type == dpPlanar && _joints[aJointIndex].direction == SimmStep::forward) ||
		 (_joints[aJointIndex].type == dpReversePlanar && _joints[aJointIndex].direction == SimmStep::inverse))
	{
		/* Process the translations first. If the joint type was determined
		 * correctly, exactly two of them should be functions.
		 */
		for (i = 0; i < _joints[aJointIndex].numDofs; i++)
		{
			if (_joints[aJointIndex].dofs[i].modelDof->getMotionType() == AbstractDof::Translational &&
				 _joints[aJointIndex].dofs[i].modelDof->getCoordinate() != NULL)
			{
				_joints[aJointIndex].dofs[i].modelDof->getAxis(axis);
				if (_joints[aJointIndex].direction == SimmStep::inverse)
					sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", -axis[0], -axis[1], -axis[2]);
				else
					sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", axis[0], axis[1], axis[2]);
				_joints[aJointIndex].pinString.append(buffer);

				_joints[aJointIndex].dofs[i].stateNumber = rDofCount++;
				_joints[aJointIndex].dofs[i].fixed = false;
				if (_joints[aJointIndex].dofs[i].constrained)
				{
					_joints[aJointIndex].dofs[i].errorNumber = rConstrainedCount++;
					_joints[aJointIndex].prescribedString.append(" 0");
				}
				else
				{
					_joints[aJointIndex].prescribedString.append(" 0?");
				}
				_joints[aJointIndex].dofs[i].initialValue = _joints[aJointIndex].dofs[i].modelDof->getValue();
				_joints[aJointIndex].dofs[i].joint = _joints[aJointIndex].index;
				_joints[aJointIndex].dofs[i].axis = axisNum++;
				newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[i]);

				/* Now create a speed that corresponds to the coordinate. */
				speedName = AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[i].name);
				addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
			}
		}

		/* Now process the rotation. If the joint type was determined
		 * correctly, exactly one of them should be a function or non-zero constant.
		 */
		int rdIndex;
		AbstractDof* rotDof = findNthFunctionRotation(_joints[aJointIndex], 0, rdIndex);
		const AbstractCoordinate* coord = rotDof->getCoordinate();
		rotDof->getAxis(axis);

		if (_joints[aJointIndex].direction == SimmStep::inverse)
			sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", -axis[0], -axis[1], -axis[2]);
		else
			sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", axis[0], axis[1], axis[2]);
		_joints[aJointIndex].pinString.append(buffer);

		if (coord == NULL)
		{
			_joints[aJointIndex].prescribedString.append(" 1");
			char numString[10];
			sprintf(numString, "%d", rDofCount);
			_joints[aJointIndex].dofs[rdIndex].name = fixedString;
			_joints[aJointIndex].dofs[rdIndex].name.append(numString);
			_joints[aJointIndex].dofs[rdIndex].constrained = false;
			_joints[aJointIndex].dofs[rdIndex].fixed = true;
			_joints[aJointIndex].dofs[rdIndex].initialValue = rotDof->getValue();
		}
		else
		{
			_joints[aJointIndex].dofs[rdIndex].fixed = false;
			if (_joints[aJointIndex].dofs[rdIndex].constrained)
			{
				_joints[aJointIndex].dofs[rdIndex].errorNumber = rConstrainedCount++;
				_joints[aJointIndex].prescribedString.append(" 0");
				_joints[aJointIndex].dofs[rdIndex].initialValue = rotDof->getValue();
			}
			else
			{
				_joints[aJointIndex].prescribedString.append(" 0?");
				_joints[aJointIndex].dofs[rdIndex].initialValue = coord->getValue();
			}
		}
		_joints[aJointIndex].dofs[rdIndex].stateNumber = rDofCount++;
		_joints[aJointIndex].dofs[rdIndex].joint = _joints[aJointIndex].index;
		_joints[aJointIndex].dofs[rdIndex].axis = 2;
		newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[rdIndex]);

		/* Now create a speed that corresponds to the coordinate. */
		speedName = AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[rdIndex].name);
		addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
	}
	else // (dpReversePlanar && forward) || (dpPlanar && inverse)
	{
		/* First process the rotation. If the joint type was determined
		 * correctly, exactly one of them should be a function or non-zero constant.
		 */
		int rdIndex;
		AbstractDof* rotDof = findNthFunctionRotation(_joints[aJointIndex], 0, rdIndex);
		const AbstractCoordinate* coord = rotDof->getCoordinate();
		rotDof->getAxis(axis);

		if (_joints[aJointIndex].direction == SimmStep::inverse)
			sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", -axis[0], -axis[1], -axis[2]);
		else
			sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", axis[0], axis[1], axis[2]);
		_joints[aJointIndex].pinString.append(buffer);

		if (coord == NULL)
		{
			_joints[aJointIndex].prescribedString.append(" 1");
			char numString[10];
			sprintf(numString, "%d", rDofCount);
			_joints[aJointIndex].dofs[rdIndex].name = fixedString;
			_joints[aJointIndex].dofs[rdIndex].name.append(numString);
			_joints[aJointIndex].dofs[rdIndex].constrained = false;
			_joints[aJointIndex].dofs[rdIndex].fixed = true;
			_joints[aJointIndex].dofs[rdIndex].initialValue = rotDof->getValue();
		}
		else
		{
			_joints[aJointIndex].dofs[rdIndex].fixed = false;
			if (_joints[aJointIndex].dofs[rdIndex].constrained)
			{
				_joints[aJointIndex].dofs[rdIndex].errorNumber = rConstrainedCount++;
				_joints[aJointIndex].prescribedString.append(" 0");
				_joints[aJointIndex].dofs[rdIndex].initialValue = rotDof->getValue();
			}
			else
			{
				_joints[aJointIndex].prescribedString.append(" 0?");
				_joints[aJointIndex].dofs[rdIndex].initialValue = coord->getValue();
			}
		}
		_joints[aJointIndex].dofs[rdIndex].stateNumber = rDofCount++;
		_joints[aJointIndex].dofs[rdIndex].joint = _joints[aJointIndex].index;
		_joints[aJointIndex].dofs[rdIndex].axis = axisNum++;
		newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[rdIndex]);

		/* Now create a speed that corresponds to the coordinate. */
		speedName = AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[rdIndex].name);
		addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());

		/* Now process the translations, starting from the end of the
		 * dof array. If the joint type was determined correctly, exactly
		 * two of them should be functions.
		 */
		for (i = _joints[aJointIndex].numDofs - 1; i >= 0; i--)
		{
			if (_joints[aJointIndex].dofs[i].modelDof->getMotionType() == AbstractDof::Translational &&
				 _joints[aJointIndex].dofs[i].modelDof->getCoordinate() != NULL)
			{
				_joints[aJointIndex].dofs[i].modelDof->getAxis(axis);
				if (_joints[aJointIndex].direction == SimmStep::inverse)
					sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", -axis[0], -axis[1], -axis[2]);
				else
					sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", axis[0], axis[1], axis[2]);
				_joints[aJointIndex].pinString.append(buffer);

				_joints[aJointIndex].dofs[i].stateNumber = rDofCount++;
				_joints[aJointIndex].dofs[i].fixed = false;
				if (_joints[aJointIndex].dofs[i].constrained)
				{
					_joints[aJointIndex].dofs[i].errorNumber = rConstrainedCount++;
					_joints[aJointIndex].prescribedString.append(" 0");
				}
				else
				{
					_joints[aJointIndex].prescribedString.append(" 0?");
				}
				_joints[aJointIndex].dofs[i].initialValue = _joints[aJointIndex].dofs[i].modelDof->getValue();
				_joints[aJointIndex].dofs[i].joint = _joints[aJointIndex].index;
				_joints[aJointIndex].dofs[i].axis = axisNum++;
				newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[i]);

				/* Now create a speed that corresponds to the coordinate. */
				speedName = AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[i].name);
				addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
			}
		}
	}
}

void SdfastFileWriter::makeSdfastUniversal(int aJointIndex, int& rDofCount, int& rConstrainedCount)
{
	int axisCount = 0;
	double axis[3];
	AbstractDof* dof;
	DofInfo* dofInfo;
	char buffer[200];
	AbstractCoordinate *newCoord;
	string speedName;

	/* If the joint type has been determined properly, there should be
	 * exactly two rotationDofs that are functions or non-zero constants.
	 */
	for (int i = 0; i < _joints[aJointIndex].numDofs; i++)
	{
		if (_joints[aJointIndex].direction == SimmStep::forward)
			dofInfo = &_joints[aJointIndex].dofs[i];
		else
			dofInfo = &_joints[aJointIndex].dofs[_joints[aJointIndex].numDofs - 1 - i];
		dof = dofInfo->modelDof;

		const AbstractCoordinate* coord = dof->getCoordinate();

		if (dof->getMotionType() == AbstractDof::Rotational &&
			(coord != NULL || NOT_EQUAL_WITHIN_ERROR(dof->getValue(), 0.0)))
		{
			dof->getAxis(axis);
			if (_joints[aJointIndex].direction == SimmStep::inverse)
				sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", -axis[0], -axis[1], -axis[2]);
			else
				sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", axis[0], axis[1], axis[2]);
			_joints[aJointIndex].pinString.append(buffer);

			if (coord == NULL)
			{
				_joints[aJointIndex].prescribedString.append(" 1");
				char numString[10];
				sprintf(numString, "%d", rDofCount);
				dofInfo->name = fixedString;
				dofInfo->name.append(numString);
				dofInfo->constrained = false;
				dofInfo->fixed = true;
				dofInfo->initialValue = dof->getValue();
			}
			else
			{
				dofInfo->fixed = false;
				if (dofInfo->constrained)
				{
					dofInfo->errorNumber = rConstrainedCount++;
					_joints[aJointIndex].prescribedString.append(" 0");
					dofInfo->initialValue = dof->getValue();
				}
				else
				{
					_joints[aJointIndex].prescribedString.append(" 0?");
					dofInfo->initialValue = coord->getValue();
				}
			}

			dofInfo->stateNumber = rDofCount++;
			dofInfo->joint = _joints[aJointIndex].index;
			dofInfo->axis = axisCount++;
			newCoord = addCoordinateToSimulationModel(*dofInfo);

			/* Now create a speed that corresponds to the coordinate. */
			speedName = AbstractSpeed::getSpeedName(dofInfo->name);
			addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
		}
	}
}

void SdfastFileWriter::makeSdfastCylindrical(int aJointIndex, int& rDofCount, int& rConstrainedCount)
{
	double axis[3];
	char buffer[200];
	AbstractCoordinate *newCoord;
	string speedName;

	/* Do the translation first. If the joint type has been determined
	 * properly, there should be exactly one translationDof that is a
	 * function.
	 */
	int tdIndex = -1;
	AbstractDof* transDof = findNthFunctionTranslation(_joints[aJointIndex], 0, tdIndex);

	_joints[aJointIndex].dofs[tdIndex].stateNumber = rDofCount++;
	_joints[aJointIndex].dofs[tdIndex].fixed = false;
	if (_joints[aJointIndex].dofs[tdIndex].constrained)
	{
		_joints[aJointIndex].dofs[tdIndex].errorNumber = rConstrainedCount++;
		_joints[aJointIndex].prescribedString.append(" 0");
		_joints[aJointIndex].dofs[tdIndex].initialValue = transDof->getValue();
	}
	else
	{
		_joints[aJointIndex].prescribedString.append(" 0?");
		_joints[aJointIndex].dofs[tdIndex].initialValue = transDof->getCoordinate()->getValue();
	}
	_joints[aJointIndex].dofs[tdIndex].joint = _joints[aJointIndex].index;
	_joints[aJointIndex].dofs[tdIndex].axis = 0;
	newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[tdIndex]);

	/* Now create a speed that corresponds to the coordinate. */
	speedName = AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[tdIndex].name);
	addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());

	/* Now do the rotation. If the joint type has been determined properly,
	 * there should be exactly one rotationDof that is a function.
	 */
	int rdIndex;
	AbstractDof* rotDof = findNthFunctionRotation(_joints[aJointIndex], 0, rdIndex);
	const AbstractCoordinate* coord = rotDof->getCoordinate();

	rotDof->getAxis(axis);
	if (_joints[aJointIndex].direction == SimmStep::inverse)
		sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", -axis[0], -axis[1], -axis[2]);
	else
		sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", axis[0], axis[1], axis[2]);
	_joints[aJointIndex].pinString = buffer;

	if (coord == NULL)
	{
		_joints[aJointIndex].prescribedString.append(" 1");
		char numString[10];
		sprintf(numString, "%d", rDofCount);
		_joints[aJointIndex].dofs[rdIndex].name = fixedString;
		_joints[aJointIndex].dofs[rdIndex].name.append(numString);
		_joints[aJointIndex].dofs[rdIndex].constrained = false;
		_joints[aJointIndex].dofs[rdIndex].fixed = true;
		_joints[aJointIndex].dofs[rdIndex].initialValue = rotDof->getValue();
	}
	else
	{
		_joints[aJointIndex].dofs[rdIndex].fixed = false;
		if (_joints[aJointIndex].dofs[rdIndex].constrained)
		{
			_joints[aJointIndex].dofs[rdIndex].errorNumber = rConstrainedCount++;
			_joints[aJointIndex].prescribedString.append(" 0");
			_joints[aJointIndex].dofs[rdIndex].initialValue = rotDof->getValue();
		}
		else
		{
			_joints[aJointIndex].prescribedString.append(" 0?");
			_joints[aJointIndex].dofs[rdIndex].initialValue = coord->getValue();
		}
	}

	_joints[aJointIndex].dofs[rdIndex].stateNumber = rDofCount++;
	_joints[aJointIndex].dofs[rdIndex].joint = _joints[aJointIndex].index;
	_joints[aJointIndex].dofs[rdIndex].axis = 1;
	newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[rdIndex]);

	/* Now create a speed that corresponds to the coordinate. */
	speedName = AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[rdIndex].name);
	addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
}

void SdfastFileWriter::makeSdfastGimbal(int aJointIndex, int& rDofCount, int& rConstrainedCount)
{
	double axis[3];
	char buffer[200];
	int axisCount = 0;
	AbstractDof* dof;
	DofInfo* dofInfo;
	AbstractCoordinate *newCoord;
	string speedName;

	/* If the joint type has been determined properly, there should be
	 * exactly three rotationDofs that are functions or non-zero constants.
	 */
	for (int i = 0; i < _joints[aJointIndex].numDofs; i++)
	{
		if (_joints[aJointIndex].direction == SimmStep::forward)
			dofInfo = &_joints[aJointIndex].dofs[i];
		else
			dofInfo = &_joints[aJointIndex].dofs[_joints[aJointIndex].numDofs - 1 - i];
		dof = dofInfo->modelDof;

		const AbstractCoordinate* coord = dof->getCoordinate();

		if (dof->getMotionType() == AbstractDof::Rotational &&
			(coord != NULL || NOT_EQUAL_WITHIN_ERROR(dof->getValue(), 0.0)))
		{
			dof->getAxis(axis);
			if (_joints[aJointIndex].direction == SimmStep::inverse)
				sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", -axis[0], -axis[1], -axis[2]);
			else
				sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", axis[0], axis[1], axis[2]);
			_joints[aJointIndex].pinString.append(buffer);

			if (coord == NULL)
			{
				_joints[aJointIndex].prescribedString.append(" 1");
				char numString[10];
				sprintf(numString, "%d", rDofCount);
				dofInfo->name = fixedString;
				dofInfo->name.append(numString);
				dofInfo->constrained = false;
				dofInfo->fixed = true;
				dofInfo->initialValue = dof->getValue();
			}
			else
			{
				dofInfo->fixed = false;
				if (dofInfo->constrained)
				{
					dofInfo->errorNumber = rConstrainedCount++;
					_joints[aJointIndex].prescribedString.append(" 0");
					dofInfo->initialValue = dof->getValue();
				}
				else
				{
					_joints[aJointIndex].prescribedString.append(" 0?");
					dofInfo->initialValue = coord->getValue();
				}
			}

			dofInfo->stateNumber = rDofCount++;
			dofInfo->joint = _joints[aJointIndex].index;
			dofInfo->axis = axisCount++;
			newCoord = addCoordinateToSimulationModel(*dofInfo);

			/* Now create a speed that corresponds to the coordinate. */
			speedName = AbstractSpeed::getSpeedName(dofInfo->name);
			addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
		}
	}
}

void SdfastFileWriter::makeSdfastBushing(int aJointIndex, int& rDofCount, int& rConstrainedCount)
{
	int i, axisNum = 0;
	double axis[3];
	char buffer[200];
	AbstractDof* rotDofs[3];
	int rdIndices[3];
	AbstractDof* transDofs[3];
	int tdIndices[3];
	AbstractCoordinate *newCoord;
	string speedName;

	if ((_joints[aJointIndex].type == dpBushing && _joints[aJointIndex].direction == SimmStep::forward) ||
		(_joints[aJointIndex].type == dpReverseBushing && _joints[aJointIndex].direction == SimmStep::inverse))
	{
		/* The translations must be processed in the same order as the rotations
		 * (since they share axes), so first make a list of the translations
		 * that is ordered the same way as the rotations.
		 */
		for (i = 0; i < 3; i++)
		{
			if (_joints[aJointIndex].direction == SimmStep::forward)
				rotDofs[i] = findNthFunctionRotation(_joints[aJointIndex], i, rdIndices[i]);
			else
				rotDofs[i] = findNthFunctionRotation(_joints[aJointIndex], 2-i, rdIndices[i]);

			transDofs[i] = findMatchingTranslationDof(_joints[aJointIndex], rotDofs[i], tdIndices[i]);
		}

		/* Now process the translations in that order. */
		for (i = 0; i < 3; i++)
		{
			_joints[aJointIndex].dofs[tdIndices[i]].stateNumber = rDofCount++;
			_joints[aJointIndex].dofs[tdIndices[i]].fixed = false;
			if (_joints[aJointIndex].dofs[tdIndices[i]].constrained)
			{
				_joints[aJointIndex].dofs[tdIndices[i]].errorNumber = rConstrainedCount++;
				_joints[aJointIndex].prescribedString.append(" 0");
				_joints[aJointIndex].dofs[tdIndices[i]].initialValue = transDofs[i]->getValue();
			}
			else
			{
				_joints[aJointIndex].prescribedString.append(" 0?");
				_joints[aJointIndex].dofs[tdIndices[i]].initialValue = transDofs[i]->getValue();
			}
			_joints[aJointIndex].dofs[tdIndices[i]].joint = _joints[aJointIndex].index;
			_joints[aJointIndex].dofs[tdIndices[i]].axis = axisNum++;
			newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[tdIndices[i]]);

			/* Now create a speed that corresponds to the coordinate. */
			speedName = AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[tdIndices[i]].name);
			addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
		}

		/* Now process the rotations in that same order. */
		for (i = 0; i < 3; i++)
		{
			rotDofs[i]->getAxis(axis);
			if (_joints[aJointIndex].direction == SimmStep::inverse)
				sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", -axis[0], -axis[1], -axis[2]);
			else
				sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", axis[0], axis[1], axis[2]);
			_joints[aJointIndex].pinString.append(buffer);

			const AbstractCoordinate* coord = rotDofs[i]->getCoordinate();

			if (coord == NULL)
			{
				_joints[aJointIndex].prescribedString.append(" 1");
				char numString[10];
				sprintf(numString, "%d", rDofCount);
				_joints[aJointIndex].dofs[rdIndices[i]].name = fixedString;
				_joints[aJointIndex].dofs[rdIndices[i]].name.append(numString);
				_joints[aJointIndex].dofs[rdIndices[i]].constrained = false;
				_joints[aJointIndex].dofs[rdIndices[i]].fixed = true;
				_joints[aJointIndex].dofs[rdIndices[i]].initialValue = rotDofs[i]->getValue();
			}
			else
			{
				_joints[aJointIndex].dofs[rdIndices[i]].fixed = false;
				if (_joints[aJointIndex].dofs[rdIndices[i]].constrained)
				{
					_joints[aJointIndex].dofs[rdIndices[i]].errorNumber = rConstrainedCount++;
					_joints[aJointIndex].prescribedString.append(" 0");
					_joints[aJointIndex].dofs[rdIndices[i]].initialValue = rotDofs[i]->getValue();
				}
				else
				{
					_joints[aJointIndex].prescribedString.append(" 0?");
					_joints[aJointIndex].dofs[rdIndices[i]].initialValue = coord->getValue();
				}
			}

			_joints[aJointIndex].dofs[rdIndices[i]].stateNumber = rDofCount++;
			_joints[aJointIndex].dofs[rdIndices[i]].joint = _joints[aJointIndex].index;
			_joints[aJointIndex].dofs[rdIndices[i]].axis = axisNum++;
			newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[rdIndices[i]]);

			/* Now create a speed that corresponds to the coordinate. */
			speedName = AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[rdIndices[i]].name);
			addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
		}
	}
	else
	{
		/* The translations must be processed in the same order as the rotations
		 * (since they share axes), so first make a list of the translations
		 * that is ordered the same way as the rotations.
		 */
		for (i = 0; i < 3; i++)
		{
			if (_joints[aJointIndex].direction == SimmStep::forward)
				rotDofs[i] = findNthFunctionRotation(_joints[aJointIndex], i, rdIndices[i]);
			else
				rotDofs[i] = findNthFunctionRotation(_joints[aJointIndex], 2-i, rdIndices[i]);

			transDofs[i] = findMatchingTranslationDof(_joints[aJointIndex], rotDofs[i], tdIndices[i]);
		}

		/* Now process the rotations in that order. */
		for (i = 0; i < 3; i++)
		{
			rotDofs[i]->getAxis(axis);
			if (_joints[aJointIndex].direction == SimmStep::inverse)
				sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", -axis[0], -axis[1], -axis[2]);
			else
				sprintf(buffer, "pin = %.10lf? %.10lf? %.10lf?\n", axis[0], axis[1], axis[2]);
			_joints[aJointIndex].pinString.append(buffer);

			const AbstractCoordinate* coord = rotDofs[i]->getCoordinate();

			if (coord == NULL)
			{
				_joints[aJointIndex].prescribedString.append(" 1");
				char numString[10];
				sprintf(numString, "%d", rDofCount);
				_joints[aJointIndex].dofs[rdIndices[i]].name = fixedString;
				_joints[aJointIndex].dofs[rdIndices[i]].name.append(numString);
				_joints[aJointIndex].dofs[rdIndices[i]].constrained = false;
				_joints[aJointIndex].dofs[rdIndices[i]].fixed = true;
				_joints[aJointIndex].dofs[rdIndices[i]].initialValue = rotDofs[i]->getValue();
			}
			else
			{
				_joints[aJointIndex].dofs[rdIndices[i]].fixed = false;
				if (_joints[aJointIndex].dofs[rdIndices[i]].constrained)
				{
					_joints[aJointIndex].dofs[rdIndices[i]].errorNumber = rConstrainedCount++;
					_joints[aJointIndex].prescribedString.append(" 0");
					_joints[aJointIndex].dofs[rdIndices[i]].initialValue = rotDofs[i]->getValue();
				}
				else
				{
					_joints[aJointIndex].prescribedString.append(" 0?");
					_joints[aJointIndex].dofs[rdIndices[i]].initialValue = coord->getValue();
				}
			}

			_joints[aJointIndex].dofs[rdIndices[i]].stateNumber = rDofCount++;
			_joints[aJointIndex].dofs[rdIndices[i]].joint = _joints[aJointIndex].index;
			_joints[aJointIndex].dofs[rdIndices[i]].axis = axisNum++;
			newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[rdIndices[i]]);

			/* Now create a speed that corresponds to the coordinate. */
			speedName = AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[rdIndices[i]].name);
			addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
		}

		/* Now process the translations in that same order. */
		for (i = 0; i < 3; i++)
		{
			_joints[aJointIndex].dofs[tdIndices[i]].stateNumber = rDofCount++;
			_joints[aJointIndex].dofs[tdIndices[i]].fixed = false;
			if (_joints[aJointIndex].dofs[tdIndices[i]].constrained)
			{
				_joints[aJointIndex].dofs[tdIndices[i]].errorNumber = rConstrainedCount++;
				_joints[aJointIndex].prescribedString.append(" 0");
				_joints[aJointIndex].dofs[tdIndices[i]].initialValue = transDofs[i]->getValue();
			}
			else
			{
				_joints[aJointIndex].prescribedString.append(" 0?");
				_joints[aJointIndex].dofs[tdIndices[i]].initialValue = transDofs[i]->getValue();
			}
			_joints[aJointIndex].dofs[tdIndices[i]].joint = _joints[aJointIndex].index;
			_joints[aJointIndex].dofs[tdIndices[i]].axis = axisNum++;
			newCoord = addCoordinateToSimulationModel(_joints[aJointIndex].dofs[tdIndices[i]]);

			/* Now create a speed that corresponds to the coordinate. */
			speedName = AbstractSpeed::getSpeedName(_joints[aJointIndex].dofs[tdIndices[i]].name);
			addSpeedToSimulationModel(speedName, 0.0, newCoord->getName());
		}
	}
}

/* This function finds the Nth rotation dof in the joint which
 * is either a function of a coordinate, or is a non-zero constant.
 * Non-zero constants are counted as functions for the purposes
 * of mapping the joint to an SD/FAST joint. N is zero-based.
 */
AbstractDof* SdfastFileWriter::findNthFunctionRotation(JointInfo& aJointInfo, int aN, int& rIndex) const
{
	int i, count = 0;
	for (i = 0; i < aJointInfo.numDofs; i++)
	{
		if (aJointInfo.dofs[i].modelDof->getMotionType() == AbstractDof::Rotational &&
			(aJointInfo.dofs[i].modelDof->getCoordinate() != NULL ||
			 NOT_EQUAL_WITHIN_ERROR(aJointInfo.dofs[i].modelDof->getValue(), 0.0)))
		{
			if (count++ == aN)
				break;
		}
	}

	if (i == aJointInfo.numDofs)
	{
		rIndex = -1;
		return NULL;
	}
	else
	{
		rIndex = i;
		return aJointInfo.dofs[i].modelDof;
	}
}

/* This function finds the Nth translation dof in the joint which
 * is a function of a coordinate. N is zero-based.
 */
AbstractDof* SdfastFileWriter::findNthFunctionTranslation(JointInfo& aJointInfo, int aN, int& rIndex) const
{
	int i, count = 0;
	for (i = 0; i < aJointInfo.numDofs; i++)
	{
		if (aJointInfo.dofs[i].modelDof->getMotionType() == AbstractDof::Translational &&
			 aJointInfo.dofs[i].modelDof->getCoordinate() != NULL)
		{
			if (count++ == aN)
				break;
		}
	}

	if (i == aJointInfo.numDofs)
	{
		rIndex = -1;
      return NULL;
	}
	else
	{
		rIndex = i;
		return aJointInfo.dofs[i].modelDof;
	}
}

/* Given a rotationDof, this functions finds the translationDof which uses
 * the same axis.
 */
AbstractDof* SdfastFileWriter::findMatchingTranslationDof(JointInfo& aJointInfo, AbstractDof* aRotDof, int& rIndex) const
{
	double rotAxis[3], transAxis[3];

	aRotDof->getAxis(rotAxis);

	for (int i = 0; i < aJointInfo.numDofs; i++)
	{
		if (aJointInfo.dofs[i].modelDof->getMotionType() == AbstractDof::Translational)
		{
			aJointInfo.dofs[i].modelDof->getAxis(transAxis);
			if (axesAreParallel(rotAxis, transAxis, false))
			{
				rIndex = i;
				return aJointInfo.dofs[i].modelDof;
			}
		}
	}

	rIndex = -1;
	return NULL;
}

/* An SD/FAST-compatible joint is one with:
 *  1. no more than 3 translation DOFs
 *  2. no 2 translation DOFs along the same axis (X, Y, Z)
 *  3. no more than 3 rotation DOFs
 *  4. no rotations between the translation DOFs
 */
bool SdfastFileWriter::isJointSdfastCompatible(const AbstractJoint* aJoint) const
{
	int numRotations = 0, numChanges = 0;
	int numTx = 0, numTy = 0, numTz = 0;
	int lastDof = -1;

	DofSet* dofs = aJoint->getDofSet();

	for (int i = 0; i < dofs->getSize(); i++)
	{
		if (dofs->get(i)->getMotionType() == AbstractDof::Translational)
		{
			if (dofs->get(i)->getName() == "tx") //TODO: these names are defined in SimmTranslationDof
				numTx++;
			else if (dofs->get(i)->getName() == "ty")
				numTy++;
			else //if (dofs->get(i)->getName() == "tz")
				numTz++;
			if (lastDof == AbstractDof::Rotational)
				numChanges++;
		}
		else
		{
			numRotations++;
			if (lastDof == AbstractDof::Translational)
				numChanges++;
		}
	}

	if (numRotations <= 3 && numChanges <= 1 &&
		 numTx <= 1 && numTy <= 1 && numTz <= 1)
		return true;

	return false;
}

//_____________________________________________________________________________
/**
 * Checks if two axes are parallel.
 *
 * @return Reference to this object.
 */
bool SdfastFileWriter::axesAreParallel(const double* aAxis1, const double* aAxis2, bool aOppositeDirAllowed) const
{
   if (aOppositeDirAllowed)
   {
      if (EQUAL_WITHIN_ERROR(DABS(aAxis1[0]), DABS(aAxis2[0])) &&
          EQUAL_WITHIN_ERROR(DABS(aAxis1[1]), DABS(aAxis2[1])) &&
          EQUAL_WITHIN_ERROR(DABS(aAxis1[2]), DABS(aAxis2[2])))
         return true;
      else
         return false;
   }
   else
   {
      if (EQUAL_WITHIN_ERROR(aAxis1[0], aAxis2[0]) &&
          EQUAL_WITHIN_ERROR(aAxis1[1], aAxis2[1]) &&
          EQUAL_WITHIN_ERROR(aAxis1[2], aAxis2[2]))
         return true;
      else
         return false;
   }
}

char* SdfastFileWriter::getDpJointName(dpJointType type, SimmStep::Direction direction)
{
   if (type == dpPin || type == dpReversePin)
      return ("pin");
   else if (type == dpCylindrical || type == dpReverseCylindrical)
      return ("cylinder");
   else if (type == dpPlanar)
   {
		if (direction == SimmStep::forward)
         return ("planar");
      else
         return ("rplanar");
   }
   else if (type == dpReversePlanar)
   {
      if (direction == SimmStep::forward)
         return ("rplanar");
      else
         return ("planar");
   }
   else if (type == dpSlider)
      return ("slider");
   else if (type == dpUniversal || type == dpReverseUniversal)
      return ("ujoint");
   else if (type == dpGimbal || type == dpReverseGimbal)
      return ("gimbal");
   else if (type == dpWeld)
      return ("pin");                  /* welds are prescribed pins */
   else if (type == dpLoop)
      return ("weld");                 /* loop joints are always welds */
   else if (type == dpBushing)
   {
      if (direction == SimmStep::forward)
         return ("bushing");
      else
         return ("rbushing");
   }
   else if (type == dpReverseBushing)
   {
      if (direction == SimmStep::forward)
         return ("rbushing");
      else
         return ("bushing");
   }

   return ("unknown");
}

bool SdfastFileWriter::checkDynamicParameters() const
{
	bool dynamicsReady = true;

	const BodySet* bodySet = _model->getDynamicsEngine().getBodySet();
	const AbstractBody* groundBody = &_model->getDynamicsEngine().getGroundBody();
	for(int index=0; index<bodySet->getSize(); index++) 
	{
		const AbstractBody* body = bodySet->get(index);
		if (body != groundBody && !_modelBodies[index].skippable)
		{
			// Check mass of body
			double mass = body->getMass();
			if (EQUAL_WITHIN_ERROR(mass, 0.0))
			{
				//cerr << "Mass of body " << body->getName() << " is zero." << endl;
				dynamicsReady = false;
			}

			// Check inertia matrix of body
			double inertia[3][3];
			body->getInertia(inertia);
			double inertiaSum = 0.0;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					inertiaSum += inertia[i][j];
			if (EQUAL_WITHIN_ERROR(inertiaSum, 0.0))
			{
				//cerr << "Inertia matrix of body " << body->getName() << " is zero." << endl;
				dynamicsReady = false;
			}
		}
	}

	return dynamicsReady;
}

void SdfastFileWriter::initialize()
{
	initInfoStructs();

	JointSet* jointSet = _model->getDynamicsEngine().getJointSet();
	for(int index=0; index<jointSet->getSize(); index++)
	{
		identifySdfastType(*jointSet->get(index), _joints[index]);
	}

	if (!isValidSdfastModel())
		goto error;

	if (!checkDynamicParameters())
		goto error;

	/* Copy the input model, and replace its dynamics engine
	 * with a new SdfastEngine. The components of this engine
	 * will get filled in by makeSdfastModel().
	 */
	_simulationModel = (AbstractModel*)_model->copy();
	_simulationEngine = new SdfastEngine();

	// Note that _simulationModel will take over ownership of _simulationEngine.
	_simulationModel->setDynamicsEngine(*_simulationEngine);

	makeSdfastModel();

	_initialized = true;

	return;

error:
	cerr << "Cannot save dynamics." << endl;

	return;
}

