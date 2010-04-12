// JointSet.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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

#include "JointSet.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Common/ScaleSet.h>

using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
JointSet::~JointSet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a JointSet.
 */
JointSet::JointSet()
{
	setNull();
}

JointSet::JointSet(Model& model) :
	ModelComponentSet<Joint>(model)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a JointSet.
 */
JointSet::JointSet(const JointSet& aJointSet):
	ModelComponentSet<Joint>(aJointSet)
{
	setNull();
	*this = aJointSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this JointSet to their null values.
 */
void JointSet::setNull()
{
	setType("JointSet");
}

/**
 * This is overridden to ensure that joints are processed from ground outward.
 */
void JointSet::createSystem(SimTK::MultibodySystem& system) const
{
    vector<bool> hasProcessed(getSize(), false);
    map<Body*, int> bodyMap;
    for (int i = 0; i < getSize(); i++)
    {
        const Joint& joint = static_cast<const Joint&>(get(i));
        bodyMap[&joint.getBody()] = i;
    }
    for (int i = 0; i < getSize(); i++)
        createSystemForOneJoint(system, i, bodyMap, hasProcessed);
}

void JointSet::createSystemForOneJoint(SimTK::MultibodySystem& system, int jointIndex, const map<Body*, int>& bodyMap, vector<bool>& hasProcessed) const
{
    if (hasProcessed[jointIndex])
        return;
    hasProcessed[jointIndex] = true;
    const Joint& joint = static_cast<const Joint&>(get(jointIndex));
    if (joint._parentBody != NULL)
    {
        // Make sure the parent joint is processed first.

        map<Body*, int>::const_iterator parent = bodyMap.find(&joint.getParentBody());
        if (parent != bodyMap.end())
        {
            int parentIndex = parent->second;
            if (!hasProcessed[parentIndex])
                createSystemForOneJoint(system, parentIndex, bodyMap, hasProcessed);
        }
    }
    static_cast<const Joint&>(get(jointIndex)).createSystem(system);
}

/**
 * Post construction initialization.
 */
void JointSet::setup(Model& aModel)
{
	// Base class
	ModelComponentSet::setup(aModel);

    setMemoryOwner(false);
    setSize(0);

    for(int i=0; i< aModel.getNumBodies(); i++){
        if (aModel.getBodySet().get(i).hasJoint()) { // Ground body doesn't have a jnt
            Joint& nextJoint = aModel.getBodySet().get(i).getJoint();
			nextJoint.setBody(aModel.getBodySet().get(i));
            append(&nextJoint);
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
#ifndef SWIG
JointSet& JointSet::operator=(const JointSet &aJointSet)
{
	Set<Joint>::operator=(aJointSet);
	return (*this);
}
#endif

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale joint set by a set of scale factors
 */
void JointSet::scale(const ScaleSet& aScaleSet)
{
	for(int i=0; i<getSize(); i++) get(i).scale(aScaleSet);
}
