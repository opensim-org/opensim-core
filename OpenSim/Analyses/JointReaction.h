#ifndef _JointReaction_h_
#define _JointReaction_h_
// JointReaction.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Matt DeMers, Ajay Seth
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyBoolArray.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


//=============================================================================
//=============================================================================
/**
 * An analysis for reporting the joint reaction loads from a model. For a given
 * joint, the reaction load is calculated as the forces and moments required to 
 * constrain the body motions to satisfy the joint as if the joint did not exist.
 * The reaction load acts at the joint center (mobilizer frame) of both the parent 
 * and child bodies and either force can be reported and expressed in the either
 * the child, parent or ground frames. The default behavior is the the force
 * on the child expressed in the ground frame.
 *
 * @author Matt DeMers, Ajay Seth, Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class Model;


class OSIMANALYSES_API JointReaction : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:

	/* Private struct containing reference indices used to set up the output
	 *  of each analysed joint.*/
	struct JointReactionKey
	{
		/* name of the joint*/
		std::string jointName;
		/* index coresponding to the location of the desrired reaction load 
		*  in the results of computeReactions()*/
		int reactionIndex;
		/* body set index of the parent or child body to which the joint
		*  reaction is applied*/
		int onBodyIndex;
		/* body set index of the parent, child, or ground body in which the 
		*  joint reaction is expressed*/
		int inFrameIndex;
	};

protected:
	//--------------------------------------------------------------------
	// Properties are the user-specified quantities that are read in from
	// file 
	//--------------------------------------------------------------------

	/** String Array containting the names of the joints to be analysed*/
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

//=============================================================================
// METHODS
//=============================================================================
public:
	//-------------------------------------------------------------------------
	// CONSTRUCTION
	//-------------------------------------------------------------------------
	virtual ~JointReaction();
	JointReaction(Model *aModel=0);
	JointReaction(const std::string &aFileName);
	JointReaction(const JointReaction &aObject);
	virtual Object* copy() const;
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
	virtual void setModel(Model *aModel);

	//-------------------------------------------------------------------------
	// INTEGRATION
	//-------------------------------------------------------------------------
	virtual int
		begin(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		end(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);

	//-------------------------------------------------------------------------
	// IO
	//-------------------------------------------------------------------------
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");


protected:
	//========================== Internal Methods =============================
	int record(double aT,double *aX,double *aY);
	void setupReactionList();
	void constructDescription();
	void constructColumnLabels();
	void setupStorage();

//=============================================================================
}; // END of class JointReaction
}; //namespace
//=============================================================================
//=============================================================================

#endif // #ifndef __JointReaction_h__
