#ifndef _IndAcc_h_
#define _IndAcc_h_
// IndAcc.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
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
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


//=============================================================================
//=============================================================================
/**
 * A class that performs a basic induced acceleration analysis.
 *
 * An induced acceleration analysis can be peformed in two ways using this
 * class:  1) during the course of a simulation and 2) after a simulation
 * has completed.  For the second way, the states recorded during
 * the simulation as well an appropriate contact force decomposition must be
 * used to construct the IndAcc instance.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class Model;

class OSIMANALYSES_API IndAcc : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	int _nc;
	int _nic;
	int _ne;
	int _cAct;
	int _cGrav;
	int _cVel;
	int _cIner;
	int _cAllAct;
	int _cAll;
	double _ti;
	double _tf;

	const char **_cNames;
	double _contactThreshold;
	bool *_contactEstablished;
	double *_feContig,***_fe;
	Storage *_yStore;
	Storage *_xStore;
	Storage **_feStore;
	Storage **_aeStore;
	Storage **_velStore;
	Storage **_posStore;
	Storage *_iPosStore;
	Storage *_iVelStore;
	char *_aeDescrip;
	char *_aeLabels;
	/** Flag that determines whether accelerations are normalized by force. */
	bool _computeNormalizedAccelerations;

private:
	/** Flag which indicates whether or not accelerations are being computed
	using a NULL decomposition. */
	bool _useNullDecomposition;

//=============================================================================
// METHODS
//=============================================================================
public:
	IndAcc(Model *aModel);
	IndAcc(Model *aModel,Storage *aStates,Storage *aControls,
		char *aBaseName,char *aDir=NULL,char *aExtension=NULL);
	virtual ~IndAcc();
private:
	void setNull();
	void initializeNumbers();
	void constructComponentNames();
	void constructDescription();
	void constructColumnLabels();
	void allocateElementVectors();
	void allocateStoragePointers();
	void allocateStorage();
	void deleteStorage();
	void createNullDecomposition();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// NUMBERS
	int getNumComponents();
	int getNumIndependentComponents();
	int getNumElements();
	// INDICES
	int getLastActuatorIndex();
	int getGravityIndex();
	int getVelocityIndex();
	int getInertialIndex();
	int getAllActuatorsIndex();
	int getAllIndex();
	// CONTACT TOLERANCE
	void setContactThreshold(double aThreshold);
	double getContactThreshold();
	// NAMES
	const char* getComponentName(int aC);
	// STORAGE
	virtual void setStorageCapacityIncrements(int aIncrement);
	Storage** getForceStorage();
	// NULL DECOMPOSITION
	bool getUseNullDecomposition();
	// NORMALIZED ACCELERATIONS
	void setComputeNormalizedAccelerations(bool aBool);
	bool getComputeNormalizedAccelerations();

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	int computeAccelerations();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void sumForceResults();
	void sumAccelerationResults();
	void sumDecomposition();

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	virtual void store();
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");
private:
	virtual int
		readDecomposition(char *aBaseName,char *aDir=NULL,
		char *aExtension=NULL);


//=============================================================================
};	// END of class IndAcc

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __IndAcc_h__
