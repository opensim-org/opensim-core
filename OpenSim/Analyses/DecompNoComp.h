#ifndef _DecompNoComp_h_
#define _DecompNoComp_h_
// DecompNoComp.h
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
 * An abstract base class for supporting the decomposition of contact
 * forces.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class Model;

class OSIMANALYSES_API DecompNoComp : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
public:
	static const char *ADDON_COMPONENT_NAMES[];

protected:
	/** Number of action components. */
	int _nc;
	/** Number of independent action components. */
	int _nic;
	/** Number of contact points. */
	int _np;
	/** Contact threashold for when to perform a decomposition. */
	double _contactThreshold;
	/** Flag to indicate whether or not to use preset contact established
	settings.  If this flag is set to true, computation of contact forces
	is not performed in order to determine if contact has been established.
	Rather, it is assumed that the caller has preset the desired
	contact-established settings. */
	bool _usePresetContactEstablishedSettings;
	/** Array of flags to indicate whether or not contact has been established
	at each of the contact pooints. */
	bool *_contactEstablished;
	/** Contiguous memory allocation for the force decomposition results. */
	double *_fContig;
	/** Arrays for storing the current force decomposition. */
	double ***_f;
	/** Storage objects for storing the time history of the decomposition. */
	Storage **_fStore;
	/** Storage objects for storing the contact point accelerations */
	Storage **_cpaStore;
	/** Flag which indicates whether or not the contact point accelerations
	are being recorded */
	bool _recordContactPointAccelerations;

private:
	/** Flag which indicates whether or not the decomposition is NULL.  A
	NULL decomposition means no decompostion- all induced contact forces
	zero. */
	bool _useNullDecomposition;

//=============================================================================
// METHODS
//=============================================================================
public:
	DecompNoComp(Model *aModel);
	DecompNoComp(Model *aModel,char *aBaseName,char *aDir=NULL,
		char *aExtension=NULL);
	virtual ~DecompNoComp();
private:
	void setNull();
	void initializeNumbers();
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
	// CONTACT THREASHOLD
	void setContactThreshold(double aThreshold);
	double getContactThreshold();
	// CONTACT ESTABLISHED
	void setUsePresetContactEstablishedSettings(bool aTrueFalse);
	bool getUsePresetContactEstablishedSettings() const;
	void setContactEstablished(int aIndex,bool aTrueFalse);
	bool getContactEstablished(int aIndex) const;
	// RECORD CONTACT POINT ACCELERATIONS
	void setRecordContactPointAccelerations(bool aTrueFalse);
	bool getRecordContactPointAccelerations();
	// STORAGE
	virtual void setStorageCapacityIncrements(int aIncrement);
	const Storage* getDecomposition(int aC) const;
	// NULL DECOMPOSITION
	bool getUseNullDecomposition();

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	virtual void compute(double *aXPrev,double *aYPrev,
		int aStep,double aDT,double aT,double *aX,double *aY) = 0;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------
	virtual int
		begin(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		end(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");
private:
	virtual int
		readDecomposition(char *aBaseName,char *aDir=NULL,
		char *aExtension=NULL);


//=============================================================================
};	// END of class DecompNoComp

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __DecompNoComp_h__
