#ifndef _DecompInteg_h_
#define _DecompInteg_h_
// DecompInteg.h
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
#include <OpenSim/Simulation/Manager/Manager.h>
#include "osimAnalysesDLL.h"
#include "Contact.h"
#include "Actuation.h"
#include "Decomp.h"
#include "ActuatorPerturbation.h"


//=============================================================================
//=============================================================================
/**
 * A class for computing induced reaction forces using the
 * "perturbed integration" methodology.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class Model;

class OSIMANALYSES_API DecompInteg : public Decomp
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Nominal manager. */
	const Manager *_managerNom;
	/** Nominal contact analysis. */
	const Contact *_contactNom;
	/** Nominal actuation analysis. */
	const Actuation *_actuationNom;
	/** Size of the integration time window. */
	double _dt;
	/** Perturbed simulation manager. */
	Manager *_manager;
	/** Perturbed contact analysis. */
	Contact *_contact;
	/** Last index at which the perturbed integration was computed. */
	int _iLast;
	/** Local states array. */
	double *_y;
	/** Actuator forces. */
	double *_fAct;
	/** Local contact points array. */
	double *_pctx;
	/** Local contact forces array. */
	double *_fctx;
	/** Actuator perturbation callback instance. */
	ActuatorPerturbation *_perturbCallback;
	/** Flag to indicate whether to print detailed integration window
	information. */
	bool _printWindow;

//=============================================================================
// METHODS
//=============================================================================
public:
	DecompInteg(const Manager *aManager,const Contact *aContact,
		const Actuation *aActuation,Model *aModelTwin,
		double aDT=0.0,double aDF=0.0);
	virtual ~DecompInteg();
private:
	void setNull();
	void constructManager();
	void constructContactAnalysis();
	void constructDescription();
	void updateStorageDescriptions();
	void allocate();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setIntegrationWindow(double aDT);
	double getIntegrationWindow() const;
//	void setPerturbation(double aDF);
	void setPerturbation(ActuatorPerturbation::PertType aPerturbationType,double aDF);
	double getPerturbation() const;
	ActuatorPerturbation::PertType getPerturbationType() const;
//	void setPerturbationFactor(double aFactor);
//	double getPerturbationFactor() const;
//	void setUseAbsolutePerturbation(bool aTrueFalse);
//	bool getUseAbsolutePerturbation() const;
	void setPrintWindow(bool aTrueFalse);
	bool getPrintWindow() const;

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	virtual void compute(double *aXPrev,double *aYPrev,
		int aStep,double aDT,double aT,double *aX,double *aY);

//=============================================================================
};	// END of class DecompInteg

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __DecompInteg_h__
