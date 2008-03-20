#ifndef _Contact_h_
#define _Contact_h_
// Contact.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson and Saryn Goldberg
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
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


//=============================================================================
//=============================================================================
/**
 * A class for recording the contact forces (and torques) generated
 * during a simulation.  This class will record only the forces (and torques)
 * returned by the getContactForces() method of Model.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class OSIMANALYSES_API Contact : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:
	Array<std::string> _scalarLabels;
	Array<std::string> _resultantForcePointLabels;
	/** Array containing group assignemnts for each contact point in resultant
		 force point calc **/
	int *_resultantForcePointGroupAssignements;
	/** Number of resultant force point groups **/
	int _nResultantForcePointGroups;

protected:
	/** Array of contact points. */
	double *_points;
	/** Array of contact velocities. */
	double *_velocities;
	/** Array of contact forces. */
	double *_forces;
	/** Array of contact powers. */
	double *_powers;
	/** Array of resultant force point (x, y, and z direction for each RFP group). */
	double *_resultantForcePoints;
	/** Array of total contactforces (x, y and z direction for each RFP group). */
	double *_totContactForces;
	/** Array of total contactforces (x, y and z direction for each RFP group). */
	double *_totContactTorques;

	/** Storage for the contact points. */
	Storage *_pStore;
	/** Storage for the contact point velocities. */
	Storage *_vStore;
	/** Storage for the contact forces. */
	Storage *_fStore;
	/** Storage for contact powers. */
	Storage *_pwrStore;
	/** Storage for resultant force points. */
	Storage *_resultantForcePointsStore;
	/** Storage for total contact forces. */
	Storage *_totFStore;
	/** Storage for total torque. */
	Storage *_totTorqueStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	Contact(Model *aModel,int *aResultantForcePointGroups=NULL);
	virtual ~Contact();
private:
	void setNull();
	void constructDescription();
	void constructColumnLabels(int nResultantForcePointgroups);
	void allocateStorage();
	void deleteStorage();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	const Array<std::string> &getScalarColumnLabels();
	const Array<std::string> &getResultantForcePointColumnLabels();
	Storage* getPointsStorage() const;
	Storage* getVelocityStorage() const;
	Storage* getForceStorage() const;
	Storage* getPowerStorage() const;
	Storage* getResultantForcePointStorage() const;
	Storage* getTotForceStorage() const;
	Storage* getTotTorqueStorage() const;
	void setStorageCapacityIncrements(int aIncrement);


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
protected:
	virtual int
		record(double aT,double *aX,double *aY);

	//--------------------------------------------------------------------------
	// UTILITIES
	//--------------------------------------------------------------------------
	void resetStorage();

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class Contact

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __Contact_h__
