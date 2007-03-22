#ifndef _Contact_h_
#define _Contact_h_
// Contact.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson and Saryn Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


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
	std::string _scalarLabels;
	std::string _resultantForcePointLabels;
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
	std::string getScalarColumnLabels();
	std::string getResultantForcePointColumnLabels();
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
		begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData=NULL);
	virtual int
		step(double *aXPrev,double *aYPrev,
		int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData=NULL);
	virtual int
		end(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData=NULL);
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
