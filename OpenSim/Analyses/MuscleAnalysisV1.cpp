// MuscleAnalysisV1.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Katherine Holzbaur, Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2006 Stanford University
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
#include <iostream>
#include <string>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Common/NaturalCubicSpline.h>
#include <OpenSim/Analyses/MuscleAnalysisV1.h>
#include <OpenSim/Actuators/Thelen2003MuscleV1.h>

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTANTS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
MuscleAnalysisV1::~MuscleAnalysisV1()
{
	delete _ncs_stdfal;
	delete _ncs_stdfv;
	delete _ncs_stdfpe;
	delete _ncs_stdfse;

}
//_____________________________________________________________________________
/**
 * Construct a MuscleAnalysisV1 object for recording the MuscleAnalysisV1 of
 * a model's generalized coodinates during a simulation.
 *
 * @param aModel Model for which the MuscleAnalysisV1 are to be recorded.
 */
MuscleAnalysisV1::MuscleAnalysisV1(Model *aModel) :
	Analysis(aModel)
{
	// NULL
	setNull();

	// CHECK MODEL
	if(_model==NULL) return;

	// STORAGE
	allocateStorageObjects();
	setStandardMuscleCurves(); //MM
}
//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
MuscleAnalysisV1::MuscleAnalysisV1(const std::string &aFileName):
Analysis(aFileName, false)
{
	setNull();
	updateFromXMLNode();
	allocateStorageObjects();
	setStandardMuscleCurves(); //MM
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
MuscleAnalysisV1::MuscleAnalysisV1(const MuscleAnalysisV1 &aMuscleAnalysisV1):
Analysis(aMuscleAnalysisV1)
{
	setNull();
	setStandardMuscleCurves();
	*this = aMuscleAnalysisV1;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* MuscleAnalysisV1::copy() const
{
	MuscleAnalysisV1 *object = new MuscleAnalysisV1(*this);
	return(object);

}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void MuscleAnalysisV1::
setNull()
{
	setType("MuscleAnalysisV1");
	setName("MuscleAnalysisV1");
	setupProperties();
	constructDescription();

	// STORAGE
	_fvStore = NULL; //(MM2011)

	_pennationAngleStore = NULL;
	_lengthStore = NULL;
	_fiberLengthStore = NULL;
	_normalizedFiberLengthStore = NULL;
	_tendonLengthStore = NULL;
	_forceStore = NULL;
	_fiberForceStore = NULL;
	_activeFiberForceStore = NULL;
	_passiveFiberForceStore = NULL;
	_activeFiberForceAlongTendonStore = NULL;
	_passiveFiberForceAlongTendonStore = NULL;

	// DEFAULT VALUES
	_muscleListProp.getValueStrArray().setSize(1);
	_muscleListProp.getValueStrArray().get(0) = "all";
	_coordinateListProp.getValueStrArray().setSize(1);
	_coordinateListProp.getValueStrArray().get(0) = "all";
	_computeMoments = true;

	//MM
	_dactStore = NULL; 
	_actStore = NULL; 
	_falStore = NULL; 
	_fseStore = NULL;
	_fpeStore = NULL; 
	_fvStore = NULL;
	_fvVmaxStore = NULL; 
	_tlStore = NULL;
	_lceStore = NULL;
	_dlceStore = NULL; 
	_uStore = NULL; 

	_ncs_stdfal = NULL;
	_ncs_stdfv  = NULL;
	_ncs_stdfpe = NULL;
	_ncs_stdfse = NULL;

	_fvErrStore = NULL;
	_falErrStore = NULL;
	_fpeErrStore = NULL;
	_fseErrStore = NULL;
	_mcltenErrStore = NULL;

	_fsePEStore = NULL;
	_fpePEStore = NULL;	
	_musclePWRStore = NULL; 
	_muscleFStore = NULL; 
	_muscleVStore = NULL; 
	_mclTdnKEPEWStore = NULL;
}
//_____________________________________________________________________________
/**
 * Set up the properties.
 */
void MuscleAnalysisV1::
setupProperties()
{
	_muscleListProp.setComment("List of muscles for which to perform the analysis."
		" Use 'all' to perform the analysis for all muscles.");
	_muscleListProp.setName("muscle_list");
	_propertySet.append( &_muscleListProp );

	_coordinateListProp.setComment("List of generalized coordinates for which to "
		"compute moment arms. Use 'all' to compute for all coordinates.");
	_coordinateListProp.setName("moment_arm_coordinate_list");
	_propertySet.append( &_coordinateListProp );

	_computeMomentsProp.setComment("Flag indicating whether moments should be computed.");
	_computeMomentsProp.setName("compute_moments");
	_propertySet.append( &_computeMomentsProp );

	//MM - Set up gold standard splines for dimensionless muscle curves

}

//_____________________________________________________________________________
/**
 *  MM 2011 10 18
 *
 * This function will populate the NaturalCubicSpline objects with curves that are
 * specified in the data files listed below. These curves are used as a point of 
 * comparision to evaluate whether the muscle model in question is operating in a
 * physiologically reasonable region or not.
 *
 */
void MuscleAnalysisV1::setStandardMuscleCurves(){
	string fileName_fal		= "delp1990_goldstandard_muscle_gsafl.sto";
	string fileName_fv		= "delp1990_goldstandard_muscle_gsfv.sto";
	string fileName_fpe		= "delp1990_goldstandard_muscle_gsfpe.sto";
	string fileName_fse		= "delp1990_goldstandard_tendon_gsfse.sto";

	_ncs_stdfal		= get1DSpline(fileName_fal);
	_ncs_stdfv		= get1DSpline(fileName_fv);
	_ncs_stdfpe		= get1DSpline(fileName_fpe);
	_ncs_stdfse		= get1DSpline(fileName_fse);	

	//Test that this actually worked with non-zero values for each curve
	double fal_val	= get1DSplineValue(_ncs_stdfal, 1.0);
	double fv_val	= get1DSplineValue(_ncs_stdfv, 1.0);
	double fpe_val	= get1DSplineValue(_ncs_stdfpe, 1.6);
	double fse_val	= get1DSplineValue(_ncs_stdfse, 1.033);

	printf("Non-zero values for fal(%f),fv(%f),fpe(%f),fse(%f) \n",fal_val,fv_val,fpe_val,fse_val);

}

//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the MuscleAnalysisV1 files.
 */
void MuscleAnalysisV1::
constructDescription()
{
	char descrip[1024];

	strcpy(descrip,"\nThis analysis gathers basic information about muscles ");
	strcat(descrip,"during a simulation (e.g., forces, tendon lenghts, moment arms, etc).");

	strcat(descrip,"\nUnits are S.I. units (second, meters, Newtons, ...)");
	if(getInDegrees()) {
		strcat(descrip,"\nAngles are in degrees.");
	} else {
		strcat(descrip,"\nAngles are in radians.");
	}
	strcat(descrip,"\n\n");
	setDescription(descrip);
}
//_____________________________________________________________________________
/**
 * Allocate storage for the muscle variables.
 */
void MuscleAnalysisV1::
allocateStorageObjects()
{
	if(_model==NULL) return;
	if (!getOn()) return;

	// CLEAR EXISTING WORK ARRAYS
	_storageList.setMemoryOwner(true);
	_storageList.setSize(0);
	_momentArmStorageArray.setMemoryOwner(true);
	_momentArmStorageArray.setSize(0);
	_muscleArray.setMemoryOwner(false);
	_muscleArray.setSize(0);

	// FOR MOMENT ARMS AND MOMEMTS
	const CoordinateSet& qSet = _model->getCoordinateSet();
	int nq = qSet.getSize();
	Storage *store;
	for(int i=0;i<nq;i++) {
		const Coordinate& q = qSet.get(i);
		string name = "MomentArm_" + q.getName();
		store = new Storage(1000,name);
		store->setDescription(getDescription());
		_storageList.append(store);
	}
	for(int i=0;i<nq;i++) {
		const Coordinate& q = qSet.get(i);
		string name = "Moment_" + q.getName();
		store = new Storage(1000,name);
		store->setDescription(getDescription());
		_storageList.append(store);
	}

	// EVERYTHING ELSE
	//_storageList.setMemoryOwner(false);

	_pennationAngleStore = new Storage(1000,"PennationAngle");
	_pennationAngleStore->setDescription(getDescription());
	_storageList.append(_pennationAngleStore );

	_lengthStore = new Storage(1000,"Length");
	_lengthStore->setDescription(getDescription());
	_storageList.append(_lengthStore );

	_fiberLengthStore = new Storage(1000,"FiberLength");
	_fiberLengthStore->setDescription(getDescription());
	_storageList.append(_fiberLengthStore );

	_normalizedFiberLengthStore = new Storage(1000,"NormalizedFiberLength");
	_normalizedFiberLengthStore->setDescription(getDescription());
	_storageList.append(_normalizedFiberLengthStore );

	_tendonLengthStore = new Storage(1000,"TendonLength");
	_tendonLengthStore->setDescription(getDescription());
	_storageList.append(_tendonLengthStore );

	_forceStore = new Storage(1000,"TendonForce");
	_forceStore->setDescription(getDescription());
	_storageList.append(_forceStore );

	_fiberForceStore = new Storage(1000,"FiberForce");
	_fiberForceStore->setDescription(getDescription());
	_storageList.append(_fiberForceStore );

	_activeFiberForceStore = new Storage(1000,"ActiveFiberForce");
	_activeFiberForceStore->setDescription(getDescription());
	_storageList.append(_activeFiberForceStore );

	_passiveFiberForceStore = new Storage(1000,"PassiveFiberForce");
	_passiveFiberForceStore->setDescription(getDescription());
	_storageList.append(_passiveFiberForceStore );

	_activeFiberForceAlongTendonStore = new Storage(1000,"ActiveFiberForceAlongTendon");
	_activeFiberForceAlongTendonStore->setDescription(getDescription());
	_storageList.append(_activeFiberForceAlongTendonStore );

	_passiveFiberForceAlongTendonStore = new Storage(1000,"PassiveFiberForceAlongTendon");
	_passiveFiberForceAlongTendonStore->setDescription(getDescription());
	_storageList.append(_passiveFiberForceAlongTendonStore );

	//MM
	_dactStore = new Storage(1000,"d/dt a"); 
	_dactStore->setDescription(getDescription());	//Weird! What is this doing? How does this work?  
	_storageList.append(_dactStore);				//Some function of the inherited class I guess

	_actStore = new Storage(1000, "a");
	_actStore->setDescription(getDescription());
	_storageList.append(_actStore);

	_falStore = new Storage(1000, "Norm. fal");
	_falStore->setDescription(getDescription());
	_storageList.append(_falStore);

	_fseStore = new Storage(1000, "Norm. fse");
	_fseStore->setDescription(getDescription());
	_storageList.append(_fseStore);
	
	_fpeStore		= new Storage(1000, "Norm. fpe"); 
	_fpeStore->setDescription(getDescription());
	_storageList.append(_fpeStore);

	_fvStore		= new Storage(1000, "Norm. fv");
	_fvStore->setDescription(getDescription());
	_storageList.append(_fvStore);

	_fvVmaxStore	= new Storage(1000, "Norm. Vmax");
	_fvVmaxStore->setDescription(getDescription());
	_storageList.append(_fvVmaxStore);

	_tlStore		= new Storage(1000, "Norm. tl");
	_tlStore->setDescription(getDescription());
	_storageList.append(_tlStore);

	_lceStore		= new Storage(1000, "Norm. lce");
	_lceStore->setDescription(getDescription());
	_storageList.append(_lceStore);

	_dlceStore		= new Storage(1000, "Norm. dlce"); 
	_dlceStore->setDescription(getDescription());
	_storageList.append(_dlceStore);

	_uStore			= new Storage(1000, "Ex.");  
	_uStore->setDescription(getDescription());
	_storageList.append(_uStore);

	_caStore		= new Storage(1000, "cos(pen)");
	_caStore->setDescription(getDescription());
	_storageList.append(_caStore);

	_fvErrStore		= new Storage(1000, "fvErr");
	_fvErrStore->setDescription(getDescription());
	_storageList.append(_fvErrStore);

	_falErrStore		= new Storage(1000, "falErr");
	_falErrStore->setDescription(getDescription());
	_storageList.append(_falErrStore);

	_fpeErrStore		= new Storage(1000, "fpeErr");
	_fpeErrStore->setDescription(getDescription());
	_storageList.append(_fpeErrStore);

	_fseErrStore		= new Storage(1000, "fseErr");
	_fseErrStore->setDescription(getDescription());
	_storageList.append(_fseErrStore);

	_mcltenErrStore		= new Storage(1000, "MTErr");
	_mcltenErrStore->setDescription(getDescription());
	_storageList.append(_mcltenErrStore);

	_fsePEStore		= new Storage(1000, "TendonPE");
	_fsePEStore->setDescription(getDescription());
	_storageList.append(_fsePEStore);

	_fpePEStore		= new Storage(1000, "MusclePE");
	_fpePEStore->setDescription(getDescription());
	_storageList.append(_fpePEStore);

	_musclePWRStore		= new Storage(1000, "MusclePWR");
	_musclePWRStore->setDescription(getDescription());
	_storageList.append(_musclePWRStore);

	_muscleFStore		= new Storage(1000, "MuscleF");
	_muscleFStore->setDescription(getDescription());
	_storageList.append(_muscleFStore);

	_muscleVStore		= new Storage(1000, "MuscleV");
	_muscleVStore->setDescription(getDescription());
	_storageList.append(_muscleVStore);


	_mclTdnKEPEWStore		= new Storage(1000, "MuscleTendonKEPEW");
	_mclTdnKEPEWStore->setDescription(getDescription());
	_storageList.append(_mclTdnKEPEWStore);

	// UPDATE ALL STORAGE OBJECTS
	updateStorageObjects();

	

}
//_____________________________________________________________________________
/**
 * Update storage objects.  This is necessary if the modle, mucle, or
 * coordinate list is changed.
 */
void MuscleAnalysisV1::
updateStorageObjects()
{
	if(_model==NULL) return;
	if (!getOn()) return;

	// POPULATE MUSCLE LIST FOR "all"
	ForceSet& fSet = _model->updForceSet();
	_muscleList = _muscleListProp.getValueStrArray();
	int nm = _muscleList.getSize();
	if((nm==1) && (_muscleList.get(0)=="all")) {
		_muscleList.setSize(0);
		int nf = fSet.getSize();
		for(int i=0;i<nf;i++) {
			Muscle *m = dynamic_cast<Muscle*>(&fSet.get(i));
            if( m ) _muscleList.append(m->getName());
		}
	}
	// POPULATE ACTIVE MUSCLE ARRAY
	Array<string> tmpMuscleList("");
	nm = _muscleList.getSize();
	_muscleArray.setSize(0);
	for(int i=0; i<nm; i++) {
		if(fSet.contains(_muscleList[i])) {
    		Muscle* mus = dynamic_cast<Muscle*>( &fSet.get(_muscleList[i]) );
			_muscleArray.append(mus);
			tmpMuscleList.append(mus->getName());
		}
	}
	_muscleList = tmpMuscleList;

	// POPULATE COORDINATE LIST FOR "all"
	CoordinateSet& qSet = _model->updCoordinateSet();
	_coordinateList = _coordinateListProp.getValueStrArray();
	int nq = qSet.getSize();
	int nActiveQ = _coordinateList.getSize();
	if((nActiveQ==1) && (_coordinateList.get(0)=="all")) {
		_coordinateList.setSize(0);
		for(int i=0;i<nq;i++) {
			Coordinate& q = qSet.get(i);
			_coordinateList.append(q.getName());
		}
	}
	// POPULATE ACTIVE MOMENT ARM ARRAY
	Array<string> tmpCoordinateList("");  // For making sure the coordinates in the list really exist.
	_momentArmStorageArray.setSize(0);
	nActiveQ = _coordinateList.getSize();
	for(int i=0; i<nActiveQ; i++) {
		string name = _coordinateList[i];
		for(int j=0; j<nq; j++) {
			Coordinate& q = qSet.get(j);
			if(name == q.getName()) {
				StorageCoordinatePair *pair = new StorageCoordinatePair();
				pair->q = &q;
				pair->momentArmStore = _storageList[j];
				pair->momentStore = _storageList[j+nq];
				_momentArmStorageArray.append(pair);
				tmpCoordinateList.append(q.getName());
			}
		}
	}
	_coordinateList = tmpCoordinateList;
	//cout<<"Number of active moment arm storage array = "<<_momentArmStorageArray.getSize()<<endl;

	// CONSTRUCT AND SET COLUMN LABELS
	constructColumnLabels();
	Storage *store;
	int size = _storageList.getSize();
	for(int i=0;i<size;i++) {
		store = _storageList[i];
		if(store==NULL) continue;
		store->setColumnLabels(getColumnLabels());
	}
}
//-----------------------------------------------------------------------------
// COLUMN LABELS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the column labels for the MuscleAnalysisV1 storage files.
 */
void MuscleAnalysisV1::
constructColumnLabels()
{
	if(!_model) return;
	int size = _muscleList.getSize();
	Array<string> labels("",size+1);
	labels[0] = "time";
	for(int i=0; i<size; i++) {
		labels[i+1] = _muscleList[i];
	}
	setColumnLabels(labels);
}


//=============================================================================
// OPERATORS
//=============================================================================
MuscleAnalysisV1& MuscleAnalysisV1::operator=(const MuscleAnalysisV1 &aAnalysis)
{
	// BASE CLASS
	Analysis::operator=(aAnalysis);

	// MEMBER VARIABLES
	_muscleListProp = aAnalysis._muscleListProp;
	_coordinateListProp = aAnalysis._coordinateListProp;
	_computeMoments = aAnalysis._computeMoments;
	allocateStorageObjects();

	return (*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the model pointer for analysis.
 */
void MuscleAnalysisV1::setModel(Model& aModel)
{
	Analysis::setModel(aModel);
	allocateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Set the list of muscles to analyze.
 *
 * @param aMuscles is the array of names of muscles to analyze.
 */
void MuscleAnalysisV1::
setMuscles(OpenSim::Array<std::string>& aMuscles)
{
	int size = aMuscles.getSize();
	_muscleListProp.getValueStrArray().setSize(aMuscles.getSize());
	for(int i=0; i<size; i++){
		_muscleListProp.getValueStrArray().get(i) = aMuscles.get(i);
	}
	updateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Set the list of coordinates.
 *
 * @param aCoordinates Array of coordinates about which to compute moment arms.
 */
void MuscleAnalysisV1::
setCoordinates(OpenSim::Array<std::string>& aCoordinates)
{
	int size = aCoordinates.getSize();
	_coordinateListProp.getValueStrArray().setSize(size);
	for(int i=0; i<size; i++){
		_coordinateListProp.getValueStrArray().get(i) = aCoordinates[i];
	}
	updateStorageObjects();
}
//-----------------------------------------------------------------------------
// STORAGE CAPACITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the capacity increments of all storage instances.
 *
 * @param aIncrement Increment by which storage capacities will be increased
 * when storage capcities run out.
 */
void MuscleAnalysisV1::
setStorageCapacityIncrements(int aIncrement)
{
	if(!_model) return;
	Storage *store;
	int size = _storageList.getSize();
	for(int i=0;i<size;i++) {
		store = _storageList[i];
		if(store==NULL) continue;
		store->setCapacityIncrement(aIncrement);
	}
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the MuscleAnalysisV1 quantities.
 */
int MuscleAnalysisV1::
record(const SimTK::State& s)
{
	if(_model==NULL) return(-1);
	if (!getOn()) return(-1);

	// MAKE SURE ALL ACTUATION QUANTITIES ARE VALID
	// COMPUTE DERIVATIVES
	// ----------------------------------
	// TIME NORMALIZATION
	double tReal = s.getTime();
	// ----------------------------------
	// LOOP THROUGH MUSCLES
	int nm = _muscleArray.getSize();
	Array<double> penang(0.0,nm);
	Array<double> len(0.0,nm),tlen(0.0,nm);
	Array<double> fiblen(0.0,nm),normfiblen(0.0,nm);
	Array<double> force(0.0,nm),fibforce(0.0,nm);
	Array<double> actfibforce(0.0,nm),passfibforce(0.0,nm);
	Array<double> actfibforcealongten(0.0,nm),passfibforcealongten(0.0,nm);

	Array<double> dact(0.0,nm);		//MM
	Array<double> act(0.0,nm);		//MM
	Array<double> fal(0.0,nm);		//MM
	Array<double> fse(0.0,nm);		//MM
	Array<double> fpe(0.0,nm);		//MM
	Array<double> fv(0.0,nm);		//MM
	Array<double> fvVmax(0.0,nm);	//MM
	Array<double> tl(0.0,nm);		//MM
	Array<double> lce(0.0,nm);		//MM
	Array<double> dlce(0.0,nm);		//MM
	Array<double> u(0.0,nm);		//MM
	Array<double> ca(0.0,nm);		//MM

	Array<double> fsePE(0.0,nm);	//MM
	Array<double> fpePE(0.0,nm);	//MM
	Array<double> mclPWR(0.0,nm);		//MM
	Array<double> mclF(0.0,nm);		//MM
	Array<double> mclV(0.0,nm);		//MM

	Array<double> mclTdnKEPEW(0.0,nm);		//MM

	Array<double> errfv(0.0,nm);	//MM
	Array<double> errfal(0.0,nm);	//MM
	Array<double> errfse(0.0,nm);	//MM
	Array<double> errfpe(0.0,nm);	//MM
	Array<double> errMT(0.0,nm);	//MM

	double sumfv;	//MM
	double sumfal;	//MM
	double sumfse;	//MM
	double sumfpe;	//MM

	double fv_std	= 0;
	double fal_std	= 0;
	double fse_std	= 0;
	double fpe_std	= 0;

	double eps = 1e-6; //Likely the integrator error

	cout << s.getTime() <<endl;

	for(int i=0; i<nm; i++) {
		//fv[i] = _muscleArray[i]->getfv(s); //MM2011
		penang[i] = _muscleArray[i]->getPennationAngle(s);
		len[i] = _muscleArray[i]->getLength(s);
		tlen[i] = _muscleArray[i]->getTendonLength(s);
		fiblen[i] = _muscleArray[i]->getFiberLength(s);
		normfiblen[i] = _muscleArray[i]->getNormalizedFiberLength(s);

		// Compute muscle forces that are dependent on Positions, Velocities
		// so that later quantities are valid and setForce is called
		_muscleArray[i]->computeActuation(s);
		force[i] = _muscleArray[i]->getForce(s);
		fibforce[i] = _muscleArray[i]->getFiberForce(s);
		actfibforce[i] = _muscleArray[i]->getActiveFiberForce(s);
		passfibforce[i] = _muscleArray[i]->getPassiveFiberForce(s);
		actfibforcealongten[i] = _muscleArray[i]->getActiveFiberForceAlongTendon(s);
		passfibforcealongten[i] = _muscleArray[i]->getPassiveFiberForceAlongTendon(s);

		//MM the extra values required to characterize the muscle
		//Muscle* mus = dynamic_cast<Muscle*>( &fSet.get(_muscleList[i]) );
		Thelen2003MuscleV1* _tmuscle = dynamic_cast<Thelen2003MuscleV1*>(_muscleArray[i]);
		dact[i] = _tmuscle->getdactdt();
		act[i]	= _tmuscle->getact();
		fal[i]	= _tmuscle->getfal();
		fse[i]	= _tmuscle->getfse();
		fpe[i]	= _tmuscle->getfpe();
		fv[i]	= _tmuscle->getfv();
		fvVmax[i]	= _tmuscle->getfvVmax();
		tl[i]	= _tmuscle->gettl();
		lce[i]	= _tmuscle->getlce();
		dlce[i]	= _tmuscle->getdlce();
		u[i]	= _tmuscle->getu();
		ca[i]	= _tmuscle->getca();

		fsePE[i]	= _tmuscle->getTendonPE();
		fpePE[i]	= _tmuscle->getMusclePE();
		mclPWR[i]		= _tmuscle->getMusclePWR();
		mclF[i]		= _tmuscle->getMuscleF();
		mclV[i]		= _tmuscle->getMuscleV();
		mclTdnKEPEW[i] = 0.0; //We're not integrating power into work just yet.


		//MM Compute simulated muscle relative to standard physiologic curves
		fv_std	= get1DSplineValue(_ncs_stdfv,  dlce[i]);
		fal_std	= get1DSplineValue(_ncs_stdfal, lce[i]);
		fse_std	= get1DSplineValue(_ncs_stdfse, tl[i]+1.0);
		fpe_std	= get1DSplineValue(_ncs_stdfpe, lce[i]);

		errfv[i]	= fv[i]  - fv_std;
		errfal[i]	= fal[i] - fal_std;
		errfse[i]	= fse[i] - fse_std;
		errfpe[i]	= fpe[i] - fpe_std;

		sumfv	= abs(fv[i]) + abs(fv_std) + eps;
		sumfal	= abs(fal[i]) + abs(fal_std) + eps;
		sumfse	= abs(fse[i]) + abs(fse_std) + eps;
		sumfpe	= abs(fpe[i]) + abs(fpe_std) + eps;

		errMT[i] = (abs(errfv[i]/sumfv)+abs(errfal[i]/sumfal)+abs(errfse[i]/sumfse)+abs(errfpe[i]/sumfpe))*(0.25) ;  

	}
	// APPEND TO STORAGE
	_pennationAngleStore->append(tReal,penang.getSize(),&penang[0]);
	_lengthStore->append(tReal,len.getSize(),&len[0]);
	_fiberLengthStore->append(tReal,fiblen.getSize(),&fiblen[0]);
	_normalizedFiberLengthStore->append(tReal,normfiblen.getSize(),&normfiblen[0]);
	_tendonLengthStore->append(tReal,tlen.getSize(),&tlen[0]);
	_forceStore->append(tReal,force.getSize(),&force[0]);
	_fiberForceStore->append(tReal,fibforce.getSize(),&fibforce[0]);
	_activeFiberForceStore->append(tReal,actfibforce.getSize(),&actfibforce[0]);
	_passiveFiberForceStore->append(tReal,passfibforce.getSize(),&passfibforce[0]);
	_activeFiberForceAlongTendonStore->append(tReal,actfibforcealongten.getSize(),&actfibforcealongten[0]);
	_passiveFiberForceAlongTendonStore->append(tReal,passfibforcealongten.getSize(),&passfibforcealongten[0]);

	//MM Append to storage
	_dactStore->append(tReal,dact.getSize(),&dact[0]);
	_actStore->append(tReal,act.getSize(),&act[0]);
	_falStore->append(tReal,fal.getSize(),&fal[0]);
	_fseStore->append(tReal,fse.getSize(),&fse[0]);
	_fpeStore->append(tReal,fpe.getSize(),&fpe[0]);
	_fvStore->append(tReal,fv.getSize(),&fv[0]);
	_fvVmaxStore->append(tReal,fvVmax.getSize(),&fvVmax[0]);
	_tlStore->append(tReal,tl.getSize(),&tl[0]);
	_lceStore->append(tReal,lce.getSize(),&lce[0]);
	_dlceStore->append(tReal,dlce.getSize(),&dlce[0]);
	_uStore->append(tReal,u.getSize(),&u[0]);
	_caStore->append(tReal,ca.getSize(),&ca[0]);

	_fsePEStore->append(tReal,fsePE.getSize(), &fsePE[0]);
	_fpePEStore->append(tReal,fpePE.getSize(), &fpePE[0]);
	_musclePWRStore->append(tReal,mclPWR.getSize(), &mclPWR[0]);
	_muscleFStore->append(tReal,mclF.getSize(), &mclF[0]);
	_muscleVStore->append(tReal,mclV.getSize(), &mclV[0]);
	_mclTdnKEPEWStore->append(tReal,mclTdnKEPEW.getSize(), &mclTdnKEPEW[0]);

	_fvErrStore->append(tReal,errfv.getSize(),&errfv[0]);
	_falErrStore->append(tReal,errfal.getSize(),&errfal[0]);
	_fpeErrStore->append(tReal,errfpe.getSize(),&errfpe[0]);
	_fseErrStore->append(tReal,errfse.getSize(),&errfse[0]);
	_mcltenErrStore->append(tReal,errMT.getSize(),&errMT[0]);


	if (_computeMoments){
	// LOOP OVER ACTIVE MOMENT ARM STORAGE OBJECTS
	Coordinate *q = NULL;
	Storage *maStore=NULL, *mStore=NULL;
	int nq = _momentArmStorageArray.getSize();
	Array<double> ma(0.0,nm),m(0.0,nm);

   _model->getMultibodySystem().realize(s,SimTK::Stage::Velocity);  // need to be at Velocity to compaute path length 

	for(int i=0; i<nq; i++) {

		q = _momentArmStorageArray[i]->q;
		maStore = _momentArmStorageArray[i]->momentArmStore;
		mStore = _momentArmStorageArray[i]->momentStore;
       
		// Make a writable copy of the state so moment arm can be computed
		SimTK::State tempState = s;

		bool locked = q->getLocked(tempState);

		_model->getMultibodySystem().realize(tempState, s.getSystemStage() );
		// LOOP OVER MUSCLES
		for(int j=0; j<nm; j++) {
            ma[j] = _muscleArray[j]->computeMomentArm(tempState,*q);
			m[j] = ma[j] * force[j];
		}
		maStore->append(s.getTime(),nm,&ma[0]);
		mStore->append(s.getTime(),nm,&m[0]);
	}
	}
	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration 
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s current system state
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysisV1::
begin(SimTK::State& s )
{
	if(!proceed()) return(0);

	// RESET STORAGE
	Storage *store;
	int size = _storageList.getSize();
	for(int i=0;i<size;i++) {
		store = _storageList[i];
		if(store==NULL) continue;
		store->purge();
	}

	// RECORD
	int status = 0;
	// Make sure cooridnates are not locked
	if (_computeMoments){
	// LOOP OVER ACTIVE MOMENT ARM STORAGE OBJECTS
		Coordinate *q = NULL;
		int nq = _momentArmStorageArray.getSize();
		for(int i=0; i<nq; i++) {
			q = _momentArmStorageArray[i]->q;
			if (q->getLocked(s))
				throw(Exception("Coordinate: "+q->getName()+" is locked and can't be varied. Aborting.")); 
		}
	}
	if(_storageList.getSize()> 0 && _storageList.get(0)->getSize() <= 0) status = record(s);

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called 
 *
 * This method should be overriden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param s current state of system
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysisV1::
step(const SimTK::State& s, int stepNumber )
{
	if(!proceed(stepNumber)) return(0);

	int status = record(s);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s current state of system
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysisV1::
end(SimTK::State& s )
{
	if (!proceed()) return 0;
	record(s);
	return(0);
}

//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration 
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysisV1::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	if(!getOn()) {
		printf("MuscleAnalysisV1.printResults: Off- not printing.\n");
		return(0);
	}

	std::string prefix = aBaseName + "_" + getName() + "_";
	Storage::printResult(_pennationAngleStore,prefix+"PennationAngle",aDir,aDT,aExtension);
	Storage::printResult(_lengthStore,prefix+"Length",aDir,aDT,aExtension);
	Storage::printResult(_fiberLengthStore,prefix+"FiberLength",aDir,aDT,aExtension);
	Storage::printResult(_normalizedFiberLengthStore,prefix+"NormalizedFiberLength",aDir,aDT,aExtension);
	Storage::printResult(_tendonLengthStore,prefix+"TendonLength",aDir,aDT,aExtension);
	Storage::printResult(_forceStore,prefix+"Force",aDir,aDT,aExtension);
	Storage::printResult(_fiberForceStore,prefix+"FiberForce",aDir,aDT,aExtension);
	Storage::printResult(_activeFiberForceStore,prefix+"ActiveFiberForce",aDir,aDT,aExtension);
	Storage::printResult(_passiveFiberForceStore,prefix+"PassiveFiberForce",aDir,aDT,aExtension);
	Storage::printResult(_activeFiberForceAlongTendonStore,prefix+"ActiveFiberForceAlongTendon",aDir,aDT,aExtension);
	Storage::printResult(_passiveFiberForceAlongTendonStore,prefix+"PassiveFiberForceAlongTendon",aDir,aDT,aExtension);

	int size = _momentArmStorageArray.getSize();
	for(int i=0;i<size;i++) {
		string fileName = prefix + _momentArmStorageArray.get(i)->momentArmStore->getName();
		Storage::printResult(_momentArmStorageArray.get(i)->momentArmStore,fileName,aDir,aDT,aExtension);
		fileName = prefix + _momentArmStorageArray.get(i)->momentStore->getName();
		Storage::printResult(_momentArmStorageArray.get(i)->momentStore,fileName,aDir,aDT,aExtension);
	}

	//MM print all of the variables I'm interested in
	Storage::printResult(_dactStore,prefix+"dact",aDir,aDT,aExtension);
	Storage::printResult(_actStore,prefix+"act",aDir,aDT,aExtension);
	Storage::printResult(_falStore,prefix+"falN",aDir,aDT,aExtension);
	Storage::printResult(_fseStore,prefix+"fseN",aDir,aDT,aExtension);
	Storage::printResult(_fpeStore,prefix+"fpeN",aDir,aDT,aExtension);
	Storage::printResult(_fvStore,prefix+"fvN",aDir,aDT,aExtension);
	Storage::printResult(_fvVmaxStore,prefix+"fvVmax",aDir,aDT,aExtension);
	Storage::printResult(_tlStore,prefix+"tlN",aDir,aDT,aExtension);
	Storage::printResult(_lceStore,prefix+"lceN",aDir,aDT,aExtension);
	Storage::printResult(_dlceStore,prefix+"dlceN",aDir,aDT,aExtension);
	Storage::printResult(_uStore,prefix+"ex",aDir,aDT,aExtension);
	Storage::printResult(_caStore,prefix+"ca",aDir,aDT,aExtension);
	

	Storage::printResult(_fsePEStore,prefix+"tendonPE",aDir,aDT,aExtension);
	Storage::printResult(_fpePEStore,prefix+"musclePE",aDir,aDT,aExtension);
	Storage::printResult(_musclePWRStore,prefix+"musclePWR",aDir,aDT,aExtension);
	Storage::printResult(_muscleFStore,prefix+"muscleF",aDir,aDT,aExtension);
	Storage::printResult(_muscleVStore,prefix+"muscleV",aDir,aDT,aExtension);
	Storage::printResult(_mclTdnKEPEWStore,prefix+"mclTdnKEPEW",aDir,aDT,aExtension);
	

	Storage::printResult(_fvErrStore,prefix+"fvERR",aDir,aDT,aExtension);
	Storage::printResult(_falErrStore,prefix+"falERR",aDir,aDT,aExtension);
	Storage::printResult(_fpeErrStore,prefix+"fpeERR",aDir,aDT,aExtension);
	Storage::printResult(_fseErrStore,prefix+"fseERR",aDir,aDT,aExtension);
	Storage::printResult(_mcltenErrStore,prefix+"mcltenERR",aDir,aDT,aExtension);

	/**
	_dactStore = NULL; 
	_actStore = NULL; 
	_falStore = NULL; 
	_fseStore = NULL;
	_fpeStore = NULL; 
	_fvStore = NULL;
	_fvVmaxStore = NULL; 
	_tlStore = NULL;
	_lceStore = NULL;
	_dlceStore = NULL; 
	_uStore = NULL; 
	*/

	return(0);
}

//_____________________________________________________________________________
/**
 * MM 2011 10 18
 *
 * This method is called when the constructor is executed. It is used to fetch data from
 * a user specified *.sto file, spline interpolate it using the NaturalCubicSpline
 * class and return the result. Note that the columns of the *.sto file need to be
 * 'time' and then 'col0'. Why? Because the class that reads in the data requires it.
 * This should be fixed.
 *
 * @param aFileName: a reference to a string of the filename to read. 
 * @returns NaturalCubicSpline* to the interpolated curve in the file.
 */
NaturalCubicSpline* MuscleAnalysisV1::get1DSpline(const std::string &aFileName){
	
	Storage curvefal(aFileName);
	OpenSim::Array<double> curvefalX, curvefalY;
	curvefalX.setSize(curvefal.getSize());
	curvefalY.setSize(0);	//MM:This is necessary as getDataColumn appends to this reference!
							//	 the documentation would have you think that the array needs to be
							//   its full size, and then it is populated. Very confusing!
	curvefal.getTimeColumn(curvefalX); //MM: Storage assumes the first column is time ... this does not generalize well
	string colname = "col0";
	curvefal.getDataColumn(colname,curvefalY,curvefalX[0]); //MM: I'm using this because I cannot grab the column
															 //    data by index alone - it barfs when I ask for column 0
															 //    (not sure why) when I use the function that would 
															 //    otherwise do this.

	return new NaturalCubicSpline(curvefalX.getSize(),&curvefalX[0],&curvefalY[0],"Active-Force Length Curve Approximation");
}


//_____________________________________________________________________________
/**
 *  MM 2011 10 18
 *
 * This function will compute the value of a 1D spline given an argument. I've written
 * this because the window dressing code required to actually do this simple operation is
 * large enough that it will pollute my code, and be annoying to re-write over and over 
 * again.
 *
 * @param aSpline: A pointer to the natural cubic spline you'd like to interpolate
 * @param xval: The value at which you'd like to evaluate the spline
 * @return double of the value of the spline at xval.
 */
double 	MuscleAnalysisV1::get1DSplineValue(const NaturalCubicSpline *aSpline, double xval){	
	SimTK::Vector tmpvec(aSpline->getArgumentSize(), 0.0);
	for(int i=0; i<aSpline->getArgumentSize();i++ )
		tmpvec[i] = xval;
		return aSpline->calcValue(tmpvec);
}