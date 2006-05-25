// Model.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/Memory.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/PropertyObj.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/PropertyStr.h>
#include "BodySet.h"
#include "Model.h"
#include "Body.h"
#include "Analysis.h"
#include "AnalysisSet.h"
#include "Callback.h"
#include "IntegCallback.h"
#include "IntegCallbackSet.h"
#include "DerivCallback.h"
#include "DerivCallbackSet.h"
#include <OpenSim/Tools/MaterialSet.h>

//=============================================================================
// STATIC CONSTANTS
//=============================================================================




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 *
 * Note that analyses registered with the model should not be deleted by the
 * model but by the calling code that created the analyses.
 */
Model::~Model()
{
	if(_analysisSet!=NULL) {
		delete _analysisSet; _analysisSet=0;
	}
	if(_integCallbackSet!=NULL) {
		delete _integCallbackSet;  _integCallbackSet=NULL;
	}
	if(_derivCallbackSet!=NULL) {
		delete _derivCallbackSet;  _derivCallbackSet=NULL;
	}
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Model::Model():
	_propBodySet(PropertyObj("Bodies", BodySet())),
	_b((BodySet&)_propBodySet.getValueObj()),
	_propMaterialSet(PropertyObj("Materials", MaterialSet())),
	_materialSet( (MaterialSet&)_propMaterialSet.getValueObj())
{
	setNull();
	setupProperties();
	init();
}
//_____________________________________________________________________________
/**
 * Construct a model from an XML properties file.
 *
 * @param aFileName Name of the file.
 */
Model::Model(const string &aFileName) :
Object(aFileName),
_propBodySet(PropertyObj("Bodies", BodySet())),
_b((BodySet&)_propBodySet.getValueObj()),
_propMaterialSet(PropertyObj("Materials", MaterialSet())),
_materialSet( (MaterialSet&)_propMaterialSet.getValueObj())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
	init();
}
//_____________________________________________________________________________
/**
 * Construct a model from an XML element.
 *
 * @param aElement XML element.
 */
Model::Model(DOMElement *aElement) : 
	Object(aElement),
	_propBodySet(PropertyObj("Bodies", BodySet())),
	_b((BodySet&)_propBodySet.getValueObj()),
	_propMaterialSet(PropertyObj("Materials", MaterialSet())),
	_materialSet( (MaterialSet&)_propMaterialSet.getValueObj())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
	init();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the values of all data members to an appropriate "null" value.
 */
void Model::
setNull()
{
	_nb = 0;
	_g[0] = _g[1] = _g[2] = 0.0;
   setName("Model");
	_t = 0.0;
	_tNormConst = 1.0;
	_analysisSet = NULL;
	_integCallbackSet = NULL;
	_derivCallbackSet = NULL;
	_modelDescriptionFileName = "";
}

//_____________________________________________________________________________
/**
 * Initialize variables post memory allocation.
 */
void Model::
init()
{
	// CALLBACK SETS
	_analysisSet = new AnalysisSet(this);
	_integCallbackSet = new IntegCallbackSet(this);
	_derivCallbackSet = new DerivCallbackSet(this);


	//_analysisSet->setMemoryOwner(false);
	//_integCallbackSet->setMemoryOwner(false);
	//_derivCallbackSet->setMemoryOwner(false);
}
//_____________________________________________________________________________
/**
 * Set up properties.
 *
 */
void Model::
setupProperties()
{
	_propertySet.append(&_propBodySet);
	_propBodySet.setName("Bodies");

	_propertySet.append(&_propMaterialSet);
	_materialSet.setName("Materials");
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.  The XML-associated variable
 * members are not copied-- the XML nodes and/or document must be generated
 * anew for a copied object.
 *
 * @return Reference to this object.
 * @see updateXMLNode()
 * @see generateXMLNode()
 */
Model& Model::operator=(const Model &aModel)
{
	_b = aModel._b;
	_materialSet = aModel._materialSet;

	for (int i = 0; i < 3; i++)
		_g[i] = aModel._g[i];

	_t = aModel._t;
	_tNormConst = aModel._tNormConst;;
	_nb = aModel._nb;

	// unused: _bNames;

	if (aModel._analysisSet)
	{
		if (!_analysisSet)
			_analysisSet = new AnalysisSet(this);
		for (i = 0; i < aModel._analysisSet->getSize(); i++)
		{
			Analysis* anAnalysis = new Analysis(aModel.getAnalysis(i));
			addAnalysis(anAnalysis);
		}
	}

	if (aModel._integCallbackSet)
	{
		if (!_integCallbackSet)
			_integCallbackSet = new IntegCallbackSet(this);
		for (i = 0; i < aModel._integCallbackSet->getSize(); i++)
		{
			IntegCallback* integ = new IntegCallback(aModel.getIntegCallback(i));
			addIntegCallback(integ);
		}
	}

	if (aModel._derivCallbackSet)
	{
		if (!_derivCallbackSet)
			_derivCallbackSet = new DerivCallbackSet(this);
		for (i = 0; i < aModel._derivCallbackSet->getSize(); i++)
		{
			DerivCallback* deriv = new DerivCallback(*(aModel._derivCallbackSet->getDerivCallback(i)));
			addDerivCallback(deriv);
		}
	}

	_modelDescriptionFileName = aModel._modelDescriptionFileName;

	return *this;
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// GRAVITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the acceleration due to gravity.
 */
void Model::
getGravity(double aG[3]) const
{
	aG[0] = _g[0];
	aG[1] = _g[1];
	aG[2] = _g[2];
}
//_____________________________________________________________________________
/**
 * Set the acceleration due to gravity.
 */
void Model::
setGravity(double aG[3])
{
	_g[0] = aG[0];
	_g[1] = aG[1];
	_g[2] = aG[2];
}
//-----------------------------------------------------------------------------
// NUMBERS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the number of bodies in this model, not counting the ground
 * inertial reference frame.
 */
int Model::
getNB() const
{
	return(_nb);
}

//-----------------------------------------------------------------------------
// NAMES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the name the model.
 *
 * @param aName Name of the model.
*/
void Model::
setName(const string &aName)
{
	// EMPTY?
	if(aName.empty()) {
		_name = DEFAULT_NAME;
		return;
	}
	
	_name = aName;
}

//_____________________________________________________________________________
/**
 * Set the name of a body.
 *
 * @param aIndex Index of the body whose name should be set.  aIndex should
 * be greater than or equal to 0 and less than the number of bodies.
 * @param aName Name of body.
 * @see getNB()
 */
void Model::
setBodyName(int aIndex,const string &aName)
{
	// CHECK
	if(aIndex<0) return;
	if(aIndex>=getNB()+1) return;

	_b.get(aIndex)->setName(aName);
}
//_____________________________________________________________________________
/**
 * Get the name of body.
 *
 * @param aIndex Index of the body whose name is desired.  aIndex should
 * be greater than or equal to 0 and less than the number of bodies.
 * @return Control name.
 * @see getNB()
 */
string Model::
getBodyName(int aIndex) const
{
	if(aIndex==getGroundID()) return("ground");
	if(aIndex<0) return("");
	if(aIndex>=getNB()+1) return(""); // Account for ground

	return(_b.get(aIndex)->getName());
}
//_____________________________________________________________________________
/**
 * Retrieve body with index
 */

Body *Model::
getBody(int aIndex) const
{
	if(aIndex<0 || aIndex>getNB()) 
		return(NULL);
	// Ground maybe special but we'll try to treat it as the rest for now. (body _nb) 
	int sz = _b.getSize();
	return _b[aIndex];

}
//_____________________________________________________________________________
/**
 * Construct blank bodies with no names, and unit Mass. Names, Geometry and other 
 * properties will be assigned later.
 * _nb is initialized from SDFast so it has proper number of bodies, use it here 
 * to populate the _b array first time around.
 */
void Model::
constructBodies()
{
	for(int i=0;i<=_nb;i++) {	// Changed loop into _nb+1 to have an actual ground body
		Body *nextBody = new Body();
		_b.append( nextBody );
	}
	_b[_nb]->setName("Ground Body");
}

//_____________________________________________________________________________
/**
 * Retrieve the material set associated with the model
 *
 */
BodySet *Model::getBodySet()
{
	return &_b;
}


//-----------------------------------------------------------------------------
// INDICES BY NAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the index of a body given its name.
 * The returned indices start at 0: for the first, 0 is returned; for
 * the second, 1; etc.
 *
 * Note that the body names need to be unique for this method
 * to work properly.
 *
 * @param aName Name of a body.
 * @return Index of the first body with the name aName; -1, if there is no
 * such body or if there is an error.  Indices start at 0.
 */
int Model::
getBodyIndex(const string &aName) const
{
	int i;
	for(i=0;i<getNB();i++) {
		if(aName == getBodyName(i)) return(i);
	}

	return(-1);
}


//_____________________________________________________________________________
/**
 * Get an object that lives in the model based on its name.
 *
 * This functionality depends on name uniqueness in the model
 * For now Model contains bodies, as more types are added
 * we may need to keep a hash-table for fast reverse lookup however that requires
 * keeping the table up to date. For now wew'll do exhaustive search!
 *
 * @param aName Name of the desired object.
 * @return Pointer to the desired object if it exists.  If it does not exist,
 * NULL is returned.
 */
VisibleObject *Model::
getVisibleObjectByName(const string &aName) const
{
	bool found = false;
	VisibleObject *foundObject = NULL;
	for(int i=0; (i<=getNB()) && (!found) ;i++){
		if(aName == getBodyName(i)) {
			found = true;
			foundObject = getBody(i);
		}
	}
	return(foundObject);
}
//-----------------------------------------------------------------------------
// TIME, CONTROLS, STATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the current time, controls, and states.
 *
 * This method is simply a convenience method for calling setTime(),
 * setControls(), and setStates().  This method, or all of the methods it
 * calls, should be called at the beginning of a dynamic analysis.
 *
 * @param aT Time.
 * @param aX Array of controls (length should be getNX()).
 * @param aY Array of states (length should be getNY()).
 */
void Model::
set(double aT,const double aX[],const double aY[])
{
	setTime(aT);
	setControls(aX);
	setStates(aY);
}
//-----------------------------------------------------------------------------
// The following two functions keep track of the xml file used in conjunction with the model
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Retrieve the name of the xml file associated with model
 *
 */
const char *Model::
getModelDescriptionFileName() const
{
	return _modelDescriptionFileName.c_str();
}
//_____________________________________________________________________________
/**
 * Set the name of the xml file associated with model
 *
 */
void Model::setModelDescriptionFileName(const string &aModelDescriptionFileName)
{
	_modelDescriptionFileName = aModelDescriptionFileName;
}
//_____________________________________________________________________________
/**
 * Register the types of objects used by Model and that need to be registered 
 * for xml serialization purposes
 *
 */
void Model::RegisterTypes()
{
	Object::RegisterType(VisibleObject());
	Object::RegisterType(VisibleProperties());
	Object::RegisterType(Transform());
	Object::RegisterType(Material());
	Object::RegisterType(Body());

}
//_____________________________________________________________________________
/**
 * Retrieve the material set associated with the model
 *
 */
MaterialSet *Model::getMaterialSet()
{
	return &_materialSet;
}


//-----------------------------------------------------------------------------
// TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the current time.
 *
 * @param Current time.
 */
void Model::
setTime(double aT)
{
	_t = aT;
}
//_____________________________________________________________________________
/**
 * Get the current time.
 *
 * @return Current time.
 */
double Model::
getTime() const
{
	return(_t);
}

//-----------------------------------------------------------------------------
// TIME NORMALIZATION CONSTANT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the constant by which time is normalized.
 *
 * The normalization constant must be greater than or equal to the constant
 * rdMath::ZERO.
 *
 * @param Time normalization constant.
 */
void Model::
setTimeNormConstant(double aNormConst)
{
	_tNormConst = aNormConst;
	if(_tNormConst < rdMath::ZERO) _tNormConst = rdMath::ZERO; 
}
//_____________________________________________________________________________
/**
 * Get the constant by which time is normalized.
 *
 * By default, the time normalization constant is 1.0.
 *
 * @return Current time.
 */
double Model::
getTimeNormConstant() const
{
	return(_tNormConst);
}

//-----------------------------------------------------------------------------
// STATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the current states for this model.
 *
 * The states are those quantities that are integrated during a simulation.
 * In all cases the states should be ordered so that the first NQ states
 * correspond to the generalized coordinates and the next NU states correspond
 * to the generalized speeds.
 *
 * When the states are set the configuration of the model is also
 * set so this method may be called in place of setConfiguration().
 *
 * @param aY Array of states.  The size of aY should be the value returned by
 * getNY().
 * @see setConfiguration().
void Model::
setStates(const double aY[])
{
	printf("Model.setStates: WARN- should be overriden.\n!");
}
 */
//_____________________________________________________________________________
/**
 * Get the current states for this model.
 *
 * The states are those quantities that are integrated during a simulations.
 *
 * @param rY Array to be filled with copies of the states.  The length of rY
 * must be at least as large as the value returned by getNY().
void Model::
getStates(double rY[]) const
{
	printf("Model.getStates: WARN- should be overriden.\n!");
}
 */
//_____________________________________________________________________________
/**
 * Get the current value of a state by index.
 *
 * @param aIndex Index of the state:  0 <= aIndex < getNY().
 * @return Value of the state.  rdMath::NAN is returned on an error.
 * @see getStates(double rY[])
 * @see getState(const char* aName);
double Model::
getState(int aIndex) const
{
	printf("Model.getState: WARN- should be overriden.\n!");
	return(0.0);
}
 */

//-----------------------------------------------------------------------------
// ACCELERATIONS
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// INERTIAL PARAMETERS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the mass of body aBody.
 *
 * aBody must be in the range 0 <= aBody < NumBodies.
 * If it is not, a mass of -1.0 is returned.
 */
double Model::
getMass(int aBody) const
{
	// CHECK aBody
	if((aBody<0)||(aBody>=_nb)) return(-1.0);

	// GET M
	double M = _b[aBody]->getMass();

	return(M);
}
//_____________________________________________________________________________
/**
 * Get the scalar inertial matrix of a body.
 *
 * @param aBody Body ID.
 * @param rI Inertia matrix expressed in terms of body-local coordinates.
 * @return -1 on an error, 0 otherwise.
 */
int Model::
getInertiaBodyLocal(int aBody,double rI[3][3]) const
{
	// CHECK aBody
	if((aBody<0)||(aBody>=_nb)) return(-1);

	// GET I
	_b[aBody]->getInertia(rI);

	return(0);
}

//=============================================================================
// EQUATIONS OF MOTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute an effective mass matrix given a Jacobian matrix and the inverse
 * of a system mass matrix.
 *
 * Ieff = Inverse( aJ * Iinv * JT)
 *
 * @param aNJX Number of rows in the Jacobian matrix.
 * @param aNU Number of degrees of freedom or generalized speeds.
 * @param aJ Pointer to a Jacobian (aJ[NJX][NU]).
 * @param aIinv Pointer to the inverse of a mass matrix (aIinv[NU][NU]).
 * @param aIeff Effective mass matrix (aIeff[NJX][NJX]).
 * 
 * @return 0 when successful, -1 if an error is encountered, -2 when a
 * singularity is encountered when performing the matrix inversion.
 */
int Model::
ComputeEffectiveMassMatrix(int aNJX,int aNU,const double *aJ,
	const double *aIinv,double *rIeff)
{
	if(aNJX<=0) return(-1);
	if(aNU<=0) return(-1);
	if(aJ==NULL) return(-1);
	if(aIinv==NULL) return(-1);
	if(rIeff==NULL) return(-1);

	// CREATE WORK MATRIX (NU x NJX)
	double *m_NU_NJX = new double[aNU*aNJX];

	// COMPUTE IInv * JT (aNU x aNJX)
	Mtx::Transpose(aNJX,aNU,aJ,m_NU_NJX);
	//printf("\nSDFast.formJacobianInverse: JT...\n");
	//Mtx::Print(aNU,aNJX,m_NU_NJX,3);
	Mtx::Multiply(aNU,aNU,aNJX,aIinv,m_NU_NJX,m_NU_NJX);
	//printf("\nSDFast.formJacobianInverse: IInv_JT...\n");
	//Mtx::Print(aNU,aNJX,IInv_JT,3);

	// COMPUTE Inverse(J * Iinv * JT) (aNJX x aNJX)
	Mtx::Multiply(aNJX,aNU,aNJX,aJ,m_NU_NJX,rIeff);
	//printf("\nSDFast.formJacobianInverse: J_IInv_JT...\n");
	//Mtx::Print(aNJX,aNJX,J_IInv_JT,3);
	int status = Mtx::Invert(aNJX,rIeff,rIeff);
	//printf("\nSDFast.formJacobianInverse: Inverse(J_IInv_JT)...\n");
	//Mtx::Print(aNJX,aNJX,J_IInv_JT,3);

	// DELETE WORK MATRIX
	if(m_NU_NJX!=NULL) delete[] m_NU_NJX;

	// PRINT STATUS
	if(status==-2) {
		printf("Model.ComputeEffectiveMassMatrix: WARN- ");
		printf("singularity encountered.\n");
	}
	return(0);
}

//_____________________________________________________________________________
/**
 * Compute the generalized inverse of a Jacobian matrix.
 *
 * rJInv has the shape rJInv[NU][NJX];
 *
 * rJInv = Inverse(aI) * aJT * Inverse( aJ * Inverse(aI) * JT)
 *
 * @param aNJX Number of rows in the Jacobian matrix
 * @param aNU Number of generalized speeds or degrees of freedom.
 * @param aJ Pointer to the system Jacobian (NJX by NU).
 * @param aI Pointer to the system mass matrix (NU by NU).
 * @param aJInv A pointer to the generalized inverse (NU by NJX).
 * 
 * @return 0 when successful, -1 if an error is encountered, -2 when the
 * Jacobian has a singularity.
 */
int Model::
ComputeJacobianInverse(int aNJX,int aNU,const double *aJ,const double *aI,
	double *rJInv)
{
	if(aNJX<=0) return(-1);
	if(aNU<=0) return(-1);
	if(aI==NULL) return(-1);
	if(aJ==NULL) return(-1);
	if(rJInv==NULL) return(-1);

	// INVERSE OF THE MASS MATRIX
	double *IInv = new double[aNU*aNU];
	Mtx::Invert(aNU,aI,IInv);
	//printf("\nSDFast.formJacobianInverse: Inverse Mass Matrix...\n");
	//Mtx::Print(aNU,aNU,IInv,3);

	// FORM JT (NUx6)
	double *JT = new double[aNU*aNJX];
	Mtx::Transpose(aNJX,aNU,aJ,JT);
	//printf("\nSDFast.formJacobianInverse: JT...\n");
	//Mtx::Print(aNU,aNJX,JT,3);

	// COMPUTE IInv * JT (aNU x aNJX)
	double *IInv_JT = new double[aNU*aNJX];
	Mtx::Multiply(aNU,aNU,aNJX,IInv,JT,IInv_JT);
	//printf("\nSDFast.formJacobianInverse: IInv_JT...\n");
	//Mtx::Print(aNU,aNJX,IInv_JT,3);

	// COMPUTE Inverse(J * IInv * JT) (aNJX x aNJX)
	double *J_IInv_JT = new double[aNJX*aNJX];
	Mtx::Multiply(aNJX,aNU,aNJX,aJ,IInv_JT,J_IInv_JT);
	//printf("\nSDFast.formJacobianInverse: J_IInv_JT...\n");
	//Mtx::Print(aNJX,aNJX,J_IInv_JT,3);
	int status = Mtx::Invert(aNJX,J_IInv_JT,J_IInv_JT);
	//printf("\nSDFast.formJacobianInverse: Inverse(J_IInv_JT)...\n");
	//Mtx::Print(aNJX,aNJX,J_IInv_JT,3);

	// COMPUTE GENERALIZED INVERSE (aNU x aNJX)
	Mtx::Multiply(aNU,aNJX,aNJX,JT,J_IInv_JT,rJInv);
	Mtx::Multiply(aNU,aNU,aNJX,IInv,rJInv,rJInv);

	// CLEANUP
	if(IInv!=NULL) delete[] IInv;
	if(JT!=NULL) delete[] JT;
	if(IInv_JT!=NULL) delete[] IInv_JT;
	if(J_IInv_JT!=NULL) delete[] J_IInv_JT;

	// PRINT STATUS
	if(status==-2) {
		printf("Model.formJcobianInverse: WARN- singularity encountered.\n");
	}
	//printf("\nSDFast.formJacobianInverse:  Generalized Inverse...\n");
	//Mtx::Print(aNU,aNJX,rJInv,3);

	return(status);
}


//=============================================================================
// ACTUATION
//=============================================================================
//-----------------------------------------------------------------------------
// COMPUTE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute all quantities associated with actuating a model.
 */
void Model::
computeActuation()
{
	printf("Model.computeActuation: WARN- should be overridden!");
}

//-----------------------------------------------------------------------------
// APPLY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply an actuator force.
 */
void Model::
applyActuatorForce(int aID)
{
	printf("Model.applyActuatorForce: WARN- should be overridden!");
}
//_____________________________________________________________________________
/**
 * Apply actuator forces.
 */
void Model::
applyActuatorForces()
{
	printf("Model.applyActuatorForces: WARN- should be overridden!");
}

//-----------------------------------------------------------------------------
// SCALARS- FORCE, SPEED, POWER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the magnitude of force experted by a particular actuator.
 *
 * @param aID Index of the actuator- should be greater than or
 * equal to zero and less than the value returned by getNA().
 * @param aForce Magnitude of the actuator force.
 */
void Model::
setActuatorForce(int aID,double aForce)
{
	printf("Model.setActuatorForce: WARN- should be overridden!\n");
}
//_____________________________________________________________________________
/**
 * Get the magnitude of force experted by a particular actuator.
 *
 * For the returned information to be valid, computeActuation() must be called
 * prior to calling this method.
 *
 * @param aID Index of the actuator- should be greater than or
 * equal to zero and less than the value returned by getNA().
 * @return Magnitude of actuator force.
 */
double Model::
getActuatorForce(int aID) const
{
	printf("Model.getActuatorForce: WARN- should be overridden!\n");
	return(0.0);
}
//_____________________________________________________________________________
/**
 * Get the stress of the actuator (force / area).
 *
 * For the returned information to be valid, computeActuation() must be called
 * prior to calling this method.
 *
 * @param aID Index of the actuator- should be greater than or
 * equal to zero and less than the value returned by getNA().
 * @return Actuator stress.
 */
double Model::
getActuatorStress(int aID) const
{
	printf("Model.getActuatorStress: WARN- should be overridden!\n");
	return(0.0);
}
//_____________________________________________________________________________
/**
 * Get the speed at which a particular actuator force is applied.
 *
 * For the returned information to be valid, computeActuation() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @return Speed at which actuator force is applied.
 */
double Model::
getActuatorSpeed(int aID) const
{
	printf("Model.getActuatorSpeed: WARN- should be overridden!\n");
	return(0.0);
}
//_____________________________________________________________________________
/**
 * Get the power delivered or absorbed by a particular actuator.
 * A positive power means the actuator is doing work on the model;
 * negative power means that the actuator is absorbing energy from the
 * model.
 *
 * For the returned information to be valid, computeActuation() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @return Power delivered (positive) or absorbed (negative).
 */
double Model::
getActuatorPower(int aID) const
{
	printf("Model.getActuatorPower: WARN- should be overridden!\n");
	return(0.0);
}


//=============================================================================
// CONTACT
//=============================================================================
//-----------------------------------------------------------------------------
// COMPUTE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute all quantities associated with simulating contact between bodies.
 * These quantities include at least the contact bodies, contact points, and
 * contact forces.
 *
 * @see getNP()
 */
void Model::
computeContact()
{
	printf("Model.computeContact: WARN- should be overridden!");
}

//-----------------------------------------------------------------------------
// APPLY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a contacted force.
 *
 * For the correct contact forces to be applied correctly, computeContact()
 * must be called prior to calling this method.
 */
void Model::
applyContactForce(int aID)
{
	printf("Model.applyContactForce: WARN- should be overridden!");
}
//_____________________________________________________________________________
/**
 * Apply the computed contacted forces.
 *
 * For the correct contact forces to be applied correctly, computeContact()
 * must be called prior to calling this method.
 */
void Model::
applyContactForces()
{
	printf("Model.applyContactForce: WARN- should be overridden!");
}

//-----------------------------------------------------------------------------
// BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the body number of BodyA for a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @return Body ID of BodyA.
 */
int Model::
getContactBodyA(int aID) const
{
	if((aID<0)||(aID>=getNP())) return(getGroundID());
	printf("Model.getContactBodyA: WARN- should be overridden!\n");
	return(getGroundID());
}
//_____________________________________________________________________________
/**
 * Get the body number of BodyB for a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @return Body ID of BodyB.
 */
int Model::
getContactBodyB(int aID) const
{
	if((aID<0)||(aID>=getNP())) return(getGroundID());
	printf("Model.getContactBodyB: WARN- should be overridden!\n");
	return(getGroundID());
}

//-----------------------------------------------------------------------------
// POINT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the contact point on BodyA expressed in the local frame of BodyA
 * for a specified contact.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @param aPoint Contact point on BodyA expressed in the local frame of
 * BodyA.
 */
void Model::
setContactPointA(int aID,const double aPoint[3])
{
	printf("Model.setContactPointA: WARN- should be overridden!\n");
}
//_____________________________________________________________________________
/**
 * Get the contact point on BodyA expressed in the local frame of BodyA
 * for a particular contact force.
 * 
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @param rPoint Contact point on BodyA expressed in the local frame of
 * BodyA.
 */
void Model::
getContactPointA(int aID,double rPoint[3]) const
{
	printf("Model.getContactPointA: WARN- should be overridden!\n");
}
//_____________________________________________________________________________
/**
 * Set the contact point on BodyB expressed in the local frame of BodyB
 * for a specified contact.
 * 
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @param aPoint Contact point on BodyB expressed in the local frame of
 * BodyB.
 */
void Model::
setContactPointB(int aID,const double aPoint[3])
{
	printf("Model.setContactPointB: WARN- should be overridden!\n");
}
//_____________________________________________________________________________
/**
 * Get the contact point on BodyB expressed in the local frame of BodyB
 * for a particular contact force.
 * 
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @param rPoint Contact point on BodyB expressed in the local frame of
 * BodyB.
 */
void Model::
getContactPointB(int aID,double rPoint[3]) const
{
	printf("Model.getContactPointB: WARN- should be overridden!\n");
}

//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the contact force acting on BodyB expressed in the local frame of
 * BodyA for a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @param rF Total contact force acting on BodyB expressed in the local frame
 * of BodyA.
 */
void Model::
getContactForce(int aID,double rF[3]) const
{
	printf("Model.getContactForceA: WARN- should be overridden!\n");
}

//_____________________________________________________________________________
/**
 * Get the normal contact force acting on BodyB expressed in the local frame
 * of BodyA for a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @param rFP Elastic normal contact force NOT corrected to enforce friction
 * constraints.
 * @param rFV Viscous normal contact force NOT corrected to enforce friction
 * constraints.
 * @param rF Total normal contact force acting on BodyB expressed in the local
 * frame of BodyA.  This is the actual normal force applied to BodyB.
 */
void Model::
getContactNormalForce(int aID,double rFP[3],double rFV[3],double rF[3]) const
{
	printf("Model.getContactForceA: WARN- should be overridden!\n");
}
//_____________________________________________________________________________
/**
 * Get the tangential contact force acting on BodyB expressed in the local
 * frame of BodyA for a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @param rFP Elastic tangential contact force NOT corrected to enforce
 * friction constraints.
 * @param rFV Viscous tangential contact force NOT corrected to enforce
 * friction constraints.
 * @param rF Total tangential contact force acting on BodyB expressed in
 * the local frame of BodyA.  This is the actual tangential force applied to
 * BodyB.
 */
void Model::
getContactTangentForce(int aID,double rFP[3],double rFV[3],double rF[3])
	const
{
	printf("Model.getContactForceA: WARN- should be overridden!\n");
}

//-----------------------------------------------------------------------------
// STIFFNESS AND VISCOSITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the instantaneous stiffness (or change in force on BodyB due
 * to a change in position of the BodyB contact point) of a specified
 * contact force.
 *
 * aDF = (stiffness) * aDX;   stiffness == df/dx  (partial of f wrt x)
 *
 * The displacement (aDX) is assumed to be expressed in the local frame of
 * BodyA and specifies a change in position of the BodyB contact point.
 * If the aDX is a unit vector, the returned value is the stiffness of the
 * contact force in the direction aDX.  However, if aDX is not a unit vector,
 * the returned value is the change in force applied to BodyB that would
 * occur for a displacment of aDX. 
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @param aDX Displacement of the BodyB contact point expressed in the
 * local frame of BodyA.
 * @param rDF Change in force applied to BodyB for the given displacement
 * (or stiffness if |aDX| = 1.0) expressed in the local frame of BodyA.
 */
void Model::
getContactStiffness(int aID,const double aDX[3],double rDF[3]) const
{
	printf("Model.getContactStiffnessA: WARN- should be overridden!\n");
}
//_____________________________________________________________________________
/**
 * Get the instantaneous viscosity (or change in force on BodyB due
 * to a change in velocity of the BodyB contact point) of a specified
 * contact force.
 *
 * aDF = (viscosity) * aDV;   viscosity == df/dv  (partial of f wrt v)
 *
 * The velocity change (aDV) is assumed to be expressed in the local frame of
 * BodyA and specifies a change in velocity of the BodyB contact point.
 * If the aDV is a unit vector, the returned value is the visocity of the
 * contact force in the direction aDV.  However, if aDV is not a unit vector,
 * the returned value is the change in force applied to BodyB that would
 * occur for a change in velocity of aDV. 
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @param aDV Change in velocity of the BodyB contact point expressed in the
 * local frame of BodyA.
 * @param rDF Change in force applied to BodyB for the given velocity change
 * (or viscosity if |aDX| = 1.0) expressed in the local frame of BodyA.
 */
void Model::
getContactViscosity(int aID,const double aDV[3],double rDF[3]) const
{
	printf("Model.getContactViscosityA: WARN- should be overridden!\n");
}

//-----------------------------------------------------------------------------
// FRICTION CORRECTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the correction that was made to the contact force to enforce friction
 * constraints.  This correction is returned as the change in contact force
 * applied to BodyB expressed in the local frame of BodyA.
 *
 * Note that this correction is NOT intended to express changes in contact
 * force due to damping terms, but rather changes due to the enforcement of
 * constraints on the tangential component of the contact force, such as
 * enforcing a coefficient of friction (e.g., fx <= mu*fy).
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @param rDFFric Change in contact force due to enforcing friction
 * constraints.  This correction is returned as the change in contact force
 * applied to BodyA expressed in the local frame of BodyA.
 */
void Model::
getContactFrictionCorrection(int aID,double rDFFric[3]) const
{
printf("Model.getContactFrictionCorrectionA: WARN- should be overridden!\n");
}


//-----------------------------------------------------------------------------
// SCALARS- FORCE, SPEED, POWER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the magnitude of force experted by a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @return Magnitude of contact force.
 */
double Model::
getContactForce(int aID) const
{
	printf("Model.getContactForce: WARN- should be overridden!\n");
	return(0.0);
}
//_____________________________________________________________________________
/**
 * Get the speed at which a particular contact force is applied.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @return Speed at which contact force is applied.
 */
double Model::
getContactSpeed(int aID) const
{
	printf("Model.getContactSpeed: WARN- should be overridden!\n");
	return(0.0);
}
//_____________________________________________________________________________
/**
 * Get the power delivered or absorbed by a particular contact force.
 * A positive power means the contact force is doing work on the model;
 * negative power means that the contact force is absorbing energy from the
 * model.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNP().
 * @return Power delivered (positive) or absorbed (negative).
 */
double Model::
getContactPower(int aID) const
{
	printf("Model.getContactPower: WARN- should be overridden!\n");
	return(0.0);
}



//=============================================================================
// PERFORMANCE AND CONSTRAINTS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the value of the performance criterion.
 * This virutal method simply sets the performance to 1.0.
 * Derived classes should override this method.
 * The parameter cd is provided so that the caller may send in client data
 * if desired.
 */
int Model::
computePerformance(double t,double *x,double *y,double *p,void *cd)
{
	*p = 1.0;
	return(0);
}
//_____________________________________________________________________________
/**
 * Compute the value of the constraint indexed by ic.
 * This virutal method simply sets the value of the constraint to 0.0.
 * Derived classes should override this method.
 * The parameter cd is provided so that the caller may send in client data
 * if desired.
 */
int Model::
computeConstraint(double t,double *x,double *y,int ic,double *c,void *cd)
{
	*c = 1.0;
	return(0);
}
//_____________________________________________________________________________
/**
 * Promote a set of controls to state variables.  This utility routine is
 * normally useful when solving static optimization problems with a dynamic
 * model.  In the base class, this method does nothing.
 *
 * @param aX Controls.
 */
void Model::
promoteControlsToStates(const double aX[],double aDT)
{
	printf("Model.promoteControlsToStates: ERR- should be overriden.\n");
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the set of analyses.
 *
 * @return Analysis set of this model.
 */
AnalysisSet* Model::
getAnalysisSet()
{
	return(_analysisSet);
}

//_____________________________________________________________________________
/**
 * Add an analysis to the end of the analysis list.
 *
 * @param aAnalysis Pointer to the analysis to add.
 */
void Model::
addAnalysis(Analysis *aAnalysis)
{
	// CHECK FOR NULL
	if(aAnalysis==NULL) {
		printf("Model.addAnalysis:  ERROR- Attempt to add NULL analysis.\n");
	}

	// ADD
	aAnalysis->setModel(this);
	_analysisSet->append(aAnalysis);
}

int  Model::getNumAnalyses() const
{
	return (_analysisSet ? _analysisSet->getSize(): 0);
}

Analysis& Model::getAnalysis(const int index) const
{
	return (*_analysisSet->get(index));
}

//=============================================================================
// INTEGRATION CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the set of integration callbacks.
 *
 * @retun Integration callback set of this model.
 */
IntegCallbackSet* Model::
getIntegCallbackSet()
{
	return(_integCallbackSet);
}

IntegCallback& Model::getIntegCallback(const int index) const
{
	return (*_integCallbackSet->getIntegCallback(index));
}


//_____________________________________________________________________________
/**
 * Add an integration callback to the model
 *
 * @param aCallback Pointer to the integration callback to add.
 */
void Model::
addIntegCallback(IntegCallback *aCallback)
{
	// CHECK FOR NULL
	if(aCallback==NULL) {
		printf("Model.addDerivCallback:  ERROR- NULL callback.\n");
	}

	// ADD
	aCallback->setModel(this);
	_integCallbackSet->append(aCallback);
}


//=============================================================================
// DERIVATIVE CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the set of derivative callbacks.
 *
 * @return Derivative callback set of this model.
 */
DerivCallbackSet* Model::
getDerivCallbackSet()
{
	return(_derivCallbackSet);
}
//_____________________________________________________________________________
/**
 * Add a derivative callback to the model
 *
 * @param aCallback Pointer to the derivative callback to add.
 */
void Model::
addDerivCallback(DerivCallback *aCallback)
{
	// CHECK FOR NULL
	if(aCallback==NULL) {
		printf("Model.addDerivCallback:  ERROR- NULL callback.\n");
	}

	// ADD
	aCallback->setModel(this);
	_derivCallbackSet->append(aCallback);
}

//=============================================================================
// FOR RRA - ADDED BY CHAND, 2/3/2006
//=============================================================================
//_____________________________________________________________________________
/**
 * Set whether to include pipeline actuators.
 *
 * @param include Boolean variable which tells whether to include pipeline actuators.
 */
void Model::
setIncludePipelineActuators(bool include)
{ // Do nothing
}

//_____________________________________________________________________________
/**
 * Compute the constrained coordinates for a simulation.
 *
 * @param y Array of states containing all coordinates.
 */
void Model::
computeConstrainedCoordinates(double* y)
{ // Do nothing
}
//_____________________________________________________________________________
/**
 * From a potentially partial specification of the generalized coordinates,
 * form a complete storage of the generalized coordinates (q's) and
 * speeds (u's).
 *
 * @param aQIn Storage containing the q's or a subset of the q's.
 * @param rQComplete Storage containing all the q's.  If q's were not
 * in aQIn, the values are set to 0.0.  When a q is constrained, its value
 * is altered to be consistent with the constraint.
 * @param rUComplete Storage containing all the u's.  The generalized speeds
 * are obtained by spline fitting the q's and differentiating the splines.
 * When a u is constrained, its value is altered to be consisten with the
 * constraint.
 */
void Model::
formCompleteStorages(const OpenSim::Storage &aQIn,
	OpenSim::Storage *&rQComplete,OpenSim::Storage *&rUComplete)
{
}


//=============================================================================
// PRINT
//=============================================================================
//_____________________________________________________________________________
/**
 * Print some basic information about the model.
 *
 * @param aOStream Output stream.
 */
void Model::
printBasicInfo(std::ostream &aOStream) const
{
	aOStream<<"             MODEL: "<<getName()<<"\n";
	aOStream<<"            bodies: "<<getNB()<<"\n";
	aOStream<<"    coordinates(q): "<<getNQ()<<"\n";
	aOStream<<"         speeds(u): "<<getNU()<<"\n";
	aOStream<<"         actuators: "<<getNA()<<"\n";
	aOStream<<"          contacts: "<<getNP()<<"\n";
	aOStream<<"     pseudo-states: "<<getNYP()<<"\n";
	aOStream<<"            states: "<<getNY()<<"\n";
}
//_____________________________________________________________________________
/**
 * Print detailed information about the model.
 *
 * @param aOStream Output stream.
 */
void Model::
printDetailedInfo(std::ostream &aOStream) const
{
	int i,n;

	aOStream<<"MODEL: "<<getName()<<"\n";

	n = getNB();
	aOStream<<"\nBODIES ("<<n<<")\n";
	for(i=0;i<n;i++) aOStream<<"body["<<i<<"] = "<<getBodyName(i)<<"\n";

	n = getNQ();
	aOStream<<"\nGENERALIZED COORDINATES ("<<n<<")\n";
	for(i=0;i<n;i++) aOStream<<"q["<<i<<"] = "<<getCoordinateName(i)<<"\n";

	n = getNU();
	aOStream<<"\nGENERALIZED SPEEDS ("<<n<<")\n";
	for(i=0;i<n;i++) aOStream<<"u["<<i<<"] = "<<getSpeedName(i)<<"\n";

	n = getNA();
	aOStream<<"\nACTUATORS ("<<n<<")\n";
	for(i=0;i<n;i++) aOStream<<"actuator["<<i<<"] = "<<getActuatorName(i)<<"\n";

	n = getNP();
	aOStream<<"\nCONTACTS ("<<n<<")\n";

	n = getNYP();
	aOStream<<"\nPSEUDO-STATES ("<<n<<")\n";
	for(i=0;i<n;i++) aOStream<<"yp["<<i<<"] = "<<getPseudoStateName(i)<<"\n";

	n = getNY();
	aOStream<<"\nSTATES ("<<n<<")\n";
	for(i=0;i<n;i++) aOStream<<"y["<<i<<"] = "<<getStateName(i)<<"\n";
}


