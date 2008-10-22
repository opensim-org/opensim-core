// SimmFileWriter.cpp
// Author: Peter Loan
/* Copyright (c)  2005, Stanford University and Peter Loan.
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
#include <sstream>
#include "SimmFileWriter.h"
#include "Model.h"
#include "AbstractMuscle.h"
#include "AbstractBody.h"
#include "AbstractCoordinate.h"
#include "ActuatorSet.h"
#include "MusclePoint.h"
#include "MusclePointSet.h"
#include "MuscleViaPoint.h"
#include <OpenSim/Actuators/Schutte1993Muscle.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Common/NatCubicSpline.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmFileWriter::SimmFileWriter() :
	_model(NULL)
{
}

//_____________________________________________________________________________
/**
 * Constructor taking a model pointer
 */
SimmFileWriter::SimmFileWriter(Model *aModel) :
	_model(NULL)
{
	_model = aModel;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmFileWriter::~SimmFileWriter()
{
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Write a SIMM joint file. Dynamics Engines can be very different, so the
 * responsibility for exporting a SIMM joint file has been pushed down to
 * the engine class.
 *
 * @param aFileName name of joint file to write.
 * @return Whether or not file writing was successful.
 */
bool SimmFileWriter::writeJointFile(const string& aFileName) const
{
	if (!_model)
		return false;

   return _model->getDynamicsEngine().writeSIMMJointFile(aFileName);
}

//_____________________________________________________________________________
/**
 * Write a SIMM muscle file.
 *
 * @param aFileName name of muscle file to write.
 * @return Whether or not file writing was successful.
 */
bool SimmFileWriter::writeMuscleFile(const string& aFileName) const
{
	if (!_model)
		return false;

   ofstream out;

   out.open(aFileName.c_str());
   out.setf(ios::fixed);
   out.precision(12);

   if (!out.good())
   {
      cout << "Unable to open output muscle file " << aFileName << endl;
      return false;
   }

   out << "/**********************************************************/\n";
   out << "/*            Muscle file created by OpenSim              */\n";
   if (_model->getInputFileName() != "")
      out << "/* name of original model file: " << _model->getInputFileName() << " */\n";
   out << "/**********************************************************/\n";
   out << "\n";

	/* TODO: hack to support dynamic parameters in all currently defined muscle models. */
	out << "begindynamicparameters" << endl;
	out << "timescale" << endl;
	out << "mass" << endl;
	out << "damping" << endl;
	out << "activation1" << endl;
	out << "activation2" << endl;
	out << "activation_time_constant" << endl;
	out << "deactivation_time_constant" << endl;
	out << "Vmax" << endl;
	out << "Vmax0" << endl;
	out << "Af" << endl;
	out << "Flen" << endl;
	out << "FmaxTendonStrain" << endl;
	out << "FmaxMuscleStrain" << endl;
	out << "KshapeActive" << endl;
	out << "KshapePassive" << endl;
	out << "muscle_density" << endl;
	out << "max_isometric_stress" << endl;
	out << "enddynamicparameters" << endl << endl;

	/* The default muscle must be defined or the Pipeline code crashes. */
	out << "beginmuscle defaultmuscle" << endl;
	out << "endmuscle" << endl << endl;

	const ActuatorSet* actuatorSet = _model->getActuatorSet();

	int i;
	for (i = 0; i < actuatorSet->getSize(); i++)
	{
		AbstractMuscle* muscle = dynamic_cast<AbstractMuscle*>(actuatorSet->get(i));
		if (muscle)
			writeMuscle(*muscle, *actuatorSet, out);
	}

   out.close();
	cout << "Wrote SIMM muscle file " << aFileName << " from model " << _model->getName() << endl;

	return true;
}

bool SimmFileWriter::writeMuscle(AbstractMuscle& aMuscle, const ActuatorSet& aActuatorSet, ofstream& aStream) const
{
	aStream << "beginmuscle " << aMuscle.getName() << endl;

	const MusclePointSet& pts = aMuscle.getAttachmentSet();

	aStream << "beginpoints" << endl;
	for (int i = 0; i < pts.getSize(); i++)
	{
		Vec3& attachment = pts.get(i)->getAttachment();
		MuscleViaPoint* mvp = dynamic_cast<MuscleViaPoint*>(pts.get(i));
		aStream << attachment[0] << " " << attachment[1] << " " << attachment[2] << " segment " << pts.get(i)->getBody()->getName();
		if (mvp)
		{
			Array<double>& range = mvp->getRange();
			const AbstractCoordinate* coord = mvp->getCoordinate();
			if (coord->getMotionType() == AbstractTransformAxis::Rotational)
				aStream << " ranges 1 " << coord->getName() << " (" << range[0] * SimTK_RADIAN_TO_DEGREE << ", " << range[1] * SimTK_RADIAN_TO_DEGREE << ")" << endl;
			else
				aStream << " ranges 1 " << coord->getName() << " (" << range[0] << ", " << range[1] << ")" << endl;
		}
		else
		{
			aStream << endl;
		}
	}
	aStream << "endpoints" << endl;

	Array<std::string> groupNames;
	aActuatorSet.getGroupNamesContaining(aMuscle.getName(),groupNames);
	if(groupNames.getSize()) {
		aStream << "begingroups" << endl;
		for(int i=0; i<groupNames.getSize(); i++)
			aStream << " " << groupNames[i];
		aStream << endl << "endgroups" << endl;
	}

	if (dynamic_cast<Schutte1993Muscle*>(&aMuscle))
	{
		Schutte1993Muscle *szh = dynamic_cast<Schutte1993Muscle*>(&aMuscle);

		aStream << "max_force " << szh->getMaxIsometricForce() << endl;
		aStream << "optimal_fiber_length " << szh->getOptimalFiberLength() << endl;
		aStream << "tendon_slack_length " << szh->getTendonSlackLength() << endl;
		aStream << "pennation_angle " << szh->getPennationAngleAtOptimalFiberLength() * SimTK_RADIAN_TO_DEGREE << endl;
		aStream << "max_contraction_velocity " << szh->getMaxContractionVelocity() << endl;
		aStream << "timescale " << szh->getTimeScale() << endl;
		if (!szh->getMuscleModelIndexUseDefault())
			aStream << "muscle_model " << szh->getMuscleModelIndex() << endl;

		if (szh->getActiveForceLengthCurve())
		{
			NatCubicSpline* ncs;
			if ((ncs = dynamic_cast<NatCubicSpline*>(szh->getActiveForceLengthCurve())))
			{
				aStream << "beginactiveforcelengthcurve" << endl;
				for (int i = 0; i < ncs->getNumberOfPoints(); i++)
					aStream << "(" << ncs->getX()[i] << ", " << ncs->getY()[i] << ")" << endl;
				aStream << "endactiveforcelengthcurve" << endl;
			}
		}

		if (szh->getPassiveForceLengthCurve())
		{
			NatCubicSpline* ncs;
			if ((ncs = dynamic_cast<NatCubicSpline*>(szh->getPassiveForceLengthCurve())))
			{
				aStream << "beginpassiveforcelengthcurve" << endl;
				for (int i = 0; i < ncs->getNumberOfPoints(); i++)
					aStream << "(" << ncs->getX()[i] << ", " << ncs->getY()[i] << ")" << endl;
				aStream << "endpassiveforcelengthcurve" << endl;
			}
		}

		if (szh->getTendonForceLengthCurve())
		{
			NatCubicSpline* ncs;
			if ((ncs = dynamic_cast<NatCubicSpline*>(szh->getTendonForceLengthCurve())))
			{
				aStream << "begintendonforcelengthcurve" << endl;
				for (int i = 0; i < ncs->getNumberOfPoints(); i++)
					aStream << "(" << ncs->getX()[i] << ", " << ncs->getY()[i] << ")" << endl;
				aStream << "endtendonforcelengthcurve" << endl;
			}
		}

		if (szh->getForceVelocityCurve())
		{
			NatCubicSpline* ncs;
			if ((ncs = dynamic_cast<NatCubicSpline*>(szh->getForceVelocityCurve())))
			{
				aStream << "beginforcevelocitycurve" << endl;
				for (int i = 0; i < ncs->getNumberOfPoints(); i++)
					aStream << "(" << ncs->getX()[i] << ", " << ncs->getY()[i] << ")" << endl;
				aStream << "endforcevelocitycurve" << endl;
			}
		}
	}
	else if (dynamic_cast<Thelen2003Muscle*>(&aMuscle))
	{
		Thelen2003Muscle *sdm = dynamic_cast<Thelen2003Muscle*>(&aMuscle);

		aStream << "max_force " << sdm->getMaxIsometricForce() << endl;
		aStream << "optimal_fiber_length " << sdm->getOptimalFiberLength() << endl;
		aStream << "tendon_slack_length " << sdm->getTendonSlackLength() << endl;
		aStream << "pennation_angle " << sdm->getPennationAngleAtOptimalFiberLength() * SimTK_RADIAN_TO_DEGREE << endl;
		aStream << "activation_time_constant " << sdm->getActivationTimeConstant() << endl;
		aStream << "deactivation_time_constant " << sdm->getDeactivationTimeConstant() << endl;
		aStream << "Vmax " << sdm->getVmax() << endl;
		aStream << "Vmax0 " << sdm->getVmax0() << endl;
		aStream << "FmaxTendonStrain " << sdm->getFmaxTendonStrain() << endl;
		aStream << "FmaxMuscleStrain " << sdm->getFmaxMuscleStrain() << endl;
		aStream << "KshapeActive " << sdm->getKshapeActive() << endl;
		aStream << "KshapePassive " << sdm->getKshapePassive() << endl;
		aStream << "damping " << sdm->getDamping() << endl;
		aStream << "Af " << sdm->getAf() << endl;
		aStream << "Flen " << sdm->getFlen() << endl;
		if (!sdm->getMuscleModelIndexUseDefault())
			aStream << "muscle_model " << sdm->getMuscleModelIndex() << endl;
	}

	aStream << "endmuscle" << endl << endl;
	return true;
}
