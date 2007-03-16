// SimmFileWriter.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include <sstream>
#include "SimmFileWriter.h"
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/SIMM/AbstractSimmMuscle.h>
#include <OpenSim/Simulation/SIMM/BodySet.h>
#include <OpenSim/Simulation/SIMM/JointSet.h>
#include <OpenSim/Simulation/SIMM/CoordinateSet.h>
#include <OpenSim/Simulation/SIMM/ActuatorSet.h>
#include <OpenSim/Simulation/SIMM/MarkerSet.h>
#include <OpenSim/Simulation/SIMM/DofSet.h>
#include <OpenSim/Simulation/SIMM/SimmMusclePoint.h>
#include <OpenSim/Simulation/SIMM/SimmMusclePointSet.h>
#include <OpenSim/Simulation/SIMM/SimmMuscleViaPoint.h>
#include <OpenSim/Simulation/SIMM/SimmZajacHill.h>
#include <OpenSim/Simulation/SIMM/SimmDarrylMuscle.h>
#include <OpenSim/Simulation/SIMM/SimmCoordinate.h>
#include <OpenSim/Simulation/SIMM/SimmRotationDof.h>
#include <OpenSim/Simulation/SIMM/SimmTranslationDof.h>
#include <OpenSim/Simulation/SIMM/SimmUnits.h>
#include <OpenSim/Simulation/SIMM/SimmMacros.h>
#include <OpenSim/Simulation/SIMM/PolyObject.h>
#include <OpenSim/Simulation/SIMM/AbstractWrapObject.h>
#include <OpenSim/Simulation/SIMM/WrapObjectSet.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <OpenSim/Tools/NatCubicSpline.h>
#include <OpenSim/Tools/rdMath.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

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
SimmFileWriter::SimmFileWriter(AbstractModel *aModel) :
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
 * Make a SIMM-compatible text label for the gravity vector.
 *
 * @param aGravity the gravity vector
 * @return Reference to the text label
 */
const string& SimmFileWriter::getGravityLabel(double aGravity[3]) const
{
	static string gravityLabels[] = {"-X","+X","-Y","+Y","-Z","+Z",""};

	if (aGravity[0] <= -9.8)
		return gravityLabels[0];
	if (aGravity[0] >= 9.8)
		return gravityLabels[1];
	if (aGravity[1] <= -9.8)
		return gravityLabels[2];
	if (aGravity[1] >= 9.8)
		return gravityLabels[3];
	if (aGravity[2] <= -9.8)
		return gravityLabels[4];
	if (aGravity[2] >= 9.8)
		return gravityLabels[5];

	return gravityLabels[6];
}

//_____________________________________________________________________________
/**
 * Write a SIMM joint file.
 *
 * @param aFileName name of joint file to write.
 * @return Whether or not file writing was successful.
 */
bool SimmFileWriter::writeJointFile(const string& aFileName) const
{
	if (!_model)
		return false;

	int i;
   ofstream out;
	int functionIndex = 1;

   out.open(aFileName.c_str());
   out.setf(ios::fixed);
   out.precision(6);

   if (!out.good())
   {
      cout << "Unable to open output file " << aFileName << endl;
      return false;
   }

   out << "/**********************************************************/\n";
   out << "/*            Joint file created by OpenSim               */\n";
   if (_model->getInputFileName() != "")
      out << "/* name of original model file: " << _model->getInputFileName() << " */\n";
   out << "/**********************************************************/\n";
   out << "\nname " << _model->getName() << endl;
   out << "\n";
#if 0
   if (!mBonePath.empty())
      out << "bone_path " << mBonePath << endl;
   if (!mMuscleFilename.empty())
      out << "\nmuscle_file " << mMuscleFilename << endl;
   if (!mMotionFilename.empty())
      out << "motion_file " << mMotionFilename << endl;
#endif

	if (_model->getLengthUnits().getType() != SimmUnits::simmUnknownUnits)
		out << "length_units " << _model->getLengthUnits().getLabel() << endl;
	if (_model->getForceUnits().getType() != SimmUnits::simmUnknownUnits)
		out << "force_units " << _model->getForceUnits().getLabel() << endl;

#if 0
   if (mMarkerRadius != 0.01)
      out << "marker_radius " << mMarkerRadius << endl;
   if (mMVGear != 0.08)
      out << "MV_gear " << mMVGear << endl;
   if (mMarkerVisibility)
      out << "marker_visibility on\n";
   else
      out << "marker_visibility off\n";
   smOptions options;
   this->getOptions(options);
   out << "solver_accuracy " << options.accuracy << endl;
   out << "solver_method " << getSolverMethodString(options.method) << endl;
   if (options.orientBodyToFrame == smYes)
      out << "solver_orient_body yes" << endl;
   else
      out << "solver_orient_body no" << endl;
   if (options.jointLimitsOn == smYes)
      out << "solver_joint_limits yes" << endl;
   else
      out << "solver_joint_limits no" << endl;
   out << "solver_max_iterations " << options.maxIterations << endl;
#endif

	double gravity[3];
	_model->getGravity(gravity);
	const string& gravityLabel = getGravityLabel(gravity);

	out << "gravity " << gravityLabel << endl;

   out << "\n/****************************************************/\n";
   out << "/*                     SEGMENTS                     */\n";
   out << "/****************************************************/\n";
	BodySet* bodySet = _model->getDynamicsEngine().getBodySet();
	for (i = 0; i < bodySet->getSize(); i++)
		writeBody(*bodySet->get(i), _model->getDynamicsEngine().getMarkerSet(), out);

   out << "\n/****************************************************/\n";
   out << "/*                      JOINTS                      */\n";
   out << "/****************************************************/\n";
	JointSet* jointSet = _model->getDynamicsEngine().getJointSet();
	for (i = 0; i < jointSet->getSize(); i++)
		writeJoint(*jointSet->get(i), functionIndex, out);

   out << "\n/****************************************************/\n";
   out << "/*                     GENCOORDS                    */\n";
   out << "/****************************************************/\n";
	CoordinateSet* coordinateSet = _model->getDynamicsEngine().getCoordinateSet();
	for (i = 0; i < coordinateSet->getSize(); i++)
		writeCoordinate(*coordinateSet->get(i), functionIndex, out);

   out << "\n/****************************************************/\n";
   out << "/*                    WRAP OBJECTS                  */\n";
   out << "/****************************************************/\n";
	for (i = 0; i < bodySet->getSize(); i++)
		writeWrapObjects(*bodySet->get(i), out);

#if 0
   out << "\n/****************************************************/\n";
   out << "/*                     FUNCTIONS                    */\n";
   out << "/****************************************************/\n";
   for (ConsFuncListIterator cf = mConstraintFunction.begin(); cf != mConstraintFunction.end(); cf++)
      (*cf)->print(out);

   out << "\n/****************************************************/\n";
   out << "/*                CONSTRAINT OBJECTS                */\n";
   out << "/****************************************************/\n";
   for (ConstraintObjectListIterator co = mConstraintObject.begin(); co != mConstraintObject.end(); co++)
      (*co)->print(out);
#endif
   
   out << "\n/****************************************************/\n";
   out << "/*                     MATERIALS                    */\n";
   out << "/****************************************************/\n";
   out << "beginmaterial mat1\nambient 0.3 0.3 0.9\ndiffuse 0.3 0.3 0.9\n";
   out << "specular 1.0 1.0 1.0\nendmaterial\n\n";

   out << "beginmaterial mat2\nambient 0.3 0.3 0.3\ndiffuse 0.3 0.3 0.3";
   out << "\nspecular 0.3 0.3 0.3\nendmaterial\n\n";

   out << "beginmaterial my_bone\nambient 0.65 0.65 0.65\nspecular 0.7 0.55 0.4";
   out << "\ndiffuse 0.55 0.4 0.35\nshininess 10\nendmaterial\n\n";

   out << "beginmaterial red\nambient 0.9 0.1 0.1\nspecular 0.7 0.2 0.2";
   out << "\ndiffuse 0.2 0.2 0.2\nshininess 10\nendmaterial\n\n";

   out << "beginmaterial blue\nambient 0.1 0.1 0.9\nspecular 0.2 0.2 0.7";
   out << "\ndiffuse 0.2 0.2 0.2\nshininess 10\nendmaterial\n\n";

   out << "\n/****************************************************/\n";
   out << "/*                     WORLD OBJECTS                */\n";
   out << "/****************************************************/\n";

	out << "beginworldobject floor\n";
   if (gravityLabel == "+X" || gravityLabel == "-X")
      out << "filename floor_yz_plane.asc\n";
   else if (gravityLabel == "+Y" || gravityLabel == "-Y")
      out << "filename floor_xz_plane.asc\n";
   else if (gravityLabel == "+Z" || gravityLabel == "-Z")
      out << "filename floor_xy_plane.asc\n";
   else
      out << "filename floor1.asc\n";

   out << "origin 0.0 0.0 0.0" << endl;
   out << "\nmaterial mat2\n";
	/* The floor bone file is in meters, so scale it to fit this model. */
	double floorScale = 1.0 / _model->getLengthUnits().convertTo(SimmUnits::simmMeters);
	out << "scale " << floorScale << " " << floorScale * 2.0 << " " << floorScale * 4.0 << endl;
   out << " endworldobject\n\n";

   out << "\n/****************************************************/\n";
   out << "/*                    MOTION OBJECTS                */\n";
   out << "/****************************************************/\n";

	/* The default ball object in SIMM is in meters, so scale it to fit this model. */
	out << "beginmotionobject ball\n";
	double scale = 0.25 / _model->getLengthUnits().convertTo(SimmUnits::simmMeters);
	out << "material blue" << endl;
	out << "scale " << scale << " " << scale << " " << scale << endl;
   out << "endmotionobject\n\n";

   out.close();
	cout << "Wrote SIMM joint file " << aFileName << " from model " << _model->getName() << endl;

	return true;
}

//_____________________________________________________________________________
/**
 * Write a body to a SIMM joint file.
 *
 * @param aBody reference to the body to write.
 * @param aStream the stream (file) to write to.
 * @return Whether or not file writing was successful.
 */
bool SimmFileWriter::writeBody(AbstractBody& aBody, const MarkerSet* aMarkerSet, ofstream& aStream) const
{
	int i;

	aStream << "beginsegment " << aBody.getName() << endl;
	aStream << "mass " << aBody.getMass() << endl;

	double massCenter[3];
	aBody.getMassCenter(massCenter);
	aStream << "masscenter " << massCenter[0] << " " << massCenter[1] << " " << massCenter[2] << endl;

	double inertia[3][3];
	aBody.getInertia(inertia);
	aStream << "inertia " << inertia[0][0] << " " << inertia[0][1] << " " << inertia[0][2] << endl;
	aStream << "        " << inertia[1][0] << " " << inertia[1][1] << " " << inertia[1][2] << endl;
	aStream << "        " << inertia[2][0] << " " << inertia[2][1] << " " << inertia[2][2] << endl;

	string fileName;
	for (i = 0; i < aBody.getDisplayer()->getNumGeometryFiles(); i++)
	{
		fileName = aBody.getDisplayer()->getGeometryFileName(i);
		int dot = fileName.find_last_of(".");
		if (dot > 0)
			fileName.erase(dot, 4);
		fileName += ".asc";
		aStream << "bone " << fileName << endl;
	}

	for (i = 0; i < aMarkerSet->getSize(); i++)
	{
		AbstractMarker* marker = aMarkerSet->get(i);

		if (marker->getBody() == &aBody)
		{
			// Write out log(_weight) + 1 as marker weight instead of _weight,
			// so that we won't have markers with radius 1000 if _weight=1000.
			// If _weight <= 1, outputWeight will be set to 1.
			//double outputWeight = (marker->getWeight() > 1.0) ? log(marker->getWeight()) + 1.0 : 1.0;
			// TODO: Got rid of weight property from markers for now...
			double outputWeight = 1;

			aStream << "marker " << marker->getName() << "\t" << marker->getOffset()[0] << " " <<
				marker->getOffset()[1] << " " << marker->getOffset()[2] << " " << outputWeight;

			if (marker->getFixed())
				aStream << " fixed" << endl;
			else
				aStream << endl;
		}
	}

	double scaleFactors[3];
	aBody.getDisplayer()->getScaleFactors(scaleFactors);

	aStream << "scale " << scaleFactors[0] << " " << scaleFactors[1] << " " << scaleFactors[2] << endl;
	aStream << "endsegment" << endl << endl;

	return true;
}

//_____________________________________________________________________________
/**
 * Write a body's wrap objects to a SIMM joint file.
 *
 * @param aBody reference to the body to write.
 * @param aStream the stream (file) to write to.
 */
void SimmFileWriter::writeWrapObjects(AbstractBody& aBody, ofstream& aStream) const
{
	int i;
	WrapObjectSet& wrapObjects = aBody.getWrapObjectSet();

	for (i = 0; i < wrapObjects.getSize(); i++) {
		AbstractWrapObject* wo = wrapObjects.get(i);
		aStream << "beginwrapobject " << wo->getName() << endl;
		aStream << "wraptype " << wo->getWrapTypeName() << endl;
		aStream << "segment " << aBody.getName() << endl;
		aStream << wo->getDimensionsString() << endl;
		if (!wo->getQuadrantNameUseDefault())
			aStream << "quadrant " << wo->getQuadrantName() << endl;
		if (!wo->getActiveUseDefault())
			aStream << "active " << (wo->getActive() ? "yes" : "no") << endl;
		aStream << "translation " << wo->getTranslation()[0] << " " <<
			wo->getTranslation()[1] << " " << wo->getTranslation()[2] << endl;
		aStream << "xyz_body_rotation " << wo->getXYZBodyRotation()[0] * rdMath::RTD <<
			" " << wo->getXYZBodyRotation()[1] * rdMath::RTD <<
			" " << wo->getXYZBodyRotation()[2] * rdMath::RTD << endl;
		aStream << "endwrapobject" << endl << endl;
	}
}

//_____________________________________________________________________________
/**
 * Write a joint to a SIMM joint file.
 *
 * @param aJoint reference to the joint to write.
 * @param aStream the stream (file) to write to.
 * @return Whether or not file writing was successful.
 */
bool SimmFileWriter::writeJoint(AbstractJoint& aJoint, int& aFunctionIndex, ofstream& aStream) const
{
	int transDofIndex = 0, rotDofIndex = 0;
	stringstream order;
	char* translationLabels[] = {"tx", "ty", "tz"};
	DofSet* dofs = aJoint.getDofSet();
	int* funcIndex = new int [dofs->getSize()];

	int i;
	for (i = 0; i < dofs->getSize(); i++)
		funcIndex[i] = -1;

	aStream << "beginjoint " << aJoint.getName() << endl;
	aStream << "segments " << aJoint.getParentBody()->getName() << " " << aJoint.getChildBody()->getName() << endl;
	for (i = 0; i < dofs->getSize(); i++)
	{
		if (dofs->get(i)->getMotionType() == AbstractDof::Translational)
		{
			SimmTranslationDof* td = dynamic_cast<SimmTranslationDof*>(dofs->get(i));
			SimmTranslationDof::AxisIndex axis = td->getAxisIndex();
			aStream << translationLabels[axis];
			if (td->getCoordinate() == NULL)
				aStream << " constant " << td->getValue() << endl;
			else
			{
				funcIndex[i] = aFunctionIndex++;
				aStream << " function f" << funcIndex[i] << "(" << td->getCoordinate()->getName() << ")" << endl;
			}
			if (transDofIndex++ == 0)
				order << " t";
		}
		else if (dofs->get(i)->getMotionType() == AbstractDof::Rotational)
		{
			SimmRotationDof* rd = dynamic_cast<SimmRotationDof*>(dofs->get(i));
			rotDofIndex++;
			aStream << "r" << rotDofIndex;
			if (rd->getCoordinate() == NULL)
				aStream << " constant " << rd->getValue() << endl;
			else
			{
				funcIndex[i] = aFunctionIndex++;
				aStream << " function f" << funcIndex[i] << "(" << rd->getCoordinate()->getName() << ")" << endl;
			}
			const double* axis = rd->getAxisPtr();
			aStream << "axis" << rotDofIndex << " " << axis[0] << " " << axis[1] << " " << axis[2] << endl;
			order << " r" << rotDofIndex;
		}
	}
	aStream << "order" << order.str() << endl;
	aStream << "endjoint" << endl << endl;

	double conversionX, conversionY;

	for (i = 0; i < dofs->getSize(); i++)
	{
		if (funcIndex[i] >= 0)
		{
			NatCubicSpline* spline;
			if ((spline = dynamic_cast<NatCubicSpline*>(dofs->get(i)->getFunction())))
			{
				if (dofs->get(i)->getMotionType() == AbstractDof::Rotational)
					conversionY = rdMath::RTD;
				else
					conversionY = 1.0;
				const AbstractCoordinate* coord = dofs->get(i)->getCoordinate();
				if (coord && (coord->getMotionType() == AbstractDof::Rotational))
					conversionX = rdMath::RTD;
				else
					conversionX = 1.0;
				aStream << "beginfunction f" << funcIndex[i] << endl;
				for (int j = 0; j < spline->getNumberOfPoints(); j++)
					aStream << "(" << spline->getX()[j] * conversionX << ", " << spline->getY()[j] * conversionY << ")" << endl;
				aStream << "endfunction" << endl << endl;
			}
		}
	}

	delete funcIndex;

	return true;
}

//_____________________________________________________________________________
/**
 * Write a coordinate to a SIMM joint file.
 *
 * @param aCoordinate reference to the coordinate to write.
 * @param aStream the stream (file) to write to.
 * @return Whether or not file writing was successful.
 */
bool SimmFileWriter::writeCoordinate(AbstractCoordinate& aCoordinate, int& aFunctionIndex, ofstream& aStream) const
{
	int RFIndex = -1, minRFIndex = -1, maxRFIndex = -1;
	double conversion;

	if (aCoordinate.getMotionType() == AbstractDof::Rotational)
		conversion = rdMath::RTD;
	else
		conversion = 1.0;

	aStream << "begingencoord " << aCoordinate.getName() << endl;
	aStream << "default_value " << aCoordinate.getDefaultValue() * conversion << endl;
	aStream << "range " << aCoordinate.getRangeMin() * conversion << " " << aCoordinate.getRangeMax() * conversion << endl;
	aStream << "tolerance " << aCoordinate.getTolerance() << endl;
	aStream << "pd_stiffness " << aCoordinate.getStiffness() << endl;

	SimmCoordinate* sc = dynamic_cast<SimmCoordinate*>(&aCoordinate);

	/* keys are for SimmCoordinates only */
	if (sc)
	{
		const Array<std::string>& keys = sc->getKeys();
		if (keys.getSize() > 0)
		{
			aStream << "keys ";
			for (int i = 0; i < MIN(2, keys.getSize()); i++)
				aStream << keys[i] << " ";
			aStream << endl;
		}
	}

	aStream << "clamped " << ((aCoordinate.getClamped()) ? ("yes") : ("no")) << endl;
	aStream << "locked " << ((aCoordinate.getLocked()) ? ("yes") : ("no")) << endl;

	/* Restraint functions are for SimmCoordinates only. */
	if (sc)
	{
		aStream << "active " << ((sc->isRestraintActive()) ? ("yes") : ("no")) << endl;

		if (sc->getRestraintFunction())
		{
			RFIndex = aFunctionIndex++;
			aStream << "restraint f" << RFIndex << endl;
		}

		if (sc->getMinRestraintFunction())
		{
			minRFIndex = aFunctionIndex++;
			aStream << "minrestraint f" << minRFIndex << endl;
		}

		if (sc->getMaxRestraintFunction())
		{
			maxRFIndex = aFunctionIndex++;
			aStream << "maxrestraint f" << maxRFIndex << endl;
		}
	}

	aStream << "endgencoord" << endl << endl;

	if (sc)
	{
		Function* func;
		NatCubicSpline* spline;

		if (RFIndex >= 0)
		{
			func = sc->getRestraintFunction();
			if ((spline = dynamic_cast<NatCubicSpline*>(func)))
			{
				aStream << "beginfunction f" << RFIndex << endl;
				for (int i = 0; i < spline->getNumberOfPoints(); i++)
					aStream << "(" << spline->getX()[i] * conversion << ", " << spline->getY()[i] << ")" << endl;
				aStream << "endfunction" << endl << endl;
			}
		}

		if (minRFIndex >= 0)
		{
			func = sc->getMinRestraintFunction();
			if ((spline = dynamic_cast<NatCubicSpline*>(func)))
			{
				aStream << "beginfunction f" << minRFIndex << endl;
				for (int i = 0; i < spline->getNumberOfPoints(); i++)
					aStream << "(" << spline->getX()[i] * conversion << ", " << spline->getY()[i] << ")" << endl;
				aStream << "endfunction" << endl << endl;
			}
		}

		if (maxRFIndex >= 0)
		{
			func = sc->getMaxRestraintFunction();
			if ((spline = dynamic_cast<NatCubicSpline*>(func)))
			{
				aStream << "beginfunction f" << maxRFIndex << endl;
				for (int i = 0; i < spline->getNumberOfPoints(); i++)
					aStream << "(" << spline->getX()[i] * conversion << ", " << spline->getY()[i] << ")" << endl;
				aStream << "endfunction" << endl << endl;
			}
		}
	}

	return true;
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
		AbstractSimmMuscle* muscle = dynamic_cast<AbstractSimmMuscle*>(actuatorSet->get(i));
		if (muscle)
			writeMuscle(*muscle, out);
	}

   out.close();
	cout << "Wrote SIMM muscle file " << aFileName << " from model " << _model->getName() << endl;

	return true;
}

bool SimmFileWriter::writeMuscle(AbstractSimmMuscle& aMuscle, ofstream& aStream) const
{
	int i;
	aStream << "beginmuscle " << aMuscle.getName() << endl;

	const SimmMusclePointSet& pts = aMuscle.getAttachmentSet();

	aStream << "beginpoints" << endl;
	for (i = 0; i < pts.getSize(); i++)
	{
		Array<double>& attachment = pts.get(i)->getAttachment();
		SimmMuscleViaPoint* mvp = dynamic_cast<SimmMuscleViaPoint*>(pts.get(i));
		aStream << attachment[0] << " " << attachment[1] << " " << attachment[2] << " segment " << pts.get(i)->getBody()->getName();
		if (mvp)
		{
			Array<double>& range = mvp->getRange();
			const AbstractCoordinate* coord = mvp->getCoordinate();
			if (coord->getMotionType() == AbstractDof::Rotational)
				aStream << " ranges 1 " << coord->getName() << " (" << range[0] * rdMath::RTD << ", " << range[1] * rdMath::RTD << ")" << endl;
			else
				aStream << " ranges 1 " << coord->getName() << " (" << range[0] << ", " << range[1] << ")" << endl;
		}
		else
		{
			aStream << endl;
		}
	}
	aStream << "endpoints" << endl;

	const Array<std::string>* groups = aMuscle.getGroupNames();
	if (groups->getSize() > 0)
	{
		aStream << "begingroups" << endl;
		for (i = 0; i < groups->getSize(); i++)
			aStream << " " << (*groups)[i];
		aStream << endl << "endgroups" << endl;
	}

	if (dynamic_cast<SimmZajacHill*>(&aMuscle))
	{
		SimmZajacHill *szh = dynamic_cast<SimmZajacHill*>(&aMuscle);

		aStream << "max_force " << szh->getMaxIsometricForce() << endl;
		aStream << "optimal_fiber_length " << szh->getOptimalFiberLength() << endl;
		aStream << "tendon_slack_length " << szh->getTendonSlackLength() << endl;
		aStream << "pennation_angle " << szh->getPennationAngle() * rdMath::RTD << endl;
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
				for (i = 0; i < ncs->getNumberOfPoints(); i++)
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
				for (i = 0; i < ncs->getNumberOfPoints(); i++)
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
				for (i = 0; i < ncs->getNumberOfPoints(); i++)
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
				for (i = 0; i < ncs->getNumberOfPoints(); i++)
					aStream << "(" << ncs->getX()[i] << ", " << ncs->getY()[i] << ")" << endl;
				aStream << "endforcevelocitycurve" << endl;
			}
		}
	}
	else if (dynamic_cast<SimmDarrylMuscle*>(&aMuscle))
	{
		SimmDarrylMuscle *sdm = dynamic_cast<SimmDarrylMuscle*>(&aMuscle);

		aStream << "max_force " << sdm->getMaxIsometricForce() << endl;
		aStream << "optimal_fiber_length " << sdm->getOptimalFiberLength() << endl;
		aStream << "tendon_slack_length " << sdm->getTendonSlackLength() << endl;
		aStream << "pennation_angle " << sdm->getPennationAngle() * rdMath::RTD << endl;
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
