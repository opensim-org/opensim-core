// fjc.cpp
// Author: Eran Guendelman
/* Copyright (c) 2006, Stanford University and Ayman Habib.
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


// INCLUDES
#include <string>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/VisibleProperties.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/AbstractJoint.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Common/SimmPoint.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmKinematicsEngine.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Common/MarkerData.h>
#include <SimTKCommon.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Vec4;
using SimTK::Mat33;
using SimTK::Mat44;

typedef std::map<std::string, std::string, std::less<std::string> > StringMap;

static void PrintUsage(ostream &aOStream);

static Vec3 transform(const Mat44 &mat, const Vec3 &vec)
{
	Vec4 vec4(vec[0], vec[1], vec[2], 1.0);
	vec4 = mat * vec4;
	return Vec3(vec4[0], vec4[1], vec4[2]);
}

//###########################################################################
// Helper functions
//###########################################################################

static void defineCoordinateSystem(const Vec3 &a, const Vec3 &b, const Vec3 &c, Mat44 &transformIntoCS)
{
	Vec3 ab = b - a, ac = c - a;
	Vec3 origin = a + (dot(ab, ac)/ab.normSqr() * ab);
	Vec3 xaxis = unitVec(cross(ab, ac)); //.normalized();
	Vec3 yaxis = unitVec(c-origin); //.normalized();
	Vec3 zaxis = cross(xaxis, yaxis);

	// matrix to be multiplied by column vector on right
	Mat44 matrix;
	for(int i=0; i<3; i++) {
		matrix(i,0) = xaxis[i];
		matrix(i,1) = yaxis[i];
		matrix(i,2) = zaxis[i];
		matrix(i,3) = origin[i];
		matrix(3,i) = 0;
	}
	matrix(3,3) = 1;

	transformIntoCS = matrix.invert();
}

static void instantaneousRotationAxis(const Vec3 &a, const Vec3 &b, const Vec3 &c, Vec3 &axis, Vec3 &point)
{
	Vec3 ab = b-a, bc = c-b;
	axis = unitVec(cross(ab, bc));

	Mat33 matrix;
	Vec3 rhs;
	for(int i=0; i<3; i++) {
		matrix(0,i) = ab[i];
		matrix(1,i) = bc[i];
		matrix(2,i) = axis[i];
	}
	rhs[0] = dot(ab, 0.5 * (a+b));
	rhs[1] = dot(bc, 0.5 * (b+c));
	rhs[2] = dot(axis, b);

	point = matrix.invert() * rhs;
}

// This can use a more efficient symmetric solver if we support symmetric matrices
static Vec3 bestFitIntersectionPoint(const Array<Vec3> &linePoints, const Array<Vec3> &lineDirections)
{
	Vec3 rhs(0);
	Mat33 A(0);
	for(int i=0; i<linePoints.getSize(); i++) {
		Vec3 di = unitVec(lineDirections[i]);
		Mat33 Ai = Mat33(1) - di * ~di;
		rhs += Ai * linePoints[i];
		A += Ai;
	}
	return A.invert() * rhs;
}

//###########################################################################
// Main code
//###########################################################################

//______________________________________________________________________________
/**
* Test program to read SIMM model elements from an XML file.
*
* @param argc Number of command line arguments (should be 1).
* @param argv Command line arguments:  simmReadXML inFile
*/
int main(int argc,char **argv)
{
	bool verbose = true;

	// SET OUTPUT FORMATTING
	IO::SetDigitsPad(4);

	// PARSE COMMAND LINE
	string inName;
	string option = "";
	if (argc < 2) {
		PrintUsage(cout);
		exit(-1);
	} else {
		int i;
		for(i=1;i<=(argc-1);i++) {
			option = argv[i];

			// PRINT THE USAGE OPTIONS
			if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
				(option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
					PrintUsage(cout);
					return(0);

				// Identify the setup file
				} else if((option=="-S")||(option=="-Setup")) {
					if (argv[i+1]==0){
						PrintUsage(cout);
						return(0);
					}
					inName = argv[i+1];
					break;

				// Print a default setup file
				} else if((option=="-PrintSetup")||(option=="-PS")) {
					ScaleTool *subject = new ScaleTool();
					subject->setName("default");
					// Add in useful objects that may need to be instantiated
					Object::setSerializeAllDefaults(true);
					subject->print("default_subject.xml");
					Object::setSerializeAllDefaults(false);
					cout << "Created file default_subject.xml with default setup" << endl;
					return(0);

				// Unrecognized
				} else {
					cout << "Unrecognized option " << option << " on command line... Ignored" << endl;
					PrintUsage(cout);
					return(0);
				}
		}
	}


	try {
		//std::string markerDataFile = "delaware3_ss_walking1.trc";
		std::string markerDataFile = "subject01_walk1.trc";

		// CONSTRUCT SUBJECT INSTANCE
		ScaleTool *subject = new ScaleTool(inName);
		Object *subjectCopy = subject->copy();
		subjectCopy->print("test_subject.xml");

		// CONSTRUCT THE MODEL
		Model *model = subject->createModel();

		// WRITE MODEL TO FILE
		Object *modelCopy = model->copy();
		modelCopy->print("gait_test.osim");

		Vec3 defaultPoint(0);
		Array<Vec3> calculatedJointCenterPosition(defaultPoint, model->getNumJoints());
		Array<bool> calculatedJointCenter(false, model->getNumJoints());
		Array<double> genericModelSegmentLength(0, model->getNumBodies());
		Array<int> inboardJoint(-1, model->getNumBodies()); // joint index that this body is a child of
		Array<int> outboardJoint(-1, model->getNumBodies()); // joint index that this body is a parent of

		MarkerData markerData(markerDataFile);
		markerData.convertToUnits(model->getLengthUnits());
		model->getDynamicsEngine().deleteUnusedMarkers(markerData.getMarkerNames());

		Storage markerStorage;
		markerData.makeRdStorage(markerStorage);

		BodySet *bs = model->getDynamicsEngine().getBodySet();
		JointSet *js = model->getDynamicsEngine().getJointSet();
		MarkerSet *ms = model->getDynamicsEngine().getMarkerSet();

		StringMap mapMarkersToAdditionalBodies;
		mapMarkersToAdditionalBodies["R.Acromium"] = "humerus_r";
		mapMarkersToAdditionalBodies["L.Acromium"] = "humerus_l";
		mapMarkersToAdditionalBodies["R.Elbow"] = "radius_r";
		mapMarkersToAdditionalBodies["L.Elbow"] = "radius_l";

		// Get list of markers on each body
		ArrayPtrs<ArrayPtrs<AbstractMarker> > markersPerBody;
		markersPerBody.setSize(model->getNumBodies());
		for(int i=0;i<model->getNumBodies();i++) {
			markersPerBody.set(i,new ArrayPtrs<AbstractMarker>);
			markersPerBody[i]->setMemoryOwner(false);
		}
		for(int i=0; i<ms->getSize(); i++) {
			AbstractMarker *marker = ms->get(i);
			int index = bs->getIndex(marker->getBody());
			if(index >= 0) markersPerBody.get(index)->append(marker);
			if(mapMarkersToAdditionalBodies.find(marker->getName()) != mapMarkersToAdditionalBodies.end()) {
				std::string additionalBody = mapMarkersToAdditionalBodies[marker->getName()];
				std::cout << "Also adding '" << marker->getName() << "' to body '" << additionalBody << "'" << std::endl;
				index = bs->getIndex(additionalBody);
				if(index >= 0) markersPerBody.get(index)->append(marker);
			}
		}

		if(verbose) {
			std::cout << "Bodies (" << model->getNumBodies() << "):" << std::endl;
			for(int i=0; i<model->getNumBodies(); i++) {
				AbstractBody *body = bs->get(i);
				std::cout << i << ": " << body->getName() << " (#markers=" << markersPerBody[i]->getSize() << ")" << std::endl;
			}
			std::cout << std::endl;
		}

		StringMap remapParentBody;
		remapParentBody["subtalar_r"] = "tibia_r";
		remapParentBody["subtalar_l"] = "tibia_l";
		remapParentBody["radioulnar_r"] = "humerus_r";
		remapParentBody["radioulnar_l"] = "humerus_l";

		if(verbose) std::cout << "Joints (" << model->getNumJoints() << "):" << std::endl;
		for(int i=0; i<model->getNumJoints(); i++) {
			AbstractJoint *joint = js->get(i);
			AbstractBody *parent = joint->getParentBody();
			AbstractBody *child = joint->getChildBody();
			if(verbose) std::cout << i << ": " << joint->getName() << " (parent=" << parent->getName() << ", child=" << child->getName() << ")" << std::endl;

			// Make tibia be parent of calcn
			if(remapParentBody.find(joint->getName()) != remapParentBody.end()) {
				parent = bs->get(remapParentBody[joint->getName()]);
				if(verbose) std::cout << "[modified]" << i << ": " << joint->getName() << " (parent=" << parent->getName() << ", child=" << child->getName() << ")" << std::endl;
			} 

			int parentIndex = bs->getIndex(parent->getName());
			int childIndex = bs->getIndex(child->getName());
			inboardJoint[childIndex] = i;
			outboardJoint[parentIndex] = i;

			// Compute length of parent segment (the distance from the parent segment's inboard joint to outboard joint) 
			// by finding the length of the vector to the child's origin in the parent frame
			double outboard[3]={0,0,0};
			model->getDynamicsEngine().transformPosition(*child,outboard,*parent,outboard);
			genericModelSegmentLength[parentIndex] = Vec3(outboard).norm();

			// Need at least 3 markers to define a reference coordinate system on parent (inboard)
			if(markersPerBody.get(parentIndex)->getSize() < 3) continue;

			// We'll use the *first three* markers to define the inboard frame.  Ideally we should read in some configuration
			// file which specifies which three we trust the most (or come up with some way that makes use of all markers to define
			// some best-fit frame.
			Array<int> inboardMarkerIndicesInData(-1);
			Array<int> inboardMarkersUsed(-1);
			for(int mindex=0; mindex<markersPerBody.get(parentIndex)->getSize() && inboardMarkerIndicesInData.getSize() < 3; mindex++) {
				int index = markerData.getMarkerIndex(markersPerBody.get(parentIndex)->get(mindex)->getName());
				if (index >= 0) {
					inboardMarkerIndicesInData.append(index);
					inboardMarkersUsed.append(mindex);
				}
			}

			// Need at least three markers to define a frame
			if(inboardMarkerIndicesInData.getSize() < 3) continue;

			// Figure out which outboard markers are usable (must exist in data)
			Array<int> outboardMarkerIndicesInData(-1);
			Array<int> outboardMarkersUsed(-1);
			for(int mindex=0; mindex<markersPerBody.get(childIndex)->getSize(); mindex++) {
				// if marker actually attached to a different body than the child... we won't use it
				if(markersPerBody.get(childIndex)->get(mindex)->getBody() != child) continue;
				int index = markerData.getMarkerIndex(markersPerBody.get(childIndex)->get(mindex)->getName());
				if (index >= 0) {
					outboardMarkerIndicesInData.append(index);
					outboardMarkersUsed.append(mindex);
				}
			}

			// Need at least one outboard marker
			if(outboardMarkerIndicesInData.getSize() == 0) continue;

			Array<Vec3> rotationPoints(defaultPoint);
			Array<Vec3> rotationAxes(defaultPoint);

			// criteria for point triplet
			double minimumDistanceSquared = 0.04 * 0.04;
			double maximumCosine = cos(10 * rdMath::DTR);
			// no criteria to begin with
			//double minimumDistanceSquared = 0;
			//double maximumCosine = 1;

			Mat44 canonicalWorldTransform;
			bool gotCanonicalTransform = false;

			for(int outboardMarkerIndex=0; outboardMarkerIndex<outboardMarkerIndicesInData.getSize(); outboardMarkerIndex++) {
				if(verbose) std::cout << "Processing outboard marker " << markersPerBody.get(childIndex)->get(outboardMarkersUsed[outboardMarkerIndex])->getName() << std::endl;
				Array<Vec3> markerTriplet(defaultPoint);
				for(int findex=0; findex<markerData.getNumFrames(); findex++) {
					MarkerFrame *frame = markerData.getFrame(findex);
				
					// Construct transform into inboard body frame	
					Vec3 markerLocations[3];
					for(int j=0; j<3; j++)
						markerLocations[j] = Vec3(frame->getMarker(inboardMarkerIndicesInData[j]).get());
					Mat44 transformIntoInboardCS;
					defineCoordinateSystem(markerLocations[0], markerLocations[1], markerLocations[2], transformIntoInboardCS);

					if(!gotCanonicalTransform) {
						canonicalWorldTransform = transformIntoInboardCS.invert();
						gotCanonicalTransform = true;
					}
		
					// Find outboard point in inboard body frame	
					Vec3 outboardMarkerPosition = Vec3(frame->getMarker(outboardMarkerIndicesInData[outboardMarkerIndex]).get());
					Vec3 position = transform(transformIntoInboardCS, outboardMarkerPosition);

					// Construct triplet of points
					if (markerTriplet.getSize() == 0) {
						markerTriplet.append(position);
					} else if (markerTriplet.getSize() == 1) {
						if ((position - markerTriplet[0]).normSqr() > minimumDistanceSquared) {
							markerTriplet.append(position);
						}
					} else {
						assert(markerTriplet.getSize() == 2);
						Vec3 bc = (position - markerTriplet[1]);
						double bcMagSqr = bc.normSqr();
						if (bcMagSqr > minimumDistanceSquared) {
							Vec3 ac = (position - markerTriplet[0]);
							double acMagSqr = ac.normSqr();
							if (acMagSqr > minimumDistanceSquared) {
								Vec3 ab = unitVec(markerTriplet[1] - markerTriplet[0]);
								bc /= sqrt(bcMagSqr);
								ac /= sqrt(acMagSqr);
								if (dot(bc, ac) < maximumCosine &&
									dot(ab, bc) < maximumCosine &&
									dot(ab, ac) < maximumCosine) {

									// We have a valid triplet, so we can calculate axis of rotation
									Vec3 rotationAxis, rotationPoint;
									instantaneousRotationAxis(markerTriplet[0], markerTriplet[1], position, rotationAxis, rotationPoint);
									rotationAxes.append(rotationAxis);
									rotationPoints.append(rotationPoint);
								
									markerTriplet[0] = markerTriplet[1];
									markerTriplet[1] = position;
								}
							}
						}
					}
				}
			}

			if(rotationPoints.getSize()) {
				// Calculate best fit intersection point for all of the computed axes of rotation
				calculatedJointCenter[i] = true;
				calculatedJointCenterPosition[i] = transform(canonicalWorldTransform,bestFitIntersectionPoint(rotationPoints, rotationAxes));
				if(verbose) std::cout << "Calculated rotation center = " << calculatedJointCenterPosition[i] << " (" << rotationPoints.getSize() << " samples)" << std::endl;
			}
		}

		std::cout << std::endl;

		Array<double> scaleSet(0, model->getNumBodies());

		for(int i=0; i<model->getNumBodies(); i++) {
			AbstractBody *body = bs->get(i);
			std::cout << "Body " << i << ": " << body->getName() << std::endl;

			if(genericModelSegmentLength[i]) {
				std::cout << "  Generic model length = " << genericModelSegmentLength[i] << std::endl;
			}
			if(inboardJoint[i]<0) {
				if(verbose) std::cout << "  No inboard joint" << std::endl;
				continue;
			} else if(outboardJoint[i]<0) {
				if(verbose) std::cout << "  No outboard joint" << std::endl;
				continue;
			} else if(!calculatedJointCenter[inboardJoint[i]]) {
				if(verbose) std::cout << "  Did not calculate inboard joint center" << std::endl;
				continue;
			} else if(!calculatedJointCenter[outboardJoint[i]]) {
				if(verbose) std::cout << "  Did not calculate outboard joint center" << std::endl;
				continue;
			} else {
				Vec3 inb = calculatedJointCenterPosition[inboardJoint[i]];
				Vec3 outb = calculatedJointCenterPosition[outboardJoint[i]];

				std::cout << "  Calculated joint centers: "
						  << js->get(inboardJoint[i])->getName()
						  << "=" << inb << ", " << js->get(outboardJoint[i])->getName() << "=" << outb << std::endl;
				std::cout << "  Distance between calculated joint centers = " << (outb-inb).norm() << std::endl;

				if(genericModelSegmentLength[i]) {
					scaleSet[i] = (outb-inb).norm()/genericModelSegmentLength[i];
					std::cout << "  SCALE = " << scaleSet[i] << std::endl;
				}
			}
		}

		// Take averages of both sides
		for(int i=0; i<bs->getSize(); i++) {
			std::string name = bs->get(i)->getName();
			if(scaleSet[bs->getIndex(name)] == 0) continue;
			if(name.substr(name.length()-2) == "_r") {
				std::string basename = name.substr(0,name.length()-2);
				std::string lname = basename + "_l";
				if(scaleSet[bs->getIndex(lname)] == 0) continue;
				std::cout << "AVERAGE (" << basename << ") = " << 0.5*(scaleSet[bs->getIndex(name)] + scaleSet[bs->getIndex(lname)]) << std::endl;
			}
		}

	// HANDLE ANY EXCEPTIONS
	} catch(Exception &x) {
		x.print(cout);
	}

}

//_____________________________________________________________________________
/**
* Print the usage for this application
*/
void PrintUsage(ostream &aOStream)
{
	aOStream<<"\n\nfjc.exe:\n\n";
	aOStream<<"Option            Argument      Action / Notes\n";
	aOStream<<"------            --------      --------------\n";
	aOStream<<"-Help, -H                       Print the command-line options for fjc.exe.\n";
	aOStream<<"-PrintSetup, -PS                Generates a template Setup file to customize scaling\n";
	aOStream<<"-Setup, -S        SetupFile     Specify an xml setup file that specifies an OpenSim model,\n";
	aOStream<<"                                a marker file, and scaling parameters.\n";
}
