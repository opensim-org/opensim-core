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
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/VisibleProperties.h>
#include <OpenSim/Tools/ScaleSet.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmPoint.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerSet.h>
#include <OpenSim/Simulation/SIMM/SimmSubject.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerData.h>

using namespace std;
using namespace OpenSim;

static void PrintUsage(ostream &aOStream);

//###########################################################################
// class MyMatrix4
//###########################################################################
class MyMatrix4
{
private:
	double x[16];

public:
	MyMatrix4()
	{
		for(int i=0; i<16; i++) x[i] = 0;
	}

	MyMatrix4(const MyMatrix4 &aMatrix)
	{
		(*this) = aMatrix;
	}

	double& operator()(int i, int j) { return x[i+4*j]; }
	double operator()(int i, int j) const { return x[i+4*j]; }

	MyMatrix4& operator=(const MyMatrix4 &aMatrix)
	{
		for(int i=0; i<16; i++) x[i] = aMatrix.x[i];
		return *this;
	}

	MyMatrix4 operator*(const MyMatrix4 &aMatrix) const
	{
		MyMatrix4 result;
		for(int i=0; i<4; i++)
			for(int j=0; j<4; j++) 
				for(int k=0; k<4; k++)
					result(i,j) += (*this)(i,k) * aMatrix(k,j);
		return result;
	}

	SimmPoint transformPoint(const SimmPoint &aPoint) const
	{
		SimmPoint result;
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++) result[i] += (*this)(i,j)*aPoint[j];
			result[i] += (*this)(i,3);
		}
		return result;
	}

	SimmPoint transformVector(const SimmPoint &aVector) const
	{
		SimmPoint result;
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++) result[i] += (*this)(i,j)*aVector[j];
		}
		return result;
	}

	void invert()
	{
		Mtx::Invert(4, x, x);
	}

	MyMatrix4 inverse()
	{
		MyMatrix4 result;
		Mtx::Invert(4, x, result.x);
		return result;
	}

};

ostream& operator<<(ostream& os, const MyMatrix4& matrix)
{
	for(int i=0; i<4; i++) {
		for(int j=0; j<4; j++)
			os << matrix(i,j) << " ";
		os << std::endl;
	}
	return os;
}

class MyMatrix3
{
private:
	double x[9];

public:
	MyMatrix3()
	{
		for(int i=0; i<9; i++) x[i] = 0;
	}

	MyMatrix3(const MyMatrix3 &aMatrix)
	{
		(*this) = aMatrix;
	}

	double& operator()(int i, int j) { return x[i+3*j]; }
	double operator()(int i, int j) const { return x[i+3*j]; }

	MyMatrix3 &operator+=(const MyMatrix3 &aMatrix)
	{
		for(int i=0; i<9; i++) x[i] += aMatrix.x[i];
		return *this;
	}

	MyMatrix3 operator-(const MyMatrix3 &aMatrix) const
	{
		MyMatrix3 result;
		for(int i=0; i<9; i++) result.x[i] = x[i] - aMatrix.x[i];
		return result;
	}

	MyMatrix3& operator=(const MyMatrix3 &aMatrix)
	{
		for(int i=0; i<9; i++) x[i] = aMatrix.x[i];
		return *this;
	}

	MyMatrix3 operator*(const MyMatrix3 &aMatrix) const
	{
		MyMatrix3 result;
		for(int i=0; i<3; i++)
			for(int j=0; j<3; j++) 
				for(int k=0; k<3; k++)
					result(i,j) += (*this)(i,k) * aMatrix(k,j);
		return result;
	}

	SimmPoint operator*(const SimmPoint &aPoint) const
	{
		SimmPoint result;
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++) result[i] += (*this)(i,j)*aPoint[j];
		}
		return result;
	}

	void invert()
	{
		Mtx::Invert(3, x, x);
	}

	MyMatrix3 inverse()
	{
		MyMatrix3 result;
		Mtx::Invert(3, x, result.x);
		return result;
	}

	static MyMatrix3 Identity() 
	{
		MyMatrix3 result;
		result(0,0)=result(1,1)=result(2,2)=1;
		return result;
	}

	static MyMatrix3 OuterProduct(const SimmPoint &aPoint1, const SimmPoint &aPoint2)
	{
		MyMatrix3 result;
		for(int i=0; i<3; i++)
			for(int j=0; j<3; j++)
				result(i,j) = aPoint1[i]*aPoint2[j];
		return result;
	}
};

ostream& operator<<(ostream& os, const MyMatrix3& matrix)
{
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			os << matrix(i,j) << " ";
		os << std::endl;
	}
	return os;
}

//###########################################################################
// Helper functions
//###########################################################################

static void defineCoordinateSystem(const SimmPoint &a, const SimmPoint &b, const SimmPoint &c, MyMatrix4 &transformIntoCS)
{
	SimmPoint ab = b - a, ac = c - a;
	SimmPoint origin = a + ((SimmPoint::DotProduct(ab, ac)/ab.magnitudeSquared()) * ab);
	SimmPoint xaxis = SimmPoint::CrossProduct(ab, ac).normalized();
	SimmPoint yaxis = (c-origin).normalized();
	SimmPoint zaxis = SimmPoint::CrossProduct(xaxis, yaxis);

	// matrix to be multiplied by column vector on right
	MyMatrix4 matrix;
	for(int i=0; i<3; i++) {
		matrix(i,0) = xaxis[i];
		matrix(i,1) = yaxis[i];
		matrix(i,2) = zaxis[i];
		matrix(i,3) = origin[i];
		matrix(3,i) = 0;
	}
	matrix(3,3) = 1;

	transformIntoCS = matrix;
	transformIntoCS.invert();
}

static void instantaneousRotationAxis(const SimmPoint &a, const SimmPoint &b, const SimmPoint &c, SimmPoint &axis, SimmPoint &point)
{
	SimmPoint ab = b-a, bc = c-b;
	axis = SimmPoint::CrossProduct(ab, bc).normalized();

	MyMatrix3 matrix;
	SimmPoint rhs;
	for(int i=0; i<3; i++) {
		matrix(0,i) = ab[i];
		matrix(1,i) = bc[i];
		matrix(2,i) = axis[i];
	}
	rhs[0] = SimmPoint::DotProduct(ab, 0.5 * (a+b));
	rhs[1] = SimmPoint::DotProduct(bc, 0.5 * (b+c));
	rhs[2] = SimmPoint::DotProduct(axis, b);

	point = matrix.inverse() * rhs;
}

// This can use a more efficient symmetric solver if we support symmetric matrices
static SimmPoint bestFitIntersectionPoint(const Array<SimmPoint> &linePoints, const Array<SimmPoint> &lineDirections)
{
	SimmPoint rhs;
	MyMatrix3 A;
	for(int i=0; i<linePoints.getSize(); i++) {
		SimmPoint di = lineDirections[i].normalized();
		MyMatrix3 Ai = MyMatrix3::Identity() - MyMatrix3::OuterProduct(di,di);
		rhs += Ai * linePoints[i];
		A += Ai;
	}
	return A.inverse() * rhs;
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
	bool verbose = false;

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
					SimmSubject *subject = new SimmSubject();
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
		std::string markerDataFile = "delaware3_ss_walking1.trc";

		// CONSTRUCT SUBJECT INSTANCE
		SimmSubject *subject = new SimmSubject(inName);
		Object *subjectCopy = subject->copy();
		subjectCopy->print("test_subject.xml");

		// CONSTRUCT THE MODEL
		SimmModel *model = subject->createModel();

		// WRITE MODEL TO FILE
		Object *modelCopy = model->copy();
		modelCopy->print("gait_test.osim");

		SimmPoint defaultPoint;
		Array<bool> calculatedJointCenter(false, model->getNJ());
		Array<SimmPoint> calculatedJointCenterPosition(defaultPoint, model->getNJ());
		Array<double> genericModelSegmentLength(0, model->getNB());
		Array<int> inboardJoint(-1, model->getNB()); // joint index that this body is a child of
		Array<int> outboardJoint(-1, model->getNB()); // joint index that this body is a parent of

		SimmMarkerData markerData(markerDataFile);
		markerData.convertToUnits(model->getLengthUnits());
		model->deleteUnusedMarkers(markerData.getMarkerNames());

		Storage markerStorage;
		markerData.makeRdStorage(markerStorage);

		if(verbose) {
			std::cout << "Bodies (" << model->getNB() << "):" << std::endl;
			for(int i=0; i<model->getNB(); i++) {
				SimmBody *body = model->getBodies()[i];
				std::cout << i << ": " << body->getName() << " (#markers=" << body->getNumMarkers() << ")" << std::endl;
			}
			std::cout << std::endl;
		}

		if(verbose) std::cout << "Joints (" << model->getNJ() << "):" << std::endl;
		for(int i=0; i<model->getNJ(); i++) {
			SimmJoint *joint = model->getSimmKinematicsEngine().getJoint(i);
			SimmBody *parent = joint->getParentBody();
			SimmBody *child = joint->getChildBody();
			if(verbose) std::cout << i << ": " << joint->getName() << " (parent=" << parent->getName() << ", child=" << child->getName() << ")" << std::endl;

			if (joint->getName() == "subtalar_r") {
				parent = model->getSimmKinematicsEngine().getBody("tibia_r");
				if(verbose) std::cout << "[modified]" << i << ": " << joint->getName() << " (parent=" << parent->getName() << ", child=" << child->getName() << ")" << std::endl;
			} else if (joint->getName() == "subtalar_l") {
				parent = model->getSimmKinematicsEngine().getBody("tibia_l");
				if(verbose) std::cout << "[modified]" << i << ": " << joint->getName() << " (parent=" << parent->getName() << ", child=" << child->getName() << ")" << std::endl;
			}

			int parentIndex = model->getBodyIndex(parent->getName());
			int childIndex = model->getBodyIndex(child->getName());
			inboardJoint[childIndex] = i;
			outboardJoint[parentIndex] = i;

			double outboard[3]={0,0,0};
			model->getSimmKinematicsEngine().convertPoint(outboard, parent, child);
			genericModelSegmentLength[parentIndex] = Mtx::Magnitude(3,outboard);

			// Need at least 3 markers to define a reference coordinate system on parent (inboard)
			if(parent->getNumMarkers() < 3) continue;

			// We'll use the *first three* markers to define the inboard frame.  Ideally we should read in some configuration
			// file which specifies which three we trust the most (or come up with some way that makes use of all markers to define
			// some best-fit frame.
			Array<int> inboardMarkerIndicesInData(-1);
			Array<int> inboardMarkersUsed(-1);
			for(int mindex=0; mindex<parent->getNumMarkers() && inboardMarkerIndicesInData.getSize() < 3; mindex++) {
				int index = markerData.getMarkerIndex(parent->getMarker(mindex)->getName());
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
			for(int mindex=0; mindex<child->getNumMarkers(); mindex++) {
				int index = markerData.getMarkerIndex(child->getMarker(mindex)->getName());
				if (index >= 0) {
					outboardMarkerIndicesInData.append(index);
					outboardMarkersUsed.append(mindex);
				}
			}

			// Need at least one outboard marker
			if(outboardMarkerIndicesInData.getSize() == 0) continue;

			Array<SimmPoint> rotationPoints(defaultPoint);
			Array<SimmPoint> rotationAxes(defaultPoint);

			// criteria for point triplet
			double minimumDistanceSquared = 0.04 * 0.04;
			double maximumCosine = cos(10 * rdMath::DTR);
			// no criteria to begin with
			//double minimumDistanceSquared = 0;
			//double maximumCosine = 1;

			MyMatrix4 canonicalWorldTransform;
			bool gotCanonicalTransform = false;

			for(int outboardMarkerIndex=0; outboardMarkerIndex<outboardMarkerIndicesInData.getSize(); outboardMarkerIndex++) {
				if(verbose) std::cout << "Processing outboard marker " << child->getMarker(outboardMarkersUsed[outboardMarkerIndex])->getName() << std::endl;
				Array<SimmPoint> markerTriplet(defaultPoint);
				for(int findex=0; findex<markerData.getNumFrames(); findex++) {
					SimmMarkerFrame *frame = markerData.getFrame(findex);
				
					// Construct transform into inboard body frame	
					SimmPoint markerLocations[3];
					for(int j=0; j<3; j++) {
						markerLocations[j] = frame->getMarker(inboardMarkerIndicesInData[j]);
					}
					MyMatrix4 transformIntoInboardCS;
					defineCoordinateSystem(markerLocations[0], markerLocations[1], markerLocations[2], transformIntoInboardCS);

					if(!gotCanonicalTransform) {
						canonicalWorldTransform = transformIntoInboardCS.inverse();
						gotCanonicalTransform = true;
					}
		
					// Find outboard point in inboard body frame	
					SimmPoint outboardMarkerPosition = frame->getMarker(outboardMarkerIndicesInData[outboardMarkerIndex]);
					SimmPoint position = transformIntoInboardCS.transformPoint(outboardMarkerPosition);

					// Construct triplet of points
					if (markerTriplet.getSize() == 0) {
						markerTriplet.append(position);
					} else if (markerTriplet.getSize() == 1) {
						if ((position - markerTriplet[0]).magnitudeSquared() > minimumDistanceSquared) {
							markerTriplet.append(position);
						}
					} else {
						assert(markerTriplet.getSize() == 2);
						SimmPoint bc = (position - markerTriplet[1]);
						double bcMagSqr = bc.magnitudeSquared();
						if (bcMagSqr > minimumDistanceSquared) {
							SimmPoint ac = (position - markerTriplet[0]);
							double acMagSqr = ac.magnitudeSquared();
							if (acMagSqr > minimumDistanceSquared) {
								SimmPoint ab = (markerTriplet[1] - markerTriplet[0]).normalized();
								bc /= sqrt(bcMagSqr);
								ac /= sqrt(acMagSqr);
								if (SimmPoint::DotProduct(bc, ac) < maximumCosine &&
									SimmPoint::DotProduct(ab, bc) < maximumCosine &&
									SimmPoint::DotProduct(ab, ac) < maximumCosine) {

									// We have a valid triplet, so we can calculate axis of rotation
									SimmPoint rotationAxis, rotationPoint;
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
				calculatedJointCenterPosition[i] = canonicalWorldTransform.transformPoint(bestFitIntersectionPoint(rotationPoints, rotationAxes));
				if(verbose) std::cout << "Calculated rotation center = " << calculatedJointCenterPosition[i] << " (" << rotationPoints.getSize() << " samples)" << std::endl;
			}
		}

		std::cout << std::endl;

		Array<double> scaleSet(0, model->getNB());

		for(int i=0; i<model->getNB(); i++) {
			SimmBody *body = model->getBodies()[i];
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
				SimmPoint inb = calculatedJointCenterPosition[inboardJoint[i]];
				SimmPoint outb = calculatedJointCenterPosition[outboardJoint[i]];

				std::cout << "  Calculated joint centers: "
						  << model->getSimmKinematicsEngine().getJoint(inboardJoint[i])->getName()
						  << "=" << inb << ", " << model->getSimmKinematicsEngine().getJoint(outboardJoint[i])->getName() << "=" << outb << std::endl;
				std::cout << "  Distance between calculated joint centers = " << (outb-inb).magnitude() << std::endl;

				if(genericModelSegmentLength[i]) {
					scaleSet[i] = (outb-inb).magnitude()/genericModelSegmentLength[i];
					std::cout << "  SCALE = " << scaleSet[i] << std::endl;
				}
			}
		}

		// Take some averages
		std::cout << "FEMUR AVERAGE SCALE = " << 0.5*(scaleSet[model->getBodyIndex("femur_r")] + scaleSet[model->getBodyIndex("femur_l")]) << std::endl;
		std::cout << "TIBIA AVERAGE SCALE = " << 0.5*(scaleSet[model->getBodyIndex("tibia_r")] + scaleSet[model->getBodyIndex("tibia_l")]) << std::endl;

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
