#ifndef __AbstractWrapObject_h__
#define __AbstractWrapObject_h__

// AbstractWrapObject.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
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


// INCLUDE
#include <iostream>
#include <string>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/Transform.h>

namespace OpenSim {

class VisibleObject;
class AbstractBody;
class AbstractDynamicsEngine;
class SimmMusclePoint;
class MuscleWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * An abstract class that specifies the interface for a muscle wrapping
 * object.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API AbstractWrapObject : public Object
{

//=============================================================================
// DATA
//=============================================================================
public:
	enum WrapQuadrant
	{
		allQuadrants,
		negativeX,
		positiveX,
		negativeY,
		positiveY,
		negativeZ,
		positiveZ
	};

	enum WrapAction
	{
		noWrap,          // the muscle line did not intersect the wrap object
		insideRadius,    // one or both muscle points are inside the wrap object
		wrapped,         // successful wrap, but may not be 'best' path
		mandatoryWrap    // successful wrap that must be used (e.g., both tangent
	};                  // points are on the constrained side of the wrap object)

protected:

	PropertyDblArray _xyzBodyRotationProp;
	Array<double>& _xyzBodyRotation;

	PropertyDblArray _translationProp;
	Array<double>& _translation;

	PropertyBool _activeProp;
	bool& _active;

	PropertyStr _quadrantNameProp;
	std::string& _quadrantName;
	WrapQuadrant _quadrant;
	int _wrapAxis;
	int _wrapSign;

	AbstractBody* _body;

	Transform _pose;
	Transform _inversePose;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractWrapObject();
	AbstractWrapObject(DOMElement* aElement);
	AbstractWrapObject(const AbstractWrapObject& aWrapObject);
	virtual ~AbstractWrapObject();
	virtual Object* copy() const = 0;
	virtual Object* copy(DOMElement* aElement) const = 0;
#ifndef SWIG
	AbstractWrapObject& operator=(const AbstractWrapObject& aWrapObject);
#endif
   void copyData(const AbstractWrapObject& aWrapObject);

	virtual void scale(Array<double>& aScaleFactors) = 0;
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	AbstractBody* getBody() const { return _body; }
	const double* getXYZBodyRotation() const { return &_xyzBodyRotation[0]; }
	const double* getTranslation() const { return &_translation[0]; }
	bool getActive() const { return _active; }
	bool getActiveUseDefault() const { return _activeProp.getUseDefault(); }
	const char* getQuadrantName() const { return _quadrantName.c_str(); }
	bool getQuadrantNameUseDefault() const { return _quadrantNameProp.getUseDefault(); }
	virtual const char* getWrapTypeName() const = 0;
	virtual std::string getDimensionsString() const { return ""; } // TODO: total SIMM hack!
	int wrapMuscleSegment(SimmMusclePoint& aPoint1, SimmMusclePoint& aPoint2,
		const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult) const;
	virtual int wrapLine(Array<double>& aPoint1, Array<double>& aPoint2,
		const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const = 0;

	virtual VisibleObject* getDisplayer() { return NULL; }
	virtual void peteTest() const;

protected:
	void setupProperties();
	void get_point_from_point_line2(double point[], double pt[], double vec[],
		double closest_pt[], double* t) const;
	void get_point_from_point_line(double point[], double pt[],
		double vec[], double closest_pt[]) const;
	void make_3x3_xrot_matrix(double a, double m[][3]) const;
	void make_4x4dircos_matrix(double angle, double axis[], double mat[][4]) const;
	double distancesqr_between_vertices(double vertex1[], double vertex2[]) const;
	double get_distsqr_point_line(double point[], double pl[], double vl[]) const;
	bool intersect_line_plane01(double pt1[], double pt2[], double plane[], double d,
		double inter[], double* t) const;
	bool intersect_lines(double p1[], double p2[], double p3[], double p4[],
		double p_int1[], double* t, double p_int2[], double* s) const;
	bool intersect_lines_scaled(double p1[], double p2[], double p3[], double p4[],
		double p_int1[], double* t, double* mag1,
		double p_int2[], double* s, double* mag2) const;
	void rotate_matrix_axis_angle(double m[][4], const double axis[3], double angle) const;
	void make_quaternion(double q[4], const double axis[3], double angle) const;
	void quat_to_matrix(const double q[4], double m[][4]) const;
	void rotate_matrix_by_quat(double m[][4], const double q[4]) const;
	void x_rotate_matrix_bodyfixed(double m[][4], double radians) const;
	void y_rotate_matrix_bodyfixed(double m[][4], double radians) const;
	void z_rotate_matrix_bodyfixed(double m[][4], double radians) const;

private:
	void setNull();
//=============================================================================
};	// END of class AbstractWrapObject
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractWrapObject_h__


