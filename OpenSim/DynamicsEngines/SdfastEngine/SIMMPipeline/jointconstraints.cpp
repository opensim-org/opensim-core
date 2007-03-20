// scale.cpp
// Author: Eran Guendelman, Peter Loan
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
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
#include <OpenSim/DynamicsEngines/SdfastEngine/sdfast.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/DofSet.h>
#include <OpenSim/DynamicsEngines/SdfastEngine/SdfastCoordinate.h>
#include <OpenSim/Common/NatCubicSpline.h>
#include <string>

extern "C" {
#include "universal.h"
extern dpModelStruct sdm;
}

using namespace OpenSim;
using namespace std;

void setJointConstraintFunctions(CoordinateSet *aCoordinateSet)
{
	for (int i = 0; i < sdm.nq; i++) {
		int jointIndex = sdm.q[i].joint;

		dpSplineFunction* func = sdm.q[i].constraint_func;

		if (func) {
			//std::cout << "Got constraint func for " << sdm.q[i].name << std::endl;

			int index = aCoordinateSet->getIndex(sdm.q[i].name);
			if(index<0) throw Exception("setJointConstraintFunctions: ERR- could not find coordinate named '"+string(sdm.q[i].name)+"'",__FILE__,__LINE__);
			SdfastCoordinate *coordinate = dynamic_cast<SdfastCoordinate*>(aCoordinateSet->get(index));
			if(!coordinate) throw Exception("setJointConstraintFunctions: ERR- dynamic_cast to SdfastCoordinate failed",__FILE__,__LINE__);
			Function *function = coordinate->getConstraintFunction();
			if(!function) throw Exception("setJointConstraintFunctions: ERR- coordinate '"+string(sdm.q[i].name)+" missing constraint function",__FILE__,__LINE__);
			NatCubicSpline *spline = dynamic_cast<NatCubicSpline*>(function);
			if(!spline) throw Exception("setJointConstraintFunctions: ERR- constraint function of '"+string(sdm.q[i].name)+"' expected to be a natCubicSpline",__FILE__,__LINE__);

			(void)malloc_function(func,spline->getSize());
			func->numpoints = spline->getSize();
			for(int j=0;j<spline->getSize();j++) {
				func->x[j] = spline->getX()[j];
				func->y[j] = spline->getY()[j];
				//std::cout << "SET FUNC " << j << ": " << func->x[j] << ", " << func->y[j] << std::endl;
			}

			// Now recalculate the coefficients for the function.
			calc_spline_coefficients(func);
		}
	}
}

void setCoordinateInitialValues(CoordinateSet *aCoordinateSet)
{
	for (int i = 0; i < sdm.nq; i++) {
		int index = aCoordinateSet->getIndex(sdm.q[i].name);
		if(index<0) throw Exception("setJointConstraintFunctions: ERR- could not find coordinate named '"+string(sdm.q[i].name)+"'",__FILE__,__LINE__);
		SdfastCoordinate *coordinate = dynamic_cast<SdfastCoordinate*>(aCoordinateSet->get(index));
		if(!coordinate) throw Exception("setJointConstraintFunctions: ERR- dynamic_cast to SdfastCoordinate failed",__FILE__,__LINE__);
		//std::cout << sdm.q[i].name << ": " << coordinate->getInitialValue() << endl;
		sdm.q[i].initial_value = coordinate->getInitialValue();
	}
}
