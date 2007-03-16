// scale.cpp
// Author: Peter Loan
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
#include <OpenSim/Simulation/SDFast/sdfast.h>

extern "C" {
#include "universal.h"

extern dpModelStruct sdm;

//_____________________________________________________________________________
/**
 * Scale the kinematic constraint functions for all Qs (SIMM DOFs) that are
 * translational and in a joint whose inboard body is SdfastBodyIndex.
 * This function assumes that the axis of each translational Q is either the
 * X, Y, or Z axis of the inboard body (which is true for all SIMM models).
 */
void scaleConstraints(int SdfastBodyIndex, double scaleFactor[3])
{
	int i, j;

	for (i = 0; i < sdm.nq; i++) {
		int jointIndex = sdm.q[i].joint;

		if (sdm.joint[jointIndex].inboard_body == SdfastBodyIndex) {
         dpSplineFunction* func = sdm.q[i].constraint_func;

			if (func) {
				// See if this Q is translational or rotational. You
				// want to scale only the translational ones.
				int info[50], slider[6];
				sdjnt(jointIndex, info, slider);
				if (slider[sdm.q[i].axis] == 1) {
					double factor, pin[3];

					// Check the axis for this Q to determine which
					// scale factor to use. This code should be more
					// general than comparing each axis component to 1.0,
					// but all SIMM models use the XYZ axes for translations,
					// so pin[] will always have one component equal to 1.0.
					sdgetpin(jointIndex, sdm.q[i].axis, pin);
					if (EQUAL_WITHIN_ERROR(pin[0], 1.0))
						factor = scaleFactor[0];
					else if (EQUAL_WITHIN_ERROR(pin[1], 1.0))
						factor = scaleFactor[1];
					else
						factor = scaleFactor[2];

					for (j = 0; j < func->numpoints; j++)
						func->y[j] *= factor;

					// Now recalculate the coefficients for the scaled function.
					calc_spline_coefficients(func);
				}
			}
		}
	}
}

}
