#ifndef __SimmMacros_h__
#define __SimmMacros_h__

// SimmMacros.h
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

namespace OpenSim {

#define DEG_TO_RAD 0.017453292519943
#define RAD_TO_DEG 57.295779513082323
#define DTOR DEG_TO_RAD
#define RTOD RAD_TO_DEG

#define TINY_NUMBER 0.0000001
#define ROUNDOFF_ERROR 0.0000000000002
#define MAX(a,b) ((a)>=(b)?(a):(b))
#define MIN(a,b) ((a)<=(b)?(a):(b))
#define DABS(a) ((a)>(double)0.0?(a):(-(a)))
#define DSIGN(a) ((a)>=0.0?(1):(-1))
#define SQR(x)	((x) * (x))
#define EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) <= ROUNDOFF_ERROR)
#define NOT_EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) > ROUNDOFF_ERROR)
#define EQUAL_WITHIN_TOLERANCE(a,b,c) (DABS(((a)-(b))) <= (c))
#define MAKE_3DVECTOR(pt1,pt2,pt3) {pt3[0]=pt2[0]-pt1[0];pt3[1]=pt2[1]-pt1[1];pt3[2]=pt2[2]-pt1[2];}
#define MAKE_3DVECTOR21(pt1,pt2,pt3) {pt3[0]=pt1[0]-pt2[0];pt3[1]=pt1[1]-pt2[1];pt3[2]=pt1[2]-pt2[2];}
#define COPY_1X3VECTOR(from,to) {\
to[0] = from[0];\
to[1] = from[1];\
to[2] = from[2];\
}
#define CALC_DETERMINANT(m) (((m[0][0]) * ((m[1][1])*(m[2][2]) - (m[1][2])*(m[2][1]))) - \
((m[0][1]) * ((m[1][0])*(m[2][2]) - (m[1][2])*(m[2][0]))) + \
((m[0][2]) * ((m[1][0])*(m[2][1]) - (m[1][1])*(m[2][0]))))

} // end of namespace OpenSim

#endif // __SimmMacros_h__


