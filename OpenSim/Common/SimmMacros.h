#ifndef __SimmMacros_h__
#define __SimmMacros_h__

// SimmMacros.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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


