#ifndef __MomentArmSolver_h__
#define __MomentArmSolver_h__
// MomentArmSolver.h
/*
* Copyright (c)  2010, Stanford University. All rights reserved. 
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

#include "Solver.h"

namespace OpenSim {

class PointForceDirection;
class Coordinate;

//=============================================================================
//=============================================================================
/**
 * Solve for the effective moment arms at all degrees-of-freedom due to one or
 * more point forces.  This may result from the underlying geometry of a Force 
 * or Actuator with a complex path (like ligaments and muscles) but this solver
 * is only concerned with the set of points and unit forces that maps a scalar
 * force value (like tension) to the resulting generalized force.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API MomentArmSolver: public Solver
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~MomentArmSolver() {};

	MomentArmSolver(const Model &model);
	/** Solve for the effective moment-arm about the specified coordinate based 
	    on the geometric distribution of forces described by the list of 
		PointForceDirections. */
	double solve(const SimTK::State &s, const Coordinate &aCoord, 
		const Array<PointForceDirection *> &pfds);

private:

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:


//=============================================================================
};	// END of class MomentArmSolver
//=============================================================================
} // namespace

#endif // __MomentArmSolver_h__
