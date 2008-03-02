#ifndef _ControlLinearNode_h_
#define _ControlLinearNode_h_
// ControlLinearNode.h:
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/PropertyDbl.h>

//=============================================================================
//=============================================================================
/**
 * A control node used to reconstruct a piecewise linear control.
 *
 * The member variables consist of a time, a value, a minimum value, and
 * a maximum value.  So that an Array<T> can be instantiated for
 * ControlLinearNode, this class implements a default constructor, a copy
 * constructor, the assignment operator (=), the equality operator (==),
 * and the less than operator (<).  The time at which a control node
 * occurs is used to determine the results of the operators == and <.
 *
 * @author Frank C. Anderson
 * @version 1.0
 * @see ControlLinear
 */
namespace OpenSim { 

class OSIMSIMULATION_API ControlLinearNode : public Object
{

//=============================================================================
// MEMBER DATA
//=============================================================================
public:
	/** Equality tolerance. */
	//static double _EqualityTolerance;

protected:
	// PROPERTIES
	/** Time at which the node occurs. */
	PropertyDbl _propT;
	/** Value of the node (may represent control value or min or max bounds, depending on which curve it's in). */
	PropertyDbl _propValue;

	// REFERENCES
	/** Reference to the value of the T property. */
	double &_t;
	/** Reference to the value of the X property. */
	double &_value;

//=============================================================================
// METHODS
//=============================================================================
public:
	ControlLinearNode(double aT=0.0,double aValue=0.0);
	ControlLinearNode(const ControlLinearNode &aNode);
	virtual Object* copy() const;
	virtual ~ControlLinearNode();
private:
	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	ControlLinearNode& operator=(const ControlLinearNode &aControl);
	bool operator==(const ControlLinearNode &aControl) const;
	bool operator<(const ControlLinearNode &aControl) const;

	friend std::ostream& operator<<(std::ostream &aOut,
		const ControlLinearNode &aControlLinearNode);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
	//static void SetEqualityTolerance(double aTol);
	//static double GetEqualityTolerance();
	void setTime(double aT);
	double getTime() const;
	void setValue(double aValue);
	double getValue() const;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	char* toString();

//=============================================================================
};	// END of class ControlLinearNode

}; //namespace
//=============================================================================
//=============================================================================

#endif // __ControlLinearNode_h__
