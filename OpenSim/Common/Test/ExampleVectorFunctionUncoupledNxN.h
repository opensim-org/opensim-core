#ifndef _ExampleVectorFunctionUncoupledNxN_h_
#define _ExampleVectorFunctionUncoupledNxN_h_
// ExampleVectorFunctionUncoupledNxN.cpp
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

/*  
* Author: Frank C. Anderson 
*/

#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/VectorFunctionUncoupledNxN.h>

//extern template class OSIMCOMMON_API Array<double>;

//=============================================================================
//=============================================================================
/**
* An abstract class for representing a vector function.
*
* A vector function is a relation between some number of independent variables 
* and some number of dependent values such that for any particular set of
* independent variables the correct number of dependent variables is returned.
* Values of the function and its derivatives
* are obtained by calling the evaluate() method.  The curve may or may not
* be finite or diferentiable; the evaluate method returns values between
* Math::MINUS_INFINITY and Math::PLUS_INFINITY, or it returns Math::NAN
* (not a number) if the curve is not defined.
* Currently, functions of up to 3 variables (x,y,z) are supported.
*
* @author Frank C. Anderson
*/
namespace OpenSim { 

	class ExampleVectorFunctionUncoupledNxN : public VectorFunctionUncoupledNxN
	{
		//=============================================================================
		// DATA
		//=============================================================================
	protected:



		//=============================================================================
		// METHODS
		//=============================================================================
	public:
		//--------------------------------------------------------------------------
		// CONSTRUCTION
		//--------------------------------------------------------------------------
		ExampleVectorFunctionUncoupledNxN():
		  VectorFunctionUncoupledNxN(1)
		  {
			  setNull();
		  };
		  ExampleVectorFunctionUncoupledNxN(int aN):
		  VectorFunctionUncoupledNxN(aN)
		  {
			  setNull();
		  };
		  ExampleVectorFunctionUncoupledNxN(const ExampleVectorFunctionUncoupledNxN &aVectorFunction) :
		  VectorFunctionUncoupledNxN(aVectorFunction)
		  {
			  setNull();

			  // ASSIGN
			  setEqual(aVectorFunction);
		  };
		  virtual ~ExampleVectorFunctionUncoupledNxN() {};
		  virtual Object* copy() const{
			  ExampleVectorFunctionUncoupledNxN *func =
				  new ExampleVectorFunctionUncoupledNxN(*this);
			  return(func);
		  };
	private:
		void setNull(){
			setType("ExampleVectorFunctionUncoupledNxN");
		};
		void setEqual(const ExampleVectorFunctionUncoupledNxN &aVectorFunction){};

		//--------------------------------------------------------------------------
		// OPERATORS
		//--------------------------------------------------------------------------
	public:
		ExampleVectorFunctionUncoupledNxN&
			operator=(const ExampleVectorFunctionUncoupledNxN &aVectorFunction){
				// BASE CLASS
				VectorFunctionUncoupledNxN::operator=(aVectorFunction);

				// DATA
				setEqual(aVectorFunction);

				return(*this);
		};

		//--------------------------------------------------------------------------
		// SET AND GET
		//--------------------------------------------------------------------------
	public:

		//--------------------------------------------------------------------------
		// EVALUATE
		//--------------------------------------------------------------------------
		virtual void calcValue(const double *aX,double *rY, int aSize){
			int N = getNX();

			// COMMON PART
			int i;
			double sum;
			double scale = 0.01;
			for(sum=0.0,i=0;i<N;i++) {
				sum += (double)i;
			}
			sum *= scale;

			// UNIQUE PART
			// Uncoupled-- each aY depends only on its corresponding aX.
			double root;
			for(i=0;i<N;i++) {
				root = scale * (double)i;
				// sin test function
				rY[i] = sum * sin(aX[i] - root);
				// parabolic test function
				//rY[i] = sum *aX[i]*aX[i]*aX[i] - sum*root*root*root; 
			}
		};
		virtual void calcValue(const Array<double> &aX,Array<double> &rY){
			calcValue(&aX[0],&rY[0], aX.getSize());
		};
		virtual void calcDerivative(const Array<double> &aX,Array<double> &rY,
			const Array<int> &aDerivWRT){
				std::cout<<"\nExampleVectorFunctionUncoupledNxN.evalute(x,y,derivWRT): not implemented.\n";
		};

		//=============================================================================
	};

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __ExampleVectorFunctionUncoupledNxN_h__
