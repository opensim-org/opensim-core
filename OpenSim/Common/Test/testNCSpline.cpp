/*BicubicSurface.cpp
* Author: Matthew Millard
* Copyright (c)  2011, Stanford University, All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
*   1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
*   2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
*   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
*   4. Credits to developers may not be removed from executables
*     created from modifications of the source.
*   5. Modifications of source code must retain the above copyright notice, this list of
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
//C:\mjhmilla\Stanford\dev\SimBody\SimTKcommon\include\SimTKcommon\Testing.h
#include <iostream>
#include <fstream>
#include <OpenSim/Common/Exception.h>
//#include <OpenSim/Common/CubicSpline.h>
#include <OpenSim/Common/NaturalCubicSpline.h>
#include <stdio.h>
#include <SimTKcommon/Testing.h>

using namespace SimTK;
using namespace OpenSim;
using namespace std;

/**
* This function computes a standard central difference dy/dx. If extrap_endpoints is set to 1, then
* the derivative at the end points is estimated by linearly extrapolating the dy/dx values beside the 
* end points
*
* @param x domain vector
* @param y range vector
& @param extrap_endpoints: (false)	Endpoints of the returned vector will be zero, because a central difference
*									is undefined at these endpoints
*							(true)	Endpoints are computed by linearly extrapolating using a first difference from 
*									the neighboring 2 points
* @returns dy/dx computed using central differences
*/
SimTK::Vector getCentralDifference(SimTK::Vector x, SimTK::Vector y, bool extrap_endpoints){
	SimTK::Vector dy(x.size());
	double dx1,dx2;
	double dy1,dy2;
	int size = x.size();
	for(int i=1; i<x.size()-1; i++){
		dx1 = x(i)-x(i-1);
		dx2 = x(i+1)-x(i);
		dy1 = y(i)-y(i-1);
		dy2 = y(i+1)-y(i);
		dy(i)= 0.5*dy1/dx1 + 0.5*dy2/dx2;
	}

	if(extrap_endpoints == true){
		dy1 = dy(2)-dy(1);
		dx1 = x(2)-x(1);
		dy(0) = dy(1) + (dy1/dx1)*(x(0)-x(1));

		dy2 = dy(size-2)-dy(size-3);
		dx2 = x(size-2)-x(size-3);
		dy(size-1) = dy(size-2) + (dy2/dx2)*(x(size-1)-x(size-2));
	}
	return dy;
}

/**
* This function will print cvs file of the column vector col0 and the matrix data
*
* @params col0: A vector that must have the same number of rows as the data matrix
*				This column vector is printed as the first column
* @params data: A matrix of data
* @params filename: The name of the file to print
*/
void printMatrixToFile(SimTK::Vector col0,SimTK::Matrix data, string filename){
	ofstream datafile;
	datafile.open(filename);

	for(int i = 0; i < data.nrow(); i++){
		datafile << col0(i) << ",";
		for(int j = 0; j < data.ncol(); j++){
			if(j<data.ncol()-1)
				datafile << data(i,j) << ",";
			else
				datafile << data(i,j) << "\n";
		}	
	}
	datafile.close();
} 

/**
* This function will compute the value and first two derivatives of 
* an analytic function at the point x. 
*
* @params x			the input value to compute values at
* @params fcnType	the function to compute. There are currently 5 choices (see below)
* @returns SimTK::Vector: a 3x1 vector of the value, first derivative and second derivative
*/
SimTK::Vector getAnalyticFunction(double x,int fcnType){
	SimTK::Vector fdF(3);
	fdF = -1;

	switch(fcnType){
		case 0:	//f(x) = 0;
			fdF = 0;
			break;
		case 1: //f(x) = 2*x
			fdF(0) = 2*x;	//f
			fdF(1) = 2;		//fx
			fdF(2) = 0;
			break;
		case 2: //f(x) = x^2
			fdF(0) = x*x;		//f
			fdF(1) = 2*x;		//fx
			fdF(2) = 2;
			break;
		case 3:	//f(x) = 2*x + x*x;
			//f
			fdF(0) = 2*x + x*x;
			fdF(1) = 2 + 2*x;
			fdF(2) = 2;
			break;
		case 4: //f(x)  =2*x + x*x + 5*x*x*x
			fdF(0) = 2*x + x*x + 5*x*x*x;
			fdF(1) = 2 + 2*x + 15*x*x;
			fdF(2) = 2 + 30*x;
			break;
		case 5: //fx(x) = sin(x)
			fdF(0) = sin(x);
			fdF(1) = cos(x);
			fdF(2) = -sin(x);
			break;
		default:
			cout << "Invalid fcnType in testBicubicSurface.cpp: getAnayticFunction";
	}


	return fdF;
}
/**
* This function tests the accuracy of the natural cubic spline sp. The accuracy of the
* spline is tested in the following manner:
*
*	a.	Spline must pass through the knots given
*		-Error between spline and input data at the knots (should be zero)
*	b.	The first derivatives are continuous at the knot points
*		-Error between the value of the first derivative at the knot point, and
*		 what a linear extrapolation would predict just to the left and right of 
*		 the knot point. (should be zero, within a tolerace affected by the step size in xD)
*	c.	The second derivatives are continuous at the knots points
*		-Error between the value of the numerically calculated derivative at the knot point, and
*		 what a linear extrapolation would predict just to the left and right of 
*		 the knot point. (should be zero, within a tolerace affected by the step size in xD)
*	d.	The second derivative is zero at the end points.
*		-Numerically calculated extrapolation of the 2nd derivative should be zero 
*		 at the end points within some tolerance
*
*
*
*
*/
SimTK::Vector testNaturalCubicSpline(SimTK::Function* sp, SimTK::Vector xK, SimTK::Vector yK, SimTK::Vector xM,SimTK::Vector xD,string name, bool print){
	int size = xK.size();
	int sizeD= xD.size();
	int sizeDK = xD.size()/(xK.size()-1);
	double deltaD = (xK(xK.size()-1)-xK(0))/xD.size();

	SimTK::Matrix ysp_K(size,2),ysp_M(size-1,2),ysp_D(sizeD,4);
	SimTK::Vector errVec(4);
	errVec = 1;
	ysp_K = 0;
	ysp_M = 0;
	ysp_D = 0;

	vector<int> derOrder(1);
	derOrder[0] = 0;
	
	

	///////////////////////////////////////////
	//1. Evaluate the spline at the knots, the mid points and then a dense sample
	///////////////////////////////////////////
		SimTK::Vector tmpV1(1);
		double xVal=0;
		for(int i=0;i<size;i++){
			xVal = xK(i);
			tmpV1(0)=xK(i);
			ysp_K(i,0) = sp->calcValue(tmpV1);
			ysp_K(i,1) = sp->calcDerivative(derOrder,tmpV1);
		}			
		for(int i=0;i<size-1;i++){
			xVal = xM(i);
			tmpV1(0) = xM(i);
			ysp_M(i,0) = sp->calcValue(tmpV1);
			ysp_M(i,1) = sp->calcDerivative(derOrder,tmpV1);
		}
		for(int i=0;i<sizeD;i++){
			xVal = xD(i);
			tmpV1(0) = xD(i);
			ysp_D(i,0) = sp->calcValue(tmpV1);
			ysp_D(i,1) = sp->calcDerivative(derOrder,tmpV1);
		}

	//////////////////////////////////////
	//2.	Compute the second derivative of the spline (using central differences), and linearly 
	//		interpolate to get the end points. The end points should go to exactly zero because the 
	//		second derivative is linear in a cubic spline, as is the linear extrapolation
	//
	//		Also compute the 3rd derivative using the same method. The 3rd derivative is required in the
	//		test to determine if the second derivative is continuous at the knots or not.
	//////////////////////////////////////

		ysp_D(2)	= getCentralDifference(xD, ysp_D(1),	true);
		ysp_D(3)	= getCentralDifference(xD, ysp_D(2),	true);

	//////////////////////////////////////
	//3. Now check to see if the splines meet the conditions of a natural cubic spline:
	//////////////////////////////////////

		SimTK::Vector tmpK(size,size),tmpM(size-1,size-1);

		//* a.	Spline passes through all knot points given
					tmpK = yK-ysp_K(0);
				errVec(0) = tmpK.norm();
			
		//	b. The first derivative is continuous at the knot points. Apply a continuity test
		//		to the data points that define the second derivative
		//
		//		Continuity test:	a linear extrapolation of first derivative of the curve in interest
		//							on either side of the point in interest should equal the point in interest;

			double ykL,ykR,y0L,dydxL,y0R,dydxR = 0;
			for(int i=1; i<size-1; i++){
					y0L = ysp_D(i*sizeDK-1,1);
					y0R = ysp_D(i*sizeDK+1,1);
					dydxL = ysp_D(i*sizeDK-1,2); //Found using central differences
					dydxR = ysp_D(i*sizeDK+1,2); //Found using central differences
					ykL = y0L + dydxL*deltaD;
					ykR = y0R - dydxR*deltaD;
				errVec(1) = (ysp_D(i*sizeDK,1)-ykL)+(ysp_D(i*sizeDK,1)-ykR);
			}
			
			
		//	c. The second derivative is continuous at the knot points. Apply a continuity test
		//		to the data points that define the second derivative. This also tests if the 
		//		first derivative is smooth.
		//
		//		Continuity test:	a linear extrapolation of first derivative of the curve in interest
		//							on either side of the point in interest should equal the point in interest;
			for(int i=1; i<size-1; i++){
				y0L = ysp_D(i*sizeDK-1,2);
				y0R = ysp_D(i*sizeDK+1,2);
				dydxL = ysp_D(i*sizeDK-1,3); //Found using central differences
				dydxR = ysp_D(i*sizeDK+1,3); //Found using central differences
				ykL = y0L + dydxL*deltaD;
				ykR = y0R - dydxR*deltaD;
				errVec(2) = (ysp_D(i*sizeDK,2)-ykL)+(ysp_D(i*sizeDK,2)-ykR);
			}	

		//////////////////////////////////////
		//* d.	The second derivative is zero at the end points
		//////////////////////////////////////

		errVec(3) = abs(ysp_D(0,2)) + abs(ysp_D(sizeD-1,2));

		

		//////////////////////////////////////
		//print the data for analysis
		//////////////////////////////////////

		if(print==true){
			string fname = name;
			fname.append("_K.csv");
			printMatrixToFile(xK,	ysp_K,	fname);

			fname = name;
			fname.append("_M.csv");
			printMatrixToFile(xM,	ysp_M,	fname);

			fname = name;
			fname.append("_D.csv");
			printMatrixToFile(xD,	ysp_D,	fname);
		}

		return errVec;

}

/**
* The test works by seeing if the tested splines have the properties of a natural cubic spline.
* To do so, this test file has several steps
*
* User Steps: Configure the script
*		a. Choose the function to be interpolated
*		b. Choose the location and number of knot points
*		c. Choose the density of a high resolution interpolation
*
* Test Script Steps:
* 0. Initialize the input vectors xK, xM, and xD for the knot locations, 
*    mid knot location and high resolution step locations respectively
* 1. Initialize the analytically computed output yK, yM and yD
* 2. Create each of the spline objects. 
* 3. Evaluate the numerical accuracy of the splines by calling testNaturalCubicSpline
*/
int main() {
    try {
	/////////////////////////////
	//Configuration Variables
	////////////////////////////
		bool printData = false;		//Set to true to print the knot, mid knot, and 
									//dense vector values, first derivatives, and 
									//second derivatives (for the splines) for analysis
									//outside of this script.
		int fcnType = 5;	//Chooses what kind of analytical test function to use
							//to initialize and test the various spline classes
        const int size =6;			//Number of knot points
		const int sizeDK = 10000;		//Number of points per knot in the densely sampled vector
		int sizeD=sizeDK*(size-1);	//Number of points in a densly sampled interpolation

		//Domain vector variables
		double xmin,xmax,deltaX,deltaD;
		xmin = Pi/4;			//Value of first knot
		xmax = Pi/2;			//Value of the final knot

		deltaX = (xmax-xmin)/(size-1);	
		deltaD = (xmax-xmin)/(sizeD-1);

	/////////////////////////////
	//Test Code body
	////////////////////////////

		SimTK::Matrix testResults(4,2); //This matrix stores the results of the 5 tests
										//in each row entry, for each of the 2 spline classes
										//tested.
										// SimTK SplineFitter results are stored in column 0
										// OpenSim::NaturalCubicSpline results are stored in column 1
		//testResults.elementwiseAssign(0.0);
		testResults = -1;
		SimTK::Vector tmpV1(1);

		//Generate initialization knot points (denoted by a 'K') 
		//		and the mid points (denoted by a 'M')		
		//		and for the densely sampled interpolation vector (denoted by a 'D')
        SimTK::Vector xK(size), xM(size-1), xD(sizeD);
		SimTK::Matrix yK(size,3), yM(size-1,3), yD(sizeD,3);



	///////////////////////////////////////////
	//0. Initialize the input vectors xK, xM and xD
	///////////////////////////////////////////
        for (int i = 0; i < size; i++) {
            xK(i) = xmin + ((double)i)*deltaX;
        	if(i<size-1){
				xM(i) = xmin + deltaX/(double)2 + ((double)i)*deltaX;
			}
        }
		for(int i = 0; i < sizeD; i++)
			xD(i) = xmin + deltaD*(double)i;

	///////////////////////////////////////////		
	//1.	Initialize the analytic function vector data to interpolate
	//		Let the user know which function is being used
	///////////////////////////////////////////

		switch(fcnType){
			case 0:
				cout << "f(x) = 0" <<endl;
				break;
			case 1:
				cout << "f(x) = 2*x" <<endl;
				break;
			case 2:
				cout << "f(x) = x^2" <<endl;
				break;
			case 3:
				cout << "f(x) = 2*x + x^2 " <<endl;
				break;
			case 4:
				cout << "f(x) = 2*x + x^2 + 5x^3 " <<endl;
				break;
		}


		//Get the function values at the knot points
		SimTK::Vector tmp(3);
		tmp = 0;
		for(int i=0; i<size;i++){
			tmp = getAnalyticFunction(xK(i),fcnType);
			for(int k=0;k<3;k++)
				yK(i,k) = tmp(k);

		}
		SimTK:: Vector yKVal = yK(0);

		//Get the function y, dy, ddy at the mid points
		for(int i=0; i<size-1;i++){
			tmp = getAnalyticFunction(xM(i),fcnType);
			for(int k=0;k<3;k++)
				yM(i,k) = tmp(k);
		}
		//Get the function y, dy, ddy at the dense points
		for(int i=0; i<sizeD;i++){
			tmp = getAnalyticFunction(xD(i),fcnType);
			for(int k=0;k<3;k++)
				yD(i,k) = tmp(k);
		}

	///////////////////////////////////////////
	//2. Create each of the splines
	///////////////////////////////////////////

		//OpenMM:CubicSplines
		//SimTK::Vector csDerivs1(xK.size());
		//SimTK::Vector csDerivs2(xK.size());
		//CubicSpline::createNaturalSpline(xK,yKVal, csDerivs1);
		//CubicSpline::createNaturalSpline(xK,csDerivs1, csDerivs2);

		//OpenSim:NaturalCubicSplines
		SimTK::Vector ncsDerivs1(xK.size());
		vector<int> derOrder(1);
		derOrder[0] = 0;		
		NaturalCubicSpline ncs(xK.size(), &xK[0], &yKVal[0],"test");					
		SimTK::Function* ncs_simtkfcn = ncs.createSimTKFunction();

		//SimTK::SplineFitter
		SimTK::Vector sfDerivs1(xK.size());
		SimTK::Spline_<Real> sTK = SimTK::SplineFitter<Real>::fitForSmoothingParameter(3,xK,yKVal,0.0).getSpline();
			
	///////////////////////////////////////////
	//3. Test the splines
	///////////////////////////////////////////

		testResults(0) = testNaturalCubicSpline(&sTK, xK, yK(0), xM, xD, "simtk_splinefitter",true);
		testResults(1) = testNaturalCubicSpline(ncs_simtkfcn, xK, yK(0), xM, xD,"opensim_natcubspline",true);

		cout << "Test Result Matrix: 0 or small numbers pass" <<endl;
		cout << "  column 0: SimTK::SplineFitter, column 1: OpenSim::NaturalCubicSpline" <<endl;
		cout << "  row 0: Passes through knots (tol 1e-14)" << endl;
		cout << "  row 1: First derivative is continuous and smooth (tol " << deltaD << ")" << endl;
		cout << "  row 2: Second derivative is continuous (tol " << 10*deltaD << ")" << endl;
		cout << "  row 3: Second derivative is zero at endpoints (tol "<< deltaD/10 <<")" << endl;
		cout << testResults << endl;

		//////////////////////////////////////
		//6. Run numerical assertions on each test
		//////////////////////////////////////

			double tol = 0;
			SimTK_START_TEST("Testing natural cubic splines");
			
			for(int k=0;k<testResults.ncol();k++){
				for(int i=0;i<testResults.nrow();i++){					
					switch(i){
						case 0: //Equal at knots
							tol = 1e-14;
							break;
						case 1: //Continuous 1st derivative
							tol = deltaD;
							break;
						case 2: //Continuous 2nd derivative
							tol = 10*deltaD;
							break;	
						case 3: //2nd derivative zero at end points
							tol = deltaD/10;
							break;
						default:
							cout << "testNCSpline: Invalid error type selected" << endl;
					}
					cout << "Testing (i,k) " << i << " " << k << " tol " << tol << " \tval " << testResults(i,k) << endl;
					SimTK_TEST_EQ_TOL(testResults(i,k),0,tol);
				}
			}
			SimTK_END_TEST();


	}catch(const OpenSim::Exception& e) {
		e.print(cerr);
		getchar();
		return 1;
	}

		
    cout << "Done. Press any key to return" << endl;
	getchar();
    return 0;
}

