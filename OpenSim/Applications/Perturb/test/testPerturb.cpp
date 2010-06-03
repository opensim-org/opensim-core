/*
* Copyright (c)  2009, Stanford University. All rights reserved. 
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
//
// author: Jack Middleton 

// INCLUDE
#include <string>
#include <iostream>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/LoadModel.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Tools/PerturbationTool.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/BodyKinematics.h>




using namespace OpenSim;
using namespace std;

const static int NUM_VALUES = 4; // check results from perturbing 3 muscles plus  gravity

int checkValues( const string &filename,  double epsilon) {
    // check results
    double diff;
    int failed = 0;
    std::string  label;

    Storage results("Results/"+filename);
    Storage standard(filename);

    // check the last state vector
    StateVector* resultsValues = results.getStateVector(results.getSize()-1);
    StateVector* standardValues = standard.getStateVector(standard.getSize()-1);

    for(int i=0;i<resultsValues->getSize();i++ ) {
        diff = resultsValues->getData()[i] - standardValues->getData()[i];
        label = results.getColumnLabels()[i+1];
//        cout << label << "  diff= " << diff << endl;
        if(  diff >  epsilon)  {
           cout << "\n" << filename << " : " << label << " FAILED standard= " << standardValues->getData()[i] << " result=" << resultsValues->getData()[i] << endl << endl;
           failed = 1;
        }
    }
    return(failed);
}

int testGait2354() {
	PerturbationTool perturb("setup_gait.xml");
	perturb.run();
    int failed = 0;

    failed += checkValues( "subject01_walk1_center_of_mass_X_deltaA_dt_0.030_df_1000.000.sto", 0.05 );
    failed += checkValues( "subject01_walk1_center_of_mass_Y_deltaA_dt_0.030_df_1000.000.sto", 0.05 );
    failed += checkValues( "subject01_walk1_center_of_mass_Z_deltaA_dt_0.030_df_1000.000.sto", 0.05 );

    failed += checkValues( "subject01_walk1_center_of_mass_X_dAdF_dt_0.030_df_1000.000.sto", 0.05 );
    failed += checkValues( "subject01_walk1_center_of_mass_Y_dAdF_dt_0.030_df_1000.000.sto", 0.05 );
    failed += checkValues( "subject01_walk1_center_of_mass_Z_dAdF_dt_0.030_df_1000.000.sto", 0.05 );

    failed += checkValues( "subject01_walk1_knee_angle_l_deltaA_dt_0.030_df_1000.000.sto", 5.0 );
    failed += checkValues( "subject01_walk1_knee_angle_l_deltaA_dt_0.030_df_1000.000.sto", 5.0 );
    failed += checkValues( "subject01_walk1_knee_angle_l_deltaA_dt_0.030_df_1000.000.sto", 5.0 );

    failed += checkValues( "subject01_walk1_knee_angle_r_dAdF_dt_0.030_df_1000.000.sto", 5.0 );
    failed += checkValues( "subject01_walk1_knee_angle_r_dAdF_dt_0.030_df_1000.000.sto", 5.0 );
    failed += checkValues( "subject01_walk1_knee_angle_r_dAdF_dt_0.030_df_1000.000.sto", 5.0 );

    failed += checkValues( "subject01_walk1_unperturbedAccel_dt_0.030_df_1000.000.sto", 1000 );

    return(failed); 

}

int main() {
    int status;
    try {


		status  = testGait2354();    //finally include applied ground reactions forces 
        if( status == 0 ) std::cout << "gait2354 test PASSED " << std::endl;
    }
//    catch(const std::exception& e) {
    catch(Exception& e) {
//        cout << "exception: " << e.what() << endl;
        e.print(cout);
        return 1;
    }
    cout << "Done" << endl;
    return status;
}

