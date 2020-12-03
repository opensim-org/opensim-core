/* -------------------------------------------------------------------------- *
 * OpenSim: sandboxReplaceWrappingMath.cpp                                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Wrap/WrapMath.h>
#include <OpenSim/Common/Mtx.h>
#include <SimTKcommon/Testing.h>

#include <ctime>
#include <stack>

using namespace OpenSim;

// https://stackoverflow.com/questions/13485266/how-to-have-matlab-tic-toc-in-c
std::stack<clock_t> tictoc_stack;
void tic() {
    tictoc_stack.push(clock());
}

double toc() {
    double elapsed = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
    //log_cout("Time elapsed: {:03.9f} seconds", elapsed);
    tictoc_stack.pop();
    return elapsed;
}

void fillMat33(SimTK::Mat33& mat, double matc[][3]) {
    SimTK::Random::Uniform rand;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double val = rand.getValue();
            mat[i][j] = val;
            matc[i][j] = val;
        }
    }
}

void compareMat33(const SimTK::Mat33& b, const double bc[][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            //log_cout("({}, {}): b = {}, bc = {}", i, j, b[i][j], bc[i][j]);
            SimTK_TEST_EQ(b[i][j], bc[i][j]);
        }
    }
}

void compareMat44(const SimTK::Mat44& b, const double bc[][4]) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            //log_cout("({}, {}): b = {}, bc = {}", i, j, b[i][j], bc[i][j]);
            // compareInvert() results don't pass SimTK::Test::numericallyEqual() without this tolerance.
            SimTK_TEST_EQ_TOL(b[i][j], bc[i][j], 1e-10);
        }
    }
}

void fillMat44(SimTK::Mat44& mat, double matc[][4]) {
    SimTK::Random::Uniform rand;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double val = rand.getValue();
            mat[i][j] = val;
            matc[i][j] = val;
        }
    }
}

void fillVec4(SimTK::Vec4& vec, double vecc[4]) {
    SimTK::Random::Uniform rand;
    for (int i = 0; i < 4; ++i) {
        double val = rand.getValue();
        vec[i] = val;
        vecc[i] = val;
    }
}

void get33From44(const SimTK::Mat44& mat44, SimTK::Mat33& mat33) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat33[i][j] = mat44[i][j];
        }
    }
}

void get33From44(const double mat44[][4], double mat33[][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat33[i][j] = mat44[i][j];
        }
    }
}

void get3From4(const SimTK::Vec4& vec4, SimTK::Vec3& vec3) {
    for (int i = 0; i < 3; ++i) {
        vec3[i] = vec4[i];
    }
}

void get3From4(const double vec4[4], double vec3[3]) {
    for (int i = 0; i < 3; ++i) {
        vec3[i] = vec4[i];
    }
}

void compareOneAxisRotation(double angle, double& t, double& tc) {

    SimTK::Mat33 mat, b;
    double Rxc[3][3], matc[3][3], bc[3][3];
    fillMat33(mat, matc);

    SimTK::Rotation Rx;
    Rx.setRotationFromAngleAboutX(angle);
    tic();
    b = mat * ~Rx;
    t = toc();

    WrapMath::Make3x3DirCosMatrix(angle, Rxc);
    tic();
    Mtx::Multiply(3, 3, 3, (double*)matc, (double*)Rxc, (double*)bc);
    tc = toc();

    // Need to transpose the SimTK rotation matrix to match rotation convention.
    compareMat33(~Rx, Rxc);
    compareMat33(b, bc);
}

void compareAngleAxisRotation(double angle, const SimTK::Vec3 axis, double& t, double& tc) {
    SimTK::Mat44 x_temp;
    SimTK::Mat33 x, b;
    double bc_temp[4][4];
    double bc[3][3];
    fillMat44(x_temp, bc_temp);
    get33From44(x_temp, x);

    tic();
    SimTK::Rotation R;
    R.setRotationFromAngleAboutNonUnitVector(angle, axis);
    b = x * ~R; // This multiplication matches the convention of WrapMath::RotateMatrixAxisAngle.
    t = toc();

    tic();
    WrapMath::RotateMatrixAxisAngle(bc_temp, axis, angle);
    get33From44(bc_temp, bc);
    tc = toc();

    compareMat33(b, bc);
}

void compareAngleAxis4x4Rotation(double angle, const SimTK::Vec3& axis, double& t, double& tc) {
    double mat[4][4];
    SimTK::Vec4 x_temp;
    SimTK::Vec3 x, b;
    double xc[4], bc[4];
    fillVec4(x_temp, xc);
    get3From4(x_temp, x);

    tic();
    SimTK::Rotation R;
    R.setRotationFromAngleAboutNonUnitVector(angle, axis);
    b = ~R * x;
    t = toc();

    tic();
    WrapMath::ConvertAxisAngleTo4x4DirCosMatrix(axis, angle, mat);
    Mtx::Multiply(4, 4, 1, (double*)mat, (double*)xc, (double*)bc);
    tc = toc();

    for (int i = 0; i < 3; ++i) {
        //log_cout("({}): b = {}, bc = {}", i, b[i], bc[i]);
        SimTK_TEST_EQ(b[0], bc[0]);
    }

}

void compareInvert(double& t, double& tc) {
    SimTK::Mat44 M, Minv;
    double Mc[4][4], Mcinv[4][4];

    fillMat44(M, Mc);

    tic();
    Minv = M.invert();
    t = toc();

    tic();
    Mtx::Invert(4, &Mc[0][0], &Mcinv[0][0]);
    tc = toc();

    compareMat44(Minv, Mcinv);
}

void compareReport(const SimTK::Vector& T, const SimTK::Vector& Tc) {
    double avg_t = T.sum() / T.size();
    double avg_tc = Tc.sum() / Tc.size();
    double ratio = avg_t / avg_tc;
    log_cout("Simbody: {} s, WrapMath: = {} s, ratio: {} \n", avg_t, avg_tc, ratio);
}

int main() {

    double angle = 0.1;
    SimTK::Vec3 axis(1.0, 2.0, 3.0);
    int N = 10000;
    SimTK::Vector T(N);
    SimTK::Vector Tc(N);
    double t;
    double tc;

    log_cout("compareOneAxisRotation");
    log_cout("----------------------");
    for (int i = 0; i < N; ++i) {
        compareOneAxisRotation(angle, t, tc);
        T[i] = t;
        Tc[i] = tc;
    }
    compareReport(T, Tc);

    log_cout("compareAngleAxisRotation");
    log_cout("------------------------");
    for (int i = 0; i < N; ++i) {
        compareAngleAxisRotation(angle, axis, t, tc);
        T[i] = t;
        Tc[i] = tc;
    }
    compareReport(T, Tc);

    log_cout("compareAngleAxis4x4Rotation");
    log_cout("---------------------------");
    for (int i = 0; i < N; ++i) {
        compareAngleAxis4x4Rotation(angle, axis, t, tc);
        T[i] = t;
        Tc[i] = tc;
    }
    compareReport(T, Tc);

    log_cout("compareInvert");
    log_cout("-------------");
    for (int i = 0; i < N; ++i) {
        compareInvert(t, tc);
        T[i] = t;
        Tc[i] = tc;
    }
    compareReport(T, Tc);
}

