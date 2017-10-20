// ----------------------------------------------------------------------------
// tropter: test_adolc_parallelized_openmp.cpp
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

// This test file exists to explore if it is possible to parallelize code
// in an active section. It seems that this is not possible; the
// parallelization should occur outside of the parallel section.

//#define CATCH_CONFIG_MAIN
//#include <catch.hpp>
//#include "testing.h"
//
//#include <omp.h>
//
//void do_something_in_parallel() {
//    int nthreads, tid;
//    #pragma omp parallel private(nthreads, tid)
//    {
//        tid = omp_get_thread_num();
//        printf("Hello world from thread = %d\n", tid);
//        if (tid == 0) {
//            nthreads = omp_get_num_threads();
//            printf("Number of threads = %d\n", nthreads);
//        }
//    }
//}
//
//TEST_CASE("Can use ADOL-C with OpenMP", "[adolc][parallel_openmp]") {
//    do_something_in_parallel();
//}

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <ctime>
#include <cmath>
#include <chrono>

#include <adolc/adolc.h>
#include <cassert>

#ifdef _OPENMP
#include <omp.h>
#include <adolc/adolc_openmp.h>
#endif

int main(){
//TEST_CASE("Can use ADOL-C with OpenMP TODO", "[adolc][parallel_openmp]") {
//    #pragma omp parallel ADOLC_OPENMP
//    {
//        short int tag = omp_get_thread_num();
//        trace_on(tag);
//        double px = 1.5;
//        adouble x;
//        double py;
//        x <<= px;
//        adouble y = x*x;
//        y >>= py;
//        trace_off();
//        double** J = myalloc(1, 1);
//        int success = jacobian(tag, 1, 1, &px, J);
//        assert(success == 3);
//        std::cout << "J: " << J[0][0] << std::endl;
//    }

//    #pragma omp parallel ADOLC_OPENMP
//    {
//        int N = 5;
//        short int tag = omp_get_thread_num();
//        trace_on(tag);
//
//        double* px = new double[N];
//        for (int i = 0; i < N; ++i) px[i] = 1.5;
//        adouble* x = new adouble[N];
//        double* py = new double[N];
//        adouble* y = new adouble[N];
//
//        for (int i = 0; i < N; ++i) x[i] <<= px[i];
//
//        for (int i = 0; i < N; ++i) {
//            y[i] = x[i] * x[i];
//        }
//
//        for (int i = 0; i < N; ++i) y[i] >>= py[i];
//
//        trace_off();
//
//        double** J = myalloc(N, N);
//        int success = jacobian(tag, N, N, px, J);
//        assert(success == 3);
//
//        #pragma omp barrier
//        for (int i = 0; i < N; ++i) {
//            std::cout << "J[" << i << "][" << i << "]: " << J[i][i]
//                    << std::endl;
//        }
//    }

    using Clock = std::chrono::high_resolution_clock;

    int N = 10000;
    double* x = new double[N];
    for (int i = 0; i < N; ++i) x[i] = 1.5 + i;
    double* py = new double[N];
    double* y = new double[N];

    auto t1 = Clock::now();
//    #pragma omp parallel
    {
//        #pragma omp for
        for (int i = 0; i < N; ++i) {
            y[i] = 0;
            for (int j = 0; j < 50; ++j) {
                y[i] += sin(x[i]*x[i]) + exp(x[i]) + log(sqrt(x[i] * x[i]));
            }
        }
    }

    auto t2 = Clock::now();
    std::cout << "Time: " <<
            std::chrono::duration_cast<std::chrono::microseconds>
            (t2-t1)
            .count() << std::endl;
//    for (int i = 0; i < N; ++i) std::cout << y[i] << " " << std::endl;

//    int N = 5;
//    short int tag = 0;
//    trace_on(tag);
//
//    double* px = new double[N];
//    for (int i = 0; i < N; ++i) px[i] = 1.5;
//    adouble* x = new adouble[N];
//    double* py = new double[N];
//    adouble* y = new adouble[N];
//
//    for (int i = 0; i < N; ++i) x[i] <<= px[i];
//
//    #pragma omp parallel ADOLC_OPENMP_NC
//    {
//        #pragma omp for
//        for (int i = 0; i < N; ++i) {
//            y[i] = x[i]*x[i];
//        }
//    }
//
//    for (int i = 0; i < N; ++i) y[i] >>= py[i];
//
//    trace_off();
//
//    double** J = myalloc(N, N);
//    int success = jacobian(tag, N, N, px, J);
//    assert(success == 3);
//
//    for (int i = 0; i < N; ++i) {
//        std::cout << "J[" << i << "][" << i << "]: " << J[i][i]
//                << std::endl;
//    }

////    #pragma omp parallel ADOLC_OPENMP
////    {
////        // different paths for each thread
////        int index = omp_get_thread_num();
//    short int index = 0;
//
//        double* x = new double[2];
//        x[0] = 0.7346;
//        x[1] = 3.0;
//        double y;
//        adouble* xa = new adouble[2];
//        adouble* ya = new adouble[1];
//        trace_on(index);
//
//        xa[0] <<= x[0];
//        xa[1] <<= x[1];
//        ya[0] = x[0] * x[0] + 2 * x[1];
//        ya[0] >>= y;
//
//        trace_off();
//
////        double* grad = new double[2];
////    std::vector<double> grad(2);
////
////        int status = gradient(index, 2, x, grad.data());
////    std::vector<double> func(2);
////        int stat = function(index, 1, 2, x, func.data());
////        printf("status = %d %d\n", status, stat);
////    std::cout << grad[0] << " " << grad[1] << std::endl;
////    std::cout << func[0] << std::endl;
////        printf("grad = %f\n", grad[1]);
////
//////#pragma omp barrier
////        //{
//            printf("y = %f\n", y);
//            printf("grad = %f\n", grad[0]);
//        //}
////    }


    return 0;
}

