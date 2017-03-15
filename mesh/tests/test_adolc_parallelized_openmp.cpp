#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include "testing.h"

#include <omp.h>

void do_something_in_parallel() {
    int nthreads, tid;
    #pragma omp parallel private(nthreads, tid)
    {
        tid = omp_get_thread_num();
        printf("Hello world from thread = %d\n", tid);
        if (tid == 0) {
            nthreads = omp_get_num_threads();
            printf("Number of threads = %d\n", nthreads);
        }
    }
}

TEST_CASE("Can use ADOL-C with OpenMP", "[adolc][parallel_openmp]") {

    do_something_in_parallel();
}
