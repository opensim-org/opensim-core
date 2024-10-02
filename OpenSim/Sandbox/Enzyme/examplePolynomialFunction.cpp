#include <cstdio>
#include <OpenSim/Common/PolynomialFunction.h>

using namespace OpenSim;

extern int enzyme_dup;
extern int enzyme_dupnoneed;
extern int enzyme_out;
extern int enzyme_const;

template < typename return_type, typename ... T >
return_type __enzyme_fwddiff(void*, T ... );

template < typename return_type, typename ... T >
return_type __enzyme_autodiff(void*, T ... );

template < typename T, typename ... arg_types >
auto wrapperPolynomial(T obj, arg_types && ... args) {
    return obj.calcValue(args ... );
}

void testPolynomialFunction() {
    // f = x^2
    SimTK::Vector coefficients(3);
    coefficients[0] = 1;
    coefficients[1] = 0;
    coefficients[2] = 0;
    PolynomialFunction func(coefficients);

    SimTK::Vector y(1, 5.0);
    SimTK::Vector dy(1, 1.0);
    SimTK::Real dfdy = __enzyme_fwddiff<SimTK::Real>(
            (void*)wrapperPolynomial<PolynomialFunction, SimTK::Vector>, 
            enzyme_const, func, 
            enzyme_dup, &y, &dy);
    printf("PolynomialFunction\n");
    printf("f(y) = %f, f'(y) = %f\n\n", wrapperPolynomial(func, y), dfdy);
}

int main() {
    testPolynomialFunction();
    return EXIT_SUCCESS;
}