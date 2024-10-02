#include <cstdio>
#include <OpenSim/Common/PolynomialFunction.h>
#include <OpenSim/Actuators/DeGrooteFregly2016Muscle.h>
#include <OpenSim/Common/Sine.h>

using namespace OpenSim;

extern int enzyme_dup;
extern int enzyme_dupnoneed;
extern int enzyme_out;
extern int enzyme_const;

template < typename return_type, typename ... T >
return_type __enzyme_fwddiff(void*, T ... );

template < typename return_type, typename ... T >
return_type __enzyme_autodiff(void*, T ... );

// Does not compile.
#if false

template < typename T, typename ... arg_types >
auto wrapperMuscle(T obj, arg_types && ... args) {
    return obj.calcTendonForceMultiplier(args ... );
}

void testDeGrooteFreglyMuscle() {
    DeGrooteFregly2016Muscle muscle;
    muscle.finalizeFromProperties();

    SimTK::Real y = 1.1;
    SimTK::Real dy = 1.0;
    SimTK::Real f = muscle.calcTendonForceMultiplier(y);
    SimTK::Real dfdy = muscle.calcTendonForceMultiplierDerivative(y);
    printf("Muscle analytic derivative\n");
    printf("f(y) = %f, f'(y) = %f\n\n", f, dfdy);

    dfdy = __enzyme_fwddiff<SimTK::Real>(
            (void*)wrapperMuscle<DeGrooteFregly2016Muscle, SimTK::Real>, 
            enzyme_const, muscle, 
            enzyme_dup, &y, &dy);
    printf("Muscle derivative w/ Enzyme\n");
    printf("f(y) = %f, f'(y) = %f\n\n", wrapperMuscle(muscle, y), dfdy);
}

#endif

// Does not compile.
#if false

template < typename T, typename ... arg_types >
auto wrapperSine(T obj, arg_types && ... args) {
    return obj.calcValue(args ... );
}

void testSineFunction() {
    Sine func;

    SimTK::Vector y(1, 1.27);
    SimTK::Real dy = 1.0;
    SimTK::Real f = func.calcValue(y);
    SimTK::Real dfdy = func.calcDerivative({0}, y);
    printf("Sine analytic derivative\n");
    printf("f(y) = %f, f'(y) = %f\n\n", f, dfdy);

    dfdy = __enzyme_fwddiff<SimTK::Real>(
            (void*)wrapperSine<Sine, SimTK::Vector>, 
            enzyme_const, func, 
            enzyme_dup, &y, &dy);
    printf("Sine derivative w/ Enzyme\n");
    printf("f(y) = %f, f'(y) = %f\n\n", wrapperSine(func, y), dfdy);
}

#endif

#if true

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

#endif

// Compiles!
#if true

class PolynomialFunctionStdVector {
public:
    PolynomialFunctionStdVector(const std::vector<double>& coefficients) {
        _coefficients = coefficients;
    }

    double calcValue(const std::vector<double>& x) {
        double result = 0;
        for (int i = 0; i < static_cast<int>(_coefficients.size()); i++) {
            result += _coefficients[i] * pow(x[0], _coefficients.size() - i - 1);
        }
        return result;
    }

private:
    std::vector<double> _coefficients;
};

template < typename T, typename ... arg_types >
auto wrapperPolynomialStdVector(T obj, arg_types && ... args) {
    return obj.calcValue(args ... );
}

void testPolynomialFunctionStdVector() {
    // f = x^2
    std::vector<double> coefficients = {1, 0, 0};
    PolynomialFunctionStdVector func(coefficients);

    std::vector<double> y = {5.0};
    std::vector<double> dy = {1.0};
    double dfdy = __enzyme_fwddiff<double>(
            (void*)wrapperPolynomialStdVector<PolynomialFunctionStdVector, std::vector<double>>, 
            enzyme_const, func, 
            enzyme_dup, &y, &dy);
    printf("PolynomialFunctionStdVector\n");
    printf("f(y) = %f, f'(y) = %f\n\n", wrapperPolynomialStdVector(func, y), dfdy);
}

#endif

// This does not compile.
// Fails when calling SimTK::MatrixHelper::getElt().
#if false

class PolynomialFunctionSimTKVector {
public:
    PolynomialFunctionSimTKVector(const SimTK::Vector& coefficients) {
        _coefficients = coefficients;
    }

    double calcValue(const SimTK::Vector& x) {
        double result = 0;
        for (int i = 0; i < static_cast<int>(_coefficients.size()); i++) {
            result += _coefficients[i] * pow(x[0], _coefficients.size() - i - 1);
        }
        return result;
    }

private:
    SimTK::Vector _coefficients;
};

template < typename T, typename ... arg_types >
auto wrapperPolynomialSimTKVector(T obj, arg_types && ... args) {
    return obj.calcValue(args ... );
}

void testPolynomialFunctionSimTKVector() {
    // f = x^2
    SimTK::Vector coefficients(3);
    coefficients[0] = 1;
    coefficients[1] = 0;
    coefficients[2] = 0;
    PolynomialFunctionSimTKVector func(coefficients);

    SimTK::Vector y(1, 5.0);
    SimTK::Vector dy(1, 1.0);
    double dfdy = __enzyme_fwddiff<double>(
            (void*)wrapperPolynomialSimTKVector<PolynomialFunctionSimTKVector, SimTK::Vector>, 
            enzyme_const, func, 
            enzyme_dup, &y, &dy);
    printf("PolynomialFunctionSimTKVector\n");
    printf("f(y) = %f, f'(y) = %f\n\n", wrapperPolynomialSimTKVector(func, y), dfdy);
}

#endif

int main() {
    // testDeGrooteFreglyMuscle();
    // testSineFunction();
    testPolynomialFunction();
    testPolynomialFunctionStdVector();
    // testPolynomialFunctionSimTKVector();
    return 0;
}