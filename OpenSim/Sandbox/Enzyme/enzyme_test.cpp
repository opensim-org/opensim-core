#include <cstdio>

extern int enzyme_dup;
extern int enzyme_dupnoneed;
extern int enzyme_out;
extern int enzyme_const;

template < typename return_type, typename ... T >
return_type __enzyme_fwddiff(void*, T ... );

template < typename return_type, typename ... T >
return_type __enzyme_autodiff(void*, T ... );

double f(double x) { return x * x; }

int main() {
    double x = 5.0;
    double dx = 1.0;
    double df_dx = __enzyme_fwddiff<double>((void*)f, enzyme_dup, x, dx); 
    printf("f(x) = %f, f'(x) = %f", f(x), df_dx);
}