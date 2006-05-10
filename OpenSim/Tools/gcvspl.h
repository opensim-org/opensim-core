
#ifdef __cplusplus
extern "C" {
#endif

/*
	constants
*/
const double zero = 0.0 ;
const double half = 0.5 ;
const double one  = 1.0 ;
const double two  = 2.0 ;

/*
	function prototypes
*/
void   basis(int m, int n, double *x, double *b, double *bl, double *q) ;
void   prep(int m, int n, double *x, double *w, double *we, double *el) ;
void   bansol(double *e, double *y, double *c, int m, int n) ;
void   bandet(double *e, int m, int n) ;
void   search(int n, double *x, double t, int *l) ;
void   gcvspl(double *x, double  *y, double *w, int m, int n,
				  double *c, double var, double *wk, int ier) ;
double splc(int m, int n, double *y, double *w, double var, double p,
				double eps, double *c, double *stat, double *b, double *we,
				double el, double *bwe) ;
double splder(int ider, int m, int n, double t, double *x,
					double *c, int *l, double *q) ;
double trinv(double *b, double *e, int m, int n) ;

#ifdef __cplusplus
}
#endif
