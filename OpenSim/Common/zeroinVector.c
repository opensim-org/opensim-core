/*
 ************************************************************************
 *	    		    C math library
 * function ZEROIN - obtain a function zero within the given range
 *
 * Input
 *	double zeroin(ax,bx,f,tol)
 *	double ax; 			Root will be seeked for within
 *	double bx;  			a range [ax,bx]
 *	double (*f)(double x);		Name of the function whose zero
 *					will be seeked for
 *	double tol;			Acceptable tolerance for the root
 *					value.
 *					May be specified as 0.0 to cause
 *					the program to find the root as
 *					accurate as possible
 *
 * Output
 *	Zeroin returns an estimate for the root with accuracy
 *	4*EPSILON*abs(x) + tol
 *
 * Algorithm
 *	G.Forsythe, M.Malcolm, C.Moler, Computer methods for mathematical
 *	computations. M., Mir, 1980, p.180 of the Russian edition
 *
 *	The function makes use of the bissection procedure combined with
 *	the linear or quadric inverse interpolation.
 *	At every step program operates on three abscissae - a, b, and c.
 *	b - the last and the best approximation to the root
 *	a - the last but one approximation
 *	c - the last but one or even earlier approximation than a that
 *		1) |f(b)| <= |f(c)|
 *		2) f(b) and f(c) have opposite signs, i.e. b and c confine
 *		   the root
 *	At every step Zeroin selects one of the two new approximations, the
 *	former being obtained by the bissection procedure and the latter
 *	resulting in the interpolation (if a,b, and c are all different
 *	the quadric interpolation is utilized, otherwise the linear one).
 *	If the latter (i.e. obtained by the interpolation) point is 
 *	reasonable (i.e. lies within the current interval [b,c] not being
 *	too close to the boundaries) it is accepted. The bissection result
 *	is used in the other case. Therefore, the range of uncertainty is
 *	ensured to be reduced at least by the factor 1.6
 *
 ************************************************************************
 */

#include <math.h>
#include <float.h>

double solve(rdArray<double> ax,rdArray<double> bx,rdArray<double> tol)
{
	int size = _function.getSize();

	rdArray<double> a(0.0,size), b(0.0,size), c(0.0,size);
	rdArray<double> fa(0.0,size),fb(0.0,size),fc(0.0,size);
	rdArray<double> prev_step(0.0,size);
	rdArray<double> tol_act(0.0,size);
	rdArray<double> p(0.0,size);
	rdArray<double> q(0.0,size);
	rdArray<double> new_step(0.0,size);

	bool finished = false;
	rdArray<bool>   converged(false,size);


	// INITIALIZATIONS
	a = ax;
	b = bx;
	fa = _function->evaluate(a);
	fb = _function->evaluate(b);
	c = a;
	fc = fa;


	// ITERATION LOOP
	while(!finished) {

		// ABSCISSAE MANIPULATION LOOP
		for(j=0;j<size;j++) {

			// Continue?
			// If a function is already converged no need to do any manipulation.
			if(converged[j]) continue;
   
			// Make c on opposite side of b.
			// (was down at very bottom)
			 if( (fb[j]>0.0 && fc[j]>0.0) || (fb[j]<0.0 && fc[j]<0.0) ) {
				c[j] = a[j];
				fc[j] = fa[j];
			 }

			// Record previous step
			prev_step[j] = b[j] - a[j];

			// Swap data for b to be the best approximation.
			if( fabs(fc[j]) < fabs(fb[j]) ) {
				a[j] = b[j];  b[j] = c[j];  c[j] = a[j];
				fa[j]= fb[j]; fb[j]= fc[j]; fc[j]= fa[j];
			}
			tol_act[j] = 2.0*DBL_EPSILON*fabs(b[j]) + 0.5*tol[j];
			new_step[j] = 0.5 * (c[j]-b[j]);

			 // Converged?
			 if(fabs(new_step[j])<=tol_act[j] || fb[j]==(double)0.0 ) {
				 converged[j] = true;
				 continue;
			 }

			// Interpolate if prev_step was large enough and in true direction
			if( fabs(prev_step[j])>=tol_act[i] && fabs(fa[j])>fabs(fb[j]) ) {
				register double t1,cb,t2;
				cb = c[j]-b[j];

				// Only two distinct roots, must use linear interpolation.
				if(a[j]==c[j]) {
					t1 = fb[j]/fa[j];
					p[j] = cb*t1;
					q[j] = 1.0 - t1;

				// Quadratic interpolation
				} else {
					q[j] = fa[j]/fc[j];  t1 = fb[j]/fc[j];  t2 = fb[j]/fa[j];
					p[j] = t2 * ( cb*q[j]*(q[j]-t1) - (b[j]-a[j])*(t1-1.0) );
					q[j] = (q[j]-1.0) * (t1-1.0) * (t2-1.0);
				}

				// Change sign of q or p?
				if( p[j]>(double)0.0 ) {
					q[j] = -q[j];
				} else {
					p[j] = -p[j];
				}

				// If the interpolate is bad, use bissection.
				if( p[j]<(0.75*cb*q[j] - 0.5*fabs(tol_act[j]*q[j]))	&& p[j]<fabs(0.5*prev_step[j]*q[j]) )
					new_step[j] = p[j]/q[j];
			}

			// Adjust step to be not less than tolerance.
			if( fabs(new_step[j]) < tol_act[j] )
				if( new_step[j] > (double)0.0 )
					new_step[j] = tol_act[j];
				else
					new_step[j] = -tol_act[j];

			// Save previous approximation.
			a[j] = b[j];  fa[j] = fb[j];


			b[j] += new_step[j];
			 
		} // END ABSCISSAE LOOP
	 

		// NEW FUNCTION EVALUATION
		fb = _function->evaluate(b);


		// FINISHED?
		for(j=0;j<size;j++) {
			finished = true;
			if(!converged[j]) {
				finished = false;
				break;
			}
		}
	}

	// PRINT
	cout<<"RootSolver:  found solution!\n";
	cout<<b;
	cout<<fb;
	
	return(b);
}
