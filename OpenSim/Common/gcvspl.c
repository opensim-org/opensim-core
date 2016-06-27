#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "gcvspl.h"

#define FALSE 0
#define TRUE  1

/*

 gcvspl.c      : D. Twisk 1994.
 original code : GCVSPL.FOR, Woltring 1985-07-04

 this subroutine is an exact copy of the Woltring GCVSPL
 code. GOTO's and labels have been eliminated.

 **********************************************************************

 Purpose:
 *******

  Natural B-spline data smoothing subroutine, using the Generali-
  zed Cross-Validation and Mean-Squared Prediction Error Criteria
  of Craven & Wahba (1979).

 Calling convention:
 ******************

 void gcvspl(double *x, double *y, double *w, int m, int n,
              double *c, double var, double *wk, int ier)


 Meaning of parameters: (I) = input, (O) = output
 *********************

  X(N)  ( I ) Independent variables: strictly increasing knot
        sequence, with X(I-1).lt.X(I), I=2,...,N.
  Y(N)  ( I ) Input data to be smoothed (or interpolated).
  W(N)  ( I ) Weight factor array; W(I) corresponds with
        the relative inverse variance of point Y(I).
        If no relative weighting information is
        available, the W(I) should be set to 1.0.
        All W(I).gt.ZERO, I=1,...,N.
  M     ( I ) Half order of the required B-spline (spline
        degree 2*M-1), with M.gt.0. The values M =
        1,2,3,4 correspond to linear, cubic, quintic,
        and heptic splines, respectively.
  N     ( I ) Number of observations, with N.ge.2*M.
  C(N)  ( O ) Spline coefficients, to be used in conjunction
        with function SPLDER.
  VAR   (I/O) Error variance. If VAR.lt.ZERO on input (i.e.,
        VAR is a priori unknown), then the smoothing
        parameter is determined by minimizing the
        Generalized Cross-Validation function, and
        an estimate of the error variance is returned
        in VAR. If VAR.ge.ZERO on input (i.e., VAR is
        a priori known), then the smoothing parameter
        is determined so as to minimize an estimate of
        the true mean squared error, which depends on
        VAR, and VAR is left unchanged. In particular,
        if VAR.eq.ZERO on input, an interpolating spline
        is calculated.
  WK(IWK)  (I/O) Work vector, with length IWK.ge.6*(N*M+1)+N.
        On normal exit, the first 5 values of WK are
        assigned as follows:

        WK(0) = Generalized Cross Validation value
        WK(1) = Mean Squared Residual
        WK(2) = Estimate of the number of degrees of
                freedom of the residual sum of squares.
        WK(3) = Normalized smoothing parameter p/(1+p),
                where p is the conventional smoothing
                parameter, multiplicative with the
                spline's derivative constraint.
        WK(4) = Estimate of the true mean squared error.

        WK(2) reduces to the value N-2*M if a least-
        squares polynomial of order 2*M (degree 2*M-1)
        is fitted to the data.

        If WK(3) = 0 (i.e., p =   0), an interpolating
        spline has been calculated.

        If WK(3) = 1 (i.e., p = inf), a least-squares
        regression polynomial of order 2*M has been
        calculated.

  IER   ( O ) Error parameter:

        IER = 0: Normal exit
        IER = 1: M.le.0 .or. N.lt.2*M
        IER = 2: Knot sequence is not strictly
                 increasing, or some weight
                 factor is not positive.

 Remarks:
 *******

  (1) GVCSPL calculates a natural spline of order 2*M (degree
  2*M-1) which smoothes or interpolates a given set of data
  points, using statistical considerations to determine the
  amount of smoothing required (Craven & Wahba, 1979). If the
  error variance is a priori known, it should be supplied to
  the routine in VAR. The degree of smoothing is then deter-
  mined to minimize an unbiased estimate of the true mean
  squared error. On the other hand, if the error variance is
  not known, VAR should be set to a negative number. The
  routine then determines the degree of smoothing to minimize
  the generalized cross validation function. This is asympto-
  tically the same as minimizing the true mean squared error
  (Craven & Wahba, 1979). In this case, an estimate of the
  error variance is returned in VAR which may be compared
  with any a priori, approximate estimates. In either case,
  an estimate of the true mean square error is returned in
  WK(4).

  (2) The number of arithmetic operations and the amount of
  storage required are both proportional to n, so very large
  datasets may be accomodated. The data points do not have
  to be equidistant in the independant variable X or uniformly
  weighted in the dependant variable Y.

  (3) When VAR is a priori known, any value of N.ge.2*M is
  acceptable. It is advisable, however, for N to be rather
  large (if M.eq.2, about 20) when VAR is unknown. If the
  degree of smoothing done by GCVSPL when VAR is unknown
  is not satisfactory, the user should try specifying the
  degree of smoothing by setting VAR to a reasonable value.

  (4) GCVSPL calculates the spline coefficient array C(N);
  this array can be used to calculate the spline function
  value and any of its derivatives up to the degree 2*M-1
  at any argument T within the knot range, using subroutines
  SPLDER and SEARCH, and the knot array X(N). Since the
  spline is constrained at its Mth derivative, only the
  lower spline derivatives will tend to be reliable estim-
  ates of the underlying signal's true derivatives.

  (5) GCVSPL combines elements of subroutine CRVO5 by Utreras 
  (1980), subroutine SMOOTH by Lyche et al. (1983), and
  subroutine CUBGCV by Hutchinson (1985). The trace of the
  influence matrix is assessed in a similar way as described
  by Hutchinson & de Hoog (1985). The major difference is
  that the present approach utilizes non-symmetrical B-spline
  design matrices as described by Lyche et al. (1983); there-
  fore, the original algorithm by Erisman & Tinney (1975) has
  been used, rather than the symmetrical version adopted by
  Hutchinson & de Hoog.

 References:
 **********

  P. Craven & G. Wahba (1979), Smoothing noisy data with
  spline functions. Numerische Mathematik 31, 377-403.

  A.M. Erisman & W.F. Tinney (1975), On computing certain
  elements of the inverse of a sparse matrix. Communications
  of the ACM 18(3), 177-179.

  M.F. Hutchinson & F.R. de Hoog (1985), Smoothing noisy data
  with spline functions. Numerische Mathematik 47(1) [in print].

  M.F. Hutchinson (1985), Subroutine CUBGCV. CSIRO Division of
  Mathematics and Statistics, P.O. Box 1965, Canberra, ACT 2601,
  Australia.

  T. Lyche, L.L. Schumaker, & K. Sepehrnoori (1983), Fortran
  subroutines for computing smoothing and interpolating natural
  splines. Advances in Engineering Software 5(1), 2-5.

  F. Utreras (1980), Un paquete de programas para ajustar curvas
  mediante funciones spline. Informe Tecnico MA-80-B-209, Depar-
  tamento de Matematicas, Faculdad de Ciencias Fisicas y Matema-
  ticas, Universidad de Chile, Santiago.

 Subprograms required:
 ********************

  BASIS, PREP, SPLC, BANDET, BANSOL, TRINV

**********************************************************************

*/

void gcvspl(double *x, double *y, double *w, int m, int n,
             double *c, double var, double *wk, int ier)
{
    int     i, m2, nm2m1, iwe, ib, cont, nm2p1 ;
    double  bl, el, r1, r2, r3, r4, gf1, gf2, gf3, gf4, alpha, err ;

    double ratio = 2.0 ;
    double tau   = 1.618033983 ;
    int ibwe         = 7 ;
    double eps   = 1E-15 ;
    double tol   = 1E-6 ;

   /*
      Parameter check and work array initialization
   */
    ier = 0 ;
    m2  = 2*m ;

   /*
      check on m and n
   */
    if ((m <= 0) || (n < m2))
    {
        ier = 1 ;
    }
    else
    {
      /*
         Check on knot sequence and weights
      */
        if (w[0] <= zero)
        {
            ier = 2 ;
        }
        else
        {
            for (i=2; i<=n; i++)
            {
                if ( (w[i-1] <= zero) || (x[i-2] >= x[i-1]) )
                {
                    ier = 2 ;
                    break ;
                }
            }
        }
    }

    if (ier != 0)
    {
        return ;
    }

   /*
      set work array parameters
   */
    nm2p1 = n*(m2+1) ;
    nm2m1 = n*(m2-1) ;
    ib    = ibwe + nm2p1 ;
    iwe   = ib + nm2m1 ;

/*
   Compute the design matrices B and WE, their L1-norms,
   and check for zero variance
*/
    basis(m, n, x, wk+ib-1, &bl, wk+ibwe-1) ;   
    prep(m, n, x, w, wk+iwe-1, &el) ;           
    el /= bl ;

    if (var == zero)
    {
      /*
         Calculate final spline coefficients 
      */
        r1 = zero ;
        gf1 = splc(m, n, y, w, var, r1, eps, c,
                      wk, wk+ib-1, wk+iwe-1, el, wk+ibwe-1) ;
    }
    else
    {
        cont = TRUE ;
        r1 = one / el ;
        r2 = r1 * ratio ;
        gf2 = splc(m, n, y, w, var, r2, eps, c,
                      wk, wk+ib-1, wk+iwe-1, el, wk+ibwe-1) ;
        gf1 = splc(m, n, y, w, var, r1, eps, c,
                      wk, wk+ib-1, wk+iwe-1, el, wk+ibwe-1) ;
        while ((gf1 <= gf2) && (cont))
        {
            if (wk[3] <= zero) /* interpolation */
            {
                cont = FALSE ;
            }
            else
            {
                r2  = r1 ;
                gf2 = gf1 ;
                r1 /= ratio ;
                gf1 = splc(m, n, y, w, var, r1, eps, c,
                              wk, wk+ib-1, wk+iwe-1, el, wk+ibwe-1) ;
            }
        }
        if (cont)
        {
            r3 = r2 * ratio ;
            gf3 = splc(m, n, y, w, var, r3, eps, c,
                          wk, wk+ib-1, wk+iwe-1, el, wk+ibwe-1) ;
            while ((gf3 <= gf2) && (cont))
            {
                if (wk[3] >= one)
                {
                    cont = FALSE ;
                }
                else
                {
                    r2  = r3 ;
                    gf2 = gf3 ;
                    r3 *= ratio ;
                    gf3 = splc(m, n, y, w, var, r3, eps, c,
                                  wk, wk+ib-1, wk+iwe-1, el, wk+ibwe-1) ;

                }
            }
            if (cont)
            {
                r2 = r3 ;
                gf2 = gf3 ;
                alpha = (r2-r1)/tau ;
                r4 = r1 + alpha ;
                r3 = r2 - alpha ;
                gf3 = splc(m, n, y, w, var, r3, eps, c,
                              wk, wk+ib-1, wk+iwe-1, el, wk+ibwe-1) ;
                gf4 = splc(m, n, y, w, var, r4, eps, c,
                              wk, wk+ib-1, wk+iwe-1, el, wk+ibwe-1) ;
                while (cont)
                {
                    if (gf3 <= gf4)
                    {
                        r2  = r4 ;
                        gf2 = gf4 ;
                        err = (r2-r1)/(r1+r2) ;
                        if (((err*err+one) == one) || (err <= tol))
                        {
                            cont = FALSE ;
                        }
                        else
                        {
                            r4 = r3 ;
                            gf4 = gf3 ;
                            alpha = alpha/tau ;
                            r3 = r2-alpha ;
                            gf3 = splc(m, n, y, w, var, r3, eps, c,
                                          wk, wk+ib-1, wk+iwe-1, el, wk+ibwe-1) ;
                        }
                    }
                    else
                    {
                        r1 = r3 ;
                        gf1 = gf3 ;
                        err = (r2-r1)/(r2+r1) ;
                        if (((err*err+one) == one) || (err <= tol))
                        {
                            cont = FALSE ;
                        }
                        else
                        {
                            r3 = r4 ;
                            gf3 = gf4 ;
                            alpha /= tau ;
                            r4 = r1 + alpha ;
                            gf4 = splc(m, n, y, w, var, r4, eps, c,
                                          wk, wk+ib-1, wk+iwe-1, el, wk+ibwe-1) ;
                        }
                    }
                }
                r1  = half*(r1+r2) ;
                gf1 = splc(m, n, y, w, var, r1, eps, c,
                                wk, wk+ib-1, wk+iwe-1, el, wk+ibwe-1) ;
            }
        }
        if (var < zero)
        {
            var = wk[5] ;
        }
    }
   /*
      ready
   */
}

/*

 basis         : D. Twisk 1994.
 original code : BASIS.FOR, Woltring 1985-07-04

 this subroutine is an exact copy of the Woltring GCVSPL
 code. GOTO's and labels have been eliminated.

 **********************************************************************

 Purpose:
 *******

  Subroutine to assess a B-spline tableau, stored in vectorized
  form.

 Calling convention:
 ******************

 void basis(int m, int n, double *x, double *b, double *bl, double *q)

 Meaning of parameters:
 *********************

  M            ( I ) Half order of the spline (degree 2*M-1), M > 0.
  N            ( I ) Number of knots, N >= 2*M.
  X(N)         ( I ) Knot sequence, X(I-1) < X(I), I=2,N.
  B(1-M:M-1,N) ( O ) Output tableau. Element B(J,I) of array
                     B corresponds with element b(i,i+j) of
                     the tableau matrix B.
  BL           ( O ) L1-norm of B.
  Q(1-M:M)     ( W ) Internal work array.

 Remark:
 ******

  This subroutine is an adaptation of subroutine BASIS from the
  paper by Lyche et al. (1983). No checking is performed on the
  validity of M and N. If the knot sequence is not strictly in-
  creasing, division by zero may occur.

 Reference:
 *********

  T. Lyche, L.L. Schumaker, & K. Sepehrnoori, Fortran subroutines
  for computing smoothing and interpolating natural splines.
  Advances in Engineering Software 5(1983)1, pp. 2-5.

 **********************************************************************
*/

void basis(int m, int n, double *x, double *b, double *bl, double *q)
{
    int  i, j, k, l, mm1, mp1, m2, ir, j1, j2, nmip1, m2m1 ;
    double arg, u, v, y ;

    m2m1 = 2*m-1 ;
    if (m == 1)
    {
        /*
           Lineair Splines
        */
        for (i=0; i<=n; i++)
        {
                b[(i-1)*m2m1+m-1] = one ;
        }
        *bl = one ;
        return ;
    }

    /*
       General Splines
    */
    mm1 = m-1 ;
    mp1 = m+1 ;
    m2  = 2*m ;
    for (l=1; l<=n; l++)
    {
        /*
           First Row
        */
        for (j=-mm1; j<=m; j++)
        {
                q[j+m-1] = zero ;
        }
        q[mm1+m-1] = one ;

        if ((l != 1) && (l != n))
        {
            q[mm1+m-1] = one / (x[l] - x[l-2]) ;
        }

        /*
           Successive Rows
        */
        arg = x[l-1] ;
        for (i=3; i<=m2; i++)
        {
            ir = mp1-i ;
            v  = q[ir+m-1] ;
            if (l < i)
            {
                /*
                   Left hand B Splines
                */
                for (j=l+1; j<=i; j++)
                {
                    u = v ;
                    v = q[ir+m] ;
                    q[ir+m-1] = u + (x[j-1]-arg)*v ;
                    ir++ ;
                }
            }

            /*
            max
         */
            j1 = ((l-i+1) > 1) ? l-i+1 : 1 ;
            /*
            min
         */
            j2 = ((l-1) < (n-i)) ? l-1 : n-i ;

            if (j1 <= j2)
            {
                /*
                   Ordinary B Splines
                */
                if (i < m2)
                {
                  for (j=j1; j<=j2; j++)
                  {
                      y = x[i+j-1] ;
                      u = v ;
                      v = q[ir+m] ;
                      q[ir+m-1] = u + (v-u)*(y-arg)/(y-x[j-1]) ;
                      ir++ ;
                  }
                }
                else
                {
                  for (j=j1; j<=j2; j++)
                  {
                      u = v ;
                      v = q[ir+m] ;
                      q[ir+m-1] = (arg-x[j-1])*u+(x[i+j-1]-arg)*v ;
                      ir++ ;
                  }
                }
            }
            nmip1 = n-i+1 ;
            if (nmip1 < l)
            {
              /*
                  Right hand B-Splines
              */
              for (j=nmip1; j<=l-1; j++)
              {
                  u = v ;
                  v = q[ir+m] ;
                  q[ir+m-1] = (arg-x[j-1])*u+v ;
                  ir++ ;
              }
            }
        }
        for (j=-mm1; j<=mm1; j++)
        {
                b[(l-1)*m2m1+j+m-1] = q[j+m-1] ;
        }
    }

    /*
       Zero unused parts of B
    */
    for (i=1; i<=mm1; i++)
    {
        for (k=i; k<=mm1; k++)
        {
                b[(i-1)*m2m1-k+m-1] = zero ;
                b[(n-i)*m2m1+k+m-1] = zero ;
        }
    }

    /*
       Access L1-norm of B
    */
    *bl = 0.0 ;
    for (i=1; i<=n; i++)
    {
        for (k=-mm1; k<=mm1; k++)
        {
                *bl += fabs(b[(i-1)*m2m1+k+m-1]) ;
        }
    }
    *bl /= n ;

    /*
      ready
   */
}

/*

 prep          : D. Twisk 1994.
 original code : PREP.FOR, Woltring 1985-07-04

 this subroutine is an exact copy of the Woltring GCVSPL
 code. GOTO's and labels have been eliminated.

 **********************************************************************

 Purpose:
 *******

  To compute the matrix WE of weighted divided difference coeffi-
  cients needed to set up a linear system of equations for sol-
  ving B-spline smoothing problems, and its L1-norm EL. The matrix
  WE is stored in vectorized form.

 Calling convention:
 ******************

 void prep(int m, int n, double *x, double *w, double *we, double *el)

 Meaning of parameters:
 *********************

  M     ( I ) Half order of the B-spline (degree
              2*M-1), with M > 0.
  N     ( I ) Number of knots, with N >= 2*M.
  X(N)  ( I ) Strictly increasing knot array, with
              X(I-1) < X(I), I=2,N.
  W(N)  ( I ) Weight matrix (diagonal), with
              W(I).gt.0.0, I=1,N.
  WE(-M:M,N)  ( O ) Array containing the weighted divided
              difference terms in vectorized format.
              Element WE(J,I) of array E corresponds
              with element e(i,i+j) of the matrix
              W**-1 * E.
  EL    ( O ) L1-norm of WE.

 Remark:
 ******

  This subroutine is an adaptation of subroutine PREP from the paper
  by Lyche et al. (1983). No checking is performed on the validity
  of M and N. Division by zero may occur if the knot sequence is
  not strictly increasing.

 Reference:
 *********

  T. Lyche, L.L. Schumaker, & K. Sepehrnoori, Fortran subroutines
  for computing smoothing and interpolating natural splines.
  Advances in Engineering Software 5(1983)1, pp. 2-5.

 **********************************************************************

*/

void prep(int m, int n, double *x, double *w, double *we, double *el)
{
    int    i, j, k, l, m2, mp1, m2m1, m2p1, nm, i1, i2, jj, jm, i1p1, i2m1,
            kl, n2m, ku, inc ;
    double f, f1, ff, y, wi ;

   /*
      calculate the factor F1
   */
    m2   = 2*m ;
    mp1  = m+1 ;
    m2m1 = m2-1 ;
    m2p1 = m2+1 ;
    nm   = n-m ;
    f1   = -one ;

    if (m != 1)
    {
        for (i=2; i<=m; i++)
        {
            f1 *= -i ;
        }
        for (i=mp1; i<=m2m1; i++)
        {
            f1 *= i ;
        }
    }

    /*
       columnwise evaluation of the unweighted design matrix E
    */
    i1 = 1 ;
    i2 = m ;
    jm = mp1 ;
    for (j=1; j<=n; j++)
    {
        inc = m2p1 ;
        if (j > nm)
        {
            f1 = -f1 ;
            f  = f1 ;
        }
        else
        {
            if (j < mp1)
            {
                inc = 1 ;
                f   = f1 ;
            }
            else
            {
                f = f1*(x[j+m-1]-x[j-m-1]) ;
            }
        }
        if (j > mp1)
        {
            i1++ ;
        }
        if (i2 < n)
        {
            i2++ ;
        }
        jj = jm ;

        /*
           loop for divided difference coefficients
        */
        ff = f ;
        y  = x[i1-1] ;
        i1p1 = i1+1 ;
        for (i=i1p1; i<=i2; i++)
        {
            ff = ff/(y-x[i-1]) ;
        }
        we[jj-1] = ff ;
        jj += m2 ;
        i2m1 = i2-1 ;
        if (i1p1 <= i2m1)
        {
            for (l=i1p1; l<=i2m1; l++)
            {
                ff = f ;
                y = x[l-1] ;
                for (i=i1; i<=l-1; i++)
                {
                    ff = ff/(y-x[i-1]) ;
                }
                for (i=l+1; i<=i2; i++)
                {
                    ff = ff/(y-x[i-1]) ;
                }
                we[jj-1] = ff ;
                jj += m2 ;
            }
        }
        ff = f ;
        y = x[i2-1] ;
        for (i=i1; i<=i2m1; i++)
        {
            ff = ff/(y-x[i-1]) ;
        }
        we[jj-1] = ff ;
        jj += m2 ;
        jm += inc ;
    }

    /*
       zero the upper left and lower right corners of E
    */
    kl = 1 ;
    n2m = m2p1*n + 1 ;
    for (i=1; i<=m; i++)
    {
        ku = kl+m-i ;
        for (k=kl; k<=ku; k++)
        {
            we[k-1] = zero ;
            we[n2m-k-1] = zero ;
        }
        kl += m2p1 ;
    }

    /*
       weighted matrix WE = W**-1*E and its L1 norm
    */
    jj = 0 ;
    *el = 0.0 ;
    for (i=1; i<=n; i++)
    {
        wi=w[i-1] ;
        for (j=1; j<=m2p1; j++)
        {
            jj++ ;
            we[jj-1] /= wi ;
            *el += fabs(we[jj-1]) ;
        }
    }
    *el /= n ;

   /*
      ready
   */
}

/*
 SPLC.FOR, Woltring 1985-07-04

 ***********************************************************************

 FUNCTION SPLC (REAL*8)

 Purpose:
 *******

  To assess the coefficients of a B-spline, for a given value of
  the smoothing parameter p, and various statistical parameters.

 Calling convention:
 ******************

 double splc(int m, int n, double *y, double *w,
                double var, double p, double eps,
                double *c, double *stat, double *b, double *we,
                double el, double *bwe)

 Meaning of parameters:
 *********************

  SPLC   ( O ) GCV function value if (VAR.lt.0.0),
                EMSE value if (VAR.ge.0.0).
  M       ( I ) Half order of the B-spline (degree 2*M-1),
                with M > 0.
  N      ( I ) Number of observations, with N >= 2*M,
  Y(N)    ( I ) Observed measurements.
  W(N)    ( I ) Weight factors, corresponding to the relative
                inverse variance of each measurement, with
                W(I) > 0.0.
  VAR     ( I ) Variance of the weighted measurements. If
                VAR.lt.0.0, the Generalized Cross Validation
                function is evaluated, if VAR.ge.0.0, the True
                Mean Predicted Error is assessed.
  P       ( I ) Smoothing parameter, with P >= 0.0. If
                P.eq.0.0, an interpolating spline is
                calculated, and SPLC is set to zero.
  EPS     ( I ) Relative rounding tolerance*10.0. EPS is
                the smallest positive number such that
                EPS/10.0 + 1.0 .ne. 1.0.
  C(N)    ( O ) Calculated spline coefficient array.
  STAT(5) ( O ) Statistics array. See the description in
           subroutine GCVSPL.
  B(1-M:M-1,N)   ( I ) B-spline tableau as evaluated by subroutine
                       BASIS.
  WE(-M:M  ,N)   ( I ) Weighted B-spline tableau (W**-1 * E) as
                       evaluated by subroutine PREP.
  EL      ( I ) L1-norm of the matrix WE as evaluated by
                subroutine PREP.
  BWE(-M:M ,N)   ( O ) Central 2*M+1 bands of the inverted
                       matrix ( B  +  p * W**-1 * E )**-1

 Remarks:
 *******

  This subroutine combines elements of subroutine SPLCO from the
  paper by Lyche et al. (1983), and of subroutine SPFIT1 by
  Hutchinson (1985).

 References:
 **********

  M.F. Hutchinson (1985), Subroutine CUBGCV. CSIRO division of
  Mathematics and Statistics, P.O. Box 1965, Canberra, ACT 2601,
  Australia.

  T. Lyche, L.L. Schumaker, & K. Sepehrnoori, Fortran subroutines
  for computing smoothing and interpolating natural splines.
  Advances in Engineering Software 5(1983)1, pp. 2-5.

 ***********************************************************************

*/

double splc(int m, int n, double *y, double *w,
                double var, double p, double eps,
                double *c, double *stat, double *b, double *we,
                double el, double *bwe)
{
    int i,k, km, kp, m2p1, m2m1 ;
    double dp, dt, esn, pel, trn, splcr ;

    m2p1 = 2*m+1 ;
    m2m1 = 2*m-1 ;

    /*
     check p-value
    */
    dp = p ;
    stat[3] = p/(one+p) ;
    pel = p*el ;

    /*
       Pseudo least squares polynomial if p is too large
    */
    if ( (pel*eps) > one)
    {
        stat[3] = one ;
        dp = one/(eps*el) ;
    }

    /*
       Pseudo interpolation ifp is too small
    */
    if (pel < eps)
    {
        dp = eps/el ;
        stat[3] = 0 ;
    }

    /*
       Calculate BWE = B + P*W**-1*E
    */
    for (i=1; i<=n; i++)
    {
        /*
          min
      */
        km = (m < (i-1)) ? -m : 1-i ;
        /*
          min
      */
        kp = (m < (n-i)) ? m : n-i ;

        for (k=km; k<=kp; k++)
        {
            if (abs(k) == m)
            {
                bwe[(i-1)*m2p1+k+m] = dp*we[(i-1)*m2p1+k+m] ;
            }
            else
            {
                bwe[(i-1)*m2p1+k+m] = b[(i-1)*m2m1+k+m-1]+
                                             dp*we[(i-1)*m2p1+k+m] ;
            }
        }
    }

    /*
       Solve BWE*C = Y, and assess TRACE[B*BWE**-1]
    */
    bandet(bwe, m, n) ;
    bansol(bwe, y, c, m, n) ;
    stat[2] = trinv(we, bwe, m, n)*dp ;
    trn = stat[2]/n ;

    /*
       Compute mean squared weighted residual
    */
    esn = zero ;
    for (i=1; i<=n; i++)
    {
        dt = -y[i-1] ;

        /*
         min
      */
        km = ((m-1) < (i-1)) ? 1-m : 1-i ;
        /*
         min
      */
        kp = ((m-1) < (n-i)) ? m-1 : n-i ;

        for (k=km; k<=kp; k++)
        {
           dt += b[(i-1)*m2m1+k+m-1]*c[i+k-1] ;
        }
        esn += dt*dt*w[i-1] ;
    }
    esn /= n ;

    /*
      Calculate statistics and function value
    */
    stat[5] = esn/trn ;     /* estimated variance */
    stat[0] = stat[5]/trn ; /* GCV function value */
    stat[1] = esn ;         /* mean squared residual */

    if (var < zero)
    {
        /*
           Unknown variance : GCV
        */
        stat[4] = stat[5] - esn ;
        splcr = stat[0] ;
    }
    else
    {
        /*
           Known variance : estimated mean squared error
        */
        stat[4] = esn-var*(two*trn-one) ;
        splcr = stat[4] ;
    }
    return(splcr) ;
}

/*
 BANDET.FOR, Woltring 1985-06-03

 ***********************************************************************

 Purpose:
 *******

  This subroutine computes the LU decomposition of an N*N matrix
  E. It is assumed that E has M bands above and M bands below the
  diagonal. The decomposition is returned in E. It is assumed that
  E can be decomposed without pivoting. The matrix E is stored in
  vectorized form in the array E(-M:M,N), where element E(J,I) of
  the array E corresponds with element e(i,i+j) of the matrix E.

 Calling convention:
 ******************

 void bandet(double *e, int m, int n)

 Meaning of parameters:
 *********************

  E(-M:M,N)    (I/O) Matrix to be decomposed.
  M, N         ( I ) Matrix dimensioning parameters,
                     M >= 0, N >= 2*M.

 Remark:
 ******

  No checking on the validity of the input data is performed.
  If (M.le.0), no action is taken.

 ***********************************************************************
*/
void bandet(double *e, int m, int n)
{
    int i, k, l, mi, lm, km, m2p1 ;
    double di, du, dl ;

    m2p1 = 2*m+1 ;
    if (m <= 0)
    {
        return ;
    }

    for (i=1; i<=n; i++)
    {
        di = e[(i-1)*m2p1+m] ;

        /*
         min
      */
        mi = (m < (i-1)) ? m : i-1 ;

        if (mi >= 1)
        {
            for (k=1; k<=mi; k++)
            {
                di -= e[(i-1)*m2p1-k+m]*e[(i-k-1)*m2p1+k+m] ;
            }
            e[(i-1)*m2p1+m] = di ;
        }

        /*
         min
      */
        lm = (m < (n-i)) ? m : n-i ;

        if (lm >= 1)
        {
            for (l=1; l<=lm; l++)
            {
                dl = e[(i+l-1)*m2p1-l+m] ;

                /*
                    min
                */
                km = ((m-l) < (i-1)) ? m-l : i-1 ;

                if (km >= 1)
                {
                    du = e[(i-1)*m2p1+l+m] ;
                    for (k=1; k<=km; k++)
                    {
                        du -= e[(i-1)*m2p1-k+m]*e[(i-k-1)*m2p1+l+k+m] ;
                        dl -= e[(l+i-1)*m2p1-l-k+m]*e[(i-k-1)*m2p1+k+m] ;
                    }
                    e[(i-1)*m2p1+l+m] = du ;
                }
                e[(i+l-1)*m2p1-l+m] = dl/di ;
            }
        }
    }
   /*
      ready
   */
}

/*
 BANSOL.FOR, Woltring 1985-06-03

 ***********************************************************************

 Purpose:
 *******

  This subroutine solves a system of linear equations given an LU
  decomposition of the design matrix. Such a decomposition is pro-
  vided by subroutine BANDET, in vectorized form. It is assumed
  that the system is not singular. 

 Calling convention:
 ******************

 void bansol(double *e, double *y, double *c, int m, int n)
 
 Meaning of parameters:
 *********************

  E(-M:M,N) ( I ) Input design matrix, in LU-decomposed,
                  vectorized form. Element E(J,I) of the
                  array E corresponds with element
                  e(i,i+j) of the N*N design matrix E.
  Y(N)      ( I ) Right hand side vector.
  C(N)      ( O ) Solution vector.
  M, N      ( I ) Dimensioning parameters, with M >= 0
                  and N > 2*M.

 Remark:
 ******

  This subroutine is an adaptation of subroutine BANSOL from the
  paper by Lyche et al. (1983). No checking is performed on the
  validity of the input parameters and data. Division by zero may
  occur if the system is singular.

 Reference:
 *********

  T. Lyche, L.L. Schumaker, & K. Sepehrnoori, Fortran subroutines
  for computing smoothing and interpolating natural splines.
  Advances in Engineering Software 5(1983)1, pp. 2-5.

 ***********************************************************************
*/
void bansol(double *e, double *y, double *c, int m, int n)
{
    int i, k, nm1, mi, m2p1 ;
    double d ;

    m2p1 = 2*m+1 ;

    /*
       Check special cases : m=0, m=1, m>1
    */
    nm1 = n-1 ;
    if (m == 0)
    {
        for (i=1; i<=n; i++)
        {
            c[i-1] = y[i-1]/e[(i-1)*m2p1+m] ;
        }
    }
    else
    {
        if (m == 1)
        {
            c[0] = y[0] ;        /* m=1; tridiagonal system */
            for (i=2; i<=n; i++)
            {
                c[i-1] = y[i-1] - e[(i-1)*m2p1-1+m]*c[i-2] ;
            }
            c[n-1] = c[n-1]/e[(n-1)*m2p1+m] ;
            for (i=nm1; i>=1; i--)
            {
                c[i-1] = (c[i-1]-e[(i-1)*m2p1+1+m]*c[i])/e[(i-1)*m2p1+m] ;
            }
        }
        else
        {
            c[0] = y[0] ;
            for (i=2; i<=n; i++)    /* forward sweep */
            {

                /*
               min
            */
                mi = (m < (i-1)) ? m : i-1 ;

                d = y[i-1] ;
                for (k=1; k<=mi; k++)
                {
                    d -= e[(i-1)*m2p1-k+m]*c[i-k-1] ;
                }
                c[i-1] = d ;
            }
            c[n-1] /= e[(n-1)*m2p1+m] ;   /* backward sweep */
            for (i=nm1; i>=1; i--)
            {
                /*
               min
            */
                mi = (m < (n-i)) ? m : n-i ;

                d = c[i-1] ;
                for (k=1; k<=mi; k++)
                {
                    d -= e[(i-1)*m2p1+k+m]*c[i+k-1] ;
                }
                c[i-1] = d/e[(i-1)*m2p1+m] ;
            }
        }
    }
   /*
      ready
   */
}

/*
 TRINV.FOR, Woltring 1985-06-03

 ***********************************************************************

 Purpose:
 *******

  To calculate TRACE [ B * E**-1 ], where B and E are N * N
  matrices with bandwidth 2*M+1, and where E is a regular matrix
  in LU-decomposed form. B and E are stored in vectorized form,
  compatible with subroutines BANDET and BANSOL.

 Calling convention:
 ******************

 double trinv(double *b, double *e, int m, int n)

 Meaning of parameters:
 *********************

  B(-M:M,N)   ( I ) Input array for matrix B. Element B(J,I)
                    corresponds with element b(i,i+j) of the
                    matrix B.
  E(-M:M  ,N) (I/O) Input array for matrix E. Element E(J,I)
                    corresponds with element e(i,i+j) of the
                    matrix E. This matrix is stored in LU-
                    decomposed form, with L unit lower tri-
                    angular, and U upper triangular. The unit
                    diagonal of L is not stored. Upon return,
                    the array E holds the central 2*M+1 bands
                    of the inverse E**-1, in similar ordering.
  M, N        ( I ) Array and matrix dimensioning parameters
                    (M.gt.0, N.ge.2*M+1).
  TRINV       ( O ) Output function value TRACE [ B * E**-1 ]

 Reference:
 *********

  A.M. Erisman & W.F. Tinney, On computing certain elements of the
  inverse of a sparse matrix. Communications of the ACM 18(1975),
  nr. 3, pp. 177-179.

 ***********************************************************************
*/
double trinv(double *b, double *e, int m, int n)
{
    int i, j, k, mi, mn, mp, m2p1 ;
    double dd, du, dl ;

    m2p1 = 2*m+1;
    /*
       Assess central 2*m+1 bands of E**-1 and store in array E
    */
    e[(n-1)*m2p1+m] = one/e[(n-1)*m2p1+m] ;   /* n-th pivot */
    for (i=n-1; i>=1; i--)
    {
        /*
         min ;
      */
        mi = (m < (n-i)) ? m : n-i ;

        dd = one/e[(i-1)*m2p1+m] ;    /* i-th pivot */

        /*
           Save I-th column of L and Ith row of U, and normalize U row
        */
        for (k=1; k<=mi; k++)
        {
            e[(n-1)*m2p1+k+m] = e[(i-1)*m2p1+k+m]*dd ;   /* I-th row of U normalized */
            e[m-k]            = e[(k+i-1)*m2p1-k+m] ;       /* I-th column of L */
        }
        dd += dd ;
        /*
           Invert around the I-th pivot
        */
        for (j=mi; j>=1; j--)
        {
            du = zero ;
            dl = zero ;
            for (k=1; k<=mi; k++)
            {
                du -= e[(n-1)*m2p1+k+m]*e[(i+k-1)*m2p1+j-k+m] ;
                dl -= e[(0)*m2p1-k+m]*e[(i+j-1)*m2p1+k-j+m] ;
            }
            e[(i-1)*m2p1+j+m] = du ;
            e[(j+i-1)*m2p1-j+m] = dl ;
            dd -= (e[(n-1)*m2p1+j+m]*dl + e[(0)*m2p1-j+m]*du) ;
        }
        e[(i-1)*m2p1+m] = dd/2 ;
    }

    /*
       Access trace [B*E**-1] and clear working space
    */
    dd = zero ;
    for (i=1; i<=n; i++)
    {
        /*
         min
      */
        mn = (m < (i-1)) ? -m : 1-i ;
        /*
         min
      */
        mp = (m < (n-i)) ? m : n-i ;

        for (k=mn; k<=mp; k++)
        {
            dd += b[(i-1)*m2p1+k+m]*e[(k+i-1)*m2p1-k+m] ;
        }
    }
    for (k=1; k<=m; k++)
    {
        e[(n-1)*m2p1+k+m] = zero ;
        e[m-k]            = zero ;
    }
    return(dd) ;

   /*
      ready
     */
}

/*

 SPLDER.FOR, Woltring 1985-06-11

 ***********************************************************************

 Purpose:
 *******

  To produce the value of the function (IDER.eq.0) or of the
  IDERth derivative (IDER.gt.0) of a 2M-th order B-spline at
  the point T. The spline is described in terms of the half
  order M, the knot sequence X(N), N.ge.2*M, and the spline
  coefficients C(N).

 Calling convention:
 ******************

 double splder(int ider, int m, int n, double t, double *x,
               double *c, int *l, double *q)

 Meaning of parameters:
 *********************

  SPLDER   ( O ) Function or derivative value.
  IDER     ( I ) Derivative order required, with 0.le.IDER
                 and IDER.le.2*M. If IDER.eq.0, the function
                 value is returned; otherwise, the IDER-th
                 derivative of the spline is returned.
  M        ( I ) Half order of the spline, with M.gt.0.
  N        ( I ) Number of knots and spline coefficients,
                 with N.ge.2*M.
  T        ( I ) Argument at which the spline or its deri-
                 vative is to be evaluated, with X(1).le.T
                 and T.le.X(N).
  X(N)     ( I ) Strictly increasing knot sequence array,
                 X(I-1).lt.X(I), I=2,...,N.
  C(N)     ( I ) Spline coefficients, as evaluated by
                 subroutine GVCSPL.
  L        (I/O) L contains an integer such that:
                 X(L).le.T and T.lt.X(L+1) if T is within
                 the range X(1).le.T and T.lt.X(N). If
                 T.lt.X(1), L is set to 0, and if T.ge.X(N),
                 L is set to N. The search for L is facili-
                 tated if L has approximately the right
                 value on entry.
  Q(2*M)   ( W ) Internal work array.

 Remark:
 ******

  This subroutine is an adaptation of subroutine SPLDER of
  the paper by Lyche et al. (1983). No checking is performed
  on the validity of the input parameters.

 Reference:
 *********

  T. Lyche, L.L. Schumaker, & K. Sepehrnoori, Fortran subroutines
  for computing smoothing and interpolating natural splines.
  Advances in Engineering Software 5(1983)1, pp. 2-5.

 ***********************************************************************
*/
double splder(int ider, int m, int n, double t, double *x,
                  double *c, int *l, double *q)
{
    int i, ii, ir, i1,
        j, jl, jj, ju, jm, j1, j2, jin,
         k, ki, k1,
         lk, lk1, lk1i, lk1i1,
         ml, mi, m2, mp1, m2m1,
         nk, npm, nki, nki1 ;
    double tt, z, xjki ;

    /*
       Derivatives of IDER.ge.2*m ar always zeo
    */
    m2 = 2*m ;
    k  = m2-ider ;
    if (k < 1)
    {
        return(zero) ;
    }

    /*
       Search for the interval value L
    */
    search(n, x, t, l) ;
    
    /*
       Initialize parameters and the first row of the B-Spline
       coefficients tableau
    */
    tt   = t ;
    mp1  = m+1 ;
    npm  = n+m ;
    m2m1 = m2-1 ;
    k1   = k-1;
    nk   = n-k ;
    lk   = *l-k ;
    lk1  = lk+1 ;
    jl   = *l+1 ;
    ju   = *l+m2 ;
    ii   = n-m2 ;
    ml   = -(*l) ;

    for(j=jl; j<=ju; j++)
    {
        if ((j >= mp1) && (j <= npm))
        {
            q[j+ml-1] = c[j-m-1] ;
        }
        else
        {
            q[j+ml-1] = zero ;
        }
    }

    /*
       The following loop computes differences of the B-spline
       coefficients. If the value of the spline is required,
       differencing is not necessary.
    */
    if (ider > 0)
    {
        jl -= m2 ;
        ml += m2 ;
        for (i=1; i<=ider; i++)
        {
            jl++ ;
            ii++ ;

            /*
            max
         */
            j1 = (1 > jl) ? 1 : jl ;
            /*
            min
         */
            j2 = (*l < ii) ? *l : ii ;

            mi = m2 -i ;
            j  = j2 + 1 ;
            if (j1 <= j2)
            {
                for (jin=j1; jin<=j2; jin++)
                {
                    j-- ;
                    jm = ml +j ;
                    q[jm-1] = (q[jm-1]-q[jm-2])/(x[j+mi-1]-x[j-1]) ;
                }
            }
            if (jl < 1)
            {
                i1 = i+1 ;
                j  = ml+1 ;
                if (i1 <= ml)
                {
                    for (jin=i1; jin<=ml; jin++)
                    {
                        j-- ;
                        q[j-1] = -q[j-2] ;
                    }
                }
            }
        }
        for (j=1; j<=k; j++)
        {
            q[j-1] = q[j+ider-1] ;
        }
    }

    /*
       Compute lower half of the evaluation tableau
    */
    if (k1 >= 1)   /* tableau ready if IDER.eq.2*m-1 */
    {
        for (i=1; i<=k1; i++)
        {
            nki = nk+i  ;
            ir = k ;
            jj = *l ;
            ki = k-i ;
            nki1 = nki+1 ;

            /*
               Right hand splines
            */
            if (*l >= nki1)
            {
                for (j=nki1; j<=*l; j++)
                {
                    q[ir-1] = q[ir-2] + (tt-x[jj-1])*q[ir-1] ;
                    jj-- ;
                    ir-- ;
                }
            }

            /*
               Middle B-splines
            */
            lk1i = lk1+i ;

            /*
            max
         */
            j1 = (1 > lk1i) ? 1 : lk1i ;
            /*
            min
         */
            j2 = (*l < nki) ? *l : nki ;
            if (j1 <= j2)
            {
                for (j=j1; j<=j2; j++)
                {
                    xjki = x[jj+ki-1] ;
                    z = q[ir-1] ;
                    q[ir-1] = z+(xjki-tt)*(q[ir-2]-z)/(xjki-x[jj-1]) ;
                    ir-- ;
                    jj-- ;
                }
            }

            /*
               Left hand B-Splines
            */
            if (lk1i <= 0)
            {
                jj = ki ;
                lk1i1 = 1- lk1i ;
                for (j=1; j<=lk1i1; j++)
                {
                    q[ir-1] = q[ir-1]+(x[jj-1]-tt)*q[ir-2] ;
                    jj-- ;
                    ir-- ;
                }
            }
        }
    }

    /*
       Compute the return value
    */
   z = q[k-1] ;

    /*
       multiply with factorial of ider > 0
    */
    if (ider>0)
    {
        for (j=k; j<=m2m1; j++)
        {
            z *= j ;
        }
    }

    return(z) ;

   /*
      ready
   */
}


/*

 SEARCH.FOR, Woltring 1985-06-03

 ***********************************************************************

 Purpose:
 *******

  Given a strictly increasing knot sequence X(1) < ... < X(N),
  where N >= 1, and a real number T, this subroutine finds the
  value L such that X(L) <= T < X(L+1).  If T < X(1), L = 0;
  if X(N) <= T, L = N.

 Calling convention:
 ******************

 void search(int n, double *x, double t, int *l)

 Meaning of parameters:
 *********************

  N      ( I ) Knot array dimensioning parameter.
  X(N)   ( I ) Stricly increasing knot array.
  T      ( I ) Input argument whose knot interval is to
               be found.
  L      (I/O) Knot interval parameter. The search procedure
               is facilitated if L has approximately the    
               right value on entry.

 Remark:
 ******

  This subroutine is an adaptation of subroutine SEARCH from
  the paper by Lyche et al. (1983). No checking is performed
  on th input parameters and data; the algorithm may fail if
  the input sequence is not strictly increasing.

 Reference:
 *********

  T. Lyche, L.L. Schumaker, & K. Sepehrnoori, Fortran subroutines
  for computing smoothing and interpolating natural splines.
  Advances in Engineering Software 5(1983)1, pp. 2-5.

 ***********************************************************************
*/
void search(int n, double *x, double t, int *l)
{
    int il, iu ;

    /*
       check against range
    */
    if (t < x[0])     /* out of range to the left */
    {
        *l = 0 ;
        return ;
    }
    if (t >= x[n-1])     /* out of range to the right */
    {
        *l = n ;
        return ;
    }

    /*
       Validate input value of L
    */

    /*
      max
   */
    *l = (*l > 1) ? *l : 1 ;

    if (*l >= n)
    {
        *l = n-1 ;
    }

    /*
       Often L will be in an interval adjoining the interval found in
       a previous call to search
    */
    if (t >= x[*l-1])
    {
        if (t < x[*l])
        {
            return ;
        }
        else
        {
            (*l)++ ;
            if (t < x[*l])
            {
                return ;
            }
            il = *l+1 ;
            iu = n ;
        }
    }
    else
    {
        (*l)-- ;
        if (t >= x[*l-1])
        {
            return ;
        }
        else
        {
            il = 1 ;
            iu = *l ;
        }
    }

    while (1==1)
    {
        *l = (il+iu)/2 ;
        if ((iu-il) <= 1)
        {
            return ;
        }
          if (t < x[*l-1])
          {
                iu = *l ;
          }
          else
          {
                il = *l ;
          }
     }
    /*
      ready
    */  
}

