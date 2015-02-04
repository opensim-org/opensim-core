/*******************************************************************************

   ELLIPSOID_WRAP.C

   Author: Ken Smith, Craig Robinson, Frans van der Helm, Peter Loan

   Copyright (c) 1994-2002 MusculoGraphics, Inc.
   All rights reserved.

   Description: 

   The algorithm for wrapping muscles around an ellipsoid was originally
   developed in Fortran by Frans van der Helm, translated to C by Craig,
   modified by Pete, and modified again by Kenny.  Pete and Kenny spent
   quite a bit of time in early '99 trying to make the algorithm stable
   (i.e. eliminate all muscle skips & jumps during wrapping).
   
   The routines contributed by Frans and still in use are:
   
      calc_line_intersect_ellipsoid - the main routine, wraps a muscle
                                      around an ellipsoid
      
      calc_r - find a tangent point on the ellipsoid surface
      
      dell   - find wrapping line segments along ellipsoid surface
   
   Another routine, afstll(), was conttributed by Frans but replaced by
   pt_to_ellipsoid() which was written by Dave Eberly and located by
   searching usenet for "ellipsoid ellipse plane".
   
   ALGORITHM OVERVIEW
   
   These are the basic steps taken in computing a wrap path around an
   ellipsoid:
   
   1. Find the intersections of the line segment connecting the two
      muscle points (p1 & p2) with the ellipsoid.  Call the intersection
      points r1 & r2.  We sometimes refer to the line segment r1->r2 that
      passes through the ellipsoid as the "muscle line".
   
   2. Find a third point, c1, to use with r1 and r2 to define a "wrapping
      plane".  Finding a good c1 such that it creates a desirable (i.e.
      short) wrapping path turns out to be one of the most difficult steps.
      This is described in more detail below.
   
   3. Find two "tangent points" on the ellipsoid surface.  These points
      must lie in the wrapping plane found in step 2, and create line
      segments that are tangent to the ellipsoid when connected to their
      respective muscle end-points.  The routine calc_r() is called to find
      each tangent point.  calc_r() takes a muscle end-point and a point
      on the ellipsoid surface as input.  It adjusts the surface point to
      become a tangent point.  Since there are two possible tangent points
      in the wrapping plane, calc_r() uses the initial position of the
      surface point to determine which direction around the ellipsoid
      to search for a tangent point.  Therefore it is important to pass
      a good surface point into calc_r() (i.e. one that directs it to
      search in the direction of the shortest wrapping path).
   
   4. Connect the two tangent points with a series of line segments.  The
      routine dell() does this.
   
   CHOOSING A WRAPPING PLANE

   Choosing the plane that the wrapping path lies in ended up taking more
   effort than we expected.  Also, it is likely that there are better
   solutions than the one we ended up with.  This section describes our
   current solution, as well as some that didn't work out so well.

   (1) The Frans Solution:  The method used in Frans' contributed code was
       to pick the coordinate axis that is most parallel to the muscle line,
       then determine the point along the muscle line where that coordinate axis'
       value is zero (i.e. if the muscle line is most parallel to the x-axis,
       then determine the point where the muscle line intersects the yz-plane).
       This point is called "sv".  Pass sv to the pt_to_ellipsoid() routine
       to find the closest point on the ellipsoid surface, c1.

       This solution works well in most cases because it takes advantage of the
       special-case handling in pt_to_ellipsoid() that it triggered because
       sv is always on one of the coordinate planes.  In this situation
       pt_to_ellipsoid() discards one dimension and solves the 2d pt-to-ellipse
       problem which always returns a c1 on one of the ellipsoid's axis-
       aligned "equator" ellipses.  When the muscle line is mostly parallel
       to a major axis, this choice of c1 works well.

       The main problem with the Frans solution occurs when the muscle line
       approaches a 45-degree angle with its most-parallel axis, and then
       switches axes that it is most parallel to.  This can cause large jumps
       in muscle wrapping path.

   (2) The Fan Solution:  We looked closely at the behavior of pt_to_ellipsoid().
       If you call pt_to_ellipsoid() repeatedly, passing input points all along
       the length of the muscle line, you get an interesting "fan" of vectors
       from the muscle line to the ellipsoid surface.  Depending on the
       orientation of the muscle line, this fan can be smoothly continuous (much
       like an oriental fan) or sharply discontinuous (i.e. half the fan blades
       pointing forward along the muscle line, the other half pointing 180-degrees
       in the opposite direction).

       We tried using the fan in various ways to deliver a good c1 point.
       Some of the things that we tried were:
         
         a. binary searching the fan for the most perpendicular vector,
         b. choosing the longest vector,
         c. averaging all the vectors together.

       All of these solutions have problems when the discontinuity of the
       fan becomes very sharp, however we noticed that the Frans solution
       performs well for muscle line orintations that generate sharp
       discontinuities in the fan.  Therefore, our saving grace was that the
       fan and Frans solutions appear to compliment each other.  Each one
       works well where the other one fails.

   (3) Our Solution -- Frans + Fan Hybrid:  What we ended up with is
       a solution that first computes the Frans result, then evauates its
       "strength".  A Frans result is considered strong if the muscle line is
       very parallel to one of the major axes.  As the muscle line moves toward
       45-degrees from its most parallel axis, the strength of the Frans result
       is gradually reduced. Any major axis with an angle of 45 degrees or more
       from the muscle line has a Frans result strength of zero.

       When the strength of the Frans result dips below a certain threshold, we
       begin using fan solution 'c' (averaging all fan vectors) to influence the
       result.  We gradually fade from Frans to Fan solutions.  At 45 degrees the
       Fan solution momentarily takes over completely just as the Frans solution
       is switching its most-parallel major axes.
       
       Another issue that we had to address was the situation where a Frans
       solution generated an sv point that was outside of the ellipsoid.  This
       created several problems.  Therefore we additionally reduce the strength
       of a Frans result as its sv point approaches the ends of the r1->r2
       muscle line.
   
   (4) The Midpoint Solution:  For a while we used a solution that simply picked
       the midpoint of the muscle line as the sv point.  This worked well enough
       in most cases, but suffered when the fan became sharply discontinuous.
       In this case the resulting c1 point could end up being nearly coincident
       with either r1 or r2, making it impossible to create a wrapping plane
       from the three points.  Although RIC used several versions of SIMM with
       the midpoint solutions, we ultimately replaced it with the hybrid solution
       described above.
   
   CONSTRAINING WRAPPING TO ONE HALF OF THE ELLIPSOID
   
   FINDING THE CLOSEST POINT ON AN ELLIPSOID -- SPECIAL CASES
   
*******************************************************************************/

#ifdef WRAP_LIB
#include <universal.h>
#else
#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "wefunctions.h"
#endif


/*************** DEFINES (for this file only) *********************************/
#define E_INSIDE_RADIUS   -1
#define E_NO_WRAP 0
#define E_WRAPPED 1
#define E_MANDATORY_WRAP 2

#define ELLIPSOID_TOLERANCE_1 1e-4     /* tolerance for pt_to_ellipsoid() special case detection */
#define ELLIPSOID_TINY        0.00000001
#define MU_BLEND_MIN          0.7073   /* 100% fan (must be greater than cos(45)!) */
#define MU_BLEND_MAX          0.9      /* 100% Frans */
#define NUM_FAN_SAMPLES       300      /* IMPORTANT: larger numbers produce less jitter */
#define NUM_DISPLAY_SAMPLES   30
#define N_STEPS               16
#define SV_BOUNDARY_BLEND     0.3

#define PREVENT_PT_TO_ELLIPSOID_SPECIAL_CASE_HANDLING 0

#define _UNFACTOR \
   for (i = 0; i < 3; i++) \
   { \
      p1[i] /= factor; \
      p2[i] /= factor; \
      m[i]  /= factor; \
      a[i]  /= factor; \
      r1[i] /= factor; \
      r2[i] /= factor; \
   }

#define PT_TO_ELLIPSOID(_PT, _M, _A, _P1A) \
          pt_to_ellipsoid(_A[0],_A[1],_A[2], \
                          _PT[0],_PT[1],_PT[2], \
                          &_P1A[0],&_P1A[1],&_P1A[2], -1)

#define PT_TO_ELLIPSOID_2(_PT, _M, _A, _P1A, _SPECIAL_CASE_AXIS) \
          pt_to_ellipsoid(_A[0],_A[1],_A[2], \
                          _PT[0],_PT[1],_PT[2], \
                          &_P1A[0],&_P1A[1],&_P1A[2], _SPECIAL_CASE_AXIS)
                          

/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static const float red[] = { 1, 0, 0 }, green[] = { 0, 1, 0 }, blue[] = { 0, 0, 1 };
static const float cyan[] = { 0, 1, 1 }, magenta[] = { 1, 0, 1 }, yellow[] = { 1, 1, 0 };
static const float white[] = { 1, 1, 1 }, pink[] = { 1, 0.5, 0.5 };
static const float* fan_color = white;
static double factor, sv[3], c1[3];



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void ellipsoid_normal_at_pt(const double m[],  const double a[],
                                   const double pt[], double n[], double nLength);

static double pt_to_ellipse(double a, double b, double u, double v, double* x, double* y);

static double pt_to_ellipsoid(double a, double b, double c,
                              double u, double v, double w,
                              double* x, double* y, double* z, int specialCaseAxis);

static int calc_r(double p1e, double r1[], double p1[], double m[], double a[],
              double vs[], double vs4);

static void dell(double r1[], double r2[], double m[], double a[], double vs[], 
             double vs4, SBoolean far_side_wrap, double *afst, double** wrap_pts, int* num_wrap_pts);

/**** for constraints ****/
static double pt_to_ellipse2(double a, double b, double u, double v, double* x, double* y);

static double pt_to_ellipsoid2(double a, double b, double c,
                              double u, double v, double w,
                              double* x, double* y, double* z, int specialCaseAxis);
//void cons_ellipsoid_normal_at_pt(const double m[],  const double a[],
//                                   const double pt[], double n[], double nLength);


/* use pt_to_ellipsoid2: which has a much smaller tolerance when checking
 * if values are equal to zero.  For some reason this helps, especially with
 * points near axes. - for constraint calculations */
double calc_distance_to_ellipsoid(double p1[], double radii[], double projpt[])
{
   double distance;
   distance = pt_to_ellipsoid2(radii[0], radii[1], radii[2], p1[0], p1[1], p1[2],
      &projpt[0], &projpt[1], &projpt[2], -1);
   return distance;
}

void cons_ellipsoid_normal_at_pt(const double m[],  const double a[],
                                   const double pt[], double n[], double desired_length)
{
//   ellipsoid_normal_at_pt(m, a, pt, n, nLength);
/* -------------------------------------------------------------------------
   ellipsoid_normal_at_pt - return the normal vector to the ellipsoid at the
     specified point.
   m - ellipsoid origin
   a - ellipsoid radii
---------------------------------------------------------------------------- */
    double x = pt[0] - m[0];
    double y = pt[1] - m[1];
    double z = pt[2] - m[2], length, ratio;

    n[0] = 2 * x / SQR(a[0]);
    n[1] = 2 * y / SQR(a[1]);
    n[2] = 2 * z / SQR(a[2]);

    length = VECTOR_MAGNITUDE(n);

    ratio = desired_length / length;

    n[0] *= ratio;
    n[1] *= ratio;
    n[2] *= ratio;
}

/**** ****/

/* -------------------------------------------------------------------------
   calc_line_intersect_ellipsoid - calculate an optimal wrapping path about
      the specified ellipsoid.
---------------------------------------------------------------------------- */
int calc_line_intersect_ellipsoid (
    double      p1[3],      /* input:  muscle point 1       */
    double      p2[3],      /* input:  muscle point 2       */
    double      m[3],       /* input:  ellipsoid origin (0,0,0) */
    double      a[3],       /* input:  ellipsoid xyz radii      */
    double*     rlen,       /* output: length of surface wrap   */
    double      r1[],       /* output: wrap tangent point 1     */
    double      r2[],       /* output: wrap tangent point 2     */
    double**    wrap_pts,   /* output: intermediate surface wrap pts*/
    int*        num_wrap_pts,   /* output: number of intermediate pts   */
    int     wrap_axis,  /* input:  constraint axis      */
    int     wrap_sign,  /* input:  constraint direction     */
    dpMuscleWrapStruct* mwrap, /* in/out: muscle wrap structure */
    int*        p_flag,     /* output: did wrapping occur?      */
    dpWrapObject*   wo)
{
   int i,j, bestMu;
   
   double p1p2[3], p1m[3], p2m[3], ppm, f1[3], f2[3], aa, bb, cc, disc, l1, l2,
          vs[3], p1e, p2e, p1c1[3], vs4, dist, t[3], fanWeight = DBL_MIN;

   double r1r2[3], t_sv[3][3], t_c1[3][3];
   double mu[3];
   
   SBoolean far_side_wrap = no;
      
   *p_flag       = TRUE;
   *num_wrap_pts = 0;
   *wrap_pts     = NULL;
   
#if VISUAL_WRAPPING_DEBUG
   enable_debug_shapes(yes);

   wo->num_debug_glyphs = 0;
#endif

   /* This algorithm works best if the coordinates (p1, p2, m, a) are all somewhat
    * close to 1.0. So use the ellipsoid dimensions to calculate a multiplication
    * factor that will be applied to all of the coordinates. You want to use just
    * the ellipsoid dimensions because they do not change from one call to the
    * next. You don't want the factor to change because the algorithm uses
    * some vectors (r1, r2, c1) from the previous call.
    */
   factor = 3.0 / (a[0] + a[1] + a[2]);

   for (i = 0; i < 3; i++)
   {
      p1[i] *= factor;
      p2[i] *= factor;
      m[i]  *= factor;
      a[i]  *= factor;
   }

   p1e = -1.0;
   p2e = -1.0;

   for (i = 0; i < 3;i++)
   {
      p1e += SQR((p1[i] - m[i]) / a[i]);
      p2e += SQR((p2[i] - m[i]) / a[i]);
   }

   /* check if p1 and p2 are inside the ellipsoid */
   if (p1e < -0.0001 || p2e < -0.0001)
   {
      /* p1 or p2 is inside the ellipsoid */
      *p_flag = FALSE;
      *rlen   = 0.0;
      
      _UNFACTOR; /* transform back to starting coordinate system */
      
      return E_INSIDE_RADIUS;
   }

   MAKE_3DVECTOR21(p1, p2, p1p2);
   MAKE_3DVECTOR21(p1, m, p1m);
   normalize_vector(p1m, p1m);
   MAKE_3DVECTOR21(p2, m, p2m);
   normalize_vector(p2m, p2m);

   ppm = DOT_VECTORS(p1m, p2m) - 1.0;   /* angle between p1->m and p2->m: -2.0 to 0.0 */

   if (fabs(ppm) < 0.0001)
   {
      /* vector p1m and p2m are colinear */
      *p_flag = FALSE;
      *rlen   = 0.0;
      
      _UNFACTOR; /* transform back to starting coordinate system */
      
      return E_NO_WRAP;
   }

   /* check if the line through p1 and p2 intersects the ellipsoid */
   for (i = 0; i < 3;i++)
   {
      f1[i] = p1p2[i] / a[i];
      f2[i] = (p2[i] - m[i]) / a[i];
   }
   aa = DOT_VECTORS(f1, f1);
   bb = 2.0 * DOT_VECTORS(f1, f2);
   cc = DOT_VECTORS(f2, f2) - 1.0;
   disc = SQR(bb) - 4.0 * aa * cc;

   if (disc < 0.0)
   {
      /* no intersection */
      *p_flag = FALSE;
      *rlen = 0.0;
      
      _UNFACTOR; /* transform back to starting coordinate system */
      
      return E_NO_WRAP;
   }

   l1 = (-bb + sqrt(disc)) / (2.0 * aa);
   l2 = (-bb - sqrt(disc)) / (2.0 * aa);
   
   if ( ! (0.0 < l1 && l1 < 1.0) || ! (0.0 < l2 && l2 < 1.0) )
   {
      /* no intersection */
      *p_flag = FALSE;
      *rlen   = 0.0;
      
      _UNFACTOR; /* transform back to starting coordinate system */
      
      return E_NO_WRAP;
   }

   /* r1 & r2: intersection points of p1->p2 with the ellipsoid */
   for (i = 0; i < 3; i++)
   {
      r1[i] = p2[i] + l1 * p1p2[i];
      r2[i] = p2[i] + l2 * p1p2[i];
   }
   
#if VISUAL_WRAPPING_DEBUG
   add_debug_line(wo, factor, r1, r2, 2.0, "", "", yellow); /* the "muscle line" */
#endif

   /* ==== COMPUTE WRAPPING PLANE (begin) ==== */
   
   MAKE_3DVECTOR21(r2, r1, r1r2);

   /* (1) Frans technique: choose the most parallel coordinate axis, then set
    * 'sv' to the point along the muscle line that crosses the plane where
    * that major axis equals zero.  This takes advantage of the special-case
    * handling in pt_to_ellipsoid() that reduces the 3d point-to-ellipsoid
    * problem to a 2d point-to-ellipse problem.  The 2d case returns a nice
    * c1 in situations where the "fan" has a sharp discontinuity.
    */
   normalize_vector(p1p2, mu);
      
   for (i = 0; i < 3; i++)
   {
      mu[i] = fabs(mu[i]);
      
      t[i] = (m[i] - r1[i]) / r1r2[i];
      
      for (j = 0; j < 3; j++)
         t_sv[i][j] = r1[j] + t[i] * r1r2[j];
      
      PT_TO_ELLIPSOID_2(t_sv[i], m, a, t_c1[i], i);
   }

   /* pick most parallel major axis */
   for (bestMu = 0, i = 1; i < 3; i++)
      if (mu[i] > mu[bestMu])
         bestMu = i;

   if (mwrap->wrap_algorithm == WE_HYBRID_ALGORITHM ||
       mwrap->wrap_algorithm == WE_AXIAL_ALGORITHM  ||
       mwrap->wrap_algorithm == WE_FAN_ALGORITHM)
   {
      if (mwrap->wrap_algorithm == WE_HYBRID_ALGORITHM && mu[bestMu] > MU_BLEND_MIN)
      {
         /* If Frans' technique produces an sv that is not within the r1->r2
          * line segment, then that means that sv will be outside the ellipsoid.
          * This can create an sv->c1 vector that points roughly 180-degrees
          * opposite to the fan solution's sv->c1 vector.  This creates problems
          * when interpolating between the Frans and fan solutions because the
          * interpolated c1 can become colinear to the muscle line during
          * interpolation.  Therefore we detect Frans-solution sv points near
          * the ends of r1->r2 here, and fade out the Frans result for them.
          */

         double s = 1.0;
         
         if (t[bestMu] < 0.0 || t[bestMu] > 1.0)
            s = 0.0;
         
         else if (t[bestMu] < SV_BOUNDARY_BLEND)
            s = t[bestMu] / SV_BOUNDARY_BLEND;
         
         else if (t[bestMu] > (1.0 - SV_BOUNDARY_BLEND))
            s = (1.0 - t[bestMu]) / SV_BOUNDARY_BLEND;
         
         if (s < 1.0)
            mu[bestMu] = MU_BLEND_MIN + s * (mu[bestMu] - MU_BLEND_MIN);
#if 0
         fprintf(stderr, mu[bestMu] < MU_BLEND_MAX ? "mu{%.3f} " : "mu[%.3f] ", mu[bestMu]);
         fprintf(stderr, s < 1.0 ? "t{%.2f}\n" : "t %.2f\n", s);
#endif         
      }
      
      if (mwrap->wrap_algorithm == WE_AXIAL_ALGORITHM || mu[bestMu] > MU_BLEND_MIN)
      {
         /* if the Frans solution produced a strong result, copy it into
          * sv and c1.
          */
         for (i = 0; i < 3; i++)
         {
            c1[i] = t_c1[bestMu][i];
            sv[i] = t_sv[bestMu][i];
         }
#if VISUAL_WRAPPING_DEBUG && 0
         if (mu[bestMu] < MU_BLEND_MAX && mwrap->wrap_algorithm == WE_HYBRID_ALGORITHM)
            add_debug_line(wo, factor, t_sv[bestMu], t_c1[bestMu], 1.0, "", "Frans", blue);
#endif
      }
#if 0 
      fprintf(stderr, bestMu == 0 ? "[%.3f] "  : "%.3f ",  mu[0]);
      fprintf(stderr, bestMu == 1 ? "[%.3f] "  : "%.3f ",  mu[1]);
      fprintf(stderr, bestMu == 2 ? "[%.3f]\n" : "%.3f\n", mu[2]);
#endif         
   
      if (mwrap->wrap_algorithm == WE_FAN_ALGORITHM ||
          mwrap->wrap_algorithm == WE_HYBRID_ALGORITHM && mu[bestMu] < MU_BLEND_MAX)
      {
         /* (2) Fan technique: sample the fan at fixed intervals and average the
          * fan "blade" vectors together to determine c1.  This only works when
          * the fan is smoothly continuous.  The sharper the discontinuity, the
          * more jumpy c1 becomes.
          */
         double v_sum[3] = {0,0,0};
         
         for (i = 0; i < 3; i++)
            t_sv[2][i] = r1[i] + 0.5 * r1r2[i];
         
         for (i = 1; i < NUM_FAN_SAMPLES - 1; i++)
         {
            double v[3], tt = (double) i / NUM_FAN_SAMPLES;
         
            for (j = 0; j < 3; j++)
               t_sv[0][j] = r1[j] + tt * r1r2[j];

            PT_TO_ELLIPSOID(t_sv[0], m, a, t_c1[0]);
      
            MAKE_3DVECTOR21(t_c1[0], t_sv[0], v);

            normalize_vector(v, v);

            /* add sv->c1 "fan blade" vector to the running total */
            for (j = 0; j < 3; j++)
               v_sum[j] += v[j];

#if VISUAL_WRAPPING_DEBUG && 0
            if (i % (NUM_FAN_SAMPLES / NUM_DISPLAY_SAMPLES) == 0)
               add_debug_line(wo, factor, t_sv[0], t_c1[0], 1.0, NULL, NULL, fan_color);
#endif
         }
         /* use vector sum to determine c1 */
         normalize_vector(v_sum, v_sum);
         
         for (i = 0; i < 3; i++)
           t_c1[0][i] = t_sv[2][i] + v_sum[i];
         
         if (mwrap->wrap_algorithm == WE_FAN_ALGORITHM || mu[bestMu] <= MU_BLEND_MIN)
         {
            PT_TO_ELLIPSOID(t_c1[0], m, a, c1);
            
            for (i = 0; i < 3; i++)
               sv[i] = t_sv[2][i];
         
            fanWeight = 1.0;
         }
         else
         {
            double tt = (mu[bestMu] - MU_BLEND_MIN) / (MU_BLEND_MAX - MU_BLEND_MIN);
            
            double oneMinusT = 1.0 - tt;
            
            PT_TO_ELLIPSOID(t_c1[0], m, a, t_c1[1]);
            
            for (i = 0; i < 3; i++)
            {
               t_c1[2][i] = tt * c1[i] + oneMinusT * t_c1[1][i];
               
               sv[i] = tt * sv[i] + oneMinusT * t_sv[2][i];
            }
            PT_TO_ELLIPSOID(t_c1[2], m, a, c1);
            
#if VISUAL_WRAPPING_DEBUG && 0
            add_debug_line(wo, factor, t_sv[2], t_c1[1], 1.0, "", "fan", blue);
#endif
            fanWeight = oneMinusT;
         }
      }
   }
   else /* mwrap->wrap_algorithm == WE_MIDPOINT_ALGORITHM */
   {
      for (i = 0; i < 3; i++)
         sv[i] = r1[i] + 0.5 * (r2[i] - r1[i]);
      
      PT_TO_ELLIPSOID(sv, m, a, c1);
   }

#if VISUAL_WRAPPING_DEBUG
   add_debug_line(wo, factor, sv, c1, 2.0, "", "c1", green);
#endif
   /* ==== COMPUTE WRAPPING PLANE (end) ==== */

   /* The old way of initializing r1 used the intersection point
    * of p1p2 and the ellipsoid. This caused the muscle path to
    * "jump" to the other side of the ellipsoid as sv[] came near
    * a plane of the ellipsoid. It jumped to the other side while
    * c1[] was still on the first side. The new way of initializing
    * r1 sets it to c1 so that it will stay on c1's side of the
    * ellipsoid.
    */
   {
      SBoolean use_c1_to_find_tangent_pts = yes;
   
      if (mwrap->wrap_algorithm == WE_AXIAL_ALGORITHM)
         use_c1_to_find_tangent_pts = (SBoolean) (t[bestMu] > 0.0 && t[bestMu] < 1.0);
   
      if (use_c1_to_find_tangent_pts)
         for (i = 0; i < 3; i++)
            r1[i] = r2[i] = c1[i];
   }
   
   /* if wrapping is constrained to one half of the ellipsoid,
    * check to see if we need to flip c1 to the active side of
    * the ellipsoid.
    */
   if (wrap_sign != 0)
   {
      dist = c1[wrap_axis] - m[wrap_axis];
      
      if (DSIGN(dist) != wrap_sign)
      {
         double orig_c1[3];
         
         for (i = 0; i < 3; i++)
            orig_c1[i] = c1[i];
            
         c1[wrap_axis] = - c1[wrap_axis];
         
         for (i = 0; i < 3; i++)
            r1[i] = r2[i] = c1[i];

#if VISUAL_WRAPPING_DEBUG
         add_debug_line(wo, factor, sv, c1, 2.0, "", "c1\'", green);
#endif
         
         if (fanWeight == DBL_MIN)
            fanWeight = 1.0 - (mu[bestMu] - MU_BLEND_MIN) / (MU_BLEND_MAX - MU_BLEND_MIN);
         
         if (fanWeight > 1.0)
            fanWeight = 1.0;
         
         if (fanWeight > 0.0)
         {
            double tc1[3], bisection = (orig_c1[wrap_axis] + c1[wrap_axis]) / 2.0;
               
            c1[wrap_axis] = c1[wrap_axis] + fanWeight * (bisection - c1[wrap_axis]);
            
            for (i = 0; i < 3; i++)
               tc1[i] = c1[i];
            
            PT_TO_ELLIPSOID(tc1, m, a, c1);

#if VISUAL_WRAPPING_DEBUG
            add_debug_line(wo, factor, sv, c1, 2.0, "", "c1\'\'", green);
#endif
         }
      }
   }

   /* use p1, p2, and c1 to create parameters for the wrapping
    * plane.
    */
   MAKE_3DVECTOR21(p1, c1, p1c1);
   cross_vectors(p1p2, p1c1, vs);
   normalize_vector(vs, vs);
 
   vs4 = - DOT_VECTORS(vs, c1);
   
   /* find r1 & r2 by starting at c1 moving toward p1 & p2 */
#if VISUAL_WRAPPING_DEBUG && 1
   add_debug_line(wo, factor, sv, r1, 3.0, "", "r1\'", white);
   add_debug_line(wo, factor, sv, r2, 3.0, "", "r2\'", white);
#endif

   calc_r(p1e, r1, p1, m, a, vs, vs4);
   calc_r(p2e, r2, p2, m, a, vs, vs4);
   
#if VISUAL_WRAPPING_DEBUG && 1
   add_debug_line(wo, factor, sv, r1, 3.0, "", "", pink);
   add_debug_line(wo, factor, sv, r2, 3.0, "", "", pink);
#endif

   /* create a series of line segments connecting r1 & r2 along the
    * surface of the ellipsoid.
    */
  calc_wrap_path:
   dell(r1, r2, m, a, vs, vs4, far_side_wrap, rlen, wrap_pts, num_wrap_pts);

   if (wrap_sign != 0 && *num_wrap_pts > 2 && ! far_side_wrap)
   {
      double r1p1[3], r2p2[3], r1w1[3], r2w2[3];
      
      double *w1 = &(*wrap_pts)[3];
      double *w2 = &(*wrap_pts)[(*num_wrap_pts - 2) * 3];
      
      /* check for wrong-way wrap by testing angle of first and last
       * wrap path segments:
       */
      MAKE_3DVECTOR(r1, p1, r1p1);
      MAKE_3DVECTOR(r1, w1, r1w1);
      MAKE_3DVECTOR(r2, p2, r2p2);
      MAKE_3DVECTOR(r2, w2, r2w2);

      normalize_vector(r1p1, r1p1);
      normalize_vector(r1w1, r1w1);
      normalize_vector(r2p2, r2p2);
      normalize_vector(r2w2, r2w2);

      if (DOT_VECTORS(r1p1, r1w1) > 0.0 || DOT_VECTORS(r2p2, r2w2) > 0.0)
      {
         /* NOTE: I added the ability to call dell() a 2nd time in this
          *  situation to force a far-side wrap instead of aborting the
          *  wrap.   -- KMS 9/3/99
          */
         far_side_wrap = yes;
         
         goto calc_wrap_path;
      }
   }

   /* store the center point and tangent points for next time */
   for (i = 0; i < 3; i++)
   {
      mwrap->c[i]  = c1[i];
      mwrap->r1[i] = r1[i];
      mwrap->r2[i] = r2[i];
   }

   /* unfactor the output coordinates */
   *rlen /= factor;
  
   for (i = 0; i < *num_wrap_pts * 3; i++)
      (*wrap_pts)[i] /= factor;

   _UNFACTOR; /* transform back to starting coordinate system */
   
   return E_MANDATORY_WRAP;

} /* calc_line_intersect_ellipsoid */

/* -------------------------------------------------------------------------
   ellipsoid_normal_at_pt - return the normal vector to the ellipsoid at the
     specified point.
---------------------------------------------------------------------------- */
static void ellipsoid_normal_at_pt (const double m[], const double a[],
                                    const double pt[], double n[], double desired_length)
{
    double x = pt[0] - m[0];
    double y = pt[1] - m[1];
    double z = pt[2] - m[2], length, ratio;

    n[0] = 2 * x / SQR(a[0]);
    n[1] = 2 * y / SQR(a[1]);
    n[2] = 2 * z / SQR(a[2]);

    length = VECTOR_MAGNITUDE(n);

    ratio = desired_length / length;

    n[0] *= ratio;
    n[1] *= ratio;
    n[2] *= ratio;
}

/* -------------------------------------------------------------------------
   pt_to_ellipse - this routine is courtesy of Dave Eberly
      www.magic-software.com
   
   Input:   Ellipse (x/a)^2+(y/b)^2 = 1, point (u,v).
  
   Output:  Closest point (x,y) on ellipse to (u,v), function returns
            the distance sqrt((x-u)^2+(y-v)^2).
---------------------------------------------------------------------------- */
static double pt_to_ellipse (double a, double b, double u, double v, double* x, double* y)
{
    /* Graphics Gems IV algorithm for computing distance from point to
     * ellipse (x/a)^2 + (y/b)^2 = 1.  The algorithm as stated is not stable
     * for points near the coordinate axes.  The first part of this code
     * handles those points separately.
     */
    double a2 = a*a, b2 = b*b;
    double u2 = u*u, v2 = v*v;
    double a2u2 = a2*u2, b2v2 = b2*v2;
    double dx, dy, xda, ydb;
    int i, which;
    double t, P, Q, P2, Q2, f, fp;
    
    SBoolean nearXOrigin = (SBoolean) EQUAL_WITHIN_ERROR(0.0,u);
    SBoolean nearYOrigin = (SBoolean) EQUAL_WITHIN_ERROR(0.0,v);
    
    fan_color = pink;
    
    /* handle points near the coordinate axes */
    if (nearXOrigin && nearYOrigin)
    {
        fan_color = yellow;

        if (a < b)
        {
            *x = (u < 0.0 ? -a : a);
            *y = v;
            return a;
        } else {
            *x = u;
            *y = (v < 0.0 ? -b : b);
            return b;
        }
    }

    if (nearXOrigin)  /* u == 0 */
    {
        fan_color = red;

        if ( a >= b || fabs(v) >= b - a2/b )
        {
            *x = u;
            *y = ( v >= 0 ? b : -b );
            dy = *y - v;
            return fabs(dy);
        }
        else
        {
            *y = b2 * v / (b2-a2);
            dy = *y - v;
            ydb = *y / b;
            *x = a * sqrt(fabs(1 - ydb*ydb));
            return sqrt(*x * *x + dy*dy);
        }
    }

    if (nearYOrigin)  /* v == 0 */
    {
        fan_color = red;

        if ( b >= a || fabs(u) >= a - b2/a )
        {
            *x = ( u >= 0 ? a : -a );
            dx = *x - u;
            *y = v;
            return fabs(*x - u);
        }
        else
        {
            *x = a2 * u / (a2-b2);
            dx = *x - u;
            xda = *x / a;
            *y = b * sqrt(fabs(1 - xda*xda));
            return sqrt(dx*dx + *y * *y);
        }
    }

    /* initial guess */
    if ( (u/a)*(u/a) + (v/b)*(v/b) < 1.0 )
    {
        which = 0;
        t = 0.0;
    }
    else
    {
        double max = a;
        
        which = 1;
        
        if ( b > max )
            max = b;

        t = max * sqrt(u*u + v*v);
    }

    for (i = 0; i < 64; i++)
    {
        P = t+a2;
        P2 = P*P;
        Q = t+b2;
        Q2 = Q*Q;
        f = P2*Q2 - a2u2*Q2 - b2v2*P2;
        
         if ( fabs(f) < 1e-09 )
            break;

        fp = 2.0 * (P*Q*(P+Q) - a2u2*Q - b2v2*P);
        t -= f / fp;
    }
#if 0
    if (i >= 64)
        fprintf(stderr, "2D: i: %d, f: %.0e\n", i, fabs(f));
#endif

    *x = a2 * u / P;
    *y = b2 * v / Q;
    dx = *x - u;
    dy = *y - v;

    return sqrt(dx*dx + dy*dy);

} /* pt_to_ellipse */

/* -------------------------------------------------------------------------
   pt_to_ellipsoid - this routine is courtesy of Dave Eberly
      www.magic-software.com
   
   Input:   Ellipsoid (x/a)^2+(y/b)^2+(z/c)^2 = 1, point (u,v,w).
  
   Output:  Closest point (x,y,z) on ellipsoid to (u,v,w), function returns
            the distance sqrt((x-u)^2+(y-v)^2+(z-w)^2).
---------------------------------------------------------------------------- */
static double pt_to_ellipsoid (double a, double b, double c,
                                 double u, double v, double w,
                               double* x, double* y, double* z,
                               int specialCaseAxis)
{
    /* Graphics Gems IV algorithm for computing distance from point to
     * ellipsoid (x/a)^2 + (y/b)^2 +(z/c)^2 = 1.  The algorithm as stated
     * is not stable for points near the coordinate planes.  The first part
     * of this code handles those points separately.
     */
    int i,j;
    
#if PREVENT_PT_TO_ELLIPSOID_SPECIAL_CASE_HANDLING
    /* handle points near the coordinate planes by preventing them from
     * getting too close to the coordinate planes.
     */
    if (EQUAL_WITHIN_ERROR(u,0.0))
        u = (u < 0.0 ? - TINY_NUMBER : TINY_NUMBER);

    if (EQUAL_WITHIN_ERROR(v,0.0))
        v = (v < 0.0 ? - TINY_NUMBER : TINY_NUMBER);

    if (EQUAL_WITHIN_ERROR(w,0.0))
        w = (w < 0.0 ? - TINY_NUMBER : TINY_NUMBER);

#else    
    /* handle points near the coordinate planes by reducing the problem
     * to a 2-dimensional pt-to-ellipse.
     *
     * if uvw is close to more than one coordinate plane,  pick the 2d
     * elliptical cross-section with the narrowest radius.
     */
    if (specialCaseAxis < 0)
    {
       double abc[3], uvw[3], minEllipseRadiiSum = FLT_MAX;
       
       abc[0] = a; abc[1] = b; abc[2] = c;
       uvw[0] = u; uvw[1] = v; uvw[2] = w;

       for (i = 0; i < 3; i++)
       {
           if (EQUAL_WITHIN_ERROR(0.0,uvw[i]))
           {
               double ellipseRadiiSum = 0;

               for (j = 0; j < 3; j++)
                   if (j != i)
                       ellipseRadiiSum += abc[j];

               if (minEllipseRadiiSum > ellipseRadiiSum)
               {
                   specialCaseAxis = i;
                   minEllipseRadiiSum = ellipseRadiiSum;
               }
           }
       }
    }
    if (specialCaseAxis == 0) /* use elliptical cross-section in yz plane */
    {
       *x = u;
       //printf(" yz plane ");
       return pt_to_ellipse(b, c, v, w, y, z);
    }
    if (specialCaseAxis == 1) /* use elliptical cross-section in xz plane */
    {
       *y = v;
       //printf(" xz plane ");
       return pt_to_ellipse(c, a, w, u, z, x);
    }
    if (specialCaseAxis == 2) /* use elliptical cross-section in xy plane */
    {
       *z = w;
       //printf(" xy plane ");
       return pt_to_ellipse(a, b, u, v, x, y);
    }
#endif

    fan_color = white;

    /* if we get to here, then the point is not close to any of the major planes,
     * so we can solve the general case.
     */
    {
        double a2 = a*a, b2 = b*b, c2 = c*c;
        double u2 = u*u, v2 = v*v, w2 = w*w;
        double a2u2 = a2*u2, b2v2 = b2*v2, c2w2 = c2*w2;
        double dx, dy, dz, t, f;

        /* initial guess */
        if ( (u/a)*(u/a) + (v/b)*(v/b) + (w/c)*(w/c) < 1.0 )
        {
            t = 0.0;
        }
        else
        {
            double max = a;
        
            if ( b > max )
                max = b;
            if ( c > max )
                max = c;

            t = max*sqrt(u*u+v*v+w*w);
        }

        for (i = 0; i < 64; i++)
        {
            double P = t+a2, P2 = P*P;
            double Q = t+b2, Q2 = Q*Q;
            double R = t+c2, _R2 = R*R;
            double PQ, PR, QR, PQR, fp;

            f = P2*Q2*_R2 - a2u2*Q2*_R2 - b2v2*P2*_R2 - c2w2*P2*Q2;
        
            if ( fabs(f) < 1e-09 )
            {
                *x = a2 * u / P;
                *y = b2 * v / Q;
                *z = c2 * w / R;
            
                dx = *x - u;
                dy = *y - v;
                dz = *z - w;
            
                return sqrt(dx*dx+dy*dy+dz*dz);
            }

            PQ = P*Q;
            PR = P*R;
            QR = Q*R;
            PQR = P*Q*R;
            fp = 2.0*(PQR*(QR+PR+PQ)-a2u2*QR*(Q+R)-b2v2*PR*(P+R)-c2w2*PQ*(P+Q));
        
            t -= f/fp;
        }
#if 0
        if (i >= 64)
            fprintf(stderr, "3D: i: %d, f: %.0e\n", i, fabs(f));
#endif
    }
    return -1.0;

} /* pt_to_ellipsoid */

/* -------------------------------------------------------------------------
   calc_r - this routine adjusts a point (r1) such that the point remains in
     a specified plane (vs, vs4), and creates a TANGENT line segment connecting
     it to a specified point (p1) outside of the ellipsoid.
   
   Input:
     p1e   ellipsoid parameter for 'p1'?
     r1    point to be adjusted to satisfy tangency-within-a-plane constraint
     p1    point outside of ellipsoid
     m     ellipsoid origin
     a     ellipsoid axis
     vs    plane vector
     vs4   plane coefficient
   
   Output:
     r1    tangent point on ellipsoid surface and in vs plane
---------------------------------------------------------------------------- */
static int calc_r (double p1e, double r1[], double p1[], double m[], double a[],
               double vs[], double vs4)
{
   int i, j, k, nit, nit2, maxit=50, maxit2=1000;
   double nr1[3], d1, v[4], ee[4], ssqo, ssq, pcos, p1r1[3], p1m[3], dedth[4][4];
   double fakt, alpha=0.01, dedth2[4][4], diag[4], ddinv2[4][4], vt[4], dd;

   if (fabs(p1e) < 0.0001)
   {
      for (i = 0; i < 3; i++)
     r1[i] = p1[i];
   }
   else
   {
      for (i = 0; i < 3; i++)
     nr1[i] = 2.0 * (r1[i] - m[i])/(SQR(a[i]));

      d1 = -DOT_VECTORS(nr1, r1);
      ee[0] = DOT_VECTORS(vs, r1) + vs4;
      ee[1] = -1.0;

      for (i = 0; i < 3; i++)
     ee[1] += SQR((r1[i] - m[i]) / a[i]);

      ee[2] = DOT_VECTORS(nr1, r1) + d1;
      ee[3] = DOT_VECTORS(nr1, p1) + d1;

      ssqo = SQR(ee[0]) + SQR(ee[1]) + SQR(ee[2]) + SQR(ee[3]);
      ssq = ssqo;

      nit = 0;

      while ((ssq > ELLIPSOID_TINY) && (nit < maxit))
      {
     nit++;

     for (i = 0; i < 3; i++)
     {
        dedth[i][0] = vs[i];
        dedth[i][1] = 2.0 * (r1[i] - m[i]) / SQR(a[i]);
        dedth[i][2] = 2.0 * (2.0 * r1[i] - m[i]) / SQR(a[i]);
        dedth[i][3] = 2.0 * p1[i] / SQR(a[i]);
     }

     dedth[3][0] = 0.0;
     dedth[3][1] = 0.0;
     dedth[3][2] = 1.0;
     dedth[3][3] = 1.0;
#if 1
     MAKE_3DVECTOR21(p1, r1, p1r1);
     normalize_vector(p1r1, p1r1);

         MAKE_3DVECTOR21(p1, m, p1m);
         normalize_vector(p1m, p1m);

     pcos = DOT_VECTORS(p1r1, p1m);

     if (pcos > 0.1)
        dd = 1.0 - pow(pcos, 100);
     else
#endif
        dd = 1.0;

     for (i = 0; i < 4; i++)
     {
        v[i] = 0.0;
        
        for (j = 0; j < 4; j++)
           v[i] -= dedth[i][j] * ee[j];
     }

     for (i = 0; i < 4; i++)
     {
        for (j = 0; j < 4; j++)
        {
           dedth2[i][j] = 0.0;
           
           for (k = 0; k < 4; k++)
          dedth2[i][j] += dedth[i][k] * dedth[j][k];
        }
     }

     for (i = 0; i < 4; i++)
        diag[i] = dedth2[i][i];

     nit2 = 0;
     
     while ((ssq >= ssqo) && (nit2 < maxit2))
     {
        for (i = 0; i < 4; i++)
           dedth2[i][i] = diag[i] * (1.0 + alpha);

        invert_4x4matrix(dedth2, ddinv2);

        for (i = 0; i < 4; i++)
        {
           vt[i] = 0.0;
           
           for (j = 0; j < 4; j++)
          vt[i] += dd * ddinv2[i][j] * v[j] / 16.0;
        }

        for (i = 0; i < 3; i++)
           r1[i] += vt[i];
     
        d1 += vt[3];

        for (i = 0; i < 3; i++)
           nr1[i] = 2.0 * (r1[i] - m[i])/SQR(a[i]);
        
        ee[0] = DOT_VECTORS(vs, r1) + vs4;
        ee[1] = -1.0;

        for (i = 0; i < 3; i++)
           ee[1] += SQR((r1[i] - m[i])/a[i]);

        ee[2] = DOT_VECTORS(nr1, r1) + d1;
        ee[3] = DOT_VECTORS(nr1, p1) + d1;

        ssqo = ssq;

        ssq = SQR(ee[0]) + SQR(ee[1]) + SQR(ee[2]) + SQR(ee[3]);

        alpha *= 4.0;
        nit2++;
     }

     alpha /= 8.0;

     fakt = 0.5;

     nit2 = 0;
     
     while ((ssq <= ssqo) && (nit2 < maxit2))
     {
        fakt *= 2.0;

        for (i = 0; i < 3; i++)
           r1[i] += vt[i] * fakt;

        d1 += vt[3] * fakt;

        for (i = 0; i < 3; i++)
           nr1[i] = 2.0 * (r1[i] - m[i]) / SQR(a[i]);

        ee[0] = DOT_VECTORS(vs, r1) + vs4;
        ee[1] = -1.0;
        
        for (i=0; i<3; i++)
           ee[1] += SQR((r1[i] - m[i]) / a[i]);

        ee[2] = DOT_VECTORS(nr1, r1) + d1;
        ee[3] = DOT_VECTORS(nr1, p1) + d1;

        ssqo = ssq;     
        
        ssq = SQR(ee[0]) + SQR(ee[1]) + SQR(ee[2]) + SQR(ee[3]);

        nit2++;
     }

     for (i = 0; i < 3; i++)
        r1[i] -= vt[i] * fakt;

     d1 -= vt[3] * fakt;

     for (i=0; i<3; i++)
        nr1[i] = 2.0 * (r1[i] - m[i]) / SQR(a[i]);

     ee[0] = DOT_VECTORS(vs, r1) + vs4;
     ee[1] = -1.0;
     
     for (i = 0; i < 3; i++)
        ee[1] += SQR((r1[i] - m[i]) / a[i]);
     
     ee[2] = DOT_VECTORS(nr1, r1) + d1;
     ee[3] = DOT_VECTORS(nr1, p1) + d1;

     ssq = SQR(ee[0]) + SQR(ee[1]) + SQR(ee[2]) + SQR(ee[3]);
     ssqo = ssq;        
      }
   }   
   return 1;

} /* calc_r */

/* -------------------------------------------------------------------------
   dell - calculate the distance between two points on the surface of an
     ellipsoid over the surface of the ellipsoid.
   
   Input:
     r1, r2:   the points
     m:        center of the ellipsoid
     a:        axes of the ellipsoid
     vs:       orientation plane
   
   Output:
     afst:     the distance over the surface of the ellipsoid
     wrap_pts: line segments connecting r1 to r2 along the ellipsoid surface
---------------------------------------------------------------------------- */
static void dell (double r1[], double r2[], double m[], double a[], 
              double vs[], double vs4, SBoolean far_side_wrap,
              double *afst,
              double** wrap_pts, int* num_wrap_pts)
{
   int i, j, k, l, imax, index, numPts = 101;
   int numSegs = numPts - 1;
   int numInteriorPts = numPts - 2;
   
   double u[3], ux[3], mu, a0[3], ar1[3], ar2[3], phi, dphi, phi0, len,
      r0[3][3], vsy[3], vsz[3], rphi[3][3], t[3], r[3], f1[3], f2[3], dr[3],
      aa, bb, cc, mu3, s[100][3], dv[3], q[3], p[3];

   MAKE_3DVECTOR21(r1, r2, dr);
   len = VECTOR_MAGNITUDE(dr);

   /* If the distance between r1 and r2 is very small, then don't bother
    * calculating 100 wrap points along the surface of the ellipsoid.
    * Just use r1 and r2 as the surface points and return the distance
    * between them as the distance along the ellipsoid.
    */
   if (len < 0.0001)
   {
      *num_wrap_pts = 2;
      *wrap_pts = (double*) simm_malloc(6 * sizeof(double));

      (*wrap_pts)[0] = r1[0];
      (*wrap_pts)[1] = r1[1];
      (*wrap_pts)[2] = r1[2];

      (*wrap_pts)[3] = r2[0];
      (*wrap_pts)[4] = r2[1];
      (*wrap_pts)[5] = r2[2];

      *afst = len;
      return;
   }

   ux[0] = 1.0;
   ux[1] = 0.0;
   ux[2] = 0.0;

   imax = 0;
   
   for (i = 1; i < 3; i++)
      if (fabs(vs[i]) > fabs(vs[imax]))
     imax = i;

   clear_vector(u, 3);

   u[imax] = 1.0;

   mu = (-DOT_VECTORS(vs, m) - vs4) / DOT_VECTORS(vs, u);

   for (i=0;i<3;i++)
      a0[i] = m[i] + mu * u[i];

   MAKE_3DVECTOR21(r1, a0, ar1);
   normalize_vector(ar1, ar1);
   MAKE_3DVECTOR21(r2, a0, ar2);
   normalize_vector(ar2, ar2);

   phi0 = acos(DOT_VECTORS(ar1, ar2));
   
   if (far_side_wrap)
      dphi = - (2 * M_PI - phi0) / (double)numSegs;
   else
      dphi = phi0 / (double)numSegs;

   cross_vectors(ar1, ar2, vsz);
   normalize_vector(vsz, vsz);
   cross_vectors(vsz, ar1, vsy);

   for (i = 0; i < 3; i++)
   {
      for (j = 0; j < 3; j++)
      {
     rphi[i][j] = 0.0;
     r0[i][j] = 0.0;
      }
   }

   for (i = 0; i < 3; i++)
   {
      r0[i][0] = ar1[i];
      r0[i][1] = vsy[i];
      r0[i][2] = vsz[i];
   }

   rphi[2][2] = 1;

   for (i = 0; i < numInteriorPts; i++)
   {
      phi = (i + 1) * dphi;
      rphi[0][0] = cos(phi);
      rphi[0][1] = -sin(phi);
      rphi[1][0] = sin(phi);
      rphi[1][1] = cos(phi);

      for (j = 0; j < 3; j++)
      {
         r[j] = 0.0;

         for (k = 0; k < 3; k++)
         {
            t[k] = 0.0;

            for (l = 0; l < 3; l++)
               t[k] += rphi[k][l] * ux[l];

            r[j] += r0[j][k] * t[k];
         }
      }

      for (j = 0; j < 3; j++)
      {
         f1[j] = r[j]/a[j];
         f2[j] = (a0[j] - m[j])/a[j];
      }

      aa = DOT_VECTORS(f1, f1);
      bb = 2.0 * (DOT_VECTORS(f1, f2));
      cc = DOT_VECTORS(f2, f2) - 1.0;
      mu3 = (-bb + sqrt(SQR(bb) - 4.0 * aa * cc)) / (2.0 * aa);

      for (j = 0; j < 3; j++)
         s[i][j] = a0[j] + mu3 * r[j];
   }

   *num_wrap_pts = numPts;
   
   if (*wrap_pts == NULL)
      *wrap_pts = (double*) simm_malloc((*num_wrap_pts) * 3 * sizeof(double));

   (*wrap_pts)[0] = r1[0];
   (*wrap_pts)[1] = r1[1];
   (*wrap_pts)[2] = r1[2];

   (*wrap_pts)[(numPts-1)*3] = r2[0];
   (*wrap_pts)[(numPts-1)*3+1] = r2[1];
   (*wrap_pts)[(numPts-1)*3+2] = r2[2];

   for (i = 0; i < numInteriorPts; i++)
   {
      index = (i+1) * 3;
      
      (*wrap_pts)[index]   = s[i][0];
      (*wrap_pts)[index+1] = s[i][1];
      (*wrap_pts)[index+2] = s[i][2];
   }

   *afst = 0.0;

   for (i = 0; i < numSegs; i++)
   {
      p[0] = (*wrap_pts)[i*3];
      p[1] = (*wrap_pts)[i*3+1];
      p[2] = (*wrap_pts)[i*3+2];

      q[0] = (*wrap_pts)[(i+1)*3];
      q[1] = (*wrap_pts)[(i+1)*3+1];
      q[2] = (*wrap_pts)[(i+1)*3+2];

      MAKE_3DVECTOR21(q, p, dv); 

      *afst += VECTOR_MAGNITUDE(dv);
   }
} /* dell */




/* -------------------------------------------------------------------------
   pt_to_ellipse - this routine is courtesy of Dave Eberly
      www.magic-software.com
   
   Input:   Ellipse (x/a)^2+(y/b)^2 = 1, point (u,v).
  
   Output:  Closest point (x,y) on ellipse to (u,v), function returns
            the distance sqrt((x-u)^2+(y-v)^2).
---------------------------------------------------------------------------- */
static double pt_to_ellipse2 (double a, double b, double u, double v, double* x, double* y)
{
    /* Graphics Gems IV algorithm for computing distance from point to
     * ellipse (x/a)^2 + (y/b)^2 = 1.  The algorithm as stated is not stable
     * for points near the coordinate axes.  The first part of this code
     * handles those points separately.
     */
    double a2 = a*a, b2 = b*b;
    double u2 = u*u, v2 = v*v;
    double a2u2 = a2*u2, b2v2 = b2*v2;
    double dx, dy, xda, ydb;
    int i, which;
    double t, P, Q, P2, Q2, f, fp;
    
    SBoolean nearXOrigin = (SBoolean) EQUAL_WITHIN_ERROR(0.0,u);
    SBoolean nearYOrigin = (SBoolean) EQUAL_WITHIN_ERROR(0.0,v);
    
    /* handle points near the coordinate axes */
    if (nearXOrigin && nearYOrigin)
    {
        if (a < b)
        {
            *x = (u < 0.0 ? -a : a);
            *y = v;
            return a;
        } else {
            *x = u;
            *y = (v < 0.0 ? -b : b);
            return b;
        }
    }

    if (nearXOrigin)  /* u == 0 */
    {
        if ( a >= b || fabs(v) >= b - a2/b )
        {
            *x = u;
            *y = ( v >= 0 ? b : -b );
            dy = *y - v;
            return fabs(dy);
        }
        else
        {
            *y = b2 * v / (b2-a2);
            dy = *y - v;
            ydb = *y / b;
            *x = a * sqrt(fabs(1 - ydb*ydb));
            return sqrt(*x * *x + dy*dy);
        }
    }

    if (nearYOrigin)  /* v == 0 */
    {
        if ( b >= a || fabs(u) >= a - b2/a )
        {
            *x = ( u >= 0 ? a : -a );
            dx = *x - u;
            *y = v;
            return fabs(*x - u);
        }
        else
        {
            *x = a2 * u / (a2-b2);
            dx = *x - u;
            xda = *x / a;
            *y = b * sqrt(fabs(1 - xda*xda));
            return sqrt(dx*dx + *y * *y);
        }
    }

    /* initial guess */
    if ( (u/a)*(u/a) + (v/b)*(v/b) < 1.0 )
    {
        which = 0;
        t = 0.0;
    }
    else
    {
        double max = a;
        
        which = 1;
        
        if ( b > max )
            max = b;

        t = max * sqrt(u*u + v*v);
    }

    for (i = 0; i < 64; i++)
    {
        P = t+a2;
        P2 = P*P;
        Q = t+b2;
        Q2 = Q*Q;
        f = P2*Q2 - a2u2*Q2 - b2v2*P2;
        
        //dkb - in original (above) was: if ( fabs(f) < 1e-09 )
         if (fabs(f) < 1e-20)
            break;

        fp = 2.0 * (P*Q*(P+Q) - a2u2*Q - b2v2*P);
        t -= f / fp;
    }

    *x = a2 * u / P;
    *y = b2 * v / Q;
    dx = *x - u;
    dy = *y - v;

    return sqrt(dx*dx + dy*dy);

} /* pt_to_ellipse */

/* -------------------------------------------------------------------------
   pt_to_ellipsoid - this routine is courtesy of Dave Eberly
      www.magic-software.com
   
   Input:   Ellipsoid (x/a)^2+(y/b)^2+(z/c)^2 = 1, point (u,v,w).
  
   Output:  Closest point (x,y,z) on ellipsoid to (u,v,w), function returns
            the distance sqrt((x-u)^2+(y-v)^2+(z-w)^2).
---------------------------------------------------------------------------- */
static double pt_to_ellipsoid2 (double a, double b, double c,
                                 double u, double v, double w,
                               double* x, double* y, double* z,
                               int specialCaseAxis)
{
    /* Graphics Gems IV algorithm for computing distance from point to
     * ellipsoid (x/a)^2 + (y/b)^2 +(z/c)^2 = 1.  The algorithm as stated
     * is not stable for points near the coordinate planes.  The first part
     * of this code handles those points separately.
     */
    int i,j;
    
    /* handle points near the coordinate planes by reducing the problem
     * to a 2-dimensional pt-to-ellipse.
     *
     * if uvw is close to more than one coordinate plane,  pick the 2d
     * elliptical cross-section with the narrowest radius.
     */
    if (specialCaseAxis < 0)
    {
       double abc[3], uvw[3], minEllipseRadiiSum = FLT_MAX;
       
       abc[0] = a; abc[1] = b; abc[2] = c;
       uvw[0] = u; uvw[1] = v; uvw[2] = w;

       for (i = 0; i < 3; i++)
       {
           if (EQUAL_WITHIN_ERROR(0.0,uvw[i]))
           {
               double ellipseRadiiSum = 0;

               for (j = 0; j < 3; j++)
                   if (j != i)
                       ellipseRadiiSum += abc[j];

               if (minEllipseRadiiSum > ellipseRadiiSum)
               {
                   specialCaseAxis = i;
                   minEllipseRadiiSum = ellipseRadiiSum;
               }
           }
       }
    }
    if (specialCaseAxis == 0) /* use elliptical cross-section in yz plane */
    {
       *x = u;
       return pt_to_ellipse2(b, c, v, w, y, z);
    }
    if (specialCaseAxis == 1) /* use elliptical cross-section in xz plane */
    {
       *y = v;
       return pt_to_ellipse2(c, a, w, u, z, x);
    }
    if (specialCaseAxis == 2) /* use elliptical cross-section in xy plane */
    {
       *z = w;
       return pt_to_ellipse2(a, b, u, v, x, y);
    }

    /* if we get to here, then the point is not close to any of the major planes,
     * so we can solve the general case.
     */
    {
        double a2 = a*a, b2 = b*b, c2 = c*c;
        double u2 = u*u, v2 = v*v, w2 = w*w;
        double a2u2 = a2*u2, b2v2 = b2*v2, c2w2 = c2*w2;
        double dx, dy, dz, t, f;

        /* initial guess */
        if ( (u/a)*(u/a) + (v/b)*(v/b) + (w/c)*(w/c) < 1.0 )
        {
            t = 0.0;
        }
        else
        {
            double max = a;
        
            if ( b > max )
                max = b;
            if ( c > max )
                max = c;

            t = max*sqrt(u*u+v*v+w*w);
        }

        for (i = 0; i < 64; i++)
        {
            double P = t+a2, P2 = P*P;
            double Q = t+b2, Q2 = Q*Q;
            double R = t+c2, _R2 = R*R;
            double PQ, PR, QR, PQR, fp;

            f = P2*Q2*_R2 - a2u2*Q2*_R2 - b2v2*P2*_R2 - c2w2*P2*Q2;
        
         // dkb  - in original (above) was:  if ( fabs(f) < 1e-09 )
            if ( fabs(f) < 1e-20)
            {
                *x = a2 * u / P;
                *y = b2 * v / Q;
                *z = c2 * w / R;
            
                dx = *x - u;
                dy = *y - v;
                dz = *z - w;
                return sqrt(dx*dx+dy*dy+dz*dz);
            }

            PQ = P*Q;
            PR = P*R;
            QR = Q*R;
            PQR = P*Q*R;
            fp = 2.0*(PQR*(QR+PR+PQ)-a2u2*QR*(Q+R)-b2v2*PR*(P+R)-c2w2*PQ*(P+Q));
        
            t -= f/fp;
        }
    }
    return -1.0;

} /* pt_to_ellipsoid2 */
