/*******************************************************************************

   CONSTRAINT.C

   Author: Krystyne Blaikie

   Date: 20-DEC-01

   Copyright (c) 2000 MusculoGraphics, Inc.
   All rights reserved.

   Description: 

   Routines:
   markAffectedGencoords:
   calculateConstraintResids:
   updateConstraintInfo:

*******************************************************************************/
#include "universal.h"
#include "functions.h"
#include "globals.h"
#include "cefunctions.h"

/*************** DEFINES (for this file only) *********************************/
#if EXPERIMENTAL
#define DEBUG_LEVEL 1
#endif
#if OPENSMAC
#undef ENGINE
#define ENGINE 1
#endif

/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/


/*************** GLOBAL VARIABLES (used in only a few files) ******************/
extern int JACOBIAN;

/*************** EXTERNED VARIABLES (declared in another file) ****************/
extern char badConstraintErrorMsg[];
extern char badGencoordErrorMsg2[];


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/


/************************* static function prototypes ***********************************/



/* mark and store all gencoords and joints that are found in loops
 * created by constraints */
void markAffectedGencoords(ModelStruct *ms)
{
   int i, j, k, parent, child, jnt;
   int *path, nq, nj, *jnts;
   GeneralizedCoord **qs, *gc;

   qs = (GeneralizedCoord**)simm_malloc(6 * ms->numjoints * sizeof(GeneralizedCoord*));
   jnts = (int *)simm_malloc(ms->num_constraint_objects * ms->numjoints * sizeof(int));
   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      /* Free the joints and qs arrays, if they exist. They will either
       * be left NULL, or re-malloced later.
       */
      FREE_IFNOTNULL(ms->constraintobj[i].joints);
      FREE_IFNOTNULL(ms->constraintobj[i].qs);

      if (ms->constraintobj[i].active == no)
      {
         ms->constraintobj[i].num_qs = 0;
         ms->constraintobj[i].num_jnts = 0;
         continue;
      }
      nq = 0;
      nj = 0;

      if (!ms->constraintobj[i].points)
      {
         continue;
      }
      parent = ms->constraintobj[i].segment;
      child = ms->constraintobj[i].points[0].segment;

      path = GET_PATH(ms->modelnum, parent, child);
      for (j = 0; path[j] != ms->numjoints + 1; j++)
      {
         jnt = ABS(path[j]) - 1;
         jnts[nj] = jnt;
         nj++;
         for (k = 0; k < 6; k++)
         {
            gc = ms->joint[jnt].dofs[k].gencoord;
            if (gc && gc->used_in_model == yes)
            {
               gc->used_in_constraint = yes;
               qs[nq] = gc;
               nq++;
            }
         }
      }

      ms->constraintobj[i].num_qs = nq;
      ms->constraintobj[i].num_jnts = nj;
      ms->constraintobj[i].joints = (int *)simm_malloc(nj * sizeof(int));
      ms->constraintobj[i].qs = (GeneralizedCoord**)simm_malloc(nq * sizeof(GeneralizedCoord*));
      for (j = 0; j < nj; j++)
         ms->constraintobj[i].joints[j] = jnts[j];
      for (j = 0; j < nq; j++)
         ms->constraintobj[i].qs[j] = qs[j];

#if EXPERIMENTAL
      printf("Constraint %d\n", i);
      printf("%s -> %s:\n", ms->segment[parent].name, ms->segment[child].name);
      printf("joints: ");
      for (j = 0; j < ms->constraintobj[i].num_jnts; j++)
         printf("%s ", ms->joint[ms->constraintobj[i].joints[j]].name);
      printf("\ngencoords: ");
      for (j = 0; j < ms->constraintobj[i].num_qs; j++)
         printf("%s ", ms->constraintobj[i].qs[j]->name);
      printf("\n");
#endif
   }

   free(qs);
   free(jnts);
}


/* CALCULATE CONSTRAINT RESIDUALS: 
 * assume gencoords have already been set to the values in the Q array.
 * Determine the residuals for each constraint point in each constraint
 * object that is being solved.  The residual for each point equals the 
 * shortest distance from the point to its constraint object.  If the
 * projected point is not within the active quadrant, residuals are increased.
 */
void calculateConstraintResids(void *data, int numQ, double q[], int numResid, double resid[], 
                               int startIndex, int endIndex,
                               int *iflag)
{
   int i, j, index, seg_index, constraintAxis, constraintSign;
   double pt[3], projpt[3], vec[3], distance;
   LCStruct *solveInfo = (LCStruct *)data;

//if (!JACOBIAN) printf("cons resid\n");
   index = startIndex;
   for (i = 0; i < solveInfo->model->num_constraint_objects; i++)
   {
      if (index > endIndex)
      {
//         printf("problem\n");
         break;
      }
      if (solveInfo->model->constraintobj[i].active == no)
         continue;
      if (solveInfo->constraintInfo->consUsed[i] == no)
         continue;
      if (solveInfo->model->constraintobj[i].constraintType == constraint_plane)
      {
         PlaneStruct plane;
         double planeLength, planeWidth;

         if (solveInfo->model->constraintobj[i].xforms_valid == no)
            recalc_constraint_xforms(&solveInfo->model->constraintobj[i]);
         
         plane.a = solveInfo->model->constraintobj[i].plane.a;
         plane.b = solveInfo->model->constraintobj[i].plane.b;
         plane.c = solveInfo->model->constraintobj[i].plane.c;
         plane.d = solveInfo->model->constraintobj[i].plane.d;

         for (j = 0; j < solveInfo->model->constraintobj[i].numPoints; j++)
         {
            COPY_1X3VECTOR(solveInfo->model->constraintobj[i].points[j].offset, pt);
            convert(solveInfo->model, pt, solveInfo->model->constraintobj[i].points[j].segment,
               solveInfo->model->constraintobj[i].segment);
            project_point_onto_plane(pt, &plane, projpt);
            SUB_VECTORS(projpt, pt, vec);
            distance = VECTOR_MAGNITUDE(vec);

            /* determine whether projected point is within plane limits */
            planeLength = solveInfo->model->constraintobj[i].radius.xyz[0];
            planeWidth = solveInfo->model->constraintobj[i].radius.xyz[1];
            if ((fabs(projpt[0]) > planeLength) || (fabs(projpt[2]) > planeWidth))
            {
               distance = 10.0;
            }

            resid[index++] = distance;
//if (!JACOBIAN) printf("resid[%d] (cons %d) = %f\n", index-1, i, resid[index-1]);
         }
      }
      else
      {
         constraintAxis = solveInfo->model->constraintobj[i].constraintAxis;
         constraintSign = solveInfo->model->constraintobj[i].constraintSign;
         for (j = 0; j < solveInfo->model->constraintobj[i].numPoints; j++)
         {
            seg_index = solveInfo->model->constraintobj[i].points[j].segment;
            COPY_1X3VECTOR(solveInfo->model->constraintobj[i].points[j].offset, pt);
            /* convert point from point_segment to constraint_segment frame */
            convert(solveInfo->model, pt, seg_index, solveInfo->model->constraintobj[i].segment);
            /* convert from constraint_segment frame to constraint object frame */
            convert_to_constraint_object_frame(&solveInfo->model->constraintobj[i], pt);
            /* calculate distance */
            if (solveInfo->model->constraintobj[i].constraintType == constraint_sphere)
            {
               double radius = solveInfo->model->constraintobj[i].radius.xyz[0];
               distance = VECTOR_MAGNITUDE(pt) - radius;
               /* check that point is in active quadrant */
               if (solveInfo->model->constraintobj[i].constraintSign != 0)
               {
                  double vec[3];
                  
                  /* find projected point */
                  vec[0] = pt[0];
                  vec[1] = pt[1];
                  vec[2] = pt[2];
                  normalize_vector(vec, vec);
                  projpt[0] = vec[0] * radius;
                  projpt[1] = vec[1] * radius;
                  projpt[2] = vec[2] * radius;
                  /* point on inactive quadrant if SIGN(pt[axis]) != constraint_sign */
                  if (DSIGN(projpt[constraintAxis]) != constraintSign)
                     distance = 100.0;
               }
            }
            else if (solveInfo->model->constraintobj[i].constraintType == constraint_ellipsoid)
            {
               distance = calc_distance_to_ellipsoid(pt, solveInfo->model->constraintobj[i].radius.xyz, projpt);
               /* check whether projpt is in the active quadrant */
               if (constraintSign != 0)
               {
                  /* point on inactive quadrant if SIGN(pt[axis]) != constraint_sign */
                  if (DSIGN(projpt[constraintAxis]) != constraintSign)
                     distance = 100.0;
               }
            }
            else if (solveInfo->model->constraintobj[i].constraintType == constraint_cylinder)
            {
               double radius = solveInfo->model->constraintobj[i].radius.xyz[0];
               double axis[] = {0.0, 0.0, 1.0}, origin[] = {0.0, 0.0, 0.0};
               distance = sqrt(get_distsqr_point_line(pt, origin, axis)) - radius;
               if (solveInfo->model->constraintobj[i].constraintSign != 0)
               {
                  /* find projected point on cylinder axis */
                  get_point_from_point_line(pt, origin, axis, projpt);
                  /* find projected point on cylinder */
                  vec[0] = pt[0] - projpt[0];
                  vec[1] = pt[1] - projpt[1];
                  vec[2] = pt[2] - projpt[2];
                  normalize_vector(vec, vec);
                  projpt[0] = vec[0] * radius;
                  projpt[1] = vec[1] * radius;
                  projpt[2] = vec[2] * radius;
                  /* point on inactive quadrant if SIGN(pt[axis]) != constraint_sign */
                  if (DSIGN(projpt[constraintAxis]) != constraintSign)
                     distance = 100.0;
               }
            }
            else
               distance = 0.0;
            resid[index] = distance * solveInfo->model->constraintobj[i].points[j].weight;
//if (!JACOBIAN) printf("resid[%d] (cons %d) = %f\n", index, i, resid[index]);
            index++;
         }
      }
   }
}




/* check if any of the qs in the constraints have changed */
void updateConstraintInfo(ModelStruct *ms)
{
   int i, j, k, nq, jnt;
   GeneralizedCoord* gc;
   ReturnCode rc;

   for (i = 0; i < ms->numgencoords; i++)
      ms->gencoord[i]->used_in_constraint = no;
   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      nq = 0;
      ms->constraintobj[i].qs = (GeneralizedCoord**)simm_malloc(6 * ms->constraintobj[i].num_jnts * sizeof(GeneralizedCoord*));
      for (j = 0; j < ms->constraintobj[i].num_jnts; j++)
      {
         jnt = ms->constraintobj[i].joints[j];
         for (k = 0; k < 6; k++)
         {
            gc = ms->joint[jnt].dofs[k].gencoord;
            if (gc && gc->used_in_model == yes)
            {
               gc->used_in_constraint = yes;
               ms->constraintobj[i].qs[nq] = gc;
               nq++;
            }
         }
      }
      ms->constraintobj[i].num_qs = nq;
      /* reallocate space for arrays given new sizes */
      ms->constraintobj[i].qs = (GeneralizedCoord**)simm_realloc(ms->constraintobj[i].qs, nq * sizeof(GeneralizedCoord*), &rc);
   }
}

static void transformConstraintPlane(ConstraintObject *co)
{
   double norm[3], dotProd;

   norm[0] = norm[1] = 0.0;
   norm[2] = 1.0;
   transform_vec(co->from_local_xform, norm);
   co->plane.a = norm[0];
   co->plane.b = norm[1];
   co->plane.c = norm[2];
   /* determine "d" value if translation changed */
   dotProd = DOT_VECTORS(norm, co->translation.xyz);
   co->plane.d = -1 * dotProd;
}

/* recalculate constraint transforms if the transforms were invalidated */
void recalc_constraint_xforms(ConstraintObject* co)
{

   if ( ! co->xforms_valid)
   {
      identity_matrix(co->from_local_xform);

      if (co->rotationAngle != 0.0)
         rotate_matrix_axis_angle(co->from_local_xform, co->rotationAxis.xyz, co->rotationAngle);
   
      translate_matrix(co->from_local_xform, co->translation.xyz);
   
      invert_4x4transform(co->from_local_xform, co->to_local_xform);

      /* if have a plane, transform the plane normal and its offset - translation from
       * the origin along the normal */
      //dkb check - if the normal was previously transformed should you
      // transform again? or should you transform default normal (001)?
      if (co->constraintType == constraint_plane)
      {
         transformConstraintPlane(co);
      }
   
      co->xforms_valid = yes;
   }
}

/* ==============================================================================
 * ==== CONSTRAINT OBJECT TRANSFORMATION SUPPORT
 */

#define CHECK_CO_XFORMS(_CO) { if ( ! (_CO)->xforms_valid) recalc_constraint_xforms(_CO); }


void convert_to_constraint_object_frame(ConstraintObject* co, double* pt)
{
   if (co && pt)
   {
      CHECK_CO_XFORMS(co);
   
      transform_pt(co->to_local_xform, pt);
   }
}

void convert_from_constraint_object_frame(ConstraintObject* co, double* pt)
{
   if (co && pt)
   {
      CHECK_CO_XFORMS(co);
   
      transform_pt(co->from_local_xform, pt);
   }
}

/* recalculate the constraint functions if constraint objects have changed.  If errorMsg is
 * set to yes, display any error messages if they occur. */
void ce_recalc_loops_and_constraints(ModelStruct* ms, int constraint_object, 
                                     SBoolean displayErrorMsg)
{
   int i, j;

   if (ms == NULL || constraint_object < 0)
      return;

   /* if there are constraints in the system -- if the co is used in a constraint
    * resolve the constraints.
    */
   if (ms->num_constraint_objects > 0)
   {
      ConstraintStatus constraintStatus;
      LoopStatus ls;

      solveAllLoopsAndConstraints(ms, &ls, &constraintStatus, yes);

      /* after joints are changed, the default values may no longer satisfy the constraints */
      ms->defaultConstraintsOK = no;

#if ! ENGINE
      if (displayErrorMsg == yes)
      {
         if (constraintStatus == constraintBroken)
         {
            glutMessageBox(GLUT_OK, 1, GLUT_ICON_INFORMATION,
               "Constraint Solver: Unable to Satisfy Constraints.", badConstraintErrorMsg);
         }
         else if (constraintStatus == gcOutOfRange)
         {
            glutMessageBox(GLUT_OK, 1, GLUT_ICON_INFORMATION,
               "Constraint Solver: Check Restraint Functions", badGencoordErrorMsg2);
         }
      }
#endif
   }

#if ! ENGINE
   queue_model_redraw(ms);
#endif
}
