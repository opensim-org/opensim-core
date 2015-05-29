/*******************************************************************************

   DP420.C

   Author: Peter Loan

   Date: 29-NOV-2009

   Copyright (c) 2009 MusculoGraphics, Inc.
   All rights reserved.

   Description: This file contains functions that convert a SIMM model into
    a Dynamics Pipeline model for use with dynamics DLLs created by SIMM 4.2 and 4.2.1.

   Routines:

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "sdfunctions.h"


/*************** DEFINES (for this file only) *********************************/


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/


/*************** GLOBAL VARIABLES (used in only a few files) ******************/


/*************** EXTERNED VARIABLES (declared in another .c file) *************/
extern SDSegment* SDseg;
extern int num_SD_segs;
extern int* joint_order;


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void freeDP420DefaultMuscle(dp420MuscleStruct* dm);
static void freeDP420Muscle(dp420MuscleStruct* ms, dp420MuscleStruct* dm);
static void freeDP420Function(dp420SplineFunction* sf);
static dp420WrapObject* getDP420WrapObject(dp420ModelStruct* dp, char name[]);
static void copyConstraintsToDP420Constraints(ModelStruct* ms, dp420ModelStruct* dp);
static void copyMuscPointToDP420MuscPoint(dpMusclePoint* from, dp420MusclePoint* to, ModelStruct* ms);
static void copyWrapObjectsToDP420(ModelStruct* ms, dp420ModelStruct* dp);
static void copyQsToDP420(ModelStruct* ms, dp420ModelStruct* dp);
static void copyFunctionToDP420Function(dpFunction* from, dp420SplineFunction** to);
static void copySegmentsToDP420(ModelStruct* ms, dp420ModelStruct* dp);
static void copyMusclesToDP420(ModelStruct* ms, dp420ModelStruct* dp, int muscleList[]);
static ReturnCode copy_nondefault_dyn_params420(dpMuscleStruct* from, dp420MuscleStruct* to,
                                                                dpMuscleStruct* deffrom, dp420MuscleStruct* defto);
static ReturnCode copy_nonnull_dyn_params420(dpMuscleStruct* from, dp420MuscleStruct* to);
static void copyDP420DefaultMuscle(dpMuscleStruct* from, dp420MuscleStruct* to, ModelStruct* ms, dp420ModelStruct* dp);
static void copyMuscleToDP420Muscle(dpMuscleStruct* from, dpMuscleStruct* defFrom,
                                                dp420MuscleStruct* to, dp420MuscleStruct* defTo,
                                                ModelStruct* ms, dp420ModelStruct* dp);
static int get_sd_floor_num420(dp420ModelStruct* dp, char* name);


static void freeDP420DefaultMuscle(dp420MuscleStruct* dm)
{
   if (dm)
   {
      int i;

      FREE_IFNOTNULL(dm->name);
      if (dm->num_orig_points && dm->mp_orig)
      {
         for (i = 0; i < *dm->num_orig_points; i++)
         {
            FREE_IFNOTNULL(dm->mp_orig[i].ranges);
            FREE_IFNOTNULL(dm->mp_orig[i].wrap_pts);
         }
         FREE_IFNOTNULL(dm->mp_orig);
         FREE_IFNOTNULL(dm->num_orig_points);
      }
      FREE_IFNOTNULL(dm->mp);
      FREE_IFNOTNULL(dm->max_isometric_force);
      FREE_IFNOTNULL(dm->pennation_angle);
      FREE_IFNOTNULL(dm->optimal_fiber_length);
      FREE_IFNOTNULL(dm->resting_tendon_length);
      FREE_IFNOTNULL(dm->max_contraction_vel);
      FREE_IFNOTNULL(dm->momentarms);
      if (dm->tendon_force_len_curve)
      {
         freeDP420Function(dm->tendon_force_len_curve);
         FREE_IFNOTNULL(dm->tendon_force_len_curve);
      }
      if (dm->active_force_len_curve)
      {
         freeDP420Function(dm->active_force_len_curve);
         FREE_IFNOTNULL(dm->active_force_len_curve);
      }
      if (dm->passive_force_len_curve)
      {
         freeDP420Function(dm->passive_force_len_curve);
         FREE_IFNOTNULL(dm->passive_force_len_curve);
      }
      if (dm->force_vel_curve)
      {
         freeDP420Function(dm->force_vel_curve);
         FREE_IFNOTNULL(dm->force_vel_curve);
      }

        if (dm->dynamic_params)
      {
         for (i = 0; i < dm->num_dynamic_params; i++)
            FREE_IFNOTNULL(dm->dynamic_params[i]);
         FREE_IFNOTNULL(dm->dynamic_params);
      }
      FREE_IFNOTNULL(dm->excitation_format);
      if (dm->excitation)
      {
         freeDP420Function(dm->excitation);
         FREE_IFNOTNULL(dm->excitation);
      }
      FREE_IFNOTNULL(dm->muscle_model_index);
      if (dm->wrapStruct)
      {
         for (i = 0; i < dm->numWrapStructs; i++)
         {
            FREE_IFNOTNULL(dm->wrapStruct[i]->wrap_object);
            FREE_IFNOTNULL(dm->wrapStruct[i]);
         }
         FREE_IFNOTNULL(dm->wrapStruct);
      }
   }
}


static void freeDP420Muscle(dp420MuscleStruct* ms, dp420MuscleStruct* dm)
{
   if (ms && dm)
   {
      int i;

      if (ms->name != dm->name)
         FREE_IFNOTNULL(ms->name);
      if (ms->mp_orig && ms->num_orig_points && ms->mp_orig != dm->mp_orig)
      {
         for (i = 0; i < *ms->num_orig_points; i++)
         {
            FREE_IFNOTNULL(ms->mp_orig[i].ranges);
            FREE_IFNOTNULL(ms->mp_orig[i].wrap_pts);
         }
         FREE_IFNOTNULL(ms->mp_orig);
      }
      if (ms->num_orig_points != dm->num_orig_points)
         FREE_IFNOTNULL(ms->num_orig_points);
      if (ms->mp != dm->mp)
         FREE_IFNOTNULL(ms->mp);
      if (ms->max_isometric_force != dm->max_isometric_force)
         FREE_IFNOTNULL(ms->max_isometric_force);
      if (ms->pennation_angle != dm->pennation_angle)
         FREE_IFNOTNULL(ms->pennation_angle);
      if (ms->optimal_fiber_length != dm->optimal_fiber_length)
         FREE_IFNOTNULL(ms->optimal_fiber_length);
      if (ms->resting_tendon_length != dm->resting_tendon_length)
         FREE_IFNOTNULL(ms->resting_tendon_length);
      if (ms->max_contraction_vel != dm->max_contraction_vel)
         FREE_IFNOTNULL(ms->max_contraction_vel);
      FREE_IFNOTNULL(ms->momentarms);
      if (ms->tendon_force_len_curve && ms->tendon_force_len_curve != dm->tendon_force_len_curve)
      {
         freeDP420Function(ms->tendon_force_len_curve);
         FREE_IFNOTNULL(ms->tendon_force_len_curve);
      }
      if (ms->active_force_len_curve && ms->active_force_len_curve != dm->active_force_len_curve)
      {
         freeDP420Function(ms->active_force_len_curve);
         FREE_IFNOTNULL(ms->active_force_len_curve);
      }
      if (ms->passive_force_len_curve && ms->passive_force_len_curve != dm->passive_force_len_curve)
      {
         freeDP420Function(ms->passive_force_len_curve);
         FREE_IFNOTNULL(ms->passive_force_len_curve);
      }
      if (ms->force_vel_curve && ms->force_vel_curve != dm->force_vel_curve)
      {
         freeDP420Function(ms->force_vel_curve);
         FREE_IFNOTNULL(ms->force_vel_curve);
      }

        /* The dynamic_params array (of pointers) is always unique to each muscle,
       * but the doubles that they point to could be in the default muscle.
       */
      if (ms->dynamic_params)
      {
         for (i = 0; i < ms->num_dynamic_params; i++)
         {
            if (ms->dynamic_params[i] != dm->dynamic_params[i])
               FREE_IFNOTNULL(ms->dynamic_params[i]);
         }
         FREE_IFNOTNULL(ms->dynamic_params);
      }
      if (ms->excitation_format != dm->excitation_format)
         FREE_IFNOTNULL(ms->excitation_format);
      if (ms->excitation && ms->excitation != dm->excitation)
      {
         freeDP420Function(ms->excitation);
         FREE_IFNOTNULL(ms->excitation);
      }
      if (ms->muscle_model_index != dm->muscle_model_index)
         FREE_IFNOTNULL(ms->muscle_model_index);
      if (ms->wrapStruct && ms->wrapStruct != dm->wrapStruct)
      {
         for (i = 0; i < ms->numWrapStructs; i++)
         {
            //FREE_IFNOTNULL(ms->wrapStruct[i]->wrap_object); the wrap object is freed later
            FREE_IFNOTNULL(ms->wrapStruct[i]);
         }
         FREE_IFNOTNULL(ms->wrapStruct);
      }
   }
}


static void freeDP420Function(dp420SplineFunction* sf)
{
   if (sf)
   {
      FREE_IFNOTNULL(sf->x);
      FREE_IFNOTNULL(sf->y);
      FREE_IFNOTNULL(sf->b);
      FREE_IFNOTNULL(sf->c);
      FREE_IFNOTNULL(sf->d);
   }
}

void freeDP420ModelStruct(dp420ModelStruct* dp)
{
   int i, j;

   FREE_IFNOTNULL(dp->name);

   if (dp->dynamic_param_names)
   {
      for (i = 0; i < dp->num_dynamic_params; i++)
         FREE_IFNOTNULL(dp->dynamic_param_names[i]);
      FREE_IFNOTNULL(dp->dynamic_param_names);
   }

   FREE_IFNOTNULL(dp->contacts);
   FREE_IFNOTNULL(dp->bilat_contacts);

   if (dp->spring)
   {
      for (i = 0; i < dp->num_springs; i++)
         FREE_IFNOTNULL(dp->spring[i].floor_name);
      FREE_IFNOTNULL(dp->spring);
   }

   if (dp->spring_floor)
   {
      for (i = 0; i < dp->num_spring_floors; i++)
      {
         FREE_IFNOTNULL(dp->spring_floor[i].name);
         if (dp->spring_floor[i].ph)
         {
            freeDPPolyhedron(dp->spring_floor[i].ph);
            FREE_IFNOTNULL(dp->spring_floor[i].ph);
         }
      }
      FREE_IFNOTNULL(dp->spring_floor);
   }

   if (dp->force_matte)
   {
      for (i = 0; i < dp->num_force_mattes; i++)
      {
         FREE_IFNOTNULL(dp->force_matte[i].name);
         if (dp->force_matte[i].ph)
         {
            freeDPPolyhedron(dp->force_matte[i].ph);
            FREE_IFNOTNULL(dp->force_matte[i].ph);
         }
      }
      FREE_IFNOTNULL(dp->force_matte);
   }

   if (dp->q)
   {
      for (i = 0; i < dp->nq; i++)
      {
         FREE_IFNOTNULL(dp->q[i].name);
         freeDP420Function(dp->q[i].restraint_func);
         freeDP420Function(dp->q[i].min_restraint_func);
         freeDP420Function(dp->q[i].max_restraint_func);
         freeDP420Function(dp->q[i].constraint_func);
      }
      FREE_IFNOTNULL(dp->q);
   }

   if (dp->body_segment)
   {
      for (i = 0; i < dp->num_body_segments; i++)
      {
         FREE_IFNOTNULL(dp->body_segment[i].name);
         for (j = 0; j < dp->body_segment[i].num_objects; j++)
         {
            freeDPPolyhedron(dp->body_segment[i].object[j]);
            FREE_IFNOTNULL(dp->body_segment[i].object[j]);
         }
         FREE_IFNOTNULL(dp->body_segment[i].object);
      }
      FREE_IFNOTNULL(dp->body_segment);
   }

   if (dp->muscles)
   {
      for (i = 0; i < dp->num_muscles; i++)
         freeDP420Muscle(&dp->muscles[i], &dp->default_muscle);
      FREE_IFNOTNULL(dp->muscles);
   }

   freeDP420DefaultMuscle(&dp->default_muscle);

   FREE_IFNOTNULL(dp->joint);

   if (dp->wrap_object)
   {
      for (i = 0; i < dp->num_wrap_objects; i++)
         FREE_IFNOTNULL(dp->wrap_object[i].name);
      FREE_IFNOTNULL(dp->wrap_object);
   }

   if (dp->constraint_object)
   {
      for (i = 0; i < dp->num_constraint_objects; i++)
      {
         FREE_IFNOTNULL(dp->constraint_object[i].name);
         for (j = 0; j < dp->constraint_object[i].numPoints; j++)
            FREE_IFNOTNULL(dp->constraint_object[i].points[j].name);
         FREE_IFNOTNULL(dp->constraint_object[i].points);
      }
      FREE_IFNOTNULL(dp->constraint_object);
   }

    FREE_IFNOTNULL(dp);
}

static dp420WrapObject* getDP420WrapObject(dp420ModelStruct* dp, char name[])
{
   
   int i;
   
   if (name == NULL)
      return NULL;
   
   for (i = 0; i < dp->num_wrap_objects; i++)
      if (STRINGS_ARE_EQUAL(name, dp->wrap_object[i].name))
         return &dp->wrap_object[i];
      
   return NULL; 
}

static void copyConstraintsToDP420Constraints(ModelStruct* ms, dp420ModelStruct* dp)
{
   int i, j, k, constraintNum;
   double tmp_mat[4][4], tmp_inv[4][4];

   /* Always turn on enforce_constraints for the dpModel (whether or not there
    * are any active constraint objects). This parameter is only turned off by
    * the simulation code if instructed to do so by a motion.
    */
   dp->enforce_constraints = 1;

   /* The SD/FAST constraint numbers for the constraint points start after
    * the last Q constraint, and are numbered consecutively.
    */
   constraintNum = dp->nq - (ms->numgencoords - ms->numunusedgencoords);

   dp->num_constraint_objects = ms->num_constraint_objects;
   if (dp->num_constraint_objects > 0)
   {
      dp->constraint_object = (dpConstraintObject*)simm_malloc(dp->num_constraint_objects * sizeof(dpConstraintObject));
      for (i = 0; i < dp->num_constraint_objects; i++)
      {
         ConstraintObject* from = &ms->constraintobj[i];
         dpConstraintObject* to = &dp->constraint_object[i];
         SegmentStruct* seg = &ms->segment[from->segment];
         mstrcpy(&to->name, from->name);
         to->constraint_type = from->constraintType;
         to->active = from->active;
         to->segment = get_sd_seg_num(seg->name);
         for (j = 0; j < 3; j++)
            to->co_radius[j] = from->radius.xyz[j];
         to->height = from->height;
         to->constraint_axis = from->constraintAxis;
         to->constraint_sign = from->constraintSign;
         to->numPoints = to->cp_array_size = from->numPoints;

         /* Form new transform matrices from the mass center of the segment to the
          * constraint object, rather than from the origin of the segment. SD/FAST expects
          * all segment-specific points (e.g., muscle points) to be specified w.r.t.
          * the segment's mass center.
          */
         copy_4x4matrix(from->from_local_xform, tmp_mat);
         for (j = 0; j < 3; j++)
            tmp_mat[3][j] -= seg->masscenter[j];
         invert_4x4transform(tmp_mat, tmp_inv);
         copy_4x4matrix(tmp_mat, to->from_local_xform);
         copy_4x4matrix(tmp_inv, to->to_local_xform);
         to->plane.a = from->plane.a;
         to->plane.b = from->plane.b;
         to->plane.c = from->plane.c;
         to->plane.d = from->plane.d;

         to->points = (dpConstraintPoint*)simm_malloc(to->numPoints * sizeof(dpConstraintPoint));
         for (j = 0; j < to->numPoints; j++)
         {
            seg = &ms->segment[from->points[j].segment];
            mstrcpy(&to->points[j].name, from->points[j].name);
            to->points[j].segment = get_sd_seg_num(seg->name);
            to->points[j].offset[XX] = from->points[j].offset[XX] - seg->masscenter[XX];
            to->points[j].offset[YY] = from->points[j].offset[YY] - seg->masscenter[YY];
            to->points[j].offset[ZZ] = from->points[j].offset[ZZ] - seg->masscenter[ZZ];
            to->points[j].weight = from->points[j].weight;
            to->points[j].constraint_num = constraintNum++;
         }
      }
   }
   else
   {
      dp->constraint_object = NULL;
   }
}

static void copyMuscPointToDP420MuscPoint(dpMusclePoint* from, dp420MusclePoint* to, ModelStruct* ms)
{
   int i;
   DofStruct* dof;

   to->segment = get_sd_seg_num(ms->segment[from->segment].name);
   to->refpt = -1;
   to->selected = from->selected;
   to->point[XX] = from->point[XX] - ms->segment[from->segment].masscenter[XX];
   to->point[YY] = from->point[YY] - ms->segment[from->segment].masscenter[YY];
   to->point[ZZ] = from->point[ZZ] - ms->segment[from->segment].masscenter[ZZ];
    if (from->isVia == dpYes)
    {
        to->numranges = 1;
        to->ranges = (dp420PointRange*)simm_malloc(sizeof(dp420PointRange));
        /* Find the SIMM DOF which uses the gencoord as an unconstrained Q. */
        dof = find_unconstrained_sd_dof(ms, (GeneralizedCoord*)from->viaRange.gencoord);
        if (dof)
            to->ranges[0].genc = dof->sd.state_number;
        else
            to->ranges[0].genc = 0; // TODO: serious error is index of gencoord in DP cannot be found!
        to->ranges[0].start = from->viaRange.start;
        to->ranges[0].end = from->viaRange.end;
    }
    else
    {
        to->numranges = 0;
        to->ranges = NULL;
    }

   to->is_auto_wrap_point = from->is_auto_wrap_point;
   to->wrap_distance = from->wrap_distance;
   to->num_wrap_pts = from->num_wrap_pts;
   if (to->num_wrap_pts > 0)
   {
      to->wrap_pts = (double*)simm_malloc(to->num_wrap_pts * 3 * sizeof(double));
      for (i = 0; i < to->num_wrap_pts * 3; i++)
         to->wrap_pts[i] = from->wrap_pts[i];
   }
   else
      to->wrap_pts = NULL;
}


static void copyWrapObjectsToDP420(ModelStruct* ms, dp420ModelStruct* dp)
{
   int i;

   dp->num_wrap_objects = ms->num_wrap_objects;

   if (dp->num_wrap_objects == 0)
   {
      dp->wrap_object = NULL;
      return;
   }

   dp->wrap_object = (dp420WrapObject*)simm_malloc(dp->num_wrap_objects * sizeof(dp420WrapObject));

   for (i = 0; i < dp->num_wrap_objects; i++)
   {
      dpWrapObject* from = ms->wrapobj[i];
      dp420WrapObject* to = &dp->wrap_object[i];

      mstrcpy(&to->name, from->name);
        to->wrap_type = from->wrap_type;
      to->active = from->active;
      to->wrap_algorithm = from->wrap_algorithm;
      to->segment = get_sd_seg_num(ms->segment[from->segment].name);
      to->radius[0] = from->radius[0];
      to->radius[1] = from->radius[1];
      to->radius[2] = from->radius[2];
      to->height = from->height;
      to->wrap_axis = from->wrap_axis;
      to->wrap_sign = from->wrap_sign;
      copy_4x4matrix(from->from_local_xform, to->from_local_xform);
      to->from_local_xform[3][XX] -= ms->segment[from->segment].masscenter[XX];
      to->from_local_xform[3][YY] -= ms->segment[from->segment].masscenter[YY];
      to->from_local_xform[3][ZZ] -= ms->segment[from->segment].masscenter[ZZ];
      invert_4x4transform(to->from_local_xform, to->to_local_xform);
   }
}


static void copyQsToDP420(ModelStruct* ms, dp420ModelStruct* dp)
{
   int i, j;
   DofStruct* dof;
   JointStruct* jnt;
   dp420QStruct* q;
   GeneralizedCoord* gc;

   dp->nu = dp->nq;
   dp->num_gencoords = 0;

   if (dp->nq == 0)
   {
      dp->q = NULL;
      return;
   }

   dp->q = (dp420QStruct*)simm_malloc(dp->nq * sizeof(dp420QStruct));

   for (i = 0; i < dp->nq; i++)
   {
      dof = find_nth_q_dof(ms, i);
      jnt = find_nth_q_joint(ms, i);
      if (dof->gencoord) //dkb
            gc = dof->gencoord;
      else
         gc = NULL;//dkb
      q = &dp->q[i];

      mstrcpy(&q->name, dof->sd.name);

      if (dof->sd.fixed == yes)
      {
         q->type = dpFixedQ;
      }
      else if (dof->sd.constrained == no)
      {
         /* Locked gencoords are modeled as fixed Qs (as of version 4.1.1). */
         if (dof->gencoord && gc->locked == yes)
                q->type = dpFixedQ;
         else
            q->type = dpUnconstrainedQ;
      }
      else
      {
         q->type = dpConstrainedQ;
      }
      q->joint = jnt->sd_num;
      q->axis = dof->sd.axis;
      q->conversion = dof->sd.conversion;
      q->initial_value = dof->sd.initial_value;
      q->initial_velocity = 0.0;

      if (dof->sd.constrained == no && dof->sd.fixed == no)
      {
         q->range_start = gc->range.start;
         q->range_end = gc->range.end;
            q->pd_stiffness = gc->pd_stiffness;
      }
      else
      {
         q->range_start = -99999.9;
         q->range_end = 99999.9;
            q->pd_stiffness = 0.0;
      }
      if (dof->sd.fixed == yes || dof->sd.constrained == yes)
      {
         q->restraint_func = NULL;
         q->min_restraint_func = NULL;
         q->max_restraint_func = NULL;
         q->function_active = dpNo;
      }
      else
      {
         if (gc->restraint_function)
         {
            /* The index of the function in the ModelStruct and the dpModelStruct
             * should be the same.
             */
                q->restraint_func = &dp->constraint_function[getFunctionIndex(ms, gc->restraint_function)];
            q->min_restraint_func = NULL;
            q->max_restraint_func = NULL;
            if (gc->restraintFuncActive == yes)
               q->function_active = dpYes;
            else
               q->function_active = dpNo;
         }
         else
         {
            q->restraint_func = NULL;
            q->function_active = dpNo;

            /* The index of the functions in the ModelStruct and the dpModelStruct
             * should be the same.
             */
            if (gc->min_restraint_function)
               q->min_restraint_func = &dp->constraint_function[getFunctionIndex(ms, gc->min_restraint_function)];
            else
               q->min_restraint_func = NULL;

            if (gc->max_restraint_function)
               q->max_restraint_func = &dp->constraint_function[getFunctionIndex(ms, gc->max_restraint_function)];
            else
               q->max_restraint_func = NULL;
         }
      }
      if (dof->sd.constrained == yes)
      {
         DofStruct* ind_dof = find_unconstrained_sd_dof(ms, dof->gencoord);
         /* The index of the functions in the ModelStruct and the dpModelStruct should be the same. */
         q->constraint_func = &dp->constraint_function[getFunctionIndex(ms, dof->function)];
         q->constraint_num = dof->sd.error_number;
         q->q_ind = ind_dof->sd.state_number;
      }
      else
      {
         q->constraint_func = NULL;
         q->constraint_num = -1;
         q->q_ind = -1;
      }

      if (dof->sd.fixed == yes || dof->sd.constrained == yes)
         q->output = dpNo;
      else
         q->output = dpYes;

      q->torque = 0.0;
      
      if (q->type == dpUnconstrainedQ)
         dp->num_gencoords++;
   }
}

static void copyFunctionToDP420Function(dpFunction* from, dp420SplineFunction** to)
{
   dp420SplineFunction* f;

   f = *to = (dp420SplineFunction*)simm_calloc(1, sizeof(dp420SplineFunction));

    f->type = from->type;

   f->cutoff_frequency = 0;
   f->usernum = 0;
   f->defined = dpYes;

   f->numpoints = f->coefficient_array_size = from->numpoints;
   f->x = (double*)simm_calloc(f->numpoints, sizeof(double));
   f->y = (double*)simm_calloc(f->numpoints, sizeof(double));
   f->b = (double*)simm_calloc(f->numpoints, sizeof(double));
   f->c = (double*)simm_calloc(f->numpoints, sizeof(double));
   f->d = (double*)simm_calloc(f->numpoints, sizeof(double));

   /* b, c, and d are not copied because they are recalculated
    * by the simulation.
    */
   memcpy(f->x, from->x, f->numpoints * sizeof(double));
   memcpy(f->y, from->y, f->numpoints * sizeof(double));
}


static void copySegmentsToDP420(ModelStruct* ms, dp420ModelStruct* dp)
{
   int i, j, k;

   dp->num_body_segments = num_SD_segs;

   dp->body_segment = (dpBodyStruct*)simm_malloc(dp->num_body_segments * sizeof(dpBodyStruct));

   for (i = 0; i < num_SD_segs; i++)
   {
      mstrcpy(&dp->body_segment[i].name, SDseg[i].name);
      dp->body_segment[i].output = (i > 0) ? dpYes : dpNo;
      dp->body_segment[i].mass = SDseg[i].mass;
      for (j = 0; j < 3; j++)
      {
         dp->body_segment[i].mass_center[j] = SDseg[i].mass_center[j];
         dp->body_segment[i].body_to_joint[j] = SDseg[i].body_to_joint[j];
         dp->body_segment[i].inboard_to_joint[j] = SDseg[i].inboard_to_joint[j];
      }
      for (j = 0; j < 3; j ++)
         for (k = 0; k < 3; k++)
            dp->body_segment[i].inertia[j][k] = SDseg[i].inertia[j][k];

      /* These fields are filled in later by the simulation code. */
      dp->body_segment[i].contactable = dpNo;
      dp->body_segment[i].contact_joints = NULL;
      for (j = 0; j < 3; j++)
      {
         dp->body_segment[i].contact_force[j] = 0.0;
         dp->body_segment[i].impact_force[j] = 0.0;
         dp->body_segment[i].impact_point[j] = 0.0;
      }

      /* If a SIMM segment with contactable objects gets split into more
       * than one SD segment, you want to copy the objects only to the
       * [one] SD segment with a valid simm_segment value.
       */
      if (SDseg[i].simm_segment >= 0)
      {
         SegmentStruct* seg = &ms->segment[SDseg[i].simm_segment];
         dp->body_segment[i].object = (dpPolyhedronStruct**)simm_malloc(seg->numContactObjects * sizeof(dpPolyhedronStruct*));
         dp->body_segment[i].num_objects = seg->numContactObjects;
         for (j = 0; j < seg->numContactObjects; j++)
            copyPolyhedronToDPPolyhedron(seg->contactObject[j].poly, &dp->body_segment[i].object[j],
                                         SDseg[i].simm_segment, i - 1, ms);
      }
      else
      {
         dp->body_segment[i].num_objects = 0;
         dp->body_segment[i].object = NULL;
      }
   }
}


static void copyMusclesToDP420(ModelStruct* ms, dp420ModelStruct* dp, int muscleList[])
{
   int i, index;

   dp->num_muscles = 0;
   dp->muscles = NULL;

   if (muscleList == NULL)
      return;

   for (i = 0; i < ms->nummuscles; i++)
      if (muscleList[i])
         dp->num_muscles++;

   /* If there are no muscles to copy, don't copy the default muscle; just return. */
   if (dp->num_muscles == 0)
      return;

   dp->muscles = (dp420MuscleStruct*)simm_malloc(dp->num_muscles * sizeof(dp420MuscleStruct));

    copyDP420DefaultMuscle(ms->default_muscle, &dp->default_muscle, ms, dp);

   for (i = 0, index = 0; i < ms->nummuscles; i++)
   {
      if (muscleList[i])
         copyMuscleToDP420Muscle(ms->muscle[i], ms->default_muscle, &dp->muscles[index++], &dp->default_muscle, ms, dp);
   }
}


static ReturnCode copy_nondefault_dyn_params420(dpMuscleStruct* from, dp420MuscleStruct* to,
                                                                dpMuscleStruct* deffrom, dp420MuscleStruct* defto)
{
   int i;

   to->num_dynamic_params = from->num_dynamic_params;

   if (to->num_dynamic_params > 0)
   {
      // A muscle's param name array is always the same as its default muscle's.
      to->dynamic_param_names = defto->dynamic_param_names;
      to->dynamic_params = (double**)simm_malloc(to->num_dynamic_params * sizeof(double*));

      if (to->dynamic_params == NULL)
         return code_bad;

      for (i = 0; i < to->num_dynamic_params; i++)
      {
         if (copy_nddouble(from->dynamic_params[i], &to->dynamic_params[i],
                           deffrom->dynamic_params[i], defto->dynamic_params[i]) == code_bad)
            return code_bad;
      }
   }
   else
   {
      to->dynamic_param_names = NULL;
      to->dynamic_params = NULL;
   }

   return code_fine;
}


/* This function copies the parameter names and all of the non-null values.
 */
static ReturnCode copy_nonnull_dyn_params420(dpMuscleStruct* from, dp420MuscleStruct* to)
{
   to->num_dynamic_params = from->num_dynamic_params;

   if (to->num_dynamic_params > 0)
   {
      int i;

      to->dynamic_params = (double**)simm_malloc(to->num_dynamic_params * sizeof(double*));
      if (to->dynamic_params == NULL)
         return code_bad;

      for (i = 0; i < to->num_dynamic_params; i++)
      {
         if (copy_nndouble(from->dynamic_params[i], &to->dynamic_params[i]) == code_bad)
            return code_bad;
      }
   }
   else
   {
      to->dynamic_params = NULL;
   }

   return code_fine;
}


static void copyDP420DefaultMuscle(dpMuscleStruct* from, dp420MuscleStruct* to, ModelStruct* ms, dp420ModelStruct* dp)
{
   int i;

   if (from->name == NULL)
      to->name = NULL;
   else
      mstrcpy(&to->name, from->name);

   to->has_wrapping_points = dpNo; // deprecated in SIMM 5.0 and not used by Pipeline 4.2
   to->has_force_points = dpNo;    // deprecated in SIMM 5.0 and not used by Pipeline 4.2

   if (from->path == NULL || from->path->num_orig_points == 0)
   {
      to->num_orig_points = NULL;
      to->mp_orig = NULL;
   }
   else
   {
      to->num_orig_points = (int*)simm_malloc(sizeof(int));
      *to->num_orig_points = from->path->num_orig_points;
      to->mp_orig = (dp420MusclePoint*)simm_malloc((*to->num_orig_points) * sizeof(dp420MusclePoint));
      for (i = 0; i < *to->num_orig_points; i++)
         copyMuscPointToDP420MuscPoint(&from->path->mp_orig[i], &to->mp_orig[i], ms);
   }

   to->nummomentarms = 0;
   to->momentarms = NULL;

   copy_nndouble(from->max_isometric_force, &to->max_isometric_force);
   copy_nndouble(from->pennation_angle, &to->pennation_angle);
   copy_nndouble(from->max_contraction_vel, &to->max_contraction_vel);
   copy_nndouble(from->optimal_fiber_length,&to->optimal_fiber_length);
   copy_nndouble(from->resting_tendon_length,&to->resting_tendon_length);

    // For the names of the dynamic parameters, use the array in the model struct.
    to->dynamic_param_names = dp->dynamic_param_names;

    copy_nonnull_dyn_params420(from, to);

   if (from->active_force_len_func == NULL)
      to->active_force_len_curve = NULL;
   else
      copyFunctionToDP420Function(*from->active_force_len_func, &to->active_force_len_curve);

   if (from->passive_force_len_func == NULL)
      to->passive_force_len_curve = NULL;
   else
      copyFunctionToDP420Function(*from->passive_force_len_func, &to->passive_force_len_curve);

   if (from->tendon_force_len_func == NULL)
      to->tendon_force_len_curve = NULL;
   else
      copyFunctionToDP420Function(*from->tendon_force_len_func, &to->tendon_force_len_curve);

   if (from->force_vel_func == NULL)
      to->force_vel_curve = NULL;
   else
      copyFunctionToDP420Function(*from->force_vel_func, &to->force_vel_curve);

    if (from->excitation_abscissa)
    {
        if (*from->excitation_abscissa == TIME)
            to->excitation_abscissa = -2;
        else
        {
            DofStruct* dof = find_unconstrained_sd_dof(ms, (GeneralizedCoord*)(*from->excitation_abscissa));
            if (dof)
                to->excitation_abscissa = dof->sd.state_number;
            else
                to->excitation_abscissa = 0;
        }
    }
    else
    {
        to->excitation_abscissa = -2;
    }

    if (from->excitation_func == NULL)
    {
      to->excitation = NULL;
        to->excitation_format = NULL;
    }
   else
    {
      copyFunctionToDP420Function(*from->excitation_func, &to->excitation);
      to->excitation_format = (dpFunctionType*)simm_malloc(sizeof(dpFunctionType));
        *to->excitation_format = (*from->excitation_func)->type;
    }

   copy_nnint(from->muscle_model_index, &to->muscle_model_index);
   /* In the Pipeline, muscle_model_index is zero-based. */
   if (to->muscle_model_index)
      (*to->muscle_model_index)--;

   /* These are set by when simulation when reading dllparams.txt, but
    * for the default muscle they are never used.
    */
   to->numStateParams = 0;
   to->stateParams = NULL;
}


static void copyMuscleToDP420Muscle(dpMuscleStruct* from, dpMuscleStruct* defFrom,
                                                dp420MuscleStruct* to, dp420MuscleStruct* defTo,
                                                ModelStruct* ms, dp420ModelStruct* dp)
{
   int i;

   /* Start by zero-ing out the entire muscle structure. */
   memset(to, 0, sizeof(dp420MuscleStruct));

   if (from->name == defFrom->name)
      to->name = defTo->name;
   else
      mstrcpy(&to->name, from->name);

   to->display = from->display;
   to->output = from->output;
   to->selected = from->selected;
   to->has_wrapping_points = dpNo; // deprecated in SIMM 5.0 and not used by Pipeline 4.2
   to->has_force_points = dpNo;    // deprecated in SIMM 5.0 and not used by Pipeline 4.2

   if (from->path == NULL || from->path->num_orig_points == 0)
   {
      to->num_orig_points = NULL;
      to->mp_orig = NULL;
   }
   else
   {
      to->num_orig_points = (int*)simm_malloc(sizeof(int));
      *to->num_orig_points = from->path->num_orig_points;
      to->mp_orig = (dp420MusclePoint*)simm_malloc((*to->num_orig_points) * sizeof(dp420MusclePoint));
      for (i = 0; i < *to->num_orig_points; i++)
         copyMuscPointToDP420MuscPoint(&from->path->mp_orig[i], &to->mp_orig[i], ms);
   }

   if (from->max_isometric_force == defFrom->max_isometric_force)
      to->max_isometric_force = defTo->max_isometric_force;
   else
   {
      to->max_isometric_force = (double*)simm_malloc(sizeof(double));
      *to->max_isometric_force = *from->max_isometric_force;
   }

   if (from->pennation_angle == defFrom->pennation_angle)
      to->pennation_angle = defTo->pennation_angle;
   else
   {
      to->pennation_angle = (double*)simm_malloc(sizeof(double));
      *to->pennation_angle = *from->pennation_angle;
   }

   if (from->optimal_fiber_length == defFrom->optimal_fiber_length)
      to->optimal_fiber_length = defTo->optimal_fiber_length;
   else
   {
      to->optimal_fiber_length = (double*)simm_malloc(sizeof(double));
      *to->optimal_fiber_length = *from->optimal_fiber_length;
   }

   if (from->resting_tendon_length == defFrom->resting_tendon_length)
      to->resting_tendon_length = defTo->resting_tendon_length;
   else
   {
      to->resting_tendon_length = (double*)simm_malloc(sizeof(double));
      *to->resting_tendon_length = *from->resting_tendon_length;
   }

   if (from->tendon_force_len_func == defFrom->tendon_force_len_func)
      to->tendon_force_len_curve = defTo->tendon_force_len_curve;
   else
      copyFunctionToDP420Function(*from->tendon_force_len_func, &to->tendon_force_len_curve);

   if (from->active_force_len_func == defFrom->active_force_len_func)
      to->active_force_len_curve = defTo->active_force_len_curve;
   else
      copyFunctionToDP420Function(*from->active_force_len_func, &to->active_force_len_curve);

   if (from->passive_force_len_func == defFrom->passive_force_len_func)
      to->passive_force_len_curve = defTo->passive_force_len_curve;
   else
      copyFunctionToDP420Function(*from->passive_force_len_func, &to->passive_force_len_curve);

   if (from->force_vel_func == defFrom->force_vel_func)
      to->force_vel_curve = defTo->force_vel_curve;
   else
      copyFunctionToDP420Function(*from->force_vel_func, &to->force_vel_curve);

   if (from->max_contraction_vel == defFrom->max_contraction_vel)
      to->max_contraction_vel = defTo->max_contraction_vel;
   else
   {
      to->max_contraction_vel = (double*)simm_malloc(sizeof(double));
      *to->max_contraction_vel = *from->max_contraction_vel;
   }

    copy_nondefault_dyn_params420(from, to, defFrom, defTo);

    if (from->excitation_abscissa == defFrom->excitation_abscissa)
    {
        to->excitation_abscissa = defTo->excitation_abscissa;
    }
    else if (from->excitation_abscissa)
    {
        DofStruct* dof = find_unconstrained_sd_dof(ms, (GeneralizedCoord*)(*from->excitation_abscissa));
        if (dof)
            to->excitation_abscissa = dof->sd.state_number;
        else
            to->excitation_abscissa = 0;
    }
    else
    {
        to->excitation_abscissa = -2;
    }

   if (from->excitation_func == defFrom->excitation_func)
    {
      to->excitation = defTo->excitation;
        to->excitation_format = defTo->excitation_format;
    }
   else
    {
      copyFunctionToDP420Function(*from->excitation_func, &to->excitation);
      to->excitation_format = (dpFunctionType*)simm_malloc(sizeof(dpFunctionType));
        *to->excitation_format = (*from->excitation_func)->type;
    }

   if (from->muscle_model_index == defFrom->muscle_model_index)
      to->muscle_model_index = defTo->muscle_model_index;
   else
   {
      to->muscle_model_index = (int*)simm_malloc(sizeof(int));
      *to->muscle_model_index = *from->muscle_model_index - 1;
   }

   to->numWrapStructs = from->numWrapStructs;
   if (from->wrapStruct == defFrom->wrapStruct)
      to->wrapStruct = defTo->wrapStruct;
   else
   {
      to->wrapStruct = (dp420MuscleWrapStruct**)simm_malloc(to->numWrapStructs * sizeof(dp420MuscleWrapStruct*));
      for (i = 0; i < to->numWrapStructs; i++)
      {
         to->wrapStruct[i] = (dp420MuscleWrapStruct*)simm_calloc(1, sizeof(dp420MuscleWrapStruct));
         to->wrapStruct[i]->wrap_algorithm = from->wrapStruct[i]->wrap_algorithm;
         to->wrapStruct[i]->startPoint = from->wrapStruct[i]->startPoint;
         to->wrapStruct[i]->endPoint = from->wrapStruct[i]->endPoint;
         to->wrapStruct[i]->wrap_object = getDP420WrapObject(dp, from->wrapStruct[i]->wrap_object->name);
      }
   }

   to->mp_array_size = *(to->num_orig_points) + (to->numWrapStructs * 2);
   to->mp = (dp420MusclePoint**)simm_malloc(sizeof(dp420MusclePoint*) * to->mp_array_size);

   /* When wrapping is calculated by the simulation, the mp[] array will be filled in
    * and num_points set appropriately.
    */
   to->num_points = 0;

   /* Copy the muscle's excitation format into the excitation spline structure,
    * whether or not it comes from the default muscle.
    */
   if (to->excitation && to->excitation_format)
      to->excitation->type = *to->excitation_format;

   /* These are set by when simulation when reading dllparams.txt. */
   to->numStateParams = 0;
   to->stateParams = NULL;
}


dp420ModelStruct* copyModelToDP420Model(ModelStruct* ms, int muscleList[])
{
   int i, j, k, segNum, count;
   ReturnCode rc;
    dp420QStruct* gencoord;
   GeneralizedCoord* temp_gc;
   dp420QStruct* temp_q;
   dp420ModelStruct* dp;

   dp = (dp420ModelStruct*)simm_calloc(1, sizeof(dp420ModelStruct));

   dp->simmModel = (dpSimmModelID)ms;

    /* This is only set to yes by set_up_kinetics_input() in the
     * simulation code.
     */
    dp->newInverseSimulation = dpNo;

   /* To fill in the dpModelStruct, you need to know how
    * the SIMM segments/joints are mapped to the SD/FAST segments/joints.
    * If there are loops in the model, this is not a one-to-one mapping.
    * So you need to call make_sdfast_model() to create the mapping.
    */

   check_dynamic_params(ms);
   if (ms->dynamics_ready == no)
   {
      sprintf(errorbuffer, "Unable to create dynamic simulation for %s", ms->name);
      error(none, errorbuffer);
      return NULL;
   }

   for (i = 0; i < ms->numjoints; i++)
      ms->joint[i].type = identify_joint_type(ms, i);

   if (valid_sdfast_model(ms) == no)
   {
      sprintf(buffer, "Unable to create dynamic simulation for %s", ms->name);
      error(none, buffer);
      error(none, errorbuffer); /* errorbuffer was filled in by valid_sdfast_model() */
      error(none, "Consult the SD/FAST manual or email technical support for more details.");
      return NULL;
   }

   if (make_sdfast_model(ms, NULL, no, 0) != code_fine)
   {
      sprintf(errorbuffer, "Unable to create dynamic simulation for %s", ms->name);
      error(none, errorbuffer);
      return NULL;
   }

   /* If you made it to here, then SDseg[] was filled in with the segment info,
    * and the Q info was stored in the model's joint array. So now you can copy
    * the info to the dpModelStruct.
    */

   mstrcpy(&dp->name, ms->name);

   /* Set the gravity vector appropriately. */
   switch (ms->gravity)
   {
      case smNoAlign:
         dp->gravity[0] = dp->gravity[1] = dp->gravity[2] = 0.0;
         break;
      case smX:
         dp->gravity[1] = dp->gravity[2] = 0.0;
         dp->gravity[0] = 9.80665;
         break;
      case smNegX:
         dp->gravity[1] = dp->gravity[2] = 0.0;
         dp->gravity[0] = -9.80665;
         break;
      case smY:
         dp->gravity[0] = dp->gravity[2] = 0.0;
         dp->gravity[1] = 9.80665;
         break;
      case smNegY:
         dp->gravity[0] = dp->gravity[2] = 0.0;
         dp->gravity[1] = -9.80665;
         break;
      case smZ:
         dp->gravity[0] = dp->gravity[1] = 0.0;
         dp->gravity[2] = 9.80665;
         break;
      case smNegZ:
      default:
         dp->gravity[0] = dp->gravity[1] = 0.0;
         dp->gravity[2] = -9.80665;
         break;
   }

   dp->num_closed_loops = ms->numclosedloops;

   /* Most of the Q struct has to be copied after the constraint functions, because it contains
    * pointers to functions. However, the number of Qs is needed for copying constraint
    * objects, so it is calculated here.
    */
   for (i = 0, dp->nq = 0; i < ms->numjoints; i++)
   {
      for (j = 0; j < 6; j++)
         if (ms->joint[i].dofs[j].type == function_dof || ms->joint[i].dofs[j].sd.fixed == yes)
            dp->nq++;
   }

   /* Copy the segment info. */
   copySegmentsToDP420(ms, dp);

   /* The entire array of joint structs is created and filled in by
    * init_joints() in the simulation code, using the information
    * returned by sdjnt(). However, some of the joint information is
    * filled in here so that the simulation can make sure that the
    * passed-in model matches the one built from model.sd.
    */
   dp->num_joints = 0;
   if (ms->numjoints > 0 && joint_order)
   {
      dp->joint = (dpJointStruct*)simm_calloc(ms->numjoints, sizeof(dpJointStruct));
      for (i = 0; i < ms->numjoints; i++)
      {
         if (joint_order[i] >= 0 && ms->joint[joint_order[i]].type != dpSkippable &&
             ms->joint[joint_order[i]].type != dpUnknownJoint)
            dp->joint[dp->num_joints++].jnt_type = ms->joint[joint_order[i]].type;
      }
      dp->joint = (dpJointStruct*)simm_realloc(dp->joint, dp->num_joints * sizeof(dpJointStruct), &rc);
   }
   else
   {
      dp->joint = NULL;
   }

   /* Copy the wrap objects. */
   copyWrapObjectsToDP420(ms, dp);

    // SIMM 5.0 models do not hold dynamic_param_names in the model struct,
    // and they are not needed in the 4.2 model struct (they are also stored
    // in the default muscle).
   dp->num_dynamic_params = 0;
    dp->dynamic_param_names = NULL;

   /* Copy the spring floors. */
   /* First count how many floors there are, since they are not stored in one array.
    * Each body segment has either 0 or 1 floors. Make sure the floor has a valid
    * polyhedron, or else skip over it.
    */
   for (i = 0, count = 0; i < ms->numsegments; i++)
      if (ms->segment[i].springFloor && ms->segment[i].springFloor->poly)
         count++;

   dp->num_spring_floors = dp->spring_floor_array_size = count;
   if (dp->num_spring_floors > 0)
   {
      dp->spring_floor = (dpSpringFloor*)simm_malloc(dp->num_spring_floors * sizeof(dpSpringFloor));
      for (i = 0, count = 0; i < ms->numsegments; i++)
      {
         SpringFloor* floor = ms->segment[i].springFloor;
         if (floor && floor->poly)
         {
            dp->spring_floor[count].segment = get_sd_seg_num(ms->segment[i].name);
            mstrcpy(&dp->spring_floor[count].name, floor->name);
            /* The plane information is taken from the polyhedron's first polygon. */
            copyPolyhedronToDPPolyhedron(floor->poly, &dp->spring_floor[count].ph, i, dp->spring_floor[count].segment, ms);
            dp->spring_floor[count].plane.a = dp->spring_floor[count].ph->polygon[0].normal[0];
            dp->spring_floor[count].plane.b = dp->spring_floor[count].ph->polygon[0].normal[1];
            dp->spring_floor[count].plane.c = dp->spring_floor[count].ph->polygon[0].normal[2];
            dp->spring_floor[count].plane.d = dp->spring_floor[count].ph->polygon[0].d;
            count++;
         }
      }
   }
   else
   {
      dp->spring_floor = NULL;
   }

   /* Copy the spring points.
    * First count how many points there are, since they are not stored in one array.
    * But only count the spring points that are linked to a spring floor that has
    * a valid polyhedron.
    */
   for (i = 0, count = 0; i < ms->numsegments; i++)
   {
      for (j = 0; j < ms->segment[i].numSpringPoints; j++)
      {
         segNum = ms->segment[i].springPoint[j].floorSegment;
         if (ms->segment[segNum].springFloor && ms->segment[segNum].springFloor->poly)
            count++;
      }
   }

   dp->num_springs = dp->spring_array_size = count;
   if (dp->num_springs > 0)
   {
      dp->spring = (dpSpringStruct*)simm_malloc(dp->num_springs * sizeof(dpSpringStruct));
      for (i = 0, count = 0; i < ms->numsegments; i++)
      {
         SegmentStruct* seg = &ms->segment[i];
         for (j = 0; j < seg->numSpringPoints; j++)
         {
            segNum = seg->springPoint[j].floorSegment;
            if (ms->segment[segNum].springFloor && ms->segment[segNum].springFloor->poly)
            {
               dp->spring[count].segment = get_sd_seg_num(seg->name);
               dp->spring[count].floor = get_sd_floor_num420(dp, ms->segment[segNum].springFloor->name);
               mstrcpy(&dp->spring[count].floor_name, ms->segment[segNum].springFloor->name);
               for (k = 0; k < 3; k++)
               {
                  dp->spring[count].point[k] = seg->springPoint[j].point[k] - seg->masscenter[k];
                  dp->spring[count].force[k] = 0.0;
               }
               dp->spring[count].friction = seg->springPoint[j].friction;
               dp->spring[count].param_a = seg->springPoint[j].param_a;
               dp->spring[count].param_b = seg->springPoint[j].param_b;
               dp->spring[count].param_c = seg->springPoint[j].param_c;
               dp->spring[count].param_d = seg->springPoint[j].param_d;
               dp->spring[count].param_e = seg->springPoint[j].param_e;
               dp->spring[count].param_f = seg->springPoint[j].param_f;
               count++;
            }
         }
      }
   }
   else
   {
      dp->spring = NULL;
   }

   /* Copy the force mattes. */
   /* First count how many mattes there are, since they are not stored in one array.
    * Each body segment has either 0 or 1 mattes. If the matte's polyhedron is missing
    * (e.g., because the bone file could not be found), skip the matte.
    */
   for (i = 0, count = 0; i < ms->numsegments; i++)
      if (ms->segment[i].forceMatte && ms->segment[i].forceMatte->poly)
         count++;

   dp->num_force_mattes = dp->force_matte_array_size = count;
   if (dp->num_force_mattes > 0)
   {
      dp->force_matte = (dpForceMatte*)simm_malloc(dp->num_force_mattes * sizeof(dpForceMatte));
      for (i = 0, count = 0; i < ms->numsegments; i++)
      {
         ContactObject* matte = ms->segment[i].forceMatte;
         if (matte && matte->poly)
         {
            dp->force_matte[count].segment = get_sd_seg_num(ms->segment[i].name);
            mstrcpy(&dp->force_matte[count].name, matte->name);
            copyPolyhedronToDPPolyhedron(matte->poly, &dp->force_matte[count].ph, i, dp->force_matte[count].segment, ms);
            count++;
         }
      }
   }
   else
   {
      dp->force_matte = NULL;
   }

   /* Copy the constraint objects. */
   copyConstraintsToDP420Constraints(ms, dp);

   /* Copy the functions, which are used for kinematic constraints and gencoord restraints. */
   dp->num_constraint_functions = countUsedFunctions(ms);
   if (dp->num_constraint_functions > 0)
   {
      int index = 0;
      dp->constraint_function = (dp420SplineFunction*)simm_calloc(dp->num_constraint_functions, sizeof(dp420SplineFunction));
      for (i = 0; i < ms->func_array_size; i++)
      {
         if (ms->function[i] && ms->function[i]->used == yes)
         {
            dpFunction* from = ms->function[i];
            dp420SplineFunction* to = &dp->constraint_function[index++];

                to->type = from->type;
            to->cutoff_frequency = 0.0;
                to->usernum = from->usernum;
            to->defined = dpYes;

            to->numpoints = to->coefficient_array_size = from->numpoints;
            to->x = (double*)simm_calloc(to->numpoints, sizeof(double));
            to->y = (double*)simm_calloc(to->numpoints, sizeof(double));
            to->b = (double*)simm_calloc(to->numpoints, sizeof(double));
            to->c = (double*)simm_calloc(to->numpoints, sizeof(double));
            to->d = (double*)simm_calloc(to->numpoints, sizeof(double));

            /* b, c, and d are not copied because they are recalculated
            * by the simulation.
            */
            memcpy(to->x, from->x, to->numpoints * sizeof(double));
            memcpy(to->y, from->y, to->numpoints * sizeof(double));
         }
      }
   }
   else
   {
      dp->constraint_function = NULL;
   }

   /* Copy the Q data. This has to be done after the constraint functions are copied
    * because the Qs contain pointers to the functions. However, the number of Qs
    * is calculated earlier, outside this function.
    */
   copyQsToDP420(ms, dp);

    // SIMM 5.0 models do not contain dynamic_param_names at the model level (only in the muscles),
    // but in SIMM 4.2 models they do. So copy them from the 5.0 default muscle to the 4.2 model.
    // The same array of param names will be stored in the 4.2 model, the default muscle, and
    // all the muscles.
    dp->num_dynamic_params = ms->default_muscle->num_dynamic_params;
    if (dp->num_dynamic_params > 0)
    {
        dp->dynamic_param_names = (char**)simm_malloc(dp->num_dynamic_params * sizeof(char*));
        for (i=0; i<dp->num_dynamic_params; i++)
            mstrcpy(&dp->dynamic_param_names[i], ms->default_muscle->dynamic_param_names[i]);
    }
    else
    {
        dp->dynamic_param_names = NULL;
    }

   /* Copy the muscles. */
   copyMusclesToDP420(ms, dp, muscleList);

   return dp;
}


static int get_sd_floor_num420(dp420ModelStruct* dp, char* name)
{
   int i;

   for (i = 0; i < dp->num_spring_floors; i++)
   {
      if (STRINGS_ARE_EQUAL(name, dp->spring_floor[i].name))
         return i;
   }

   return -1;
}
