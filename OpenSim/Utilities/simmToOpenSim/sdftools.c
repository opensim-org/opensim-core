/*******************************************************************************

   SDFTOOLS.C

   Author: Peter Loan

   Date: 18-FEB-04

   Copyright (c) 2004 MusculoGraphics, Inc.
   All rights reserved.

   Description: 

   Routines:

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "sdfunctions.h"


/*************** DEFINES (for this file only) *********************************/


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static char constrained_suffix[] = "_con";
char* ground_name[] = {"$ground", "ground"};


/*************** GLOBAL VARIABLES (used in only a few files) ******************/


/*************** EXTERNED VARIABLES (declared in another .c file) *************/
extern SDSegment* SDseg;
extern int num_SD_segs;
extern int* joint_order;


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static SBoolean is_xyz_joint(JointStruct* jnt);
static SBoolean axes_are_parallel(double axis1[], double axis2[], SBoolean oppositeDirAllowed);
static SBoolean segment_affects_dynamics(ModelStruct* ms, int segNum);
static void freeDPDefaultMuscle(dpMuscleStruct* dm);
static void freeDPMuscle(dpMuscleStruct* ms, dpMuscleStruct* dm);
static void freeDPMusclePaths(dpMusclePathStruct path[], int num);
static void freeDPFunction(dpFunction* sf);
static dpQStruct* get_q_for_gencoord(const char gencoord_name[], dpModelStruct* dp);
static int get_sd_floor_num(dpModelStruct* dp, char* name);
static void copyConstraintsToDPConstraints(ModelStruct* ms, dpModelStruct* dp);
static ReturnCode copyMusclesToDP(ModelStruct* ms, dpModelStruct* dp, int muscleList[]);
static dpWrapObject* getDPWrapObject(dpModelStruct* dp, char name[]);
static void copyWrapObjectsToDP(ModelStruct* ms, dpModelStruct* dp);
static void copyQsToDP(ModelStruct* ms, dpModelStruct* dp);
static void copySegmentsToDP(ModelStruct* ms, dpModelStruct* dp);



dpJointType identify_joint_type(ModelStruct* ms, int jointnum)
{
   int i, rot_order_index;
   int num_rot_constants = 0, num_rot_functions = 0;
   int num_trans_constants = 0, num_trans_functions = 0, num_real_rot_constants = 0;
   double axis1[3], axis2[3], axis3[3];
   JointStruct* jnt;

   jnt = &ms->joint[jointnum];

   /* First give the joint a one-token name. In most cases, this is just
    * the user-given name of the joint. However, if that name has special
    * characters in it (e.g. -), those characters must be removed. Also,
    * if the name starts with a number, then an underscore is prepended.
    */
   FREE_IFNOTNULL(jnt->sd_name);
   jnt->sd_name = (char*)simm_malloc(strlen(jnt->name) + 2);
   strcpy(jnt->sd_name, jnt->name);
   convert_string(jnt->sd_name, yes);

   /* Constant, non-zero rotations are not supported in SD/FAST. So to
    * implement them, you treat them as gencoords, and then prescribe
    * their motion to be constant (like weld joints).
    */
   for (i=0; i<3; i++)
   {
      if (jnt->dofs[i].type == constant_dof && EQUAL_WITHIN_ERROR(jnt->dofs[i].value,0.0))
         num_rot_constants++;
      else
         num_rot_functions++;
   }

   for (i=3; i<6; i++)
   {
      if (jnt->dofs[i].type == constant_dof)
         num_trans_constants++;
      else
         num_trans_functions++;
   }

   /* For the purposes of optimizing the pipeline code, you want to remove
    * from the model segments that:
    *   (1) are leaf nodes (are used in only one joint and are not ground),
    *   (2) have no gencoords in their joint,
    *   (3) have zero mass, and
    *   (4) have no muscle points, wrap objects, or constraint objects.
    * Such segments are often used for holding markers, and so are not needed
    * in the dynamic simulation.
    */
   for (i = 0; i < 3; i++)
   {
      if (jnt->dofs[i].type == constant_dof)
         num_real_rot_constants++;
   }

   /* If the joint has no degrees of freedom, check to see if one of
    * the segments is a leaf node.
    */
   if (num_real_rot_constants == 3 && num_trans_constants == 3)
   {
      int parent_count = 0, child_count = 0, leaf = -1;

      for (i = 0; i < ms->numjoints; i++)
      {
         if (jnt->from == ms->joint[i].from || jnt->from == ms->joint[i].to)
            parent_count++;
         if (jnt->to == ms->joint[i].from || jnt->to == ms->joint[i].to)
            child_count++;
      }

      /* If either of the segments in this joint is not ground and is
       * used in exactly one joint, then it is potentially skippable.
       */
      if (parent_count == 1 && jnt->from != ms->ground_segment)
         leaf = jnt->from;
      else if (child_count == 1 && jnt->to != ms->ground_segment)
         leaf = jnt->to;

      if (leaf >= 0 && segment_affects_dynamics(ms, leaf) == no)
         return dpSkippable;
   }

   if (num_rot_functions == 0 && num_trans_functions == 0)
      return dpWeld;

   if (num_rot_functions == 3 && num_trans_functions == 3)
   {
      /* In SD/FAST, bushing joints use the axes of rotation as
       * the axes of translation as well. Thus for a SIMM joint
       * to properly convert to an SD/FAST bushing, the rotation
       * axes must be X, Y, and Z, since the translations are
       * always w.r.t. these axes.
       */
      if (is_xyz_joint(jnt) == no)
         return dpUnknownJoint;

      if (jnt->order[TRANS] == 0)    /* translation is first */
         return dpBushing;
      else if (jnt->order[TRANS] == 3)    /* translation is last */
         return dpReverseBushing;
      else
         return dpUnknownJoint;
   }

   if (num_rot_functions == 1 && num_trans_functions == 0)
   {
      /* If the one rotation happens after the translations,
       * then this is a [normal] pin joint. If it happens
       * before, then this is a reverse pin joint, and the
       * translations have to be added to the other body segment.
       */
      for (i=0; i<3; i++)
      {
         if (jnt->dofs[i].type == function_dof ||
            NOT_EQUAL_WITHIN_ERROR(jnt->dofs[i].value,0.0))
            break;
      }
      if (i == 3)
         return dpUnknownJoint;
      rot_order_index = i + 1;
      if (jnt->order[TRANS] < jnt->order[rot_order_index])
         return dpPin;
      else
         return dpReversePin;
   }

   if (num_rot_functions == 0 && num_trans_functions == 1)
      return dpSlider;

   if (num_rot_functions == 3 && num_trans_functions == 0)
   {
      if (jnt->order[TRANS] == 0)
         return dpGimbal;
      else if (jnt->order[TRANS] == 3)
         return dpReverseGimbal;
      else
         return dpUnknownJoint;
   }

   if (num_rot_functions == 2 && num_trans_functions == 0)
   {
      if (jnt->order[TRANS] == 0)
         return dpUniversal;
      else if (jnt->order[TRANS] == 3)
         return dpReverseUniversal;
      else
         return dpUnknownJoint;
   }

   if (num_rot_functions == 1 && num_trans_functions == 1)
   {
      (void)find_rotation_axis(jnt, axis1);
      find_translation_axis(jnt, axis2, function_dof, 0);
      if (axes_are_parallel(axis1, axis2, no) == yes)
      {
          /* If the one rotation happens after the translations,
           * then this is a [normal] cylinder joint. If it happens
           * before, then this is a reverse cylinder joint, and the
           * translations have to be added to the other body segment.
           */
          for (i=0; i<3; i++)
         {
            if (jnt->dofs[i].type == function_dof || NOT_EQUAL_WITHIN_ERROR(jnt->dofs[i].value,0.0))
               break;
         }
         if (i == 3)
            return dpUnknownJoint;
         rot_order_index = i + 1;
         if (jnt->order[TRANS] < jnt->order[rot_order_index])
            return dpCylindrical;
         else
            return dpReverseCylindrical;
      }
      else
         return dpUnknownJoint;
   }

   if (num_rot_functions == 1 && num_trans_functions == 2)
   {
      (void)find_rotation_axis(jnt, axis1);
      (void)find_translation_axis(jnt, axis2, function_dof, 0);
      (void)find_translation_axis(jnt, axis3, function_dof, 1);
      /* As long as the rotation axis is not parallel to either of
       * the translation axes, and the translation is not in the
       * middle of the transformation order, this is a valid planar joint.
       */
      if (axes_are_parallel(axis1, axis2, yes) == yes ||
          axes_are_parallel(axis1, axis3, yes) == yes)
      {
         return dpUnknownJoint;
      }
      else
      {
         if (jnt->order[TRANS] == 0)    /* translation is first */
            return dpPlanar;
         else if (jnt->order[TRANS] == 3)    /* translation is last */
            return dpReversePlanar;
         else
            return dpUnknownJoint;
      }
   }

   return dpUnknownJoint;

}


void find_sdfast_joint_order(ModelStruct* ms, JointSDF jnts[],
                             SegmentSDF segs[], int joint_order[], int ground_name_index)
{
#define UNPROCESSED -2

   int i, joints_done = 0, num_skippable = 0, joints_used = 0;

   for (i=0; i<ms->numjoints; i++)
   {
      jnts[i].used = no;
      ms->joint[i].sd_num = UNPROCESSED;
   }

   for (i=0; i<ms->numsegments; i++)
   {
      segs[i].used = no;
      segs[i].times_split = 0;
      segs[i].mass_factor = 1.0;
   }

   segs[ms->ground_segment].used = yes;
   for (i=0; i<ms->numjoints; i++)
   {
      if (ms->joint[i].from == ms->ground_segment)
      {
         joint_order[joints_done] = i;
         ms->joint[i].sd_num = joints_done++;
         segs[ms->joint[i].to].used = yes;
         jnts[i].used = yes;
         jnts[i].dir = FORWARD;
         jnts[i].inbname = (char*)simm_malloc(strlen(ground_name[ground_name_index]) + 2);
         strcpy(jnts[i].inbname, ground_name[ground_name_index]);
         jnts[i].outbname = (char*)simm_malloc(strlen(ms->segment[ms->joint[i].to].name) + 2);
         strcpy(jnts[i].outbname, ms->segment[ms->joint[i].to].name);
         jnts[i].closes_loop = no;
      }
      else if (ms->joint[i].to == ms->ground_segment)
      {
         joint_order[joints_done] = i;
         ms->joint[i].sd_num = joints_done++;
         segs[ms->joint[i].from].used = yes;
         jnts[i].used = yes;
         jnts[i].dir = INVERSE;
         jnts[i].inbname = (char*)simm_malloc(strlen(ground_name[ground_name_index]) + 2);
         strcpy(jnts[i].inbname, ground_name[ground_name_index]);
         jnts[i].outbname = (char*)simm_malloc(strlen(ms->segment[ms->joint[i].from].name) + 2);
         strcpy(jnts[i].outbname, ms->segment[ms->joint[i].from].name);
         jnts[i].closes_loop = no;
      }
   }

   joints_used = joints_done;
   while (joints_done + num_skippable < ms->numjoints)
   {
      for (i=0; i<ms->numjoints; i++)
      {
         if (jnts[i].used == yes)
            continue;
         if (segs[ms->joint[i].from].used == no && segs[ms->joint[i].to].used == no)
            continue;
         else if (ms->joint[i].type == dpSkippable)
         {
            if (ms->joint[i].sd_num == UNPROCESSED)
            {
               ms->joint[i].sd_num = -1;
               joint_order[joints_done] = i;
               num_skippable++;
               jnts[i].used = no;
            }
            continue;
         }
         else if (segs[ms->joint[i].from].used == no)
         {
            joint_order[joints_done++] = i;
            ms->joint[i].sd_num = joints_used++;
            segs[ms->joint[i].from].used = yes;
            jnts[i].used = yes;
            jnts[i].dir = INVERSE;
            jnts[i].inbname = (char*)simm_malloc(strlen(ms->segment[ms->joint[i].to].name) + 2);
            strcpy(jnts[i].inbname, ms->segment[ms->joint[i].to].name);
            jnts[i].outbname = (char*)simm_malloc(strlen(ms->segment[ms->joint[i].from].name) + 2);
            strcpy(jnts[i].outbname, ms->segment[ms->joint[i].from].name);
            jnts[i].closes_loop = no;
         }
         else if (segs[ms->joint[i].to].used == no)
         {
            joint_order[joints_done++] = i;
            ms->joint[i].sd_num = joints_used++;
            segs[ms->joint[i].to].used = yes;
            jnts[i].used = yes;
            jnts[i].dir = FORWARD;
            jnts[i].inbname = (char*)simm_malloc(strlen(ms->segment[ms->joint[i].from].name) + 2);
            strcpy(jnts[i].inbname, ms->segment[ms->joint[i].from].name);
            jnts[i].outbname = (char*)simm_malloc(strlen(ms->segment[ms->joint[i].to].name) + 2);
            strcpy(jnts[i].outbname, ms->segment[ms->joint[i].to].name);
            jnts[i].closes_loop = no;
         }
         else
         {
            joint_order[joints_done++] = i;
            ms->joint[i].sd_num = joints_used++;
            jnts[i].used = yes;
            jnts[i].dir = FORWARD;
            segs[ms->joint[i].to].mass_factor += 1.0;
            segs[ms->joint[i].to].times_split++;
            jnts[i].inbname = (char*)simm_malloc(strlen(ms->segment[ms->joint[i].from].name) + 2);
            strcpy(jnts[i].inbname, ms->segment[ms->joint[i].from].name);
            jnts[i].outbname = make_sdfast_seg_name(ms->segment[ms->joint[i].to].name,
               segs[ms->joint[i].to].times_split);
            jnts[i].closes_loop = yes;
            /* ms->numclosedloops++; loops are now counted by markLoopJoints() JPL 3/5/01 */
         }
      }
   }

#if ! SIMMTOOPENSIM
   /* Remove all special characters in the body segment names, so that the strings
    * are valid one-token C strings. Do not touch the ground segment name ($ground)
    * because SD/FAST requires exactly that name.
    */
   for (i=0; i<ms->numjoints; i++)
   {
      if (ms->joint[i].type == dpSkippable)
         continue;
      if (STRINGS_ARE_NOT_EQUAL(jnts[i].inbname, ground_name[ground_name_index]))
         convert_string(jnts[i].inbname, yes);
      if (STRINGS_ARE_NOT_EQUAL(jnts[i].outbname, ground_name[ground_name_index]))
         convert_string(jnts[i].outbname, yes);
   }
#endif
}


static SBoolean is_xyz_joint(JointStruct* jnt)
{
   int i;
   double axis[4];
   SBoolean x_taken = no, y_taken = no, z_taken = no;

   for (i=0; i<3; i++)
   {
      COPY_1X4VECTOR(jnt->parentrotaxes[i], axis);
      normalize_vector(axis, axis);
      if (x_taken == no && axes_are_parallel(axis, x_axis, no) == yes)
      {
         x_taken = yes;
         break;
      }
      else if (y_taken == no && axes_are_parallel(axis, y_axis, no) == yes)
      {
         y_taken = yes;
         break;
      }
      else if (z_taken == no && axes_are_parallel(axis, z_axis, no) == yes)
      {
         z_taken = yes;
         break;
      }
      return no;
   }

   return yes;
}


static SBoolean axes_are_parallel(double axis1[], double axis2[], SBoolean oppositeDirAllowed)
{

   if (oppositeDirAllowed == yes)
   {
      if (EQUAL_WITHIN_ERROR(DABS(axis1[0]),DABS(axis2[0])) &&
          EQUAL_WITHIN_ERROR(DABS(axis1[1]),DABS(axis2[1])) &&
          EQUAL_WITHIN_ERROR(DABS(axis1[2]),DABS(axis2[2])))
         return yes;
      else
         return no;
   }
   else
   {
      if (EQUAL_WITHIN_ERROR(axis1[0],axis2[0]) &&
          EQUAL_WITHIN_ERROR(axis1[1],axis2[1]) &&
          EQUAL_WITHIN_ERROR(axis1[2],axis2[2]))
         return yes;
      else
         return no;
   }

}



/* Finds the first rotation-dof in a joint that is either a function, or
 * a non-zero constant, then returns the axis of that rotation.
 */
int find_rotation_axis(JointStruct* jnt, double axis[])
{
   int i;

   axis[0] = axis[1] = axis[2] = 0.0;

   for (i=0; i<3; i++)
   {
      if (jnt->dofs[i].type == function_dof)
     break;
      else if (NOT_EQUAL_WITHIN_ERROR(jnt->dofs[i].value,0.0))
     break;
   }

   if (i == 3)
      return 0; /* TODO: should return -1, but not ready to handle the error */

   axis[0] = jnt->parentrotaxes[i][0];
   axis[1] = jnt->parentrotaxes[i][1];
   axis[2] = jnt->parentrotaxes[i][2];

   normalize_vector(axis, axis);

   return i;
}



/* Finds the Nth translation-dof in a joint that matches the DofType passed in,
 * then returns the axis of that translation.
 */
int find_translation_axis(JointStruct* jnt, double axis[], DofType type, int num)
{
   int i, count = 0;

   axis[0] = axis[1] = axis[2] = 0.0;

   for (i = 3; i < 6; i++)
   {
      if (jnt->dofs[i].type == type)
      {
         if (count++ == num)
            break;
      }
   }

   if (i == 6)
      return 0; /* TODO: should return -1, but not ready to handle the error */

   axis[i - 3] = 1.0;

   return i;
}


char* get_joint_type_name(dpJointType type, Direction dir)
{

   if (type == dpPin || type == dpReversePin)
      return ("pin");
   else if (type == dpCylindrical || type == dpReverseCylindrical)
      return ("cylinder");
   else if (type == dpPlanar)
   {
      if (dir == FORWARD)
         return ("planar");
      else
         return ("rplanar");
   }
   else if (type == dpReversePlanar)
   {
      if (dir == FORWARD)
         return ("rplanar");
      else
         return ("planar");
   }
   else if (type == dpSlider)
      return ("slider");
   else if (type == dpUniversal || type == dpReverseUniversal)
      return ("ujoint");
   else if (type == dpGimbal || type == dpReverseGimbal)
      return ("gimbal");
   else if (type == dpWeld)
      return ("pin");                  /* welds are prescribed pins */
   else if (type == dpBushing)
   {
      if (dir == FORWARD)
         return ("bushing");
      else
         return ("rbushing");
   }
   else if (type == dpReverseBushing)
   {
      if (dir == FORWARD)
         return ("rbushing");
      else
         return ("bushing");
   }

   return ("unknown");

}

static SBoolean segment_affects_dynamics(ModelStruct* ms, int segNum)
{
   int i, j;
   SegmentStruct* seg;

   if (segNum < 0 || segNum >= ms->numsegments)
      return yes;

   seg = &ms->segment[segNum];

   if (NOT_EQUAL_WITHIN_ERROR(seg->mass, 0.0))
      return yes;

   for (i = 0; i < ms->num_wrap_objects; i++)
   {
      if (ms->wrapobj[i]->segment == segNum)
         return yes;
   }

   for (i = 0; i < ms->nummuscles; i++)
   {
      for (j = 0; j < ms->muscle[i]->path->num_points; j++)
         if (ms->muscle[i]->path->mp[j]->segment == segNum)
            return yes;
   }

   return no;
}


void name_dofs(ModelStruct* ms)
{
   int i, j;

   for (i=0; i<ms->numjoints; i++)
   {
      for (j=0; j<6; j++)
      {
         if (ms->joint[i].dofs[j].sd.constrained == no)
         {
            GeneralizedCoord* gencoord = ms->joint[i].dofs[j].gencoord;
            ms->joint[i].dofs[j].sd.name = (char*)simm_malloc(strlen(gencoord->name) + 2);
            strcpy(ms->joint[i].dofs[j].sd.name, gencoord->name);
            convert_string(ms->joint[i].dofs[j].sd.name, yes);
            ms->joint[i].dofs[j].sd.con_name = NULL;
         }
         else
         {
            char namebuffer[CHARBUFFER];
            strcat3(namebuffer, ms->joint[i].name, "_", getjointvarname(j), CHARBUFFER);
            convert_string(namebuffer, yes);
            mstrcpy(&ms->joint[i].dofs[j].sd.name, namebuffer);
            strcat(namebuffer, constrained_suffix);
            mstrcpy(&ms->joint[i].dofs[j].sd.con_name, namebuffer);
         }
      }
   }
}


void check_dynamic_params(ModelStruct* ms)
{
   int i, j;

   ms->dynamics_ready = yes;

   for (i=0; i<ms->numsegments; i++)
   {
      if (STRINGS_ARE_NOT_EQUAL(ms->segment[i].name,"ground"))
      {
         if (ms->segment[i].mass_specified == no &&
             ms->segment[i].masscenter_specified == no &&
             ms->segment[i].inertia_specified == no)
         {
            sprintf(errorbuffer, "No mass properties specified for segment %s.", ms->segment[i].name);
            error(none, errorbuffer);
            ms->dynamics_ready = no;
         }
         else
         {
            if (ms->segment[i].mass_specified == no)
            {
               sprintf(errorbuffer, "Mass not specified for segment %s.", ms->segment[i].name);
               error(none, errorbuffer);
               ms->dynamics_ready = no;
            }
            if (ms->segment[i].masscenter_specified == no)
            {
               sprintf(errorbuffer, "Masscenter not specified for segment %s.", ms->segment[i].name);
               error(none, errorbuffer);
               ms->dynamics_ready = no;
            }
            if (ms->segment[i].inertia_specified == no)
            {
               sprintf(errorbuffer, "Inertia matrix not specified for segment %s.", ms->segment[i].name);
               error(none, errorbuffer);
               ms->dynamics_ready = no;
            }
         }
      }
      for (j = 0; j < ms->segment[i].numContactObjects; j++)
      {
         if (ms->segment[i].contactObject[j].poly == NULL)
         {
            sprintf(errorbuffer, "The bone file for contact object %s was not found.", ms->segment[i].contactObject[j].name);
            error(none, errorbuffer);
            ms->dynamics_ready = no;
         }
      }
   }
}


void freeDPModelStruct(dpModelStruct* dp)
{
   int i, j;

   FREE_IFNOTNULL(dp->name);

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
         freeDPFunction(dp->q[i].restraint_function);
         freeDPFunction(dp->q[i].min_restraint_function);
         freeDPFunction(dp->q[i].max_restraint_function);
         freeDPFunction(dp->q[i].constraint_function);
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
         freeDPMuscle(&dp->muscles[i], &dp->default_muscle);
      FREE_IFNOTNULL(dp->muscles);
   }

   freeDPDefaultMuscle(&dp->default_muscle);

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

static void freeDPDefaultMuscle(dpMuscleStruct* dm)
{
   if (dm)
   {
      int i;

      FREE_IFNOTNULL(dm->name);
      FREE_IFNOTNULL(dm->max_isometric_force);
      FREE_IFNOTNULL(dm->pennation_angle);
      FREE_IFNOTNULL(dm->optimal_fiber_length);
      FREE_IFNOTNULL(dm->resting_tendon_length);
      FREE_IFNOTNULL(dm->max_contraction_vel);
      FREE_IFNOTNULL(dm->momentarms);
      FREE_IFNOTNULL(dm->tendon_force_len_func);
      FREE_IFNOTNULL(dm->active_force_len_func);
      FREE_IFNOTNULL(dm->passive_force_len_func);
      FREE_IFNOTNULL(dm->force_vel_func);
      FREE_IFNOTNULL(dm->excitation_func);

      if (dm->dynamic_param_names)
      {
         for (i = 0; i < dm->num_dynamic_params; i++)
            FREE_IFNOTNULL(dm->dynamic_param_names[i]);
         FREE_IFNOTNULL(dm->dynamic_param_names);
      }

      if (dm->dynamic_params)
      {
         for (i = 0; i < dm->num_dynamic_params; i++)
            FREE_IFNOTNULL(dm->dynamic_params[i]);
         FREE_IFNOTNULL(dm->dynamic_params);
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

/* does not free the path (path) */
static void freeDPMuscle(dpMuscleStruct* ms, dpMuscleStruct* dm)
{
   if (ms && dm)
   {
      int i;

      if (ms->name != dm->name)
         FREE_IFNOTNULL(ms->name);
      if (ms->path)
      {
         for (i = 0; i < ms->path->num_orig_points; i++)
            FREE_IFNOTNULL(ms->path->mp_orig[i].wrap_pts);
         FREE_IFNOTNULL(ms->path->mp_orig);
         FREE_IFNOTNULL(ms->path->mp);
      }
      FREE_IFNOTNULL(ms->path);
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
      if (ms->tendon_force_len_func != dm->tendon_force_len_func)
         FREE_IFNOTNULL(ms->tendon_force_len_func);
      if (ms->active_force_len_func != dm->active_force_len_func)
         FREE_IFNOTNULL(ms->active_force_len_func);
      if (ms->passive_force_len_func != dm->passive_force_len_func)
         FREE_IFNOTNULL(ms->passive_force_len_func);
      if (ms->force_vel_func != dm->force_vel_func)
         FREE_IFNOTNULL(ms->force_vel_func);
      if (ms->excitation_func != dm->excitation_func)
         FREE_IFNOTNULL(ms->excitation_func);

      /* The dynamic_param_names should always be equal to the default muscle's,
       * which should always be equal to the model's array of names.
       */
      if (ms->dynamic_param_names && ms->dynamic_param_names != dm->dynamic_param_names)
      {
         for (i = 0; i < ms->num_dynamic_params; i++)
            FREE_IFNOTNULL(ms->dynamic_param_names[i]);
         FREE_IFNOTNULL(ms->dynamic_param_names);
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

void freeDPPolyhedron(dpPolyhedronStruct* ph)
{
   if (ph)
   {
      int i;

      FREE_IFNOTNULL(ph->name);
      for (i = 0; i < ph->num_vertices; i++)
         FREE_IFNOTNULL(ph->vertex[i].polygons);
      for (i = 0; i < ph->num_polygons; i++)
         FREE_IFNOTNULL(ph->polygon[i].vertex_index);
      FREE_IFNOTNULL(ph->vertex);
      FREE_IFNOTNULL(ph->polygon);
   }
}


static void freeDPFunction(dpFunction* sf)
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

/* copy the SIMM model into the dp model structure */

dpModelStruct* copyModelToDPModel(ModelStruct* ms, int muscleList[])
{
   int i, j, k, segNum, count;
   ReturnCode rc;
    dpQStruct* gencoord;
   GeneralizedCoord *temp_gc;
   dpQStruct* temp_q;
   dpModelStruct* dp;

   dp = (dpModelStruct*)simm_calloc(1, sizeof(dpModelStruct));

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
      freeDPModelStruct(dp);
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
      freeDPModelStruct(dp);
      return NULL;
   }

   if (make_sdfast_model(ms, NULL, no, 0) != code_fine)
   {
      sprintf(errorbuffer, "Unable to create dynamic simulation for %s", ms->name);
      error(none, errorbuffer);
      freeDPModelStruct(dp);
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
   copySegmentsToDP(ms, dp);

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
   copyWrapObjectsToDP(ms, dp);

   // DKB Nov. 2009 - moved copy muscles after copy Qs since muscles may have references to Qs

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
               dp->spring[count].floor = get_sd_floor_num(dp, ms->segment[segNum].springFloor->name);
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
   copyConstraintsToDPConstraints(ms, dp);

   /* Copy the functions, which are used for kinematic constraints, gencoord restraints
    * and moving muscle point definitions. */
   dp->num_functions = countUsedFunctions(ms);
   if (dp->num_functions > 0)
   {
      int index = 0;
      dp->function = (dpFunction**)simm_calloc(dp->num_functions, sizeof(dpFunction*));
      for (i = 0; i < ms->func_array_size; i++)
      {
         if (ms->function[i] && ms->function[i]->used == dpYes)
         {
            dpFunction* from = ms->function[i];
            dpFunction *to = NULL;
            dp->function[index] = (dpFunction*)simm_calloc(1, sizeof(dpFunction));
            to = dp->function[index++];
            // DKB Sept 17, 2009 added malloc because copy expects coeff_array_size to be set and memory to be allocated
            malloc_function(to, from->coefficient_array_size);  
            copy_function(from, to);
         }
      }
   }
   else
   {
      dp->function = NULL;
   }

   /* Copy the Q data. This has to be done after the constraint functions are copied
    * because the Qs contain pointers to the functions. However, the number of Qs
    * is calculated earlier, outside this function.
    */
   copyQsToDP(ms, dp);

   /* Copy the muscles. */   
   if (copyMusclesToDP(ms, dp, muscleList) == code_bad)  //moved after the copy Qs because muscles may refer to Qs
   {
      sprintf(errorbuffer, "Unable to create dynamic simulation for %s - error in muscles", ms->name);
      error(none, errorbuffer);
      freeDPModelStruct(dp);
      return NULL;
   }

   for (i = 0; i < dp->num_muscles; i++)
   {
      for (j = 0; j < dp->muscles[i].path->num_orig_points; j++)
      {
         if (dp->muscles[i].path->mp_orig[j].isVia)
         {
            temp_q = (dpQStruct *)dp->muscles[i].path->mp_orig[j].viaRange.gencoord;
            printf("muscle %s has via point for %s\n", dp->muscles[i].name, temp_q->name);
         }
      }
      for (j = 0; j < dp->muscles[i].path->num_orig_points; j++)
      {
         if (dp->muscles[i].path->mp_orig[j].isMovingPoint)
         {
            if (dp->muscles[i].path->mp_orig[j].gencoord[0])
            {
               temp_q = (dpQStruct *)dp->muscles[i].path->mp_orig[j].gencoord[0];
               printf("muscle %s has mmpX point for %s\n", dp->muscles[i].name, temp_q->name);
            }
            if (dp->muscles[i].path->mp_orig[j].gencoord[1])
            {
               temp_q = (dpQStruct *)dp->muscles[i].path->mp_orig[j].gencoord[1];
               printf("muscle %s has mmpY point for %s\n", dp->muscles[i].name, temp_q->name);
            }
            if (dp->muscles[i].path->mp_orig[j].gencoord[2])
            {
               temp_q = (dpQStruct *)dp->muscles[i].path->mp_orig[j].gencoord[2];
               printf("muscle %s has mmpZ point for %s\n", dp->muscles[i].name, temp_q->name);
            }
         }
      }
   }

   return dp;
}

static dpQStruct* get_q_for_gencoord(const char gencoord_name[], dpModelStruct* dp)
{
   int i;

   for (i = 0; i < dp->nq; i++)
   {
      if (STRINGS_ARE_EQUAL(gencoord_name, dp->q[i].name))
         return &dp->q[i];
   }

   return NULL;
}


static int get_sd_floor_num(dpModelStruct* dp, char* name)
{
   int i;

   for (i = 0; i < dp->num_spring_floors; i++)
   {
      if (STRINGS_ARE_EQUAL(name, dp->spring_floor[i].name))
         return i;
   }

   return -1;
}


static void copyConstraintsToDPConstraints(ModelStruct* ms, dpModelStruct* dp)
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

/* copy muscles from the SIMM model structure to the DP structure
 * when copied from the ModelStruct, all via point gencoords, moving muscle point gencoords,
 * excitation abscissas , are refering to GENCOORDS (GeneralizedCoordStruct) they need
 * to be converted to Qs (dpQStruct)
 * Assuming the Qs have been copied to the dp struct, convert
 * and ?? wrpa object gencoords*/
static ReturnCode copyMusclesToDP(ModelStruct* ms, dpModelStruct* dp, int muscleList[])
{
   int i, j, index;

   dp->num_muscles = 0;
   dp->muscles = NULL;

   if (muscleList == NULL)
      return code_fine;

   for (i = 0; i < ms->nummuscles; i++)
      if (muscleList[i])
         dp->num_muscles++;

   /* If there are no muscles to copy, don't copy the default muscle; just return. */
   if (dp->num_muscles == 0)
      return code_fine;

   if (copy_default_muscle_dp(ms->default_muscle, &dp->default_muscle, dp) == code_bad)
      return code_bad;

   dp->muscles = (dpMuscleStruct*)simm_malloc(dp->num_muscles * sizeof(dpMuscleStruct));

   for (i = 0, index = 0; i < ms->nummuscles; i++)
   {
      if (muscleList[i])
      {
         if (copy_muscle_dp(ms->muscle[i], &dp->muscles[index++], ms->default_muscle, &dp->default_muscle, dp) == code_bad)
         {
            // if there's a problem, not going to create model, need to free muscle struct
            FREE_IFNOTNULL(dp->muscles);

            return code_bad;
         }

         /* DKB: Sept. 2009 - when making the pipeline model, some segments are skippable,
          * they don't affect the dynamics, so they are not included in the model.  Therefore,
          * the order of the segments in SIMM (ms) and pipeline (dp) DO NOT CORRESPOND!  All
          * segment references need to be updated so they are correct in the DP model.
          * As well, the pipeline assumes that all muscle points are relative to the segment's mass
          * center, not it's origin.  When the muscle file is read in in the stand-alone, the mass-center
          * is subtracted.  Make sure to do this here for all muscle points.
          */
         for (j = 0; j < dp->muscles[index-1].path->num_orig_points; j++)
         {
            int simm_seg = dp->muscles[index-1].path->mp_orig[j].segment;
            dp->muscles[index-1].path->mp_orig[j].segment = get_sd_seg_num(ms->segment[simm_seg].name);
            dp->muscles[index-1].path->mp_orig[j].point[XX] -= ms->segment[simm_seg].masscenter[XX];
            dp->muscles[index-1].path->mp_orig[j].point[YY] -= ms->segment[simm_seg].masscenter[YY];
            dp->muscles[index-1].path->mp_orig[j].point[ZZ] -= ms->segment[simm_seg].masscenter[ZZ];
         }
         /* Note: because the wrapStruct has a wrap_object pointer to the actual wrap_object in
          * the dpModel, the segment stored in the dp model does not need to be adjusted. */

      }
   }
   return code_fine;
}

static dpWrapObject* getDPWrapObject(dpModelStruct* dp, char name[])
{
   int i;
   
   if (name == NULL)
      return NULL;
   
   for (i = 0; i < dp->num_wrap_objects; i++)
      if (STRINGS_ARE_EQUAL(name, dp->wrap_object[i].name))
         return &dp->wrap_object[i];
      
   return NULL; 
}


static void copyWrapObjectsToDP(ModelStruct* ms, dpModelStruct* dp)
{
   int i;

   dp->num_wrap_objects = ms->num_wrap_objects;

   if (dp->num_wrap_objects == 0)
   {
      dp->wrap_object = NULL;
      return;
   }

   dp->wrap_object = (dpWrapObject*)simm_malloc(dp->num_wrap_objects * sizeof(dpWrapObject));

   for (i = 0; i < dp->num_wrap_objects; i++)
   {
      dpWrapObject* from = ms->wrapobj[i];
      dpWrapObject* to = &dp->wrap_object[i];

      memcpy(to, from, sizeof(dpWrapObject));
      mstrcpy(&to->name, from->name);
//check that xform is being copied properly, make sure that from->segment is the correct index  dkb
      to->segment = get_sd_seg_num(ms->segment[from->segment].name);
      to->from_local_xform[3][XX] -= ms->segment[from->segment].masscenter[XX];
      to->from_local_xform[3][YY] -= ms->segment[from->segment].masscenter[YY];
      to->from_local_xform[3][ZZ] -= ms->segment[from->segment].masscenter[ZZ];
      invert_4x4transform(to->from_local_xform, to->to_local_xform);
   }
}


static void copyQsToDP(ModelStruct* ms, dpModelStruct* dp)
{
   int i, j;
   DofStruct* dof;
   JointStruct* jnt;
   dpQStruct* q;
   GeneralizedCoord* gc;

   dp->nu = dp->nq;
   dp->num_gencoords = 0;

   if (dp->nq == 0)
   {
      dp->q = NULL;
      return;
   }

   dp->q = (dpQStruct*)simm_malloc(dp->nq * sizeof(dpQStruct));

   for (i = 0; i < dp->nq; i++)
   {
      dof = find_nth_q_dof(ms, i);
      jnt = find_nth_q_joint(ms, i);
      gc = dof->gencoord;
      q = &dp->q[i];

      mstrcpy(&q->name, dof->sd.name);

      if (dof->sd.fixed == yes)
      {
         q->type = dpFixedQ;
      }
      else if (dof->sd.constrained == no)
      {
         /* Locked gencoords are modeled as fixed Qs (as of version 4.1.1). */
         if (gc && gc->locked == yes)
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
         q->restraint_function = NULL;
         q->min_restraint_function = NULL;
         q->max_restraint_function = NULL;
         q->function_active = dpNo;
      }
      else
      {
         if (gc->restraint_function)
         {
            // The index of the functions in the ModelStruct and the dpModelStruct should be the same.
            q->restraint_function = dp->function[getFunctionIndex(ms, gc->restraint_function)];
            q->min_restraint_function = NULL;
            q->max_restraint_function = NULL;
            if (gc->restraintFuncActive == yes)
               q->function_active = dpYes;
            else
               q->function_active = dpNo;
         }
         else
         {
            q->restraint_function = NULL;
            q->function_active = dpNo;

            // The index of the functions in the ModelStruct and the dpModelStruct should be the same.
            if (gc->min_restraint_function)
               q->min_restraint_function = dp->function[getFunctionIndex(ms, gc->min_restraint_function)];
            else
               q->min_restraint_function = NULL;

            if (gc->max_restraint_function)
               q->max_restraint_function = dp->function[getFunctionIndex(ms, gc->max_restraint_function)];
            else
               q->max_restraint_function = NULL;
         }
      }
      if (dof->sd.constrained == yes)
      {
         DofStruct* ind_dof = find_unconstrained_sd_dof(ms, dof->gencoord);
         // The index of the functions in the ModelStruct and the dpModelStruct should be the same.
         q->constraint_function = dp->function[getFunctionIndex(ms, dof->function)];
         q->constraint_num = dof->sd.error_number;
         q->q_ind = ind_dof->sd.state_number;
      }
      else
      {
         q->constraint_function = NULL;
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


static void copySegmentsToDP(ModelStruct* ms, dpModelStruct* dp)
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


void copyPolyhedronToDPPolyhedron(PolyhedronStruct* from, dpPolyhedronStruct** to,
                                             int SimmSegNum, int SDSegNum, ModelStruct* ms)
{
   int i, j;
   dpPolyhedronStruct* ph;
   dpVertexStruct* v;

   ph = *to = (dpPolyhedronStruct*)simm_malloc(sizeof(dpPolyhedronStruct));

   mstrcpy(&ph->name, from->name);
   ph->body_segment = SDSegNum;
   ph->bc.x1 = from->bc.x1 - ms->segment[SimmSegNum].masscenter[XX];
   ph->bc.x2 = from->bc.x2 - ms->segment[SimmSegNum].masscenter[XX];
   ph->bc.y1 = from->bc.y1 - ms->segment[SimmSegNum].masscenter[YY];
   ph->bc.y2 = from->bc.y2 - ms->segment[SimmSegNum].masscenter[YY];
   ph->bc.z1 = from->bc.z1 - ms->segment[SimmSegNum].masscenter[ZZ];
   ph->bc.z2 = from->bc.z2 - ms->segment[SimmSegNum].masscenter[ZZ];

   ph->num_vertices = from->num_vertices;
   ph->vertex = (dpVertexStruct*)simm_malloc(ph->num_vertices * sizeof(dpVertexStruct));
   for (i = 0; i < ph->num_vertices; i++)
   {
      for (j = 0; j < 3; j++)
      {
         ph->vertex[i].coord[j] = from->vertex[i].coord[j] - ms->segment[SimmSegNum].masscenter[j];
         ph->vertex[i].normal[j] = from->vertex[i].normal[j];
      }
      ph->vertex[i].polygon_count = from->vertex[i].polygon_count;
      ph->vertex[i].polygons = (int*)simm_malloc(ph->vertex[i].polygon_count * sizeof(int));
      for (j = 0; j < ph->vertex[i].polygon_count; j++)
         ph->vertex[i].polygons[j] = from->vertex[i].polygons[j];
   }

   ph->num_polygons = from->num_polygons;
   ph->polygon = (dpPolygonStruct*)simm_malloc(ph->num_polygons * sizeof(dpPolygonStruct));
   for (i = 0; i < ph->num_polygons; i++)
   {
      ph->polygon[i].normal[XX] = from->polygon[i].normal[XX];
      ph->polygon[i].normal[YY] = from->polygon[i].normal[YY];
      ph->polygon[i].normal[ZZ] = from->polygon[i].normal[ZZ];
      ph->polygon[i].num_vertices = from->polygon[i].num_vertices;
      ph->polygon[i].vertex_index = (int*)simm_malloc(ph->polygon[i].num_vertices * sizeof(int));
      for (j = 0; j < ph->polygon[i].num_vertices; j++)
         ph->polygon[i].vertex_index[j] = from->polygon[i].vertex_index[j];
      v = &ph->vertex[ph->polygon[i].vertex_index[0]];
      ph->polygon[i].d = -v->coord[XX] * ph->polygon[i].normal[XX] -
                         v->coord[YY] * ph->polygon[i].normal[YY] -
                         v->coord[ZZ] * ph->polygon[i].normal[ZZ];
   }
}


/* This routine finds the nth rotation [axis] in a joint. For example,
 * if the order of the joint is: r1 t r3 r2, r1 is the first rotation,
 * r3 is the second rotation, and r2 is the third rotation. order[0]
 * holds the position of the translation, order[1] holds the position
 * of the r1 rotation, etc.
 */
int find_nth_rotation(JointStruct* jnt, int n)
{
   int i, min = 9, min_index, max = -9, max_index;

   for (i=1; i<4; i++)
   {
      if (jnt->order[i] < min)
      {
         min = jnt->order[i];
         min_index = i;
      }
      if (jnt->order[i] > max)
      {
         max = jnt->order[i];
         max_index = i;
      }
   }

   if (n == 1)
      return min_index;

   if (n == 3)
      return max_index;

   for (i=1; i<4; i++)
      if (i != min_index && i != max_index)
     return i;
   
   return -1;
}

/* returns the dof structure which corresponds to the nth Q in the SD/FAST code */

DofStruct* find_nth_q_dof(ModelStruct* ms, int n)
{
   int i, j;

   for (i=0; i<ms->numjoints; i++)
   {
      for (j=0; j<6; j++)
      {
         if ((ms->joint[i].dofs[j].type == function_dof ||
            ms->joint[i].dofs[j].sd.fixed == yes) &&
            ms->joint[i].dofs[j].sd.state_number == n)
            return &ms->joint[i].dofs[j];
      }
   }

   return NULL;
}


/* returns the joint structure which contains the nth Q in the SD/FAST code */

JointStruct* find_nth_q_joint(ModelStruct* ms, int n)
{
   int i, j;

   for (i=0; i<ms->numjoints; i++)
   {
      for (j=0; j<6; j++)
      {
         if (ms->joint[i].dofs[j].type == function_dof &&
            ms->joint[i].dofs[j].sd.state_number == n)
            return &ms->joint[i];
         if (ms->joint[i].dofs[j].sd.fixed == yes &&
            ms->joint[i].dofs[j].sd.state_number == n)
            return &ms->joint[i];
      }
   }

   return NULL;
}


DofStruct* find_unconstrained_sd_dof(ModelStruct* ms, GeneralizedCoord* gencoord)
{
   int i, j;

   for (i=0; i<ms->numjoints; i++)
   {
      for (j=0; j<6; j++)
      {
         if (ms->joint[i].dofs[j].sd.constrained == no &&
            ms->joint[i].dofs[j].gencoord == gencoord)
            return &ms->joint[i].dofs[j];
      }
   }

   return NULL;
}


/* GET_SD_SEG_NUM: given a segment name, this function returns the
 * number of that segment in the Pipeline code. It assumes that the
 * SDseg[] array has already been filled (which happens as the
 * SD/FAST model file is generated).
 */
int get_sd_seg_num(char simm_name[])
{
   int i;
   char* name;

   /* When SDseg[] was created, the body segment names were
    * cleaned-up using convert_string(). So call it here too so
    * that the names will match exactly. Malloc enough space for
    * 'name' so that there's room for a leading underscore, if
    * convert_string needs to add one.
    */
   name = (char*)simm_malloc(strlen(simm_name) + 2);
   strcpy(name, simm_name);
   convert_string(name, yes);

   for (i=0; i<num_SD_segs; i++)
   {
      if (STRINGS_ARE_EQUAL(name,SDseg[i].name))
         break;
   }

   free (name);

   if (i == num_SD_segs)
      return -1;
   else
      return i-1;
}
