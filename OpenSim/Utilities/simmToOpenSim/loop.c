/*******************************************************************************

   LOOP.C

   Author: Krystyne Blaikie

   Date: 20-NOV-00

   Copyright (c) 2000 MusculoGraphics, Inc.
   All rights reserved.

   Description: 

   Routines:

*******************************************************************************/
#include "universal.h"
#include "functions.h"
#include "globals.h"


/*************** DEFINES (for this file only) *********************************/
#if EXPERIMENTAL
#define DEBUG_LEVEL 1
#endif


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/


/*************** GLOBAL VARIABLES (used in only a few files) ******************/


/*************** EXTERNED VARIABLES (declared in another file) ****************/


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/


/* Given the list of joints and segments in the closed loop, find the
 * path from the given segment to the "first" segment in the loop in the
 * specified direction.  Return the path length.
 */
int getLoopPath(int seg, int *path, int dir, LoopStruct *loop)
{
   int i, j;

   if (dir == INVERSE)
   {
      /* find the path from the given segment, backwards through the array
       * to the first segment.  Joints used in reverse direction. */
      i = 0;
      for (j = seg-1; j >= 0; j--)
      {
         path[i++] = -1 * loop->joints[j];
      }
      path[i] = 0;
   }
   else
   {
      /* make a list of joints from the given segment to the end of the
       * joint array.  Joints used in forward direction */
      i = 0;
      for (j = seg; j < loop->num_jnts; j++)
      {
         path[i++] = loop->joints[j];
      }
      path[i] = 0;
   }
   return (i);
}

/* Set the gencoord values without affecting the display.
 * If the given gencoord value is different than the value in the model,
 * reset the value and invalidate all transformation matrices that use
 * that gencoord.  If the gencoord value has not changed, return 0.  If the
 * gencoord is clamped and the value is out of range, return 2.  Else,
 * return 1.
 */
int setGencoordValue2(ModelStruct *ms, GeneralizedCoord* gencoord, double value)
{
   int i;
   
   /* if the new value is the same as the old value, check if the value 
    * is out of range and return.
    */
   if (DABS(value - gencoord->value) <= gencoord->tolerance)
   {
      if (gencoord->clamped == yes)
      {
         if (value < gencoord->range.start)
            return 2;
         if (value > gencoord->range.end)
            return 3;
      }   
      return 0;
   }

   if (gencoord->type == rotation_gencoord)
      checkGencoordRange(gencoord, &value);
   gencoord->value = value;

   /* invalidate all joint matrices that use the gencoord */
   for (i=0; i<gencoord->numjoints; i++)
      invalidate_joint_matrix(ms, &ms->joint[gencoord->jointnum[i]]);
   
   if (gencoord->clamped == yes)
   {
      if (value < gencoord->range.start)
         return 2;
      if (value > gencoord->range.end)
         return 3;
   }   

   return 1;   
}

/* convert a point from one frame to another across a loop joint.
 * Don't use the paths between the bodies since these don't include
 * the loop joint.  If one of the bodies is ground, still use the
 * the actual joint given so the loop joint is used to calculate
 * the transform */
int convert2(ModelStruct *ms, double p[], int n1, int n2)
{
   
   int i;
   double result[3];
   DMatrix* mat;
   
   if (n1 == n2)
      return (0);
   
   /* If you want to convert a point in A to B, and the joint is defined A -> B,
    * then you need to use the inverse transformation matrix.
    */
   for (i = 0; i < ms->numjoints; i++)
   {
      if ((ms->joint[i].from == n1) && (ms->joint[i].to == n2))
      {
         mat = get_conversion(ms, i, INVERSE);
         break;
      } 
      else if ((ms->joint[i].from == n2) && (ms->joint[i].to == n1))
      {
         mat = get_conversion(ms, i, FORWARD);
         break;
      }
   }
   if (i == ms->numjoints)
   {
      printf("No joint from %d to %d\n", n1, n2);
   }
   
   result[0] = p[0]*(*mat)[0][0] + p[1]*(*mat)[1][0] +
      p[2]*(*mat)[2][0] + (*mat)[3][0];
   result[1] = p[0]*(*mat)[0][1] + p[1]*(*mat)[1][1] +
      p[2]*(*mat)[2][1] + (*mat)[3][1];
   result[2] = p[0]*(*mat)[0][2] + p[1]*(*mat)[1][2] +
      p[2]*(*mat)[2][2] + (*mat)[3][2];
   
   p[0] = result[0];
   p[1] = result[1];
   p[2] = result[2];
   
   return 1;
   
}


/* create structures to hold info about each closed loop.  Store joints
 * and segments in the loops. */
SBoolean makeLoops(ModelStruct *ms)
{
int i, j, m, n, index, segments;
int from, to, jnt, dir;
int num_loops, num_segs, num_jnts;
char buffer[255];
int loop_joint, user_loop, num_user_loop_joints;
SBoolean changed;

   if (ms->numclosedloops == 0)
      return no;

   segments = ms->numsegments;
   ms->loop = (LoopStruct *)simm_malloc(ms->numclosedloops * sizeof(LoopStruct));

   for (i = 0; i < ms->numclosedloops; i++)
   {
      ms->loop[i].model_num = ms->modelnum;
      ms->loop[i].ms = ms;
      ms->loop[i].first_iter = yes;
   }

   num_loops = 0;
   for (i = 0; i < ms->numjoints; i++)
   {
      if (ms->joint[i].loop_joint == yes)
      {
         num_jnts = 0;
         num_segs = 0;
         index = ms->joint[i].from * segments + ms->joint[i].to;
         for (j = 0; ms->pathptrs[index][j] != (ms->numjoints + 1); j++)
            num_jnts++;
         
         num_jnts += 1;
         num_segs = num_jnts + 1;
         ms->loop[num_loops].segs = (int *)simm_malloc(num_segs * sizeof(int));
         ms->loop[num_loops].joints = (int *)simm_malloc(num_jnts * sizeof(int));
         ms->loop[num_loops].num_segs = num_segs;
         ms->loop[num_loops].num_jnts = num_jnts;
         num_loops++;
      }
   }

   /* if there are any closed loops in the model, store which 
    * segments are used in the closed loop.  Look at the bodies in
    * the loop joint, and copy the path connecting the two bodies (m->n) (which
    * goes around the loop without going through loop joint), then add the
    * beginning segment (m) to the end of the list.
    */
   num_loops = 0;
   for (i = 0; i < ms->numjoints; i++)
   {
      if (ms->joint[i].loop_joint == yes)
      {
         m = ms->joint[i].from;
         n = ms->joint[i].to;

         index = m * segments + n;
         if (ms->pathptrs[index] == NULL)
         {
            (void)sprintf(buffer,
               "No path between %s and %s in closed loop", ms->segment[m].name,
                ms->segment[n].name);
             error(none, buffer);
         }
         /* copy existing path between m and n */
         for (j = 0; ms->pathptrs[index][j] != (ms->numjoints + 1); j++)
            ms->loop[num_loops].joints[j] = ms->pathptrs[index][j];

         /* have path from m->n now add path from n->m (loop joint), reverse dir */
         ms->loop[num_loops++].joints[j] = -(i + 1);
      }
   }
   
   for (i = 0; i < num_loops; i++)
   {
      for (j = 0; j < ms->loop[i].num_jnts; j++)
      {
         jnt = ms->loop[i].joints[j];
         if (jnt > 0)
         {
            dir = FORWARD;
            jnt = jnt - 1;
            from = ms->joint[jnt].from;
            to = ms->joint[jnt].to;
         }
         else
         {
            dir = INVERSE;
            jnt = abs(jnt) - 1;
            to = ms->joint[jnt].from;
            from = ms->joint[jnt].to;
         }
         if (j == 0)
            ms->loop[i].segs[0] = from;
         ms->loop[i].segs[j+1] = to;
      }
   }

   changed = no;
   for (i = 0; i < ms->numclosedloops; i++)
   {
      loop_joint = -1;
      user_loop = -1;
      num_user_loop_joints = 0;
      for (j = 0; j < ms->loop[i].num_jnts; j++)
      {
         jnt = abs(ms->loop[i].joints[j]) - 1;
         if (ms->joint[jnt].user_loop == yes)
         {
            user_loop = jnt;
            ms->joint[jnt].user_loop = no;
            num_user_loop_joints++;
         }
         if (ms->joint[jnt].loop_joint == yes)
            loop_joint = jnt;
      }
      if ((user_loop != -1) && (user_loop != loop_joint))
      {
         ms->joint[loop_joint].loop_joint = no;
         ms->joint[user_loop].loop_joint = yes;
         changed = yes;
      }
      if (num_user_loop_joints > 1)
      {
         sprintf(errorbuffer, "Too many loopjoints defined in model for loop %d.\n", i);
         error(none, errorbuffer);
      }
   }

////////////// display
   /*
   for (i = 0; i < ms->numclosedloops; i++)
   {
      printf("loopjoint%d\n", i); 
      for (j = 0; j < ms->loop[i].num_jnts; j++)
      {
         jnt = ms->loop[i].joints[j];
         if (jnt > 0)
            jnt = jnt - 1;
         else
            jnt = abs(jnt) - 1;
         printf("%s ", ms->joint[jnt].name);
      }
      printf("\n");
      printf("segs = \n");
      for (j = 0; j < ms->loop[i].num_segs; j++)
         printf("%s ", ms->segment[ms->loop[i].segs[j]].name);
      printf("\n");
   }
*/
   ////////////////

   return changed;
}


/* store the number of qs and residuals for each each loop.  Store the
 * gencoord indices for each q */
void updateLoopInfo(ModelStruct *ms)
{
   int i, j, k, nq, jnt;
   GeneralizedCoord* gc;

   for (i = 0; i < ms->numclosedloops; i++)
   {
      nq = 0;
      ms->loop[i].qs = (GeneralizedCoord**)simm_malloc(6 * ms->loop[i].num_jnts * sizeof(GeneralizedCoord*));
      for (j = 0; j < ms->loop[i].num_jnts; j++)
      {
         jnt = abs(ms->loop[i].joints[j]) - 1;
         for (k = 0; k < 6; k++)
         {
            gc = ms->joint[jnt].dofs[k].gencoord;
            if (gc && gc->used_in_model == yes)
            {
               gc->used_in_loop = yes;
               ms->loop[i].qs[nq] = gc;
               nq++;
            }
         }
      }
      ms->loop[i].num_qs = nq;
      ms->loop[i].num_resids = 3 * ms->loop[i].num_jnts;
   }

#if EXPERIMENTAL
   if (DEBUG_LEVEL > 0)
   {
      printf("\nStore loop info\n");
      for (i = 0; i < ms->numclosedloops; i++)
      {
         printf("loop%d:\n", i);
         printf("  num_jnts = %d\n  num_segs = %d\n", ms->loop[i].num_jnts,
            ms->loop[i].num_segs);
         printf("  num_qs = %d\n  num_resids = %d\n", ms->loop[i].num_qs,
            ms->loop[i].num_resids);
         printf("  jnts: ");
         for (j = 0; j < ms->loop[i].num_jnts; j++)
            printf(" %d", abs(ms->loop[i].joints[j]) -1);
         printf("\n  segs: ", i);
         for (j = 0; j < ms->loop[i].num_segs; j++)
            printf("%d ", ms->loop[i].segs[j]);
         printf("\n");
         printf("\n  segs: ", i);
         for (j = 0; j < ms->loop[i].num_segs; j++)
            printf("%s ", ms->segment[ms->loop[i].segs[j]].name);
         printf("\n");
/*         printf("  qs: ");
         for (j = 0; j < ms->loop[i].num_qs; j++)
            printf("%d ", ms->loop[i].qs[j]);
         printf("\n");
*/
         printf("  qs: ");
         for (j = 0; j < ms->loop[i].num_qs; j++)
            printf("%s ", ms->loop[i].qs[j]->name);
         printf("\n");
      }
   }
#endif
}

/* Determine whether any closed loops exist in the model.  If so, mark
 * one of the joints in the closed loop as the loop joint.  To determine
 * whether closed loops exist, go through the joint list and mark each segment
 * used in the joint.  If both segments in a joint have already been used, 
 * mark that joint as the loop joint.
 */
void markLoopJoints(ModelStruct *ms)
{
   int i, from, to, joints_handled, joints_handled_last_time, loop_jnt = -1;
   SBoolean *segment_used, *joint_used;

   /* initialize segment_used array */
   segment_used = (SBoolean *)simm_malloc(ms->numsegments * sizeof(SBoolean));
   for (i = 0; i < ms->numsegments; i++)
      segment_used[i] = no;

   /* initialize joint_used array */
   joint_used = (SBoolean *)simm_malloc(ms->numjoints * sizeof(SBoolean));
   for (i = 0; i < ms->numjoints; i++)
      joint_used[i] = no;

   /* find closed loops. Look at each joint and see if the segments defining
    * it have been used before:
    *    if both have been used, the joint closes a loop
    *    if one has been used, mark the other as used
    *    if neither has been used, you can't process the joint yet.
    * Keep going until all joints have been processed.
    */
   ms->numclosedloops = 0;
   segment_used[0] = yes;
   joints_handled = 0;

   while (joints_handled < ms->numjoints)
   {
      joints_handled_last_time = joints_handled;
      for (i=0; i<ms->numjoints; i++)
      {
         if (joint_used[i] == no)
         {
            from = ms->joint[i].from;
            to = ms->joint[i].to;
            if ((segment_used[from] == yes) && (segment_used[to] == yes))
            {
               ms->joint[i].loop_joint = yes;
               joint_used[i] = yes;
               ms->numclosedloops++;
               joints_handled++;
            }
            else if (segment_used[from] == yes)
            {
               segment_used[to] = yes;
               joint_used[i] = yes;
               joints_handled++;
            }
            else if (segment_used[to] == yes)
            {
               segment_used[from] = yes;
               joint_used[i] = yes;
               joints_handled++;
            }
         }
      }
      /* If you did not process any joints this time around,
       * then there's a problem (e.g., a missing joint).
       * So break out and let makepaths() report the error later.
       */
      if (joints_handled_last_time == joints_handled)
         break;
   }

#if EXPERIMENTAL
   if (DEBUG_LEVEL > 0)
   {
      printf("Model %s: %d closed loops\n", ms->name, ms->numclosedloops);
      for (i = 0; i < ms->numjoints; i++)
      {
         printf("joint %d (%s->%s)", i, ms->segment[ms->joint[i].from].name,
            ms->segment[ms->joint[i].to].name);
         if (ms->joint[i].loop_joint == yes)
            printf(" - loop joint");
         printf("\n");
      }
      for (i = 0; i < ms->numgencoords; i++)
      {
         printf("gencoord%d = %-20s", i, ms->gencoord[i]->name);
         printf(": range % 10.3f -> % 10.3f\n", ms->gencoord[i]->range.start,
               ms->gencoord[i]->range.end);
      }
   }
#endif

   free(segment_used);
   free(joint_used);
}


void checkGencoordRange(GeneralizedCoord* gencoord, double *value)
{
   long count;
   double min1, min2, min, max1, max2, max, mid;

   if (gencoord->clamped == yes)
      return;

   mid = (gencoord->range.start + gencoord->range.end)/2;
   max1 = mid + 180;
   min1 = mid - 180;

   min2 = gencoord->range.start;
   max2 = gencoord->range.end;

   max = _MAX(max1, max2) + 180;
   min = _MIN(min1, min2) - 180;

   if (*value > max)
   {
      count = (*value - max + 359.99999) / 360.0;
      *value -= 360.0 * count;
   }
   else if (*value < min)
   {
      count = (min - *value + 359.99999) / 360.0;
      *value += 360.0 * count;
   }
}
