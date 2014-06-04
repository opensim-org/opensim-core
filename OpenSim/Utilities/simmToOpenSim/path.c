/*******************************************************************************

   PATH.C

   Author: Peter Loan

   Date: 8-MAY-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains routines which find paths between
      the segments (bones) in a model.

   Routines:
      makepaths  : main routine to find paths between every pair of segments
      printpaths : prints out the path list for debugging
      enterpath  : trys to add a joint onto an existing path
      setpath    : given a path, copies it into the path list

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"


/*************** DEFINES (for this file only) *********************************/



/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static int enterpath(ModelStruct* ms, int j, int k, int newend);
static ReturnCode setpath(ModelStruct* ms, int list[]);



/* MAKEPATHS: */

ReturnCode makepaths(ModelStruct* ms)
{
   int i, j, k, m, n, jnt, segments, old_count, np;
   int* path = NULL;
   int from, to, nsq, nj, index, total, numpaths = 0;
   Direction dir;

   if (ms->numsegments == 1)
   {
      ms->pathptrs = (int **)simm_malloc(sizeof(int*));
      ms->pathptrs[0] = (int*)simm_malloc(sizeof(int));
      ms->pathptrs[0][0] = 0;
      return code_fine;
   }

   for (i=0, nj=0; i<ms->numjoints; i++)
   {
      if (ms->joint[i].loop_joint == no)
      {
         nj++;
      }
   }  
   segments = ms->numsegments;
   nsq = segments*segments;
   total = segments*segments - segments - 2*nj;

   if (ms->pathptrs != NULL)
   {
      for (i=0; i<nsq; i++)
         FREE_IFNOTNULL(ms->pathptrs[i]);
      free(ms->pathptrs);
   }

   ms->pathptrs = (int **)simm_malloc(nsq*sizeof(int*));
   if (ms->pathptrs == NULL)
      return code_bad;

   for (i=0; i<nsq; i++)
      ms->pathptrs[i] = (int *)0;

   for (i=0; i<segments; i++)
   {
      path = (int*)simm_malloc(3*sizeof(int));
      path[0] = path[1] = i;
      path[2] = END_OF_ARRAY;
      if (setpath(ms, path) == code_bad) // setpath takes ownership of path
         return code_bad;
   }

   for (i=0; i<ms->numjoints; i++)
   {
      if (ms->joint[i].loop_joint == no)
      {
         path = (int*)simm_malloc(3*sizeof(int));
         path[0] = ms->joint[i].from;
         path[1] = ms->joint[i].to;
         path[2] = END_OF_ARRAY;
         if (setpath(ms, path) == code_bad) // setpath takes ownership of path
            return code_bad;

         path = (int*)simm_malloc(3*sizeof(int));
         path[0] = ms->joint[i].to;
         path[1] = ms->joint[i].from;
         path[2] = END_OF_ARRAY;
         if (setpath(ms, path) == code_bad) // setpath takes ownership of path
            return code_bad;
      }
   }

   while (numpaths < total)
   {
      old_count = numpaths;
      for (i=0; i<ms->numjoints; i++)
      {
         if (ms->joint[i].loop_joint == yes)
            continue;
         m = ms->joint[i].from;
         n = ms->joint[i].to;
         for (j=0; j<segments; j++)
         {
            if (j != m && j != n)
            {
               index = j*segments+m;
               if (ms->pathptrs[index] != NULL)
               {
                  if ((np = enterpath(ms, j, m, n)) == -1)
                     return code_bad;
                  else
                  {
                     numpaths += np;
                   //  printf("added %d to path[%d]: %d -> %d\n", n, j*segments + n, j, k);
                  }
               }
               index = j*segments+n;
               if (ms->pathptrs[index] != NULL)
               {
                  if ((np = enterpath(ms, j, n, m)) == -1)
                     return code_bad;
                  else
                  {
                     numpaths += np;
                   //  printf("added %d to path[%d]: %d -> %d\n", m, j*segments+m, j, k);
                  }
               }
            }
         }
      }
      if (numpaths == old_count && numpaths < total)
      {
         message("Error forming model: cannot find joint path between the following pairs of segments:",0,DEFAULT_MESSAGE_X_OFFSET);
         for (i=0; i<segments; i++)
         {
            for (j=i+1; j<segments; j++)
            {
               index = i*segments+j;
               if (ms->pathptrs[index] == NULL)
               {
                  (void)sprintf(buffer,"  (%s,%s)", ms->segment[i].name,
                     ms->segment[j].name);
                  message(buffer, 0, DEFAULT_MESSAGE_X_OFFSET+20);
               }
            }
         }
         return code_bad;
      }
   }

  /* Now that the paths have been made, turn them into a list of joints,
   * rather than body segments. A list of joints is faster to traverse,
   * and this will improve the speed of the See system. In these joint
   * lists, negative numbers refer to inverse joints, joint numbers start
   * at 1 (because you can't have -0) and a value of numjoints+1 marks the
   * end of each list.
   */
   for (i=0; i<segments; i++)
   {
      for (j=0; j<segments; j++)
      {
         index = i*segments+j;
         for (k=0; ms->pathptrs[index][k] != j; k++)
         {
            from = ms->pathptrs[index][k];
            to = ms->pathptrs[index][k+1];
            jnt = find_joint_between_frames(ms, from, to, &dir);
            if (dir == FORWARD)
               ms->pathptrs[index][k] = jnt+1;
            else
               ms->pathptrs[index][k] = -(jnt+1);
         }
         ms->pathptrs[index][k] = ms->numjoints+1;
//         ms->pathptrs[index][j] = END_OF_ARRAY;
      }
   }

  /* For each joint, do the following:
   * for each segment, check to see if the joint is used in the path from
   * that segment to ground.
   */
   for (k=0; k<ms->numjoints; k++)
   {
      if (ms->joint[k].in_seg_ground_path == NULL)
         ms->joint[k].in_seg_ground_path = (SBoolean*)simm_malloc(ms->numsegments*sizeof(SBoolean));
      for (i=0; i<ms->numsegments; i++)
      {
         ms->joint[k].in_seg_ground_path[i] = no;
         
         if (i == ms->ground_segment)
            continue;
         
         path = GET_PATH(ms->modelnum,ms->ground_segment,i);
         
         for (j=0; path[j] != ms->numjoints+1; j++)
            if (ABS(path[j])-1 == k)
               ms->joint[k].in_seg_ground_path[i] = yes;
      }
   }
   return code_fine;
}


void printpaths(ModelStruct* ms)
{
   int i, j, k, index, jnt;

   printf("\n\n*******************\n");
   printf("from:   to:   \n");
   for (i=0; i<ms->numsegments; i++)
   {
      for (k=0; k<ms->numsegments; k++)
      {
         index = i*ms->numsegments+k;
         printf("path from %s to %s: ", ms->segment[i].name,
            ms->segment[k].name);
         if (ms->pathptrs[index] == NULL)
         {
            printf("empty\n");
            continue;
         }
         if (ms->pathptrs[index][0] == ms->numjoints+1)
         {
            printf("empty\n");
            continue;
         }
         for (j=0; ms->pathptrs[index][j] != ms->numjoints+1; j++)
//         for (j=0; ms->pathptrs[index][j] != END_OF_ARRAY; j++)
         {
            jnt = ABS(ms->pathptrs[index][j]) - 1;
            printf("%s ", ms->joint[jnt].name);
    //        printf("%d ", ms->pathptrs[index][j]);
         }
         printf("\n");
      }
   }
   printf("\n*******************\n\n");

}


/* ENTERPATH: This routine finds the path from j to k in the pathptrs
 * list, adds newend onto the end of it, and puts the result back in
 * pathptrs as the path from j to newend.
 */
static int enterpath(ModelStruct* ms, int j, int k, int newend)
{
   int length, *oldPath, *newPath;

   // If there is already a path from j to newend, just return.
   if (ms->pathptrs[j * ms->numsegments + newend] != NULL)
      return 0;

   oldPath = ms->pathptrs[j * ms->numsegments + k];
   length = get_array_length(oldPath);

   newPath = (int*)simm_malloc((length+2) * sizeof(int));
   memcpy(newPath, oldPath, length * sizeof(int));
   newPath[length] = newend;
   newPath[length+1] = END_OF_ARRAY;

   if (setpath(ms, newPath) == code_bad) // setpath takes ownership of newPath
      return -1;

   return 1;
}


/* SETPATH: given a list of segments, this routine finds which pathpointer
 * array should be equal to it, and sets it accordingly. For example, if
 * the list is 2 5 4 7 3, then the list is the path for the segment pair (2,3).
 * This function passes ownership of 'list' to model[]->pathptrs[].
 */
static ReturnCode setpath(ModelStruct* ms, int list[])
{
   int length, start, finish, index;

   if (list == NULL)
      return code_bad;

   start = list[0];
   length = get_array_length(list);
   finish = list[length - 1];
   index = start * ms->numsegments + finish;

   ms->pathptrs[index] = list;

   return code_fine;
}
