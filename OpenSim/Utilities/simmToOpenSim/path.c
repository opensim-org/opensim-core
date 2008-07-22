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
static int enterpath(int mod, int j, int k, int newend);
static ReturnCode setpath(int mod, int list[]);



/* MAKEPATHS: */

ReturnCode makepaths(int mod)
{
   int i, j, k, m, n, jnt, segments, old_count, np;
   ModelStruct* ms;
   int* path;
   int from, to, nsq, nj, index, total, numpaths = 0;
   Direction dir;

   ms = model[mod];

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
      {
         FREE_IFNOTNULL(ms->pathptrs[i]);
      }
      free(ms->pathptrs);
   }
   
   path = (int*)simm_malloc(nsq*sizeof(int));
   ms->pathptrs = (int **)simm_malloc(nsq*sizeof(int*));
   if (path == NULL || ms->pathptrs == NULL)
      return code_bad;
   
   for (i=0; i<nsq; i++)
      ms->pathptrs[i] = (int *)0;
   
   path[2] = END_OF_ARRAY;
   for (i=0; i<segments; i++)
   {
      path[0] = path[1] = i;
      if (setpath(mod,path) == code_bad)
         return code_bad;
   }
   
   for (i=0; i<ms->numjoints; i++)
   {
      if (ms->joint[i].loop_joint == yes)
         continue;
      m = ms->joint[i].from;
      n = ms->joint[i].to;
      path[0] = m;
      path[1] = n;
      if (setpath(mod,path) == code_bad)
         return code_bad;
      path[0] = n;
      path[1] = m;
      if (setpath(mod,path) == code_bad)
         return code_bad;
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
                  if ((np = enterpath(mod,j,m,n)) == -1)
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
                  if ((np = enterpath(mod,j,n,m)) == -1)
                     return (code_bad);
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
            for (j=i+1; j<segments; j++)
            {
               index = i*segments+j;
               if (ms->pathptrs[index] == NULL)
               {
                  (void)sprintf(buffer,"  (%s,%s)", ms->segment[i].name,
                     ms->segment[j].name);
                  message(buffer,0,DEFAULT_MESSAGE_X_OFFSET+20);
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
            jnt = find_joint_between_frames(mod,from,to,&dir);
            if (dir == FORWARD)
               ms->pathptrs[index][k] = jnt+1;
            else
               ms->pathptrs[index][k] = -(jnt+1);
         }
         ms->pathptrs[index][k] = ms->numjoints+1;
//         ms->pathptrs[index][j] = END_OF_ARRAY;
      }
   }

   //printpaths(mod);

   free(path);
   
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



void printpaths(int mod)
{

   int i, j, k, index, jnt;

   printf("\n\n*******************\n");
   printf("from:   to:   \n");
   for (i=0; i<model[mod]->numsegments; i++)
   {
      for (k=0; k<model[mod]->numsegments; k++)
      {
         index = i*model[mod]->numsegments+k;
         printf("path from %s to %s: ", model[mod]->segment[i].name,
            model[mod]->segment[k].name);
         if (model[mod]->pathptrs[index] == NULL)
         {
            printf("empty\n");
            continue;
         }
         if (model[mod]->pathptrs[index][0] == model[mod]->numjoints+1)
         {
            printf("empty\n");
            continue;
         }
         for (j=0; model[mod]->pathptrs[index][j] != model[mod]->numjoints+1; j++)
//         for (j=0; model[mod]->pathptrs[index][j] != END_OF_ARRAY; j++)
         {
            jnt = ABS(model[mod]->pathptrs[index][j]) - 1;
            printf("%s ", model[mod]->joint[jnt].name);
    //        printf("%d ", model[mod]->pathptrs[index][j]);
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

static int enterpath(int mod, int j, int k, int newend)
{

   int i, index, path[128]; /* TODO: static size */

   index = j*model[mod]->numsegments + k;

   /* If there is already a path from j to newend, just return */

   if (model[mod]->pathptrs[j*model[mod]->numsegments+newend] != NULL)
   {
      return (0);
   }
   
   i = 0;
   while (model[mod]->pathptrs[index][i] != k)
   {
      path[i] = model[mod]->pathptrs[index][i];
      i++;
   }

   path[i] = k;
   path[i+1] = newend;
   path[i+2] = END_OF_ARRAY;

   if (setpath(mod,path) == code_bad)
      return (-1);

   return (1);

}



/* SETPATH: given a list of segments, this routine finds which pathpointer
 * array should be equal to it. For example, if the list is 2 5 4 7 3,
 * then the list is the path for the segment pair (2,3).
 */

static ReturnCode setpath(int mod, int list[])
{

   int i, length, start, finish, index;

   start = list[0];
   finish = find_end_of_array(list,&length);
   index = start * model[mod]->numsegments + finish;

   model[mod]->pathptrs[index] = (int*)simm_malloc(length*sizeof(int));
   if (model[mod]->pathptrs[index] == NULL)
      return (code_bad);

   for (i=0; i<length; i++)
      model[mod]->pathptrs[index][i] = list[i];
   //printf("setpath %d: %d -> %d  :", index, start, finish);
   //for (i = 0 ; i < length; i++)
   //   printf(" %d ", list[i]);
   //printf("\n");
   return (code_fine);

}


