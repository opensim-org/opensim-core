/*******************************************************************************

 ENTER.C
 
  Author: Peter Loan
  
   Date: 8-DEC-88
   
    Copyright (c) 1992-5 MusculoGraphics, Inc.
    All rights reserved.
    Portions of this source code are copyrighted by MusculoGraphics, Inc.
    
     Description: This file contains routines that keep track of various
     model parameters, such as gencoords and segments. They store the user
     names and numbers for these parameters, but also generate internal
     numbers to reference the structures which hold them. For example,
     if in a model file, function #17 is defined first, this function
     will internally be referred to as function #0, and all references
     to it (by the number 17) will be replaced by references to 0.
     
      Routines:
      enter_gencoord    : stores a gencoord reference in a list and returns id
      enter_function    : stores a function reference in a list and returns id
      load_function     : stores a function definition in a list and returns id
      get_wrap_object   : scans array of wrap objects and returns id
      enter_segment     : stores a segment number in a list and returns id
      entergroup        : stores a muscle group name in a list and returns id
      enter_joint       : stores a joint number in a list and returns id
      
*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"


/*************** DEFINES (for this file only) *********************************/



/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/



/* ENTER_GENCOORD: whenever a gencoord is read from a file, the user-specified
 * number and name are compared to the existing array of gencoords to see
 * if that one has already been defined. If it has, the routine just returns
 * the internal number of that gencoord. If it has not yet been defined, it
 * adds the name and number to a new element in the gencoords array and
 * increments model.numgencoords, unless there is no permission to add it.
 * You don't want to add a gencoord to the list when you find a new one in a
 * muscle file. In this case, you want to print an error because this new
 * gencoord was not defined in the joints file.
 */

int enter_gencoord(int mod, char username[], SBoolean permission_to_add)
{
   
   int i, new_gc;
   ReturnCode rc;
   
   if (username == NULL)
      return INVALID_GENCOORD;
   
   for (i = 0; i < model[mod]->numgencoords; i++)
      if (STRINGS_ARE_EQUAL(username,model[mod]->gencoord[i].name))
         return i;

   if (permission_to_add == no)
      return INVALID_GENCOORD;

   new_gc = model[mod]->numgencoords;

   if (model[mod]->numgencoords == model[mod]->genc_array_size)
   {
      model[mod]->genc_array_size += GENC_ARRAY_INCREMENT;
      model[mod]->gencoord = (GeneralizedCoord*)simm_realloc(model[mod]->gencoord,
         model[mod]->genc_array_size*sizeof(GeneralizedCoord),&rc);
      if (rc == code_bad)
      {
         model[mod]->genc_array_size -= GENC_ARRAY_INCREMENT;
         return INVALID_GENCOORD;
      }
   }

   init_gencoord(&model[mod]->gencoord[new_gc]);
   mstrcpy(&model[mod]->gencoord[new_gc].name,username);

   model[mod]->numgencoords++;

   return new_gc;
}


/* ENTER_FUNCTION: whenever a function number is read from a file, the
 * user-specified number is compared to the existing array of functions to
 * see if that one has already been defined. If it has, the routine just
 * returns the internal number of that function. If it has not yet been
 * defined, it adds the number to a new element in the function list.
 */

int enter_function(int mod, int usernum, SBoolean permission_to_add)
{
   int i, new_func, old_count;
   ReturnCode rc1, rc2;

   if (usernum != UNDEFINED_USERFUNCNUM)
   {
      for (i=0; i<model[mod]->func_array_size; i++)
         if (usernum == model[mod]->function[i].usernum)
            return i;
   }

   if (permission_to_add == no)
      return INVALID_FUNCTION;

   for (i=0; i<model[mod]->func_array_size; i++)
      if (model[mod]->function[i].defined == no && model[mod]->function[i].used == no)
         break;
   new_func = i;

   if (new_func == model[mod]->func_array_size)
   {
      old_count = model[mod]->func_array_size;
      model[mod]->func_array_size += FUNC_ARRAY_INCREMENT;
      model[mod]->function = (SplineFunction*)simm_realloc(model[mod]->function,
         model[mod]->func_array_size*sizeof(SplineFunction),&rc1);
      model[mod]->save.function = (SplineFunction*)simm_realloc(model[mod]->save.function,
         model[mod]->func_array_size*sizeof(SplineFunction),&rc2);
      if (rc1 == code_bad || rc2 == code_bad)
      {
         model[mod]->func_array_size = old_count;
         return INVALID_FUNCTION;
      }

      for (i=old_count; i<model[mod]->func_array_size; i++)
      {
         model[mod]->function[i].defined = no;
         model[mod]->function[i].used = no;
         model[mod]->function[i].usernum = UNDEFINED_USERFUNCNUM;
         model[mod]->save.function[i].defined = no;
         model[mod]->save.function[i].used = no;
         model[mod]->save.function[i].usernum = UNDEFINED_USERFUNCNUM;
      }
   }

   model[mod]->function[new_func].usernum = usernum;
   model[mod]->function[new_func].used = yes;
   model[mod]->function[new_func].numpoints = 0;
   model[mod]->function[new_func].coefficient_array_size = 0;

   return new_func;
}


/* LOAD_FUNCTION: */

ReturnCode load_function(int mod, int usernum, SplineFunction* func)
{
   int fnum;

   fnum = enter_function(mod, usernum, yes);

   if (fnum == INVALID_FUNCTION)
      return code_bad;

   /* If the function has already been defined, print a warning,
    * and overwrite the previous definition with the new one.
    */
   if (model[mod]->function[fnum].defined == yes)
   {
      (void)sprintf(errorbuffer, "Warning: redefinition of function f%d. Replacing earlier definition.", usernum);
      error(none, errorbuffer);
   }

   model[mod]->function[fnum].type = func->type;
   model[mod]->function[fnum].numpoints = func->numpoints;
   model[mod]->function[fnum].coefficient_array_size = func->coefficient_array_size;
   model[mod]->function[fnum].x = func->x;
   model[mod]->function[fnum].y = func->y;
   model[mod]->function[fnum].b = func->b;
   model[mod]->function[fnum].c = func->c;
   model[mod]->function[fnum].d = func->d;

   model[mod]->function[fnum].defined = yes;

   return code_fine;
}


int get_wrap_object(ModelStruct* ms, char username[])
{
   
   int i;
   
   if (username == NULL)
      return (-1);
   
   for (i=0; i<ms->num_wrap_objects; i++)
      if (STRINGS_ARE_EQUAL(username,ms->wrapobj[i].name))
         return (i);
      
   return (-1); 
}



/* ENTER_SEGMENT: whenever a segment name is read from a file, the user-specified
 * name is compared to the existing array of segments to see if that one has
 * already been defined. If it has, the routine just returns the internal
 * number of that segment. If it has not yet been defined, it adds the name
 * to a new element in the segmentlist and increments model.numsegments, unless
 * there is no permission to add the new name. You don't really want to add a
 * new segment name when you find segments in a muscle file. In this case,
 * if the segment is not already known, you want to print an error message, because
 * the muscle is referencing a segment that was not defined in the joints file.
 */

int enter_segment(int mod, char username[], SBoolean permission_to_add)
{

   int i, new_seg;
   ReturnCode rc;

   if (username == NULL)
      return -1;

   for (i = 0; i < model[mod]->numsegments; i++)
      if (STRINGS_ARE_EQUAL(username, model[mod]->segment[i].name))
         return i;

   if (permission_to_add == no)
      return -1;

   new_seg = model[mod]->numsegments;

   if (model[mod]->numsegments == model[mod]->segment_array_size)
   {
      model[mod]->segment_array_size += SEGMENT_ARRAY_INCREMENT;
      model[mod]->segment = (SegmentStruct*)simm_realloc(model[mod]->segment,
         model[mod]->segment_array_size*sizeof(SegmentStruct),&rc);
      if (rc == code_bad)
      {
         model[mod]->segment_array_size -= SEGMENT_ARRAY_INCREMENT;
         return -1;
      }
   }

   init_segment(model[mod], &model[mod]->segment[new_seg]);
   mstrcpy(&model[mod]->segment[new_seg].name, username);

   model[mod]->numsegments++;

   return new_seg;
}



/* ENTER_GROUP: */

int enter_group(int mod, char username[], int muscle_number)
{
   
   int i, element, new_group;
   MuscleGroup* mg;
   ReturnCode rc;
   
   if (mod < 0 || muscle_number < 0)
      return (-1);
   
   for (i=0; i<model[mod]->numgroups; i++)
      if (STRINGS_ARE_EQUAL(username,model[mod]->muscgroup[i].name))
      {
         mg = &model[mod]->muscgroup[i];
         if (mg->number_of_muscles == mg->muscindex_array_size)
         {
            mg->muscindex_array_size += MUSCINDEX_ARRAY_INCREMENT;
            mg->muscle_index = (int*)simm_realloc(mg->muscle_index,
               mg->muscindex_array_size*sizeof(int),&rc);
            if (rc == code_bad)
            {
               mg->muscindex_array_size -= MUSCINDEX_ARRAY_INCREMENT;
               return (-1);
            }
         }
         element = mg->number_of_muscles++;
         mg->muscle_index[element] = muscle_number;
         return (i);
      }
      
      /* If you get to here, then the group name has not yet been recorded, meaning
      * that this is a new group. The variable "i" now equals the number of groups
      * already recorded, so it becomes the index of the group array where you
      * want to record this new group. First make sure the muscle group array is
      * big enough to hold the new group.
      */
      
      new_group = i;
      
      if (model[mod]->numgroups == model[mod]->muscgroup_array_size)
      {
         model[mod]->muscgroup_array_size += MUSCGROUP_ARRAY_INCREMENT;
         model[mod]->muscgroup = (MuscleGroup*)simm_realloc(model[mod]->muscgroup,
            model[mod]->muscgroup_array_size*sizeof(MuscleGroup),&rc);
         if (rc == code_bad)
         {
            model[mod]->muscgroup_array_size -= MUSCGROUP_ARRAY_INCREMENT;
            return (-1);
         }
      }
      
      mg = &model[mod]->muscgroup[new_group];
      
      mstrcpy(&mg->name,username);
      
      mg->muscindex_array_size = MUSCINDEX_ARRAY_INCREMENT;
      mg->muscle_index = (int*)simm_malloc(mg->muscindex_array_size*sizeof(int));
      if (mg->muscle_index == NULL)
         return (-1);
      
      mg->muscle_index[0] = muscle_number;
      mg->number_of_muscles = 1;
      mg->menu.title = NULL;
      mg->menu.option = NULL;
      mg->menu.numoptions = 0;
      
      model[mod]->numgroups++;
      
      return (new_group);
      
}



/* ENTER_SEGMENT_GROUP: */

int enter_segment_group (ModelStruct* ms, const char* username, int seg_num)
{
   int i, element, new_group;
   SegmentGroup* sg;
   ReturnCode rc;
   
   if (seg_num < 0)
      return -1;
   
   for (i = 0; i < ms->numseggroups; i++)
   {
      if (STRINGS_ARE_EQUAL(username, ms->seggroup[i].name))
      {
         sg = &ms->seggroup[i];
         
         if (sg->num_segments == sg->seg_array_size)
         {
            sg->seg_array_size += MUSCINDEX_ARRAY_INCREMENT;
            
            sg->segment = (int*) simm_realloc(sg->segment,
               sg->seg_array_size * sizeof(int), &rc);
            if (rc == code_bad)
            {
               sg->seg_array_size -= MUSCINDEX_ARRAY_INCREMENT;
               return -1;
            }
         }
         element = sg->num_segments++;
         
         sg->segment[element] = seg_num;
         
         return i;
      }
   }
   
   /* If you get to here, then the group name has not yet been recorded, meaning
   * that this is a new group. The variable "i" now equals the number of groups
   * already recorded, so it becomes the index of the group array where you
   * want to record this new group. First make sure the group array is
   * big enough to hold the new group.
   */
   
   new_group = i;
   
   if (ms->numseggroups == ms->seggroup_array_size)
   {
      ms->seggroup_array_size += MUSCGROUP_ARRAY_INCREMENT;
      
      if (ms->seggroup == NULL)
      {
         ms->seggroup = (SegmentGroup*) simm_malloc(ms->seggroup_array_size * sizeof(SegmentGroup));
         
         rc = (ms->seggroup ? code_fine : code_bad);
      }
      else
         ms->seggroup = (SegmentGroup*) simm_realloc(ms->seggroup,
         ms->seggroup_array_size * sizeof(SegmentGroup), &rc);
      if (rc == code_bad)
      {
         ms->seggroup_array_size -= MUSCGROUP_ARRAY_INCREMENT;
         return -1;
      }
   }
   
   sg = &ms->seggroup[new_group];
   
   mstrcpy(&sg->name, username);
   
   sg->seg_array_size = MUSCINDEX_ARRAY_INCREMENT;
   
   sg->segment = (int*) simm_malloc(sg->seg_array_size * sizeof(int));
   
   if (sg->segment == NULL)
      return -1;
   
   sg->segment[0] = seg_num;
   sg->num_segments = 1;
   
   ms->numseggroups++;
   
   return new_group;
}




/* ENTER_GENCOORD_GROUP: */

int enter_gencoord_group (ModelStruct* ms, const char* username, int gc_num)
{
   int i, element, new_group;
   GencoordGroup* gg;
   ReturnCode rc;
   
   if (gc_num < 0)
      return -1;
   
   for (i = 0; i < ms->numgencgroups; i++)
   {
      if (STRINGS_ARE_EQUAL(username, ms->gencgroup[i].name))
      {
         gg = &ms->gencgroup[i];
         
         if (gg->num_gencoords == gg->genc_array_size)
         {
            gg->genc_array_size += MUSCINDEX_ARRAY_INCREMENT;
            
            gg->gencoord = (int*) simm_realloc(gg->gencoord,
               gg->genc_array_size * sizeof(int), &rc);
            if (rc == code_bad)
            {
               gg->genc_array_size -= MUSCINDEX_ARRAY_INCREMENT;
               return -1;
            }
         }
         element = gg->num_gencoords++;
         
         gg->gencoord[element] = gc_num;
         
         return i;
      }
   }
   
   /* If you get to here, then the group name has not yet been recorded, meaning
   * that this is a new group. The variable "i" now equals the number of groups
   * already recorded, so it becomes the index of the group array where you
   * want to record this new group. First make sure the group array is
   * big enough to hold the new group.
   */
   
   new_group = i;
   
   if (ms->numgencgroups == ms->gencgroup_array_size)
   {
      ms->gencgroup_array_size += MUSCGROUP_ARRAY_INCREMENT;
      
      if (ms->gencgroup == NULL)
      {
         ms->gencgroup = (GencoordGroup*) simm_malloc(ms->gencgroup_array_size * sizeof(GencoordGroup));
         
         rc = (ms->gencgroup ? code_fine : code_bad);
      }
      else
         ms->gencgroup = (GencoordGroup*) simm_realloc(ms->gencgroup,
         ms->gencgroup_array_size * sizeof(GencoordGroup), &rc);
      if (rc == code_bad)
      {
         ms->gencgroup_array_size -= MUSCGROUP_ARRAY_INCREMENT;
         return -1;
      }
   }
   
   gg = &ms->gencgroup[new_group];
   
   mstrcpy(&gg->name, username);
   
   gg->genc_array_size = MUSCINDEX_ARRAY_INCREMENT;
   
   gg->gencoord = (int*) simm_malloc(gg->genc_array_size * sizeof(int));
   
   if (gg->gencoord == NULL)
      return -1;
   
   gg->gencoord[0] = gc_num;
   gg->num_gencoords = 1;
   
   ms->numgencgroups++;
   
   return new_group;
}


/* ENTER_JOINT: whenever a joint name is read from a file, the user-specified
 * name is compared to the existing array of joints to see if that one has
 * already been defined. If it has, the routine just returns the internal
 * number of that joint. If it has not yet been defined, it adds the name
 * to a new element in the jointlist and increments model.numjoints, unless
 * there is no permission to add the new name.
 */

int enter_joint(int mod, char username[], SBoolean permission_to_add)
{

   int i, new_jnt;
   ReturnCode rc;

   if (username == NULL)
      return -1;

   for (i = 0; i < model[mod]->numjoints; i++)
      if (STRINGS_ARE_EQUAL(username, model[mod]->joint[i].name))
         return i;

   if (permission_to_add == no)
      return -1;

   new_jnt = model[mod]->numjoints;

   if (model[mod]->numjoints == model[mod]->joint_array_size)
   {
      model[mod]->joint_array_size += JOINT_ARRAY_INCREMENT;
      model[mod]->joint = (JointStruct*)simm_realloc(model[mod]->joint,
         model[mod]->joint_array_size*sizeof(JointStruct),&rc);
      if (rc == code_bad)
      {
         model[mod]->joint_array_size -= JOINT_ARRAY_INCREMENT;
         return -1;
      }
   }

   init_joint(model[mod], &model[mod]->joint[new_jnt]);
   mstrcpy(&model[mod]->joint[new_jnt].name, username);

   model[mod]->numjoints++;

   return new_jnt;
}

