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
GeneralizedCoord* enter_gencoord(ModelStruct* model, const char username[], SBoolean permission_to_add)
{
   int i, new_gc;
   ReturnCode rc = code_fine;
   
   if (username == NULL)
      return NULL;
   
   for (i = 0; i < model->numgencoords; i++)
      if (STRINGS_ARE_EQUAL(username, model->gencoord[i]->name))
         return model->gencoord[i];

   if (permission_to_add == no)
      return NULL;

   new_gc = model->numgencoords;

   if (model->numgencoords == model->genc_array_size)
   {
      model->genc_array_size += GENC_ARRAY_INCREMENT;
      model->gencoord = (GeneralizedCoord**)simm_realloc(model->gencoord, model->genc_array_size*sizeof(GeneralizedCoord*), &rc);
      if (rc == code_bad)
      {
         model->genc_array_size -= GENC_ARRAY_INCREMENT;
         return NULL;
      }
   }

   model->gencoord[new_gc] = (GeneralizedCoord*)simm_malloc(sizeof(GeneralizedCoord));
   init_gencoord(model->gencoord[new_gc]);
   mstrcpy(&model->gencoord[new_gc]->name, username);

   model->numgencoords++;

   return model->gencoord[new_gc];
}


dpWrapObject* get_wrap_object(ModelStruct* model, char username[])
{
   int i;

   if (username == NULL)
      return NULL;

   for (i=0; i<model->num_wrap_objects; i++)
      if (STRINGS_ARE_EQUAL(username, model->wrapobj[i]->name))
         return model->wrapobj[i];

   return NULL; 
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
int enter_segment(ModelStruct* model, const char username[], SBoolean permission_to_add)
{
   int i, new_seg;
   ReturnCode rc = code_fine;

   if (username == NULL)
      return -1;

   for (i = 0; i < model->numsegments; i++)
      if (STRINGS_ARE_EQUAL(username, model->segment[i].name))
         return i;

   if (permission_to_add == no)
      return -1;

   new_seg = model->numsegments;

   if (model->numsegments == model->segment_array_size)
   {
      model->segment_array_size += SEGMENT_ARRAY_INCREMENT;
      model->segment = (SegmentStruct*)simm_realloc(model->segment, model->segment_array_size*sizeof(SegmentStruct), &rc);
      if (rc == code_bad)
      {
         model->segment_array_size -= SEGMENT_ARRAY_INCREMENT;
         return -1;
      }
   }

   init_segment(model, &model->segment[new_seg]);
   mstrcpy(&model->segment[new_seg].name, username);

   model->numsegments++;

   return new_seg;
}


int enter_muscle_group(ModelStruct* model, const char username[], int muscleIndex)
{
   int i, j, new_group;
   MuscleGroup* mg;
   ReturnCode rc = code_fine;

   if (model == NULL || muscleIndex < 0)
      return -1;

   for (i=0; i<model->numgroups; i++)
   {
      if (STRINGS_ARE_EQUAL(username, model->muscgroup[i].name))
      {
         mg = &model->muscgroup[i];

         // If the muscle is already in the group, just return the index of the group.
         for (j=0; j<mg->num_muscles; j++)
         {
            if (muscleIndex == mg->muscle_index[j])
               return i;
         }

         // Make room for more groups.
         if (mg->num_muscles == mg->muscindex_array_size)
         {
            mg->muscindex_array_size += MUSCINDEX_ARRAY_INCREMENT;
            mg->muscle_index = (int*)simm_realloc(mg->muscle_index, mg->muscindex_array_size*sizeof(int), &rc);
            if (rc == code_bad)
            {
               mg->muscindex_array_size -= MUSCINDEX_ARRAY_INCREMENT;
               return -1;
            }
         }

         // Add the muscle to the group.
         mg->muscle_index[mg->num_muscles++] = muscleIndex;
         return i;
      }
   }

   // If you get to here, then the group name has not yet been recorded, meaning
   // that this is a new group. The variable "i" now equals the number of groups
   // already recorded, so it becomes the index of the group array where you
   // want to record this new group. First make sure the muscle group array is
   // big enough to hold the new group.

   new_group = i;

   if (model->numgroups == model->muscgroup_array_size)
   {
      model->muscgroup_array_size += MUSCGROUP_ARRAY_INCREMENT;
      model->muscgroup = (MuscleGroup*)simm_realloc(model->muscgroup, model->muscgroup_array_size*sizeof(MuscleGroup), &rc);
      if (rc == code_bad)
      {
         model->muscgroup_array_size -= MUSCGROUP_ARRAY_INCREMENT;
         return -1;
      }
   }

   mg = &model->muscgroup[new_group];

   mstrcpy(&mg->name,username);

   mg->muscindex_array_size = MUSCINDEX_ARRAY_INCREMENT;
   mg->muscle_index = (int*)simm_malloc(mg->muscindex_array_size*sizeof(int));
   if (mg->muscle_index == NULL)
      return -1;

   mg->muscle_index[0] = muscleIndex;
   mg->num_muscles = 1;
   mg->menu.title = NULL;
   mg->menu.option = NULL;
   mg->menu.numoptions = 0;

   model->numgroups++;

   return new_group;
}


int enter_segment_group(ModelStruct* model, const char username[], int segmentIndex)
{
   int i, j, new_group;
   SegmentGroup* sg;
   ReturnCode rc = code_fine;

   if (segmentIndex < 0)
      return -1;

   for (i = 0; i < model->numseggroups; i++)
   {
      if (STRINGS_ARE_EQUAL(username, model->seggroup[i].name))
      {
         sg = &model->seggroup[i];

         // If the segment is already in the group, just return the index of the group.
         for (j=0; j<sg->num_segments; j++)
         {
            if (segmentIndex == sg->segment[j])
               return i;
         }

         // Make room for more groups.
         if (sg->num_segments == sg->seg_array_size)
         {
            sg->seg_array_size += MUSCINDEX_ARRAY_INCREMENT;

            sg->segment = (int*) simm_realloc(sg->segment, sg->seg_array_size * sizeof(int), &rc);
            if (rc == code_bad)
            {
               sg->seg_array_size -= MUSCINDEX_ARRAY_INCREMENT;
               return -1;
            }
         }

         // Add the segment to the group.
         sg->segment[sg->num_segments++] = segmentIndex;

         return i;
      }
   }

   // If you get to here, then the group name has not yet been recorded, meaning
   // that this is a new group. The variable "i" now equals the number of groups
   // already recorded, so it becomes the index of the group array where you
   // want to record this new group. First make sure the group array is
   // big enough to hold the new group.

   new_group = i;

   if (model->numseggroups == model->seggroup_array_size)
   {
      model->seggroup_array_size += MUSCGROUP_ARRAY_INCREMENT;

      if (model->seggroup == NULL)
      {
         model->seggroup = (SegmentGroup*) simm_malloc(model->seggroup_array_size * sizeof(SegmentGroup));
         rc = (model->seggroup ? code_fine : code_bad);
      }
      else
         model->seggroup = (SegmentGroup*) simm_realloc(model->seggroup, model->seggroup_array_size * sizeof(SegmentGroup), &rc);
      if (rc == code_bad)
      {
         model->seggroup_array_size -= MUSCGROUP_ARRAY_INCREMENT;
         return -1;
      }
   }

   sg = &model->seggroup[new_group];

   mstrcpy(&sg->name, username);

   sg->seg_array_size = MUSCINDEX_ARRAY_INCREMENT;

   sg->segment = (int*)simm_malloc(sg->seg_array_size * sizeof(int));

   if (sg->segment == NULL)
      return -1;

   sg->segment[0] = segmentIndex;
   sg->num_segments = 1;

   model->numseggroups++;

   return new_group;
}


int enter_gencoord_group(ModelStruct* model, const char username[], GeneralizedCoord* gencoord)
{
   int i, j, new_group;
   GencoordGroup* gg;
   ReturnCode rc = code_fine;

   if (gencoord == NULL)
      return -1;

   for (i = 0; i < model->numgencgroups; i++)
   {
      if (STRINGS_ARE_EQUAL(username, model->gencgroup[i].name))
      {
         gg = &model->gencgroup[i];

         // If the gencoord is already in the group, just return the index of the group.
         for (j=0; j<gg->num_gencoords; j++)
         {
            if (gencoord == gg->gencoord[j])
               return i;
         }

         // Make room for more groups.
         if (gg->num_gencoords == gg->genc_array_size)
         {
            gg->genc_array_size += MUSCINDEX_ARRAY_INCREMENT;

            gg->gencoord = (GeneralizedCoord**)simm_realloc(gg->gencoord, gg->genc_array_size*sizeof(GeneralizedCoord*), &rc);
            if (rc == code_bad)
            {
               gg->genc_array_size -= MUSCINDEX_ARRAY_INCREMENT;
               return -1;
            }
         }

         // Add the gencoord to the group.
         gg->gencoord[gg->num_gencoords++] = gencoord;

         return i;
      }
   }

   // If you get to here, then the group name has not yet been recorded, meaning
   // that this is a new group. The variable "i" now equals the number of groups
   // already recorded, so it becomes the index of the group array where you
   // want to record this new group. First make sure the group array is
   // big enough to hold the new group.

   new_group = i;

   if (model->numgencgroups == model->gencgroup_array_size)
   {
      model->gencgroup_array_size += MUSCGROUP_ARRAY_INCREMENT;

      if (model->gencgroup == NULL)
      {
         model->gencgroup = (GencoordGroup*)simm_malloc(model->gencgroup_array_size*sizeof(GencoordGroup));
         rc = (model->gencgroup ? code_fine : code_bad);
      }
      else
         model->gencgroup = (GencoordGroup*)simm_realloc(model->gencgroup, model->gencgroup_array_size*sizeof(GencoordGroup), &rc);
      if (rc == code_bad)
      {
         model->gencgroup_array_size -= MUSCGROUP_ARRAY_INCREMENT;
         return -1;
      }
   }

   gg = &model->gencgroup[new_group];

   mstrcpy(&gg->name, username);

   gg->genc_array_size = MUSCINDEX_ARRAY_INCREMENT;

   gg->gencoord = (GeneralizedCoord**)simm_malloc(gg->genc_array_size * sizeof(GeneralizedCoord*));

   if (gg->gencoord == NULL)
      return -1;

   gg->gencoord[0] = gencoord;
   gg->num_gencoords = 1;

   model->numgencgroups++;

   return new_group;
}


/* ENTER_JOINT: whenever a joint name is read from a file, the user-specified
 * name is compared to the existing array of joints to see if that one has
 * already been defined. If it has, the routine just returns the internal
 * number of that joint. If it has not yet been defined, it adds the name
 * to a new element in the jointlist and increments model.numjoints, unless
 * there is no permission to add the new name.
 */
int enter_joint(ModelStruct* model, const char username[], SBoolean permission_to_add)
{
   int i, new_jnt;
   ReturnCode rc = code_fine;

   if (username == NULL)
      return -1;

   for (i = 0; i < model->numjoints; i++)
      if (STRINGS_ARE_EQUAL(username, model->joint[i].name))
         return i;

   if (permission_to_add == no)
      return -1;

   new_jnt = model->numjoints;

   if (model->numjoints == model->joint_array_size)
   {
      model->joint_array_size += JOINT_ARRAY_INCREMENT;
      model->joint = (JointStruct*)simm_realloc(model->joint, model->joint_array_size*sizeof(JointStruct),&rc);
      if (rc == code_bad)
      {
         model->joint_array_size -= JOINT_ARRAY_INCREMENT;
         return -1;
      }
   }

   init_joint(model, &model->joint[new_jnt]);
   mstrcpy(&model->joint[new_jnt].name, username);

   model->numjoints++;

   return new_jnt;
}

#if ! OPENSIM_BUILD

int enter_preference(const char name[], const char value[])
{
   int i;

   // If a preference with the same name is already in the list,
   // replace its value with the new one.
   for (i=0; i<root.num_preferences; i++)
   {
      if (STRINGS_ARE_EQUAL(name, root.preference[i].name))
      {
         FREE_IFNOTNULL(root.preference[i].value);
         mstrcpy(&root.preference[i].value, value);
         return i;
      }
   }

   // Make room for more preferences.
   if (root.num_preferences == root.preference_array_size)
   {
      ReturnCode rc;

      root.preference_array_size += 10;
      if (root.preference == NULL)
      {
         root.preference = (SimmPreference*)simm_calloc(root.preference_array_size, sizeof(SimmPreference));
         rc = (root.preference) ? code_fine : code_bad;
      }
      else
      {
         root.preference = (SimmPreference*)simm_realloc(root.preference, root.preference_array_size*sizeof(SimmPreference), &rc);
      }
      if (rc == code_bad)
      {
         root.preference_array_size -= 10;
         return -1;
      }
   }

   // The preference is new; add it to the end of the list.
   mstrcpy(&root.preference[root.num_preferences].name, name);
   mstrcpy(&root.preference[root.num_preferences].value, value);

   return root.num_preferences++;
}

#endif
