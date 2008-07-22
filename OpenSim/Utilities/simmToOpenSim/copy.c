/*******************************************************************************

   COPY.C

   Author: Peter Loan

   Date: 17-MAY-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains routines that copy values
      in some structure or array to another structure or array.

   Routines:
      copy_4x4matrix : copies a 4x4 matrix
      copy_1x4vector : copies a 1x4 array (vector)
      copy_dof       : copies a REFEQ structure which holds a dof
      copy_function  : copies a function's points and spline coefficients
      copy_mps       : copies a MusclePoint array
      copy_nndouble  : mallocs and copies a double if 'from' is not null
      copy_nddouble  : mallocs and copies a double if 'from' is not default
      copy_nnint     : mallocs and copies a int if 'from' is not null
      copy_ndint     : mallocs and copies a int if 'from' is not default

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "defunctions.h"


/*************** DEFINES (for this file only) *********************************/



/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/



/* COPY_4X4MATRIX: copies a 4x4 matrix */

void copy_4x4matrix(double from[][4], double to[][4])
{

   to[0][0] = from[0][0];
   to[0][1] = from[0][1];
   to[0][2] = from[0][2];
   to[0][3] = from[0][3];
   to[1][0] = from[1][0];
   to[1][1] = from[1][1];
   to[1][2] = from[1][2];
   to[1][3] = from[1][3];
   to[2][0] = from[2][0];
   to[2][1] = from[2][1];
   to[2][2] = from[2][2];
   to[2][3] = from[2][3];
   to[3][0] = from[3][0];
   to[3][1] = from[3][1];
   to[3][2] = from[3][2];
   to[3][3] = from[3][3];

}



/* COPY_1x4VECTOR: copies a 1x4 vector */

void copy_1x4vector(double from[], double to[])
{

   to[0] = from[0];
   to[1] = from[1];
   to[2] = from[2];
   to[3] = from[3];

}


void copy_point(Coord3D *from, Coord3D *to)
{
   to->xyz[0] = from->xyz[0];
   to->xyz[1] = from->xyz[1];
   to->xyz[2] = from->xyz[2];
}

/* COPY_DOF: copies a dof. Used to save and restore dofs in the joint
 * editor and to copy dofs from the model-file-reading buffer to a
 * model structure.
 */

void copy_dof(DofStruct* from, DofStruct* to)
{

   to->type = from->type;
   to->value = from->value;
   to->funcnum = from->funcnum;
   to->gencoord = from->gencoord;

}


void copy_gencoord_info(GeneralizedCoord *from, SaveGencoords *to)
{
   to->value = from->value;
   to->velocity = from->velocity;
   to->tolerance = from->tolerance;
   to->pd_stiffness = from->pd_stiffness;
   to->default_value = from->default_value;
   to->clamped = from->clamped;
   to->locked = from->locked;
   to->restraintFuncActive = from->restraintFuncActive;
   to->range = from->range;
   to->restraint_func_num = from->restraint_func_num;
   to->restraint_user_num = from->restraint_user_num;
   to->used_in_model = from->used_in_model;
}

/* COPY_Function: copies a function structure. This structure contains
 * the original (x,y) data points and the coefficients (b,c,d) of
 * the natural cubic spline interpolants. It assumes that space
 * for x,y,a,b,c has already been allocated.
 */

void copy_function(SplineFunction* from, SplineFunction* to)
{
   int i;

   to->type = from->type;
   to->numpoints = from->numpoints;
   to->usernum = from->usernum;
   to->defined = from->defined;
   to->used = yes;

   for (i=0; i<to->numpoints; i++)
   {
      to->x[i] = from->x[i];
      to->y[i] = from->y[i];
      to->b[i] = from->b[i];
      to->c[i] = from->c[i];
      to->d[i] = from->d[i];
   }
}



/* COPY_MPS: */

ReturnCode copy_mps(MusclePoint* from, MusclePoint* to)
{
   int i;

   to->segment = from->segment;
   to->refpt = from->refpt;
   to->selected = from->selected;
   to->point[XX] = from->point[XX];
   to->point[YY] = from->point[YY];
   to->point[ZZ] = from->point[ZZ];
   to->ground_pt[XX] = from->ground_pt[XX];
   to->ground_pt[YY] = from->ground_pt[YY];
   to->ground_pt[ZZ] = from->ground_pt[ZZ];
   to->old_seg = from->old_seg;
   to->old_point[XX] = from->old_point[XX];
   to->old_point[YY] = from->old_point[YY];
   to->old_point[ZZ] = from->old_point[ZZ];
   to->funcnum[XX] = from->funcnum[XX];
   to->funcnum[YY] = from->funcnum[YY];
   to->funcnum[ZZ] = from->funcnum[ZZ];
   to->gencoord[XX] = from->gencoord[XX];
   to->gencoord[YY] = from->gencoord[YY];
   to->gencoord[ZZ] = from->gencoord[ZZ];
   to->movingpoint = from->movingpoint;


   to->numranges = from->numranges;
   if (to->numranges > 0)
   {
      to->ranges = (PointRange*)simm_malloc(to->numranges*sizeof(PointRange));
      if (to->ranges == NULL)
         return code_bad;

      for (i=0; i<to->numranges; i++)
      {
         to->ranges[i].genc = from->ranges[i].genc;
         to->ranges[i].start = from->ranges[i].start;
         to->ranges[i].end = from->ranges[i].end;
      }
   }
   else
      to->ranges = NULL;

   to->is_auto_wrap_point = from->is_auto_wrap_point;
   to->wrap_distance = from->wrap_distance;
   to->num_wrap_pts = from->num_wrap_pts;
   if (to->num_wrap_pts > 0)
   {
      to->wrap_pts = (double*)simm_malloc(to->num_wrap_pts*3*sizeof(double));
      if (to->wrap_pts == NULL)
         return code_bad;
      for (i=0; i<to->num_wrap_pts*3; i++)
         to->wrap_pts[i] = from->wrap_pts[i];
   }
   else
      to->wrap_pts = NULL;

   to->undeformed_point[XX] = from->undeformed_point[XX];
   to->undeformed_point[YY] = from->undeformed_point[YY];
   to->undeformed_point[ZZ] = from->undeformed_point[ZZ];

   return code_fine;

}



/* COPY_NNDOUBLE: */

ReturnCode copy_nndouble(double* from, double** to)
{
   if (from == NULL)
      *to = NULL;
   else
   {
      if ((*to = (double*)simm_malloc(sizeof(double))) == NULL)
         return (code_bad);
      **to = *from;
   }

   return code_fine;
}



/* COPY_NDDOUBLE: */

ReturnCode copy_nddouble(double* from, double** to, double* deffrom, double* defto)
{
   if (from == deffrom)
      *to = defto;
   else
   {
      if (from == NULL)
         *to = NULL;
      else
      {
         if ((*to = (double*)simm_malloc(sizeof(double))) == NULL)
            return (code_bad);
         **to = *from;
      }
   }

   return code_fine;
}



/* COPY_NNINT: */

ReturnCode copy_nnint(int* from, int** to)
{
   if (from == NULL)
      *to = NULL;
   else
   {
      if ((*to = (int*)simm_malloc(sizeof(int))) == NULL)
         return (code_bad);
      **to = *from;
   }

   return code_fine;
}



/* COPY_NDINT: */

ReturnCode copy_ndint(int* from, int** to, int* deffrom, int* defto)
{
   if (from == deffrom)
      *to = defto;
   else
   {
      if (from == NULL)
         *to = NULL;
      else
      {
         if ((*to = (int*)simm_malloc(sizeof(int))) == NULL)
            return (code_bad);
         **to = *from;
      }
   }

   return code_fine;
}

WorldObject* copy_world_objects(WorldObject from[], int num)
{
   int i;
   WorldObject* to = NULL;

   if (num <= 0)
      return to;

   to = (WorldObject*)simm_malloc(sizeof(WorldObject)*num);
   memcpy(to, from, sizeof(WorldObject)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      mstrcpy(&to[i].filename, from[i].filename);
      if (from[i].wobj != NULL)
      {
         to[i].wobj = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
         copy_polyhedron(from[i].wobj, to[i].wobj);
      }
   }

   return to;
}

JointStruct* copy_joints(JointStruct from[], int num, ModelStruct* ms)
{
   int i, j;
   JointStruct* to = NULL;

   if (num <= 0)
      return to;

   to = (JointStruct*)simm_malloc(sizeof(JointStruct)*num);
   memcpy(to, from, sizeof(JointStruct)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      mstrcpy(&to[i].sd_name, from[i].sd_name);
      mstrcpy(&to[i].solverType, from[i].solverType);
      for (j=0; j<6; j++)
      {
         mstrcpy(&to[i].dofs[j].sd.name, from[i].dofs[j].sd.name);
         mstrcpy(&to[i].dofs[j].sd.con_name, from[i].dofs[j].sd.con_name);
      }
      if (ms->numsegments > 0)
      {
         to[i].in_seg_ground_path = (SBoolean*)simm_malloc(ms->numsegments*sizeof(SBoolean));
         memcpy(to[i].in_seg_ground_path, from[i].in_seg_ground_path, ms->numsegments*sizeof(SBoolean));
      }
#if INCLUDE_MOCAP_MODULE
      mstrcpy(&to[i].mocap_segment, from[i].mocap_segment);
#endif
   }

   return to;
}

SegmentStruct* copy_segments(SegmentStruct from[], int num)
{
   int i, j;
   SegmentStruct* to = NULL;

   if (num <= 0)
      return to;

   to = (SegmentStruct*)simm_malloc(sizeof(SegmentStruct)*num);
   memcpy(to, from, sizeof(SegmentStruct)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      if (from[i].numBones > 0)
      {
         to[i].bone = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct)*to[i].numBones);
         for (j=0; j<to[i].numBones; j++)
            copy_polyhedron(&from[i].bone[j], &to[i].bone[j]);
      }
      to[i].boneArraySize = from[i].numBones;
      if (from[i].numgroups > 0)
      {
         to[i].group = (int*)simm_malloc(sizeof(int)*to[i].numgroups);
         memcpy(to[i].group, from[i].group, sizeof(int)*to[i].numgroups);
      }
      if (from[i].numSpringPoints > 0)
      {
         to[i].springPoint = (SpringPoint*)simm_malloc(sizeof(SpringPoint)*to[i].numSpringPoints);
         memcpy(to[i].springPoint, from[i].springPoint, sizeof(SpringPoint)*to[i].numSpringPoints);
         for (j=0; j<from[i].numSpringPoints; j++)
            mstrcpy(&to[i].springPoint[j].name, from[i].springPoint[j].name);
      }
      to[i].springPointArraySize = from[i].numSpringPoints;
      if (from[i].springFloor != NULL)
      {
         to[i].springFloor = (SpringFloor*)simm_malloc(sizeof(SpringFloor));
         memcpy(to[i].springFloor, from[i].springFloor, sizeof(SpringFloor));
         mstrcpy(&to[i].springFloor->name, from[i].springFloor->name);
         mstrcpy(&to[i].springFloor->filename, from[i].springFloor->filename);
         if (from[i].springFloor->poly != NULL)
         {
            to[i].springFloor->poly = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
            copy_polyhedron(from[i].springFloor->poly, to[i].springFloor->poly);
         }
         to[i].springFloor->numPoints = 0;
         to[i].springFloor->points = NULL;
      }
      if (from[i].numContactObjects > 0)
      {
         to[i].contactObject = (ContactObject*)simm_malloc(sizeof(ContactObject)*to[i].numContactObjects);
         memcpy(to[i].contactObject, from[i].contactObject, sizeof(ContactObject)*to[i].numContactObjects);
         for (j=0; j<from[i].numContactObjects; j++)
         {
            mstrcpy(&to[i].contactObject[j].name, from[i].contactObject[j].name);
            mstrcpy(&to[i].contactObject[j].filename, from[i].contactObject[j].filename);
            if (from[i].contactObject[j].poly != NULL)
            {
               to[i].contactObject[j].poly = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
               copy_polyhedron(from[i].contactObject[j].poly, to[i].contactObject[j].poly);
            }
         }
      }
      to[i].contactObjectArraySize = from[i].numContactObjects;
      if (from[i].forceMatte != NULL)
      {
         to[i].forceMatte = (ContactObject*)simm_malloc(sizeof(ContactObject));
         memcpy(to[i].forceMatte, from[i].forceMatte, sizeof(ContactObject));
         mstrcpy(&to[i].forceMatte->name, from[i].forceMatte->name);
         mstrcpy(&to[i].forceMatte->filename, from[i].forceMatte->filename);
         if (from[i].forceMatte->poly != NULL)
         {
            to[i].forceMatte->poly = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
            copy_polyhedron(from[i].forceMatte->poly, to[i].forceMatte->poly);
         }
      }
      if (from[i].numMarkers > 0)
      {
         to[i].marker = (Marker*)simm_malloc(sizeof(Marker)*from[i].numMarkers);
         memcpy(to[i].marker, from[i].marker, sizeof(Marker)*from[i].numMarkers);
         for (j=0; j<from[i].numMarkers; j++)
            mstrcpy(&to[i].marker[j].name, from[i].marker[j].name);
      }
      to[i].markerArraySize = from[i].numMarkers;
#if INCLUDE_BONE_EDITOR_EXTRAS
      to[i].pts_file = NULL;
      to[i].raw_vertices = NULL;
#endif
      if (from[i].num_deforms > 0)
      {
         to[i].deform = (DeformObject*)simm_malloc(sizeof(DeformObject)*from[i].num_deforms);
         memcpy(to[i].deform, from[i].deform, sizeof(DeformObject)*from[i].num_deforms);
         for (j=0; j<from[i].num_deforms; j++)
         {
            mstrcpy(&to[i].deform[j].name, from[i].deform[j].name);
            to[i].deform[j].innerBox = (float*)simm_malloc(sizeof(float)*144);
            memcpy(to[i].deform[j].innerBox, from[i].deform[j].innerBox, sizeof(float)*144);
            to[i].deform[j].innerBoxUndeformed = (float*)simm_malloc(sizeof(float)*144);
            memcpy(to[i].deform[j].innerBoxUndeformed, from[i].deform[j].innerBoxUndeformed, sizeof(float)*144);
            to[i].deform[j].outerBox = (float*)simm_malloc(sizeof(float)*144);
            memcpy(to[i].deform[j].outerBox, from[i].deform[j].outerBox, sizeof(float)*144);
            to[i].deform[j].outerBoxUndeformed = (float*)simm_malloc(sizeof(float)*144);
            memcpy(to[i].deform[j].outerBoxUndeformed, from[i].deform[j].outerBoxUndeformed, sizeof(float)*144);
         }
      }
      to[i].deform_obj_array_size = from[i].num_deforms;
#if INCLUDE_MOCAP_MODULE
      mstrcpy(&to[i].gait_scale_segment, from[i].gait_scale_segment);
      mstrcpy(&to[i].mocap_segment, from[i].mocap_segment);
      mstrcpy(&to[i].mocap_scale_chain_end1, from[i].mocap_scale_chain_end1);
      mstrcpy(&to[i].mocap_scale_chain_end2, from[i].mocap_scale_chain_end2);
#endif
   }

   return to;
}

void copy_menu(Menu* to, Menu* from)
{
   int i;

   memcpy(to, from, sizeof(Menu));
   mstrcpy(&to->title, from->title);
   if (from->numoptions > 0)
   {
      to->option = (MenuItem*)simm_malloc(sizeof(MenuItem)*from->numoptions);
      memcpy(to->option, from->option, sizeof(MenuItem)*from->numoptions);
      for (i=0; i<from->numoptions; i++)
         mstrcpy(&to->option[i].name, from->option[i].name);
   }
}

MuscleGroup* copy_muscle_groups(MuscleGroup from[], int num)
{
   int i;
   MuscleGroup* to = NULL;

   if (num <= 0)
      return to;

   to = (MuscleGroup*)simm_malloc(sizeof(MuscleGroup)*num);
   memcpy(to, from, sizeof(MuscleGroup)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      if (from[i].number_of_muscles > 0)
      {
         to[i].muscle_index = (int*)simm_malloc(sizeof(int)*from[i].number_of_muscles);
         memcpy(to[i].muscle_index, from[i].muscle_index, sizeof(int)*from[i].number_of_muscles);
      }
      to[i].muscindex_array_size = from[i].number_of_muscles;
      copy_menu(&to[i].menu, &from[i].menu);
   }

   return to;
}

SegmentGroup* copy_segment_groups(SegmentGroup from[], int num)
{
   int i;
   SegmentGroup* to = NULL;

   if (num <= 0)
      return to;

   to = (SegmentGroup*)simm_malloc(sizeof(SegmentGroup)*num);
   memcpy(to, from, sizeof(SegmentGroup)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      if (from[i].num_segments > 0)
      {
         to[i].segment = (int*)simm_malloc(sizeof(int)*from[i].num_segments);
         memcpy(to[i].segment, from[i].segment, sizeof(int)*from[i].num_segments);
      }
      to[i].seg_array_size = from[i].num_segments;
   }

   return to;
}

GencoordGroup* copy_gencoord_groups(GencoordGroup from[], int num)
{
   int i;
   GencoordGroup* to = NULL;

   if (num <= 0)
      return to;

   to = (GencoordGroup*)simm_malloc(sizeof(GencoordGroup)*num);
   memcpy(to, from, sizeof(GencoordGroup)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      if (from[i].num_gencoords > 0)
      {
         to[i].gencoord = (int*)simm_malloc(sizeof(int)*from[i].num_gencoords);
         memcpy(to[i].gencoord, from[i].gencoord, sizeof(int)*from[i].num_gencoords);
      }
      to[i].genc_array_size = from[i].num_gencoords;
   }

   return to;
}

GeneralizedCoord* copy_gencoords(GeneralizedCoord from[], int num)
{
   int i;
   GeneralizedCoord* to = NULL;

   if (num <= 0)
      return to;

   to = (GeneralizedCoord*)simm_malloc(sizeof(GeneralizedCoord)*num);
   memcpy(to, from, sizeof(GeneralizedCoord)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      if (from[i].numjoints > 0)
      {
         to[i].jointnum = (int*)simm_malloc(sizeof(int)*from[i].numjoints);
         memcpy(to[i].jointnum, from[i].jointnum, sizeof(int)*from[i].numjoints);
      }
      if (from[i].numgroups > 0)
      {
         to[i].group = (int*)simm_malloc(sizeof(int)*to[i].numgroups);
         memcpy(to[i].group, from[i].group, sizeof(int)*to[i].numgroups);
      }
#if INCLUDE_MOCAP_MODULE
      mstrcpy(&to[i].mocap_segment, from[i].mocap_segment);
#endif
   }

   return to;
}

SplineFunction* copy_functions(SplineFunction from[], int fromArraySize, int* toArraySize)
{
   int i, index = 0;
   SplineFunction* to = NULL;

   *toArraySize = 0;
   for (i = 0; i < fromArraySize; i++)
      if (from[i].used == yes)
         (*toArraySize)++;

   if (*toArraySize <= 0)
      return to;

   to = (SplineFunction*)simm_malloc(sizeof(SplineFunction)*(*toArraySize));

   for (i=0; i<fromArraySize; i++)
   {
      if (from[i].used == yes)
      {
         malloc_function(&to[index], from[i].numpoints);
         copy_function(&from[i], &to[index]);
         index++;
      }
   }

   return to;
}

void copy_muscles(ModelStruct* from, ModelStruct* to)
{
   int i;

   copy_default_muscle(&from->default_muscle, &to->default_muscle);

   if (from->nummuscles == 0)
   {
      to->muscle = NULL;
      return;
   }

   /* malloc memory for all the muscle structures */
   to->muscle = (MuscleStruct*)simm_malloc(from->nummuscles*sizeof(MuscleStruct));

   /* copy the muscle structures */
   for (i=0; i<from->nummuscles; i++)
      copy_musc(&from->muscle[i], &to->muscle[i], &from->default_muscle, &to->default_muscle);
}

LigamentPoint* copy_ligament_points(LigamentPoint from[], int num)
{
   int i;
   LigamentPoint* to = NULL;

   if (num <= 0)
      return to;

   to = (LigamentPoint*)simm_malloc(sizeof(LigamentPoint)*num);
   memcpy(to, from, sizeof(LigamentPoint)*num);

   for (i=0; i<num; i++)
   {
      to[i].ranges = (PointRange*)simm_malloc(sizeof(PointRange)*from[i].numranges);
      memcpy(to[i].ranges, from[i].ranges, sizeof(PointRange)*from[i].numranges);
   }

   return to;
}

LigamentLine* copy_ligament_lines(LigamentLine from[], int num)
{
   int i;
   LigamentLine* to = NULL;

   if (num <= 0)
      return to;

   to = (LigamentLine*)simm_malloc(sizeof(LigamentLine)*num);
   memcpy(to, from, sizeof(LigamentLine)*num);

   for (i=0; i<num; i++)
   {
      to[i].pt = copy_ligament_points(from[i].pt, from[i].numpoints);
      to[i].pt_array_size = from[i].numpoints;
   }

   return to;
}

LigamentStruct* copy_ligaments(LigamentStruct from[], int num)
{
   int i;
   LigamentStruct* to = NULL;

   if (num <= 0)
      return to;

   to = (LigamentStruct*)simm_malloc(sizeof(LigamentStruct)*num);
   memcpy(to, from, sizeof(LigamentStruct)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      to[i].line = copy_ligament_lines(from[i].line, from[i].numlines);
      to[i].line_array_size = from[i].numlines;
   }

   return to;
}

/* Copy only pointer-based sections of DisplayStruct.*/
void copy_display_struct(DisplayStruct* from, DisplayStruct* to)
{
   int i;

   if (from == NULL || to == NULL)
      return;

   for (i=0; i<MAXSAVEDVIEWS; i++)
   {
      if (from->view_name[i])
         mstrcpy(&to->view_name[i], from->view_name[i]);
      else
         to->view_name[i] = NULL;
   }
   to->applied_motion = NULL;
   to->current_motion = NULL;
   if (from->nummuscleson > 0 && from->muscleson != NULL)
   {
      to->muscleson = (int*)simm_malloc(sizeof(int)*from->nummuscleson);
      memcpy(to->muscleson, from->muscleson, sizeof(int)*from->nummuscleson);
   }
   to->devs = NULL;
   to->dev_values = NULL;
#if INCLUDE_SNAPSHOT
   to->snapshot_file_base_name = NULL;
   to->snapshot_file_suffix = NULL;
#endif

   if (from->mat.num_materials > 0 && from->mat.materials != NULL)
   {
      to->mat.materials = (MaterialStruct*)simm_calloc(from->mat.num_materials,sizeof(MaterialStruct));
      for (i=0; i<from->mat.num_materials; i++)
         copy_material(&from->mat.materials[i], &to->mat.materials[i]);
      to->mat.num_materials = from->mat.num_materials;
      to->mat.material_array_size = to->mat.num_materials;
   }
}

ContactPair* copy_contact_pairs(ContactPair from[], int num)
{
   int i;
   ContactPair* to = NULL;

   if (num <= 0)
      return to;

   to = (ContactPair*)simm_malloc(sizeof(ContactPair)*num);
   memcpy(to, from, sizeof(ContactPair)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].body1, from[i].body1);
      mstrcpy(&to[i].body2, from[i].body2);
   }

   return to;
}

ContactGroup* copy_contact_groups(ContactGroup from[], int num)
{
   int i, j;
   ContactGroup* to = NULL;

   if (num <= 0)
      return to;

   to = (ContactGroup*)simm_malloc(sizeof(ContactGroup)*num);
   memcpy(to, from, sizeof(ContactGroup)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      to[i].element = (char**)simm_malloc(sizeof(char*)*from[i].numElements);
      for (j=0; j<from[i].numElements; j++)
         mstrcpy(&to[i].element[j], to[i].element[j]);
   }

   return to;
}

MotionObject* copy_motion_objects(MotionObject from[], int num)
{
   int i;
   MotionObject* to = NULL;

   if (num <= 0)
      return to;

   to = (MotionObject*)simm_malloc(sizeof(MotionObject)*num);
   memcpy(to, from, sizeof(MotionObject)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      mstrcpy(&to[i].filename, from[i].filename);
      mstrcpy(&to[i].materialname, from[i].materialname);
      copy_polyhedron(&from[i].shape, &to[i].shape);
      to[i].shape.gl_display = -1;
   }

   return to;
}

WrapObject* copy_wrap_objects(WrapObject from[], int num)
{
   int i;
   WrapObject* to = NULL;

   if (num <= 0)
      return to;

   to = (WrapObject*)simm_malloc(sizeof(WrapObject)*num);
   memcpy(to, from, sizeof(WrapObject)*num);

   for (i=0; i<num; i++)
      mstrcpy(&to[i].name, from[i].name);

   return to;
}

Deformity* copy_deformities(Deformity from[], int num, ModelStruct* ms)
{
   int i, j;
   Deformity* to = NULL;

   if (num <= 0)
      return to;

   to = (Deformity*)simm_malloc(sizeof(Deformity)*num);
   memcpy(to, from, sizeof(Deformity)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      to[i].deform_name = (char**)simm_malloc(sizeof(char*)*from[i].num_deforms);
      to[i].deform = (DeformObject**)simm_malloc(sizeof(DeformObject*)*from[i].num_deforms);
      for (j=0; j<from[i].num_deforms; j++)
      {
         mstrcpy(&to[i].deform_name[j], to[i].deform_name[j]);
         to[i].deform[j] = lookup_deform(ms, from[i].deform_name[j]);
      }
   }

   return to;
}

ConstraintPoint* copy_constraint_points(ConstraintPoint from[], int num)
{
   int i;
   ConstraintPoint* to = NULL;

   if (num <= 0)
      return to;

   to = (ConstraintPoint*)simm_malloc(sizeof(ConstraintPoint)*num);
   memcpy(to, from, sizeof(ConstraintPoint)*num);

   for (i=0; i<num; i++)
      mstrcpy(&to[i].name, from[i].name);

   return to;
}

ConstraintObject* copy_constraint_objects(ConstraintObject from[], int num)
{
   int i;
   ConstraintObject* to = NULL;

   if (num <= 0)
      return to;

   to = (ConstraintObject*)simm_malloc(sizeof(ConstraintObject)*num);
   memcpy(to, from, sizeof(ConstraintObject)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      to[i].points = copy_constraint_points(from[i].points, from[i].numPoints);
      to[i].cp_array_size = from[i].numPoints;
      to[i].joints = (int*)simm_malloc(sizeof(int)*from[i].num_jnts);
      memcpy(to[i].joints, from[i].joints, sizeof(int)*from[i].num_jnts);
      to[i].qs = (int*)simm_malloc(sizeof(int)*from[i].num_qs);
      memcpy(to[i].qs, from[i].qs, sizeof(int)*from[i].num_qs);
   }

   return to;
}

MotionSequence** copy_motions(MotionSequence* from[], int num)
{
   int i;
   MotionSequence** to = NULL;

   if (num <= 0)
      return to;

   to = (MotionSequence**)simm_malloc(sizeof(MotionSequence*)*num);

   for (i=0; i<num; i++)
      to[i] = copy_motion(from[i]);

   return to;
}

/* COPY_MOTION: does not copy all of a motion. In particular,
 * the mopt is copied only with memcpy().
 */
MotionSequence* copy_motion(MotionSequence* from)
{
   int j;
   MotionSequence* to = NULL;

   if (from == NULL)
      return to;

   to = (MotionSequence*)simm_malloc(sizeof(MotionSequence));

   memcpy(to, from, sizeof(MotionSequence));
   mstrcpy(&to->name, from->name);
   if (from->deriv_names)
   {
      to->deriv_names = (char**)simm_malloc(sizeof(char*)*from->number_of_datacolumns);
      for (j=0; j<from->number_of_datacolumns; j++)
         mstrcpy(&to->deriv_names[j], from->deriv_names[j]);
   }
   if (from->deriv_data)
   {
      to->deriv_data = (double**)simm_malloc(sizeof(double*)*from->number_of_datacolumns);
      for (j=0; j<from->number_of_datacolumns; j++)
      {
         to->deriv_data[j] = (double*)simm_malloc(sizeof(double)*from->number_of_datarows);
         memcpy(to->deriv_data[j], from->deriv_data[j], sizeof(double)*from->number_of_datarows);
      }
   }
   mstrcpy(&to->units, from->units);
   if (from->columnname)
   {
      to->columnname = (char**)simm_malloc(sizeof(char*)*from->number_of_datacolumns);
      for (j=0; j<from->number_of_datacolumns; j++)
         mstrcpy(&to->columnname[j], from->columnname[j]);
   }
   if (from->motiondata)
   {
      to->motiondata = (double**)simm_malloc(sizeof(double*)*from->number_of_datacolumns);
      for (j=0; j<from->number_of_datacolumns; j++)
      {
         to->motiondata[j] = (double*)simm_malloc(sizeof(double)*from->number_of_datarows);
         memcpy(to->motiondata[j], from->motiondata[j], sizeof(double)*from->number_of_datarows);
      }
   }
   if (from->data_std_dev)
   {
      to->data_std_dev = (double**)simm_malloc(sizeof(double*)*from->number_of_datacolumns);
      for (j=0; j<from->number_of_datacolumns; j++)
      {
         if (from->data_std_dev[j])
         {
            to->data_std_dev[j] = (double*)simm_malloc(sizeof(double)*from->number_of_datarows);
            memcpy(to->data_std_dev[j], from->data_std_dev[j], sizeof(double)*from->number_of_datarows);
         }
         else
         {
            to->data_std_dev[j] = NULL;
         }
      }
   }
   if (from->num_events > 0)
   {
      to->event = (MotionEvent*)simm_malloc(sizeof(MotionEvent)*from->num_events);
      memcpy(to->event, from->event, sizeof(MotionEvent)*from->num_events);
      for (j=0; j<from->num_events; j++)
         mstrcpy(&to->event[j].name, from->event[j].name);
   }

   return to;
}

ModelStruct* copy_model(ModelStruct* ms)
{
   int i;

   ModelStruct* copy = (ModelStruct*)simm_malloc(sizeof(ModelStruct));
   memcpy(copy, ms, sizeof(ModelStruct));

   mstrcpy(&copy->name, ms->name);
   mstrcpy(&copy->forceUnits, ms->forceUnits);
   mstrcpy(&copy->lengthUnits, ms->lengthUnits);
   mstrcpy(&copy->HTRtranslationUnits, ms->HTRtranslationUnits);

   // copy pathptrs
   copy->pathptrs = NULL;
   // copy segment_drawing_order
   copy->segment_drawing_order = NULL;
   // copy musclegroupsubmenu
   copy->musclegroupsubmenu = NULL;
   // copy gencslider
   copy->gencslider.sl = NULL;
   copy->gencslider.numsliders = 0;
   // copy gencform
   copy->gencform.title = NULL;
   copy->gencform.option = NULL;
   copy->gencform.numoptions = 0;
   // copy gc_chpanel
   copy->gc_chpanel.title = NULL;
   copy->gc_chpanel.checkbox = NULL;
   copy->gc_chpanel.numoptions = 0;
   // copy gc_lockPanel
   copy->gc_lockPanel.title = NULL;
   copy->gc_lockPanel.checkbox = NULL;
   copy->gc_lockPanel.numoptions = 0;
   // copy dynparamsform
   copy->dynparamsform.title = NULL;
   copy->dynparamsform.option = NULL;
   copy->dynparamsform.numoptions = 0;

   mstrcpy(&copy->jointfilename, ms->jointfilename);
   mstrcpy(&copy->bonepathname, ms->bonepathname);
   mstrcpy(&copy->musclefilename, ms->musclefilename);
   for (i=0; i<copy->num_motion_files; i++)
      mstrcpy(&copy->motionfilename[i], ms->motionfilename[i]);
   mstrcpy(&copy->mocap_dir, ms->mocap_dir);

   copy->worldobj = copy_world_objects(ms->worldobj, ms->numworldobjects);
   copy->world_array_size = ms->numworldobjects;

   copy->joint = copy_joints(ms->joint, ms->numjoints, ms);
   copy->joint_array_size = ms->numjoints;

   copy->segment = copy_segments(ms->segment, ms->numsegments);
   copy->segment_array_size = ms->numsegments;

   copy->muscgroup = copy_muscle_groups(ms->muscgroup, ms->numgroups);
   copy->muscgroup_array_size = ms->numgroups;

   copy->seggroup = copy_segment_groups(ms->seggroup, ms->numseggroups);
   copy->seggroup_array_size = ms->numseggroups;

   copy->gencgroup = copy_gencoord_groups(ms->gencgroup, ms->numgencgroups);
   copy->gencgroup_array_size = ms->numgencgroups;

   copy->gencoord = copy_gencoords(ms->gencoord, ms->numgencoords);
   copy->genc_array_size = ms->numgencoords;

   copy->function = copy_functions(ms->function, ms->func_array_size, &copy->func_array_size);

   copy_muscles(ms, copy);

   copy->ligament = copy_ligaments(ms->ligament, ms->numligaments);
   copy->ligament_array_size = ms->numligaments;

   copy_display_struct(&ms->dis, &copy->dis);

   // copy save
   nullify_muscle(&copy->save.default_muscle);
   copy->save.muscle = NULL;
   copy->save.segment = NULL;
   copy->save.joint = NULL;
   copy->save.gencoord = NULL;
   copy->save.function = NULL;
   copy->save.muscgroup = NULL;
   copy->save.wrapobj = NULL;
   copy->save.wrapobjnames = NULL;
   copy->save.muscwrap_associations = NULL;
   copy->save.deform = NULL;
   copy->save.marker = NULL;
   copy->save.conspt_associations = NULL;
   copy->save.constraintobj = NULL;
   copy->save.constraintobjnames = NULL;

   copy->dynamic_param_names = (char**)simm_malloc(sizeof(char*)*ms->num_dynamic_params);
   for (i=0; i<ms->num_dynamic_params; i++)
      mstrcpy(&copy->dynamic_param_names[i], ms->dynamic_param_names[i]);

   copy->contactPair = copy_contact_pairs(ms->contactPair, ms->numContactPairs);
   copy->contactPairArraySize = ms->numContactPairs;

   copy->contactGroup = copy_contact_groups(ms->contactGroup, ms->numContactGroups);
   copy->contactGroupArraySize = ms->numContactGroups;

   copy->motion_objects = copy_motion_objects(ms->motion_objects, ms->num_motion_objects);

   copy->wrapobj = copy_wrap_objects(ms->wrapobj, ms->num_wrap_objects);
   copy->wrap_object_array_size = ms->num_wrap_objects;

   copy->deformity = copy_deformities(ms->deformity, ms->num_deformities, ms);
   copy->deformity_array_size = ms->num_deformities;

   // copy loop

   copy->constraintobj = copy_constraint_objects(ms->constraintobj, ms->num_constraint_objects);
   copy->constraint_object_array_size = ms->num_constraint_objects;

   copy->motion = copy_motions(ms->motion, ms->num_motions);
   copy->motion_array_size = ms->num_motions;

   check_definitions(copy);

   return copy;
}


