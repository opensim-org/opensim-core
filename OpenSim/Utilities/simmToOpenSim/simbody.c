/*******************************************************************************

   SIMBODY.C

   Author: Peter Loan

   Date: 06-AUG-2008

   Copyright (c) 2008 MusculoGraphics, Inc.
   All rights reserved.

   Description: 

   Routines:

*******************************************************************************/

#include <float.h>
#include <errno.h>

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "sdfunctions.h"


/*************** DEFINES (for this file only) *********************************/
typedef struct {
   int jointnum;
   int dofnum;
} GencoordInfo;

/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/


/*************** GLOBAL VARIABLES (used in only a few files) ******************/


/*************** EXTERNED VARIABLES (declared in another .c file) *************/
extern SDSegment* SDseg;
extern int num_SD_segs;
extern int* joint_order;
extern int gAngleUnits;

/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void convert_fixed_joints(ModelStruct* ms, GencoordInfo gcInfo[]);
static int add_constant_function_to_model(ModelStruct* ms, double y_value);
static void make_simbody_joint(ModelStruct* ms, FILE* fp, JointSDF* jntsdf, int jointnum, int sdnum,
                               SegmentSDF segs[], GencoordInfo gcInfo[], int* dofcount, int* constrainedcount,
                               char geometryDirectory[]);
static void write_opensim20_ground_body(FILE* fp, ModelStruct* ms, SegmentStruct* seg, char geometryDirectory[]);
static void write_opensim20_coordinate(FILE* fp, ModelStruct* ms, JointStruct* joint, int dofIndex);
static void write_opensim20_transformAxis(FILE* fp, ModelStruct* ms, JointStruct* joint, int dofIndex,
                                          Direction dir, SBoolean writeFunction);
static void write_opensim20_constraint(FILE* fp, ModelStruct* ms, DofStruct* dof, int dofIndex);
static SBoolean joint_is_fixed(JointStruct* joint);

/******************* PROTOTYPES for FUNCTIONS in SIMBODY.C  *******************/
void convert_dof_to_function(ModelStruct* ms, DofStruct* dof, int gencoord);
void extract_joint_locations_and_orientations(ModelStruct* ms,
                                              JointStruct* joint,
                                              int dof_order[],
                                              Direction dir,
                                              double locationInParent[],
                                              double orientationInParent[],
                                              double locationInChild[],
                                              double orientationInChild[]);
void write_vtk_bone(PolyhedronStruct* bone, char geometryDirectory[], char filename[]);
int segment_has_wrap_objects(ModelStruct* ms, SegmentStruct* seg);
void write_xml_wrap_object(FILE* fp, WrapObject* wo);


ReturnCode make_simbody_model(FILE* fp, ModelStruct* ms, char geometryDirectory[])
{
   int i, j, dofcount, constrainedcount, *joint_order;
   JointSDF* jnts;
   SegmentSDF* segs;
   GencoordInfo* gcInfo;

   joint_order = (int*)malloc(ms->numjoints*sizeof(int));
   for (i = 0; i < ms->numjoints; i++)
      joint_order[i] = -1;
   jnts = (JointSDF*)simm_calloc(ms->numjoints, sizeof(JointSDF));
   segs = (SegmentSDF*)simm_calloc(ms->numsegments, sizeof(SegmentSDF));
   // Make extra room for gcInfo because when fixed joints are converted, a new
   // gencoord is added to the model.
   gcInfo = (GencoordInfo*)simm_calloc(ms->numgencoords + ms->numjoints, sizeof(GencoordInfo));

   // Make the joint transformation matrices once you have changed the gencoords.
   for (i=0; i<ms->numjoints; i++)
      make_conversion(ms->modelnum, i);

   // Re-initialize the SD/FAST variables. Mark all dofs as
   // constrained (not true degrees of freedom).
   // Then for each gencoord, find the ONE dof which best
   // corresponds to the gencoord and mark it unconstrained.
   // That is, of all the dofs that are functions of a particular
   // gencoord, one should be a direct mapping (between dof and
   // gencoord) and the others should be non-trivial functions.
   for (i=0; i<ms->numjoints; i++)
   {
      for (j=0; j<6; j++)
      {
         ms->joint[i].dofs[j].sd.name = NULL;
         ms->joint[i].dofs[j].sd.con_name = NULL;
         ms->joint[i].dofs[j].sd.initial_value = 0.0;
         ms->joint[i].dofs[j].sd.constrained = yes;
         ms->joint[i].dofs[j].sd.fixed = no;
         ms->joint[i].dofs[j].sd.state_number = -1;
         ms->joint[i].dofs[j].sd.error_number = -1;
         ms->joint[i].dofs[j].sd.joint = -1;
         ms->joint[i].dofs[j].sd.axis = -1;
         ms->joint[i].dofs[j].sd.conversion = 0.0;
         ms->joint[i].dofs[j].sd.conversion_sign = 1.0;
      }
   }

   // Give the joints one-token names. In most cases, this is just
   // the user-given name of the joint. However, if that name has special
   // characters in it (e.g. -), those characters must be removed. Also,
   // if the name starts with a number, then an underscore is prepended.
   // Set the type to dpUnknownJoint; the type is not needed by the Simbody
   // exporter, except that it can't be dpSkippable.
   for (i = 0; i < ms->numjoints; i++)
   {
      JointStruct* jnt = &ms->joint[i];
      FREE_IFNOTNULL(jnt->sd_name);
      jnt->sd_name = (char*)simm_malloc(strlen(jnt->name) + 2);
      strcpy(jnt->sd_name, jnt->name);
      convert_string(jnt->sd_name, yes);
      ms->joint[i].type = dpUnknownJoint;
   }

   for (i=0; i<ms->numgencoords; i++)
   {
      if (ms->gencoord[i].used_in_model == yes)
      {
         if (!mark_unconstrained_dof(ms, i, &gcInfo[i].jointnum, &gcInfo[i].dofnum))
         {
            sprintf(errorbuffer, "At least one DOF must be a \"simple\" function of gencoord %s (2 points, slope=1, passes thru zero).",
               ms->gencoord[i].name);
            error(none,errorbuffer);
            return code_bad;
         }
      }
   }

   // Now give the dofs names for use in the OpenSim model file.
   // Names of unconstrained dofs will simply be the names of
   // the gencoords to which they correspond. Names of
   // constrained dofs will be formed from the joint name
   // and the dof keyword (e.g. "hip_tx" and "knee_r1").
   name_dofs(ms->modelnum);

   find_sdfast_joint_order(ms, jnts, segs, joint_order);

   // Malloc space for the array of segment names. These names include $ground
   // and the names of the "split" body segments. There can be at most
   // (numjoints + 1) segment names.
   // Free the SDseg array, if it was used previously.
   if (SDseg)
   {
      for (i = 0; i < num_SD_segs; i++)
         FREE_IFNOTNULL(SDseg[i].name);
      FREE_IFNOTNULL(SDseg);
   }

   SDseg = (SDSegment*)simm_calloc(ms->numjoints + 1, sizeof(SDSegment));
   SDseg[0].name = (char*)simm_malloc(strlen(ms->segment[ms->ground_segment].name) + 2);
   strcpy(SDseg[0].name, ms->segment[ms->ground_segment].name);
   convert_string(SDseg[0].name, yes);
   SDseg[0].simm_segment = ms->ground_segment;
   SDseg[0].mass_center[0] = ms->segment[ms->ground_segment].masscenter[0];
   SDseg[0].mass_center[1] = ms->segment[ms->ground_segment].masscenter[1];
   SDseg[0].mass_center[2] = ms->segment[ms->ground_segment].masscenter[2];
   SDseg[0].mass = 0.0;
   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
         SDseg[0].inertia[i][j] = 0.0;

   num_SD_segs = 1;

   convert_fixed_joints(ms, gcInfo);

   // Bodies (with joints and wrap objects)
   fprintf(fp, "\t\t\t<BodySet>\n");
   fprintf(fp, "\t\t\t<objects>\n");
   write_opensim20_ground_body(fp, ms, &ms->segment[ms->ground_segment], geometryDirectory);
   for (i=0, dofcount=0, constrainedcount=0; i<ms->numjoints; i++)
   {
      make_simbody_joint(ms, fp, &jnts[joint_order[i]], joint_order[i], ms->joint[i].sd_num, segs, gcInfo,
         &dofcount, &constrainedcount, geometryDirectory);
   }
   fprintf(fp, "\t\t\t</objects>\n");
   fprintf(fp, "\t\t\t</BodySet>\n");

   // Constraints are used when the DOF is constrained to a coordinate
   // that is used in another joint.
   fprintf(fp, "\t\t\t<ConstraintSet>\n");
   fprintf(fp, "\t\t\t<objects>\n");
   for (i = 0; i < ms->numjoints; i++)
   {
      JointStruct* joint = &ms->joint[joint_order[i]];
      for (j = 0; j < 6; j++)
      {
         if (joint->dofs[j].sd.constrained == yes && joint->dofs[j].gencoord >= 0 &&
            gcInfo[joint->dofs[j].gencoord].jointnum != joint_order[i])
            write_opensim20_constraint(fp, ms, &joint->dofs[j], j);
      }
   }
   for (i=0; i<ms->numsegments; i++)
   {
      for (j=0; j<segs[i].times_split; j++)
      {
         char* new_name = make_sdfast_seg_name(ms->segment[i].name, j+1);
         fprintf(fp, "\t\t\t\t<WeldConstraint name=\"%s_%s\">\n", ms->segment[i].name, new_name);
         fprintf(fp, "\t\t\t\t\t<isDisabled> false </isDisabled>\n");
         fprintf(fp, "\t\t\t\t\t<body_1> %s </body_1>\n", ms->segment[i].name);
         fprintf(fp, "\t\t\t\t\t<body_2> %s </body_2>\n", new_name);
         fprintf(fp, "\t\t\t\t\t<location_body_1> 0.0 0.0 0.0 </location_body_1>\n");
         fprintf(fp, "\t\t\t\t\t<orientation_body_1> 0.0 0.0 0.0 </orientation_body_1>\n");
         fprintf(fp, "\t\t\t\t\t<location_body_2> 0.0 0.0 0.0 </location_body_2>\n");
         fprintf(fp, "\t\t\t\t\t<orientation_body_2> 0.0 0.0 0.0 </orientation_body_2>\n");
         fprintf(fp, "\t\t\t\t</WeldConstraint>\n");
         FREE_IFNOTNULL(new_name);
      }
   }
   fprintf(fp, "\t\t\t</objects>\n");
   fprintf(fp, "\t\t\t</ConstraintSet>\n");

   free(joint_order);
   for (i=0; i<ms->numjoints; i++)
   {
      FREE_IFNOTNULL(jnts[i].inbname);
      FREE_IFNOTNULL(jnts[i].outbname);
   }
   free(jnts);
   free(segs);
   free(gcInfo);

   return code_fine;
}


static void make_simbody_joint(ModelStruct* ms, FILE* fp, JointSDF* jntsdf, int jointnum, int sdnum,
                               SegmentSDF segs[], GencoordInfo gcInfo[], int* dofcount, int* constrainedcount,
                               char geometryDirectory[])
{
   int i, j, dof_order[6];
   SegmentStruct* seg1;
   SegmentStruct* seg2;
   SegmentSDF* segsdf;
   JointStruct* joint;
   double locationInParent[3], locationInChild[3], orientationInParent[3], orientationInChild[3];

   joint = &ms->joint[jointnum];

   if (jntsdf->dir == FORWARD)
   {
      seg1 = &ms->segment[joint->from];
      seg2 = &ms->segment[joint->to];
      segsdf = &segs[joint->to];
   }
   else
   {
      seg1 = &ms->segment[joint->to];
      seg2 = &ms->segment[joint->from];
      segsdf = &segs[joint->from];
   }

   // Add the body name to the list of segment names. This list contains 'real'
   // segment names as well as the names of 'split' body segments.
   mstrcpy(&(SDseg[num_SD_segs].name),jntsdf->outbname);

   // If there are loops in the model, then SIMM segments get split and there
   // will be more SD segments than SIMM segments. So that unsplittable segment
   // parameters (like contact objects) can be assigned to the SD segments,
   // each SD segment has an index of its corresponding SIMM segment. But for
   // segments that were split, only one piece will have a valid index.
   if (jntsdf->closes_loop == no)
   {
      if (jntsdf->dir == FORWARD)
         SDseg[num_SD_segs].simm_segment = joint->to;
      else
         SDseg[num_SD_segs].simm_segment = joint->from;
   }
   else
   {
      SDseg[num_SD_segs].simm_segment = -1;
   }

   SDseg[num_SD_segs].mass = seg2->mass / segsdf->mass_factor;
   SDseg[num_SD_segs].inertia[0][0] = seg2->inertia[0][0] / segsdf->mass_factor;
   SDseg[num_SD_segs].inertia[0][1] = seg2->inertia[0][1] / segsdf->mass_factor;
   SDseg[num_SD_segs].inertia[0][2] = seg2->inertia[0][2] / segsdf->mass_factor;
   SDseg[num_SD_segs].inertia[1][0] = seg2->inertia[1][0] / segsdf->mass_factor;
   SDseg[num_SD_segs].inertia[1][1] = seg2->inertia[1][1] / segsdf->mass_factor;
   SDseg[num_SD_segs].inertia[1][2] = seg2->inertia[1][2] / segsdf->mass_factor;
   SDseg[num_SD_segs].inertia[2][0] = seg2->inertia[2][0] / segsdf->mass_factor;
   SDseg[num_SD_segs].inertia[2][1] = seg2->inertia[2][1] / segsdf->mass_factor;
   SDseg[num_SD_segs].inertia[2][2] = seg2->inertia[2][2] / segsdf->mass_factor;
   SDseg[num_SD_segs].mass_center[0] = seg2->masscenter[0];
   SDseg[num_SD_segs].mass_center[1] = seg2->masscenter[1];
   SDseg[num_SD_segs].mass_center[2] = seg2->masscenter[2];

   fprintf(fp, "\t\t\t\t<Body name=\"%s\">\n", jntsdf->outbname);
   fprintf(fp, "\t\t\t\t\t<mass> %.12lf </mass>\n", SDseg[num_SD_segs].mass);
   fprintf(fp, "\t\t\t\t\t<mass_center> %.12lf %.12lf %.12lf </mass_center>\n", SDseg[num_SD_segs].mass_center[0], SDseg[num_SD_segs].mass_center[1], SDseg[num_SD_segs].mass_center[2]);
   fprintf(fp, "\t\t\t\t\t<inertia_xx> %.12lf </inertia_xx>\n", SDseg[num_SD_segs].inertia[0][0]);
   fprintf(fp, "\t\t\t\t\t<inertia_yy> %.12lf </inertia_yy>\n", SDseg[num_SD_segs].inertia[1][1]);
   fprintf(fp, "\t\t\t\t\t<inertia_zz> %.12lf </inertia_zz>\n", SDseg[num_SD_segs].inertia[2][2]);
   fprintf(fp, "\t\t\t\t\t<inertia_xy> %.12lf </inertia_xy>\n", SDseg[num_SD_segs].inertia[0][1]);
   fprintf(fp, "\t\t\t\t\t<inertia_xz> %.12lf </inertia_xz>\n", SDseg[num_SD_segs].inertia[0][2]);
   fprintf(fp, "\t\t\t\t\t<inertia_yz> %.12lf </inertia_yz>\n", SDseg[num_SD_segs].inertia[1][2]);

   // Figure out in what order the 6 DOFs should be processed.
   dof_order[joint->order[TRANS]] = TX;
   dof_order[joint->order[TRANS]+1] = TY;
   dof_order[joint->order[TRANS]+2] = TZ;
   if (joint->order[ROT1] < joint->order[TRANS])
      dof_order[joint->order[ROT1]] = R1;
   else
      dof_order[joint->order[ROT1]+2] = R1;
   if (joint->order[ROT2] < joint->order[TRANS])
      dof_order[joint->order[ROT2]] = R2;
   else
      dof_order[joint->order[ROT2]+2] = R2;
   if (joint->order[ROT3] < joint->order[TRANS])
      dof_order[joint->order[ROT3]] = R3;
   else
      dof_order[joint->order[ROT3]+2] = R3;

   if (jntsdf->dir == INVERSE)
   {
      int tmp;
      tmp = dof_order[5];
      dof_order[5] = dof_order[0];
      dof_order[0] = tmp;
      tmp = dof_order[4];
      dof_order[4] = dof_order[1];
      dof_order[1] = tmp;
      tmp = dof_order[3];
      dof_order[3] = dof_order[2];
      dof_order[2] = tmp;
   }

   extract_joint_locations_and_orientations(ms, joint, dof_order, jntsdf->dir, locationInParent, orientationInParent,
      locationInChild, orientationInChild);

   fprintf(fp, "\t\t\t\t\t<Joint>\n");
   fprintf(fp, "\t\t\t\t\t\t<CustomJoint name=\"%s\">\n", joint->name);
   fprintf(fp, "\t\t\t\t\t\t\t<parent_body> %s </parent_body>\n", jntsdf->inbname);
   fprintf(fp, "\t\t\t\t\t\t\t<location_in_parent> %.12lf %.12lf %.12lf </location_in_parent>\n",
      locationInParent[0], locationInParent[1], locationInParent[2]);
   fprintf(fp, "\t\t\t\t\t\t\t<orientation_in_parent> %.12lf %.12lf %.12lf </orientation_in_parent>\n",
      orientationInParent[0], orientationInParent[1], orientationInParent[2]);
   fprintf(fp, "\t\t\t\t\t\t\t<location> %.12lf %.12lf %.12lf </location>\n",
      locationInChild[0], locationInChild[1], locationInChild[2]);
   fprintf(fp, "\t\t\t\t\t\t\t<orientation> %.12lf %.12lf %.12lf </orientation>\n",
      orientationInChild[0], orientationInChild[1], orientationInChild[2]);

   fprintf(fp, "\t\t\t\t\t\t\t<!--Generalized coordinates parameterizing this joint.-->\n");

   fprintf(fp, "\t\t\t\t\t\t\t<CoordinateSet>\n");
   fprintf(fp, "\t\t\t\t\t\t\t<objects>\n");
   for (i=0; i<6; i++)
   {
      DofStruct* dof = &joint->dofs[dof_order[i]];
      // This DOF shows up in the coordinate list if it represents an unconstrained coordinate,
      // or if it's constrained to a coordinate that is used in another joint. Or if it is one
      // of the newly created gencoords.
      if (dof->type == function_dof)
      {
         if ((gcInfo[dof->gencoord].jointnum == jointnum && gcInfo[dof->gencoord].dofnum == dof_order[i]) ||
              gcInfo[dof->gencoord].jointnum != jointnum ||
              ms->gencoord[dof->gencoord].defined == no)
            write_opensim20_coordinate(fp, ms, joint, dof_order[i]);
      }
   }
   fprintf(fp, "\t\t\t\t\t\t\t</objects>\n");
   fprintf(fp, "\t\t\t\t\t\t\t</CoordinateSet>\n");

   fprintf(fp, "\t\t\t\t\t\t\t<TransformAxisSet>\n");
   fprintf(fp, "\t\t\t\t\t\t\t<objects>\n");
   for (i=0; i<6; i++)
   {
      DofStruct* dof = &joint->dofs[dof_order[i]];
      // DOFs that are functions always show up in the transform axis list. If the DOF represents
      // an unconstrained coordinate or if it's constrained to a coordinate in another joint,
      // then it shows up without a function. If the DOF is constrained to a coordinate in this
      // joint, it shows up with a function (i.e., it's a function-based mobilizer).
      if (dof->type == function_dof)
      {
         if ((gcInfo[dof->gencoord].jointnum == jointnum && gcInfo[dof->gencoord].dofnum == dof_order[i]) ||
              gcInfo[dof->gencoord].jointnum != jointnum)
            write_opensim20_transformAxis(fp, ms, joint, dof_order[i], jntsdf->dir, no);
         else
            write_opensim20_transformAxis(fp, ms, joint, dof_order[i], jntsdf->dir, yes);
      }
   }
   fprintf(fp, "\t\t\t\t\t\t\t</objects>\n");
   fprintf(fp, "\t\t\t\t\t\t\t</TransformAxisSet>\n");

   fprintf(fp, "\t\t\t\t\t\t</CustomJoint>\n");
   fprintf(fp, "\t\t\t\t\t</Joint>\n");

   num_SD_segs++;

   // Don't output bones, wrap object, etc., for the "split" segments.
   if (jntsdf->closes_loop == no)
   {
      if (seg2->numBones > 0)
      {
         char vtpFileName[CHARBUFFER];
         SBoolean madeDir = no;

         fprintf(fp, "\t\t\t\t\t<VisibleObject name=\"\">\n");
         fprintf(fp, "\t\t\t\t\t\t<geometry_files> ");
         for (j = 0; j < seg2->numBones; j++)
         {
            change_filename_suffix(seg2->bone[j].name, vtpFileName, "vtp");
            fprintf(fp, "%s ", vtpFileName);
            // Only write the bone file if the PolyhedronStruct is not empty
            if (seg2->bone[j].num_vertices > 0 && seg2->bone[j].num_polygons > 0)
            {
               // The first time you write a bone file, check to see if the
               // output directory needs to be created.
               if (madeDir == no)
               {
                  if (geometryDirectory)
                  {
                     if (makeDir(geometryDirectory) != 0 && errno != EEXIST)
                     {
                        printf("Warning: Unable to create geometry directory %s.\n", geometryDirectory);
                        printf("         VTK geometry files will not be output for this model.\n");
                     }
                  }
                  // Whether or not you successfully created the geometry directory,
                  // you don't want to try to create it again.
                  madeDir = yes;
               }
               write_vtk_bone(&seg2->bone[j], geometryDirectory, vtpFileName);
            }
         }
         fprintf(fp, "</geometry_files>\n");
         fprintf(fp, "\t\t\t\t\t\t<scale_factors> %.12f %.12f %.12f </scale_factors>\n", seg2->bone_scale[0], seg2->bone_scale[1], seg2->bone_scale[2]);
         fprintf(fp, "\t\t\t\t\t</VisibleObject>\n");
      }
      if (segment_has_wrap_objects(ms, seg2))
      {
         fprintf(fp, "\t\t\t\t\t<WrapObjectSet>\n");
         fprintf(fp, "\t\t\t\t\t<objects>\n");
         for (j = 0; j < ms->num_wrap_objects; j++)
         {
            if (&ms->segment[ms->wrapobj[j].segment] == seg2)
               write_xml_wrap_object(fp, &ms->wrapobj[j]);
         }
         fprintf(fp, "\t\t\t\t\t</objects>\n");
         fprintf(fp, "\t\t\t\t\t</WrapObjectSet>\n");
      }
   }

   fprintf(fp, "\t\t\t\t</Body>\n");
}

static void write_opensim20_ground_body(FILE* fp, ModelStruct* ms, SegmentStruct* seg, char geometryDirectory[])
{
	int j;
	SBoolean madeDir = no;

   fprintf(fp, "\t\t\t\t<Body name=\"%s\">\n", seg->name);
   fprintf(fp, "\t\t\t\t\t<mass> %.12lf </mass>\n", seg->mass);
   fprintf(fp, "\t\t\t\t\t<mass_center> %.12lf %.12lf %.12lf </mass_center>\n", seg->masscenter[0], seg->masscenter[1], seg->masscenter[2]);
   fprintf(fp, "\t\t\t\t\t<inertia_xx> %.12lf </inertia_xx>\n", seg->inertia[0][0]);
   fprintf(fp, "\t\t\t\t\t<inertia_yy> %.12lf </inertia_yy>\n", seg->inertia[1][1]);
   fprintf(fp, "\t\t\t\t\t<inertia_zz> %.12lf </inertia_zz>\n", seg->inertia[2][2]);
   fprintf(fp, "\t\t\t\t\t<inertia_xy> %.12lf </inertia_xy>\n", seg->inertia[0][1]);
   fprintf(fp, "\t\t\t\t\t<inertia_xz> %.12lf </inertia_xz>\n", seg->inertia[0][2]);
   fprintf(fp, "\t\t\t\t\t<inertia_yz> %.12lf </inertia_yz>\n", seg->inertia[1][2]);
   fprintf(fp, "\t\t\t\t\t<Joint/>\n");

   // The ground body does not have a joint, but it can have bones and wrap objects.
   if (seg->numBones > 0)
   {
      char vtpFileName[CHARBUFFER];

      fprintf(fp, "\t\t\t\t\t<VisibleObject name=\"\">\n");
      fprintf(fp, "\t\t\t\t\t\t<geometry_files> ");
      for (j = 0; j < seg->numBones; j++)
      {
         change_filename_suffix(seg->bone[j].name, vtpFileName, "vtp");
         fprintf(fp, "%s ", vtpFileName);
         // Only write the bone file if the PolyhedronStruct is not empty
         if (seg->bone[j].num_vertices > 0 && seg->bone[j].num_polygons > 0)
         {
            // The first time you write a bone file, check to see if the
            // output directory needs to be created.
            if (madeDir == no)
            {
               if (geometryDirectory)
               {
                  if (makeDir(geometryDirectory) != 0 && errno != EEXIST)
                  {
                     printf("Warning: Unable to create geometry directory %s.\n", geometryDirectory);
                     printf("         VTK geometry files will not be output for this model.\n");
                  }
               }
               // Whether or not you successfully created the geometry directory,
               // you don't want to try to create it again.
               madeDir = yes;
            }
            write_vtk_bone(&seg->bone[j], geometryDirectory, vtpFileName);
         }
      }
      fprintf(fp, "</geometry_files>\n");
      fprintf(fp, "\t\t\t\t\t\t<scale_factors> %.12f %.12f %.12f </scale_factors>\n", seg->bone_scale[0], seg->bone_scale[1], seg->bone_scale[2]);
      fprintf(fp, "\t\t\t\t\t</VisibleObject>\n");
   }
   if (segment_has_wrap_objects(ms, seg))
   {
      fprintf(fp, "\t\t\t\t\t<WrapObjectSet>\n");
      fprintf(fp, "\t\t\t\t\t<objects>\n");
      for (j = 0; j < ms->num_wrap_objects; j++)
      {
         if (&ms->segment[ms->wrapobj[j].segment] == seg)
            write_xml_wrap_object(fp, &ms->wrapobj[j]);
      }
      fprintf(fp, "\t\t\t\t\t</objects>\n");
      fprintf(fp, "\t\t\t\t\t</WrapObjectSet>\n");
   }
   fprintf(fp, "\t\t\t\t</Body>\n");
}

static void write_opensim20_coordinate(FILE* fp, ModelStruct* ms, JointStruct* joint, int dofIndex)
{
   DofStruct* dof = &joint->dofs[dofIndex];
   GeneralizedCoord* gc = &ms->gencoord[dof->gencoord];
   double conversion = 1.0;

   fprintf(fp, "\t\t\t\t\t\t\t\t<Coordinate name=\"%s\">\n", dof->sd.name);
   if (dof->sd.constrained == no)
   {
      int j;
      // If the gencoord is [primarily] translational, then do not convert its
	   // range, default_value, etc. If it's rotational, then use DEG_TO_RAD.
      if (gc->type == rotation_gencoord && gAngleUnits == RADIANS)
         conversion = DEG_TO_RAD;

      fprintf(fp, "\t\t\t\t\t\t\t\t\t<default_value> %.12lf </default_value>\n", gc->default_value * conversion);
      fprintf(fp, "\t\t\t\t\t\t\t\t\t<initial_value> %.12lf </initial_value>\n", gc->default_value * conversion);
      if (NOT_EQUAL_WITHIN_ERROR(gc->tolerance, 0.0))
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<tolerance> %.12lf </tolerance>\n", gc->tolerance * conversion);
      if (NOT_EQUAL_WITHIN_ERROR(gc->pd_stiffness, 0.0))
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<stiffness> %.12lf </stiffness>\n", gc->pd_stiffness);
      fprintf(fp, "\t\t\t\t\t\t\t\t\t<range> %.12lf %.12lf </range>\n", gc->range.start * conversion, gc->range.end * conversion);
      if (gc->keys[0] != null_key)
      {
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<keys> %s ", get_simmkey_name((int)gc->keys[0]));
         if (gc->keys[1] != gc->keys[0])
            fprintf(fp, "%s ", get_simmkey_name((int)gc->keys[1]));
         fprintf(fp,"</keys>\n");
      }
		fprintf(fp, "\t\t\t\t\t\t\t\t\t<clamped> %s </clamped>\n", (gc->clamped == yes) ? ("true") : ("false"));
		fprintf(fp, "\t\t\t\t\t\t\t\t\t<locked> %s </locked>\n", (gc->locked == yes) ? ("true") : ("false"));
		fprintf(fp, "\t\t\t\t\t\t\t\t\t<restraint_active> %s </restraint_active>\n", (gc->restraintFuncActive == yes) ? ("true") : ("false"));
		if (gc->restraint_func_num != -1)
		{
			SplineFunction* sf = &ms->function[gc->restraint_func_num];

			fprintf(fp, "\t\t\t\t\t\t\t\t\t<restraint_function>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t<natCubicSpline name=\"f%d\">\n", gc->restraint_func_num);
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t\t<x>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->x[j] * conversion);
			fprintf(fp, "</x>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t\t<y>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->y[j]);
			fprintf(fp, "</y>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t</natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t</restraint_function>\n");
		}
		if (gc->min_restraint_func_num != -1)
		{
			SplineFunction* sf = &ms->function[gc->min_restraint_func_num];

			fprintf(fp, "\t\t\t\t\t\t\t\t\t<min_restraint_function>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t<natCubicSpline name=\"f%d\">\n", gc->min_restraint_func_num);
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t\t<x>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->x[j] * conversion);
			fprintf(fp, "</x>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t\t<y>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->y[j]);
			fprintf(fp, "</y>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t</natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t</min_restraint_function>\n");
		}
		if (gc->max_restraint_func_num != -1)
		{
			SplineFunction* sf = &ms->function[gc->max_restraint_func_num];

			fprintf(fp, "\t\t\t\t\t\t\t\t\t<max_restraint_function>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t<natCubicSpline name=\"f%d\">\n", gc->max_restraint_func_num);
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t\t<x>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->x[j] * conversion);
			fprintf(fp, "</x>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t\t<y>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->y[j]);
			fprintf(fp, "</y>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t\t</natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t\t\t\t\t</max_restraint_function>\n");
		}
   } else { // dof->sd.constrained = yes
      if (gAngleUnits == RADIANS)
      {
         // check if dof (dependent coordinate) is r1, r2, or r3
         if (dofIndex < 3)
            conversion = DEG_TO_RAD;
      }
      fprintf(fp, "\t\t\t\t\t\t\t\t\t<default_value> %.12lf </default_value>\n", dof->value * conversion);
      fprintf(fp, "\t\t\t\t\t\t\t\t\t<initial_value> %.12lf </initial_value>\n", dof->value * conversion);
      fprintf(fp, "\t\t\t\t\t\t\t\t\t<range> -99999.9 99999.9 </range>\n"); //TODO20: maybe not specify, so default is used?
		fprintf(fp, "\t\t\t\t\t\t\t\t\t<clamped> false </clamped>\n");
		fprintf(fp, "\t\t\t\t\t\t\t\t\t<locked> false </locked>\n");
   }
   fprintf(fp, "\t\t\t\t\t\t\t\t</Coordinate>\n");
}

static void write_opensim20_transformAxis(FILE* fp, ModelStruct* ms, JointStruct* joint, int dofIndex, Direction dir, SBoolean writeFunction)
{
   DofStruct* dof = &joint->dofs[dofIndex];
   GeneralizedCoord* gc = &ms->gencoord[dof->gencoord];

   fprintf(fp, "\t\t\t\t\t\t\t\t<TransformAxis name=\"%s\">\n", getjointvarname(dofIndex));
   fprintf(fp, "\t\t\t\t\t\t\t\t\t<is_rotation> %s </is_rotation>\n", (dofIndex > 2) ? ("false") : ("true"));
   if (dir == FORWARD)
   {
      if (dofIndex == TX)
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> 1.0 0.0 0.0 </axis>\n");
      else if (dofIndex == TY)
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> 0.0 1.0 0.0 </axis>\n");
      else if (dofIndex == TZ)
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> 0.0 0.0 1.0 </axis>\n");
      else
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> %.12lf %.12lf %.12lf </axis>\n", joint->parentrotaxes[dofIndex][0],
         joint->parentrotaxes[dofIndex][1], joint->parentrotaxes[dofIndex][2]);
   }
   else
   {
      if (dofIndex == TX)
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> -1.0 0.0 0.0 </axis>\n");
      else if (dofIndex == TY)
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> 0.0 -1.0 0.0 </axis>\n");
      else if (dofIndex == TZ)
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> 0.0 0.0 -1.0 </axis>\n");
      else
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> %.12lf %.12lf %.12lf </axis>\n", -joint->parentrotaxes[dofIndex][0],
         -joint->parentrotaxes[dofIndex][1], -joint->parentrotaxes[dofIndex][2]);
   }
   if (writeFunction == yes)
   {
      int i;
      SplineFunction* sf = &ms->function[dof->funcnum];
      double ind_conv = 1.0, dep_conv = 1.0;

      if (gAngleUnits == RADIANS)
      {
         // check if gencoord (independent coordinate) is rotational
         if (ms->gencoord[dof->gencoord].type == rotation_gencoord)
            ind_conv = DEG_TO_RAD;
         // check if dof (dependent coordinate) is r1, r2, or r3
         if (dofIndex < 3)
            dep_conv = DEG_TO_RAD;
      }
      fprintf(fp, "\t\t\t\t\t\t\t\t\t<function name=\"f%d\">\n", sf->usernum);
      fprintf(fp, "\t\t\t\t\t\t\t\t\t\t<natCubicSpline>\n");
      fprintf(fp, "\t\t\t\t\t\t\t\t\t\t\t<x> ");
      for (i=0; i<sf->numpoints; i++)
         fprintf(fp, "%.12lf ", sf->x[i] * ind_conv);
      fprintf(fp, "</x>\n");
      fprintf(fp, "\t\t\t\t\t\t\t\t\t\t\t<y> ");
      for (i=0; i<sf->numpoints; i++)
         fprintf(fp, "%.12lf ", sf->y[i] * dep_conv);
      fprintf(fp, "</y>\n");
      fprintf(fp, "\t\t\t\t\t\t\t\t\t\t</natCubicSpline>\n");
      fprintf(fp, "\t\t\t\t\t\t\t\t\t</function>\n");
      // If you are writing the function, the 'coordinate' is the independent coordinte in
      // the function.
      fprintf(fp, "\t\t\t\t\t\t\t\t\t<coordinate> %s </coordinate>\n", ms->gencoord[dof->gencoord].name);
   }
   else
   {
      // If you're not writing the function, then this transform axis is its own
      // coordinate (either unconstrained, or constrained to a coordinate in
      // another joint with a CoordinateCoupler.
      fprintf(fp, "\t\t\t\t\t\t\t\t\t<coordinate> %s </coordinate>\n", dof->sd.name);
   }
   fprintf(fp, "\t\t\t\t\t\t\t\t</TransformAxis>\n");
}

static void write_opensim20_constraint(FILE* fp, ModelStruct* ms, DofStruct* dof, int dofIndex)
{
   if (dof->gencoord >= 0 && dof->funcnum >= 0)
   {
      int i;
      SplineFunction* func = &ms->function[dof->funcnum];
      double ind_conv = 1.0, dep_conv = 1.0;

      if (gAngleUnits == RADIANS)
      {
         // check if gencoord (independent coordinate) is rotational
         if (ms->gencoord[dof->gencoord].type == rotation_gencoord)
            ind_conv = DEG_TO_RAD;
         // check if dof (dependent coordinate) is r1, r2, or r3
         if (dofIndex < 3)
            dep_conv = DEG_TO_RAD;
      }

      fprintf(fp, "\t\t\t\t<CoordinateCouplerConstraint name=\"%s\">\n", dof->sd.con_name);
      fprintf(fp, "\t\t\t\t\t<isDisabled> false </isDisabled>\n");
      fprintf(fp, "\t\t\t\t\t<dependent_coordinate_name> %s </dependent_coordinate_name>\n", dof->sd.name);
      fprintf(fp, "\t\t\t\t\t<independent_coordinate_names> %s </independent_coordinate_names>\n", ms->gencoord[dof->gencoord].name);
      fprintf(fp, "\t\t\t\t\t<coupled_coordinates_function name=\"f%d\">\n", func->usernum);
      fprintf(fp, "\t\t\t\t\t<natCubicSpline name=\"f%d\">\n", func->usernum);
      fprintf(fp, "\t\t\t\t\t\t<x> ");
      for (i=0; i<func->numpoints; i++)
         fprintf(fp, "%.12lf ", func->x[i] * ind_conv);
      fprintf(fp, "</x>\n");
      fprintf(fp, "\t\t\t\t\t\t<y> ");
      for (i=0; i<func->numpoints; i++)
         fprintf(fp, "%.12lf ", func->y[i] * dep_conv);
      fprintf(fp, "</y>\n");
      fprintf(fp, "\t\t\t\t\t</natCubicSpline>\n");
      fprintf(fp, "\t\t\t\t\t</coupled_coordinates_function>\n");
      fprintf(fp, "\t\t\t\t</CoordinateCouplerConstraint>\n");
   }
}

// The current version of Simbody cannot handle joints with no degrees of freedom.
// For now, deal with them by making one DOF a constrained function of a gencoord.
// The DOF chosen is the last one in the joint, so that in case the others are
// non-zero they can be more easily converted into the <location_in_parent> and
// <orientation_in_parent> parameters. A new gencoord is created for this constraint,
// so that it can be implemented as a function-based mobilizer in Simbody.
static void convert_fixed_joints(ModelStruct* ms, GencoordInfo gcInfo[])
{
   int i;

   for (i=0; i<ms->numjoints; i++)
   {
      JointStruct* joint = &ms->joint[i];
      if (joint_is_fixed(joint))
      {
         int dofnum;
         if (joint->order[TRANS] == 3)
            dofnum = TX;
         else if (joint->order[ROT1] == 3)
            dofnum = R1;
         else if (joint->order[ROT2] == 3)
            dofnum = R2;
         else
            dofnum = R3;
         convert_dof_to_function(ms, &joint->dofs[dofnum], -1);
         // Fill in gcInfo with values that will cause this constraint
         // to be implemented as a function-based mobilizer. That is,
         // specify that the gencoord used for this new function is
         // "based" in this joint, but in another DOF.
         gcInfo[joint->dofs[dofnum].gencoord].jointnum = i;
         gcInfo[joint->dofs[dofnum].gencoord].dofnum = 5 - dofnum;
      }
   }
}

static int add_constant_function_to_model(ModelStruct* ms, double y_value)
{
   int functionIndex = findUnusedFunctionNumber(ms->modelnum);
   SplineFunction *sf = &ms->function[functionIndex];
   malloc_function(sf, 2);
   sf->used = sf->defined = yes;
   sf->type = linear;
   sf->numpoints = 2;
   sf->x[0] = -999999.9999999;
   sf->x[1] = 999999.9999999;
   sf->y[0] = sf->y[1] = y_value;
   calc_spline_coefficients(sf);

   return functionIndex;
}

void convert_dof_to_function(ModelStruct* ms, DofStruct* dof, int gencoord)
{
   dof->type = function_dof;
   dof->funcnum = add_constant_function_to_model(ms, dof->value);
   if (gencoord < 0)
   {
      // Create a new gencoord for use in this function. The range and
      // other parameters of this gencoord are not specified, but they
      // are not needed for writing out an OpenSim model because the
      // OpenSim coordinate is constrained anyway.
      dof->gencoord = enter_gencoord(ms->modelnum, dof->sd.name, yes);
   }
   else
   {
      dof->gencoord = gencoord;
   }
   dof->sd.initial_value = dof->value;
   dof->sd.constrained = yes;
   dof->sd.fixed = no;
}

static SBoolean joint_is_fixed(JointStruct* joint)
{
   int i;
   for (i=0; i<6; i++)
      if (joint->dofs[i].type == function_dof)
         return no;
   return yes;
}
