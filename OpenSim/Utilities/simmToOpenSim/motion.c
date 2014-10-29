/*******************************************************************************

   MOTION.C

   Author: Peter Loan

   Date: 10-SEP-90

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description:

   Routines:
      apply_motion_to_model : moves a model according to a motion
      link_model_to_motion : checks motion column names
      check_motion_wrapping : checks boundaries of motion variable
      find_nth_motion       : returns the appropriate motion

*******************************************************************************/
#include <assert.h>

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "normio.h"
#include "normtools.h"
//#include "password.h"

/*************** DEFINES (for this file only) *********************************/
#define MOTION_OFFSET 0.0001
#define OTHER_NAMES_SIZE 3000

typedef struct {
   int segment;
   ContactObject* forceMatte;
   double pointOfApplication[3];
   double force[3];
   double freeTorque[3];
   double magnitude;
   int column[9];
} ForceMatteForce;


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
// default motion objects
static MotionObject sDefaultMotionObjects[] = {
   {
      "force" ,              /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 0.002, 1.0 },   /* startingScale (see NOTE below for more info) */
      { 0,0,0 },             /* starting XYZRotation */
      flat_shading,          /* drawmode */
      YY,                    /* vector_axis */
      "def_motion"           /* materialname */
   },
   {
      "ball" ,               /* name */
      "unit_sphere.asc",     /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1,1,1 },             /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      gouraud_shading,       /* drawmode */
      YY,                    /* vector_axis */
      "def_motion"           /* materialname */
   },
   {
      "contact" ,            /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 1.0, 1.0 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      flat_shading,          /* drawmode */
      YY,                    /* vector_axis */
      "def_motion"           /* materialname */
   },
   {
      "joint_force" ,        /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 1.0, 1.0 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      flat_shading,          /* drawmode */
      YY,                    /* vector_axis */
      "def_joint_vector"     /* materialname */
   },
   {
      "joint_torque" ,       /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 1.0, 1.0 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      flat_shading,          /* drawmode */
      YY,                    /* vector_axis */
      "def_joint_vector"     /* materialname */
   },
   {
      "spring_force" ,       /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 0.25, 1.0, 0.25 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      flat_shading,          /* drawmode */
      YY,                    /* vector_axis */
      "def_muscle_point"     /* materialname */
   },
   {
      "torque" ,             /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 0.0, 0.0, 0.0 },     /* startingScale---------> free torques in OpenSim MOT files map to this object, */
      { 0,0,0 },             /* starting XYZRotation    but you normally don't want to see them. So make the */
      flat_shading,          /* drawmode                starting scale 0, 0, 0. */
      YY,                    /* vector_axis */
      "def_joint_vector"     /* materialname */
   },
   {
      "rightFootPrint",      /* name */
      "rightFootPrint.asc",  /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 1.0, 1.0 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      solid_fill,            /* drawmode */
      YY,                    /* vector_axis */
      "foot_print_mat"       /* materialname */
   },
   {
      "leftFootPrint",       /* name */
      "leftFootPrint.asc",   /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 1.0, 1.0 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      solid_fill,            /* drawmode */
      YY,                    /* vector_axis */
      "foot_print_mat"       /* materialname */
   },
   {
      "forceplate",          /* name */
      "forceplate.asc",      /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 1.0, 1.0 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      outlined_polygons,     /* drawmode */
      YY,                    /* vector_axis */
      "def_world_object"     /* materialname */
   }
};
/* NOTE: the starting scale factors (0.75, 0.002, 0.75) for the default "force"
 *  arrow motion object were chosen to match SIMM's previous behavior.  -- KMS 12/20/99
 * NOTE: arrow.asc was changed to include the X and Z 0.75 factors. Thus the
 *  "force" scale factor was changed to 1.0, 0.002, 1.0 -- JPL 9/6/00
 */


/*************** GLOBAL VARIABLES (used in only a few files) ******************/


/*************** EXTERNED VARIABLES (declared in another file) ****************/
extern char natural_cubic_text[];
extern char gcv_text[];


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void update_current_motion_frame(MotionSequence* motion);
static SBoolean link_model_to_motion(ModelStruct* model, MotionSequence* motion,
                                     int* num_other_data, char* otherNames);
static MotionObjectInstance* store_motion_object_instance(ModelStruct *model, MotionModelOptions* mopt,
                                                          int seg, int motion_object, int compenant, int column);
static void add_vector(double pt[], double vec[], double freeTorque[], ForceMatteForce* fmf);
static void transformMotion(MotionSequence* newMotion, MotionSequence* oldMotion,
                            ModelStruct* model, ForceMatteForce forces[], int numForceMattes);
static SBoolean columnIsGroundForce(MotionSequence* motion, ModelStruct* model, int index);
static int column_is_marker_data(MotionSequence* motion, int column);


/* -------------------------------------------------------------------------
   add_default_motion_objects - 
---------------------------------------------------------------------------- */
void add_default_motion_objects(ModelStruct* model)
{
   int i, n = sizeof(sDefaultMotionObjects) / sizeof(MotionObject);

   for (i = 0; i < n; i++)
   {
      MotionObject* mo;
      ReturnCode rc;

      int j = model->num_motion_objects++;

      if (model->motion_objects == NULL)
         model->motion_objects = (MotionObject*) simm_malloc(model->num_motion_objects * sizeof(MotionObject));
      else
         model->motion_objects = (MotionObject*) simm_realloc(model->motion_objects,
                                                           model->num_motion_objects * sizeof(MotionObject), &rc);
      if (model->motion_objects == NULL)
      {
         model->num_motion_objects = 0;
         return;
      }
      mo = &model->motion_objects[j];

      memset(mo, 0, sizeof(MotionObject));

      mstrcpy(&mo->name, sDefaultMotionObjects[i].name);
      mstrcpy(&mo->filename, sDefaultMotionObjects[i].filename);
      rc = lookup_polyhedron(&mo->shape, mo->filename, model);
      mstrcpy(&mo->materialname, sDefaultMotionObjects[i].materialname);
      mo->material = enter_material(model, mo->materialname, declaring_element);
      mo->drawmode = sDefaultMotionObjects[i].drawmode;
      mo->vector_axis = sDefaultMotionObjects[i].vector_axis;

      for (j = 0; j < 3; j++)
      {
         mo->startingPosition[j]    = sDefaultMotionObjects[i].startingPosition[j];
         mo->startingScale[j]       = sDefaultMotionObjects[i].startingScale[j];
         mo->startingXYZRotation[j] = sDefaultMotionObjects[i].startingXYZRotation[j];
      }
   }
}


int get_motion_number(ModelStruct* model, MotionSequence* motion)
{
   int i;

   if (model && motion)
   {
      for (i = 0; i < model->motion_array_size; i++)
      {
         if (model->motion[i] == motion)
            return i;
      }
   }

   return -1;
}


MotionSequence* get_motion_by_name(ModelStruct* model, const char name[])
{
   if (model)
   {
        int i;

      for (i=0; i<model->motion_array_size; i++)
      {
         if (model->motion[i] && STRINGS_ARE_EQUAL(model->motion[i]->name, name))
            return model->motion[i];
      }
   }

   return NULL;
}


int get_motion_frame_number(MotionSequence* motion, double value)
{
   double percent;
   int frame;

   percent = (motion->mopt.current_value - motion->min_value) /
      (motion->max_value - motion->min_value);

   frame = percent * (motion->number_of_datarows - 1) + MOTION_OFFSET;

   return frame;
}

SBoolean add_motion_to_model(MotionSequence* motion, ModelStruct* model, SBoolean showTopLevelMessages)
{
   int i, j, name_len, num_others, motion_num;
   char otherNames[OTHER_NAMES_SIZE];
   IntBox bbox;
   Form* form;
   ReturnCode rc = code_fine, rc2 = code_fine;
   SliderArray* sa;

   motion->mopt.current_value = motion->min_value;
   motion->mopt.current_frame = 0;

   motion->mopt.gencoords = (double**)simm_malloc(model->numgencoords * sizeof(double*));
   motion->mopt.genc_velocities = (double**)simm_malloc(model->numgencoords * sizeof(double*));
   motion->mopt.muscles = (double**)simm_malloc(model->nummuscles * sizeof(double*));
   motion->mopt.ligaments = (double**)simm_malloc(model->numligaments * sizeof(double*));
   if ((motion->mopt.gencoords == NULL && model->numgencoords > 0) ||
      (motion->mopt.muscles == NULL && model->nummuscles > 0) ||
      (motion->mopt.ligaments == NULL && model->numligaments > 0))
   {
      error(none,"Cannot link motion to model.");
      return no;
   }

   NULLIFY_STRING(otherNames);

   if (link_model_to_motion(model, motion, &num_others, otherNames) == no)
   {
      FREE_IFNOTNULL(motion->mopt.gencoords);
      FREE_IFNOTNULL(motion->mopt.genc_velocities);
      FREE_IFNOTNULL(motion->mopt.muscles);
      FREE_IFNOTNULL(motion->mopt.ligaments);
      FREE_IFNOTNULL(motion->mopt.other_data);

      if (motion->mopt.motion_object_instance)
      {
         for (j = 0; j < motion->mopt.num_motion_object_instances; j++)
            free_motion_object_instance(motion->mopt.motion_object_instance[j], model);
      }
      FREE_IFNOTNULL(motion->mopt.motion_object_instance);
      motion->mopt.num_motion_object_instances = 0;

      return no;
   }

   /* Make sure the motion has a unique name */
   for (i = 0; i < model->motion_array_size; i++)
   {
      if (model->motion[i] && model->motion[i] != motion &&
          STRINGS_ARE_EQUAL(model->motion[i]->name, motion->name))
         break;
   }
   if (i < model->motion_array_size)
   {
      (void)sprintf(buffer,"%s (%d)", motion->name, get_motion_number(model, motion) + 1);
      FREE_IFNOTNULL(motion->name);
      mstrcpy(&motion->name, buffer);
   }

   // Sort the events by x_coord.
   sort_events(motion, NULL);

   motion_num = get_motion_number(model, motion);
#if ! OPENSMAC && ! ENGINE
   make_marker_trails(model, motion);
    make_force_trails(model, motion);

   make_motion_curve_menu(model, motion, motion_num);

   // Before adding the first motion, remove the "none loaded" entry.
   if (model->num_motions == 1)
   {
      glueRemoveMenuItem(model->motionplotmenu, 1);
      glueRemoveMenuItem(model->motionmenu, 1);
   }
   glueAddSubmenuEntry(model->motionplotmenu, motion->name, motion->mopt.curvemenu);
   glueAddMenuEntryWithValue(model->motionmenu, motion->name, motion_num);
   glueAddMenuEntryWithValue(model->motionwithrealtimemenu, motion->name, motion_num);

   /* Add a field to the gencoord form */
   form = &model->gencform;
   form->option = (FormItem*)simm_realloc(form->option, (form->numoptions+1)*sizeof(FormItem), &rc);
   if (rc == code_bad)
   {
      error(none,"Cannot link motion to the model.");
      return no;
   }

   mstrcpy(&form->option[form->numoptions].name, motion->name);
   SET_BOX1221(form->option[form->numoptions].box,0,75,
      -FORM_FIELD_YSPACING*form->numoptions,
      form->option[form->numoptions].box.y2-FORM_FIELD_HEIGHT);
   storeDoubleInForm(&form->option[form->numoptions], motion->mopt.current_value, 3);
   form->option[form->numoptions].justify = yes;
   form->option[form->numoptions].active = yes;
   form->option[form->numoptions].visible = yes;
   form->option[form->numoptions].editable = yes;
   form->option[form->numoptions].use_alternate_colors = no;
   form->option[form->numoptions].decimal_places = 3;
   form->option[form->numoptions].data = motion;
   form->numoptions++;
   name_len = glueGetStringWidth(root.gfont.defaultfont, motion->name);
   if (name_len > model->longest_genc_name)
      model->longest_genc_name = name_len;

   /* Add a slider to the gencoord slider array */
   sa = &model->gencslider;
   sa->sl = (Slider*)simm_realloc(sa->sl,(sa->numsliders+1)*sizeof(Slider),&rc);
   if (rc == code_bad)
   {
      error(none,"Cannot link motion to the model.");
      return no;
   }

   SET_BOX1221(bbox, 0, 170 + 2 * FORM_FIELD_HEIGHT, -FORM_FIELD_YSPACING * sa->numsliders,
               bbox.y2 - FORM_FIELD_HEIGHT);
   make_slider(&sa->sl[sa->numsliders], horizontal_slider, bbox, FORM_FIELD_HEIGHT,
               motion->min_value, motion->min_value, motion->max_value,
               (motion->max_value - motion->min_value)/
               (motion->number_of_datarows - 1), NULL, motion);
   sa->numsliders++;

   reposition_gencoord_sliders(model);

   glueAddMenuEntryWithValue(model->xvarmenu, motion->name, model->numgencoords + motion_num);
   model->dis.devs = (int*)simm_realloc(model->dis.devs, (model->dis.numdevs + 2) * sizeof(int), &rc);
   model->dis.dev_values = (int*)simm_realloc(model->dis.dev_values,    (model->dis.numdevs + 2) * sizeof(int), &rc2);
   if (rc == code_bad || rc2 == code_bad)
   {
      error(none,"Cannot link motion to the model.");
      return no;
   }

   model->dis.devs[model->dis.numdevs++] = motion->keys[0];
   model->dis.devs[model->dis.numdevs++] = motion->keys[1];

   if (showTopLevelMessages == yes)
   {
      (void)sprintf(buffer,"Linked motion %s to model %s (%d \'other\' data columns).",
         motion->name, model->name, num_others);
      message(buffer, 0, DEFAULT_MESSAGE_X_OFFSET);
      if (num_others > 0)
      {
         simm_printf(no, "The following have been loaded as \'otherdata\' in motion %s:\n", motion->name);
         simmPrintMultiLines(otherNames, no, 80, 50);
      }
   }

   make_and_queue_simm_event(MOTION_ADDED, model->modelnum, model->motion[motion_num], NULL, ZERO, ZERO);

#endif /* !OPENSMAC && ! ENGINE */

   return yes;
}

static SBoolean link_model_to_motion(ModelStruct* model, MotionSequence* motion, int* num_other_data, char* otherNames)
{
   int i, j, ref, seg, musc_index, lig_index, cur_len = 1;
   dpFunctionType sType;
   double cutoffFreq;
   SBoolean full = no;
   char* ptr = NULL;
   GeneralizedCoord* gencoord = NULL;

    // In case this function is being called on a motion already added to a model
    // (e.g., after normalizing or smoothing), free the pointers that are about to
    // be re-allocated.
   for (i=0; i<motion->mopt.num_motion_object_instances; )
   {
      if (motion->mopt.motion_object_instance[i]->num_channels > 0)
      {
         free_motion_object_instance(motion->mopt.motion_object_instance[i], model);
         for (j=i; j<motion->mopt.num_motion_object_instances - 1; j++)
            motion->mopt.motion_object_instance[j] = motion->mopt.motion_object_instance[j+1];
         motion->mopt.num_motion_object_instances--;
      }
      else
      {
         i++;
      }
   }
   if (motion->mopt.num_motion_object_instances == 0)
      FREE_IFNOTNULL(motion->mopt.motion_object_instance);

    FREE_IFNOTNULL(motion->mopt.other_data);
    *num_other_data = 0;

   for (i = 0; i < model->numgencoords; i++)
   {
      motion->mopt.gencoords[i] = NULL;
      motion->mopt.genc_velocities[i] = NULL;
   }

   for (i = 0; i < model->nummuscles; i++)
      motion->mopt.muscles[i] = NULL;

   for (i = 0; i < model->numligaments; i++)
      motion->mopt.ligaments[i] = NULL;

   // Since you don't yet know how many columns are other data, make room
   // for the maximum number, and realloc later.
   motion->mopt.other_data = (double**)simm_malloc(motion->number_of_datacolumns*sizeof(double*));
   if (motion->mopt.other_data == NULL)
   {
      error(none,"Cannot link motion to the model.");
      return no;
   }

   for (i = 0; i < motion->number_of_datacolumns; i++)
      motion->mopt.other_data[i] = NULL;

   for (i = 0; i < motion->number_of_datacolumns; i++)
   {
      int motion_object;
      double cutoffFreq;
      dpFunctionType sType;

      if ((gencoord = name_is_gencoord(motion->columnname[i], model, NULL, &sType, &cutoffFreq, yes)))
         motion->mopt.gencoords[getGencoordIndex(model, gencoord)] = motion->motiondata[i];
      else if ((gencoord = name_is_gencoord(motion->columnname[i], model, "_vel", &sType, &cutoffFreq, yes)))
         motion->mopt.genc_velocities[getGencoordIndex(model, gencoord)] = motion->motiondata[i];
      else if ((musc_index = name_is_muscle(model, motion->columnname[i], NULL, &sType, &cutoffFreq, yes)) >= 0)
         motion->mopt.muscles[musc_index] = motion->motiondata[i];
      else if ((lig_index = getLigamentIndex(model, motion->columnname[i])) >= 0)
         motion->mopt.ligaments[lig_index] = motion->motiondata[i];
      else if ((seg = name_is_marker(model, motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, yes)) >= 0 && motion_object >= 0 && ref >= 0)
      {
         MotionObjectInstance* mi = store_motion_object_instance(model, &motion->mopt, seg, motion_object, ref, i);
         mi->type = MarkerMotionObject;
         if (mi->name == NULL)
         {
            char* p = NULL;
            strcpy(buffer, motion->columnname[i]);
            p = strrchr(buffer, '_');
            if (p)
               *p = STRING_TERMINATOR;
            mstrcpy(&mi->name, buffer);
         }
      }
      else if ((seg = name_is_forceplate(model, motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, yes)) >= 0 && motion_object >= 0 && ref >= 0)
      {
         MotionObjectInstance* mi = store_motion_object_instance(model, &motion->mopt, seg, motion_object, ref, i);
         mi->type = ForceMotionObject;
         if (mi->name == NULL)
         {
            char* p = NULL;
            strcpy(buffer, motion->columnname[i]);
            p = strrchr(buffer, '_');
            if (p)
               *p = STRING_TERMINATOR;
            mstrcpy(&mi->name, buffer);
         }
      }
      else if ((seg = name_is_body_segment(model, motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, yes)) >= 0 && motion_object >= 0 && ref >= 0)
      {
         MotionObjectInstance* mi = store_motion_object_instance(model, &motion->mopt, seg, motion_object, ref, i);
         mi->type = ForceMotionObject;
         if (mi->name == NULL)
         {
            char* p = NULL;
            strcpy(buffer, motion->columnname[i]);
            p = strrchr(buffer, '_');
            if (p)
               *p = STRING_TERMINATOR;
            mstrcpy(&mi->name, buffer);
         }
      }
      else
      {
         motion->mopt.other_data[(*num_other_data)++] = motion->motiondata[i];
         // Build up a character string list of 'otherdata' names so you can
         // inform the user which ones you found.
         if (otherNames)
            addNameToString(motion->columnname[i], otherNames, OTHER_NAMES_SIZE);
      }
   }

   return yes;
}


/* -------------------------------------------------------------------------
   name_is_marker - parse the specified 'name' to
      discover its encoded marker name and animation component. If there is
      a match, use the "ball" motion object and return "ground" as the segment
      because marker data in a motion is always w.r.t. ground.
---------------------------------------------------------------------------- */
int name_is_marker(ModelStruct* model, char name[], int* motion_object, int* component,
                   dpFunctionType* functionType, double* cutoffFrequency, SBoolean stripEnd)
{
   int i, j, seg, maxLen, Mlen = 0, len = strlen(name);
   SBoolean foundOne;
   char *ptr, *newEnd;

   /* First check to see if the string begins with the name of a marker.
    * To handle models which have overlapping marker names, like
    * "L.Knee" and "L.Knee.Med", check for a match with all markers, and take
    * the longest match.
    */
   for (i = 0, foundOne = no, maxLen = -1; i < model->numsegments; i++)
   {
      for (j = 0; j < model->segment[i].numMarkers; j++)
      {
         Mlen = strlen(model->segment[i].marker[j]->name);
         if (len >= Mlen)
         {
            if (strings_equal_n_case_insensitive(name, model->segment[i].marker[j]->name, Mlen))
            {
               if (Mlen > maxLen)
               {
                  foundOne = yes;
                  maxLen = Mlen;
               }
            }
         }
      }
   }

   if (foundOne == no)
      return -1;

   /* Move ptr past the marker name. */
   ptr = &name[maxLen];
   len -= maxLen;

   /* The name is of a marker, so assume a segment of "ground" and then
    * look for the component name.
    */
   seg = model->ground_segment;

   /* The motion object is the pre-defined "ball" object. */
   if (motion_object)
   {
      *motion_object = -1;

      for (i = 0; i < model->num_motion_objects; i++)
      {
         if (STRINGS_ARE_EQUAL(model->motion_objects[i].name, "ball"))
         {
            *motion_object = i;
            break;
         }
      }
      if (i == model->num_motion_objects)
         return -1;
   }

   /* Now look for the component. */
   if (component)
   {
      *component = -1;

      /* determine the motion component from the next part of the name. */
      if (!strncmp(ptr, "_tx", 3) || !strncmp(ptr, "_px", 3))
         *component = MO_TX;
      else if (!strncmp(ptr, "_ty", 3) || !strncmp(ptr, "_py", 3))
         *component = MO_TY;
      else if (!strncmp(ptr, "_tz", 3) || !strncmp(ptr, "_pz", 3))
         *component = MO_TZ;
      else if (!strncmp(ptr, "_vx", 3))
         *component = MO_VX;
      else if (!strncmp(ptr, "_vy", 3))
         *component = MO_VY;
      else if (!strncmp(ptr, "_vz", 3))
         *component = MO_VZ;
      else if (!strncmp(ptr, "_sx", 3))
         *component = MO_SX;
      else if (!strncmp(ptr, "_sy", 3))
         *component = MO_SY;
      else if (!strncmp(ptr, "_sz", 3))
         *component = MO_SZ;
      else if (!strncmp(ptr, "_cr", 3))
         *component = MO_CR;
      else if (!strncmp(ptr, "_cg", 3))
         *component = MO_CG;
      else if (!strncmp(ptr, "_cb", 3))
         *component = MO_CB;
      else
         return -1;
   }
   else
   {
      return -1;
   }

   /* Move ptr past the component name. */
   ptr += 3;
   len -= 3;

   /* Store a pointer to the character right after the component.
    * This will be the new end of the string if stripEnd == yes.
    */
   newEnd = ptr;

   /* If functionType and cutoffFrequency are not NULL, check to see if the name ends in
    * natural_cubic_text or gcv_text (followed by an optional cutoff frequency). If it
    * does, set *functionType to the appropriate type. If no function label is found, set
    * the type to step_func.
    */
   if (functionType && cutoffFrequency)
   {
      int matched_spl = 0;
      int spl_len = strlen(natural_cubic_text);
      int gcv_len = strlen(gcv_text);

      *functionType = dpStepFunction;
      *cutoffFrequency = -1.0; // by default there is no smoothing

      if (len >= spl_len)
      {
         if (!strncmp(ptr, natural_cubic_text, spl_len))
         {
            *functionType = dpNaturalCubicSpline;
            ptr += spl_len;
            len -= spl_len;
            matched_spl = 1;
         }
      }

      if (!matched_spl && len >= gcv_len)
      {
         if (!strncmp(ptr, gcv_text, gcv_len))
         {
            ptr += gcv_len;
            len -= gcv_len;
            *functionType = dpGCVSpline;
            if (len > 0)
            {
               char* intPtr = buffer;

               /* Move over the underscore and look for an integer. */
               if (*(ptr++) != '_')
               {
                  return -1;
               }
               else
               {
                  len--; /* for the underscore character */
                  while (*ptr >= '0' && *ptr <= '9')
                  {
                     *(intPtr++) = *(ptr++);
                     len--;
                  }
                  *intPtr = STRING_TERMINATOR;
                  *cutoffFrequency = atof(buffer);
               }
            }
         }
      }
   }

   /* If there are extra characters after the suffixes, return an error. */
   if (len > 0)
      return -1;

   /* Strip off the text for the function type and cutoff frequency. */
   if (stripEnd == yes)
      *newEnd = STRING_TERMINATOR;

   return seg;

}


/* -------------------------------------------------------------------------
   name_is_forceplate - parse the specified 'name' to
      discover its encoded forceplate name and animation component. If there is
      a match, use the "force" motion object and return "ground" as the segment
      because forceplate data in a motion is always w.r.t. ground.
---------------------------------------------------------------------------- */
int name_is_forceplate(ModelStruct* model, char name[], int* motion_object, int* component,
                       dpFunctionType* functionType, double* cutoffFrequency, SBoolean stripEnd)
{
   int i, j, seg, plateNum, Flen = 0, len = strlen(name);
   char *ptr, *newEnd;

   /* If the column name starts with "forceplate" and a number, then
    * it is forceplate data.
    */
   if (sscanf(name, "forceplate%d", &plateNum) != 1)
      return -1;

   /* The name is of a forceplate, so assume a segment of "ground" and then
    * look for the component name.
    */
   seg = model->ground_segment;

   sprintf(buffer, "forceplate%d", plateNum);
   Flen = strlen(buffer);

   /* Move ptr past the forceplate name and number. */
   ptr = &name[Flen];
   len -= Flen;

   /* The motion object is the pre-defined "force" object. */
   if (motion_object)
   {
      *motion_object = -1;

      for (i = 0; i < model->num_motion_objects; i++)
      {
         if (STRINGS_ARE_EQUAL(model->motion_objects[i].name, "force"))
         {
            *motion_object = i;
            break;
         }
      }
      if (i == model->num_motion_objects)
         return -1;
   }

   /* Now look for the component. */
   if (component)
   {
      *component = -1;

      /* determine the motion component from the next part of the name. */
      if (!strncmp(ptr, "_tx", 3) || !strncmp(ptr, "_px", 3))
         *component = MO_TX;
      else if (!strncmp(ptr, "_ty", 3) || !strncmp(ptr, "_py", 3))
         *component = MO_TY;
      else if (!strncmp(ptr, "_tz", 3) || !strncmp(ptr, "_pz", 3))
         *component = MO_TZ;
      else if (!strncmp(ptr, "_vx", 3))
         *component = MO_VX;
      else if (!strncmp(ptr, "_vy", 3))
         *component = MO_VY;
      else if (!strncmp(ptr, "_vz", 3))
         *component = MO_VZ;
      else if (!strncmp(ptr, "_sx", 3))
         *component = MO_SX;
      else if (!strncmp(ptr, "_sy", 3))
         *component = MO_SY;
      else if (!strncmp(ptr, "_sz", 3))
         *component = MO_SZ;
      else if (!strncmp(ptr, "_cr", 3))
         *component = MO_CR;
      else if (!strncmp(ptr, "_cg", 3))
         *component = MO_CG;
      else if (!strncmp(ptr, "_cb", 3))
         *component = MO_CB;
      else if (!strncmp(ptr, "_torque_x", 9)) // free torque
         *component = MO_X;
      else if (!strncmp(ptr, "_torque_y", 9)) // free torque
         *component = MO_Y;
      else if (!strncmp(ptr, "_torque_z", 9)) // free torque
         *component = MO_Z;
      else
         return -1;
   }
   else
   {
      return -1;
   }

   // Move ptr past the component name.
   if (*component == MO_X || *component == MO_Y || *component == MO_Z)
   {
      ptr += 9;
      len -= 9;
   }
   else
   {
      ptr += 3;
      len -= 3;
   }

   /* Store a pointer to the character right after the component.
    * This will be the new end of the string if stripEnd == yes.
    */
   newEnd = ptr;

   /* If functionType and cutoffFrequency are not NULL, check to see if the name ends in
    * natural_cubic_text or gcv_text (followed by an optional cutoff frequency). If it
    * does, set *functionType to the appropriate type. If no function label is found, set
    * the type to step_func.
    */
   if (functionType && cutoffFrequency)
   {
      int matched_spl = 0;
      int spl_len = strlen(natural_cubic_text);
      int gcv_len = strlen(gcv_text);

      *functionType = dpStepFunction;
      *cutoffFrequency = -1.0; // by default there is no smoothing

      if (len >= spl_len)
      {
         if (!strncmp(ptr, natural_cubic_text, spl_len))
         {
            *functionType = dpNaturalCubicSpline;
            ptr += spl_len;
            len -= spl_len;
            matched_spl = 1;
         }
      }

      if (!matched_spl && len >= gcv_len)
      {
         if (!strncmp(ptr, gcv_text, gcv_len))
         {
            ptr += gcv_len;
            len -= gcv_len;
            *functionType = dpGCVSpline;
            if (len > 0)
            {
               char* intPtr = buffer;

               /* Move over the underscore and look for an integer. */
               if (*(ptr++) != '_')
               {
                  return -1;
               }
               else
               {
                  len--; /* for the underscore character */
                  while (*ptr >= '0' && *ptr <= '9')
                  {
                     *(intPtr++) = *(ptr++);
                     len--;
                  }
                  *intPtr = STRING_TERMINATOR;
                  *cutoffFrequency = atof(buffer);
               }
            }
         }
      }
   }

   /* If there are extra characters after the suffixes, return an error. */
   if (len > 0)
      return -1;

   /* Strip off the text for the function type and cutoff frequency. */
   if (stripEnd == yes)
      *newEnd = STRING_TERMINATOR;

   return seg;

}


/* -------------------------------------------------------------------------
   motion_object_instance_contains_component - 
---------------------------------------------------------------------------- */
static SBoolean motion_object_instance_contains_component(MotionObjectInstance* mi, int component)
{
   if (mi->channels)
   {
      int i;
      
      for (i = 0; i < mi->num_channels; i++)
         if (mi->channels[i].component == component)
            return yes;
   }
   return no;
}

/* -------------------------------------------------------------------------
   store_motion_object_instance - 
---------------------------------------------------------------------------- */
static MotionObjectInstance* store_motion_object_instance(ModelStruct* model,
   MotionModelOptions* mopt, int seg, int motion_object, int motion_component, int column)
{
   ReturnCode rc;
   int i;
   MotionObjectInstance* mi = NULL;

   // Scan the motion's object instance array to see if we need to create
   // a new motion object instance, or use an existing one. There could be
   // multiple incomplete instances, in which case you want to add the new
   // column to the first one that is missing this component.
   for (i=0; i<mopt->num_motion_object_instances; i++)
   {
      if (mopt->motion_object_instance[i]->segment == seg &&
         mopt->motion_object_instance[i]->object == motion_object &&
         motion_object_instance_contains_component(mopt->motion_object_instance[i], motion_component) == no)
      {
         // You found an existing motion object instance that does not yet have
         // this component defined. 'i' is now the index of this instance.
         break;
      }
   }

   if (i == mopt->num_motion_object_instances)
   {
      // Add a new element to the array of motion object instances.
      i = mopt->num_motion_object_instances++;

      if (mopt->motion_object_instance == NULL)
      {
         mopt->motion_object_instance = (MotionObjectInstance**)
            simm_malloc(mopt->num_motion_object_instances * sizeof(MotionObjectInstance*));
      }
      else
      {
         mopt->motion_object_instance = (MotionObjectInstance**)
            simm_realloc(mopt->motion_object_instance,
            mopt->num_motion_object_instances * sizeof(MotionObjectInstance*), &rc);
      }

      // Initialize a new motion object instance.
      if (mopt->motion_object_instance)
      {
         MotionObject* mo = &model->motion_objects[motion_object];

         mi = mopt->motion_object_instance[i] = (MotionObjectInstance*)simm_calloc(1, sizeof(MotionObjectInstance));
         
         mi->type          = UnknownMotionObject;
         mi->object        = motion_object;
         mi->segment       = seg;
         mi->drawmode      = mo->drawmode;
         mi->current_value = -1.0;
         mi->visible       = yes;

         mi->currentMaterial.name = NULL;
         copy_material(&model->dis.mat.materials[mo->material], &mi->currentMaterial);
      }
      else
         mopt->num_motion_object_instances--;
   }
   else
      mi = mopt->motion_object_instance[i];

   // Add a new animation channel to the motion object instance.
   if (mi)
   {
      i = mi->num_channels++;

      if (mi->channels == NULL)
      {
         mi->channels = (MotionObjectChannel*)
            simm_malloc(mi->num_channels * sizeof(MotionObjectChannel));
      }
      else
      {
         mi->channels = (MotionObjectChannel*)
            simm_realloc(mi->channels, mi->num_channels * sizeof(MotionObjectChannel), &rc);
      }

      if (mi->channels)
      {
         mi->channels[i].component = motion_component;
         mi->channels[i].column = column;
      }
   }

   return mi;
}


double check_motion_wrapping(ModelStruct* model, MotionSequence* motion, double change)
{
   double new_value, range;

   if (model == NULL || motion == NULL)
      return 0.0;

   new_value = motion->mopt.current_value + change;

   if (motion->wrap == no)
      return new_value;

   range = motion->max_value - motion->min_value;

   while (new_value > motion->max_value)
      new_value -= range;

   while (new_value < motion->min_value)
      new_value += range;

   return new_value;
}


/* -------------------------------------------------------------------------
   circularize_motion_index - this interesting routine was added to allow
      realtime motion to continuously stream into a fixed-duration
      MotionSequence, by wrapping the samples around the end of the buffer.
      
      -- KMS 2/23/00
---------------------------------------------------------------------------- */
static int circularize_motion_index(MotionSequence* motion, int i)
{
   if (motion->is_realtime && motion->realtime_circular_index > 0)
   {
      i += motion->realtime_circular_index;
      
      if (i >= motion->number_of_datarows)
         i -= motion->number_of_datarows;
   }

   return i;
}

void free_motion(MotionSequence* motion, ModelStruct* model)
{
   if (motion)
   {
      int i;

      FREE_IFNOTNULL(motion->name);
      FREE_IFNOTNULL(motion->units);

      if (motion->motiondata)
      {
         for (i = 0; i < motion->number_of_datacolumns; i++)
            FREE_IFNOTNULL(motion->motiondata[i]);
         FREE_IFNOTNULL(motion->motiondata);
      }

      if (motion->columnname)
      {
         for (i = 0; i < motion->number_of_datacolumns; i++)
            FREE_IFNOTNULL(motion->columnname[i]);
         FREE_IFNOTNULL(motion->columnname);
      }

      if (motion->deriv_data)
      {
         for (i = 0; i < motion->number_of_datacolumns; i++)
            FREE_IFNOTNULL(motion->deriv_data[i]);
         FREE_IFNOTNULL(motion->deriv_data);
      }

      if (motion->deriv_names)
      {
         for (i = 0; i < motion->number_of_datacolumns; i++)
            FREE_IFNOTNULL(motion->deriv_names[i]);
         FREE_IFNOTNULL(motion->deriv_names);
      }

      if (motion->data_std_dev)
      {
         for (i = 0; i < motion->number_of_datacolumns; i++)
            FREE_IFNOTNULL(motion->data_std_dev[i]);
         FREE_IFNOTNULL(motion->data_std_dev);
      }

      if (motion->event)
      {
         for (i = 0; i < motion->num_events; i++)
            FREE_IFNOTNULL(motion->event[i].name);
         FREE_IFNOTNULL(motion->event);
      }

        FREE_IFNOTNULL(motion->mopt.gencoords);
        FREE_IFNOTNULL(motion->mopt.genc_velocities);
        FREE_IFNOTNULL(motion->mopt.muscles);
        FREE_IFNOTNULL(motion->mopt.ligaments);
        FREE_IFNOTNULL(motion->mopt.other_data);

      if (motion->mopt.motion_object_instance)
      {
         for (i = 0; i < motion->mopt.num_motion_object_instances; i++)
            free_motion_object_instance(motion->mopt.motion_object_instance[i], model);
         FREE_IFNOTNULL(motion->mopt.motion_object_instance);
      }
   }
}

int apply_motion_to_model(ModelStruct* model, MotionSequence* motion, double value, SBoolean update_modelview, SBoolean draw_plot)
{
   int i, item, motion_num;
   double percent;

   if (!motion)
      return 0;

   motion->mopt.current_value = value;

   if (motion->mopt.current_value < motion->min_value)
      motion->mopt.current_value  = motion->min_value;
   else if (motion->mopt.current_value > motion->max_value)
      motion->mopt.current_value  = motion->max_value;

   if (EQUAL_WITHIN_ERROR(motion->max_value, motion->min_value) || motion->number_of_datarows == 1)
   {
      motion->mopt.current_value = motion->min_value;
      motion->mopt.current_frame = 0;
   }
   else
   {
#if 0 // can't use this method with variable spacing in the X column
      percent = (motion->mopt.current_value - motion->min_value) / (motion->max_value - motion->min_value);

      motion->mopt.current_frame = percent * (motion->number_of_datarows - 1) + MOTION_OFFSET;

      motion->mopt.current_value = motion->min_value + motion->mopt.current_frame *
         (motion->max_value - motion->min_value) / (motion->number_of_datarows - 1);

      //printf("percent = %lf, frame = %d, value = %lf\n", percent, motion->mopt.current_frame, motion->mopt.current_value);
#else
      update_current_motion_frame(motion);
      //printf("frame = %d, value = %lf, frameValue = %lf\n", motion->mopt.current_frame, motion->mopt.current_value, motion->motiondata[motion->x_column][motion->mopt.current_frame]);
#endif
   }

   motion->mopt.current_frame = circularize_motion_index(motion, motion->mopt.current_frame);

   if ((model->numclosedloops > 0) && (model->useIK == yes) && motion->enforce_loops == yes)
   {
      LoopStatus loopStatus;
      ConstraintStatus consStatus;
      /* if the gencoord is in the motion file, set it to the value in the 
       * motion file.  If it's not in the motion file and is in a loop, set it
       * to zero.  Then invalidate all the joint matrices, and solve all the
       * loops.  Gencoords get set to whatever new values are calculated by
       * the IK solver (may not be the same as those in the motion file.
       */
      for (i = 0; i < model->numgencoords; i++)
      {
         int j;
         if (motion->mopt.gencoords[i] != NULL)
            model->gencoord[i]->value = motion->mopt.gencoords[i][motion->mopt.current_frame];
         else if (model->gencoord[i]->used_in_loop == yes)
            model->gencoord[i]->value = 0.0;
         for (j = 0; j < model->gencoord[i]->numjoints; j++)
            invalidate_joint_matrix(model, &model->joint[model->gencoord[i]->jointnum[j]]);
      }
      solveAllLoopsAndConstraints(model, &loopStatus, &consStatus, yes);
      if (loopStatus == loopChanged)
      {
#if 0
         sprintf(buffer, "Loops solved using motion file values, values changed\n");
         error(none, buffer);
#endif
      }
   }
   else if (model->num_constraint_objects > 0)// && motion->enforce_constraints == yes)
   {
      ConstraintStatus constraintStatus;
      LoopStatus loopStatus;
      /* if the gencoord is in the motion file, copy its value into the gc struct.
       * then invalidate all the joints that use it.
       * Solve all the constraints with the new values.  Gencoords are set
       * to whatever values are calculated by the constraint solver (may not
       * be the values in the motion file. Don't change any values if the
       * gencoord is locked.  If the constraints are unchanged, the gencoord
       * values must be set (set_gencoord_value was not called in solveall. 
       */
      for (i = 0; i < model->numgencoords; i++)
      {
         int j;
         if ((motion->mopt.gencoords[i] != NULL) &&
            (model->gencoord[i]->locked == no))
         {
            model->gencoord[i]->value = motion->mopt.gencoords[i][motion->mopt.current_frame];
            for (j = 0; j < model->gencoord[i]->numjoints; j++)
               invalidate_joint_matrix(model, &model->joint[model->gencoord[i]->jointnum[j]]);
         }
      }

      solveAllLoopsAndConstraints(model, &loopStatus, &constraintStatus, motion->enforce_constraints);//yes);
      if (constraintStatus == constraintUnchanged)
      {
         for (i = 0; i < model->numgencoords; i++)
         {
            set_gencoord_value(model, model->gencoord[i], model->gencoord[i]->value, no);
         }
      }
      else if (constraintStatus == constraintChanged)
      {
#if 0
         sprintf(buffer, "Constraints solved using motion file values, values changed\n");
         error(none, buffer);
#endif
      }
      //dkb aug 19, 2002
      for (i = 0; i < model->numgencoords; i++)
      {
         set_gencoord_value(model, model->gencoord[i], model->gencoord[i]->value, no);
      }
   }
   else
   {
      /* if solver is off or there are no loops, and there are no constraints,
       * set the gencoords to the values in the motion file */
      for (i = 0; i < model->numgencoords; i++)
      {
         if (motion->mopt.gencoords[i] != NULL)
         {
            if (model->gencoord[i]->locked == no)
            {
               set_gencoord_value(model, model->gencoord[i], motion->mopt.gencoords[i][motion->mopt.current_frame], no);
            }
            else if (update_modelview == yes)
            {
               /* If the gencoord is locked, but still appears in the motion file,
                * print a warning message if the motion file value is not within
                * a tolerance equal to the default precision of motion file data.
                */
               if (NOT_EQUAL_WITHIN_TOLERANCE(motion->mopt.gencoords[i][motion->mopt.current_frame], model->gencoord[i]->value, 0.000001))
               {
                  sprintf(errorbuffer, "Gencoord %s is locked at %f\n",
                  model->gencoord[i]->name, model->gencoord[i]->value);
                  error(none, errorbuffer);
               }
            }
         }
      }
   }

#if ! OPENSMAC && ! ENGINE
   for (i=0; i<model->nummuscles; i++)
   {
      if (motion->mopt.muscles[i] != NULL)
         model->muscle[i]->dynamic_activation = motion->mopt.muscles[i][motion->mopt.current_frame];
   }  

   for (i=0; i<model->numligaments; i++)
   {
      if (motion->mopt.ligaments[i] != NULL)
         model->ligament[i].activation = motion->mopt.ligaments[i][motion->mopt.current_frame];
   }

   for (i=0; i<model->numgencoords; i++)
   {
      if (motion->mopt.genc_velocities[i] != NULL)
         set_gencoord_velocity(model, model->gencoord[i], motion->mopt.genc_velocities[i][motion->mopt.current_frame]);
      else
         set_gencoord_velocity(model, model->gencoord[i], 0.0);
   }

   if (update_modelview == yes)
   {
      /* Find out which form item is linked to this motion and update it. */
      for (item = 0; item < model->gencform.numoptions; item++)
      {
         if (model->gencform.option[item].data == motion)
         {
            storeDoubleInForm(&model->gencform.option[item], motion->mopt.current_value, 3);
            break;
         }
      }

      /* Find out which slider is linked to this motion and update it. */
      for (item = 0; item < model->gencform.numoptions; item++)
      {
         if (model->gencform.option[item].data == motion)
         {
            model->gencslider.sl[item].value = motion->mopt.current_value;
            break;
         }
      }

      model->dis.applied_motion = motion;
   }

   if (draw_plot == yes && motion->show_cursor == yes)
      set_plot_cursors(model, motion);
#endif

   /* If there is no gencoord data in this motion (all columns of
    * data in motion file are "othercurves," then return 0, meaning you didn't
    * change the model configuration by trying to apply the motion to it. Otherwise
    * return 1, so the calling routine knows that it should redraw the model.
    * JPL/TODO 7/24/96: This doesn't work since standard deviations have been
    * added to motion files, because num_datacolumns  gets decremented by num_std_dev,
    * but num_other does not (so they may happen to be equal)

   if (motion->number_of_datacolumns == motion->number_of_othercurves)
      return 0;
   else
    */

   return 1;
}

static void update_current_motion_frame(MotionSequence* motion)
{
   int i;

   // Current_value has changed. For efficiency, first check to see if it is still
   // near current_frame.
   if (motion->mopt.current_frame == 0)
   {
      // See if current value is between frames 0 and 1.
      // If it is, figure out which one it is closer to.
      double leftDiff = motion->mopt.current_value - motion->motiondata[motion->x_column][0] - TINY_NUMBER;
      double rightDiff = motion->motiondata[motion->x_column][1] + TINY_NUMBER - motion->mopt.current_value;

      if (leftDiff >= 0.0 && rightDiff >= 0.0)
      {
         if (leftDiff < rightDiff)
            motion->mopt.current_frame = 0;
         else
            motion->mopt.current_frame = 1;
         return;
      }
   }
   else if (motion->mopt.current_frame == motion->number_of_datarows - 1)
   {
      // See if current value is between last and second-to-last frames.
      // If it is, figure out which one it is closer to.
      double leftDiff = motion->mopt.current_value - motion->motiondata[motion->x_column][motion->number_of_datarows - 2] - TINY_NUMBER;
      double rightDiff = motion->motiondata[motion->x_column][motion->number_of_datarows - 1] + TINY_NUMBER - motion->mopt.current_value;

      if (leftDiff >= 0.0 && rightDiff >= 0.0)
      {
         if (leftDiff < rightDiff)
            motion->mopt.current_frame = motion->number_of_datarows - 2;
         else
            motion->mopt.current_frame = motion->number_of_datarows - 1;
         return;
      }
   }
   else
   {
      // See if current value is between current frame and previous frame.
      // If it is, figure out which one it is closer to.
      int frame = motion->mopt.current_frame;
      double leftDiff = motion->mopt.current_value - motion->motiondata[motion->x_column][frame - 1] - TINY_NUMBER;
      double rightDiff = motion->motiondata[motion->x_column][frame] + TINY_NUMBER - motion->mopt.current_value;

      if (leftDiff >= 0.0 && rightDiff >= 0.0)
      {
         if (leftDiff < rightDiff)
            motion->mopt.current_frame = frame - 1;
         else
            motion->mopt.current_frame = frame;
         return;
      }

      // See if current value is between current frame and next frame.
      // If it is, figure out which one it is closer to.
      leftDiff = motion->mopt.current_value - motion->motiondata[motion->x_column][frame] - TINY_NUMBER;
      rightDiff = motion->motiondata[motion->x_column][frame + 1] + TINY_NUMBER - motion->mopt.current_value;

      if (leftDiff >= 0.0 && rightDiff >= 0.0)
      {
         if (leftDiff < rightDiff)
            motion->mopt.current_frame = frame;
         else
            motion->mopt.current_frame = frame + 1;
         return;
      }
   }

   // Find the first frame that is greater than current value, then
   // figure out whether current value is closer to that frame or the
   // previous frame.
   for (i=0; i<motion->number_of_datarows; i++)
   {
      if (motion->motiondata[motion->x_column][i] + TINY_NUMBER >= motion->mopt.current_value)
      {
         if (i == 0)
         {
            motion->mopt.current_frame = 0;
            return;
         }
         else
         {
            double leftDiff = motion->mopt.current_value - motion->motiondata[motion->x_column][i - 1] - TINY_NUMBER;
            double rightDiff = motion->motiondata[motion->x_column][i] + TINY_NUMBER - motion->mopt.current_value;

            if (leftDiff >= 0.0 && rightDiff >= 0.0)
            {
               if (leftDiff < rightDiff)
                  motion->mopt.current_frame = i - 1;
               else
                  motion->mopt.current_frame = i;
               return;
            }
         }
      }
   }
}

ReturnCode write_motion(MotionSequence *motion, const char filename[])
{
   int i, j;
   FILE* file = simm_fopen(filename, "w");

   if (file == NULL)
      return code_bad;

   fprintf(file, "/* %s\n */\n\n", filename);
   fprintf(file, "name %s\n", motion->name);
   fprintf(file, "datacolumns %d\n", motion->number_of_datacolumns);
   fprintf(file, "datarows %d\n", motion->number_of_datarows);
   fprintf(file, "x_column %d\n", motion->x_column + 1);
   //fprintf(file, "range %lf %f\n", motion->min_value, motion->max_value);
   if (motion->units)
      fprintf(file, "units %s\n", motion->units);
   if (motion->wrap)
      fprintf(file, "wrap\n");

   if (motion->enforce_loops == yes)
      fprintf(file, "enforce_loops yes\n");
   else
      fprintf(file, "enforce_loops no\n");
   if (motion->enforce_constraints == yes)
      fprintf(file, "enforce_constraints yes\n");
   else
      fprintf(file, "enforce_constraints no\n");

   if (motion->show_cursor)
      fprintf(file, "cursor %.2f %.2f %.2f\n", motion->cursor_color[0], motion->cursor_color[1], motion->cursor_color[2]);

   for (i = 0; i < motion->num_events; i++)
      fprintf(file, "event %lf %s\n", motion->event[i].xCoord, motion->event[i].name);

   fprintf(file, "event_color %f %f %f\n", motion->event_color[0], motion->event_color[1], motion->event_color[2]);

   if (motion->calc_derivatives)
      fprintf(file, "calc_derivatives yes\n");

   for (i=0; i<motion->mopt.num_motion_object_instances; i++)
   {
      MotionObjectInstance* mi = motion->mopt.motion_object_instance[i];

      if (mi->type == FootPrintObject)
      {
         // The scale factor is the magnitude of any row or column in the rotation matrix.
         double eulerRot[3], nvec[3];
            memset(nvec, 0, 3 * sizeof(double));
         extract_xyz_rot_bodyfixed(mi->currentXform, eulerRot);
         fprintf(file, "%s %lf %lf %lf %lf %lf %lf %lf\n", mi->name,
            normalize_vector(mi->currentXform[0], nvec),                            // scale factor
            mi->currentXform[3][0], mi->currentXform[3][1], mi->currentXform[3][2], // translations
                eulerRot[0] * RTOD, eulerRot[1] * RTOD, eulerRot[2] * RTOD);            // XYZ Euler rotations
      }
      else if (mi->type == ForcePlateObject)
      {
         double eulerRot[3], nvec[3];
            memset(nvec, 0, 3 * sizeof(double));
         extract_xyz_rot_bodyfixed(mi->currentXform, eulerRot);
         fprintf(file, "%s", mi->name);
         // Scale factors are the magnitudes of the rows.
            fprintf(file, " %lf %lf %lf", VECTOR_MAGNITUDE(mi->currentXform[0]), VECTOR_MAGNITUDE(mi->currentXform[1]),
                VECTOR_MAGNITUDE(mi->currentXform[2]));
         fprintf(file, " %lf %lf %lf", mi->currentXform[3][0], mi->currentXform[3][1], mi->currentXform[3][2]);
         fprintf(file, " %lf %lf %lf\n", eulerRot[0] * RTOD, eulerRot[1] * RTOD, eulerRot[2] * RTOD);
      }
   }

   fprintf(file, "endheader\n\n");
   
   for (j = 0; j < motion->number_of_datacolumns; j++)
      fprintf(file, "%s\t", motion->columnname[j]);
   fprintf(file, "\n");
   
   for (i = 0; i < motion->number_of_datarows; i++)
   {
      for (j = 0; j < motion->number_of_datacolumns; j++)
         fprintf(file, "%lf\t", motion->motiondata[j][i]);
      
      fprintf(file, "\n");
   }
   fclose(file);

   return code_fine;
}


MotionSequence* createMotionStruct(ModelStruct* model)
{
   int i;
   ReturnCode rc = code_fine;
   MotionSequence* motion;

   /* Find, or make, an empty slot for this new motion. */
   for (i = 0; i < model->motion_array_size; i++)
   {
      if (model->motion[i] == NULL)
         break;
   }

   if (i == model->motion_array_size)
   {
      int j, old_size = model->motion_array_size;
      model->motion_array_size += MOTION_ARRAY_INCREMENT;
      model->motion = (MotionSequence**)simm_realloc(model->motion,
         (unsigned)((model->motion_array_size)*sizeof(MotionSequence*)), &rc);
      if (rc == code_bad)
         return NULL;

      for (j = old_size; j < model->motion_array_size; j++)
         model->motion[j] = NULL;
   }

   model->motion[i] = (MotionSequence*)simm_calloc(1, sizeof(MotionSequence));
   if (model->motion[i] == NULL)
      return NULL;

   motion = model->motion[i];
   model->num_motions++;

   return motion;
}

#if OPENSMAC
MotionSequence* applyForceMattesToMotion(ModelStruct* model, MotionSequence* motionKin, MotionSequence* motionAnalog, SBoolean addToModel)
{
   int i, j, k, numForceMatteForces = 0;
   double value, increment;
   ForceMatteForce forces[10];
   MotionSequence* newMotion = NULL;

   if (motionAnalog->number_of_datarows < 2)
      return newMotion;

   for (i=0; i<model->numsegments; i++)
   {
      if (model->segment[i].forceMatte)
      {
         forces[numForceMatteForces].segment = i;
         forces[numForceMatteForces].forceMatte = model->segment[i].forceMatte;
         numForceMatteForces++;
      }
   }

   if (numForceMatteForces == 0)
      return newMotion;

   if (addToModel == yes)
      newMotion = createMotionStruct(model);
   else
      newMotion = (MotionSequence*)simm_calloc(1, sizeof(MotionSequence));
   transformMotion(newMotion, motionAnalog, model, forces, numForceMatteForces);
   add_motion_to_model(newMotion, model, no);

   increment = (motionAnalog->max_value - motionAnalog->min_value) / (motionAnalog->number_of_datarows - 1);
   for (i=0; i<motionAnalog->number_of_datarows; i++)
   {
      value = motionAnalog->min_value + i * increment;
      apply_motion_to_model(model, motionKin, value, no, no);
      motionAnalog->mopt.current_frame = i;
      for (k=0; k<numForceMatteForces; k++)
      {
         memset(forces[k].pointOfApplication, 0, 3*sizeof(double));
         memset(forces[k].force, 0, 3*sizeof(double));
         memset(forces[k].freeTorque, 0, 3*sizeof(double));
         forces[k].magnitude = 0.0;
      }
      for (j=0; j<motionAnalog->mopt.num_motion_object_instances; j++)
      {
         MotionObjectInstance* moi = motionAnalog->mopt.motion_object_instance[j];
         if (moi->segment == model->ground_segment && STRINGS_ARE_EQUAL(model->motion_objects[moi->object].name, "force"))
         {
            double pt[3], pt2[3], vec[3], vec2[3], inter[3], freeTorque[3];
            for (k=0; k<moi->num_channels; k++)
            {
               if (moi->channels[k].component == MO_TX)
                  pt[0] = motionAnalog->motiondata[moi->channels[k].column][motionAnalog->mopt.current_frame];
               else if (moi->channels[k].component == MO_TY)
                  pt[1] = motionAnalog->motiondata[moi->channels[k].column][motionAnalog->mopt.current_frame];
               else if (moi->channels[k].component == MO_TZ)
                  pt[2] = motionAnalog->motiondata[moi->channels[k].column][motionAnalog->mopt.current_frame];
               else if (moi->channels[k].component == MO_VX)
                  vec[0] = motionAnalog->motiondata[moi->channels[k].column][motionAnalog->mopt.current_frame];
               else if (moi->channels[k].component == MO_VY)
                  vec[1] = motionAnalog->motiondata[moi->channels[k].column][motionAnalog->mopt.current_frame];
               else if (moi->channels[k].component == MO_VZ)
                  vec[2] = motionAnalog->motiondata[moi->channels[k].column][motionAnalog->mopt.current_frame];
               else if (moi->channels[k].component == MO_X)
                  freeTorque[0] = motionAnalog->motiondata[moi->channels[k].column][motionAnalog->mopt.current_frame];
               else if (moi->channels[k].component == MO_Y)
                  freeTorque[1] = motionAnalog->motiondata[moi->channels[k].column][motionAnalog->mopt.current_frame];
               else if (moi->channels[k].component == MO_Z)
                  freeTorque[2] = motionAnalog->motiondata[moi->channels[k].column][motionAnalog->mopt.current_frame];
            }
            if (VECTOR_MAGNITUDE(vec) > 5.0)
            {
               for (k=0; k<numForceMatteForces; k++)
               {
                  memcpy(pt2, pt, 3*sizeof(double));
                  memcpy(vec2, vec, 3*sizeof(double));
                  convert(model, pt2, model->ground_segment, forces[k].segment);
                  convert_vector(model, vec2, model->ground_segment, forces[k].segment);
                  if (vector_intersects_polyhedron(pt2, vec2, forces[k].forceMatte->poly, inter))
                  {
                     add_vector(pt, vec, freeTorque, &forces[k]);
                     break;
                  }
               }
            }
         }
      }
      for (k=0; k<numForceMatteForces; k++)
      {
         newMotion->motiondata[forces[k].column[0]][motionAnalog->mopt.current_frame] = forces[k].force[0];
         newMotion->motiondata[forces[k].column[1]][motionAnalog->mopt.current_frame] = forces[k].force[1];
         newMotion->motiondata[forces[k].column[2]][motionAnalog->mopt.current_frame] = forces[k].force[2];
         newMotion->motiondata[forces[k].column[3]][motionAnalog->mopt.current_frame] = forces[k].pointOfApplication[0];
         newMotion->motiondata[forces[k].column[4]][motionAnalog->mopt.current_frame] = forces[k].pointOfApplication[1];
         newMotion->motiondata[forces[k].column[5]][motionAnalog->mopt.current_frame] = forces[k].pointOfApplication[2];
         newMotion->motiondata[forces[k].column[6]][motionAnalog->mopt.current_frame] = forces[k].freeTorque[0];
         newMotion->motiondata[forces[k].column[7]][motionAnalog->mopt.current_frame] = forces[k].freeTorque[1];
         newMotion->motiondata[forces[k].column[8]][motionAnalog->mopt.current_frame] = forces[k].freeTorque[2];
      }
   }

   return newMotion;
}


/* Add a force vector to one already stored in a ForceMatteForce. */
static void add_vector(double pt[], double vec[], double freeTorque[], ForceMatteForce* fmf)
{
   int i;
   double oldMagnitude = fmf->magnitude;
   double vecMagnitude = VECTOR_MAGNITUDE(vec);
   double newMagnitude = oldMagnitude + vecMagnitude;

   for (i=0; i<3; i++)
   {
      // Add in the force.
      fmf->force[i] += vec[i];
      // The new point of application is the weighted sum of the two vectors.
      fmf->pointOfApplication[i] = (fmf->pointOfApplication[i]*oldMagnitude + pt[i]*vecMagnitude) / newMagnitude;
      // Add in the free torque. TODO: 2 free torques may complicate the calculations for force and COP addition.
      fmf->freeTorque[i] += freeTorque[i];
   }
   fmf->magnitude = newMagnitude;
}

static void transformMotion(MotionSequence* newMotion, MotionSequence* oldMotion, ModelStruct* model, ForceMatteForce* forces, int numForceMattes)
{
   int i, j, count, numGroundForceColumns;

   // Copy over all of the components that do not change.
   // Change the name of the old motion so that the new one
   // (which initially has the same name) will not get a "(2)"
   // appended to it when it is added to the model.
   memcpy(newMotion, oldMotion, sizeof(MotionSequence));
   mstrcpy(&newMotion->name, oldMotion->name);
   if (oldMotion->name[0] == '_')
      oldMotion->name[0] = '*';
   else
      oldMotion->name[0] = '_';
   mstrcpy(&newMotion->units, oldMotion->units);
   newMotion->deriv_names = NULL;
   newMotion->deriv_data = NULL;
   memset(&newMotion->mopt, 0, sizeof(MotionModelOptions));
   newMotion->mopt.num_motion_object_instances = 0;
   newMotion->mopt.motion_object_instance = NULL;
   newMotion->event_color[0] = 1.0f;
   newMotion->event_color[1] = 0.0f;
   newMotion->event_color[2] = 1.0f;

   if (oldMotion->num_events > 0)
   {
      newMotion->event = (smMotionEvent*)simm_malloc(sizeof(smMotionEvent)*oldMotion->num_events);
      memcpy(newMotion->event, oldMotion->event, sizeof(smMotionEvent)*oldMotion->num_events);
      for (j=0; j<oldMotion->num_events; j++)
         mstrcpy(&newMotion->event[j].name, oldMotion->event[j].name);
   }

   // Count how many columns in the old motion correspond to ground-based forces.
   for (i=0, numGroundForceColumns=0; i<oldMotion->number_of_datacolumns; i++)
   {
      if (columnIsGroundForce(oldMotion, model, i))
         numGroundForceColumns++;
   }

   newMotion->number_of_datacolumns = oldMotion->number_of_datacolumns - numGroundForceColumns + numForceMattes * 9;
   newMotion->columnname = (char**)simm_malloc(newMotion->number_of_datacolumns * sizeof(char*));
   newMotion->motiondata = (double**)simm_malloc(newMotion->number_of_datacolumns*sizeof(double*));
   for (i=0; i<newMotion->number_of_datacolumns; i++)
      newMotion->motiondata[i] = (double*)simm_calloc(newMotion->number_of_datarows, sizeof(double));
   newMotion->data_std_dev = (double**)simm_calloc(newMotion->number_of_datacolumns, sizeof(double*));

   // Copy over column names and data, except for the ground-based forces
   for (i=0, count=0; i<oldMotion->number_of_datacolumns; i++)
   {
      if (!columnIsGroundForce(oldMotion, model, i))
      {
         mstrcpy(&newMotion->columnname[count], oldMotion->columnname[i]);
         memcpy(newMotion->motiondata[count], oldMotion->motiondata[i], oldMotion->number_of_datarows*sizeof(double));
         count++;
      }
   }

   // Fill in the names for the force matte-based forces. The data will be filled in later.
   // There will be one force for each force matte, but they are kept in the ground frame
   // because that's how OpenSim expects them. Also, the order must be vx, vy, vz, px, py, pz, torquex, torquey, torquez.
#if 0
   for (i=0; i<numForceMattes; i++)
   {
      mstrcpy(&newMotion->columnname[count], "ground_force_vx");
      forces[i].column[0] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_force_vy");
      forces[i].column[1] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_force_vz");
      forces[i].column[2] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_force_px");
      forces[i].column[3] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_force_py");
      forces[i].column[4] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_force_pz");
      forces[i].column[5] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_torque_x");
      forces[i].column[6] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_torque_y");
      forces[i].column[7] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_torque_z");
      forces[i].column[8] = count++;
   }
#else
   for (i=0; i<numForceMattes; i++)
   {
      mstrcpy(&newMotion->columnname[count], "ground_force_vx");
      forces[i].column[0] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_force_vy");
      forces[i].column[1] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_force_vz");
      forces[i].column[2] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_force_px");
      forces[i].column[3] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_force_py");
      forces[i].column[4] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_force_pz");
      forces[i].column[5] = count++;
   }
   for (i=0; i<numForceMattes; i++)
   {
      mstrcpy(&newMotion->columnname[count], "ground_torque_x");
      forces[i].column[6] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_torque_y");
      forces[i].column[7] = count++;
      mstrcpy(&newMotion->columnname[count], "ground_torque_z");
      forces[i].column[8] = count++;
   }
#endif

   // Fill in the mopt structure.
   //link_model_to_motion(model, newMotion, &numOtherData, NULL);
}

static SBoolean columnIsGroundForce(MotionSequence* motion, ModelStruct* model, int index)
{
   int i, j;

   for (i=0; i<motion->mopt.num_motion_object_instances; i++)
   {
      if ((STRINGS_ARE_EQUAL(model->motion_objects[motion->mopt.motion_object_instance[i]->object].name, "force") ||
         STRINGS_ARE_EQUAL(model->motion_objects[motion->mopt.motion_object_instance[i]->object].name, "torque")) &&
         motion->mopt.motion_object_instance[i]->segment == model->ground_segment)
      {
         for (j=0; j<motion->mopt.motion_object_instance[i]->num_channels; j++)
         {
            if (motion->mopt.motion_object_instance[i]->channels[j].column == index)
               return yes;
         }
      }
   }

   return no;
}

#endif

void sort_events(MotionSequence* motion, int* index)
{
   int i, j, index_of_min;
   double min;
   smMotionEvent save;

   // Sort the events to have increasing xCoords. This uses
   // a simple N*2 bubble sort, but is OK since there are
   // not usually more than 10-12 events in a motion.
   for (i=0; i<motion->num_events; i++)
   {
      for (j=i, min = MAXMDOUBLE; j<motion->num_events; j++)
      {
         if (motion->event[j].xCoord < min)
         {
            min = motion->event[j].xCoord;
            index_of_min = j;
         }
      }
      if (index_of_min != i)
      {
         memcpy(&save, &motion->event[i], sizeof(smMotionEvent));
         memcpy(&motion->event[i], &motion->event[index_of_min], sizeof(smMotionEvent));
         memcpy(&motion->event[index_of_min], &save, sizeof(smMotionEvent));
         // Keep track of the event pointed to by index.
         if (index && *index == index_of_min)
            *index = i;
      }
   }
}


void normalize_motion(MotionSequence* motion, ModelStruct* model)
{
   int i, j, num_other, new_num_rows = 101;
   int new_num_columns = motion->number_of_datacolumns + 1;
   double **deriv_data = NULL, **motiondata = NULL, **data_std_dev = NULL;
   double abscissa, start, step_size, percent;
   char** columnname;
   dpFunction function;
   static const char percent_label[] = "percent";

   // Don't normalize if the motion already has new_num_rows rows and
   // min and max values of 0.0 and 100.0.
   if (EQUAL_WITHIN_ERROR(motion->min_value, 0.0) && EQUAL_WITHIN_ERROR(motion->max_value, 100.0) &&
      motion->number_of_datarows == new_num_rows)
   {
      (void)sprintf(errorbuffer, "Motion %s is already normalized.", motion->name);
      error(abort_action, errorbuffer);
      return;
   }

   start = motion->min_value;
   step_size = (motion->max_value - motion->min_value) / (double)(new_num_rows - 1);

   // In case the motion is currently being displayed, calculate and store the percent
   // so you can later set the motion to [about] the same time frame.
   percent = (motion->mopt.current_value - motion->min_value) / (motion->max_value - motion->min_value);

   // Make space for the new data (new_num_rows * new_num_columns).
   // new_num_columns should be 1 more than the motion currently has.
   // This extra column will be put in the first slot, and will contain
   // 'percent' data.
   columnname = (char**)simm_calloc(new_num_columns, sizeof(char*));
   mstrcpy(&columnname[0], percent_label);
   motiondata = (double**)simm_calloc(new_num_columns, sizeof(double*));
   if (motion->deriv_data)
      deriv_data = (double**)simm_calloc(new_num_columns, sizeof(double*));
   if (motion->data_std_dev)
      data_std_dev = (double**)simm_calloc(new_num_columns, sizeof(double*));
   for (i=0; i<new_num_columns; i++)
   {
      motiondata[i] = (double*)simm_calloc(new_num_rows, sizeof(double));
      // i = 0 is the [new] 'percent' column. The motion's original N columns
      // go into slots 1 through N.
      if (i > 0)
      {
         columnname[i] = motion->columnname[i-1];
         if (motion->deriv_data && motion->deriv_data[i-1])
            deriv_data[i] = (double*)simm_calloc(new_num_rows, sizeof(double));
         if (motion->data_std_dev && motion->data_std_dev[i-1])
            data_std_dev[i] = (double*)simm_calloc(new_num_rows, sizeof(double));
      }
   }

   malloc_function(&function, motion->number_of_datarows);
   function.numpoints = motion->number_of_datarows;
   function.source = dpJointFile;
   function.status = dpFunctionSimmDefined;
   function.type = dpNaturalCubicSpline;
   function.used = dpYes;

   // Fill in the [new] first column with 'percent'.
   for (j=0; j<new_num_rows; j++)
      motiondata[0][j] = j;

   // Fit splines to the data.
   for (i=0; i<motion->number_of_datacolumns; i++)
   {
      int markerMotionObject = column_is_marker_data(motion, i);
      function.numpoints = 0;
      for (j=0; j<motion->number_of_datarows; j++)
      {
         if (markerMotionObject < 0 || is_marker_data_visible(motion, markerMotionObject, j) == yes)
         {
            function.x[function.numpoints] = motion->motiondata[motion->x_column][j];
            function.y[function.numpoints] = motion->motiondata[i][j];
            function.numpoints++;
         }
      }
      calc_function_coefficients(&function);
      for (j=0, abscissa = start; j<new_num_rows; j++, abscissa += step_size)
         motiondata[i+1][j] = interpolate_function(abscissa, &function, zeroth, 0.0, 0.0);

      // Recalculate derivatives.
      // TODO5.0: is it better to use existing data?
      if (deriv_data && deriv_data[i])
      {
         for (j=0, abscissa = start; j<new_num_rows; j++, abscissa += step_size)
            deriv_data[i+1][j] = interpolate_function(abscissa, &function, first, 1.0, 1.0);
      }

      // Standard deviations cannot be recalculated. Assume all data points
      // are good and fit a spline.
      if (data_std_dev && data_std_dev[i])
      {
         function.numpoints = motion->number_of_datarows;
         for (j=0; j<motion->number_of_datarows; j++)
         {
            function.x[j] = motion->data_std_dev[0][j];
            function.y[j] = motion->data_std_dev[i][j];
         }
         calc_function_coefficients(&function);
         for (j=0, abscissa = start; j<new_num_rows; j++, abscissa += step_size)
            data_std_dev[i+1][j] = interpolate_function(abscissa, &function, zeroth, 0.0, 0.0);
      }
   }

   // Free the space used by the old data.
   FREE_IFNOTNULL(motion->columnname);
   for (i=0; i<motion->number_of_datacolumns; i++)
      FREE_IFNOTNULL(motion->motiondata[i]);
   FREE_IFNOTNULL(motion->motiondata);
   if (motion->deriv_data)
   {
      for (i=0; i<motion->number_of_datacolumns; i++)
         FREE_IFNOTNULL(motion->deriv_data[i]);
      free(motion->deriv_data);
   }
   if (motion->data_std_dev)
   {
      for (i=0; i<motion->number_of_datacolumns; i++)
         FREE_IFNOTNULL(motion->data_std_dev[i]);
      free(motion->data_std_dev);
   }

   motion->columnname = columnname;
   motion->motiondata = motiondata;
   motion->deriv_data = deriv_data;
   motion->data_std_dev = data_std_dev;

   motion->number_of_datacolumns = new_num_columns;
   motion->number_of_datarows = new_num_rows;

   motion->x_column = 0;
   motion->min_value = motion->motiondata[motion->x_column][0];
   motion->max_value = motion->motiondata[motion->x_column][motion->number_of_datarows-1];
   // If time_column is defined, increment it (because the percent column was inserted
   // at the beginning) and recalculate the event xCoords. If time_column is not defined,
   // there is no way to recalculate the xCoords and the events will not work properly.
   if (motion->time_column >= 0)
   {
      double oldRange, newRange;
      motion->time_column++;
      oldRange = motion->motiondata[motion->time_column][motion->number_of_datarows-1] -
         motion->motiondata[motion->time_column][0];
      newRange = motion->motiondata[motion->x_column][motion->number_of_datarows-1] -
         motion->motiondata[motion->x_column][0];
      for (i=0; i<motion->num_events; i++)
      {
         double per = (motion->event[i].xCoord - motion->motiondata[motion->time_column][0]) / oldRange;
         motion->event[i].xCoord = motion->motiondata[motion->x_column][0] + per * newRange;
      }
   }

    link_model_to_motion(model, motion, &num_other, NULL);

   // Recalculate the current frame, using the previously calculated percent.
   motion->mopt.current_value = percent * 100.0;
   motion->mopt.current_frame = percent * (motion->number_of_datarows - 1) + MOTION_OFFSET;

#if ! OPENSMAC && ! ENGINE
    make_marker_trails(model, motion);
    make_force_trails(model, motion);

   queue_model_redraw(model);
#endif

    // After interpolating, force plate forces can sometimes go negative.
    //TODO5.0: there's got to be a better way than this...
#if 1
    for (i=0; i<motion->mopt.num_motion_object_instances; i++)
    {
        for (j=0; j<motion->mopt.motion_object_instance[i]->num_channels; j++)
        {
            int comp = motion->mopt.motion_object_instance[i]->channels[j].component;
            if (comp == MO_VX || comp == MO_VY || comp == MO_VZ)
            {
                int k, col = motion->mopt.motion_object_instance[i]->channels[j].column;
                for (k=0; k<motion->number_of_datarows; k++)
                {
                    double value = ABS(motion->motiondata[col][k]);
                    if (value < 1.0)
                        motion->motiondata[col][k] = 0.0;
                }
            }
        }
    }
#endif

#if ! OPENSMAC && ! ENGINE
   // Generate an event so the tools can update themselves.
   make_and_queue_simm_event(MOTION_CHANGED, model->modelnum, motion, NULL, ZERO, ZERO);

   (void)sprintf(buffer, "Normalized motion %s to have %d time steps.", motion->name, new_num_rows);
   message(buffer, 0, DEFAULT_MESSAGE_X_OFFSET);
#endif
}


void smooth_motion(MotionSequence* motion, ModelStruct* model, double cutoff_frequency)
{
   int i, j, num_other;
   double **deriv_data = NULL, **motiondata = NULL;
   dpFunction function;

   if (motion->number_of_datarows < 6 || cutoff_frequency <= 0.0) //TODO5.0
      return;

   // Make space for the new data (new_num_rows * motion->number_of_datacolumns).
   motiondata = (double**)simm_calloc(motion->number_of_datacolumns, sizeof(double*));
   if (motion->deriv_data)
      deriv_data = (double**)simm_calloc(motion->number_of_datacolumns, sizeof(double*));
   for (i=0; i<motion->number_of_datacolumns; i++)
   {
      motiondata[i] = (double*)simm_calloc(motion->number_of_datarows, sizeof(double));
      if (motion->deriv_data && motion->deriv_data[i])
         deriv_data[i] = (double*)simm_calloc(motion->number_of_datarows, sizeof(double));
   }

   malloc_function(&function, motion->number_of_datarows);
   function.numpoints = 0; //motion->number_of_datarows;
   function.source = dpJointFile;
   function.status = dpFunctionSimmDefined;
   function.type = dpGCVSpline;
   function.cutoff_frequency = cutoff_frequency;
   function.used = dpYes;

   for (i=0; i<motion->number_of_datarows; i++)
      motiondata[motion->x_column][i] = motion->motiondata[motion->x_column][i];

   // Fit splines to the data.
   // TODO5.0: maybe it would be better to access the data through the mopt pointers
   // so you can more easily deal with it as gencoords, markers, etc.
   for (i=0; i<motion->number_of_datacolumns; i++)
   {
      if (i != motion->x_column)
      {
         int markerMotionObject = column_is_marker_data(motion, i);
         function.numpoints = 0;
         for (j=0; j<motion->number_of_datarows; j++)
         {
            if (markerMotionObject < 0 || is_marker_data_visible(motion, markerMotionObject, j) == yes)
            {
               function.x[function.numpoints] = motion->motiondata[motion->x_column][j];
               function.y[function.numpoints] = motion->motiondata[i][j];
               function.numpoints++;
            }
         }
         calc_function_coefficients(&function);
         for (j=0; j<motion->number_of_datarows; j++)
            motiondata[i][j] = interpolate_function(motiondata[motion->x_column][j], &function, zeroth, 0.0, 0.0);

         if (deriv_data && deriv_data[i])
         {
            for (j=0; j<motion->number_of_datarows; j++)
               deriv_data[i][j] = interpolate_function(motiondata[motion->x_column][j], &function, first, 1.0, 1.0);
         }
      }
   }

   // Free the space used by the old data.
   for (i=0; i<motion->number_of_datacolumns; i++)
      FREE_IFNOTNULL(motion->motiondata[i]);
   FREE_IFNOTNULL(motion->motiondata);
   if (motion->deriv_data)
   {
      for (i=0; i<motion->number_of_datacolumns; i++)
         FREE_IFNOTNULL(motion->deriv_data[i]);
      free(motion->deriv_data);
   }

   motion->motiondata = motiondata;
   motion->deriv_data = deriv_data;

   link_model_to_motion(model, motion, &num_other, NULL);

#if ! OPENSMAC && ! ENGINE
    make_marker_trails(model, motion);
    make_force_trails(model, motion);

   queue_model_redraw(model);
#endif

    // After interpolating, force plate forces can sometimes go negative.
    //TODO5.0: there's got to be a better way than this...
#if 1
    for (i=0; i<motion->mopt.num_motion_object_instances; i++)
    {
        for (j=0; j<motion->mopt.motion_object_instance[i]->num_channels; j++)
        {
            int comp = motion->mopt.motion_object_instance[i]->channels[j].component;
            if (comp == MO_VX || comp == MO_VY || comp == MO_VZ)
            {
                int k, col = motion->mopt.motion_object_instance[i]->channels[j].column;
                for (k=0; k<motion->number_of_datarows; k++)
                {
                    double value = ABS(motion->motiondata[col][k]);
                    if (value < 1.0)
                        motion->motiondata[col][k] = 0.0;
                }
            }
        }
    }
#endif

   //TODO5.0: send MOTION_CHANGED event so model viewer can update?
   // Also: how to figure out if motion is current, so it needs to be re-applied?
   (void)sprintf(buffer, "Smoothed motion %s with a cutoff frequency of %lf.", motion->name, cutoff_frequency);
   message(buffer, 0, DEFAULT_MESSAGE_X_OFFSET);
}

static int column_is_marker_data(MotionSequence* motion, int column)
{
   int i, j;

   for (i=0; i<motion->mopt.num_motion_object_instances; i++)
   {
      if (motion->mopt.motion_object_instance[i]->type == MarkerMotionObject)
      {
         for (j=0; j<motion->mopt.motion_object_instance[i]->num_channels; j++)
         {
            if (motion->mopt.motion_object_instance[i]->channels[j].column == column)
               return i;
         }
      }
   }

   return -1;
}

/* If any coordinate of a point is UNDEFINED, the point is not visible.
 * JPL 9/12/03: C3D files seem to use 0.0 as undefined, so if all three
 * coordinates are 0.0, return false as well.
 */
SBoolean is_marker_visible(double pt[])
{
   if (EQUAL_WITHIN_ERROR(pt[0], UNDEFINED_DOUBLE))
      return no;
   if (EQUAL_WITHIN_ERROR(pt[1], UNDEFINED_DOUBLE))
      return no;
   if (EQUAL_WITHIN_ERROR(pt[2], UNDEFINED_DOUBLE))
      return no;

   if (EQUAL_WITHIN_ERROR(pt[0], 0.0) && EQUAL_WITHIN_ERROR(pt[1], 0.0) && EQUAL_WITHIN_ERROR(pt[2], 0.0))
      return no;

   return yes;
}

SBoolean is_marker_visible_meters(double pt[])
{
   double bad_data = UNDEFINED_DOUBLE * 0.001;

   if (EQUAL_WITHIN_ERROR(pt[0], bad_data))
      return no;
   if (EQUAL_WITHIN_ERROR(pt[1], bad_data))
      return no;
   if (EQUAL_WITHIN_ERROR(pt[2], bad_data))
      return no;

   if (EQUAL_WITHIN_ERROR(pt[0], 0.0) && EQUAL_WITHIN_ERROR(pt[1], 0.0) && EQUAL_WITHIN_ERROR(pt[2], 0.0))
      return no;

   return yes;
}

SBoolean is_marker_data_visible(MotionSequence* motion, int motionObject, int row)
{
   double pt[3];
   MotionObjectInstance* moi = motion->mopt.motion_object_instance[motionObject];

   pt[0] = motion->motiondata[moi->channels[0].column][row];
   pt[1] = motion->motiondata[moi->channels[1].column][row];
   pt[2] = motion->motiondata[moi->channels[2].column][row];

   return is_marker_visible_meters(pt);
}


int get_frame_number(smC3DStruct* c3d, double time)
{
    int i;

    for (i=0; i<c3d->motionData->header.numFrames; i++)
    {
        if (c3d->motionData->frameList[i].time >= time)
            return i;
    }

    return c3d->motionData->header.numFrames - 1;
}

MotionObjectInstance* add_foot_print(ModelStruct* model, MotionSequence* motion, const char name[], double scale,
                                     double translation[], double eulerRotation[])
{
    int i, j, index, motion_object = -1;
    MotionObjectInstance* mi = NULL;
    ReturnCode rc;

    index = motion->mopt.num_motion_object_instances++;

    if (motion->mopt.motion_object_instance == NULL)
    {
        motion->mopt.motion_object_instance = (MotionObjectInstance**)
            simm_malloc(motion->mopt.num_motion_object_instances * sizeof(MotionObjectInstance*));
    }
    else
    {
        motion->mopt.motion_object_instance = (MotionObjectInstance**)
            simm_realloc(motion->mopt.motion_object_instance, motion->mopt.num_motion_object_instances * sizeof(MotionObjectInstance*), &rc);
    }

    if (motion->mopt.motion_object_instance == NULL)
    {
        motion->mopt.num_motion_object_instances--;
        return NULL;
    }

    for (i=0; i<model->num_motion_objects; i++)
    {
        if (STRINGS_ARE_EQUAL(model->motion_objects[i].name, name))
        {
            motion_object = i;
            break;
        }
    }
    if (motion_object == -1)
        return NULL;

    // Initialize a new motion object instance.
    mi = motion->mopt.motion_object_instance[index] = (MotionObjectInstance*)simm_calloc(1, sizeof(MotionObjectInstance));

   mstrcpy(&mi->name, model->motion_objects[motion_object].name);
    mi->type = FootPrintObject;
    mi->object = motion_object;
    mi->segment = model->ground_segment;
    mi->drawmode = model->motion_objects[motion_object].drawmode;
    mi->current_value = -1.0;
    mi->visible = yes;
    identity_matrix(mi->currentXform);
    copy_material(&model->dis.mat.materials[model->motion_objects[motion_object].material], &mi->currentMaterial);

    if (eulerRotation)
    {
        x_rotate_matrix_bodyfixed(mi->currentXform, eulerRotation[0]);
        y_rotate_matrix_bodyfixed(mi->currentXform, eulerRotation[1]);
        z_rotate_matrix_bodyfixed(mi->currentXform, eulerRotation[2]);
    }

    for (i=0; i<3; i++)
        for (j=0; j<3; j++)
            mi->currentXform[i][j] *= scale;

    if (translation)
    {
        mi->currentXform[3][0] = translation[0];
        mi->currentXform[3][1] = translation[1];
        mi->currentXform[3][2] = translation[2];
    }

    return mi;
}

MotionObjectInstance* add_force_plate(ModelStruct* model, MotionSequence* motion, const char name[], double scale[],
                                      double translation[], double eulerRotation[])
{
    int i, j, index, motion_object = -1;
    MotionObjectInstance* mi = NULL;
    ReturnCode rc;

    index = motion->mopt.num_motion_object_instances++;

    if (motion->mopt.motion_object_instance == NULL)
    {
        motion->mopt.motion_object_instance = (MotionObjectInstance**)
            simm_malloc(motion->mopt.num_motion_object_instances * sizeof(MotionObjectInstance*));
    }
    else
    {
        motion->mopt.motion_object_instance = (MotionObjectInstance**)
            simm_realloc(motion->mopt.motion_object_instance, motion->mopt.num_motion_object_instances * sizeof(MotionObjectInstance*), &rc);
    }

    if (motion->mopt.motion_object_instance == NULL)
    {
        motion->mopt.num_motion_object_instances--;
        return NULL;
    }

    for (i=0; i<model->num_motion_objects; i++)
    {
        if (STRINGS_ARE_EQUAL(model->motion_objects[i].name, name))
        {
            motion_object = i;
            break;
        }
    }
    if (motion_object == -1)
        return NULL;

    // Initialize a new motion object instance.
    mi = motion->mopt.motion_object_instance[index] = (MotionObjectInstance*)simm_calloc(1, sizeof(MotionObjectInstance));

   mstrcpy(&mi->name, model->motion_objects[motion_object].name);
    mi->type = ForcePlateObject;
    mi->object = motion_object;
    mi->segment = model->ground_segment;
    mi->drawmode = model->motion_objects[motion_object].drawmode;
    mi->current_value = -1.0;
    mi->visible = yes;
    identity_matrix(mi->currentXform);
    copy_material(&model->dis.mat.materials[model->motion_objects[motion_object].material], &mi->currentMaterial);

    if (eulerRotation)
    {
        x_rotate_matrix_bodyfixed(mi->currentXform, eulerRotation[0]);
        y_rotate_matrix_bodyfixed(mi->currentXform, eulerRotation[1]);
        z_rotate_matrix_bodyfixed(mi->currentXform, eulerRotation[2]);
    }

    if (scale)
    {
        for (i=0; i<3; i++)
            for (j=0; j<3; j++)
                mi->currentXform[i][j] *= scale[i];
    }

    if (translation)
    {
        mi->currentXform[3][0] = translation[0];
        mi->currentXform[3][1] = translation[1];
        mi->currentXform[3][2] = translation[2];
    }

    if (0)
    {
        double mat[4][4];
        printf("add_force_plate, rot = %lf %lf %lf\n", eulerRotation[0], eulerRotation[1], eulerRotation[2]);
        print_4x4matrix(mi->currentXform);
        for (i=0; i<3; i++)
            printf("row %d mag = %lf\n", i, VECTOR_MAGNITUDE(mi->currentXform[i]));
        transpose_4x4matrix(mi->currentXform, mat);
        for (i=0; i<3; i++)
            printf("col %d mag = %lf\n", i, VECTOR_MAGNITUDE(mat[i]));
    }
    return mi;
}
