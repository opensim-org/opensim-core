/*******************************************************************************

   READMODEL.C

   Author: Peter Loan

   Date: 8-DEC-88

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains routines that read in a model
      definition from an input file.

   Routines:
      read_model_file : controls the reading of a model definition
      read_gencoord   : reads in a gencoord definition
      read_joint      : controls the reading of all joint parameters
      read_restraint  : reads a gencoord restraint function
      read_segment    : reads in a segment description
      read_axis       : reads in a rotation axis for a joint
      read_refeq      : reads in a 'reference equation'
      read_order      : reads in the transformation order for a joint
      read_function    : reads in a set of x-y pairs that define a function
      malloc_function : mallocs space for a splined function

*******************************************************************************/
#include <ctype.h>

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "defunctions.h"

#include "wefunctions.h"

/*************** DEFINES (for this file only) *********************************/
#define INIT_SEGMENT_FILE_ARRAY_SIZE 5
#define SEGMENT_FILE_ARRAY_INCREMENT 10

static char* beginposeString = "beginpose";
static char* endposeString = "endpose";
static char* ignoreString1 = "read_script_file";
static char* ignoreString2 = "write_script_file";
static char* ignoreString3 = "crosshair_visibility";
static char* ignoreString4 = "muscle_point_visibility";
static char* ignoreString5 = "shadow_visibility";


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static ReturnCode read_gencoord(ModelStruct* ms, FILE* fp);
static ReturnCode set_wrap_specs(dpWrapObject* wo, char str[]);
static ReturnCode read_wrap_object(ModelStruct* ms, FILE* fp);
static ReturnCode read_restraint(ModelStruct* ms, FILE* fp, GeneralizedCoord* gc, int minmax);
static ReturnCode read_joint(ModelStruct* ms, FILE* fp);
static ReturnCode read_segment(ModelStruct* ms, FILE* fp);
static ReturnCode read_axis(ModelStruct* ms, FILE* fp, int joint, int axis);
static ReturnCode read_refeq(ModelStruct* ms, FILE* fp, DofStruct* jointvar);
static ReturnCode read_order(FILE* fp, int order[]);
static ReturnCode read_show_axis(ModelStruct* ms, FILE* fp, JointStruct* jnt);
static ReturnCode read_modelview(Scene* scene, ModelStruct* ms, FILE* fp);

static ReturnCode read_gencoord_groups(ModelStruct*, FILE*, GeneralizedCoord* gencoord);
static ReturnCode read_segment_groups(ModelStruct*, FILE*, int segnum);
static DrawingMode get_drawmode(char buffer[]);

static ReturnCode read_contact(ModelStruct* ms, FILE* fp);
static ReturnCode read_spring(ModelStruct* ms, FILE* fp);
static ReturnCode read_spring_floor(ModelStruct* ms, FILE* fp);
static void add_spring_point_to_floor(SegmentStruct* segment, SpringPoint* point);
static ReturnCode read_force_matte(ModelStruct* ms, SegmentStruct* seg, FILE* fp);
static ReturnCode read_contact_object(ModelStruct* ms, FILE* fp);
static ReturnCode read_contact_pair(ModelStruct* ms, FILE* fp);
static ReturnCode read_contact_group(ModelStruct* ms, FILE* fp);
static PolyhedronStruct* make_bone(ModelStruct* ms, SegmentStruct* seg, char filename[]);
static ReturnCode read_marker(ModelStruct* ms, SegmentStruct* seg, FILE* fp);

static ReturnCode read_constraint(ModelStruct *ms, FILE* fp);
static ReturnCode set_constraint_specs(ConstraintObject* co, char str[]);
static ConstraintPoint* read_constraint_points(ModelStruct* ms, FILE* fp, int *numpoints, int *cp_array_size);
static ReturnCode read_motion_object(ModelStruct* ms, FILE* fp);

//TODO_SCENE: ideally, this function should take a model argument, not a scene.
// The model should be added to the scene later. But for now, you need the scene
// to store some of the display info that is specified in the model file.
ReturnCode read_model_file(Scene* scene, ModelStruct* ms, char filename[], SBoolean showTopLevelMessages)
{
   char tempFileName[CHARBUFFER];
   int model_errors = 0;
   FILE* fp;
   ReturnCode status = code_fine;

#if ENGINE
    strcpy(tempFileName, ".model");
#else
   strcpy(tempFileName, glutGetTempFileName("simm-model"));
#endif

   if ((fp = preprocess_file(filename, tempFileName)) == NULL)
   {
      if (showTopLevelMessages == yes)
      {
         (void)sprintf(errorbuffer, "Unable to open model input file %s", filename);
         error(none, errorbuffer);
      }
      return code_bad;
   }

   /* Now that you know the joint file exists, store the full path in the
    * model struct because the code to read bone files needs it.
    */
   mstrcpy(&ms->jointfilename,filename);

   while (1)
   {
      if (read_string(fp,buffer) == EOF)
         break;

      if (buffer[0] == '#')
      {
         read_nonempty_line(fp,buffer);
         continue;
      }
      /* Skip over unsupported options */
      if (STRINGS_ARE_EQUAL(buffer, ignoreString1) ||
          STRINGS_ARE_EQUAL(buffer, ignoreString2) ||
          STRINGS_ARE_EQUAL(buffer, ignoreString3) ||
          STRINGS_ARE_EQUAL(buffer, ignoreString4) ||
          STRINGS_ARE_EQUAL(buffer, ignoreString5))
      {
          _read_til(fp, '\n');
          continue;
      }
      if (STRINGS_ARE_EQUAL(buffer, beginposeString)){
          // skip until endposeString
          while(STRINGS_ARE_NOT_EQUAL(buffer, endposeString))
            read_line(fp,buffer);
          _read_til(fp, '\n');
          continue;
      }
      if (STRINGS_ARE_EQUAL(buffer,"name"))
      {
         read_line(fp,buffer);
         ms->name = (char*)simm_malloc(250*sizeof(char));
         if (ms->name == NULL)
            goto input_error;
         (void)strcpy(ms->name,buffer);
         strip_trailing_white_space(ms->name);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"force_units"))
      {
         if (read_string(fp, buffer) == EOF)
            break;
         mstrcpy(&ms->forceUnits, buffer);
      }
      else if ((STRINGS_ARE_EQUAL(buffer,"length_units")) || STRINGS_ARE_EQUAL(buffer, "translation_units"))
      {
         if (read_string(fp, buffer) == EOF)
            break;
         mstrcpy(&ms->lengthUnits, buffer);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"HTR_translation_units"))
      {
         if (read_string(fp, buffer) == EOF)
            break;
         mstrcpy(&ms->HTRtranslationUnits, buffer);
      }
      else if (STRINGS_ARE_EQUAL(buffer, "MV_gear") || STRINGS_ARE_EQUAL(buffer, "motion_speed"))
      {
         if (fscanf(fp, "%lf", &ms->dis.motion_speed) != 1)
         {
            error(recover,"Error reading MV_gear value in joint file.");
            break;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"solver_accuracy"))
      {
         if (fscanf(fp, "%lf", &ms->solver.accuracy) != 1)
         {
            error(recover,"Error reading solver_accuracy value in joint file.");
            break;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"solver_max_iterations"))
      {
         if (fscanf(fp, "%d", &ms->solver.max_iterations) != 1)
         {
            error(recover,"Error reading solver_max_iterations value in joint file.");
            break;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"solver_method"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer, "LM") || STRINGS_ARE_EQUAL(buffer, "lm"))
            ms->solver.method = smLevenbergMarquart;
         else if (STRINGS_ARE_EQUAL(buffer, "GN") || STRINGS_ARE_EQUAL(buffer, "gn"))
            ms->solver.method = smGaussNewton;
         else
         {
            error(recover,"Error reading solver_method in joint file (must be LM or GN).");
            break;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"solver_joint_limits"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "YES")
            || STRINGS_ARE_EQUAL(buffer, "true") || STRINGS_ARE_EQUAL(buffer, "TRUE"))
            ms->solver.joint_limits = smYes;
         else if (STRINGS_ARE_EQUAL(buffer, "no") || STRINGS_ARE_EQUAL(buffer, "NO")
            || STRINGS_ARE_EQUAL(buffer, "false") || STRINGS_ARE_EQUAL(buffer, "FALSE"))
            ms->solver.joint_limits = smNo;
         else
         {
            error(recover,"Error reading solver_joint_limits in joint file (must be yes/true or no/false).");
            break;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"solver_orient_body"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "YES")
            || STRINGS_ARE_EQUAL(buffer, "true") || STRINGS_ARE_EQUAL(buffer, "TRUE"))
            ms->solver.orient_body = smYes;
         else if (STRINGS_ARE_EQUAL(buffer, "no") || STRINGS_ARE_EQUAL(buffer, "NO")
            || STRINGS_ARE_EQUAL(buffer, "false") || STRINGS_ARE_EQUAL(buffer, "FALSE"))
            ms->solver.orient_body = smNo;
         else
         {
            error(recover,"Error reading solver_orient_body in joint file (must be yes/true or no/false).");
            break;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"solver_fg_contact"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "YES")
            || STRINGS_ARE_EQUAL(buffer, "true") || STRINGS_ARE_EQUAL(buffer, "TRUE"))
            ms->solver.fg_contact = smYes;
         else if (STRINGS_ARE_EQUAL(buffer, "no") || STRINGS_ARE_EQUAL(buffer, "NO")
            || STRINGS_ARE_EQUAL(buffer, "false") || STRINGS_ARE_EQUAL(buffer, "FALSE"))
            ms->solver.fg_contact = smNo;
         else
         {
            error(recover,"Error reading solver_fg_contact in joint file (must be yes/true or no/false).");
            break;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"window_position"))
      {
         if (fscanf(fp, "%d %d", &scene->windowX1, &scene->windowY1) != 2)
         {
            error(recover,"Error reading window_position (X, Y) in joint file.");
            break;
         }
         else
         {
#if ! ENGINE
            int scrLeft, scrRight, scrTop, scrBottom;
            glutGetWorkArea(&scrLeft, &scrRight, &scrTop, &scrBottom);
            if (scene->windowX1 >= scrRight)
               scene->windowX1 = scrRight - 100;
#endif
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"window_size"))
      {
         if (fscanf(fp, "%d %d", &scene->windowWidth, &scene->windowHeight) != 2)
         {
            error(recover,"Error reading window_size (width, height) in joint file.");
            break;
         }
         else
         {
            if (scene->windowWidth < 50)
               scene->windowWidth = 50;
            if (scene->windowHeight < 50)
               scene->windowHeight = 50;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "gravity"))
      {
         if (fscanf(fp, "%s", buffer) != 1)
         {
            error(recover, "Error reading gravity value in joint file.");
            break;
         }
         if (STRINGS_ARE_EQUAL(buffer,"x") || STRINGS_ARE_EQUAL(buffer,"X") ||
             STRINGS_ARE_EQUAL(buffer,"+x") || STRINGS_ARE_EQUAL(buffer,"+X"))
            ms->gravity = smX;
         else if (STRINGS_ARE_EQUAL(buffer,"-x") || STRINGS_ARE_EQUAL(buffer,"-X"))
            ms->gravity = smNegX;
         else if (STRINGS_ARE_EQUAL(buffer,"y") || STRINGS_ARE_EQUAL(buffer,"Y") ||
                  STRINGS_ARE_EQUAL(buffer,"+y") || STRINGS_ARE_EQUAL(buffer,"+Y"))
            ms->gravity = smY;
         else if (STRINGS_ARE_EQUAL(buffer,"-y") || STRINGS_ARE_EQUAL(buffer,"-Y"))
            ms->gravity = smNegY;
         else if (STRINGS_ARE_EQUAL(buffer,"z") || STRINGS_ARE_EQUAL(buffer,"Z") ||
                  STRINGS_ARE_EQUAL(buffer,"+z") || STRINGS_ARE_EQUAL(buffer,"+Z"))
            ms->gravity = smZ;
         else if (STRINGS_ARE_EQUAL(buffer,"-z") || STRINGS_ARE_EQUAL(buffer,"-Z"))
            ms->gravity = smNegZ;
         else if (STRINGS_ARE_EQUAL(buffer,"none") || STRINGS_ARE_EQUAL(buffer,"zero"))
            ms->gravity = smNoAlign;
         else
         {
            error(abort_action, "Bad axis specified for gravity (must be X, -X, Y, -Y, Z, -Z, zero, or none");
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginjoint"))
      {
         if (read_joint(ms,fp) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"begingencoord"))
      {
         if (read_gencoord(ms, fp) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginsegment"))
      {
         if (read_segment(ms, fp) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer, get_function_tag(dpFunctionTypeUndefined, 0))) // old-style function definitions
      {
         if (read_function(ms, fp, no, dpNaturalCubicSpline, get_function_tag(dpFunctionTypeUndefined, 1)) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer, get_function_tag(dpStepFunction, 0)))
      {
         if (read_function(ms, fp, no, dpStepFunction, get_function_tag(dpStepFunction, 1)) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer, get_function_tag(dpLinearFunction, 0)))
      {
         if (read_function(ms, fp, no, dpLinearFunction, get_function_tag(dpLinearFunction, 1)) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer, get_function_tag(dpNaturalCubicSpline, 0)))
      {
         if (read_function(ms, fp, no, dpNaturalCubicSpline, get_function_tag(dpNaturalCubicSpline, 1)) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer, get_function_tag(dpGCVSpline, 0)))
      {
         if (read_function(ms, fp, no, dpGCVSpline, get_function_tag(dpGCVSpline, 1)) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginworldobject"))
      {
          if (read_world_object(ms, fp) == code_bad)
             goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginmotionobject"))
      {
          if (read_motion_object(ms, fp) == code_bad)
             goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"begincontact"))
      {
         if (read_contact(ms,fp) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginwrapobject"))
      {
         if (read_wrap_object(ms,fp) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"begindeformity"))
      {
         if (read_deformity(ms,fp) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "constraint_tolerance"))
      {
         double value;
         fscanf(fp, "%lg", &value);
        /* if (fscanf(fp,"%lg", &ms->constraint_tolerance) != 1)
         {
            error(recover,"Error reading constraint_tolerance.");
            ms->constraint_tolerance = DEFAULT_CONSTRAINT_TOLERANCE;
            model_errors++;
         }*/
      }
      else if (STRINGS_ARE_EQUAL(buffer, "loop_tolerance"))
      {
         if (fscanf(fp,"%lg", &ms->loop_tolerance) != 1)
         {
            error(recover,"Error reading loop_tolerance.");
            ms->loop_tolerance = DEFAULT_LOOP_TOLERANCE;
            model_errors++;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "loop_weight"))
      {
         if (fscanf(fp,"%lg", &ms->loop_weight) != 1)
         {
            error(recover,"Error reading loop_weight.");
            ms->loop_weight = DEFAULT_LOOP_WEIGHT;
            model_errors++;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "beginconstraintobject"))
      {
         if (read_constraint(ms, fp) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginmaterial"))
      {
         if (read_material(ms, fp) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"begincamera") || STRINGS_ARE_EQUAL(buffer,"beginview"))
      {
         if (read_modelview(scene, ms, fp) == code_bad)
            goto input_error;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"no_trackball"))
      {
         scene->trackball = no;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"bone_path"))
      {
         if (read_line(fp, buffer) == EOF)
         {
            error(recover,"Error reading name of bone path within joint file.");
            model_errors++;
         }
         else
         {
            strip_trailing_white_space(buffer);
            mstrcpy(&ms->bonepathname,buffer);
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"muscle_file"))
      {
         if (read_line(fp, buffer) == EOF)
         {
            error(recover,"Error reading name of muscle file within joint file.");
            model_errors++;
         }
         else
         {
            strip_trailing_white_space(buffer);
            mstrcpy(&ms->musclefilename,buffer);
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"motion_file"))
      {
         if (read_line(fp, buffer) == EOF)
         {
            error(recover,"Error reading name of motion file within joint file.");
            model_errors++;
         }
         else
         {
            if (ms->num_motion_files >= 100)
            {
               error(recover,"You can specify only 100 motion files in a joint file.");
            }
            else
            {
               strip_trailing_white_space(buffer);
               mstrcpy(&ms->motionfilename[ms->num_motion_files],buffer);
               ms->num_motion_files++;
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "inverse_kinematics_solver") || STRINGS_ARE_EQUAL(buffer, "IK_solver"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            ms->useIK = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            ms->useIK = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"background_color"))
      {
         GLfloat col[3];
         if (fscanf(fp,"%f %f %f", &col[0], &col[1], &col[2]) != 3)
         {
            error(recover,"Error reading background color for model window.");
            model_errors++;
         }
         else
         {
            ms->dis.background_color[0] = col[0];
            ms->dis.background_color[1] = col[1];
            ms->dis.background_color[2] = col[2];
            ms->dis.background_color_spec = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"rotation_axes_color"))
      {
         GLfloat col[3];
         if (fscanf(fp,"%f %f %f", &col[0], &col[1], &col[2]) != 3)
         {
            error(recover,"Error reading color for joint rotation axes.");
            model_errors++;
         }
         else
         {
            ms->dis.rotation_axes_color[0] = col[0];
            ms->dis.rotation_axes_color[1] = col[1];
            ms->dis.rotation_axes_color[2] = col[2];
            ms->dis.rotation_axes_color_spec = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "segment_axes_color"))
      {
         GLfloat col[3];
         fscanf(fp, "%f %f %f", &col[0], &col[1], &col[2]);
         error(none, "Warning: segment_axes_color is no longer supported. Axes are now colored red/green/blue.");
      }
      else if (STRINGS_ARE_EQUAL(buffer,"vertex_label_color"))
      {
         GLfloat col[3];
         if (fscanf(fp,"%f %f %f", &col[0], &col[1], &col[2]) != 3)
         {
            error(recover,"Error reading color for vertex labels.");
            model_errors++;
         }
         else
         {
            ms->dis.vertex_label_color[0] = col[0];
            ms->dis.vertex_label_color[1] = col[1];
            ms->dis.vertex_label_color[2] = col[2];
            ms->dis.vertex_label_color_spec = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"crosshairs_color"))
      {
         GLfloat col[3];
         if (fscanf(fp,"%f %f %f", &col[0], &col[1], &col[2]) != 3)
         {
            error(recover,"Error reading color for model window crosshairs.");
            model_errors++;
         }
         else
         {
            ms->dis.crosshairs_color[0] = col[0];
            ms->dis.crosshairs_color[1] = col[1];
            ms->dis.crosshairs_color[2] = col[2];
            ms->dis.crosshairs_color_spec = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"marker_visibility"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            ms->marker_visibility = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            ms->marker_visibility = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"marker_radius"))
      {
         if (fscanf(fp,"%lg", &ms->marker_radius) != 1)
         {
            error(recover,"Error reading marker_radius.");
            ms->marker_radius = DEFAULT_MARKER_RADIUS;
            model_errors++;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"masscenter_visibility"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            ms->global_show_masscenter = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            ms->global_show_masscenter = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"inertia_visibility"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            ms->global_show_inertia = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            ms->global_show_inertia = yes;
      }
      else
      {
         (void)sprintf(errorbuffer,"Unrecognized string \"%s\" found in joint file.", buffer);
         error(recover,errorbuffer);
         model_errors++;
      }
      if (model_errors > 50)
      {
         error(none,"Too many errors to continue.");
         goto input_error;
      }
   }

   goto cleanup;
   
 input_error:
   status = code_bad;
   
 cleanup:
   (void)fclose(fp);
   (void)unlink((const char*)tempFileName);

   return status;
}



/* READ_GENCOORD: */

static ReturnCode read_gencoord(ModelStruct* ms, FILE* fp)
{
   int nk;
   char key1[64], key2[64];
   SBoolean range_specified = no, restraintFuncSpecified = no;
   double rangeval1, rangeval2, gc_tolerance, pd_stiffness;
   GeneralizedCoord* gc;

   if (read_string(fp, buffer) == EOF)
   {
      sprintf(errorbuffer,"Unexpected EOF found reading gencoord definition");
      error(abort_action, errorbuffer);
      return code_bad;
   }

   gc = enter_gencoord(ms, buffer, yes);

   if (gc == NULL)
   {
      sprintf(errorbuffer, "Unable to allocate space for gencoord %s", buffer);
      error(abort_action, errorbuffer);
      return code_bad;
   }

   if (gc->defined == yes)
   {
      (void)sprintf(errorbuffer,"Error: redefinition of gencoord %s", buffer);
      error(abort_action,errorbuffer);
      return code_bad;
   }

   while (1)
   {
      if (read_string(fp,buffer) == EOF)
      {
         (void)sprintf(errorbuffer,"Unexpected EOF found reading gencoord definition");
         error(abort_action,errorbuffer);
         return code_bad;
      }

      if (STRINGS_ARE_EQUAL(buffer,"endgencoord"))
      {
         gc->defined = yes;
         break;
      }

      if (STRINGS_ARE_EQUAL(buffer,"keys"))
      {
         read_line(fp,buffer);
         nk = sscanf(buffer,"%s %s", key1, key2);
         if (nk == 1)
            gc->keys[0] = gc->keys[1] = lookup_simm_key(key1);
         else if (nk == 2)
         {
            gc->keys[0] = lookup_simm_key(key1);
            gc->keys[1] = lookup_simm_key(key2);
         }
         else
         {
            (void)sprintf(errorbuffer,"Error reading keys for gencoord %s",
               gc->name);
            error(recover,errorbuffer);
            gc->keys[0] = gc->keys[1] = null_key;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"wrap"))
      {
         gc->wrap = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"range"))
      {
         if (fscanf(fp,"%lg %lg", &rangeval1, &rangeval2) != 2)
         {
            (void)sprintf(errorbuffer, "Error reading range for gencoord %s",
               gc->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         gc->range.start = _MIN(rangeval1,rangeval2);
         gc->range.end = _MAX(rangeval1,rangeval2);
         range_specified = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"tolerance"))
      {
         if (fscanf(fp,"%lg", &gc_tolerance) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading tolerance for gencoord %s",
               gc->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         gc->tolerance = gc_tolerance;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"default_value"))
      {
         if (fscanf(fp,"%lg", &gc->default_value) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading default value for gencoord %s",
               gc->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         gc->default_value_specified = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"unclamped"))
      {
         gc->clamped = gc->clamped_save = no;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "clamped"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            gc->clamped = gc->clamped_save = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            gc->clamped = gc->clamped_save = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"locked"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            gc->locked = gc->locked_save = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            gc->locked = gc->locked_save = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"restraint"))
      {
         if (read_restraint(ms,fp,gc,2) == code_bad)
         {
            (void)sprintf(errorbuffer, "Error reading restraint for gencoord %s",
               gc->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         restraintFuncSpecified = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "active"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            gc->restraintFuncActive = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            gc->restraintFuncActive = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"minrestraint"))
      {
         if (read_restraint(ms,fp,gc,0) == code_bad)
         {
            (void)sprintf(errorbuffer, "Error reading minrestraint for gencoord %s",
               gc->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"maxrestraint"))
      {
         if (read_restraint(ms,fp,gc,1) == code_bad)
         {
            (void)sprintf(errorbuffer, "Error reading maxrestraint for gencoord %s",
               gc->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"begingroups"))
      {
         if (read_gencoord_groups(ms, fp, gc) != code_fine)
         {
            gc->numgroups = 0;
            sprintf(errorbuffer,"Error reading groups for gencoord %s.", gc->name);
            error(recover,errorbuffer);
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"visible"))
      {
         if (read_string(fp,buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            gc->slider_visible = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            gc->slider_visible = yes;
      }
#if INCLUDE_MOCAP_MODULE
      else if (STRINGS_ARE_EQUAL(buffer,"mocap_value"))
      {
        char componentName[32];
        
        if (fscanf(fp, "%s %s", buffer, componentName) != 2)
        {
           sprintf(errorbuffer, "Error reading mocap info for %s gencoord.", gc->name);
           error(abort_action, errorbuffer);
           return code_bad;
        }
        mstrcpy(&gc->mocap_segment, buffer);

        gc->mocap_seg_index = -1;
        gc->mocap_column    = -1;

        switch (tolower(componentName[0]))
        {
           case 't':
              switch (tolower(componentName[1]))
              {
                 case 'x': gc->mocap_column = _TX; break;
                 case 'y': gc->mocap_column = _TY; break;
                 case 'z': gc->mocap_column = _TZ; break;
              }
              break;

           case 'r':
              switch (tolower(componentName[1]))
              {
                 case 'x': gc->mocap_column = _RX; break;
                 case 'y': gc->mocap_column = _RY; break;
                 case 'z': gc->mocap_column = _RZ; break;
              }
              break;
        }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"mocap_adjust"))
      {
         if (fscanf(fp,"%lg", &gc->mocap_adjust) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading \'mocap_adjust\' for gencoord %s",
              gc->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
#endif
      else if (STRINGS_ARE_EQUAL(buffer,"pd_stiffness") || STRINGS_ARE_EQUAL(buffer,"PD_stiffness"))
      {
         if (fscanf(fp,"%lg", &pd_stiffness) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading pd_stiffness for gencoord %s", gc->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         gc->pd_stiffness = pd_stiffness;
      }
      else
      {
         (void)sprintf(errorbuffer, "Unknown string (%s) in definition of gencoord %s",
               buffer, gc->name);
         error(recover,errorbuffer);
      }

   }

   if (range_specified == no)
   {
      (void)sprintf(errorbuffer,"Range not specified for gencoord %s", gc->name);
      error(abort_action,errorbuffer);
      return code_bad;
   }

   if (restraintFuncSpecified == no)
      gc->restraintFuncActive = no;

   return code_fine;
}



static ReturnCode set_wrap_specs(dpWrapObject* wo, char str[])
{
   if (STRINGS_ARE_EQUAL(str,"none") || STRINGS_ARE_EQUAL(str,"full"))
   {
      wo->wrap_axis = 0;
      wo->wrap_sign = 0;
      return code_fine;
   }
   if (STRINGS_ARE_EQUAL(str,"-x") || STRINGS_ARE_EQUAL(str,"-X"))
   {
      wo->wrap_axis = 0;
      wo->wrap_sign = -1;
      return code_fine;
   }
   else if (STRINGS_ARE_EQUAL(str,"x") || STRINGS_ARE_EQUAL(str,"X"))
   {
      wo->wrap_axis = 0;
      wo->wrap_sign = 1;
      return code_fine;
   }
   else if (STRINGS_ARE_EQUAL(str,"-y") || STRINGS_ARE_EQUAL(str,"-Y"))
   {
      wo->wrap_axis = 1;
      wo->wrap_sign = -1;
      return code_fine;
   }
   else if (STRINGS_ARE_EQUAL(str,"y") || STRINGS_ARE_EQUAL(str,"Y"))
   {
      wo->wrap_axis = 1;
      wo->wrap_sign = 1;
      return code_fine;
   }
   else if (STRINGS_ARE_EQUAL(str,"-z") || STRINGS_ARE_EQUAL(str,"-Z"))
   {
      wo->wrap_axis = 2;
      wo->wrap_sign = -1;
      return code_fine;
   }
   else if (STRINGS_ARE_EQUAL(str,"z") || STRINGS_ARE_EQUAL(str,"Z"))
   {
      wo->wrap_axis = 2;
      wo->wrap_sign = 1;
      return code_fine;
   }

   return code_bad;
}

static ReturnCode set_constraint_specs(ConstraintObject* co, char str[])
{
   if (STRINGS_ARE_EQUAL(str,"none") || STRINGS_ARE_EQUAL(str,"full"))
   {
      co->constraintAxis = 0;
      co->constraintSign = 0;
      return code_fine;
   }
   if (STRINGS_ARE_EQUAL(str,"-x") || STRINGS_ARE_EQUAL(str,"-X"))
   {
      co->constraintAxis = 0;
      co->constraintSign = -1;
      return code_fine;
   }
   else if (STRINGS_ARE_EQUAL(str,"x") || STRINGS_ARE_EQUAL(str,"X"))
   {
      co->constraintAxis = 0;
      co->constraintSign = 1;
      return code_fine;
   }
   else if (STRINGS_ARE_EQUAL(str,"-y") || STRINGS_ARE_EQUAL(str,"-Y"))
   {
      co->constraintAxis = 1;
      co->constraintSign = -1;
      return code_fine;
   }
   else if (STRINGS_ARE_EQUAL(str,"y") || STRINGS_ARE_EQUAL(str,"Y"))
   {
      co->constraintAxis = 1;
      co->constraintSign = 1;
      return code_fine;
   }
   else if (STRINGS_ARE_EQUAL(str,"-z") || STRINGS_ARE_EQUAL(str,"-Z"))
   {
      co->constraintAxis = 2;
      co->constraintSign = -1;
      return code_fine;
   }
   else if (STRINGS_ARE_EQUAL(str,"z") || STRINGS_ARE_EQUAL(str,"Z"))
   {
      co->constraintAxis = 2;
      co->constraintSign = 1;
      return code_fine;
   }

   return code_bad;
}


static ReturnCode read_wrap_object(ModelStruct* ms, FILE* fp)
{
   ReturnCode rc = code_fine;
   dpWrapObject* wo;
   double xyz[3];
   DMatrix m;

   if (ms->num_wrap_objects == ms->wrap_object_array_size)
   {
      ms->wrap_object_array_size += WRAP_OBJECT_ARRAY_INCREMENT;
      ms->wrapobj = (dpWrapObject**)simm_realloc(ms->wrapobj, ms->wrap_object_array_size*sizeof(dpWrapObject*), &rc);

      if (rc == code_bad)
      {
         ms->wrap_object_array_size -= WRAP_OBJECT_ARRAY_INCREMENT;
         return code_bad;
      }
   }

   wo = ms->wrapobj[ms->num_wrap_objects] = (dpWrapObject*)simm_malloc(sizeof(dpWrapObject));

   initwrapobject(wo);

   if (fscanf(fp, "%s", buffer) != 1)
   {
      error(abort_action, "Error reading name in wrap object definition");
      return code_bad;
   }
   else
      mstrcpy(&wo->name, buffer);

   while (1)
   {
      if (read_string(fp, buffer) == EOF)
         break;

      if (buffer[0] == '#')
      {
         read_nonempty_line(fp, buffer);
         continue;
      }

      if (STRINGS_ARE_EQUAL(buffer, "wraptype"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer, "sphere"))
            wo->wrap_type = dpWrapSphere;
         else if (STRINGS_ARE_EQUAL(buffer, "cylinder"))
            wo->wrap_type = dpWrapCylinder;
         else if (STRINGS_ARE_EQUAL(buffer, "ellipsoid"))
            wo->wrap_type = dpWrapEllipsoid;
         else if (STRINGS_ARE_EQUAL(buffer, "torus"))
            wo->wrap_type = dpWrapTorus;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "wrapmethod"))
      {
         int i;

         if (read_string(fp, buffer) == EOF)
            break;

         for (i = 0; i < WE_NUM_WRAP_ALGORITHMS; i++)
         {
            if (STRINGS_ARE_EQUAL(buffer, get_wrap_algorithm_name(i)))
            {
               wo->wrap_algorithm = i;
               break;
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "segment"))
      {
         if (fscanf(fp,"%s", buffer) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading segment in definition of wrap object %s", wo->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
         else
         {
            wo->segment = enter_segment(ms, buffer, yes);

            if (wo->segment == -1)
               return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "active"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer, "no") || STRINGS_ARE_EQUAL(buffer, "false"))
            wo->active = no;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "visible"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer, "no") || STRINGS_ARE_EQUAL(buffer, "false"))
            wo->visible = no;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "show_wrap_pts"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "true"))
            wo->show_wrap_pts = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "rotation"))
      {
         sprintf(errorbuffer, "Error reading rotation for wrap object %s:", wo->name);
         error(none, errorbuffer);
         error(abort_action, "  The \'rotation\' keyword has been changed to \'xyz_body_rotation\'.");
         return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "xyz_body_rotation"))
      {
         if (fscanf(fp, "%lg %lg %lg", &xyz[0], &xyz[1], &xyz[2]) != 3)
         {
            (void)sprintf(errorbuffer, "Error reading rotation for wrap object %s", wo->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
         identity_matrix(m);
         x_rotate_matrix_bodyfixed(m, xyz[0] * DTOR);
         y_rotate_matrix_bodyfixed(m, xyz[1] * DTOR);
         z_rotate_matrix_bodyfixed(m, xyz[2] * DTOR);
         extract_rotation(m, &wo->rotation_axis, &wo->rotation_angle);
      }
      else if (STRINGS_ARE_EQUAL(buffer, "xyz_space_rotation"))
      {
         if (fscanf(fp, "%lg %lg %lg", &xyz[0], &xyz[1], &xyz[2]) != 3)
         {
            (void)sprintf(errorbuffer, "Error reading rotation for wrap object %s", wo->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         identity_matrix(m);
         x_rotate_matrix_spacefixed(m, xyz[0] * DTOR);
         y_rotate_matrix_spacefixed(m, xyz[1] * DTOR);
         z_rotate_matrix_spacefixed(m, xyz[2] * DTOR);
         extract_rotation(m, &wo->rotation_axis, &wo->rotation_angle);
      }
      else if (STRINGS_ARE_EQUAL(buffer, "rotationaxis"))
      {
         fscanf(fp, "%lg %lg %lg", &wo->rotation_axis.xyz[0], &wo->rotation_axis.xyz[1], &wo->rotation_axis.xyz[2]);
      }
      else if (STRINGS_ARE_EQUAL(buffer, "rotationangle"))
      {
         if (fscanf(fp, "%lg", &wo->rotation_angle) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading rotation for wrap object %s", wo->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "translation") || STRINGS_ARE_EQUAL(buffer, "translate"))
      {
         if (fscanf(fp,"%lg %lg %lg", &wo->translation.xyz[0], &wo->translation.xyz[1], &wo->translation.xyz[2]) != 3)
         {
            (void)sprintf(errorbuffer, "Error reading translation for wrap object %s", wo->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "radius"))
      {
         if (wo->wrap_type == dpWrapEllipsoid)
         {
            rc = ((fscanf(fp, "%lg %lg %lg", &wo->radius[0], &wo->radius[1], &wo->radius[2]) == 3) ? code_fine : code_bad);
            if (wo->radius[0] <= 0.0 || wo->radius[1] <= 0.0 || wo->radius[2] <= 0.0)
            {
               error(none, "Value must be greater than zero.");
               rc = code_bad;
            }
         }
         else if (wo->wrap_type == dpWrapTorus)
         {
            rc = ((fscanf(fp, "%lg %lg", &wo->radius[0], &wo->radius[1]) == 2) ? code_fine : code_bad);
            if (wo->radius[0] <= 0.0 || wo->radius[1] <= 0.0)
            {
               error(none, "Value must be greater than zero.");
               rc = code_bad;
            }
         }
         else
         {
            rc = ((fscanf(fp, "%lg", &wo->radius[0]) == 1) ? code_fine : code_bad);
            if (wo->radius[0] <= 0.0)
            {
               error(none, "Value must be greater than zero.");
               rc = code_bad;
            }
         }

         if (rc == code_bad)
         {
            (void)sprintf(errorbuffer, "Error reading radius for wrap object %s", wo->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "height"))
      {
         int num = fscanf(fp, "%lg", &wo->height);

         if (num == 1 && wo->height <= 0.0)
         {
            error(none, "Value must be greater than zero.");
            num = -1;
         }

         if (num != 1)
         {
            (void)sprintf(errorbuffer, "Error reading cylinder height for wrap object %s", wo->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "quadrant"))
      {
         if (read_string(fp, buffer) == EOF)
            break;

         set_wrap_specs(wo, buffer);
      }
      else if (STRINGS_ARE_EQUAL(buffer, "endwrapobject"))
         break;
   }
   wo->undeformed_translation = wo->translation;

   wo->xforms_valid = no;

   ms->num_wrap_objects++;

   return code_fine;
}


static ReturnCode read_restraint(ModelStruct* ms, FILE* fp, GeneralizedCoord* gc, int minmax)
{
   int user_num;

   if (read_string(fp, buffer) == EOF)
      return code_bad;

   // The string just read should be of the form "f<num>", such as "f32".

   if (buffer[0] != 'f')
      return code_bad;

   if (minmax == 2)
   {
      if (sscanf(&buffer[1], "%d", &user_num) != 1)
         return code_bad;
      gc->restraint_function = ms->function[enter_function(ms, user_num, yes)];
   }
   else if (minmax == 0)
   {
      if (sscanf(&buffer[1], "%d", &user_num) != 1)
         return code_bad;
      gc->min_restraint_function = ms->function[enter_function(ms, user_num, yes)];
   }
   else
   {
      if (sscanf(&buffer[1], "%d", &user_num) != 1)
         return code_bad;
      gc->max_restraint_function = ms->function[enter_function(ms, user_num, yes)];
   }

   return code_fine;
}


/* READ_JOINT: */

static ReturnCode read_joint(ModelStruct* ms, FILE* fp)
{
   int i, jointnum;
   char from[CHARBUFFER], to[CHARBUFFER], jointname[CHARBUFFER];
   char check = 0;
   int jointvarnum;
   JointStruct* jnt;

   if (fscanf(fp, "%s", jointname) != 1)
   {
      error(abort_action,"Error reading name in joint definition");
      return code_bad;
   }

   jointnum = enter_joint(ms, jointname, yes);
   if (jointnum == -1)
      return code_bad;

   jnt = &ms->joint[jointnum];

   if (jnt->defined == yes)
   {
      (void)sprintf(errorbuffer,"Error: redefinition of joint %s", jointname);
      error(abort_action,errorbuffer);
      return code_bad;
   }

   while (1)
   {
      if (read_string(fp,buffer) == EOF)
      {
         (void)sprintf(errorbuffer,"Unexpected EOF found reading joint definition");
         error(abort_action,errorbuffer);
         return code_bad;
      }

      if (STRINGS_ARE_EQUAL(buffer,"endjoint"))
      {
         jnt->defined = yes;
         break;
      }

      if (STRINGS_ARE_EQUAL(buffer,"loopjoint"))
      {
         /* IK solver, Nov. 2000:
          * the loopjoint keyword in the joint file no longer sets the loop joint,
          * since the Inverse Kinematics routines take care to keep the loop unbroken.
          * You no longer have to specify the joint at which the display gets broken.
          */
         jnt->user_loop = yes;
         continue;
      }

      if (STRINGS_ARE_EQUAL(buffer,"show"))
      {
         if (read_show_axis(ms,fp,jnt) == code_bad)
            return code_bad;
         continue;
      }
      if (STRINGS_ARE_EQUAL(buffer,"type"))
      {
         if (fscanf(fp,"%s", buffer) != 1)
         {
            sprintf(errorbuffer, "Error reading type for joint %s", jnt->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         mstrcpy(&jnt->solverType, buffer);
         continue;
      }
      if (STRINGS_ARE_EQUAL(buffer,"pretransform"))
      {
         /* NOTE: typically the joint pretransform is computed dynamically to
          * support deformations (and possibly other effects), and is not
          * specified in a joint file.  However I left this code here because
          * it is useful for testing the joint pretransform mechanism.  -- KMS
          */
         int i,j;
         
         for (i=0; i<4; i++)
         {
            for (j=0; j<4; j++)
            {
               if (fscanf(fp,"%lg", &jnt->pretransform[i][j]) != 1)
               {
                  sprintf(errorbuffer, "Error reading pretransform for joint %s", jnt->name);
                  error(abort_action,errorbuffer);
                  return code_bad;
               }
            }
         }
         invert_4x4transform(jnt->pretransform, jnt->pretransform_inverse);
         continue;
      }
      
#if INCLUDE_MOCAP_MODULE
      if (STRINGS_ARE_EQUAL(buffer,"mocap_segment"))
      {
         if (fscanf(fp, "%s", buffer) != 1)
         {
            sprintf(errorbuffer, "Error reading \'mocap_segment\' for joint %s.", jnt->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
         mstrcpy(&jnt->mocap_segment, buffer);
         continue;
      }
#endif
      
      if ((jointvarnum = getjointvarnum(buffer)) == -1)
      {
         (void)sprintf(errorbuffer,
            "Unknown string (%s) in definition of joint %s",
            buffer, jnt->name);
         error(recover,errorbuffer);
         continue;
      }
      
      if (jointvarnum >= 0 && jointvarnum < 6)
      {
         if (read_refeq(ms,fp,&jnt->dofs[jointvarnum]) == code_bad)
         {
            (void)sprintf(errorbuffer,"Error reading definition of %s in joint %s",
               getjointvarname(jointvarnum), jnt->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         else
            check |= 1 << jointvarnum;
      }
      else if (jointvarnum == 6)
      {
         if (read_order(fp,jnt->order) == code_bad)
         {
            (void)sprintf(errorbuffer,
               "Error reading order of transformations in joint %s", jnt->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (jointvarnum > 6 && jointvarnum < 10)
      {
         if (read_axis(ms,fp,jointnum,jointvarnum-7) == code_bad)
            return code_bad;
      }
      else if (jointvarnum == 10)
      {
         if (fscanf(fp,"%s %s", from, to) != 2)
         {
            (void)sprintf(errorbuffer,"Error reading segments in definition of joint %s", jnt->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         else
         {
            jnt->from = enter_segment(ms,from,yes);
            jnt->to = enter_segment(ms,to,yes);
            if (jnt->from == -1 || jnt->to == -1)
               return code_bad;
         }
      }
   }
   
   /* Each time a dof is properly defined, the corresponding bit in the 'check'
    * variable is set to 1. Thus when we see 'endjoint,' we can examine this
    * variable to see if the lower 6 bits are all 1s (which means check = 63).
    * If check != 63, then not all 6 of the dofs have been properly defined,
    * so print an error message.
    */
   
   if (check < 63)
   {
      (void)sprintf(errorbuffer, "Incomplete definition of joint %s:", jnt->name);
      error(none,errorbuffer);
      for (i=0; i<6; i++)
      {
         if (!(check & (1<<i)))
         {
            (void)sprintf(errorbuffer,"  %s undefined.", getjointvarname(i));
            error(none,errorbuffer);
         }
      }
      error(abort_action,NULL);
      return code_bad;
   }
   
   return code_fine;

}



/* READ_SEGMENT: */

static ReturnCode read_segment(ModelStruct* ms, FILE* fp)
{

   int i, segmentnum, array_size = INIT_SEGMENT_FILE_ARRAY_SIZE;
   int shadow_axis;
   char segmentname[CHARBUFFER], name[CHARBUFFER], buf[2][CHARBUFFER];
   double shadow_trans;
   SegmentStruct* seg;

   if (fscanf(fp,"%s", segmentname) != 1)
   {
      (void)sprintf(errorbuffer,"Error trying to start a segment definition");
      error(abort_action,errorbuffer);
      return code_bad;
   }

   segmentnum = enter_segment(ms,segmentname,yes);
   if (segmentnum == -1)
      return code_bad;

   seg = &ms->segment[segmentnum];

   if (seg->defined == yes)
   {
      (void)sprintf(errorbuffer,"Error: redefinition of segment %s", segmentname);
      error(abort_action,errorbuffer);
      return code_bad;
   }

   while (1)
   {
      if (read_string(fp,buffer) == EOF)
      {
         (void)sprintf(errorbuffer, "Unexpected EOF found reading segment definition");
         error(abort_action,errorbuffer);
         return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"endsegment"))
      {
         seg->defined = yes;
         break;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginfiles"))
      {
         while (1)
         {
            if (read_string(fp,buffer) == EOF)
            {
               (void)sprintf(errorbuffer, "Error reading bone files for %s segment.", seg->name);
               error(abort_action,errorbuffer);
               return code_bad;
            }
            else if (STRINGS_ARE_EQUAL(buffer,"endfiles"))
            {
               break;
            }
            else
            {
               PolyhedronStruct* ph;

               strcpy(name,buffer);
               ph = make_bone(ms,seg,name);
               if (ph != NULL)
               {
                  ph->drawmode = -1;
                  ph->material = -1;
               }
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"bone"))
      {
         PolyhedronStruct* ph;

         fscanf(fp,"%s", name);
         ph = make_bone(ms,seg,name);
         read_line(fp,buffer);
         if (ph != NULL)
         {
            int m, ns;
            ph->material = -1;
            ph->drawmode = -1;
            ns = sscanf(buffer,"%s %s", buf[0], buf[1]);
            for (ns--; ns >= 0; ns--)
            {
               m = get_drawmode(buf[ns]);
               if (m != -1)
               {
                  ph->drawmode = m;
               }
               else
               {
                  m = enter_material(ms, buf[ns], declaring_element);
                  if (m != -1)
                  {
                     ph->material = m;
                  }
                  else
                  {
                     sprintf(errorbuffer,"Error reading material and drawmode for bone %s.", ph->name);
                     error(recover,errorbuffer);
                  }
               }
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"begingroups"))
      {
         if (read_segment_groups(ms, fp, segmentnum) != code_fine)
         {
            seg->numgroups = 0;
            
            sprintf(errorbuffer,"Error reading groups for segment %s.", seg->name);
            error(recover,errorbuffer);
         }
      }
#if INCLUDE_BONE_EDITOR_EXTRAS
      else if (STRINGS_ARE_EQUAL(buffer,"pts_file"))
      {
         FILE* fp_pts;

         fscanf(fp,"%s", buffer);
         mstrcpy(&seg->pts_file,buffer);
         fp_pts = simm_fopen(seg->pts_file,"r");
         if (fp_pts != NULL)
         {
            fscanf(fp_pts,"%d", &seg->num_raw);
            seg->raw_vertices = (double*)simm_malloc(seg->num_raw*3*sizeof(double));
            for (i=0; i<seg->num_raw; i++)
               fscanf(fp_pts,"%*s %lf %lf %lf", &seg->raw_vertices[i*3],
               &seg->raw_vertices[i*3+1], &seg->raw_vertices[i*3+2]);
            fclose(fp_pts);
         }
         else
         {
            sprintf(errorbuffer,"Cannot open %s", buffer);
            error(none,errorbuffer);
         }
      }
#endif
      else if (STRINGS_ARE_EQUAL(buffer,"force_matte"))
      {
         if (read_force_matte(ms, seg, fp) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"marker"))
      {
         if (read_marker(ms, seg, fp) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"shadow"))
      {
         if (fscanf(fp,"%s %lg", buffer, &shadow_trans) != 2)
         {
            (void)sprintf(errorbuffer,"Error reading shadow axis and/or shadow translation for %s segment", seg->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         else
         {
            if (STRINGS_ARE_EQUAL(buffer,"x") || STRINGS_ARE_EQUAL(buffer,"X"))
               shadow_axis = 0;
            else if (STRINGS_ARE_EQUAL(buffer,"y") || STRINGS_ARE_EQUAL(buffer,"Y"))
               shadow_axis = 1;
            else if (STRINGS_ARE_EQUAL(buffer,"z") || STRINGS_ARE_EQUAL(buffer,"Z"))
               shadow_axis = 2;
            else
            {
               (void)sprintf(errorbuffer,"Bad axis specified in shadow for %s segment", seg->name);
               error(abort_action,errorbuffer);
               return code_bad;
            }
            seg->shadow_scale[shadow_axis] = 0.0;
            seg->shadow_trans[shadow_axis] = shadow_trans;
            seg->shadow = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"shadowcolor"))
      {
         if (fscanf(fp,"%f %f %f", &seg->shadow_color.rgb[RD],
            &seg->shadow_color.rgb[GR], &seg->shadow_color.rgb[BL]) != 3)
         {
            (void)sprintf(errorbuffer,"Error reading shadow color for %s segment", seg->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         /* Support for 'old style' colors (range: 0 to 255). If any of the
         * color components is greater than 1.0, assume that it's an old
         * style color definition.
         */
         if (seg->shadow_color.rgb[RD] > 1.0 ||
            seg->shadow_color.rgb[GR] > 1.0 ||
            seg->shadow_color.rgb[BL] > 1.0)
         {
            seg->shadow_color.rgb[RD] /= 255.0;
            seg->shadow_color.rgb[GR] /= 255.0;
            seg->shadow_color.rgb[BL] /= 255.0;
         }
         for (i=0; i<3; i++)
         {
            if (seg->shadow_color.rgb[i] < 0.0)
               seg->shadow_color.rgb[i] = 0.0;
         }
         seg->shadow_color_spec = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"material"))
      {
         if (fscanf(fp,"%s", name) != 1)
         {
            (void)sprintf(errorbuffer,"Error reading material for %s segment", seg->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         else
         {
            seg->material = enter_material(ms,name,declaring_element);
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"scale"))
      {
         if (fscanf(fp,"%f %f %f", &seg->bone_scale[0], &seg->bone_scale[1],
            &seg->bone_scale[2]) != 3)
         {
            (void)sprintf(errorbuffer,"Error reading scale factors for %s segment", seg->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"mass"))
      {
         if (fscanf(fp,"%lg", &seg->mass) != 1)
         {
            (void)sprintf(errorbuffer,"Error reading mass for %s segment", seg->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         seg->mass_specified = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"inertia"))
      {
         for (i=0; i<9; i++)
         {
            if (fscanf(fp,"%lg", &seg->inertia[i/3][i%3]) != 1)
            {
               (void)sprintf(errorbuffer,"Error reading inertia for %s segment", seg->name);
               error(abort_action,errorbuffer);
               return code_bad;
            }
         }
         seg->inertia_specified = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"masscenter"))
      {
         for (i=0; i<3; i++)
         {
            if (fscanf(fp,"%lg", &seg->masscenter[i]) != 1)
            {
               (void)sprintf(errorbuffer,"Error reading masscenter for %s segment",
                  seg->name);
               error(abort_action,errorbuffer);
               return code_bad;
            }
         }
         seg->masscenter_specified = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"show_masscenter"))
      {
         if (fscanf(fp, "%s", buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            seg->show_masscenter = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            seg->show_masscenter = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "show_inertia"))
      {
         if (fscanf(fp, "%s", buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer, "no") || STRINGS_ARE_EQUAL(buffer, "off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            seg->show_inertia = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            seg->show_inertia = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"drawmode"))
      {
         if (read_drawmode(fp, &seg->drawmode) != code_fine)
         {
            sprintf(errorbuffer, "Error reading drawmode for %s segment", seg->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"axes"))
      {
         if (fscanf(fp, "%lg", &seg->axis_length) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading axis length for %s segment", seg->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
         else
            seg->draw_axes = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "axes_length"))
      {
         if (fscanf(fp,"%lg", &seg->axis_length) != 1)
         {
            (void)sprintf(errorbuffer,"Error reading axis length for %s segment", seg->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"show_axes"))
      {
         seg->draw_axes = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"begindeform") ||
         STRINGS_ARE_EQUAL(buffer,"deform"))
      {
         if (read_deform(fp, seg, segmentnum) != code_fine)
            return code_bad;
      }
#if INCLUDE_MOCAP_MODULE
      else if (STRINGS_ARE_EQUAL(buffer,"lengthstart"))
      {
         if (fscanf(fp,"%f %f %f", &seg->lengthstart[0], &seg->lengthstart[1],
            &seg->lengthstart[2]) != 3)
         {
            (void)sprintf(errorbuffer,"Error reading lengthstart for %s segment", seg->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         else
         {
            seg->lengthstartend_specified = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"lengthend"))
      {
         if (fscanf(fp,"%f %f %f", &seg->lengthend[0], &seg->lengthend[1],
            &seg->lengthend[2]) != 3)
         {
            (void)sprintf(errorbuffer,"Error reading lengthend for %s segment", seg->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         else
         {
            seg->lengthstartend_specified = yes;
         }
      }
#endif
      else if (STRINGS_ARE_EQUAL(buffer, "htr_o"))
      {
         if (fscanf(fp, "%lg %lg %lg", &seg->htr_o[0], &seg->htr_o[1], &seg->htr_o[2]) != 3)
         {
            (void)sprintf(errorbuffer, "Error reading htr_o for %s segment", seg->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "htr_x"))
      {
         if (fscanf(fp, "%lg %lg %lg", &seg->htr_x[0], &seg->htr_x[1], &seg->htr_x[2]) != 3)
         {
            (void)sprintf(errorbuffer, "Error reading htr_x for %s segment", seg->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "htr_y"))
      {
         if (fscanf(fp, "%lg %lg %lg", &seg->htr_y[0], &seg->htr_y[1], &seg->htr_y[2]) != 3)
         {
            (void)sprintf(errorbuffer, "Error reading htr_y for %s segment", seg->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
      }
#if INCLUDE_MOCAP_MODULE
      else if (STRINGS_ARE_EQUAL(buffer,"gait_scale"))
      {
         if (fscanf(fp, "%s %lf %lf %lf", buffer, &seg->gait_scale_factor[0],
            &seg->gait_scale_factor[1], &seg->gait_scale_factor[2]) != 4)
         {
            sprintf(errorbuffer, "Error reading \'gait_scale\' for %s segment.", seg->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
         mstrcpy(&seg->gait_scale_segment, buffer);
      }         
      else if (STRINGS_ARE_EQUAL(buffer,"mocap_segment"))
      {
         if (fscanf(fp, "%s", buffer) != 1)
         {
            sprintf(errorbuffer, "Error reading \'mocap_segment\' for %s segment.", seg->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
         mstrcpy(&seg->mocap_segment, buffer);

         if (fscanf(fp, "%s", buffer) != 1)
         {
            sprintf(errorbuffer, "Error reading \'mocap_segment\' for %s segment.", seg->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
         
         if (STRINGS_ARE_EQUAL(buffer, "inherit_scale"))
            seg->mocap_scaling_method = INHERIT_SCALE;
         else if (STRINGS_ARE_EQUAL(buffer, "scale_one_to_one"))
            seg->mocap_scaling_method = SCALE_ONE_TO_ONE;
         else if (STRINGS_ARE_EQUAL(buffer, "scale_chain"))
         {
            seg->mocap_scaling_method = SCALE_CHAIN;
            
            /* read the first mocap scale chain end name:
            */
            if (fscanf(fp, "%s", buffer) != 1)
            {
               sprintf(errorbuffer, "Error reading \'mocap_segment\' for %s segment.", seg->name);
               error(abort_action, errorbuffer);
               return code_bad;
            }
            mstrcpy(&seg->mocap_scale_chain_end1, buffer);
            
            /* read the second mocap scale chain end name:
            */
            if (fscanf(fp, "%s", buffer) != 1)
            {
               sprintf(errorbuffer, "Error reading \'mocap_segment\' for %s segment.", seg->name);
               error(abort_action, errorbuffer);
               return code_bad;
            }
            mstrcpy(&seg->mocap_scale_chain_end2, buffer);
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"mocap_scale_to_segment"))
      {
         if (fscanf(fp, "%s", buffer) != 1)
         {
            sprintf(errorbuffer, "Error reading \'mocap_scale_to_segment\' for %s segment.", seg->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
         mstrcpy(&seg->mocap_scale_chain_end1, buffer);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"mocap_adjustment_xform"))
      {
         double axis[3], angle;
         
         if (fscanf(fp, "%lg %lg %lg %lg", &axis[XX], &axis[YY], &axis[ZZ], &angle) != 4)
         {
            sprintf(errorbuffer, "Error reading mocap adjustment for %s segment.", seg->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
         
         fprintf(stderr, "MOCAP ADJUST: %s [%.3f %.3f %.3f] %.1f\n",
            seg->name, axis[XX], axis[YY], axis[ZZ], angle);
         
         angle *= DTOR;
         
         rotate_matrix_axis_angle(seg->mocap_adjustment_xform, axis, angle);
      }
#endif
      else
      {
         (void)sprintf(errorbuffer, "Unknown string (%s) in definition of segment %s",
            buffer, seg->name);
         error(recover,errorbuffer);
      }
   }

   for (i=0; i<seg->numBones; i++)
   {
      if (seg->bone[i].material == -1)
         seg->bone[i].material = seg->material;
      if (seg->bone[i].drawmode == -1)
         seg->bone[i].drawmode = seg->drawmode;
   }

   return code_fine;
}



/* READ_AXIS: */

static ReturnCode read_axis(ModelStruct* ms, FILE* fp, int joint, int axis)
{
   int i;
   double total;

   for (i=0; i<3; i++)
   {
      if (fscanf(fp, "%lg", &ms->joint[joint].parentrotaxes[axis][i]) != 1)
      {
         (void)sprintf(errorbuffer, "Error reading definition of axis%d in joint %s", axis, ms->joint[joint].name);
         error(abort_action, errorbuffer);
         return code_bad;
      }
   }

   ms->joint[joint].parentrotaxes[axis][3] = 0.0;

   total = sqrt(ms->joint[joint].parentrotaxes[axis][0] * ms->joint[joint].parentrotaxes[axis][0] +
                ms->joint[joint].parentrotaxes[axis][1] * ms->joint[joint].parentrotaxes[axis][1] +
                ms->joint[joint].parentrotaxes[axis][2] * ms->joint[joint].parentrotaxes[axis][2]);

   if (EQUAL_WITHIN_ERROR(total, 0.0))
   {
      (void)sprintf(errorbuffer, "Error reading axis%d in joint %s (vector has zero length)", axis+1, ms->joint[joint].name);
      error(abort_action, errorbuffer);
      return code_bad;
   }

   for (i=0; i<3; i++)
      ms->joint[joint].parentrotaxes[axis][i] /= total;

   return code_fine;
}



/* READ_REFEQ: */

static ReturnCode read_refeq(ModelStruct* ms, FILE* fp, DofStruct* jointvar)
{
   int j, k, n;
   int userfuncnum;
   char usergencname[CHARBUFFER];

   read_string(fp,buffer);

   if (STRINGS_ARE_EQUAL(buffer,"constant"))
   {
      jointvar->type = constant_dof;
      jointvar->function = NULL;
      jointvar->gencoord = NULL;
      if (fscanf(fp,"%lg", &jointvar->value) != 1)
         return code_bad;
      else
         return code_fine;
   }

   if (STRINGS_ARE_EQUAL(buffer,"function"))
   {
      jointvar->type = function_dof;
      j = 0;
      while (1)
      {
         if (j == CHARBUFFER-1)
            return code_bad;
         buffer[j] = getc(fp);
         if (buffer[j] == (char) EOF)
            return code_bad;
         if (buffer[j] == ')')
            break;
         j++;
      }
      buffer[j+1] = STRING_TERMINATOR;
      j = 0;
      while (buffer[j++] != 'f')
         ;
      k = sscanf(&buffer[j],"%d", &userfuncnum);
      while (buffer[j++] != '(')        /* remove the open paren */
         ;
      n = j + strlen(&buffer[j]) - 1;   /* the next four lines remove the */
      while (buffer[n] != ')')          /* close paren at the end */
         n--;
      buffer[n] = STRING_TERMINATOR;
      k += sscanf(&buffer[j],"%s", usergencname); /* now read the gencoord name */
      if (k != 2)
         return code_bad;
      else
      {
         jointvar->function = ms->function[enter_function(ms, userfuncnum, yes)];
         jointvar->gencoord = enter_gencoord(ms, usergencname, yes);
         if (jointvar->gencoord == NULL)
            return code_bad;
         return code_fine;
      }
   }

   return code_bad;
}


static ReturnCode read_order(FILE* fp, int order[])
{
   int i;
   char buf[4][CHARBUFFER];

   if (fscanf(fp,"%s %s %s %s", buf[0],buf[1],buf[2],buf[3]) != 4)
      return code_bad;

   /* order[] holds the position of each matrix in the multiplication order.
    * Order[0] contains the order of the translation matrix, order[1] contains
    * the order of the axis1--rotation matrix, etc.
    */
   for (i=0; i<4; i++)
   {
      if (STRINGS_ARE_EQUAL(buf[i],"t"))
         order[0] = i;
      else if (STRINGS_ARE_EQUAL(buf[i],"r1"))
         order[1] = i;
      else if (STRINGS_ARE_EQUAL(buf[i],"r2"))
         order[2] = i;
      else if (STRINGS_ARE_EQUAL(buf[i],"r3"))
         order[3] = i;
   }

   return code_fine;
}


static ReturnCode read_show_axis(ModelStruct* ms, FILE* fp, JointStruct* jnt)
{
   double length;

   if (fscanf(fp,"%s %lg", buffer, &length) != 2)
   {
      sprintf(errorbuffer,"Error reading axis name and/or length after \"show\" keyword in joint %s", jnt->name);
      error(abort_action,errorbuffer);
      return code_bad;
   }

   if (STRINGS_ARE_EQUAL(buffer,"axis1"))
   {
      jnt->axis_length[0] = length;
      jnt->show_axes[0] = yes;
   }
   else if (STRINGS_ARE_EQUAL(buffer,"axis2"))
   {
      jnt->axis_length[1] = length;
      jnt->show_axes[1] = yes;
   }
   else if (STRINGS_ARE_EQUAL(buffer,"axis3"))
   {
      jnt->axis_length[2] = length;
      jnt->show_axes[2] = yes;
   }

   return code_fine;
}


static ReturnCode read_modelview(Scene* scene, ModelStruct* ms, FILE* fp)
{
   int i, j;

   if (ms->dis.num_file_views >= MAXSAVEDVIEWS)
   {
      sprintf(errorbuffer,"Only %d cameras are allowed in the joints file",
          MAXSAVEDVIEWS);
      error(recover,errorbuffer);
      return code_bad;
   }

   if (fscanf(fp,"%s", buffer) != 1)
   {
      error(abort_action,"Error reading name of camera");
      return code_bad;
   }

   mstrcpy(&ms->dis.view_name[ms->dis.num_file_views], buffer);

   for (i=0; i<4; i++)
   {
      for (j=0; j<4; j++)
      {
         if (fscanf(fp,"%lg", &ms->dis.saved_view[ms->dis.num_file_views][i][j]) != 1)
         {
            sprintf(errorbuffer,"Error reading matrix for camera %s",
               ms->dis.view_name[ms->dis.num_file_views]);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
   }

   if (fscanf(fp,"%s", buffer) != 1)
   {
      error(abort_action,"Error reading camera");
      return code_bad;
   }
   else if (STRINGS_ARE_NOT_EQUAL(buffer,"endcamera") && STRINGS_ARE_NOT_EQUAL(buffer,"endview"))
   {
      error(abort_action,"Error reading camera");
      return code_bad;
   }

   /* If the camera is named "default," make it the default camera,
    * which is the one that the model is set to when loaded.
    */
   if (STRINGS_ARE_EQUAL(ms->dis.view_name[ms->dis.num_file_views], "default"))
      ms->dis.default_view = ms->dis.num_file_views;

   ms->dis.view_used[ms->dis.num_file_views] = yes;
   ms->dis.num_file_views++;

   return code_fine;
}


static ReturnCode read_gencoord_groups (ModelStruct* ms, FILE* fp, GeneralizedCoord* gencoord)
{
   int num_malloced_so_far = 10;
   
   ReturnCode rc = code_fine;

   gencoord->numgroups = 0;

   gencoord->group = (int*)simm_malloc(num_malloced_so_far * sizeof(int));
   
   if (gencoord->group == NULL)
      return code_bad;

   while (1)
   {
      if (fscanf(fp, "%s", buffer) != 1)
         return code_bad;

      if (STRINGS_ARE_EQUAL(buffer, "endgroups"))
      {
         gencoord->group = (int*)simm_realloc(gencoord->group, gencoord->numgroups * sizeof(int), &rc);
         break;
      }

      if (gencoord->numgroups > num_malloced_so_far)
      {
         num_malloced_so_far += 10;
         
         gencoord->group = (int*)simm_realloc(gencoord->group, num_malloced_so_far * sizeof(int), &rc);
         
         if (rc == code_bad)
            break;
      }

      gencoord->group[gencoord->numgroups] = enter_gencoord_group(ms, buffer, gencoord);
      
      if (gencoord->group[gencoord->numgroups] == -1)
         return code_bad;
      
      gencoord->numgroups++;
   }
   return rc;
}


static ReturnCode read_segment_groups (ModelStruct* ms, FILE* fp, int segnum)
{
   int num_malloced_so_far = 10;
   SegmentStruct* seg = &ms->segment[segnum];
   ReturnCode rc = code_fine;

   seg->numgroups = 0;

   seg->group = (int*) simm_malloc(num_malloced_so_far * sizeof(int));
   
   if (seg->group == NULL)
      return code_bad;

   while (1)
   {
      if (fscanf(fp, "%s", buffer) != 1)
         return code_bad;

      if (STRINGS_ARE_EQUAL(buffer,"endgroups"))
      {
         seg->group = (int*) simm_realloc(seg->group, seg->numgroups * sizeof(int), &rc);

         break;
      }

      if (seg->numgroups > num_malloced_so_far)
      {
         num_malloced_so_far += 10;
         
         seg->group = (int*) simm_realloc(seg->group, num_malloced_so_far * sizeof(int), &rc);
         
         if (rc == code_bad)
            break;
      }

      seg->group[seg->numgroups] = enter_segment_group(ms, buffer, segnum);
      
      if (seg->group[seg->numgroups] == -1)
         return code_bad;
      
      seg->numgroups++;
   }
   return rc;
}


ReturnCode read_drawmode (FILE* fp, DrawingMode* dm)
{
   if (fscanf(fp,"%s", buffer) != 1)
      return code_bad;

   *dm = get_drawmode(buffer);

   if (*dm == -1)
      *dm = gouraud_shading;
   
   return code_fine;
}


static DrawingMode get_drawmode(char buffer[])
{
   if (STRINGS_ARE_EQUAL(buffer,"wireframe"))
      return wireframe;
   else if (STRINGS_ARE_EQUAL(buffer,"solid_fill"))
      return solid_fill;
   else if (STRINGS_ARE_EQUAL(buffer,"flat_shading"))
      return flat_shading;
   else if (STRINGS_ARE_EQUAL(buffer,"gouraud_shading"))
      return gouraud_shading;
   else if (STRINGS_ARE_EQUAL(buffer,"outlined_polygons") ||
        STRINGS_ARE_EQUAL(buffer,"outlined"))
      return outlined_polygons;
   else if (STRINGS_ARE_EQUAL(buffer,"bounding_box"))
      return bounding_box;
   else if (STRINGS_ARE_EQUAL(buffer,"none"))
      return no_surface;
   else
      return -1;
}


static ReturnCode read_contact(ModelStruct* ms, FILE* fp)
{

   while (1)
   {
      if (read_string(fp,buffer) == EOF)
      {
         (void)sprintf(errorbuffer,"Unexpected EOF found reading contact parameters");
         error(abort_action,errorbuffer);
         return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"endcontact"))
      {
         return code_fine;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"spring"))
      {
         if (read_spring(ms, fp) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"spring_floor"))
      {
         if (read_spring_floor(ms, fp) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"object"))
      {
         if (read_contact_object(ms, fp) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"contact_pair"))
      {
         if (read_contact_pair(ms, fp) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"begin_group"))
      {
         if (read_contact_group(ms, fp) == code_bad)
            return code_bad;
      }
   }
   return code_fine;
}


static ReturnCode read_spring(ModelStruct* ms, FILE* fp)
{
   int i, segnum;
    char floorName[CHARBUFFER];
   SpringPoint* sp;
   SegmentStruct* seg;

   if (fscanf(fp,"%s", buffer) != 1)
   {
      sprintf(errorbuffer, "Error reading segment name for spring");
      error(abort_action,errorbuffer);
      return code_bad;
   }
   segnum = enter_segment(ms,buffer,yes);
   seg = &ms->segment[segnum];

   if (seg->numSpringPoints >= seg->springPointArraySize)
   {
      ReturnCode rc = code_fine;

      seg->springPointArraySize += SEGMENT_ARRAY_INCREMENT;
      if (seg->springPoint == NULL)
      {
         seg->springPoint = (SpringPoint*) simm_malloc(
               seg->springPointArraySize * sizeof(SpringPoint));
      }
      else
      {
         seg->springPoint = (SpringPoint*) simm_realloc(seg->springPoint,
               seg->springPointArraySize * sizeof(SpringPoint), &rc);
      }
      if (rc == code_bad || seg->springPoint == NULL)
      {
         seg->springPointArraySize -= SEGMENT_ARRAY_INCREMENT;
         return code_bad;
      }
   }

   sp = &seg->springPoint[seg->numSpringPoints];

   if (fscanf(fp,"%lf %lf %lf %s %lf %lf %lf %lf %lf %lf %lf", &sp->point[0], &sp->point[1], &sp->point[2],
      floorName, &sp->friction, &sp->param_a, &sp->param_b, &sp->param_c,
      &sp->param_d, &sp->param_e, &sp->param_f) != 11)
   {
      sprintf(errorbuffer, "Error reading spring point definition in segment %s", seg->name);
      error(abort_action, errorbuffer);
      return code_bad;
   }
   else
   {
        sp->name = NULL;
        sp->visible = yes;
        sp->segment = segnum;

        // Find the corresponding floor object and add the spring to it.
        for (i=0; i<ms->numsegments; i++)
        {
            if (ms->segment[i].springFloor && STRINGS_ARE_EQUAL(floorName, ms->segment[i].springFloor->name))
                break;
        }
        if (i == ms->numsegments)
        {
            sprintf(errorbuffer, "Unknown spring floor (%s) referenced by spring point (was point defined before floor?)", floorName);
            error(abort_action, errorbuffer);
            return code_bad;
        }
        else
        {
            sp->floorSegment = i;
            add_spring_point_to_floor(seg, sp);
            seg->numSpringPoints++;
        }
   }

   return code_fine;
}


static ReturnCode read_spring_floor(ModelStruct* ms, FILE* fp)
{
   int segnum, count;
   SegmentStruct* seg;
   ReturnCode frc;
   char str1[CHARBUFFER], str2[CHARBUFFER], str3[CHARBUFFER];

   count = fscanf(fp, "%s", str1);
   count += _read_til_tokens(fp, str2, "\t\r\n");
   _strip_outer_whitespace(str2);
   read_line(fp, buffer);
   count += sscanf(buffer, "%s", str3);

   segnum = enter_segment(ms, str3, yes);
   if (segnum < 0)
   {
      sprintf(errorbuffer, "Error reading segment name for spring floor");
      error(abort_action,errorbuffer);
      return code_bad;
   }

   seg = &ms->segment[segnum];

   if (seg->springFloor)
   {
      (void)sprintf(errorbuffer,"Second spring_floor found in segment %s. Only one is allowed per segment.", seg->name);
      error(abort_action, errorbuffer);
      return code_bad;
   }

   if ((seg->springFloor = (SpringFloor *)simm_malloc(sizeof(SpringFloor))) == NULL)
   {
      error(abort_action,"Ran out of memory. Unable to create spring floor.");
      return code_bad;
   }
   else
   {
      mstrcpy(&seg->springFloor->name, str1);
      mstrcpy(&seg->springFloor->filename, str2);
      seg->springFloor->visible = yes;
      seg->springFloor->points = NULL;
      seg->springFloor->numPoints = 0;
      seg->springFloor->pointArraySize = 0;
#if ! ENGINE
      seg->springFloor->poly = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
      frc = lookup_polyhedron(seg->springFloor->poly, seg->springFloor->filename, ms);
      if (frc == file_missing)
      {
         (void)sprintf(errorbuffer,"Unable to locate spring_floor file %s", seg->springFloor->filename);
         error(none,errorbuffer);
         FREE_IFNOTNULL(seg->springFloor->poly);
      }
      else if (frc == file_bad)
      {
         (void)sprintf(errorbuffer,"Unable to read spring_floor from file %s", seg->springFloor->filename);
         error(none,errorbuffer);
         FREE_IFNOTNULL(seg->springFloor->poly);
      }
#endif
   }

   return code_fine;
}

static void add_spring_point_to_floor(SegmentStruct* segment, SpringPoint* point)
{
    if (segment && segment->springFloor)
    {
        SpringFloor* floor = segment->springFloor;

        // Make the list longer if necessary.
        if (floor->numPoints >= floor->pointArraySize)
        {
            ReturnCode rc = code_fine;

            floor->pointArraySize += SEGMENT_ARRAY_INCREMENT;
            if (floor->points == NULL)
                floor->points = (SpringPoint**)simm_malloc(floor->pointArraySize * sizeof(SpringPoint*));
            else
                floor->points = (SpringPoint**)simm_realloc(floor->points, floor->pointArraySize * sizeof(SpringPoint*), &rc);
            if (rc == code_bad || floor->points == NULL)
            {
                floor->pointArraySize -= SEGMENT_ARRAY_INCREMENT;
                return;
            }
        }

        // Add the point to the list.
        sprintf(buffer, "%s_pt%d", segment->springFloor->name, floor->numPoints+1);
        if (!point->name)
            mstrcpy(&point->name, buffer);
        floor->points[floor->numPoints++] = point;
    }
}


static ReturnCode read_force_matte(ModelStruct* ms, SegmentStruct* seg, FILE* fp)
{
   int segnum;
   FileReturnCode frc;
   char str1[CHARBUFFER], str2[CHARBUFFER];
   int count;
   char bufs[3][CHARBUFFER];

   if (seg->forceMatte)
   {
      (void)sprintf(errorbuffer, "Second force_matte found in segment %s. Only one is allowed per segment.", seg->name);
      error(abort_action, errorbuffer);
      return code_bad;
   }

   count = fscanf(fp, "%s", str1);
   count += _read_til_tokens(fp, str2, "\t\r\n");
   _strip_outer_whitespace(str2);
   /* If both the name and filename were specified, look for the visible/invisible keyword. */
   if (count == 2)
   {
      read_line(fp, buffer);
      if (sscanf(buffer, "%s", bufs[0]) > 0)
         count++;
   }

   if (count < 2)
   {
      if (count == 1)
      {
         sprintf(errorbuffer, "Filename undefined for force_matte %s in segment %s.", str1, seg->name);
      }
      error(abort_action,errorbuffer);
      return code_bad;
   }
   else
   {
      if ((seg->forceMatte = (ContactObject*)simm_malloc(sizeof(ContactObject))) == NULL)
      {
         error(abort_action,"Ran out of memory. Unable to create force matte.");
         return code_bad;
      }
      else
      {
         seg->forceMatte->visible = yes;
         mstrcpy(&seg->forceMatte->name, str1);
         mstrcpy(&seg->forceMatte->filename, str2);
         
#if ! ENGINE || OPENSMAC
         seg->forceMatte->poly = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
         frc = lookup_polyhedron(seg->forceMatte->poly, seg->forceMatte->filename, ms);
         if (frc == file_missing)
         {
            (void)sprintf(errorbuffer,"Unable to locate force_matte file %s", seg->forceMatte->filename);
            error(none,errorbuffer);
            FREE_IFNOTNULL(seg->forceMatte->poly);
         }
         else if (frc == file_bad)
         {
            (void)sprintf(errorbuffer,"Unable to read force_matte from file %s", seg->forceMatte->filename);
            error(none,errorbuffer);
            FREE_IFNOTNULL(seg->forceMatte->poly);
         }
#endif
      }
      /* If visible/invisible keyword specified ... */
      if (count == 3)
      {
         if (STRINGS_ARE_EQUAL(bufs[0], "invisible") || STRINGS_ARE_EQUAL(bufs[0], "INVISIBLE") ||
            STRINGS_ARE_EQUAL(bufs[0], "hidden") || STRINGS_ARE_EQUAL(bufs[0], "HIDDEN"))
            seg->forceMatte->visible = no;
         if (STRINGS_ARE_EQUAL(bufs[0], "visible") || STRINGS_ARE_EQUAL(bufs[0], "VISIBLE"))
            seg->forceMatte->visible = yes;
      }
   }

   return code_fine;
}


static ReturnCode read_contact_object(ModelStruct* ms, FILE* fp)
{
   int segnum, count;
   ContactObject* co;
   SegmentStruct* seg;
   char segname[CHARBUFFER], co_name[CHARBUFFER], co_file[CHARBUFFER];
   char bufs[3][CHARBUFFER];
   ReturnCode rc;
   FileReturnCode frc;

   count = fscanf(fp, "%s", co_name);
   count += _read_til_tokens(fp, co_file, "\t\r\n");
   _strip_outer_whitespace(co_file);
   read_line(fp, buffer);
   count += sscanf(buffer, "%s %s", segname, bufs[0]);

   if (count < 3)
   {
      if (count == 1)
         sprintf(errorbuffer, "No filename or segment defined for contact object %s.", co_name);
      else if (count == 2)
         sprintf(errorbuffer, "No segment defined for contact object %s.", co_name);
      else
         sprintf(errorbuffer, "Contact object undefined.");
      error(abort_action,errorbuffer);
      return code_bad;
   }
   else  //count = 3 or 4
   {      
      segnum = enter_segment(ms, segname, yes);
      seg = &ms->segment[segnum];

      if (seg->numContactObjects >= seg->contactObjectArraySize)
      {
         rc = code_fine;

         seg->contactObjectArraySize += SEGMENT_ARRAY_INCREMENT;
         if (seg->contactObject == NULL)
         {
            seg->contactObject = (ContactObject*) simm_malloc(
               seg->contactObjectArraySize * sizeof(ContactObject));
         }
         else
         {
            seg->contactObject = (ContactObject*) simm_realloc(seg->contactObject,
               seg->contactObjectArraySize * sizeof(ContactObject), &rc);
         }
         if (rc == code_bad || seg->contactObject == NULL)
         {
            seg->contactObjectArraySize -= SEGMENT_ARRAY_INCREMENT;
            return code_bad;
         }
      }

      co = &seg->contactObject[seg->numContactObjects];

      co->visible = yes;
      mstrcpy(&co->name, co_name);
      mstrcpy(&co->filename, co_file);
#if ! ENGINE
      co->poly = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
      frc = lookup_polyhedron(co->poly, co->filename, ms);
      if (frc == file_bad)
      {
         (void)sprintf(errorbuffer, "Unable to read contact object from file %s", co->filename);
         error(none, errorbuffer);
         FREE_IFNOTNULL(co->poly);
      }
      else if (frc == file_missing)
      {
         (void)sprintf(errorbuffer, "Unable to locate contact object file %s", co->filename);
         error(none, errorbuffer);
         FREE_IFNOTNULL(co->poly);
      }
#endif
      /* If visible/invisible keyword was specified... */
      if (count == 4)
      {
         if (STRINGS_ARE_EQUAL(bufs[0], "invisible") || STRINGS_ARE_EQUAL(bufs[0], "INVISIBLE") ||
            STRINGS_ARE_EQUAL(bufs[0], "hidden") || STRINGS_ARE_EQUAL(bufs[0], "HIDDEN"))
            co->visible = no;
         if (STRINGS_ARE_EQUAL(bufs[0], "visible") || STRINGS_ARE_EQUAL(bufs[0], "VISIBLE"))
            co->visible = yes;
      }
      seg->numContactObjects++;
   }

   return code_fine;
}


static ReturnCode read_contact_pair(ModelStruct* ms, FILE* fp)
{
   ReturnCode rc;
   ContactPair* cp;
   double restitution, mu_static, mu_dynamic;

   if (fscanf(fp,"%s %s %lf %lf %lf", buffer, msg,  &restitution, &mu_static, &mu_dynamic) != 5)
   {
      sprintf(errorbuffer, "Error reading contact pair");
      error(abort_action,errorbuffer);
      return code_bad;
   }

   if (ms->numContactPairs >= ms->contactPairArraySize)
   {
      rc = code_fine;

      ms->contactPairArraySize += SEGMENT_ARRAY_INCREMENT;
      if (ms->contactPair == NULL)
      {
         ms->contactPair = (ContactPair*) simm_malloc(
               ms->contactPairArraySize * sizeof(ContactPair));
      }
      else
      {
         ms->contactPair = (ContactPair*) simm_realloc(ms->contactPair,
               ms->contactPairArraySize * sizeof(ContactPair), &rc);
      }
      if (rc == code_bad || ms->contactPair == NULL)
      {
         ms->contactPairArraySize -= SEGMENT_ARRAY_INCREMENT;
         return code_bad;
      }
   }

   cp = &ms->contactPair[ms->numContactPairs];

   mstrcpy(&cp->body1, buffer);
   mstrcpy(&cp->body2, msg);
   cp->restitution = restitution;
   cp->mu_static = mu_static;
   cp->mu_dynamic = mu_dynamic;

   ms->numContactPairs++;

   return code_fine;
}


static ReturnCode read_contact_group(ModelStruct* ms, FILE* fp)
{
   ReturnCode rc;
   ContactGroup* cg;

   if (fscanf(fp,"%s", buffer) != 1)
   {
      sprintf(errorbuffer, "Error reading name of contact group");
      error(abort_action,errorbuffer);
      return code_bad;
   }

   if (ms->numContactGroups >= ms->contactGroupArraySize)
   {
      rc = code_fine;

      ms->contactGroupArraySize += SEGMENT_ARRAY_INCREMENT;
      if (ms->contactGroup == NULL)
      {
         ms->contactGroup = (ContactGroup*) simm_malloc(
               ms->contactGroupArraySize * sizeof(ContactGroup));
      }
      else
      {
         ms->contactGroup = (ContactGroup*) simm_realloc(ms->contactGroup,
               ms->contactGroupArraySize * sizeof(ContactGroup), &rc);
      }
      if (rc == code_bad || ms->contactGroup == NULL)
      {
         ms->contactGroupArraySize -= SEGMENT_ARRAY_INCREMENT;
         return code_bad;
      }
   }

   cg = &ms->contactGroup[ms->numContactGroups];

   mstrcpy(&cg->name, buffer);
   cg->numElements = 0;
   cg->element = NULL;

   while (1)
   {
      if (fscanf(fp,"%s", buffer) != 1)
      {
            sprintf(errorbuffer,"Error reading elements of group %s\n", cg->name);
         error(abort_action,errorbuffer);
         return code_bad;
      }

        if (STRINGS_ARE_EQUAL(buffer,"end_group"))
        {
            break;
        }
      cg->numElements++;
        if (cg->element == NULL)
            cg->element = (char**)simm_malloc(sizeof(char*));
        else
            cg->element = (char**)simm_realloc(cg->element,
                          cg->numElements*sizeof(char*),&rc);

        mstrcpy(&cg->element[cg->numElements-1], buffer);
   }

   ms->numContactGroups++;

   return code_fine;
}

// Note: If the bone file is missing or bad, the polyhedron is still added to the model
// so that its filename is preserved (so it can be written back to the JNT file).
static PolyhedronStruct* make_bone(ModelStruct* ms, SegmentStruct* seg, char filename[])
{
   ReturnCode rc = code_fine;
   FileReturnCode frc;
   PolyhedronStruct* ph = NULL;

   if (seg->numBones >= seg->boneArraySize)
   {
      seg->boneArraySize += SEGMENT_FILE_ARRAY_INCREMENT;
      if (seg->bone == NULL)
      {
         seg->bone = (PolyhedronStruct*)simm_malloc(seg->boneArraySize*sizeof(PolyhedronStruct));
      }
      else
      {
         seg->bone = (PolyhedronStruct*)simm_realloc(seg->bone, seg->boneArraySize*sizeof(PolyhedronStruct), &rc);
         if (rc == code_bad || seg->bone == NULL)
         {
            seg->boneArraySize -= SEGMENT_FILE_ARRAY_INCREMENT;
            return NULL;
         }
      }
   }

   ph = &seg->bone[seg->numBones++];

   preread_init_polyhedron(ph);

   frc = lookup_polyhedron(ph, filename, ms);
   if (frc == file_bad)
       simm_printf(yes, "Unable to read bone from file %s\n", filename);
   else if (frc == file_missing)
       simm_printf(yes, "Unable to locate bone file %s\n", filename);

   return ph;
}

static ReturnCode read_marker(ModelStruct* ms, SegmentStruct* seg, FILE* fp)
{
   int i, count;
   Marker* m;
   char bufs[3][CHARBUFFER];

   _read_til_tokens(fp, buffer, "\t\r\n");
   _strip_outer_whitespace(buffer);

   if (strlen(buffer) < 1)
   {
      sprintf(errorbuffer, "Error reading marker name");
      error(abort_action,errorbuffer);
      return code_bad;
   }

   if (seg->numMarkers >= seg->markerArraySize)
   {
      ReturnCode rc = code_fine;

      seg->markerArraySize += SEGMENT_ARRAY_INCREMENT;
      if (seg->marker == NULL)
      {
         seg->marker = (Marker**)simm_malloc(seg->markerArraySize * sizeof(Marker*));
      }
      else
      {
         seg->marker = (Marker**)simm_realloc(seg->marker, seg->markerArraySize * sizeof(Marker*), &rc);
      }
      if (rc == code_bad || seg->marker == NULL)
      {
         seg->markerArraySize -= SEGMENT_ARRAY_INCREMENT;
         return code_bad;
      }
   }

   m = seg->marker[seg->numMarkers] = (Marker*)simm_calloc(1, sizeof(Marker));
   m->visible = ms->marker_visibility;
   m->selected = no;
   m->fixed = no;

   read_line(fp, bufs[0]);
   count = sscanf(bufs[0], "%lf %lf %lf %lf %s %s", &m->offset[XX], &m->offset[YY], &m->offset[ZZ],
                  &m->weight, bufs[1], bufs[2]);
   if (count < 4)
   {
      if (count == 3)
      {
         sprintf(errorbuffer, "Weight undefined for marker %s in segment %s.", buffer, seg->name);
      }
      else
      {
         sprintf(errorbuffer, "Error reading definition of marker %s in segment %s.", buffer, seg->name);
      }
      error(abort_action,errorbuffer);
      return code_bad;
   }
   else
   {
      /* The visible, invisible, and fixed keywords can be in any order after the marker weight. */
      for (i = 1; i < count - 3; i++)
      {
         if (STRINGS_ARE_EQUAL(bufs[i], "invisible") || STRINGS_ARE_EQUAL(bufs[i], "INVISIBLE") ||
             STRINGS_ARE_EQUAL(bufs[i], "hidden") || STRINGS_ARE_EQUAL(bufs[i], "HIDDEN"))
            m->visible = no;
         if (STRINGS_ARE_EQUAL(bufs[i], "visible") || STRINGS_ARE_EQUAL(bufs[i], "VISIBLE"))
            m->visible = yes;
         if (STRINGS_ARE_EQUAL(bufs[i], "fixed") || STRINGS_ARE_EQUAL(bufs[i], "FIXED"))
            m->fixed = yes;
      }
      mstrcpy(&m->name, buffer);
        m->visible = ms->marker_visibility;
      seg->numMarkers++;
   }

   return code_fine;

}


static ReturnCode read_constraint(ModelStruct *ms, FILE* fp)
{
   ReturnCode rc = code_fine;
   ConstraintObject *co;
   double xyz[3];
   DMatrix m;
   int i;
   
   if (ms->num_constraint_objects == ms->constraint_object_array_size)
   {
      ms->constraint_object_array_size += CONSTRAINT_OBJECT_ARRAY_INCREMENT;
      ms->constraintobj = (ConstraintObject*)simm_realloc(ms->constraintobj,
         ms->constraint_object_array_size*sizeof(ConstraintObject),&rc);
      
      if (rc == code_bad)
      {
         ms->constraint_object_array_size -= CONSTRAINT_OBJECT_ARRAY_INCREMENT;
         return code_bad;
      }
   }
   
   co = &ms->constraintobj[ms->num_constraint_objects];
   
   initconstraintobject(co);

   /* read constraint name */
   if (fscanf(fp,"%s", buffer) != 1)
   {
      error(abort_action,"Error reading name in constraint definition");
      return code_bad;
   }
   else
      mstrcpy(&co->name,buffer);


   while (1)
   {
      if (read_string(fp,buffer) == EOF)
         break;
      
      if (buffer[0] == '#')
      {
         read_nonempty_line(fp,buffer);
         continue;
      }

      if (STRINGS_ARE_EQUAL(buffer, "constrainttype"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"sphere"))
            co->constraintType = constraint_sphere;
         else if (STRINGS_ARE_EQUAL(buffer,"cylinder"))
            co->constraintType = constraint_cylinder;
         else if (STRINGS_ARE_EQUAL(buffer,"ellipsoid"))
            co->constraintType = constraint_ellipsoid;
         else if (STRINGS_ARE_EQUAL(buffer,"plane"))
            co->constraintType = constraint_plane;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"segment"))
      {
         if (fscanf(fp,"%s", buffer) != 1)
         {
            (void)sprintf(errorbuffer,"Error reading segment in definition of constraint object %s",
               co->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         else
         {
            co->segment = enter_segment(ms,buffer,yes);
            
            if (co->segment == -1)
               return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"active"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"false"))
            co->active = no;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"visible"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"false"))
            co->visible = no;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"rotation"))
      {
         sprintf(errorbuffer, "Error reading rotation for constraint object %s:", co->name);
         error(none,errorbuffer);
         error(abort_action,"  The \'rotation\' keyword has been changed to \'xyz_body_rotation\'.");
         return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"xyz_body_rotation"))
      {
         if (fscanf(fp, "%lg %lg %lg", &xyz[0], &xyz[1], &xyz[2]) != 3)
         {
            (void)sprintf(errorbuffer, "Error reading rotation for constraint object %s",
               co->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         identity_matrix(m);
         x_rotate_matrix_bodyfixed(m, xyz[0] * DTOR);
         y_rotate_matrix_bodyfixed(m, xyz[1] * DTOR);
         z_rotate_matrix_bodyfixed(m, xyz[2] * DTOR);
         extract_rotation(m, &co->rotationAxis, &co->rotationAngle);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"xyz_space_rotation"))
      {
         if (fscanf(fp, "%lg %lg %lg", &xyz[0], &xyz[1], &xyz[2]) != 3)
         {
            (void)sprintf(errorbuffer, "Error reading rotation for constraint object %s",
               co->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         identity_matrix(m);
         x_rotate_matrix_spacefixed(m, xyz[0] * DTOR);
         y_rotate_matrix_spacefixed(m, xyz[1] * DTOR);
         z_rotate_matrix_spacefixed(m, xyz[2] * DTOR);
         extract_rotation(m, &co->rotationAxis, &co->rotationAngle);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"rotationaxis"))
      {
         fscanf(fp, "%lg %lg %lg", &co->rotationAxis.xyz[0],
            &co->rotationAxis.xyz[1], &co->rotationAxis.xyz[2]);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"rotationangle"))
      {
         if (fscanf(fp, "%lg", &co->rotationAngle) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading rotation for constraint object %s",
               co->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"translation") ||
         STRINGS_ARE_EQUAL(buffer,"translate"))
      {
         if (fscanf(fp,"%lg %lg %lg", &co->translation.xyz[0],
            &co->translation.xyz[1], &co->translation.xyz[2]) != 3)
         {
            (void)sprintf(errorbuffer, "Error reading translation for constraint object %s",
               co->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"radius"))
      {
         if (co->constraintType == constraint_ellipsoid)
         {
            rc = ((fscanf(fp,"%lg %lg %lg", &co->radius.xyz[0],
                          &co->radius.xyz[1], &co->radius.xyz[2]) == 3) ? code_fine : code_bad);
            if (co->radius.xyz[0] <= 0.0 || co->radius.xyz[1] <= 0.0 || co->radius.xyz[2] <= 0.0)
            {
               error(none, "Value must be greater than zero.");
               rc = code_bad;
            }
         }
         else if (co->constraintType == constraint_plane)
         {
            rc = ((fscanf(fp,"%lg %lg", &co->radius.xyz[0], &co->radius.xyz[1]) == 2) ? code_fine : code_bad);
            if (co->radius.xyz[0] <= 0.0 || co->radius.xyz[1] <= 0.0)
            {
               error(none, "Value must be greater than zero.");
               rc = code_bad;
            }
         }
         else
         {
            rc = ((fscanf(fp,"%lg", &co->radius.xyz[0]) == 1) ? code_fine : code_bad);
            if (co->radius.xyz[0] <= 0.0)
            {
               error(none, "Value must be greater than zero.");
               rc = code_bad;
            }
         }
         
         if (rc == code_bad)
         {
            (void)sprintf(errorbuffer, "Error reading radius for constraint object %s",
               co->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"height"))
      {
         int num = fscanf(fp,"%lg", &co->height);

         if (num == 1 && co->height <= 0.0)
         {
            error(none, "Value must be greater than zero.");
            num = -1;
         }

         if (num != 1)
         {
            (void)sprintf(errorbuffer, "Error reading cylinder height for constraint object %s",
               co->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"quadrant"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         set_constraint_specs(co, buffer);
      }
/*      else if (STRINGS_ARE_EQUAL(buffer, "plane"))
      {
         double a, b, c, d;
         if (fscanf(fp, "%s %lf %lf %lf %lf", buffer, &a, &b, &c, &d) != 4)
         {
            (void)sprintf(errorbuffer,
               "Error reading definition of plane (segment a b c d) in definition of constraint %s",
               co->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         else
         {
            co->constraintType = constraint_plane;
            
//            co->plane.segment = enter_segment(ms, buffer, yes);
//            co->plane.plane.a = a;
//            co->plane.plane.b = b;
//            co->plane.plane.c = c;
//            co->plane.plane.d = d;
         }
      }*/
      else if (STRINGS_ARE_EQUAL(buffer, "beginpoints"))
      {
         co->points = read_constraint_points(ms, fp, &co->numPoints,
            &co->cp_array_size);
         if (co->points == NULL)
         {
            (void)sprintf(errorbuffer, "Error reading points for constraint %s.",
               co->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
 /*        for (i = 0; i < co->numPoints; i++)
         {
            if (co->points[i].segment != co->points[0].segment)
            {
            (void)sprintf(errorbuffer, "Constraint points in %s must all be on same segment.",
               co->name);
            error(abort_action, errorbuffer);
            return code_bad;
            }
         }*/
      }
      else if (STRINGS_ARE_EQUAL(buffer,"endconstraintobject"))
         break;
      else
      {
         (void)sprintf(errorbuffer, "Unknown string %s in constraint %s.",
            buffer, co->name);
         error(none, errorbuffer);
      }
   }
   co->undeformed_translation = co->translation;
   
   co->xforms_valid = no;
   /* set constraint point activations and visibility to that of co if co is inactive/invisible*/
//dkb nov 2009
   /*
   for (i = 0; i < co->numPoints; i++)
   {
      if (co->visible == no)
         co->points[i].visible = co->visible;
      if (co->active == no)
         co->points[i].active = co->active;
   }*/
   
   if (co->constraintType == constraint_none)
   {
      (void)sprintf(errorbuffer, "No constraint type (plane, constraint_object) specified for %s.",
         co->name);
      error(abort_action, errorbuffer);
//      return code_bad;
   }
   ms->num_constraint_objects++;
   
   return code_fine;
   
}

static ConstraintPoint* read_constraint_points(ModelStruct* ms, FILE* fp, int *numpoints,
                       int *cp_array_size)
{
   int num;
   double x, y, z, w, tol;
   ConstraintPoint *cp;
   char buf2[CHARBUFFER], point_name[CHARBUFFER], segment_name[CHARBUFFER];
   ReturnCode rc = code_fine;

   *numpoints = 0;
   *cp_array_size = CP_ARRAY_INCREMENT;

   cp = (ConstraintPoint*)simm_malloc((*cp_array_size)*sizeof(ConstraintPoint));
   if (cp == NULL)
      return (NULL);
   
   while (1)
   {
      if (read_nonempty_line(fp, buffer) == EOF)
      {
         error(none, "EOF while reading constraint points");
         return (NULL);
      }
      
      sscanf(buffer, "%s", buf2);
      if (STRINGS_ARE_EQUAL(buf2, "endpoints"))
         return (cp);
      
      /* Check to see if you need to increase the size of the muscle-point
       * array before reading any more points.
       */
      if ((*numpoints) >= (*cp_array_size))
      {
         (*cp_array_size) += CP_ARRAY_INCREMENT;
         cp = (ConstraintPoint*)simm_realloc(cp,(*cp_array_size)*sizeof(ConstraintPoint),&rc);
         if (rc == code_bad)
         {
            (*cp_array_size) -= CP_ARRAY_INCREMENT;
            return (NULL);
         }
      }
      initconstraintpoint(&cp[*numpoints]);
     
      /* If the string you just read was not "endpoints" then it must
       * be the start of a new point. */
      /* read point name, offset, weight, and segment name and tolerance */
      num = sscanf(buffer, "%s %lf %lf %lf %lf %s %lf", point_name, &x, &y, &z, &w,
         segment_name, &tol);
      if ((num != 6) && (num != 7))
         return NULL;

      cp[*numpoints].offset[XX] = x;
      cp[*numpoints].offset[YY] = y;
      cp[*numpoints].offset[ZZ] = z;
      cp[*numpoints].weight = w;
//      cp[*numpoints].visible = yes;
//      cp[*numpoints].active = yes;
      if (num == 7)
         cp[*numpoints].tolerance = tol;
      mstrcpy(&cp[*numpoints].name, point_name);
      cp[*numpoints].segment = enter_segment(ms, segment_name, yes);
      if (cp[*numpoints].segment == -1)
      {
         (void)sprintf(errorbuffer,"Memory error while reading constraint points.", segment_name);
         error(none,errorbuffer);
         return (NULL);
      }
      (*numpoints)++;
   }
}

/* -------------------------------------------------------------------------
   read_motion_object - 
---------------------------------------------------------------------------- */
static ReturnCode read_motion_object(ModelStruct* ms, FILE* fp)
{
   int i;
   MotionObject* mo;
   ReturnCode rc = code_fine;

   /* read the motion object's name:
    */
   if (fscanf(fp,"%s", buffer) != 1)
   {
      error(abort_action, "Error reading name in motion object definition");
      return code_bad;
   }
   
   /* scan the list of existing motion objects to see if one already exists
    * with the name we just read:
    */
   for (i = 0; i < ms->num_motion_objects; i++)
      if (STRINGS_ARE_EQUAL(buffer, ms->motion_objects[i].name))
         break;
   
   /* create a new motion object if necessary:
    */
   if (i == ms->num_motion_objects)
   {
      i = ms->num_motion_objects++;
   
      if (ms->motion_objects == NULL)
         ms->motion_objects = (MotionObject*) simm_malloc(ms->num_motion_objects * sizeof(MotionObject));
      else
         ms->motion_objects = (MotionObject*) simm_realloc(ms->motion_objects,
                                                           ms->num_motion_objects * sizeof(MotionObject),
                                                           &rc);
      if (ms->motion_objects == NULL)
      {
         ms->num_motion_objects = 0;
         return code_bad;
      }
      mo = &ms->motion_objects[i];
      
      memset(mo, 0, sizeof(MotionObject));
   
      mstrcpy(&mo->name, buffer);
      
      mstrcpy(&mo->materialname, "def_motion");
      mo->material = enter_material(ms, mo->materialname, declaring_element);

      for (i = XX; i <= ZZ; i++)
      {
         mo->startingPosition[i] = 0.0;
         mo->startingScale[i] = 1.0;
         mo->startingXYZRotation[i] = 0.0;
      }
      mo->drawmode = gouraud_shading;
      mo->vector_axis = YY;
   }
   else
      mo = &ms->motion_objects[i];
      
   mo->defined_in_file = yes;
   
   /* read the motion object's parameters:
    */
   while (1)
   {
      if (read_string(fp, buffer) == EOF || STRINGS_ARE_EQUAL(buffer, "endmotionobject"))
         break;

      if (buffer[0] == '#')
      {
         read_nonempty_line(fp,buffer);
         continue;
      }

      if (STRINGS_ARE_EQUAL(buffer, "filename"))
      {
         if (fscanf(fp, "%s", buffer) != 1)
         {
            simm_printf(yes, "Error reading filename for motion object: %s.\n", mo->name);
            return code_bad;
         }
         FREE_IFNOTNULL(mo->filename);
         mstrcpy(&mo->filename, buffer);
#if ! OPENSMAC && ! ENGINE
         free_polyhedron(&mo->shape, no, ms);
         if (lookup_polyhedron(&mo->shape, mo->filename, ms) != code_fine)
         {
            simm_printf(yes, "Error reading motion object: %s\n", mo->filename);
            return code_bad;
         }
#endif
      }
      if (STRINGS_ARE_EQUAL(buffer, "position") || STRINGS_ARE_EQUAL(buffer,"origin"))
      {
         if (fscanf(fp, "%lg %lg %lg", &mo->startingPosition[0],
                                        &mo->startingPosition[1],
                                        &mo->startingPosition[2]) != 3)
         {
            simm_printf(yes, "Error reading position for motion object: %s.\n", mo->name);
            return code_bad;
         }
      }
      if (STRINGS_ARE_EQUAL(buffer, "scale"))
      {
         if (fscanf(fp, "%lg %lg %lg", &mo->startingScale[0],
                                        &mo->startingScale[1],
                                        &mo->startingScale[2]) != 3)
         {
            simm_printf(yes, "Error reading scale for motion object: %s.\n", mo->name);
            return code_bad;
         }
      }
      if (STRINGS_ARE_EQUAL(buffer, "xyzrotation"))
      {
         if (fscanf(fp, "%lg %lg %lg", &mo->startingXYZRotation[0],
                                        &mo->startingXYZRotation[1],
                                        &mo->startingXYZRotation[2]) != 3)
         {
            simm_printf(yes, "Error reading xyzrotation for motion object: %s.\n", mo->name);
            return code_bad;
         }
      }
      if (STRINGS_ARE_EQUAL(buffer, "material"))
      {
         if (fscanf(fp, "%s", buffer) != 1)
         {
            simm_printf(yes, "Error reading material for motion object: %s.\n", mo->name);
            return code_bad;
         }
         FREE_IFNOTNULL(mo->materialname);
            free(mo->materialname);

         mstrcpy(&mo->materialname, buffer);

         mo->material = enter_material(ms, mo->materialname, declaring_element);
      }
      if (STRINGS_ARE_EQUAL(buffer, "drawmode"))
      {
         if (read_drawmode(fp, &mo->drawmode) != code_fine)
         {
            simm_printf(yes,"Error reading drawmode for motion object: %s.\n", mo->name);
            return code_bad;
         }
      }
      if (STRINGS_ARE_EQUAL(buffer, "vectoraxis"))
      {
         if (fscanf(fp, "%s", buffer) != 1)
         {
            simm_printf(yes, "Error reading vectoraxis for motion object: %s.\n", mo->name);
            return code_bad;
         }
         switch (buffer[0])
         {
            case 'x': case 'X': mo->vector_axis = XX; break;
            case 'y': case 'Y': mo->vector_axis = YY; break;
            case 'z': case 'Z': mo->vector_axis = ZZ; break;
         }
      }
   }
   return rc;
}
