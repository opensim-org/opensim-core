/*******************************************************************************

   SIMMTOOPENSIM.C

   Author: Peter Loan, Frank C. Anderson

   Date: November 10, 2006

   Description:
    Reads in SIMM joint and muscle files and writes an OpenSim model file.
    
   Routines:

*******************************************************************************/

#include <float.h>
#include <errno.h>

#include "universal.h"
#include "functions.h"
#include "wefunctions.h"
#include "sdfunctions.h"


/*************** DEFINES (for this file only) *********************************/
#define GROUND_NAME "ground"

typedef struct {
   int jointnum;
   int dofnum;
    SBoolean constrained;
} GencoordInfo;

/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static double gWorld_x[] = {1.0, 0.0, 0.0, 0.0};
static double gWorld_y[] = {0.0, 1.0, 0.0, 0.0};
static double gWorld_z[] = {0.0, 0.0, 1.0, 0.0};

/*************** GLOBAL VARIABLES (used in only a few files) ******************/


/*************** EXTERNED VARIABLES (declared in another .c file) *************/
extern SDSegment* SDseg;
extern int num_SD_segs;
extern int* joint_order;
extern int gAngleUnits;
extern char buffer[];

/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void make_string_xml_safe(char str[]);
static void make_names_xml_safe(ModelStruct* ms);
static void write_xml_gravity(FILE* fp, ModelStruct* ms);
static void write_xml_muscle_groups(FILE* fp, ModelStruct* ms);
static void write_xml_muscle(FILE* fp, ModelStruct* ms, dpMuscleStruct* m, const char* muscleClassName, int writingDefault, int angleUnits);
static void write_xml_muscles(FILE* fp, ModelStruct* ms, int angleUnits);
static void write_xml_markers(FILE* fp, ModelStruct* ms, const char* markerSetOut);
static void write_vtk_bone(PolyhedronStruct* bone, char geometryDirectory[], char filename[]);
static int segment_has_wrap_objects(ModelStruct* ms, SegmentStruct* seg);
static void write_xml_wrap_object(FILE* fp, dpWrapObject* wo, int angleUnits);
static void write_xml_units(FILE* fp, ModelStruct* ms);
static void write_xml_defaults(FILE* fp, ModelStruct* ms, int angleUnits);
static void extract_joint_locations_and_orientations(ModelStruct* ms,
               JointStruct* joint, int dof_order[], double locationInParent[],
               double orientationInParent[], double locationInChild[], double orientationInChild[]);
static ReturnCode write_simbody_engine(FILE* fp, ModelStruct* ms, char geometryDirectory[], const char* markerSetOut, int angleUnits);
static void write_opensim_function(FILE* fp, dpFunction* function, double ind_conv, double dep_conv, int tab_level);
static void write_opensim_constant_function(FILE* fp, double value, int tab_level);
static SBoolean function_is_simple_linear(dpFunction* function);
static ReturnCode make_simbody_model(FILE* fp, ModelStruct* ms, char geometryDirectory[], int angleUnits);
static void make_simbody_joint(ModelStruct* ms, FILE* fp, JointSDF* jntsdf, int jointnum, int sdnum,
                               SegmentSDF segs[], GencoordInfo gcInfo[], int* dofcount, int* constrainedcount,
                               char geometryDirectory[], int angleUnits);
static void write_opensim_ground_body(FILE* fp, ModelStruct* ms, SegmentStruct* seg, char geometryDirectory[], int angleUnits);
static void write_opensim_coordinate(FILE* fp, ModelStruct* ms, JointStruct* joint, int dofIndex, int angleUnits);
static void write_opensim_transformAxis(FILE* fp, ModelStruct* ms, JointStruct* joint, int dofIndex,
                                        Direction dir, SBoolean writeFunction, int current_dof, int angleUnits);
static void write_opensim_constraint(FILE* fp, ModelStruct* ms, DofStruct* dof, int dofIndex, int angleUnits);
static void convert_fixed_joints(ModelStruct* ms, GencoordInfo gcInfo[]);
static dpFunction* add_constant_function_to_model(ModelStruct* ms, double y_value);
void convert_dof_to_function(ModelStruct* ms, DofStruct* dof, GeneralizedCoord* gencoord);
static SBoolean joint_is_fixed(JointStruct* joint);


ReturnCode write_opensim_model(ModelStruct* ms, char filename[], char geometryDirectory[], const char* markerSetOut, int angleUnits)
{
   int i;
    FILE* fp;
   ReturnCode rc = code_fine;

   // First see if all of the joints can be converted to OpenSim joints.
   // If any joint has a [function] translation after a rotation, it cannot be converted.
   // TODO2.0: it's OK to have a translation at the end of the order if the
   // joint will be processed in reverse.
   for (i=0; i<ms->numjoints; i++)
   {
      if (ms->joint[i].order[0] > 0)
      {
         if (ms->joint[i].dofs[TX].type == function_dof ||
            ms->joint[i].dofs[TY].type == function_dof ||
            ms->joint[i].dofs[TZ].type == function_dof)
         {
            sprintf(buffer, "Joint %s cannot be converted to OpenSim because it contains a [function] translation after a rotation.\n", ms->joint[i].name);
            message(buffer, 0, DEFAULT_MESSAGE_X_OFFSET);
            return code_bad;
         }
      }
   }

   // Make sure the name of the ground segment is "ground".
   if (STRINGS_ARE_NOT_EQUAL(ms->segment[ms->ground_segment].name, GROUND_NAME))
   {
      free(ms->segment[ms->ground_segment].name);
      mstrcpy(&ms->segment[ms->ground_segment].name, GROUND_NAME);
   }

    // Make sure names are legal: meeting the following criteria Alphanumeric with _ or - or . allowed
    make_names_xml_safe(ms);

    fp = fopen(filename, "w");
    fprintf(fp, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    fprintf(fp, "<OpenSimDocument Version=\"10902\">\n");
    fprintf(fp, "<Model name=\"%s\">\n", ms->name);
    write_xml_defaults(fp, ms, angleUnits);
    //if (angleUnits == DEGREES)
    //  fprintf(fp, "\t<angle_units> degrees </angle_units>\n");
    //else
    //  fprintf(fp, "\t<angle_units> radians </angle_units>\n");
    write_xml_units(fp, ms);
    write_xml_muscles(fp, ms, angleUnits);
   rc = write_simbody_engine(fp, ms, geometryDirectory, markerSetOut, angleUnits);
    fprintf(fp, "</Model>\n");
    fprintf(fp, "</OpenSimDocument>\n");
    fclose(fp);

   if (rc == code_bad)
      remove(filename);

   return rc;
}

#if SIMMTOOPENSIM || OPENSMAC

void convert_string(char str[], SBoolean prependUnderscore)
{
   int i, len;

   len = strlen(str);

   for (i = 0; i < len; i++)
   {
       if (str[i]=='>' || str[i]=='<' || str[i]=='&')
           str[i]='_';
   }
}

#endif

// Fix names in the model to take out XML meta characters that cause parsing problems (bug 505). -Ayman
static void make_names_xml_safe(ModelStruct* ms)
{
    int i, j;

   for (i = 0; i < ms->numsegments; i++)
   {
      SegmentStruct* seg = &ms->segment[i];
      char* name = ms->segment[i].name;
        convert_string(name, yes);
      for (j=0; j<ms->segment[i].numMarkers; j++)
      {
         char* markerName = ms->segment[i].marker[j]->name;
            convert_string(markerName, yes);
      }
   }

    for (i = 0; i < ms->numgencoords; i++)
    {
        char* name = ms->gencoord[i]->name;
        convert_string(name, yes);
    }

    for (i = 0; i < ms->numjoints; i++)
    {
        char* name = ms->joint[i].name;
        convert_string(name, yes);
    }

    for (i = 0; i < ms->nummuscles; i++)
    {
        char* name = ms->muscle[i]->name;
        convert_string(name, yes);
    }

    for (i = 0; i < ms->numgroups; i++)
    {
        char* name = ms->muscgroup[i].name;
        convert_string(name, yes);
    }

    for (i = 0; i < ms->num_wrap_objects; i++)
    {
        char* name = ms->wrapobj[i]->name;
        convert_string(name, yes);
    }
}


static void write_xml_gravity(FILE* fp, ModelStruct* ms)
{
    if (ms->gravity == smX)
        fprintf(fp, "\t\t\t<gravity>9.80665 0.0 0.0</gravity>\n");
    else if (ms->gravity == smNegX)
        fprintf(fp, "\t\t\t<gravity>-9.80665 0.0 0.0</gravity>\n");
    else if (ms->gravity == smY)
        fprintf(fp, "\t\t\t<gravity>0.0 9.80665 0.0</gravity>\n");
    else if (ms->gravity == smNegY)
        fprintf(fp, "\t\t\t<gravity>0.0 -9.80665 0.0</gravity>\n");
    else if (ms->gravity == smZ)
        fprintf(fp, "\t\t\t<gravity>0.0 0.0 9.80665</gravity>\n");
    else if (ms->gravity == smNegZ)
        fprintf(fp, "\t\t\t<gravity>0.0 0.0 -9.80665</gravity>\n");
    else if (ms->gravity == smNoAlign)
        fprintf(fp, "\t\t\t<gravity>0.0 0.0 0.0</gravity>\n");
    else // -Y is the default
        fprintf(fp, "\t\t\t<gravity>0.0 -9.80665 0.0</gravity>\n");
}


static void write_xml_muscle_groups(FILE* fp, ModelStruct* ms)
{
    int i, j;

    fprintf(fp, "\t<groups>\n");
    for (i = 0; i < ms->numgroups; i++)
    {
        fprintf(fp, "\t\t<ObjectGroup name=\"%s\">\n", ms->muscgroup[i].name);
        fprintf(fp, "\t\t\t<members>");
        for (j = 0; j < ms->muscgroup[i].num_muscles; j++)
            fprintf(fp, "%s ", ms->muscle[ms->muscgroup[i].muscle_index[j]]->name);
        fprintf(fp, "</members>\n");
        fprintf(fp, "\t\t</ObjectGroup>\n");
    }
    fprintf(fp, "\t</groups>\n");
}


#define NEED_TO_WRITE_MUSCLE_VALUE(field) \
    m->field && (writingDefault || m->field != ms->default_muscle->field)

static void write_xml_muscle(FILE* fp, ModelStruct* ms, dpMuscleStruct* m, const char* muscleClassName, int writingDefault, int angleUnits)
{
    int j;
   double ind_conv = 1.0;

    fprintf(fp, "\t\t<%s name=\"%s\">\n", muscleClassName, m->name);

    /* Attachment points. */
    if (m->path && m->path->num_orig_points > 0) {
        fprintf(fp, "\t\t\t<GeometryPath>\n");
        fprintf(fp, "\t\t\t<PathPointSet>\n");
        fprintf(fp, "\t\t\t<objects>\n");
        for (j = 0; j < m->path->num_orig_points; j++)
        {
            dpMusclePoint* mp = &m->path->mp_orig[j];
            if (mp->isMovingPoint)
            {
                fprintf(fp, "\t\t\t\t<MovingPathPoint>\n");
                fprintf(fp, "\t\t\t\t\t<body> %s </body>\n", ms->segment[getMusclePointSegment(m,j)].name);
            if (mp->function[0])
            {
               GeneralizedCoord* gencoord = (GeneralizedCoord*)mp->gencoord[0];
               if (angleUnits == RADIANS)
               {
                  // check if gencoord (independent coordinate) is rotational
                  if (gencoord->type == rotation_gencoord)
                     ind_conv = DEG_TO_RAD;
               }
               fprintf(fp, "\t\t\t\t\t<x_coordinate> %s </x_coordinate>\n", gencoord->name);
               fprintf(fp, "\t\t\t\t\t<x_location>\n");
               write_opensim_function(fp, mp->function[0], ind_conv, 1.0, 6);
               fprintf(fp, "\t\t\t\t\t</x_location>\n");
            }
            else
            {
               fprintf(fp, "\t\t\t\t\t<x_location>\n");
               write_opensim_constant_function(fp, mp->point[0], 6);
               fprintf(fp, "\t\t\t\t\t</x_location>\n");
            }
            if (mp->function[1])
            {
               GeneralizedCoord* gencoord = (GeneralizedCoord*)mp->gencoord[1];
               if (angleUnits == RADIANS)
               {
                  // check if gencoord (independent coordinate) is rotational
                  if (gencoord->type == rotation_gencoord)
                     ind_conv = DEG_TO_RAD;
               }
               fprintf(fp, "\t\t\t\t\t<y_coordinate> %s </y_coordinate>\n", gencoord->name);
               fprintf(fp, "\t\t\t\t\t<y_location>\n");
               write_opensim_function(fp, mp->function[1], ind_conv, 1.0, 6);
               fprintf(fp, "\t\t\t\t\t</y_location>\n");
            }
            else
            {
               fprintf(fp, "\t\t\t\t\t<y_location>\n");
               write_opensim_constant_function(fp, mp->point[1], 6);
               fprintf(fp, "\t\t\t\t\t</y_location>\n");
            }
            if (mp->function[2])
            {
               GeneralizedCoord* gencoord = (GeneralizedCoord*)mp->gencoord[2];
               if (angleUnits == RADIANS)
               {
                  // check if gencoord (independent coordinate) is rotational
                  if (gencoord->type == rotation_gencoord)
                     ind_conv = DEG_TO_RAD;
               }
               fprintf(fp, "\t\t\t\t\t<z_coordinate> %s </z_coordinate>\n", gencoord->name);
               fprintf(fp, "\t\t\t\t\t<z_location>\n");
               write_opensim_function(fp, mp->function[2], ind_conv, 1.0, 6);
               fprintf(fp, "\t\t\t\t\t</z_location>\n");
            }
            else
            {
               fprintf(fp, "\t\t\t\t\t<z_location>\n");
               write_opensim_constant_function(fp, mp->point[2], 6);
               fprintf(fp, "\t\t\t\t\t</z_location>\n");
            }
            // Check if OpenSim can handle one of the attachment coordinates being a constant.
            // If it can't then you have to make up a gencoord name and use it in the natural
            // cubic spline (2 points, constant value).
                fprintf(fp, "\t\t\t\t</MovingPathPoint>\n");
            }
            else if (mp->isVia)
            {
                // If the gencoord used for this via point range is [primarily] translational, then do
                // not convert the range values. If it's rotational, then use DEG_TO_RAD (if angleUnits
            // is RADIANS, which means the user wants radians in the output file).
                double conversion;
            GeneralizedCoord* gencoord = (GeneralizedCoord*)mp->viaRange.gencoord;
                if (gencoord->type == rotation_gencoord && angleUnits == RADIANS)
                    conversion = DEG_TO_RAD;
                else
                    conversion = 1.0;

                fprintf(fp, "\t\t\t\t<ConditionalPathPoint>\n");
                fprintf(fp, "\t\t\t\t\t<location> %.12lf %.12lf %.12lf </location>\n", mp->point[0],
               mp->point[1], mp->point[2]);
                fprintf(fp, "\t\t\t\t\t<body> %s </body>\n", ms->segment[getMusclePointSegment(m,j)].name);
                fprintf(fp, "\t\t\t\t\t<coordinate> %s </coordinate>\n", gencoord->name);
                fprintf(fp, "\t\t\t\t\t<range> %.12lf %.12lf </range>\n", mp->viaRange.start * conversion,
               mp->viaRange.end * conversion);
                fprintf(fp, "\t\t\t\t</ConditionalPathPoint>\n");
            }
            else // regular muscle point
            {
                fprintf(fp, "\t\t\t\t<PathPoint>\n");
                fprintf(fp, "\t\t\t\t\t<location> %.12lf %.12lf %.12lf </location>\n", mp->point[0],
               mp->point[1], mp->point[2]);
                fprintf(fp, "\t\t\t\t\t<body> %s </body>\n", ms->segment[getMusclePointSegment(m,j)].name);
                fprintf(fp, "\t\t\t\t</PathPoint>\n");
            }
        }
        fprintf(fp, "\t\t\t</objects>\n");
        fprintf(fp, "\t\t\t</PathPointSet>\n");

        if (m->numWrapStructs > 0)
        {
            fprintf(fp, "\t\t\t<PathWrapSet>\n");
            fprintf(fp, "\t\t\t<objects>\n");
            for (j = 0; j < m->numWrapStructs; j++)
            {
                dpWrapObject* wo = m->wrapStruct[j]->wrap_object;
                fprintf(fp, "\t\t\t\t<PathWrap>\n");
                fprintf(fp, "\t\t\t\t\t<wrap_object> %s </wrap_object>\n", wo->name);
                if (wo->wrap_type == dpWrapEllipsoid)
                    fprintf(fp, "\t\t\t\t\t<method> %s </method>\n", get_wrap_algorithm_name(m->wrapStruct[j]->wrap_algorithm));
                if (m->wrapStruct[j]->startPoint > 0 || m->wrapStruct[j]->endPoint > 0)
                    fprintf(fp,"\t\t\t\t\t<range> %d %d </range>\n", m->wrapStruct[j]->startPoint, m->wrapStruct[j]->endPoint);
                fprintf(fp, "\t\t\t\t</PathWrap>\n");
            }
            fprintf(fp, "\t\t\t</objects>\n");
            fprintf(fp, "\t\t\t</PathWrapSet>\n");
        }

        fprintf(fp, "\t\t\t</GeometryPath>\n");
    }

    /* Simple (double) parameters. */
    if (NEED_TO_WRITE_MUSCLE_VALUE(max_isometric_force))
        fprintf(fp, "\t\t\t<max_isometric_force> %.12lf </max_isometric_force>\n", *m->max_isometric_force);
    if (NEED_TO_WRITE_MUSCLE_VALUE(optimal_fiber_length))
        fprintf(fp, "\t\t\t<optimal_fiber_length> %.12lf </optimal_fiber_length>\n", *m->optimal_fiber_length);
    if (NEED_TO_WRITE_MUSCLE_VALUE(resting_tendon_length))
        fprintf(fp, "\t\t\t<tendon_slack_length> %.12lf </tendon_slack_length>\n", *m->resting_tendon_length);
    if (NEED_TO_WRITE_MUSCLE_VALUE(pennation_angle))
   {
      if (angleUnits == RADIANS)
           fprintf(fp, "\t\t\t<pennation_angle> %.12lf </pennation_angle>\n", *m->pennation_angle * DEG_TO_RAD);
      else
           fprintf(fp, "\t\t\t<pennation_angle> %.12lf </pennation_angle>\n", *m->pennation_angle);
   }
    if (NEED_TO_WRITE_MUSCLE_VALUE(max_contraction_vel))
        fprintf(fp, "\t\t\t<max_contraction_velocity> %.12lf </max_contraction_velocity>\n", *m->max_contraction_vel);

    /* muscle model. */
    //if (m->muscle_model_index)    // Conservative fix in case muscle model is not specified. Ayman 1/07
    //  fprintf(fp, "\t\t\t<muscle_model> %d </muscle_model>\n", *m->muscle_model_index);

    /* Dynamic parameters. */
    for (j = 0; j < m->num_dynamic_params; j++)
    {
        // map dynamic parameter "timescale" to "time_scale" property name
        if (NEED_TO_WRITE_MUSCLE_VALUE(dynamic_params[j]))
        {
            if(!strcmp(m->dynamic_param_names[j],"timescale"))
                fprintf(fp, "\t\t\t<time_scale> %.12lf </time_scale>\n", *m->dynamic_params[j]);
            else
                fprintf(fp, "\t\t\t<%s> %.12lf </%s>\n", m->dynamic_param_names[j], *m->dynamic_params[j], m->dynamic_param_names[j]);
        }
    }

    /* Tendon force-length curve -- only write if non-default value. */
    if (NEED_TO_WRITE_MUSCLE_VALUE(tendon_force_len_func))
    {
        fprintf(fp, "\t\t\t<tendon_force_length_curve>\n");
        write_opensim_function(fp, *m->tendon_force_len_func, 1.0, 1.0, 5);
        fprintf(fp, "\t\t\t</tendon_force_length_curve>\n");
    }

    /* Active force-length curve -- only write if non-default value. */
    if (NEED_TO_WRITE_MUSCLE_VALUE(active_force_len_func))
    {
        fprintf(fp, "\t\t\t<active_force_length_curve>\n");
        write_opensim_function(fp, *m->active_force_len_func, 1.0, 1.0, 5);
        fprintf(fp, "\t\t\t</active_force_length_curve>\n");
    }

    /* Passive force-length curve -- only write if non-default. */
    if (NEED_TO_WRITE_MUSCLE_VALUE(passive_force_len_func))
    {
        fprintf(fp, "\t\t\t<passive_force_length_curve>\n");
        write_opensim_function(fp, *m->passive_force_len_func, 1.0, 1.0, 5);
        fprintf(fp, "\t\t\t</passive_force_length_curve>\n");
    }

    /* Force-velocity curve -- only write if non-default. */
    if (NEED_TO_WRITE_MUSCLE_VALUE(force_vel_func))
    {
        fprintf(fp, "\t\t\t<force_velocity_curve>\n");
        write_opensim_function(fp, *m->force_vel_func, 1.0, 1.0, 5);
        fprintf(fp, "\t\t\t</force_velocity_curve>\n");
    }

    fprintf(fp, "\t\t</%s>\n", muscleClassName);

}

static void write_xml_muscles(FILE* fp, ModelStruct* ms, int angleUnits)
{
    int i;
    char muscleClassName[64];

    fprintf(fp, "\t<ForceSet>\n");
    write_xml_muscle_groups(fp, ms);
    fprintf(fp, "\t<objects>\n");
    for (i = 0; i < ms->nummuscles; i++)
    {
        dpMuscleStruct* m = ms->muscle[i];

        // For now, model = 9 and map to Thelen2003Muscle.
        // All other values map to Schutte1993Muscle. Eventually, each model
        // index should map to a distinct muscle class in OpenSim,
        // especially model = 7, which is a ligament model.
        if (m->muscle_model_index == NULL || *m->muscle_model_index == 4)
            strcpy(muscleClassName, "Schutte1993Muscle");
        else if (*m->muscle_model_index == 9)
            strcpy(muscleClassName, "Thelen2003Muscle");
        else if (*m->muscle_model_index == 1 || *m->muscle_model_index == 2)
            strcpy(muscleClassName, "Delp1990Muscle");
        else{   // Warn about unsupported but use Schutte1993Muscle
            strcpy(muscleClassName, "Schutte1993Muscle");
            printf("Warning: muscle %s has unsupported muscle model %d.\n", m->name, *m->muscle_model_index);
        }
        write_xml_muscle(fp, ms, m, muscleClassName, 0, angleUnits);
    }
    fprintf(fp, "\t</objects>\n");
    fprintf(fp, "\t</ForceSet>\n");
}


static void write_xml_markers(FILE* fp, ModelStruct* ms, const char* markerSetOut)
{
    int i, j;
    char tabs[5];

    if (markerSetOut) {
        fp = fopen(markerSetOut, "w");
        fprintf(fp, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        strcpy(tabs, "");
    } else {
        strcpy(tabs, "\t\t\t");
    }

    fprintf(fp, "%s<MarkerSet>\n", tabs);
    fprintf(fp, "%s\t<objects>\n", tabs);

    for (i = 0; i < ms->numsegments; i++)
    {
        SegmentStruct* ss = &ms->segment[i];
        for (j = 0; j < ms->segment[i].numMarkers; j++)
        {
            fprintf(fp, "%s\t\t<Marker name=\"%s\">\n", tabs, ss->marker[j]->name);
            fprintf(fp, "%s\t\t\t<body> %s </body>\n", tabs, ss->name);
            fprintf(fp, "%s\t\t\t<location> %.12lf %.12lf %.12lf </location>\n", tabs, ss->marker[j]->offset[0], ss->marker[j]->offset[1], ss->marker[j]->offset[2]);
            //fprintf(fp, "%s\t\t\t<weight> %.12lf </weight>\n", tabs, ss->marker[j]->weight);
            fprintf(fp, "%s\t\t\t<fixed> %s </fixed>\n", tabs, (ss->marker[j]->fixed == yes) ? ("true") : ("false"));
            fprintf(fp, "%s\t\t</Marker>\n", tabs);
        }
    }

    fprintf(fp, "%s\t</objects>\n", tabs);
    fprintf(fp, "%s</MarkerSet>\n", tabs);

    if(markerSetOut) {
        fclose(fp);
        printf("Wrote MarkerSet file %s\n", markerSetOut);
    }
}


static void write_vtk_bone(PolyhedronStruct* bone, char geometryDirectory[], char filename[])
{
    int i, j, count;
    char path[4096];
    FILE* fp;

    build_full_path(geometryDirectory, filename, path);
    fp = fopen(path, "w");
    if (fp == NULL)
        return;

    fprintf(fp, "<?xml version=\"1.0\"?>\n");
    fprintf(fp, "<VTKFile type=\"PolyData\" version=\"0.1\" byte_order=\"LittleEndian\" compressor=\"vtkZLibDataCompressor\">\n");
    fprintf(fp, "\t<PolyData>\n");
    fprintf(fp, "\t\t<Piece NumberOfPoints=\"%d\" NumberOfVerts=\"0\" NumberOfLines=\"0\" NumberOfStrips=\"0\" NumberOfPolys=\"%d\">\n",
        bone->num_vertices, bone->num_polygons);
    fprintf(fp, "\t\t<PointData Normals=\"Normals\">\n");
    fprintf(fp, "\t\t<DataArray type=\"Float32\" Name=\"Normals\" NumberOfComponents=\"3\" format=\"ascii\">\n");
    for (i = 0; i < bone->num_vertices; i++)
        fprintf(fp, "\t\t\t%9.6lf %9.6lf %9.6lf\n", bone->vertex[i].normal[0], bone->vertex[i].normal[1], bone->vertex[i].normal[2]);
    fprintf(fp, "\t\t</DataArray>\n");
    fprintf(fp, "\t</PointData>\n");
    fprintf(fp, "\t<Points>\n");
    fprintf(fp, "\t\t<DataArray type=\"Float32\" NumberOfComponents=\"3\" format=\"ascii\">\n");
    for (i = 0; i < bone->num_vertices; i++)
        fprintf(fp, "\t\t\t%9.6lf %9.6lf %9.6lf\n", bone->vertex[i].coord[0], bone->vertex[i].coord[1], bone->vertex[i].coord[2]);
    fprintf(fp, "\t\t</DataArray>\n");
    fprintf(fp, "\t</Points>\n");
    fprintf(fp, "\t\t\t<Polys>\n");
    fprintf(fp, "\t\t\t\t<DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n");
    for (i = 0; i < bone->num_polygons; i++)
    {
        fprintf(fp, "\t\t\t\t\t");
        for (j = 0; j < bone->polygon[i].num_vertices; j++)
            fprintf(fp, "%d ", bone->polygon[i].vertex_index[j]);
        fprintf(fp, "\n");
    }
    fprintf(fp, "\t\t\t\t</DataArray>\n");
    fprintf(fp, "\t\t\t\t<DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n");
    fprintf(fp, "\t\t\t\t\t");
    for (i = 0, count = 0; i < bone->num_polygons; i++)
    {
        count += bone->polygon[i].num_vertices;
        fprintf(fp, "%d ", count);
    }
    fprintf(fp, "\n");
    fprintf(fp, "\t\t\t\t</DataArray>\n");
    fprintf(fp, "\t\t\t</Polys>\n");
    fprintf(fp, "\t\t</Piece>\n");
    fprintf(fp, "\t</PolyData>\n");
    fprintf(fp, "</VTKFile>\n");

    fclose(fp);
}

static int segment_has_wrap_objects(ModelStruct* ms, SegmentStruct* seg)
{
    int i;

    for (i = 0; i < ms->num_wrap_objects; i++)
        if (&ms->segment[ms->wrapobj[i]->segment] == seg)
            return 1;

    return 0;
}

static void write_xml_wrap_object(FILE* fp, dpWrapObject* wo, int angleUnits)
{
    double xyz[3];
    const char *wrap_quadrant;
    double conversion;

    // The angles output by extract_xyz_rot_bodyfixed are in radians,
    // so this conversion factor is different than in the other functions.
    if (angleUnits == DEGREES)
        conversion = RAD_TO_DEG;
    else
        conversion = 1.0;

    switch (wo->wrap_type)
    {
       case dpWrapSphere:
           fprintf(fp, "\t\t\t\t\t\t<WrapSphere name=\"%s\">\n", wo->name);
            fprintf(fp, "\t\t\t\t\t\t\t<radius> %.8lf </radius>\n", wo->radius[0]);
            break;
        case dpWrapCylinder:
            fprintf(fp, "\t\t\t\t\t\t<WrapCylinder name=\"%s\">\n", wo->name);
            fprintf(fp, "\t\t\t\t\t\t\t<radius> %.8lf </radius>\n", wo->radius[0]);
            fprintf(fp, "\t\t\t\t\t\t\t<length> %.8lf </length>\n", wo->height);
            break;
        case dpWrapEllipsoid:
            fprintf(fp, "\t\t\t\t\t\t<WrapEllipsoid name=\"%s\">\n", wo->name);
            fprintf(fp, "\t\t\t\t\t\t\t<dimensions> %.8lf %.8lf %.8lf </dimensions>\n",
                wo->radius[0], wo->radius[1], wo->radius[2]);
            break;
        case dpWrapTorus:
            fprintf(fp, "\t\t\t\t\t\t<WrapTorus name=\"%s\">\n", wo->name);
            fprintf(fp, "\t\t\t\t\t\t\t<inner_radius> %.8lf </inner_radius>\n", wo->radius[0]);
            fprintf(fp, "\t\t\t\t\t\t\t<outer_radius> %.8lf </outer_radius>\n", wo->radius[1]);
            break;
    }

    fprintf(fp,"\t\t\t\t\t\t\t<active> %s </active>\n", wo->active ? "true" : "false");

    recalc_xforms(wo);
    extract_xyz_rot_bodyfixed(wo->from_local_xform, xyz);

    fprintf(fp, "\t\t\t\t\t\t\t<xyz_body_rotation> %.8lf %.8lf %.8lf </xyz_body_rotation>\n",
        xyz[0] * conversion, xyz[1] * conversion, xyz[2] * conversion);

    fprintf(fp,"\t\t\t\t\t\t\t<translation> %.8lf %.8lf %.8lf </translation>\n",
        wo->translation.xyz[0], wo->translation.xyz[1], wo->translation.xyz[2]);

    if (wo->wrap_sign != 0)
    {
        switch ((wo->wrap_axis + 1) * wo->wrap_sign)
        {
           default:
                break;
            case  1: wrap_quadrant =  "x";
                break;
            case -1: wrap_quadrant = "-x";
                break;
            case  2: wrap_quadrant =  "y";
                break;
            case -2: wrap_quadrant = "-y";
                break;
            case  3: wrap_quadrant =  "z";
                break;
            case -3: wrap_quadrant = "-z";
                break;
        }
        fprintf(fp,"\t\t\t\t\t\t\t<quadrant> %s </quadrant>\n", wrap_quadrant);
    }

    switch (wo->wrap_type)
    {
       case dpWrapSphere:
           fprintf(fp, "\t\t\t\t\t\t</WrapSphere>\n");
            break;
        case dpWrapCylinder:
            fprintf(fp, "\t\t\t\t\t\t</WrapCylinder>\n");
            break;
        case dpWrapEllipsoid:
            fprintf(fp, "\t\t\t\t\t\t</WrapEllipsoid>\n");
            break;
        case dpWrapTorus:
            fprintf(fp, "\t\t\t\t\t\t</WrapTorus>\n");
            break;
    }
}

static void write_xml_units(FILE* fp, ModelStruct* ms)
{
   if (ms->forceUnits != NULL)
      fprintf(fp, "\t<force_units> %s </force_units>\n", ms->forceUnits);

   if (ms->lengthUnits != NULL)
      fprintf(fp, "\t<length_units> %s </length_units>\n", ms->lengthUnits);
}

static void write_xml_defaults(FILE* fp, ModelStruct* ms, int angleUnits)
{
    fprintf(fp, "\t<defaults>\n");
    if (ms && ms->default_muscle)
    {
        // We need to write a default muscle for all supported types of muscles (since
        // the default mechanism works by matching class names)
        write_xml_muscle(fp, ms, ms->default_muscle, "Schutte1993Muscle", 1, angleUnits);
        write_xml_muscle(fp, ms, ms->default_muscle, "Thelen2003Muscle", 1, angleUnits);
        write_xml_muscle(fp, ms, ms->default_muscle, "Delp1990Muscle", 1, angleUnits);
    }
    fprintf(fp, "\t</defaults>\n");
}

// This function deals with the constant translations and rotations that are in a joint.
// Simbody does not allow them in DOFs, so they are handled as follows:
//   1. The string of constants at the beginning of the DOF list are converted into
//      locationInParent and orientationInParent.
//   2. The string of constants at the end of the DOF list are converted into
//      locationInChild and orientationInChild.
//   3. The constants in between functions are turned into functions that are
//      constrained to remain at the proper value.
static void extract_joint_locations_and_orientations(ModelStruct* ms,
               JointStruct* joint, int dof_order[], double locationInParent[],
               double orientationInParent[], double locationInChild[], double orientationInChild[])
{
   int i, first_function=6, last_function=6, order[4];
   double parentTransform[4][4], childTransform[4][4], childInverse[4][4];
   double dof_value[6], x[4], y[4], z[4], ra1[4], ra2[4], ra3[4];

   for (i=0; i<6; i++)
   {
      if (joint->dofs[dof_order[i]].type == function_dof)
      {
         first_function = i;
         break;
      }
   }

   for (i=5; i>=0; i--)
   {
      if (joint->dofs[dof_order[i]].type == function_dof)
      {
         last_function = i;
         break;
      }
   }

   // Constants that are in between functions are converted to functions.
   // The gencoord used for these converted constants is the
   // 'first_function' gencoord.
   for (i=first_function+1; i<last_function; i++)
   {
      DofStruct* dof = &joint->dofs[dof_order[i]];
      if (dof->type == constant_dof && NOT_EQUAL_WITHIN_ERROR(dof->value, 0.0))
         convert_dof_to_function(ms, dof, joint->dofs[dof_order[first_function]].gencoord);
   }

   // Clear the dof_value array so you can fill in only
   // the values you care about.
   for (i=0; i<6; i++)
      dof_value[i] = 0.0;

   // Constants at the beginning go into the parent matrix.
   for (i=0; i<first_function; i++)
      dof_value[dof_order[i]] = joint->dofs[dof_order[i]].value;

   //TODO2.0: reverse axes for INVERSE, or maybe just negate dof value?

   /* initialize the [parent] x, y, and z axes */
   COPY_1X4VECTOR(gWorld_x,joint->parentframe[XX]);
   COPY_1X4VECTOR(gWorld_y,joint->parentframe[YY]);
   COPY_1X4VECTOR(gWorld_z,joint->parentframe[ZZ]);
   order[TRANS] = joint->order[TRANS]+1;
   order[ROT1] = joint->order[ROT1]+1;
   order[ROT2] = joint->order[ROT2]+1;
   order[ROT3] = joint->order[ROT3]+1;
   COPY_1X4VECTOR(joint->parentframe[XX],x);
   COPY_1X4VECTOR(joint->parentframe[YY],y);
   COPY_1X4VECTOR(joint->parentframe[ZZ],z);
   COPY_1X4VECTOR(joint->parentrotaxes[R1],ra1);
   normalize_vector(ra1, ra1);
   COPY_1X4VECTOR(joint->parentrotaxes[R2],ra2);
   normalize_vector(ra2, ra2);
   COPY_1X4VECTOR(joint->parentrotaxes[R3],ra3);
   normalize_vector(ra3, ra3);

   calc_joint_transform(order, dof_value, parentTransform,
                        x, y, z, ra1, ra2, ra3, BF, FORWARD, joint);

   // Clear the dof_value array so you can fill in only
   // the values you care about.
   for (i=0; i<6; i++)
      dof_value[i] = 0.0;

   // Constants at the end go into the child matrix.
   for (i=last_function+1; i<6; i++)
      dof_value[dof_order[i]] = joint->dofs[dof_order[i]].value;

   // Reset the axes
   COPY_1X4VECTOR(joint->parentframe[XX],x);
   COPY_1X4VECTOR(joint->parentframe[YY],y);
   COPY_1X4VECTOR(joint->parentframe[ZZ],z);
   COPY_1X4VECTOR(joint->parentrotaxes[R1],ra1);
   normalize_vector(ra1, ra1);
   COPY_1X4VECTOR(joint->parentrotaxes[R2],ra2);
   normalize_vector(ra2, ra2);
   COPY_1X4VECTOR(joint->parentrotaxes[R3],ra3);
   normalize_vector(ra3, ra3);

   // Calculate the forward transform for the DOFs after the
   // last function, and then invert it.
   calc_joint_transform(order, dof_value, childTransform,
                        x, y, z, ra1, ra2, ra3, BF, FORWARD, joint);
   invert_4x4matrix(childTransform, childInverse);

   // Extract the translations from the matrices.
   for (i=0; i<3; i++)
   {
      locationInParent[i] = parentTransform[3][i];
      locationInChild[i] = childTransform[3][i];
   }

   // Extract the rotations from the matrices.
   extract_xyz_rot_bodyfixed(parentTransform, orientationInParent);
   extract_xyz_rot_bodyfixed(childInverse, orientationInChild);
}

static ReturnCode write_simbody_engine(FILE* fp, ModelStruct* ms, char geometryDirectory[], const char* markerSetOut, int angleUnits)
{
   // Gravity
    write_xml_gravity(fp, ms);

   // Bodies (with joints, wrap objects), constraints
   if (make_simbody_model(fp, ms, geometryDirectory, angleUnits) == code_bad)
      return code_bad;

   // Markers
    write_xml_markers(fp, ms, markerSetOut);

   return code_fine;
}

static void write_opensim_function(FILE* fp, dpFunction* function, double ind_conv, double dep_conv, int tab_level)
{
   int i;
   char tabs[CHARBUFFER];

   for (i=0; i<tab_level; i++)
      tabs[i] = '\t';
   tabs[tab_level] = STRING_TERMINATOR;

   if (function_is_simple_linear(function))
   {
      double m = (function->y[1] - function->y[0]) / (function->x[1] - function->x[0]);
      double b = function->y[0] - function->x[0] * m;
      fprintf(fp, "%s<LinearFunction>\n", tabs);
      fprintf(fp, "%s\t<coefficients> %lf %lf </coefficients>\n", tabs, m, b);
      fprintf(fp, "%s</LinearFunction>\n", tabs);
      return;
   }

   if (function->type == dpNaturalCubicSpline || function->type == dpFunctionTypeUndefined)
      fprintf(fp, "%s<NaturalCubicSpline>\n", tabs);
   else if (function->type == dpStepFunction)
      fprintf(fp, "%s<StepFunction>\n", tabs);
   else if (function->type == dpLinearFunction)
      fprintf(fp, "%s<PiecewiseLinearFunction>\n", tabs);
   else if (function->type == dpGCVSpline)
    {
      fprintf(fp, "%s<GCVSpline>\n", tabs);
      fprintf(fp, "%s\t<half_order> 3 </half_order>\n", tabs);
    }

   fprintf(fp, "%s\t<x> ", tabs);
   for (i=0; i<function->numpoints; i++)
      fprintf(fp, "%.12lf ", function->x[i] * ind_conv);
   fprintf(fp, "</x>\n");

    fprintf(fp, "%s\t<y> ", tabs);
    for (i=0; i<function->numpoints; i++)
        fprintf(fp, "%.12lf ", function->y[i] * dep_conv);
    fprintf(fp, "</y>\n");

   if (function->type == dpNaturalCubicSpline || function->type == dpFunctionTypeUndefined)
      fprintf(fp, "%s</NaturalCubicSpline>\n", tabs);
   else if (function->type == dpStepFunction)
      fprintf(fp, "%s</StepFunction>\n", tabs);
   else if (function->type == dpLinearFunction)
      fprintf(fp, "%s</PiecewiseLinearFunction>\n", tabs);
   else if (function->type == dpGCVSpline)
      fprintf(fp, "%s</GCVSpline>\n", tabs);
}

static void write_opensim_constant_function(FILE* fp, double value, int tab_level)
{
   int i;
   char tabs[CHARBUFFER];

   for (i=0; i<tab_level; i++)
      tabs[i] = '\t';
   tabs[tab_level] = STRING_TERMINATOR;

   fprintf(fp, "%s<Constant>\n", tabs);
   fprintf(fp, "%s\t<value> %lf </value>\n", tabs, value);
   fprintf(fp, "%s</Constant>\n", tabs);
}

static SBoolean function_is_simple_linear(dpFunction* function)
{
   if (function->numpoints == 2 && EQUAL_WITHIN_ERROR(function->x[0], function->y[0]))
   {
      double slope = (function->y[1] - function->y[0]) / (function->x[1] - function->x[0]);
      if (EQUAL_WITHIN_ERROR(slope, 1.0))
         return yes;
   }

   return no;
}

static ReturnCode make_simbody_model(FILE* fp, ModelStruct* ms, char geometryDirectory[], int angleUnits)
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
      make_conversion(ms, i);

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
      if (ms->gencoord[i]->used_in_model == yes)
      {
            if (!mark_unconstrained_dof(ms, ms->gencoord[i], &gcInfo[i].jointnum, &gcInfo[i].dofnum, &gcInfo[i].constrained))
         {
#if 0
            sprintf(errorbuffer, "At least one DOF must be a \"simple\" function of gencoord %s (2 points, slope=1, passes thru zero).",
               ms->gencoord[i]->name);
            error(none,errorbuffer);
            return code_bad;
#endif
         }
      }
   }

   // Now give the dofs names for use in the OpenSim model file.
   // Names of unconstrained dofs will simply be the names of
   // the gencoords to which they correspond. Names of
   // constrained dofs will be formed from the joint name
   // and the dof keyword (e.g. "hip_tx" and "knee_r1").
   name_dofs(ms);

   find_sdfast_joint_order(ms, jnts, segs, joint_order, SIMBODY_GROUND);

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

   //convert_fixed_joints(ms, gcInfo);

   // Bodies (with joints and wrap objects)
   fprintf(fp, "\t\t\t<BodySet>\n");
   fprintf(fp, "\t\t\t<objects>\n");
   write_opensim_ground_body(fp, ms, &ms->segment[ms->ground_segment], geometryDirectory, angleUnits);
   for (i=0, dofcount=0, constrainedcount=0; i<ms->numjoints; i++)
   {
      make_simbody_joint(ms, fp, &jnts[joint_order[i]], joint_order[i], ms->joint[i].sd_num, segs, gcInfo,
         &dofcount, &constrainedcount, geometryDirectory, angleUnits);
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
         if (joint->dofs[j].sd.constrained == yes && joint->dofs[j].gencoord &&
            gcInfo[getGencoordIndex(ms, joint->dofs[j].gencoord)].jointnum != joint_order[i])
            write_opensim_constraint(fp, ms, &joint->dofs[j], j, angleUnits);
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
                               char geometryDirectory[], int angleUnits)
{
   int i, j, dof_order[6], current_dof = 0;
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
        int rev_dof_order[6];

        for (i=0; i<6; i++)
            rev_dof_order[i] = dof_order[5-i];
        extract_joint_locations_and_orientations(ms, joint, rev_dof_order, locationInParent, orientationInParent,
            locationInChild, orientationInChild);
    }
    else
    {
        extract_joint_locations_and_orientations(ms, joint, dof_order, locationInParent, orientationInParent,
            locationInChild, orientationInChild);
    }

   fprintf(fp, "\t\t\t\t\t<Joint>\n");

    if (joint_is_fixed(joint))
    {
        fprintf(fp, "\t\t\t\t\t\t<WeldJoint name=\"%s\">\n", joint->name);
        fprintf(fp, "\t\t\t\t\t\t\t<parent_body> %s </parent_body>\n", jntsdf->inbname);
        fprintf(fp, "\t\t\t\t\t\t\t<location_in_parent> %.12lf %.12lf %.12lf </location_in_parent>\n",
            locationInParent[0], locationInParent[1], locationInParent[2]);
        fprintf(fp, "\t\t\t\t\t\t\t<orientation_in_parent> %.12lf %.12lf %.12lf </orientation_in_parent>\n",
            orientationInParent[0], orientationInParent[1], orientationInParent[2]);
        fprintf(fp, "\t\t\t\t\t\t\t<location> %.12lf %.12lf %.12lf </location>\n",
            locationInChild[0], locationInChild[1], locationInChild[2]);
        fprintf(fp, "\t\t\t\t\t\t\t<orientation> %.12lf %.12lf %.12lf </orientation>\n",
            orientationInChild[0], orientationInChild[1], orientationInChild[2]);
        fprintf(fp, "\t\t\t\t\t\t</WeldJoint>\n");
    } else {
        fprintf(fp, "\t\t\t\t<CustomJoint name=\"%s\">\n", joint->name);
        fprintf(fp, "\t\t\t\t\t<reverse> %s </reverse>\n", jntsdf->dir == FORWARD ? "false" : "true");
        fprintf(fp, "\t\t\t\t\t<parent_body> %s </parent_body>\n", jntsdf->inbname);
        fprintf(fp, "\t\t\t\t\t<location_in_parent> %.12lf %.12lf %.12lf </location_in_parent>\n",
            locationInParent[0], locationInParent[1], locationInParent[2]);
        fprintf(fp, "\t\t\t\t\t<orientation_in_parent> %.12lf %.12lf %.12lf </orientation_in_parent>\n",
            orientationInParent[0], orientationInParent[1], orientationInParent[2]);
        fprintf(fp, "\t\t\t\t\t<location> %.12lf %.12lf %.12lf </location>\n",
            locationInChild[0], locationInChild[1], locationInChild[2]);
        fprintf(fp, "\t\t\t\t\t<orientation> %.12lf %.12lf %.12lf </orientation>\n",
            orientationInChild[0], orientationInChild[1], orientationInChild[2]);

        fprintf(fp, "\t\t\t\t\t<!--Generalized coordinates parameterizing this joint.-->\n");

        fprintf(fp, "\t\t\t\t\t<CoordinateSet>\n");
        fprintf(fp, "\t\t\t\t\t<objects>\n");
      // The coordinates must be listed in the same order that they are referenced in the transform axis list.
      // First do the rotations.
        for (i=0; i<6; i++)
        {
         if (dof_order[i] < 3)
         {
            DofStruct* dof = &joint->dofs[dof_order[i]];
            // This DOF shows up in the coordinate list if it represents an unconstrained coordinate,
            // or if it's constrained to a coordinate that is used in another joint. Or if it is one
            // of the newly created gencoords.
            if (dof->type == function_dof)
            {
               int index = getGencoordIndex(ms, dof->gencoord);
               if ((gcInfo[index].jointnum == jointnum && gcInfo[index].dofnum == dof_order[i]) ||
                  gcInfo[index].jointnum != jointnum || ms->gencoord[index]->defined == no)
                  write_opensim_coordinate(fp, ms, joint, dof_order[i], angleUnits);
            }
         }
        }
      // Now do the translations.
        for (i=0; i<6; i++)
        {
         if (dof_order[i] >= 3)
         {
            DofStruct* dof = &joint->dofs[dof_order[i]];
            // This DOF shows up in the coordinate list if it represents an unconstrained coordinate,
            // or if it's constrained to a coordinate that is used in another joint. Or if it is one
            // of the newly created gencoords.
            if (dof->type == function_dof)
            {
               int index = getGencoordIndex(ms, dof->gencoord);
               if ((gcInfo[index].jointnum == jointnum && gcInfo[index].dofnum == dof_order[i]) ||
                  gcInfo[index].jointnum != jointnum || ms->gencoord[index]->defined == no)
                  write_opensim_coordinate(fp, ms, joint, dof_order[i], angleUnits);
            }
         }
        }
        fprintf(fp, "\t\t\t\t\t</objects>\n");
        fprintf(fp, "\t\t\t\t\t</CoordinateSet>\n");

      // In an OpenSim CustomJoint, the SpatialTransform always
      // has 6 transform axes (3 rotation, 3 translation). The
      // 3 rotations are always specified first, and the 3 translations
      // are all w.r.t. the parent reference frame.
        fprintf(fp, "\t\t\t\t\t\t\t<SpatialTransform>\n");
      // First output the rotations.
        for (i=0; i<6; i++)
        {
         if (dof_order[i] < 3)
         {
            DofStruct* dof = &joint->dofs[dof_order[i]];
            // DOFs that are functions always show up in the transform axis list. If the DOF represents
            // an unconstrained coordinate or if it's constrained to a coordinate in another joint,
            // then it shows up without a function. If the DOF is constrained to a coordinate in this
            // joint, it shows up with a function (i.e., it's a function-based mobilizer).
            if (dof->type == function_dof)
            {
               int index = getGencoordIndex(ms, dof->gencoord);
               if (/*(gcInfo[index].jointnum == jointnum && gcInfo[index].dofnum == dof_order[i] && gcInfo[index].constrained == no) || */
                  gcInfo[index].jointnum != jointnum)
                  write_opensim_transformAxis(fp, ms, joint, dof_order[i], FORWARD, no, current_dof++, angleUnits);
               else
                  write_opensim_transformAxis(fp, ms, joint, dof_order[i], FORWARD, yes, current_dof++, angleUnits);
            }
            else
            {
               // Constant rotational DOFs still need to be added to the SpatialTransform, but with values of
               // zero because they have already been added to either orientation_in_parent or orientation.
               double saved_value = dof->value;
               dof->value = 0.0;
               write_opensim_transformAxis(fp, ms, joint, dof_order[i], FORWARD, yes, current_dof++, angleUnits);
               dof->value = saved_value;
            }
         }
        }
      // Now output the translations.
        for (i=0; i<6; i++)
        {
         if (dof_order[i] >= 3)
         {
            DofStruct* dof = &joint->dofs[dof_order[i]];
            // DOFs that are functions always show up in the transform axis list. If the DOF represents
            // an unconstrained coordinate or if it's constrained to a coordinate in another joint,
            // then it shows up without a function. If the DOF is constrained to a coordinate in this
            // joint, it shows up with a function (i.e., it's a function-based mobilizer).
            if (dof->type == function_dof)
            {
               int index = getGencoordIndex(ms, dof->gencoord);
               if (/*(gcInfo[index].jointnum == jointnum && gcInfo[index].dofnum == dof_order[i] && gcInfo[index].constrained == no) || */
                  gcInfo[index].jointnum != jointnum)
                  write_opensim_transformAxis(fp, ms, joint, dof_order[i], FORWARD, no, current_dof++, angleUnits);
               else
                  write_opensim_transformAxis(fp, ms, joint, dof_order[i], FORWARD, yes, current_dof++, angleUnits);
            }
            else
            {
               // Constant translational DOFs still need to be added to the SpatialTransform, but with values of
               // zero because they have already been added to either location_in_parent or location.
               double saved_value = dof->value;
               dof->value = 0.0;
               write_opensim_transformAxis(fp, ms, joint, dof_order[i], FORWARD, yes, current_dof++, angleUnits);
               dof->value = saved_value;
            }
         }
        }
        fprintf(fp, "\t\t\t\t\t\t\t</SpatialTransform>\n");
        fprintf(fp, "\t\t\t\t\t\t</CustomJoint>\n");
    }
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
            change_filename_suffix(seg2->bone[j].name, vtpFileName, "vtp", CHARBUFFER);
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
            if (&ms->segment[ms->wrapobj[j]->segment] == seg2)
               write_xml_wrap_object(fp, ms->wrapobj[j], angleUnits);
         }
         fprintf(fp, "\t\t\t\t\t</objects>\n");
         fprintf(fp, "\t\t\t\t\t</WrapObjectSet>\n");
      }
   }

   fprintf(fp, "\t\t\t\t</Body>\n");
}

static void write_opensim_ground_body(FILE* fp, ModelStruct* ms, SegmentStruct* seg, char geometryDirectory[], int angleUnits)
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
         change_filename_suffix(seg->bone[j].name, vtpFileName, "vtp", CHARBUFFER);
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
         if (&ms->segment[ms->wrapobj[j]->segment] == seg)
            write_xml_wrap_object(fp, ms->wrapobj[j], angleUnits);
      }
      fprintf(fp, "\t\t\t\t\t</objects>\n");
      fprintf(fp, "\t\t\t\t\t</WrapObjectSet>\n");
   }
   fprintf(fp, "\t\t\t\t</Body>\n");
}

static void write_opensim_coordinate(FILE* fp, ModelStruct* ms, JointStruct* joint, int dofIndex, int angleUnits)
{
   DofStruct* dof = &joint->dofs[dofIndex];
   GeneralizedCoord* gc = dof->gencoord;
   double conversion = 1.0;

   fprintf(fp, "\t\t\t\t\t\t\t\t<Coordinate name=\"%s\">\n", dof->sd.name);
   if (dof->sd.constrained == no)
   {
      // If the gencoord is [primarily] translational, then do not convert its
       // range, default_value, etc. If it's rotational, then use DEG_TO_RAD.
      if (gc->type == rotation_gencoord && angleUnits == RADIANS)
         conversion = DEG_TO_RAD;

      fprintf(fp, "\t\t\t\t\t\t\t\t\t<default_value> %.12lf </default_value>\n", gc->default_value * conversion);
      fprintf(fp, "\t\t\t\t\t\t\t\t\t<initial_value> %.12lf </initial_value>\n", gc->default_value * conversion);
      //if (NOT_EQUAL_WITHIN_ERROR(gc->tolerance, 0.0))
      //   fprintf(fp, "\t\t\t\t\t\t\t\t\t<tolerance> %.12lf </tolerance>\n", gc->tolerance * conversion);
      //if (NOT_EQUAL_WITHIN_ERROR(gc->pd_stiffness, 0.0))
      //   fprintf(fp, "\t\t\t\t\t\t\t\t\t<stiffness> %.12lf </stiffness>\n", gc->pd_stiffness);
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
   } else { // dof->sd.constrained = yes
      if (angleUnits == RADIANS)
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

static void write_opensim_transformAxis(FILE* fp, ModelStruct* ms, JointStruct* joint, int dofIndex,
                                        Direction dir, SBoolean writeFunction, int current_dof, int angleUnits)
{
   DofStruct* dof = &joint->dofs[dofIndex];
   GeneralizedCoord* gc = dof->gencoord;

    if (current_dof >= 0 && current_dof <= 2)
        fprintf(fp, "\t\t\t\t\t\t\t\t<TransformAxis name=\"rotation%d\">\n", current_dof+1);
    else
        fprintf(fp, "\t\t\t\t\t\t\t\t<TransformAxis name=\"translation%d\">\n", current_dof-2);
   //fprintf(fp, "\t\t\t\t\t\t\t\t\t<is_rotation> %s </is_rotation>\n", (dofIndex > 2) ? ("false") : ("true"));
   if (dir == FORWARD)
   {
      if (dofIndex == TX)
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> 1.0 0.0 0.0 </axis>\n");
      else if (dofIndex == TY)
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> 0.0 1.0 0.0 </axis>\n");
      else if (dofIndex == TZ)
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> 0.0 0.0 1.0 </axis>\n");
      else
      {
         double axis[4];
         COPY_1X4VECTOR(joint->parentrotaxes[dofIndex], axis);
         normalize_vector(axis, axis);
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> %.12lf %.12lf %.12lf </axis>\n", axis[0], axis[1], axis[2]);
      }
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
      {
         double axis[4];
         COPY_1X4VECTOR(joint->parentrotaxes[dofIndex], axis);
         normalize_vector(axis, axis);
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<axis> %.12lf %.12lf %.12lf </axis>\n", -axis[0], -axis[1], -axis[2]);
      }
   }
   if (writeFunction == yes)
   {
      dpFunction* sf = dof->function;
      double ind_conv = 1.0, dep_conv = 1.0;

      if (angleUnits == RADIANS)
      {
         // check if dof (dependent coordinate) is r1, r2, or r3
         if (dofIndex < 3)
            dep_conv = DEG_TO_RAD;
      }
      if (sf)
      {
         // check if gencoord (independent coordinate) is rotational
         if (dof->gencoord->type == rotation_gencoord)
            ind_conv = DEG_TO_RAD;
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<function name=\"f%d\">\n", sf->usernum);
         write_opensim_function(fp, sf, ind_conv, dep_conv, 10);
         fprintf(fp, "\t\t\t\t\t\t\t\t\t</function>\n");
         // If you are writing the function, the 'coordinate' is the independent coordinte in
         // the function.
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<coordinates> %s </coordinates>\n", dof->gencoord->name);
      }
      else
      {
         fprintf(fp, "\t\t\t\t\t\t\t\t\t<function>\n");
         write_opensim_constant_function(fp, dof->value * dep_conv, 10);
         fprintf(fp, "\t\t\t\t\t\t\t\t\t</function>\n");
      }
   }
   else
   {
      // If you're not writing the function, then this transform axis is its own
      // coordinate (either unconstrained, or constrained to a coordinate in
      // another joint with a CoordinateCoupler.
      fprintf(fp, "\t\t\t\t\t\t\t\t\t<coordinates> %s </coordinates>\n", dof->sd.name);
   }
   fprintf(fp, "\t\t\t\t\t\t\t\t</TransformAxis>\n");
}

static void write_opensim_constraint(FILE* fp, ModelStruct* ms, DofStruct* dof, int dofIndex, int angleUnits)
{
   if (dof->gencoord >= 0 && dof->function != NULL)
   {
      dpFunction* func = dof->function;
      double ind_conv = 1.0, dep_conv = 1.0;

      if (angleUnits == RADIANS)
      {
         // check if gencoord (independent coordinate) is rotational
         if (dof->gencoord->type == rotation_gencoord)
            ind_conv = DEG_TO_RAD;
         // check if dof (dependent coordinate) is r1, r2, or r3
         if (dofIndex < 3)
            dep_conv = DEG_TO_RAD;
      }

      fprintf(fp, "\t\t\t\t<CoordinateCouplerConstraint name=\"%s\">\n", dof->sd.con_name);
      fprintf(fp, "\t\t\t\t\t<isDisabled> false </isDisabled>\n");
      fprintf(fp, "\t\t\t\t\t<dependent_coordinate_name> %s </dependent_coordinate_name>\n", dof->sd.name);
      fprintf(fp, "\t\t\t\t\t<independent_coordinate_names> %s </independent_coordinate_names>\n", dof->gencoord->name);
      fprintf(fp, "\t\t\t\t\t<coupled_coordinates_function name=\"f%d\">\n", func->usernum);
      write_opensim_function(fp, func, ind_conv, dep_conv, 6);
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
         int index = getGencoordIndex(ms, joint->dofs[dofnum].gencoord);
         if (joint->order[TRANS] == 3)
            dofnum = TX;
         else if (joint->order[ROT1] == 3)
            dofnum = R1;
         else if (joint->order[ROT2] == 3)
            dofnum = R2;
         else
            dofnum = R3;
         convert_dof_to_function(ms, &joint->dofs[dofnum], NULL);
         // Fill in gcInfo with values that will cause this constraint
         // to be implemented as a function-based mobilizer. That is,
         // specify that the gencoord used for this new function is
         // "based" in this joint, but in another DOF.
         gcInfo[index].jointnum = i;
         gcInfo[index].dofnum = 5 - dofnum;
      }
   }
}

static dpFunction* add_constant_function_to_model(ModelStruct* ms, double y_value)
{
   dpFunction f;

   malloc_function(&f, 2);

   f.type = dpLinearFunction;
   f.cutoff_frequency = -1.0;
   f.numpoints = 2;
   f.x[0] = -999999.9999999;
   f.x[1] = 999999.9999999;
   f.y[0] = f.y[1] = y_value;

   calc_function_coefficients(&f);

   return load_simm_function(ms, &f, no);
}

void convert_dof_to_function(ModelStruct* ms, DofStruct* dof, GeneralizedCoord* gencoord)
{
   dof->type = function_dof;
   dof->function = add_constant_function_to_model(ms, dof->value);
   if (gencoord == NULL)
   {
      // Create a new gencoord for use in this function. The range and
      // other parameters of this gencoord are not specified, but they
      // are not needed for writing out an OpenSim model because the
      // OpenSim coordinate is constrained anyway.
      dof->gencoord = enter_gencoord(ms, dof->sd.name, yes);
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
