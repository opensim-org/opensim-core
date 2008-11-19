/*******************************************************************************

   SIMMTOOPENSIM.C

   Author: Peter Loan, Frank C. Anderson

   Date: November 10, 2006

   Description:
	Reads in SIMM joint and muscle files and writes an OpenSim model file.
	
	July 17, 2007
	Added a flag to specify whether to write SimmKinematicsEngine- or
	SimbodyEngine-based model.

   Routines:

*******************************************************************************/

#include <float.h>
#include <errno.h>

#include "universal.h"
#include "main.h"
#include "wefunctions.h"
#include "sdfunctions.h"
#if OPENSIM_BUILD
#include <OpenSim/version.h>
#endif

typedef enum {
   SIMM_ENGINE=0,
   SIMBODY_ENGINE
} EngineType;

static char* gEngineName[] = {"Simm", "Simbody"};

#define GROUND_NAME "ground"

static double gWorld_x[] = {1.0, 0.0, 0.0, 0.0};
static double gWorld_y[] = {0.0, 1.0, 0.0, 0.0};
static double gWorld_z[] = {0.0, 0.0, 1.0, 0.0};

static EngineType gEngineType = SIMM_ENGINE;
static char* gMarkerSetOut = NULL;
int gAngleUnits = RADIANS;

ReturnCode make_simbody_model(FILE* fp, ModelStruct* ms, char geometryDirectory[]);
void convert_dof_to_function(ModelStruct* ms, DofStruct* dof, int gencoord);
ReturnCode write_xml_model(ModelStruct* ms, char filename[], char geometryDirectory[]);


static void printUsage(char programName[])
{
	printf("Usage: %s -j joints_in [ -m muscles_in] [-e simm | simbody] -x xml_out [-ms markerset_out] [-g geometry_directory] [-a degrees | radians]\n", programName);
}


// Fix names in the model to take out XML meta characters that cause parsing problems (bug 505). -Ayman
static void convert_model_names(ModelStruct* ms)
{
	int i=0;
	int j=0;
	// Check that names are legal. Segments, then muscles, groups, then wrap objects
	for (i = 0; i < ms->numsegments; i++)
	{
		char* name = ms->segment[i].name;
		convert_string(name, no);
			for (j=0; j<ms->segment[i].numMarkers; j++){
				char* markerName = ms->segment[i].marker[j].name;
				convert_string(markerName, no);
			}
	}

	for (i = 0; i < ms->nummuscles; i++)
	{
		char* name = ms->muscle[i].name;
		convert_string(name, no);
	}

	for (i = 0; i < ms->numgroups; i++)
	{
		char* name = ms->muscgroup[i].name;
		convert_string(name, no);
	}
}

int main(int argc, char* argv[])
{
   ModelStruct* ms;
	char *jointIn = NULL, *muscleIn = NULL, *xmlOut = NULL, *geometryDirectory = NULL;

#if OPENSIM_BUILD
	printf("simmToOpenSim, version %s, build date %s %s\n", OpenSimVersion, __TIME__, __DATE__);
#endif

   if (argc < 5)
   {
		printUsage(argv[0]);
      exit(0);
   }
	else
	{
		int i;

		for (i = 1; i < argc - 1; i++)
		{
			if (STRINGS_ARE_EQUAL_CI(argv[i], "-J"))
			{
				jointIn = argv[i+1];
				i++; 
			}
			else if (STRINGS_ARE_EQUAL_CI(argv[i], "-M"))
			{
				muscleIn = argv[i+1];
				i++; 
			}
			else if (STRINGS_ARE_EQUAL_CI(argv[i], "-E"))
			{
				if (STRINGS_ARE_EQUAL_CI(argv[i+1], "SIMM"))
				{
					printf("\n\nGenerating an OpenSim model with the SIMM kinematics engine as the underlying dynamics engine.\n");
					printf("Note that this model will not support dynamic simulations or analyses.\n");
					printf("However, it is possible to convert this model to use SDFast as the underlying engine\n");
					printf("once the OpenSim model has been generated using the utility makeSDFastModel.exe.\n\n");
					gEngineType = SIMM_ENGINE;
				}
				else if (STRINGS_ARE_EQUAL_CI(argv[i+1], "SIMBODY"))
				{
					printf("\n\nGenerating an OpenSim model with Simbody as the underlying dynamics engine.\n\n");
               gEngineType = SIMBODY_ENGINE;
				}
				else
				{
					printf("\nDynamics engine type unrecognized \"-e\" must be \"simm\" or \"simbody\".\n");
					printf("\nProceeding assuming conversion to a model based on a SIMM kinematics engine is desired.\n");
				}
				i++; 
			}
			else if (STRINGS_ARE_EQUAL_CI(argv[i], "-X"))
			{
				xmlOut = argv[i+1];
				i++; 
			}
			else if (STRINGS_ARE_EQUAL_CI(argv[i], "-MS"))
			{
				gMarkerSetOut = argv[i+1];
				i++; 
			}
			else if (STRINGS_ARE_EQUAL_CI(argv[i], "-G"))
			{
				geometryDirectory = argv[i+1];
				i++; 
			}
			else if (STRINGS_ARE_EQUAL_CI(argv[i], "-A"))
			{
				if (STRINGS_ARE_EQUAL_CI(argv[i+1], "DEGREES"))
					gAngleUnits = DEGREES;
				else if (STRINGS_ARE_EQUAL_CI(argv[i+1], "RADIANS"))
					gAngleUnits = RADIANS;
				else
					printf("Parameter after \"-a\" must be one of: degrees, DEGREES, radians, RADIANS.\n");
				i++; 
			}
			else
			{
				printf("Unrecognized option (%s) on command line.", argv[i]);
				printUsage(argv[0]);
				exit(0);
			}
		}
	}

	if (!jointIn || !xmlOut)
	{
		printUsage(argv[0]);
		exit(0);
	}

   ms = model[0] = (ModelStruct*)simm_calloc(1,sizeof(ModelStruct));
   ms->modelnum = 0;

   init_model(ms);

   mstrcpy(&ms->jointfilename, jointIn);

   read_model_file(ms->modelnum, ms->jointfilename, yes);

   // If the muscle file name was specified on the command line,
	// override the one specified in the joint file (if any).
   if (muscleIn)
      mstrcpy(&ms->musclefilename, muscleIn);

   if (check_definitions(ms) == code_bad)
      exit(0);

   find_ground_joint(ms); /* must be called before makepaths() ! */

   if (makepaths(ms->modelnum) == code_bad)
      exit(0);

	init_gencoords(ms);

	set_gencoord_info(ms);

   printf("Read joint file %s\n", ms->jointfilename);

	if (ms->musclefilename)
	{
		SBoolean foo;
		read_muscle_file(ms, ms->musclefilename, &foo, yes);
	}

   // Make sure the name of the ground segment is "ground".
   if (STRINGS_ARE_NOT_EQUAL(ms->segment[ms->ground_segment].name, GROUND_NAME))
   {
      free(ms->segment[ms->ground_segment].name);
      mstrcpy(&ms->segment[ms->ground_segment].name, GROUND_NAME);
   }

	// Make sure names are legal: meeting the following criteria Alphanumeric with _ or - or . allowed
	convert_model_names(ms);

   if (write_xml_model(ms, xmlOut, geometryDirectory) == code_fine)
	   printf("Wrote OpenSim model file %s\n", xmlOut);
   else
      printf("Creation of OpenSim model from %s failed.\n", ms->jointfilename);

	exit(0);
}

void write_xml_gravity(FILE* fp, ModelStruct* ms)
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

void write_xml_muscle_groups(FILE* fp, ModelStruct* ms)
{
	int i, j;

	fprintf(fp, "\t<groups>\n");
	for (i = 0; i < ms->numgroups; i++)
	{
		fprintf(fp, "\t\t<ObjectGroup name=\"%s\">\n", ms->muscgroup[i].name);
		fprintf(fp, "\t\t\t<members>");
		for (j = 0; j < ms->muscgroup[i].number_of_muscles; j++)
			fprintf(fp, "%s ", ms->muscle[ms->muscgroup[i].muscle_index[j]].name);
		fprintf(fp, "</members>\n");
		fprintf(fp, "\t\t</ObjectGroup>\n");
	}
	fprintf(fp, "\t</groups>\n");
}

#define NEED_TO_WRITE_MUSCLE_VALUE(field) \
	m->field && (writingDefault || m->field != ms->default_muscle.field)

void write_xml_muscle(FILE* fp, ModelStruct* ms, MuscleStruct* m, const char* muscleClassName, int writingDefault)
{
	int j;

	fprintf(fp, "\t\t<%s name=\"%s\">\n", muscleClassName, m->name);

	/* Attachment points. */
	if (m->musclepoints && m->musclepoints->num_orig_points > 0) {
		fprintf(fp, "\t\t\t<MusclePointSet>\n");
		fprintf(fp, "\t\t\t<objects>\n");
		for (j = 0; j < m->musclepoints->num_orig_points; j++)
		{
			MusclePoint* mp = &m->musclepoints->mp_orig[j];
			if (mp->isMovingPoint)
			{
				fprintf(fp, "\t\t\t\t<MovingMusclePoint>\n");
				fprintf(fp, "\t\t\t\t\t<body> %s </body>\n", ms->segment[getMusclePointSegment(m,j)].name);
            // Check if OpenSim can handle one of the attachment coordinates being a constant.
            // If it can't then you have to make up a gencoord name and use it in the natural
            // cubic spline (2 points, constant value).
				fprintf(fp, "\t\t\t\t</MovingMusclePoint>\n");
			}
			else if (mp->isVia)
			{
				// If the gencoord used for this via point range is [primarily] translational, then do
				// not convert the range values. If it's rotational, then use DEG_TO_RAD (if gAngleUnits
            // is RADIANS, which means the user wants radians in the output file).
				double conversion;
				if (ms->gencoord[mp->viaRange.genc].type == rotation_gencoord && gAngleUnits == RADIANS)
					conversion = DEG_TO_RAD;
				else
					conversion = 1.0;

				fprintf(fp, "\t\t\t\t<MuscleViaPoint>\n");
				fprintf(fp, "\t\t\t\t\t<location> %.12lf %.12lf %.12lf </location>\n", mp->point[0],
               mp->point[1], mp->point[2]);
				fprintf(fp, "\t\t\t\t\t<body> %s </body>\n", ms->segment[getMusclePointSegment(m,j)].name);
				fprintf(fp, "\t\t\t\t\t<coordinate> %s </coordinate>\n", ms->gencoord[mp->viaRange.genc].name);
				fprintf(fp, "\t\t\t\t\t<range> %.12lf %.12lf </range>\n", mp->viaRange.start * conversion,
               mp->viaRange.end * conversion);
				fprintf(fp, "\t\t\t\t</MuscleViaPoint>\n");
			}
			else // regular muscle point
			{
				fprintf(fp, "\t\t\t\t<MusclePoint>\n");
				fprintf(fp, "\t\t\t\t\t<location> %.12lf %.12lf %.12lf </location>\n", mp->point[0],
               mp->point[1], mp->point[2]);
				fprintf(fp, "\t\t\t\t\t<body> %s </body>\n", ms->segment[getMusclePointSegment(m,j)].name);
				fprintf(fp, "\t\t\t\t</MusclePoint>\n");
			}
		}
		fprintf(fp, "\t\t\t</objects>\n");
		fprintf(fp, "\t\t\t</MusclePointSet>\n");
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
      if (gAngleUnits == RADIANS)
		   fprintf(fp, "\t\t\t<pennation_angle> %.12lf </pennation_angle>\n", *m->pennation_angle * DEG_TO_RAD);
      else
		   fprintf(fp, "\t\t\t<pennation_angle> %.12lf </pennation_angle>\n", *m->pennation_angle);
   }
	if (NEED_TO_WRITE_MUSCLE_VALUE(max_contraction_vel))
		fprintf(fp, "\t\t\t<max_contraction_velocity> %.12lf </max_contraction_velocity>\n", *m->max_contraction_vel);

	/* muscle model. */
	if (m->muscle_model_index)	// Conservative fix in case muscle model is not specified. Ayman 1/07
		fprintf(fp, "\t\t\t<muscle_model> %d </muscle_model>\n", *m->muscle_model_index);

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

	if (m->numWrapStructs > 0)
	{
		fprintf(fp, "\t\t\t<MuscleWrapSet>\n");
		fprintf(fp, "\t\t\t<objects>\n");
		for (j = 0; j < m->numWrapStructs; j++)
		{
			WrapObject* wo = &ms->wrapobj[m->wrapStruct[j]->wrap_object];
			fprintf(fp, "\t\t\t\t<MuscleWrap>\n");
			fprintf(fp, "\t\t\t\t\t<wrap_object> %s </wrap_object>\n", wo->name);
			if (wo->wrap_type == wrap_ellipsoid)
				fprintf(fp, "\t\t\t\t\t<method> %s </method>\n", get_wrap_algorithm_name(m->wrapStruct[j]->wrap_algorithm));
			if (m->wrapStruct[j]->startPoint > 0 || m->wrapStruct[j]->endPoint > 0)
				fprintf(fp,"\t\t\t\t\t<range> %d %d </range>\n", m->wrapStruct[j]->startPoint, m->wrapStruct[j]->endPoint);
			fprintf(fp, "\t\t\t\t</MuscleWrap>\n");
		}
		fprintf(fp, "\t\t\t</objects>\n");
		fprintf(fp, "\t\t\t</MuscleWrapSet>\n");
	}

	/* Tendon force-length curve -- only write if non-default value. */
	if (NEED_TO_WRITE_MUSCLE_VALUE(tendon_force_len_curve))
	{
		fprintf(fp, "\t\t\t<tendon_force_length_curve>\n");
		fprintf(fp, "\t\t\t\t<natCubicSpline>\n");
		fprintf(fp, "\t\t\t\t\t<x>");
		for (j = 0; j < m->tendon_force_len_curve->numpoints; j++)
			fprintf(fp, "%.12lf ", m->tendon_force_len_curve->x[j]);
		fprintf(fp, "</x>\n");
		fprintf(fp, "\t\t\t\t\t<y>");
		for (j = 0; j < m->tendon_force_len_curve->numpoints; j++)
			fprintf(fp, "%.12lf ", m->tendon_force_len_curve->y[j]);
		fprintf(fp, "</y>\n");
		fprintf(fp, "\t\t\t\t</natCubicSpline>\n");
		fprintf(fp, "\t\t\t</tendon_force_length_curve>\n");
	}

	/* Active force-length curve -- only write if non-default value. */
	if (NEED_TO_WRITE_MUSCLE_VALUE(active_force_len_curve))
	{
		fprintf(fp, "\t\t\t<active_force_length_curve>\n");
		fprintf(fp, "\t\t\t\t<natCubicSpline>\n");
		fprintf(fp, "\t\t\t\t\t<x>");
		for (j = 0; j < m->active_force_len_curve->numpoints; j++)
			fprintf(fp, "%.12lf ", m->active_force_len_curve->x[j]);
		fprintf(fp, "</x>\n");
		fprintf(fp, "\t\t\t\t\t<y>");
		for (j = 0; j < m->active_force_len_curve->numpoints; j++)
			fprintf(fp, "%.12lf ", m->active_force_len_curve->y[j]);
		fprintf(fp, "</y>\n");
		fprintf(fp, "\t\t\t\t</natCubicSpline>\n");
		fprintf(fp, "\t\t\t</active_force_length_curve>\n");
	}

	/* Passive force-length curve -- only write if non-default. */
	if (NEED_TO_WRITE_MUSCLE_VALUE(passive_force_len_curve))
	{
		fprintf(fp, "\t\t\t<passive_force_length_curve>\n");
		fprintf(fp, "\t\t\t\t<natCubicSpline>\n");
		fprintf(fp, "\t\t\t\t\t<x>");
		for (j = 0; j < m->passive_force_len_curve->numpoints; j++)
			fprintf(fp, "%.12lf ", m->passive_force_len_curve->x[j]);
		fprintf(fp, "</x>\n");
		fprintf(fp, "\t\t\t\t\t<y>");
		for (j = 0; j < m->passive_force_len_curve->numpoints; j++)
			fprintf(fp, "%.12lf ", m->passive_force_len_curve->y[j]);
		fprintf(fp, "</y>\n");
		fprintf(fp, "\t\t\t\t</natCubicSpline>\n");
		fprintf(fp, "\t\t\t</passive_force_length_curve>\n");
	}

	/* Force-velocity curve -- only write if non-default. */
	if (NEED_TO_WRITE_MUSCLE_VALUE(force_vel_curve))
	{
		fprintf(fp, "\t\t\t<force_velocity_curve>\n");
		fprintf(fp, "\t\t\t\t<natCubicSpline>\n");
		fprintf(fp, "\t\t\t\t\t<x>");
		for (j = 0; j < m->force_vel_curve->numpoints; j++)
			fprintf(fp, "%.12lf ", m->force_vel_curve->x[j]);
		fprintf(fp, "</x>\n");
		fprintf(fp, "\t\t\t\t\t<y>");
		for (j = 0; j < m->force_vel_curve->numpoints; j++)
			fprintf(fp, "%.12lf ", m->force_vel_curve->y[j]);
		fprintf(fp, "</y>\n");
		fprintf(fp, "\t\t\t\t</natCubicSpline>\n");
		fprintf(fp, "\t\t\t</force_velocity_curve>\n");
	}

	fprintf(fp, "\t\t</%s>\n", muscleClassName);

}

void write_xml_muscles(FILE* fp, ModelStruct* ms)
{
	int i;
	char muscleClassName[64];

	fprintf(fp, "\t<ActuatorSet>\n");
	write_xml_muscle_groups(fp, ms);
	fprintf(fp, "\t<objects>\n");
	for (i = 0; i < ms->nummuscles; i++)
	{
		MuscleStruct* m = &ms->muscle[i];

		// For now, model = 9 and map to Thelen2003Muscle.
		// All other values map to Schutte1993Muscle. Eventually, each model
		// index should map to a distinct muscle class in OpenSim,
		// especially model = 7, which is a ligament model.
		if (m->muscle_model_index == NULL || *m->muscle_model_index == 4)
			strcpy(muscleClassName, "Schutte1993Muscle");
		else if (*m->muscle_model_index == 9)
			strcpy(muscleClassName, "Thelen2003Muscle");
		else{	// Warn about unsupported but use Schutte1993Muscle
			strcpy(muscleClassName, "Schutte1993Muscle");
		    printf("Warning: muscle %s has unsupported muscle model %d.\n", m->name, *m->muscle_model_index);
		}
		write_xml_muscle(fp, ms, m, muscleClassName, 0);
	}
	fprintf(fp, "\t</objects>\n");
	fprintf(fp, "\t</ActuatorSet>\n");
}


void write_xml_markers(FILE* fp, ModelStruct* ms)
{
	int i, j;
	char tabs[5];

	if(gMarkerSetOut) {
		fp = fopen(gMarkerSetOut, "w");
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
			fprintf(fp, "%s\t\t<Marker name=\"%s\">\n", tabs, ss->marker[j].name);
			fprintf(fp, "%s\t\t\t<body> %s </body>\n", tabs, ss->name);
			fprintf(fp, "%s\t\t\t<location> %.12lf %.12lf %.12lf </location>\n", tabs, ss->marker[j].offset[0], ss->marker[j].offset[1], ss->marker[j].offset[2]);
			fprintf(fp, "%s\t\t\t<weight> %.12lf </weight>\n", tabs, ss->marker[j].weight);
			fprintf(fp, "%s\t\t\t<fixed> %s </fixed>\n", tabs, (ss->marker[j].fixed == yes) ? ("true") : ("false"));
			fprintf(fp, "%s\t\t</Marker>\n", tabs);
		}
	}

	fprintf(fp, "%s\t</objects>\n", tabs);
	fprintf(fp, "%s</MarkerSet>\n", tabs);

	if(gMarkerSetOut) {
		fclose(fp);
		printf("Wrote MarkerSet file %s\n", gMarkerSetOut);
	}
}


void write_vtk_bone(PolyhedronStruct* bone, char geometryDirectory[], char filename[])
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

int segment_has_wrap_objects(ModelStruct* ms, SegmentStruct* seg)
{
	int i;

	for (i = 0; i < ms->num_wrap_objects; i++)
		if (&ms->segment[ms->wrapobj[i].segment] == seg)
			return 1;

	return 0;
}

void write_xml_wrap_object(FILE* fp, WrapObject* wo)
{
	double xyz[3];
	const char *wrap_quadrant;
	double conversion;

	// The angles output by extract_xyz_rot_bodyfixed are in radians,
	// so this conversion factor is different than in the other functions.
	if (gAngleUnits == DEGREES)
		conversion = RAD_TO_DEG;
	else
		conversion = 1.0;

	switch (wo->wrap_type)
	{
	   case wrap_sphere:
		   fprintf(fp, "\t\t\t\t\t\t<WrapSphere name=\"%s\">\n", wo->name);
			fprintf(fp, "\t\t\t\t\t\t\t<radius> %.8lf </radius>\n", wo->radius.xyz[0]);
			break;
		case wrap_cylinder:
			fprintf(fp, "\t\t\t\t\t\t<WrapCylinder name=\"%s\">\n", wo->name);
			fprintf(fp, "\t\t\t\t\t\t\t<radius> %.8lf </radius>\n", wo->radius.xyz[0]);
			fprintf(fp, "\t\t\t\t\t\t\t<length> %.8lf </length>\n", wo->height);
			break;
		case wrap_ellipsoid:
			fprintf(fp, "\t\t\t\t\t\t<WrapEllipsoid name=\"%s\">\n", wo->name);
			fprintf(fp, "\t\t\t\t\t\t\t<dimensions> %.8lf %.8lf %.8lf </dimensions>\n",
				wo->radius.xyz[0], wo->radius.xyz[1], wo->radius.xyz[2]);
			break;
		case wrap_torus:
			fprintf(fp, "\t\t\t\t\t\t<WrapTorus name=\"%s\">\n", wo->name);
			fprintf(fp, "\t\t\t\t\t\t\t<inner_radius> %.8lf </inner_radius>\n", wo->radius.xyz[0]);
			fprintf(fp, "\t\t\t\t\t\t\t<outer_radius> %.8lf </outer_radius>\n", wo->radius.xyz[1]);
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
	   case wrap_sphere:
		   fprintf(fp, "\t\t\t\t\t\t</WrapSphere>\n");
			break;
		case wrap_cylinder:
			fprintf(fp, "\t\t\t\t\t\t</WrapCylinder>\n");
			break;
		case wrap_ellipsoid:
			fprintf(fp, "\t\t\t\t\t\t</WrapEllipsoid>\n");
			break;
		case wrap_torus:
			fprintf(fp, "\t\t\t\t\t\t</WrapTorus>\n");
			break;
	}
}

void write_xml_bodies(FILE* fp, ModelStruct* ms, char geometryDirectory[])
{
	int i, j;
	SBoolean madeDir = no;

	fprintf(fp, "\t\t\t<BodySet>\n");
	fprintf(fp, "\t\t\t<objects>\n");
	for (i = 0; i < ms->numsegments; i++)
	{
		SegmentStruct* ss = &ms->segment[i];
		fprintf(fp, "\t\t\t\t<%sBody name=\"%s\">\n", gEngineName[gEngineType], ss->name);
		fprintf(fp, "\t\t\t\t\t<mass> %.12lf </mass>\n", ss->mass);
		fprintf(fp, "\t\t\t\t\t<mass_center> %.12lf %.12lf %.12lf </mass_center>\n", ss->masscenter[0], ss->masscenter[1], ss->masscenter[2]);
		fprintf(fp, "\t\t\t\t\t<inertia> %.12lf %.12lf %.12lf %.12lf %.12lf %.12lf %.12lf %.12lf %.12lf </inertia>\n", ss->inertia[0][0], ss->inertia[0][1], ss->inertia[0][2],
			ss->inertia[1][0], ss->inertia[1][1], ss->inertia[1][2],
			ss->inertia[2][0], ss->inertia[2][1], ss->inertia[2][2]);
		if (ss->numBones > 0)
		{
			char vtpFileName[CHARBUFFER];

			fprintf(fp, "\t\t\t\t\t<VisibleObject name=\"\">\n");
			fprintf(fp, "\t\t\t\t\t\t<geometry_files> ");
			for (j = 0; j < ss->numBones; j++)
			{
				change_filename_suffix(ss->bone[j].name, vtpFileName, "vtp");
				fprintf(fp, "%s ", vtpFileName);
				// Only write the bone file if the PolyhedronStruct is not empty
				if (ss->bone[j].num_vertices > 0 && ss->bone[j].num_polygons > 0)
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
					write_vtk_bone(&ss->bone[j], geometryDirectory, vtpFileName);
				}
			}
			fprintf(fp, "</geometry_files>\n");
			fprintf(fp, "\t\t\t\t\t\t<scale_factors> %.12f %.12f %.12f </scale_factors>\n", ss->bone_scale[0], ss->bone_scale[1], ss->bone_scale[2]);
			fprintf(fp, "\t\t\t\t\t</VisibleObject>\n");
		}
		if (segment_has_wrap_objects(ms, ss))
		{
			fprintf(fp, "\t\t\t\t\t<WrapObjectSet>\n");
			fprintf(fp, "\t\t\t\t\t<objects>\n");
			for (j = 0; j < ms->num_wrap_objects; j++)
			{
				if (ms->wrapobj[j].segment == i)
					write_xml_wrap_object(fp, &ms->wrapobj[j]);
			}
			fprintf(fp, "\t\t\t\t\t</objects>\n");
			fprintf(fp, "\t\t\t\t\t</WrapObjectSet>\n");
		}
		fprintf(fp, "\t\t\t\t</%sBody>\n", gEngineName[gEngineType]);
	}
	fprintf(fp, "\t\t\t</objects>\n");
	fprintf(fp, "\t\t\t</BodySet>\n");
}


void write_xml_coordinates(FILE* fp, ModelStruct* ms)
{
	int i, j;

	fprintf(fp, "\t\t\t<CoordinateSet>\n");
	fprintf(fp, "\t\t\t<objects>\n");
	for (i = 0; i < ms->numgencoords; i++)
	{
      double conversion;
      // If the gencoord is [primarily] translational, then do not convert its
	   // range, default_value, etc. If it's rotational, then use DEG_TO_RAD
      // (if gAngleUnits == RADIANS, which means the user wants radians in the
      // output file).
      if (ms->gencoord[i].type == rotation_gencoord && gAngleUnits == RADIANS)
         conversion = DEG_TO_RAD;
      else
         conversion = 1.0;

		fprintf(fp, "\t\t\t\t<%sCoordinate name=\"%s\">\n", gEngineName[gEngineType], ms->gencoord[i].name);
		fprintf(fp, "\t\t\t\t\t<range> %.12lf %.12lf </range>\n", ms->gencoord[i].range.start * conversion, ms->gencoord[i].range.end * conversion);
		fprintf(fp, "\t\t\t\t\t<default_value> %.12lf </default_value>\n", ms->gencoord[i].default_value * conversion);
		fprintf(fp, "\t\t\t\t\t<tolerance> %.12lf </tolerance>\n", ms->gencoord[i].tolerance);
		fprintf(fp, "\t\t\t\t\t<stiffness> %.12lf </stiffness>\n", ms->gencoord[i].pd_stiffness);
      if (ms->gencoord[i].keys[0] != null_key)
      {
         fprintf(fp, "\t\t\t\t\t<keys> %s ", get_simmkey_name((int)ms->gencoord[i].keys[0]));
         if (ms->gencoord[i].keys[1] != ms->gencoord[i].keys[0])
            fprintf(fp, "%s ", get_simmkey_name((int)ms->gencoord[i].keys[1]));
         fprintf(fp,"</keys>\n");
      }
		fprintf(fp, "\t\t\t\t\t<clamped> %s </clamped>\n", (ms->gencoord[i].clamped == yes) ? ("true") : ("false"));
		fprintf(fp, "\t\t\t\t\t<locked> %s </locked>\n", (ms->gencoord[i].locked == yes) ? ("true") : ("false"));
		fprintf(fp, "\t\t\t\t\t<restraint_active> %s </restraint_active>\n", (ms->gencoord[i].restraintFuncActive == yes) ? ("true") : ("false"));
		if (ms->gencoord[i].restraint_func_num != -1)
		{
			SplineFunction* sf = &ms->function[ms->gencoord[i].restraint_func_num];

			fprintf(fp, "\t\t\t<restraint_function>\n");
			fprintf(fp, "\t\t\t\t\t\t<natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t\t\t<x>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->x[j] * conversion);
			fprintf(fp, "</x>\n");
			fprintf(fp, "\t\t\t\t\t\t\t<y>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->y[j]);
			fprintf(fp, "</y>\n");
			fprintf(fp, "\t\t\t\t\t\t</natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t</restraint_function>\n");
		}
		if (ms->gencoord[i].min_restraint_func_num != -1)
		{
			SplineFunction* sf = &ms->function[ms->gencoord[i].min_restraint_func_num];

			fprintf(fp, "\t\t\t\t\t<min_restraint_function>\n");
			fprintf(fp, "\t\t\t\t\t\t<natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t\t\t<x>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->x[j] * conversion);
			fprintf(fp, "</x>\n");
			fprintf(fp, "\t\t\t\t\t\t\t<y>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->y[j]);
			fprintf(fp, "</y>\n");
			fprintf(fp, "\t\t\t\t\t\t</natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t</min_restraint_function>\n");
		}
		if (ms->gencoord[i].max_restraint_func_num != -1)
		{
			SplineFunction* sf = &ms->function[ms->gencoord[i].max_restraint_func_num];

			fprintf(fp, "\t\t\t\t\t<max_restraint_function>\n");
			fprintf(fp, "\t\t\t\t\t\t<natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t\t\t<x>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->x[j] * conversion);
			fprintf(fp, "</x>\n");
			fprintf(fp, "\t\t\t\t\t\t\t<y>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->y[j]);
			fprintf(fp, "</y>\n");
			fprintf(fp, "\t\t\t\t\t\t</natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t</max_restraint_function>\n");
		}
		fprintf(fp, "\t\t\t\t</%sCoordinate>\n", gEngineName[gEngineType]);
	}
	fprintf(fp, "\t\t\t</objects>\n");
	fprintf(fp, "\t\t\t</CoordinateSet>\n");
}


void write_xml_speeds(FILE* fp, ModelStruct* ms)
{
	int i;

	fprintf(fp, "\t\t\t<SpeedSet>\n");
	fprintf(fp, "\t\t\t<objects>\n");
	for (i = 0; i < ms->numgencoords; i++)
	{
		fprintf(fp, "\t\t\t\t<%sSpeed name=\"%s_u\">\n", gEngineName[gEngineType] ,ms->gencoord[i].name);
		fprintf(fp, "\t\t\t\t\t<default_value> 0.0 </default_value>\n");
		fprintf(fp, "\t\t\t\t\t<coordinate> %s </coordinate>\n",ms->gencoord[i].name);
		fprintf(fp, "\t\t\t\t</%sSpeed>\n", gEngineName[gEngineType]);
	}
	fprintf(fp, "\t\t\t</objects>\n");
	fprintf(fp, "\t\t\t</SpeedSet>\n");
}


void write_xml_joints(FILE* fp, ModelStruct* ms)
{
	int i, j, k, m, rotCount;

	fprintf(fp, "\t\t\t<JointSet>\n");
	fprintf(fp, "\t\t\t<objects>\n");
	for (i = 0; i < ms->numjoints; i++)
	{
		JointStruct* js = &ms->joint[i];
		fprintf(fp, "\t\t\t\t<%sJoint name=\"%s\">\n", gEngineName[gEngineType], ms->joint[i].name);
		fprintf(fp, "\t\t\t\t\t<bodies> %s %s </bodies>\n", ms->segment[js->from].name, ms->segment[js->to].name);
		fprintf(fp, "\t\t\t\t\t<DofSet>\n");
		fprintf(fp, "\t\t\t\t\t<objects>\n");
		for (j = 0, rotCount = 0; j < 4; j++)
		{
			if (j == js->order[TRANS]) // TX, TY, and TZ
			{
				char* translationNames[] = {"tx", "ty", "tz"};

				for (k = 3; k < 6; k++)
				{
					fprintf(fp, "\t\t\t\t\t\t<%sTranslationDof name=\"%s\">\n", gEngineName[gEngineType], translationNames[k-3]);
					if (js->dofs[k].type == constant_dof)
					{
						fprintf(fp, "\t\t\t\t\t\t\t<Value>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t<Constant>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t\t<value> %.12lf </value>\n", js->dofs[k].value);
						fprintf(fp, "\t\t\t\t\t\t\t\t</Constant>\n");
						fprintf(fp, "\t\t\t\t\t\t\t</Value>\n");
					}
					else
					{
						SplineFunction* sf = &ms->function[js->dofs[k].funcnum];
                  double conversion;
						// This dof is a function of a gencoord. If the gencoord is [primarily] translational,
						// then do not convert its X points. If it's rotational, then use DEG_TO_RAD
						// (if gAngleUnits == RADIANS, which means the user wants radians in the output file).
                  // The Y points are never converted because the dof itself is translational.
                  if (ms->gencoord[js->dofs[k].gencoord].type == rotation_gencoord && gAngleUnits == RADIANS)
                     conversion = DEG_TO_RAD;
                  else
                     conversion = 1.0;

						fprintf(fp, "\t\t\t\t\t\t\t<coordinate> %s </coordinate>\n", ms->gencoord[js->dofs[k].gencoord].name);
						fprintf(fp, "\t\t\t\t\t\t\t<Value>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t<natCubicSpline>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t\t<x>");
						for (m = 0; m < sf->numpoints; m++)
							fprintf(fp, "%.12lf ", sf->x[m] * conversion);
						fprintf(fp, "</x>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t\t<y>");
						for (m = 0; m < sf->numpoints; m++)
							fprintf(fp, "%.12lf ", sf->y[m]);
						fprintf(fp, "</y>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t</natCubicSpline>\n");
						fprintf(fp, "\t\t\t\t\t\t\t</Value>\n");
					}
					fprintf(fp, "\t\t\t\t\t\t</%sTranslationDof>\n", gEngineName[gEngineType]);
				}
			}
			else // R1, R2, R3
			{
            int axis;
            double conversionA;

            if (gAngleUnits == RADIANS)
               conversionA = DEG_TO_RAD;
            else
               conversionA = 1.0;

            if (j == js->order[ROT1])
               axis = 0;
            else if (j == js->order[ROT2])
               axis = 1;
            else // ROT3
               axis = 2;

            // Rename the rotation dofs so that they're always in the order: r1, r2, r3 in the XML file.
            // These names aren't needed by simTK, but they could come in handy for debugging.
            fprintf(fp, "\t\t\t\t\t\t<%sRotationDof name=\"%s\">\n", gEngineName[gEngineType], getjointvarname(rotCount++));
            fprintf(fp, "\t\t\t\t\t\t\t<axis> %.12lf %.12lf %.12lf </axis>\n", js->parentrotaxes[axis][0], js->parentrotaxes[axis][1], js->parentrotaxes[axis][2]);
            if (js->dofs[axis].type == constant_dof)
            {
               fprintf(fp, "\t\t\t\t\t\t\t<Value>\n");
               fprintf(fp, "\t\t\t\t\t\t\t\t<Constant>\n");
               fprintf(fp, "\t\t\t\t\t\t\t\t\t<value> %.12lf </value>\n", js->dofs[axis].value * conversionA);
               fprintf(fp, "\t\t\t\t\t\t\t\t</Constant>\n");
               fprintf(fp, "\t\t\t\t\t\t\t</Value>\n");
				}
				else
				{
               SplineFunction* sf = &ms->function[js->dofs[axis].funcnum];
               double conversion;
					// This dof is a function of a gencoord. If the gencoord is [primarily] translational,
					// then do not convert its X points. If it's rotational, then use DEG_TO_RAD
					// (if gAngleUnits == RADIANS, which means the user wants radians in the output file).
               // The Y points are always converted because the dof itself is rotational.
               if (ms->gencoord[js->dofs[axis].gencoord].type == rotation_gencoord && gAngleUnits == RADIANS)
                  conversion = DEG_TO_RAD;
               else
                  conversion = 1.0;

					fprintf(fp, "\t\t\t\t\t\t\t<coordinate> %s </coordinate>\n", ms->gencoord[js->dofs[axis].gencoord].name);
					fprintf(fp, "\t\t\t\t\t\t\t<Value>\n");
					fprintf(fp, "\t\t\t\t\t\t\t\t<natCubicSpline>\n");
					fprintf(fp, "\t\t\t\t\t\t\t\t\t<x>");
					for (m = 0; m < sf->numpoints; m++)
						fprintf(fp, "%.12lf ", sf->x[m] * conversion);
					fprintf(fp, "</x>\n");
					fprintf(fp, "\t\t\t\t\t\t\t\t\t<y>");
					for (m = 0; m < sf->numpoints; m++)
						fprintf(fp, "%.12lf ", sf->y[m] * conversionA);
					fprintf(fp, "</y>\n");
					fprintf(fp, "\t\t\t\t\t\t\t\t</natCubicSpline>\n");
					fprintf(fp, "\t\t\t\t\t\t\t</Value>\n");
				}
				fprintf(fp, "\t\t\t\t\t\t</%sRotationDof>\n", gEngineName[gEngineType]);
			}
		}
		fprintf(fp, "\t\t\t\t\t</objects>\n");
		fprintf(fp, "\t\t\t\t\t</DofSet>\n");
		fprintf(fp, "\t\t\t\t</%sJoint>\n", gEngineName[gEngineType]);
	}
	fprintf(fp, "\t\t\t</objects>\n");
	fprintf(fp, "\t\t\t</JointSet>\n");
}


ReturnCode write_xml_ke(FILE* fp, ModelStruct* ms, char geometryDirectory[])
{
	fprintf(fp, "\t<DynamicsEngine>\n");
	fprintf(fp, "\t\t<%sEngine>\n", gEngineName[gEngineType]);
	write_xml_gravity(fp, ms);
	write_xml_markers(fp, ms);
	write_xml_bodies(fp, ms, geometryDirectory);
	write_xml_coordinates(fp, ms);
	if (gEngineType == SIMBODY_ENGINE)
      write_xml_speeds(fp, ms);
	write_xml_joints(fp, ms);
	fprintf(fp, "\t\t</%sEngine>\n", gEngineName[gEngineType]);
	fprintf(fp, "\t</DynamicsEngine>\n");

   return code_fine;
}

void write_xml_units(FILE* fp, ModelStruct* ms)
{
   if (ms->forceUnits != NULL)
      fprintf(fp, "\t<force_units> %s </force_units>\n", ms->forceUnits);

   if (ms->lengthUnits != NULL)
      fprintf(fp, "\t<length_units> %s </length_units>\n", ms->lengthUnits);
}

void write_xml_defaults(FILE* fp, ModelStruct* ms)
{
	fprintf(fp, "\t<defaults>\n");
	// We need to write a default muscle for all supported types of muscles (since
	// the default mechanism works by matching class names)
	write_xml_muscle(fp, ms, &ms->default_muscle, "Schutte1993Muscle", 1);
	write_xml_muscle(fp, ms, &ms->default_muscle, "Thelen2003Muscle", 1);
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
void extract_joint_locations_and_orientations(ModelStruct* ms,
                                              JointStruct* joint,
                                              int dof_order[],
                                              Direction dir,
                                              double locationInParent[],
                                              double orientationInParent[],
                                              double locationInChild[],
                                              double orientationInChild[])
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

   //TODO20: reverse axes for INVERSE, or maybe just negate dof value?

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
   COPY_1X4VECTOR(joint->parentrotaxes[R2],ra2);
   COPY_1X4VECTOR(joint->parentrotaxes[R3],ra3);

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
   COPY_1X4VECTOR(joint->parentrotaxes[R2],ra2);
   COPY_1X4VECTOR(joint->parentrotaxes[R3],ra3);

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

ReturnCode write_opensim20_engine(FILE* fp, ModelStruct* ms, char geometryDirectory[])
{
	fprintf(fp, "\t<DynamicsEngine>\n");
	fprintf(fp, "\t\t<%sEngine>\n", gEngineName[gEngineType]);

   // Gravity
	write_xml_gravity(fp, ms);

   // Bodies (with joints, wrap objects), constraints
   if (make_simbody_model(fp, ms, geometryDirectory) == code_bad)
      return code_bad;

   // Markers
	write_xml_markers(fp, ms);

	fprintf(fp, "\t\t</%sEngine>\n", gEngineName[gEngineType]);
	fprintf(fp, "\t</DynamicsEngine>\n");

   return code_fine;
}

ReturnCode write_xml_model(ModelStruct* ms, char filename[], char geometryDirectory[])
{
	FILE* fp;
   ReturnCode rc = code_fine;

	fp = fopen(filename, "w");
	fprintf(fp, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
	fprintf(fp, "<OpenSimDocument Version=\"10600\">\n");
	fprintf(fp, "<Model name=\"%s\">\n", ms->name);
	write_xml_defaults(fp, ms);
	if (gAngleUnits == DEGREES)
		fprintf(fp, "\t<angle_units> degrees </angle_units>\n");
	else
		fprintf(fp, "\t<angle_units> radians </angle_units>\n");
	write_xml_units(fp, ms);
	write_xml_muscles(fp, ms);
   if (gEngineType == SIMM_ENGINE)
	   rc = write_xml_ke(fp, ms, geometryDirectory);
   else if (gEngineType == SIMBODY_ENGINE)
      rc = write_opensim20_engine(fp, ms, geometryDirectory);
	fprintf(fp, "</Model>\n");
	fprintf(fp, "</OpenSimDocument>\n");
	fclose(fp);

   if (rc == code_bad)
      remove(filename);

   return rc;
}


void simm_exit(void)
{

#if INCLUDE_EVA_REALTIME
   stop_realtime_mocap_stream();
#endif

/*   gexit(); TODO: is there an OpenGL equivalent? */
   exit(0);

}

static SBoolean avoid_clipping_planes = no;

SBoolean avoid_gl_clipping_planes()
{
   return avoid_clipping_planes;
}


void start_simm()
{

}

SBoolean is_in_demo_mode ()
{
   return no;
}


#define UNDEFINED_DOUBLE 99999.99

SBoolean isVisible(double pt[])
{
   if (pt[0] >= UNDEFINED_DOUBLE)
      return no;
   if (pt[1] >= UNDEFINED_DOUBLE)
      return no;
   if (pt[2] >= UNDEFINED_DOUBLE)
      return no;
   return yes;
}

