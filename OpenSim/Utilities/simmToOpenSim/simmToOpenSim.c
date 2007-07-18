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


static char EngineType[256];
static char EngineName[256];
ModelStruct* sMotionModel = NULL;
char* markerSetOut = NULL;


void write_xml_model(ModelStruct* ms, char filename[], char geometryDirectory[], int angleUnits);
int makeDir(const char aDirName[]);

static void printUsage(char programName[])
{
	printf("Usage: %s -j joints_in [ -m muscles_in] [-e simm | simbody] -x xml_out [-ms markerset_out] [-g geometry_directory] [-a degrees | radians]\n", programName);
}

int main(int argc, char* argv[])
{
   ModelStruct* ms;
	char *jointIn = NULL, *muscleIn = NULL, *xmlOut = NULL, *geometryDirectory = NULL;
	int angleUnits = RADIANS;
	strcpy(EngineType,"Simm");
	strcpy(EngineName,"SimmKinematicsEngine");

	printf("simmToOpenSim, version 0.8.1 (April 20, 2007)\n");

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
			if (STRINGS_ARE_EQUAL(argv[i], "-j") || STRINGS_ARE_EQUAL(argv[i], "-J"))
			{
				jointIn = argv[i+1];
				i++; 
			}
			else if (STRINGS_ARE_EQUAL(argv[i], "-m") || STRINGS_ARE_EQUAL(argv[i], "-M"))
			{
				muscleIn = argv[i+1];
				i++; 
			}
			else if (STRINGS_ARE_EQUAL(argv[i], "-e") || STRINGS_ARE_EQUAL(argv[i], "-E"))
			{
				if (STRINGS_ARE_EQUAL(argv[i+1], "simm") || STRINGS_ARE_EQUAL(argv[i+1], "SIMM") || STRINGS_ARE_EQUAL(argv[i+1], "Simm"))
				{
					printf("\n\nGenerating an OpenSim model with the SIMM kinematics engine as the underlying dynamics engine.\n");
					printf("Note that this model will not support dynamic simulations or analyses.\n");
					printf("However, it is possible to convert this model to use SDFast as the underlying engine\n");
					printf("once the OpenSim modelhas been generated using the utility makeSDFastModel.exe.\n\n");
					strcpy(EngineType,"Simm");
					strcpy(EngineName,"SimmKinematicsEngine");
				}
				else if (STRINGS_ARE_EQUAL(argv[i+1], "simbody") || STRINGS_ARE_EQUAL(argv[i+1], "Simbody") || STRINGS_ARE_EQUAL(argv[i+1], "SimBody") || STRINGS_ARE_EQUAL(argv[i+1], "SIMBODY"))
				{
					printf("\n\nGenerating an OpenSim model with Simbody as the underlying dynamics engine.\n\n");
					strcpy(EngineType,"Simbody");
					strcpy(EngineName,"SimbodyEngine");
				}
				else
				{
					printf("\nDynamics engine type unrecognized \"-e\" must be \"simm\" or \"simbody\".\n");
					printf("\nProceeding assuming conversion to a model based on a SIMM kinematics engine is desired.\n");
				}
				i++; 
			}
			else if (STRINGS_ARE_EQUAL(argv[i], "-x") || STRINGS_ARE_EQUAL(argv[i], "-X"))
			{
				xmlOut = argv[i+1];
				i++; 
			}
			else if (STRINGS_ARE_EQUAL(argv[i], "-ms") || STRINGS_ARE_EQUAL(argv[i], "-MS"))
			{
				markerSetOut = argv[i+1];
				i++; 
			}
			else if (STRINGS_ARE_EQUAL(argv[i], "-g") || STRINGS_ARE_EQUAL(argv[i], "-G"))
			{
				geometryDirectory = argv[i+1];
				i++; 
			}
			else if (STRINGS_ARE_EQUAL(argv[i], "-a") || STRINGS_ARE_EQUAL(argv[i], "-A"))
			{
				if (STRINGS_ARE_EQUAL(argv[i+1], "degrees") || STRINGS_ARE_EQUAL(argv[i+1], "DEGREES"))
					angleUnits = DEGREES;
				else if (STRINGS_ARE_EQUAL(argv[i+1], "radians") || STRINGS_ARE_EQUAL(argv[i+1], "RADIANS"))
					angleUnits = RADIANS;
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

   read_model_file(ms->modelnum, ms->jointfilename);

   /* If the muscle file name was specified on the command line,
	 * override the one specified in the joint file (if any).
    */
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
		read_muscle_file(ms, ms->musclefilename, &foo);
		// read_muscle_file already prints this message...
		//printf("Read muscle file %s\n", ms->musclefilename);
	}

   write_xml_model(ms, xmlOut, geometryDirectory, angleUnits);

	printf("Wrote OpenSim model file %s\n", xmlOut);
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

void write_xml_muscle(FILE* fp, ModelStruct* ms, MuscleStruct* m, const char* muscleClassName, int angleUnits, int writingDefault)
{
	double conversion;
	int j;

	if (angleUnits == RADIANS)
		conversion = DEG_TO_RAD;
	else
		conversion = 1.0;

	fprintf(fp, "\t\t<%s name=\"%s\">\n", muscleClassName, m->name);

	/* Attachment points. */
	if(m->num_orig_points) {
		fprintf(fp, "\t\t\t<MusclePointSet>\n");
		fprintf(fp, "\t\t\t<objects>\n");
		for (j = 0; j < *m->num_orig_points; j++)
		{
			MusclePoint* mp = &m->mp_orig[j];
			if (mp->numranges == 0)
			{
				fprintf(fp, "\t\t\t\t<MusclePoint>\n");
				fprintf(fp, "\t\t\t\t\t<location>%.12lf %.12lf %.12lf</location>\n", mp->point[0], mp->point[1], mp->point[2]);
				fprintf(fp, "\t\t\t\t\t<body>%s</body>\n", ms->segment[mp->segment].name);
				fprintf(fp, "\t\t\t\t</MusclePoint>\n");
			}
			else if (mp->numranges == 1)
			{
				// If the gencoord used for this via point range is [primarily] translational, then do
				// not convert the range values. If it's rotational, then use the passed-in
				// conversion factor, which is either 1.0 or DEG_TO_RAD.
				double conv;
				if (ms->gencoord[mp->ranges[0].genc].type == translation_gencoord)
					conv = 1.0;
				else
					conv = conversion;

				fprintf(fp, "\t\t\t\t<MuscleViaPoint>\n");
				fprintf(fp, "\t\t\t\t\t<location>%.12lf %.12lf %.12lf</location>\n", mp->point[0], mp->point[1], mp->point[2]);
				fprintf(fp, "\t\t\t\t\t<body>%s</body>\n", ms->segment[mp->segment].name);
				fprintf(fp, "\t\t\t\t\t<coordinate>%s</coordinate>\n", ms->gencoord[mp->ranges[0].genc].name);
				fprintf(fp, "\t\t\t\t\t<range>%.12lf %.12lf</range>\n", mp->ranges[0].start * conv, mp->ranges[0].end * conv);
				fprintf(fp, "\t\t\t\t</MuscleViaPoint>\n");
			}
			else
			{
				printf("Warning: muscle %s has attachments with more than one range.\n", m->name);
				printf("         This is not currently supported in simmToXML.\n");
			}
		}
		fprintf(fp, "\t\t\t</objects>\n");
		fprintf(fp, "\t\t\t</MusclePointSet>\n");
	}

	/* Simple (double) parameters. */
	if (NEED_TO_WRITE_MUSCLE_VALUE(max_isometric_force))
		fprintf(fp, "\t\t\t<max_isometric_force>%.12lf</max_isometric_force>\n", *m->max_isometric_force);
	if (NEED_TO_WRITE_MUSCLE_VALUE(optimal_fiber_length))
		fprintf(fp, "\t\t\t<optimal_fiber_length>%.12lf</optimal_fiber_length>\n", *m->optimal_fiber_length);
	if (NEED_TO_WRITE_MUSCLE_VALUE(resting_tendon_length))
		fprintf(fp, "\t\t\t<tendon_slack_length>%.12lf</tendon_slack_length>\n", *m->resting_tendon_length);
	if (NEED_TO_WRITE_MUSCLE_VALUE(pennation_angle))
		fprintf(fp, "\t\t\t<pennation_angle>%.12lf</pennation_angle>\n", *m->pennation_angle * conversion);
	if (NEED_TO_WRITE_MUSCLE_VALUE(max_contraction_vel))
		fprintf(fp, "\t\t\t<max_contraction_velocity>%.12lf</max_contraction_velocity>\n", *m->max_contraction_vel);

	/* muscle model. */
	if (m->muscle_model_index)	// Conservative fix in case muscle model is not specified. Ayman 1/07
		fprintf(fp, "\t\t\t<muscle_model>%d</muscle_model>\n", *m->muscle_model_index);

	/* Dynamic parameters. */
	for (j = 0; j < m->num_dynamic_params; j++)
	{
		// map dynamic parameter "timescale" to "time_scale" property name
		if (NEED_TO_WRITE_MUSCLE_VALUE(dynamic_params[j]))
		{
			if(!strcmp(m->dynamic_param_names[j],"timescale"))
				fprintf(fp, "\t\t\t<time_scale>%.12lf</time_scale>\n", *m->dynamic_params[j]);
			else
				fprintf(fp, "\t\t\t<%s>%.12lf</%s>\n", m->dynamic_param_names[j], *m->dynamic_params[j], m->dynamic_param_names[j]);
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

void write_xml_muscles(FILE* fp, ModelStruct* ms, int angleUnits)
{
	int i;
	char muscleClassName[64];

	fprintf(fp, "\t<ActuatorSet>\n");
	write_xml_muscle_groups(fp, ms);
	fprintf(fp, "\t<objects>\n");
	for (i = 0; i < ms->nummuscles; i++)
	{
		MuscleStruct* m = &ms->muscle[i];

		// For now, model = 9 and model = NULL map to SimmDarrylMuscle.
		// All other values map to SimmZajacHill. Eventually, each model
		// index should map to a distinct muscle class in OpenSim,
		// especially model = 7, which is a ligament model.
		if (m->muscle_model_index == NULL || *m->muscle_model_index == 9)
			strcpy(muscleClassName, "SimmDarrylMuscle");
		else
			strcpy(muscleClassName, "SimmZajacHill");

		write_xml_muscle(fp, ms, m, muscleClassName, angleUnits, 0);
	}
	fprintf(fp, "\t</objects>\n");
	fprintf(fp, "\t</ActuatorSet>\n");
}


void write_xml_markers(FILE* fp, ModelStruct* ms)
{
	int i, j;
	char tabs[5];

	if(markerSetOut) {
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
			fprintf(fp, "%s\t\t<Marker name=\"%s\">\n", tabs, ss->marker[j].name);
			fprintf(fp, "%s\t\t\t<body>%s</body>\n", tabs, ss->name);
			fprintf(fp, "%s\t\t\t<location>%.12lf %.12lf %.12lf</location>\n", tabs, ss->marker[j].offset[0], ss->marker[j].offset[1], ss->marker[j].offset[2]);
			fprintf(fp, "%s\t\t\t<weight>%.12lf</weight>\n", tabs, ss->marker[j].weight);
			fprintf(fp, "%s\t\t\t<fixed>%s</fixed>\n", tabs, (ss->marker[j].fixed == yes) ? ("true") : ("false"));
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

int segment_has_wrap_objects(ModelStruct* ms, int segment_num)
{
	int i;

	for (i = 0; i < ms->num_wrap_objects; i++)
		if (ms->wrapobj[i].segment == segment_num)
			return 1;

	return 0;
}

void write_xml_wrap_object(FILE* fp, WrapObject* wo, int angleUnits)
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
	   case wrap_sphere:
		   fprintf(fp, "\t\t\t\t\t\t<WrapSphere name=\"%s\">\n", wo->name);
			fprintf(fp, "\t\t\t\t\t\t\t<radius> %.8lf </radius>\n", wo->radius.xyz[0]);
			break;
		case wrap_cylinder:
			fprintf(fp, "\t\t\t\t\t\t<WrapCylinder name=\"%s\">\n", wo->name);
			fprintf(fp, "\t\t\t\t\t\t\t<radius> %.8lf </radius>\n", wo->radius.xyz[0]);
			fprintf(fp, "\t\t\t\t\t\t\t<length> %.8lf </length>\n", wo->radius.xyz[1]);
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

void write_xml_bodies(FILE* fp, ModelStruct* ms, char geometryDirectory[], int angleUnits)
{
	int i, j;
	SBoolean madeDir = no;

	fprintf(fp, "\t\t\t<BodySet>\n");
	fprintf(fp, "\t\t\t<objects>\n");
	for (i = 0; i < ms->numsegments; i++)
	{
		SegmentStruct* ss = &ms->segment[i];
		fprintf(fp, "\t\t\t\t<%sBody name=\"%s\">\n",EngineType,ss->name);
		fprintf(fp, "\t\t\t\t\t<mass>%.12lf</mass>\n", ss->mass);
		fprintf(fp, "\t\t\t\t\t<mass_center>%.12lf %.12lf %.12lf</mass_center>\n", ss->masscenter[0], ss->masscenter[1], ss->masscenter[2]);
		fprintf(fp, "\t\t\t\t\t<inertia>%.12lf %.12lf %.12lf %.12lf %.12lf %.12lf %.12lf %.12lf %.12lf</inertia>\n", ss->inertia[0][0], ss->inertia[0][1], ss->inertia[0][2],
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
			fprintf(fp, "\t\t\t\t\t</VisibleObject>\n");
		}
		if (segment_has_wrap_objects(ms, i))
		{
			fprintf(fp, "\t\t\t\t\t<WrapObjectSet>\n");
			fprintf(fp, "\t\t\t\t\t<objects>\n");
			for (j = 0; j < ms->num_wrap_objects; j++)
			{
				if (ms->wrapobj[j].segment == i)
					write_xml_wrap_object(fp, &ms->wrapobj[j], angleUnits);
			}
			fprintf(fp, "\t\t\t\t\t</objects>\n");
			fprintf(fp, "\t\t\t\t\t</WrapObjectSet>\n");
		}
		fprintf(fp, "\t\t\t\t</%sBody>\n",EngineType);
	}
	fprintf(fp, "\t\t\t</objects>\n");
	fprintf(fp, "\t\t\t</BodySet>\n");
}


void write_xml_coordinates(FILE* fp, ModelStruct* ms, int angleUnits)
{
	int i, j;
	SplineFunction* sf;
	double conversion;

	if (angleUnits == RADIANS)
		conversion = DEG_TO_RAD;
	else
		conversion = 1.0;

	fprintf(fp, "\t\t\t<CoordinateSet>\n");
	fprintf(fp, "\t\t\t<objects>\n");
	for (i = 0; i < ms->numgencoords; i++)
	{
		// If the gencoord is [primarily] translational, then do not convert its
		// range, default_value, etc. If it's rotational, then use the passed-in
		// conversion factor, which is either 1.0 or DEG_TO_RAD.
		double conv;
		if (ms->gencoord[i].type == translation_gencoord)
			conv = 1.0;
		else
			conv = conversion;

		fprintf(fp, "\t\t\t\t<%sCoordinate name=\"%s\">\n", EngineType, ms->gencoord[i].name);
		fprintf(fp, "\t\t\t\t\t<range>%.12lf %.12lf</range>\n", ms->gencoord[i].range.start * conv, ms->gencoord[i].range.end * conv);
		fprintf(fp, "\t\t\t\t\t<default_value>%.12lf</default_value>\n", ms->gencoord[i].default_value * conv);
		fprintf(fp, "\t\t\t\t\t<tolerance>%.12lf</tolerance>\n", ms->gencoord[i].tolerance);
		fprintf(fp, "\t\t\t\t\t<stiffness>%.12lf</stiffness>\n", ms->gencoord[i].pd_stiffness);
      if (ms->gencoord[i].keys[0] != null_key)
      {
         fprintf(fp, "\t\t\t\t\t<keys>%s", get_simmkey_name((int)ms->gencoord[i].keys[0]));
         if (ms->gencoord[i].keys[1] != ms->gencoord[i].keys[0])
            fprintf(fp, " %s", get_simmkey_name((int)ms->gencoord[i].keys[1]));
         fprintf(fp,"</keys>\n");
      }
		fprintf(fp, "\t\t\t\t\t<clamped>%s</clamped>\n", (ms->gencoord[i].clamped == yes) ? ("true") : ("false"));
		fprintf(fp, "\t\t\t\t\t<locked>%s</locked>\n", (ms->gencoord[i].locked == yes) ? ("true") : ("false"));
		fprintf(fp, "\t\t\t\t\t<restraint_active>%s</restraint_active>\n", (ms->gencoord[i].restraintFuncActive == yes) ? ("true") : ("false"));
		if (ms->gencoord[i].restraint_func_num != -1)
		{
			sf = &ms->function[ms->gencoord[i].restraint_func_num];

			fprintf(fp, "\t\t\t<restraint_function>\n");
			fprintf(fp, "\t\t\t\t\t\t<natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t\t\t<x>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->x[j] * conv);
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
			sf = &ms->function[ms->gencoord[i].min_restraint_func_num];

			fprintf(fp, "\t\t\t\t\t<min_restraint_function>\n");
			fprintf(fp, "\t\t\t\t\t\t<natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t\t\t<x>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->x[j] * conv);
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
			sf = &ms->function[ms->gencoord[i].max_restraint_func_num];

			fprintf(fp, "\t\t\t\t\t<max_restraint_function>\n");
			fprintf(fp, "\t\t\t\t\t\t<natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t\t\t<x>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->x[j] * conv);
			fprintf(fp, "</x>\n");
			fprintf(fp, "\t\t\t\t\t\t\t<y>");
			for (j = 0; j < sf->numpoints; j++)
				fprintf(fp, "%.12lf ", sf->y[j]);
			fprintf(fp, "</y>\n");
			fprintf(fp, "\t\t\t\t\t\t</natCubicSpline>\n");
			fprintf(fp, "\t\t\t\t\t</max_restraint_function>\n");
		}
		fprintf(fp, "\t\t\t\t</%sCoordinate>\n",EngineType);
	}
	fprintf(fp, "\t\t\t</objects>\n");
	fprintf(fp, "\t\t\t</CoordinateSet>\n");
}


void write_xml_speeds(FILE* fp, ModelStruct* ms, int angleUnits)
{
	int i, j;
	SplineFunction* sf;
	double conversion;

	if (angleUnits == RADIANS)
		conversion = DEG_TO_RAD;
	else
		conversion = 1.0;

	fprintf(fp, "\t\t\t<SpeedSet>\n");
	fprintf(fp, "\t\t\t<objects>\n");
	for (i = 0; i < ms->numgencoords; i++)
	{
		// If the gencoord is [primarily] translational, then do not convert its
		// range, default_value, etc. If it's rotational, then use the passed-in
		// conversion factor, which is either 1.0 or DEG_TO_RAD.
		double conv;
		if (ms->gencoord[i].type == translation_gencoord)
			conv = 1.0;
		else
			conv = conversion;

		fprintf(fp, "\t\t\t\t<%sSpeed name=\"%s_u\">\n", EngineType,ms->gencoord[i].name);
		fprintf(fp, "\t\t\t\t\t<default_value> 0.0 </default_value>\n");
		fprintf(fp, "\t\t\t\t\t<coordinate> %s </coordinate>\n",ms->gencoord[i].name);
		fprintf(fp, "\t\t\t\t</%sCoordinate>\n",EngineType);
	}
	fprintf(fp, "\t\t\t</objects>\n");
	fprintf(fp, "\t\t\t</SpeedSet>\n");
}


void write_xml_joints(FILE* fp, ModelStruct* ms, int angleUnits)
{
	int i, j, k, m, rotCount;
	double conversion;

	if (angleUnits == RADIANS)
		conversion = DEG_TO_RAD;
	else
		conversion = 1.0;

	fprintf(fp, "\t\t\t<JointSet>\n");
	fprintf(fp, "\t\t\t<objects>\n");
	for (i = 0; i < ms->numjoints; i++)
	{
		JointStruct* js = &ms->joint[i];
		fprintf(fp, "\t\t\t\t<%sJoint name=\"%s\">\n",EngineType,ms->joint[i].name);
		fprintf(fp, "\t\t\t\t\t<bodies>%s %s</bodies>\n", ms->segment[js->from].name, ms->segment[js->to].name);
		fprintf(fp, "\t\t\t\t\t<DofSet>\n");
		fprintf(fp, "\t\t\t\t\t<objects>\n");
		for (j = 0, rotCount = 0; j < 4; j++)
		{
			if (j == js->order[TRANS]) // TX, TY, and TZ
			{
				char* translationNames[] = {"tx", "ty", "tz"};

				for (k = 3; k < 6; k++)
				{
					fprintf(fp, "\t\t\t\t\t\t<%sTranslationDof name=\"%s\">\n",EngineType,translationNames[k-3]);
					if (js->dofs[k].type == constant_dof)
					{
						fprintf(fp, "\t\t\t\t\t\t\t<Value>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t<Constant>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t\t<value>%.12lf</value>\n", js->dofs[k].value);
						fprintf(fp, "\t\t\t\t\t\t\t\t</Constant>\n");
						fprintf(fp, "\t\t\t\t\t\t\t</Value>\n");
					}
					else
					{
						SplineFunction* sf = &ms->function[js->dofs[k].funcnum];
						// This dof is a function of a gencoord. If the gencoord is [primarily] translational,
						// then do not convert its X points. If it's rotational, then use the passed-in
						// conversion factor, which is either 1.0 or DEG_TO_RAD. The Y points are never
						// converted with the passed-in factor because the dof itself is translational.
						double conv;
						if (ms->gencoord[js->dofs[k].gencoord].type == translation_gencoord)
							conv = 1.0;
						else
							conv = conversion;

						fprintf(fp, "\t\t\t\t\t\t\t<coordinate>%s</coordinate>\n", ms->gencoord[js->dofs[k].gencoord].name);
						fprintf(fp, "\t\t\t\t\t\t\t<Value>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t<natCubicSpline>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t\t<x>");
						for (m = 0; m < sf->numpoints; m++)
							fprintf(fp, "%.12lf ", sf->x[m] * conv);
						fprintf(fp, "</x>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t\t<y>");
						for (m = 0; m < sf->numpoints; m++)
							fprintf(fp, "%.12lf ", sf->y[m]);
						fprintf(fp, "</y>\n");
						fprintf(fp, "\t\t\t\t\t\t\t\t</natCubicSpline>\n");
						fprintf(fp, "\t\t\t\t\t\t\t</Value>\n");
					}
					fprintf(fp, "\t\t\t\t\t\t</%sTranslationDof>\n",EngineType);
				}
			}
			else // R1, R2, R3
			{
				int axis;

				if (j == js->order[ROT1])
					axis = 0;
				else if (j == js->order[ROT2])
					axis = 1;
				else // ROT3
					axis = 2;

				/* Rename the rotation dofs so that they're always in the order: r1, r2, r3 in the XML file.
				 * These names aren't needed by simTK, but they could come in handy for debugging.
				 */
				fprintf(fp, "\t\t\t\t\t\t<%sRotationDof name=\"%s\">\n",EngineType,getjointvarname(rotCount++));
				fprintf(fp, "\t\t\t\t\t\t\t<axis>%.12lf %.12lf %.12lf</axis>\n", js->parentrotaxes[axis][0], js->parentrotaxes[axis][1], js->parentrotaxes[axis][2]);
				if (js->dofs[axis].type == constant_dof)
				{
					fprintf(fp, "\t\t\t\t\t\t\t<Value>\n");
					fprintf(fp, "\t\t\t\t\t\t\t\t<Constant>\n");
					fprintf(fp, "\t\t\t\t\t\t\t\t\t<value>%.12lf</value>\n", js->dofs[axis].value * conversion);
					fprintf(fp, "\t\t\t\t\t\t\t\t</Constant>\n");
					fprintf(fp, "\t\t\t\t\t\t\t</Value>\n");
				}
				else
				{
					SplineFunction* sf = &ms->function[js->dofs[axis].funcnum];

					// This dof is a function of a gencoord. If the gencoord is [primarily] translational,
					// then do not convert its X points. If it's rotational, then use the passed-in
					// conversion factor, which is either 1.0 or DEG_TO_RAD. The Y points are always
					// converted with the passed-in factor because the dof itself is rotational.
					double conv;
					if (ms->gencoord[js->dofs[axis].gencoord].type == translation_gencoord)
						conv = 1.0;
					else
						conv = conversion;

					fprintf(fp, "\t\t\t\t\t\t\t<coordinate>%s</coordinate>\n", ms->gencoord[js->dofs[axis].gencoord].name);
					fprintf(fp, "\t\t\t\t\t\t\t<Value>\n");
					fprintf(fp, "\t\t\t\t\t\t\t\t<natCubicSpline>\n");
					fprintf(fp, "\t\t\t\t\t\t\t\t\t<x>");
					for (m = 0; m < sf->numpoints; m++)
						fprintf(fp, "%.12lf ", sf->x[m] * conv);
					fprintf(fp, "</x>\n");
					fprintf(fp, "\t\t\t\t\t\t\t\t\t<y>");
					for (m = 0; m < sf->numpoints; m++)
						fprintf(fp, "%.12lf ", sf->y[m] * conversion);
					fprintf(fp, "</y>\n");
					fprintf(fp, "\t\t\t\t\t\t\t\t</natCubicSpline>\n");
					fprintf(fp, "\t\t\t\t\t\t\t</Value>\n");
				}
				fprintf(fp, "\t\t\t\t\t\t</%sRotationDof>\n",EngineType);
			}
		}
		fprintf(fp, "\t\t\t\t\t</objects>\n");
		fprintf(fp, "\t\t\t\t\t</DofSet>\n");
		fprintf(fp, "\t\t\t\t</%sJoint>\n",EngineType);
	}
	fprintf(fp, "\t\t\t</objects>\n");
	fprintf(fp, "\t\t\t</JointSet>\n");
}


void write_xml_ke(FILE* fp, ModelStruct* ms, char geometryDirectory[], int angleUnits)
{
	fprintf(fp, "\t<DynamicsEngine>\n");
	fprintf(fp, "\t\t<%s>\n",EngineName);
	write_xml_gravity(fp, ms);
	write_xml_markers(fp, ms);
	write_xml_bodies(fp, ms, geometryDirectory, angleUnits);
	write_xml_coordinates(fp, ms, angleUnits);
	if(STRINGS_ARE_EQUAL(EngineType,"Simbody")) write_xml_speeds(fp, ms, angleUnits);
	write_xml_joints(fp, ms, angleUnits);
	fprintf(fp, "\t\t</SimmKinematicsEngine>\n");
	fprintf(fp, "\t</%s>\n",EngineName);
}

void write_xml_units(FILE* fp, ModelStruct* ms)
{
   if (ms->forceUnits != NULL)
      fprintf(fp, "\t<force_units>%s</force_units>\n", ms->forceUnits);

   if (ms->lengthUnits != NULL)
      fprintf(fp, "\t<length_units>%s</length_units>\n", ms->lengthUnits);
}

void write_xml_defaults(FILE* fp, ModelStruct* ms, int angleUnits)
{
	fprintf(fp, "\t<defaults>\n");
	// We need to write a default muscle for all supported types of muscles (since
	// the default mechanism works by matching class names)
	write_xml_muscle(fp, ms, &ms->default_muscle, "SimmZajacHill", angleUnits, 1);
	write_xml_muscle(fp, ms, &ms->default_muscle, "SimmDarrylMuscle", angleUnits, 1);
	fprintf(fp, "\t</defaults>\n");
}

void write_xml_model(ModelStruct* ms, char filename[], char geometryDirectory[], int angleUnits)
{
	FILE* fp;

	fp = fopen(filename, "w");
	fprintf(fp, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
	fprintf(fp, "<Model name=\"%s\">\n", ms->name);
	write_xml_defaults(fp, ms, angleUnits);
	if (angleUnits == DEGREES)
		fprintf(fp, "\t<angle_units> degrees </angle_units>\n");
	else
		fprintf(fp, "\t<angle_units> radians </angle_units>\n");
	write_xml_units(fp, ms);
	write_xml_muscles(fp, ms, angleUnits);
	write_xml_ke(fp, ms, geometryDirectory, angleUnits);
	fprintf(fp, "</Model>\n");
	fclose(fp);
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

