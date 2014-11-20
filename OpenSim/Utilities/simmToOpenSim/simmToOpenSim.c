/*******************************************************************************

   SIMMTOOPENSIM.C

   Author: Peter Loan, Frank C. Anderson

   Date: November 10, 2006

   Description:
    Reads in SIMM joint and muscle files and writes an OpenSim model file.
    
   Routines:

*******************************************************************************/

#include "universal.h"
#include "main.h"
#if OPENSIM_BUILD
#include <OpenSim/version.h>
#endif
#ifdef WIN32
#pragma warning( disable : 4996 )
#endif

static void printUsage(char programName[]);
ReturnCode write_opensim_model(ModelStruct* ms, char filename[], char geometryDirectory[],
                               const char* markerSetOut, int angleUnits);


static void printUsage(char programName[])
{
    printf("Usage: %s -j joints_in [-m muscles_in] -x xml_out [-ms markerset_out] [-g geometry_directory] [-a degrees | radians]\n", programName);
}


int main(int argc, char* argv[])
{
   Scene* scene;
   ModelStruct* model;
    char *jointIn = NULL, *muscleIn = NULL, *xmlOut = NULL, *geometryDirectory = NULL;
   char* markerSetOut = NULL;
   int angleUnits = RADIANS;

#if OPENSIM_BUILD
    printf("simmToOpenSim, version %s, build date %s %s\n", OpenSimVersion, __TIME__, __DATE__);
#else
   printf("simmToOpenSim, version %s\n", program_version);
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
            else if (STRINGS_ARE_EQUAL_CI(argv[i], "-X"))
            {
                xmlOut = argv[i+1];
                i++; 
            }
            else if (STRINGS_ARE_EQUAL_CI(argv[i], "-MS"))
            {
                markerSetOut = argv[i+1];
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
                    angleUnits = DEGREES;
                else if (STRINGS_ARE_EQUAL_CI(argv[i+1], "RADIANS"))
                    angleUnits = RADIANS;
                else
                    printf("Parameter after \"-a\" must be one of: degrees, DEGREES, radians, RADIANS.\n");
                i++; 
            }
            else
            {
                printf("Unrecognized option (%s) on command line.", argv[i]);
                printUsage(argv[0]);
                //exit(0);
            }
        }
    }

    if (!jointIn || !xmlOut)
    {
        printUsage(argv[0]);
        exit(0);
    }

   scene = get_scene();
   init_scene(scene);

   model = get_modelstruct();
   init_model(model);

   scene->model = (ModelStruct**)simm_malloc(sizeof(ModelStruct*));
   scene->model[0] = model;
   scene->num_models = 1;

   mstrcpy(&model->jointfilename, jointIn);

   read_model_file(scene, model, model->jointfilename, yes);

   // If the muscle file name was specified on the command line,
    // override the one specified in the joint file (if any).
   if (muscleIn)
      mstrcpy(&model->musclefilename, muscleIn);

   if (check_definitions(model) == code_bad)
      exit(0);

   find_ground_joint(model); /* must be called before makepaths() ! */

   if (makepaths(model) == code_bad)
      exit(0);

    init_gencoords(model);

    set_gencoord_info(model);

   printf("Read joint file %s\n", model->jointfilename);

    if (model->musclefilename)
    {
        SBoolean foo;
        read_muscle_file(model, model->musclefilename, &foo, yes);
    }

   if (write_opensim_model(model, xmlOut, geometryDirectory, markerSetOut, angleUnits) == code_fine)
       printf("Wrote OpenSim model file %s\n", xmlOut);
   else
      printf("Creation of OpenSim model from %s failed.\n", model->jointfilename);

    exit(0);
}
