/*******************************************************************************

   WORLDOBJECTS.C

   Author: Peter Loan

   Date: 21-MAR-91

   Copyright (c) 1992-4 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: 

   Routines:
      readworldobjects        : reads a world object from input file
      compute_polygon_normals : computes normals for world object polygons

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "normio.h"
#include "normtools.h"

#if OPENSMAC
#undef ENGINE
#define ENGINE 1
#endif

/*************** DEFINES (for this file only) *********************************/
#define ARRAY_INCREMENT 5


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void init_world_object(ModelStruct* ms, WorldObject* obj);


#if ! ENGINE
PickIndex pack_world_value(int world_obj)
{
   if (world_obj >= MAX_OBJECTS)
      return 0;
   else
      return (WORLD_OBJECT << OBJECT_BITS) + (PickIndex)world_obj;
}


void unpack_world_value(PickIndex value, int* world_obj)
{
   *world_obj = value;
}
#endif

ReturnCode read_world_object(ModelStruct* ms, FILE* fp)
{
   int i, j;
   ReturnCode rc = code_fine;
   WorldObject* obj;
   char fname[CHARBUFFER], name[CHARBUFFER], mess[CHARBUFFER];
   double scale[3], translation[3], rotation[3];

   if (ms->numworldobjects >= ms->world_array_size)
   {
      ms->world_array_size += ARRAY_INCREMENT;
      ms->worldobj = (WorldObject*)simm_realloc(ms->worldobj,
                 ms->world_array_size*sizeof(WorldObject),&rc);
      if (rc == code_bad)
      {
         ms->world_array_size -= ARRAY_INCREMENT;
         return code_bad;
      }
   }

   obj = &ms->worldobj[ms->numworldobjects];

   init_world_object(ms, obj);
   scale[0] = scale[1] = scale[2] = 1.0;
   translation[0] = translation[1] = translation[2] = 0.0;
   rotation[0] = rotation[1] = rotation[2] = 0.0;

   if (fscanf(fp, "%s", name) != 1)
   {
      error(abort_action, "Error reading name in object definition");
      return code_bad;
   }
   else
      mstrcpy(&obj->name,name);

    while (1)
    {
        if (read_string(fp,buffer) == EOF)
        {
            (void)sprintf(errorbuffer, "Unexpected EOF found reading definition of object %s", obj->name);
            error(abort_action,errorbuffer);
            return code_bad;
        }
        else if (STRINGS_ARE_EQUAL(buffer, "endworldobject"))
        {
            break;
        }
        else if (STRINGS_ARE_EQUAL(buffer, "filename"))
        {
         FileReturnCode frc;
            if (fscanf(fp, "%s", fname) != 1)
            {
                (void)sprintf(errorbuffer, "Error reading filename in definition of worldobject %s", obj->name);
                error(abort_action, errorbuffer);
                return code_bad;        
            }
#if ! ENGINE
            frc = lookup_polyhedron(obj->wobj, fname, ms);

            if (frc == file_missing)
         {
                (void)sprintf(mess, "Unable to locate object file %s", fname);
                error(none, mess);
         }
         else if (frc == file_bad)
            {
                (void)sprintf(mess, "Unable to read object from file %s", fname);
                error(none, mess);
            }
            else
            {
                /* simm_printf(no, "Read object file %s\n", fname); */
            //TODO5.0 shouldn't the filename be copied anyway, even if the file was not found?
            }
            mstrcpy(&obj->filename, fname);
#else
            mstrcpy(&obj->filename, fname);
#endif
        }
        else if (STRINGS_ARE_EQUAL(buffer, "origin"))
        {
            if (fscanf(fp, "%lg %lg %lg", &translation[0], &translation[1], &translation[2]) != 3)
            {
                (void)sprintf(errorbuffer, "Error reading origin in definition of worldobject %s", obj->name);
                error(none, errorbuffer);
                error(none, "Using default value of 0.0 0.0 0.0");
            translation[0] = translation[1] = translation[2] = 0.0;
            }
        }
        else if (STRINGS_ARE_EQUAL(buffer, "scale"))
        {
            if (fscanf(fp, "%lg %lg %lg", &scale[0], &scale[1], &scale[2]) != 3)
            {
                (void)sprintf(errorbuffer, "Error reading scale factor in definition of worldobject %s", obj->name);
                error(none, errorbuffer);
                error(none, "Using default values of 1.0 1.0 1.0");
            scale[0] = scale[1] = scale[2] = 1.0;
            }
        }
        else if (STRINGS_ARE_EQUAL(buffer, "rotation"))
        {
            if (fscanf(fp, "%lg %lg %lg", &rotation[0], &rotation[1], &rotation[2]) != 3)
            {
                (void)sprintf(errorbuffer, "Error reading XYZ Euler rotations in definition of worldobject %s", obj->name);
                error(none, errorbuffer);
                error(none, "Using default values of 0.0 0.0 0.0");
            rotation[0] = rotation[1] = rotation[2] = 0.0;
            }
        }
        else if (STRINGS_ARE_EQUAL(buffer, "drawmode"))
        {
            if (fscanf(fp, "%s", buffer) != 1)
            {
                (void)sprintf(errorbuffer, "Error reading drawmode in definition of worldobject %s", obj->name);
                error(none, errorbuffer);
                error(none, "Using default value of solid_fill");
            }
            if (STRINGS_ARE_EQUAL(buffer, "wireframe"))
                obj->drawmode = wireframe;
            else if (STRINGS_ARE_EQUAL(buffer, "solid_fill"))
                obj->drawmode = solid_fill;
            else if (STRINGS_ARE_EQUAL(buffer, "flat_shading"))
                obj->drawmode = flat_shading;
            else if (STRINGS_ARE_EQUAL(buffer, "gouraud_shading"))
                obj->drawmode = gouraud_shading;
            else if (STRINGS_ARE_EQUAL(buffer, "outlined_polygons"))
                obj->drawmode = outlined_polygons;
            else if (STRINGS_ARE_EQUAL(buffer, "bounding_box"))
            {
                (void)sprintf(errorbuffer, "Warning: bounding_box drawmode not supported for worldobjects (%s).", obj->name);
                error(none, errorbuffer);
                error(none, "Using solid_fill instead.");
                obj->drawmode = solid_fill;
            }
            else if (STRINGS_ARE_EQUAL(buffer, "none"))
                obj->drawmode = no_surface;
            else
            {
                (void)sprintf(errorbuffer, "Unknown drawmode found for worldobject (%s).", obj->name);
                error(none, errorbuffer);
                error(none, "Using solid_fill instead.");
                obj->drawmode = solid_fill;
            }
        }
        else if (STRINGS_ARE_EQUAL(buffer, "material"))
        {
            if (fscanf(fp,"%s", fname) != 1)
            {
                (void)sprintf(errorbuffer, "Error reading material in definition of worldobject %s", obj->name);
                error(none, errorbuffer);
                error(none, "Using default value of def_bone");
                obj->material = enter_material(ms, "def_bone", just_checking_element);
            }
            else
            {
                obj->material = enter_material(ms,fname,declaring_element);
            }
        }
      else if (STRINGS_ARE_EQUAL(buffer, "texture_file"))
      {
         FileReturnCode frc;
            if (fscanf(fp, "%s", fname) != 1)
            {
                (void)sprintf(errorbuffer, "Error reading name of texture file in definition of worldobject %s", obj->name);
                error(abort_action, errorbuffer);
                return code_bad;        
            }

            mstrcpy(&obj->texture_filename, fname);
        }
      else if (STRINGS_ARE_EQUAL(buffer, "texture_coords_file"))
      {
         FileReturnCode frc;
            if (fscanf(fp, "%s", fname) != 1)
            {
                (void)sprintf(errorbuffer, "Error reading name of texture coordinates file in definition of worldobject %s", obj->name);
                error(abort_action, errorbuffer);
                return code_bad;        
            }

            mstrcpy(&obj->texture_coords_filename, fname);
        }
        else
        {
            (void)sprintf(errorbuffer, "Unknown string (%s) in definition of object %s", buffer, obj->name);
            error(recover, errorbuffer);
        }
    }

   identity_matrix(obj->transform);

   x_rotate_matrix_bodyfixed(obj->transform, rotation[0] * DTOR);
   y_rotate_matrix_bodyfixed(obj->transform, rotation[1] * DTOR);
   z_rotate_matrix_bodyfixed(obj->transform, rotation[2] * DTOR);

   for (i=0; i<3; i++)
      for (j=0; j<3; j++)
         obj->transform[i][j] *= scale[j];

   obj->transform[3][0] = translation[0];
   obj->transform[3][1] = translation[1];
   obj->transform[3][2] = translation[2];

#if ! ENGINE
   // If a texture coords file was specified, load it now.
   // This must be done after the world object's polyhedron has been read in, because
   // it stores the texture coordinates in the vertex structures.
   if (obj->texture_coords_filename)
   {
      FileReturnCode frc = lookup_texture_coord_file(obj->wobj, obj->texture_coords_filename, ms);

      if (frc == file_missing)
      {
         (void)sprintf(mess, "Unable to locate texture coordinate file %s", obj->texture_coords_filename);
         error(none, mess);
      }
      else if (frc == file_bad)
      {
         (void)sprintf(mess, "Unable to read texture coordinates from file %s", obj->texture_coords_filename);
         error(none, mess);
      }
      else
      {
         //TODO5.0 how to indicate an error???
      }
   }
#endif

    ms->numworldobjects++;

   return code_fine;
}


static void init_world_object(ModelStruct* ms, WorldObject* obj)
{
   obj->filename = NULL;
   obj->texture_filename = NULL;
   obj->texture_coords_filename = NULL;
   identity_matrix(obj->transform);
   obj->drawmode = solid_fill;
   obj->selected = no;
   obj->material = ms->dis.mat.default_world_object_material;
   obj->drawmodemenu = 0;

#if ENGINE
   obj->wobj = NULL;
#else
   obj->wobj = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
   preread_init_polyhedron(obj->wobj);
#endif
}

