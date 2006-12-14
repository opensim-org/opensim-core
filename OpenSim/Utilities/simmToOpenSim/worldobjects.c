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


/*************** DEFINES (for this file only) *********************************/
#define ARRAY_INCREMENT 5


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void init_world_object(ModelStruct* ms, WorldObject* obj);


#ifndef ENGINE
unsigned int pack_world_value(int world_obj)
{

   if (world_obj >= MAX_OBJECTS)
      return 0;
   else
      return (WORLD_OBJECT << OBJECT_BITS) + world_obj;

}


void unpack_world_value(unsigned int value, int* world_obj)
{

   *world_obj = value;

}
#endif

ReturnCode read_world_object(int mod, FILE** fp)
{

   ReturnCode rc;
   WorldObject* obj;
   char fname[CHARBUFFER], name[CHARBUFFER];
#ifndef ENGINE
   char mess[CHARBUFFER];
#endif

   if (model[mod]->numworldobjects >= model[mod]->world_array_size)
   {
      model[mod]->world_array_size += ARRAY_INCREMENT;
      model[mod]->worldobj = (WorldObject*)simm_realloc(model[mod]->worldobj,
			     model[mod]->world_array_size*sizeof(WorldObject),&rc);
      if (rc == code_bad)
      {
	 model[mod]->world_array_size -= ARRAY_INCREMENT;
	 return (code_bad);
      }
   }

   obj = &model[mod]->worldobj[model[mod]->numworldobjects];

   init_world_object(model[mod],obj);

   if (fscanf(*fp,"%s", name) != 1)
   {
      error(abort_action,"Error reading name in object definition");
      return (code_bad);
   }
   else
      mstrcpy(&obj->name,name);

	while (1)
	{
		if (read_string(fp,buffer) == EOF)
		{
			(void)sprintf(errorbuffer,"Unexpected EOF found reading definition of object %s",
				obj->name);
			error(abort_action,errorbuffer);
			return (code_bad);
		}
		else if (STRINGS_ARE_EQUAL(buffer,"endworldobject"))
		{
			break;
		}
		else if (STRINGS_ARE_EQUAL(buffer,"filename"))
		{
			if (fscanf(*fp,"%s", fname) != 1)
			{
				(void)sprintf(errorbuffer,"Error reading filename in definition of worldobject %s", obj->name);
				error(abort_action,errorbuffer);
				return (code_bad);	    
			}
#ifndef ENGINE
			rc = lookup_polyhedron(obj->wobj, fname, model[mod]);

			if (rc == code_bad)
			{
				(void)sprintf(mess,"Unable to read object from file %s", fname);
				error(none,mess);
			}
			else
			{
				/* simm_printf(no,"Read object file %s\n", fname); */

				mstrcpy(&obj->filename,fname);
			}
#else
			mstrcpy(&obj->filename,fname);
#endif
		}
		else if (STRINGS_ARE_EQUAL(buffer,"origin"))
		{
			if (fscanf(*fp,"%g %g %g", &obj->origin_x, &obj->origin_y, &obj->origin_z) != 3)
			{
				(void)sprintf(errorbuffer,"Error reading origin in definition of worldobject %s", obj->name);
				error(none,errorbuffer);
				error(none,"Using default value of 0.0 0.0 0.0");
			}
		}
		else if (STRINGS_ARE_EQUAL(buffer,"scale"))
		{
			if (fscanf(*fp,"%g %g %g", &obj->scale_factor[0], &obj->scale_factor[1],
				&obj->scale_factor[2]) != 3)
			{
				(void)sprintf(errorbuffer,"Error reading scale factor in definition of worldobject %s", obj->name);
				error(none,errorbuffer);
				error(none,"Using default values of 1.0 1.0 1.0");
			}
		}
		else if (STRINGS_ARE_EQUAL(buffer,"drawmode"))
		{
			if (fscanf(*fp,"%s", buffer) != 1)
			{
				(void)sprintf(errorbuffer,"Error reading drawmode in definition of worldobject %s", obj->name);
				error(none,errorbuffer);
				error(none,"Using default value of solid_fill");
			}
			if (STRINGS_ARE_EQUAL(buffer,"wireframe"))
				obj->drawmode = wireframe;
			else if (STRINGS_ARE_EQUAL(buffer,"solid_fill"))
				obj->drawmode = solid_fill;
			else if (STRINGS_ARE_EQUAL(buffer,"flat_shading"))
				obj->drawmode = flat_shading;
			else if (STRINGS_ARE_EQUAL(buffer,"gouraud_shading"))
				obj->drawmode = gouraud_shading;
			else if (STRINGS_ARE_EQUAL(buffer,"outlined_polygons"))
				obj->drawmode = outlined_polygons;
			else if (STRINGS_ARE_EQUAL(buffer,"bounding_box"))
			{
				(void)sprintf(errorbuffer,"Warning: bounding_box drawmode not supported for worldobjects (%s).", obj->name);
				error(none,errorbuffer);
				error(none,"Using solid_fill instead.");
				obj->drawmode = solid_fill;
			}
			else if (STRINGS_ARE_EQUAL(buffer,"none"))
				obj->drawmode = no_surface;
			else
			{
				(void)sprintf(errorbuffer,"Unknown drawmode found for worldobject (%s).", obj->name);
				error(none,errorbuffer);
				error(none,"Using solid_fill instead.");
				obj->drawmode = solid_fill;
			}
		}
		else if (STRINGS_ARE_EQUAL(buffer,"material"))
		{
			if (fscanf(*fp,"%s", fname) != 1)
			{
				(void)sprintf(errorbuffer,"Error reading material in definition of worldobject %s", obj->name);
				error(none,errorbuffer);
				error(none,"Using default value of def_bone");
				obj->material = enter_material(model[mod],"def_bone",just_checking_element);
			}
			else
			{
				obj->material = enter_material(model[mod],fname,declaring_element);
			}
		}
		else
		{
			(void)sprintf(errorbuffer, "Unknown string (%s) in definition of object %s",
				buffer, obj->name);
			error(recover,errorbuffer);
		}
	}

	model[mod]->numworldobjects++;

	return (code_fine);

}



static void init_world_object(ModelStruct* ms, WorldObject* obj)
{
   obj->filename = NULL;
   obj->origin_x = obj->origin_y = obj->origin_z = 0.0;
   obj->drawmode = solid_fill;
   obj->selected = no;
   obj->material = ms->dis.mat.default_world_object_material;
   obj->scale_factor[0] = obj->scale_factor[1] = obj->scale_factor[2] = 1.0;
   obj->drawmodemenu = 0;

   /* NOTE: I added the 3 statements below to fix an intermittent crash
    * that was occurring when trying to read a world object file that
    * didn't exist.  I was still new to SIMM code at the time, so my
    * confidence level for this fix was about 60%.  -- KMS 9/3/98
   obj->wobj.num_polygons = 0;
   obj->wobj.num_vertices = 0;
    * JPL 4/5/99: added call to preread_init_polyhedron() so that world
    * object polyhedron is completely initialized.
    */

   obj->wobj = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));

   preread_init_polyhedron(obj->wobj);

}

