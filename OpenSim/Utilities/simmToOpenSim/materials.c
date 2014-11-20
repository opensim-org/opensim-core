/*******************************************************************************

   MATERIALS.C

   Author: Peter Loan

   Date: 30-DEC-91

   Copyright (c) 1992-8 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: 

   Routines:
      read_material
      define_material
      enter_material


*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"
#if ! ENGINE
#include "tiffio.h"
#endif


/*************** DEFINES (for this file only) *********************************/
#define ALL_MODELS -1


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static GLfloat light0_diffuse[] = {1.0f, 0.96f, 0.87f, 1.0f};
static GLfloat light0_position[] = {0.10f, 0.01f, 0.5f, 0.0f};
static GLfloat light1_diffuse[] = {1.0f, 1.0f, 0.7f, 1.0f};
static GLfloat light1_position[] = {-0.01f, 0.01f, 0.5f, 0.0f};
static GLfloat light_model[] = {0.77f, 0.67f, 0.57f};
static MaterialStruct null_mat = {"null_mat", yes, no,
yes, 0.0f, 0.0f, 0.0f, 1.0f, /* ambient */
yes, 0.0f, 0.0f, 0.0f, 1.0f, /* diffuse */
yes, 0.0f, 0.0f, 0.0f, 1.0f, /* specular */
yes, 0.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 1.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct arrow = {"def_motion", yes, no,
yes, 0.1f, 0.7f, 0.1f, 1.0f, /* ambient */
yes, 0.2f, 0.3f, 0.2f, 1.0f, /* diffuse */
yes, 0.2f, 0.0f, 0.2f, 1.0f, /* specular */
yes, 1.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct joint_vector = {"def_joint_vector", yes, no,
yes, 0.01f, 0.9f, 0.01f,  //0.9f, 0.3f, 0.2f, 1.0f, /* ambient */
yes, 0.1f, 0.9f, 0.1f, //0.2f, 0.3f, 0.2f, 1.0f, /* diffuse */
yes, 0.14f, 0.0f, 0.2f, 1.0f, /* specular */
yes, 25.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct def_bone = {"def_bone", yes, no,
yes, 0.7f, 0.70f, 0.7f, 1.0f, /* ambient */
yes, 0.6f, 0.45f, 0.4f, 1.0f, /* diffuse */
yes, 0.7f, 0.55f, 0.4f, 1.0f, /* specular */
yes, 10.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct highlighted1_polygon = {"def_sel_poly", yes, no,
yes, 0.2f, 0.70f, 0.2f, 1.0f, /* ambient */
yes, 0.1f, 0.45f, 0.1f, 1.0f, /* diffuse */
yes, 0.4f, 0.65f, 0.4f, 1.0f, /* specular */
yes, 10.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct highlighted2_polygon = {"def_sel_poly2", yes, no,
yes, 0.2f, 0.2f, 0.7f, 1.0f, /* ambient */
yes, 0.1f, 0.1f, 0.45f, 1.0f, /* diffuse */
yes, 0.4f, 0.4f, 0.65f, 1.0f, /* specular */
yes, 10.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct marker_mat = {"def_marker", yes, no,
yes, 0.8f, 0.2f, 0.8f, 1.0f, /* ambient */
yes, 0.4f, 0.1f, 0.4f, 1.0f, /* diffuse */
yes, 1.0f, 0.3f, 1.0f, 1.0f, /* specular */
yes, 30.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct sel_marker_mat = {"sel_marker", yes, no,
yes, 0.9f, 0.9f, 0.0f, 1.0f, /* ambient */
yes, 0.2f, 0.2f, 0.0f, 1.0f, /* diffuse */
yes, 1.0f, 1.0f, 0.0f, 1.0f, /* specular */
yes, 30.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct bone_outline = {"def_bone_outline", yes, no,
yes, 0.2f, 0.0f, 0.0f, 1.0f, /* ambient */
yes, 0.2f, 0.0f, 0.0f, 1.0f, /* diffuse */
yes, 0.2f, 0.0f, 0.0f, 1.0f, /* specular */
yes, 90.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct def_min_muscle = {"def_min_muscle", yes, no,
yes, 0.2f, 0.0f, 0.0f, 1.0f, /* ambient */
yes, 0.4f, 0.0f, 0.0f, 1.0f, /* diffuse */
yes, 1.0f, 0.15f, 0.15f, 1.0f, /* specular */
yes, 40.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct def_max_muscle = {"def_max_muscle", yes, no,
yes, 0.4f, 0.0f, 0.0f, 1.0f, /* ambient */
yes, 0.7f, 0.0f, 0.0f, 1.0f, /* diffuse */
yes, 1.0f, 0.15f, 0.15f, 1.0f, /* specular */
yes, 90.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct def_muscle_point = {"def_muscle_point", yes, no,
yes, 0.0f, 0.0f, 0.0f, 1.0f, /* ambient */
yes, 0.0f, 0.0f, 0.7f, 1.0f, /* diffuse */
yes, 0.15f, 0.15f, 1.0f, 1.0f, /* specular */
yes, 90.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct sel_muscle_point = {"def_sel_muscle_point", yes, no,
yes, 0.0f, 0.0f, 0.0f, 1.0f, /* ambient */
yes, 0.7f, 0.7f, 0.0f, 1.0f, /* diffuse */
yes, 1.0f, 1.0f, 0.2f, 1.0f, /* specular */
yes, 90.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct def_world_object = {"def_world_object", yes, no,
yes, 0.2f, 0.2f, 0.7f, 1.0f, /* ambient */
yes, 0.1f, 0.1f, 0.45f, 1.0f, /* diffuse */
yes, 0.4f, 0.4f, 0.65f, 1.0f, /* specular */
yes, 10.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};

/* constraint object materials */
static MaterialStruct co_current_active = {"co_current_active", yes, no,
yes, 0.0f, 1.0f, 0.6f, 1.0f, /* ambient */
yes, 0.0f, 0.5f, 0.6f, 1.0f, /* diffuse */
yes, 0.0f, 0.7f, 0.7f, 1.0f, /* specular */
yes, 10.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct co_current_inactive = {"co_current_inactive", yes, no,
yes, 0.6f, 0.05f, 0.0f, 1.0f, /* ambient */
yes, 0.2f, 0.2f, 0.0f, 1.0f, /* diffuse */
yes, 1.0f, 1.0f, 0.0f, 1.0f, /* specular */
yes, 30.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct co_back_active = {"co_back_active", yes, no,
yes, 0.0f, 0.55f, 0.25f, 1.0f, /* ambient */
yes, 0.0f, 0.05f, 0.06f, 1.0f, /* diffuse */
yes, 0.0f, 0.07f, 0.07f, 1.0f, /* specular */
yes, 10.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct co_back_inactive = {"co_back_inactive", yes, no,
yes, 0.4f, 0.02f, 0.0f, 1.0f, /* ambient */
yes, 0.02f, 0.02f, 0.0f, 1.0f, /* diffuse */
yes, 1.0f, 1.0f, 0.0f, 1.0f, /* specular */
yes, 30.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};

/* constraint point materials */
static MaterialStruct cp_sel_ok = {"cp_sel_ok", yes, no,
yes, 0.9f, 0.9f, 0.0f, 1.0f, /* ambient */
yes, 0.2f, 0.2f, 0.0f, 1.0f, /* diffuse */
yes, 1.0f, 1.0f, 0.0f, 1.0f, /* specular */
yes, 30.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct cp_sel_broken = {"cp_sel_broken", yes, no,
//yes, 0.0f, 0.0f, 0.0f, 1.0f, /* ambient */
yes, 0.2f, 0.0f, 0.0f, 1.0f, /* ambient */
yes, 0.7f, 0.7f, 0.0f, 1.0f, /* diffuse */
yes, 1.0f, 1.0f, 0.2f, 1.0f, /* specular */
yes, 90.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct cp_current_ok = {"cp_current_ok", yes, no,
yes, 0.0f, 1.0f, 0.6f, 1.0f, /* ambient */
yes, 0.0f, 0.5f, 0.6f, 1.0f, /* diffuse */
yes, 0.0f, 0.7f, 0.7f, 1.0f, /* specular */
yes, 10.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct cp_current_broken = {"cp_current_broken", yes, no,
yes, 1.0f, 0.3f, 0.01f, 1.0f, /* ambient */
yes, 0.5f, 0.1f, 0.1f, 1.0f, /* diffuse */
yes, 0.5f, 0.5f, 0.5f, 1.0f, /* specular */
yes, 30.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct cp_back_broken = {"cp_back_broken", yes, no,
yes, 0.4f, 0.02f, 0.0f, 1.0f, /* ambient */
yes, 0.02f, 0.02f, 0.0f, 1.0f, /* diffuse */
yes, 1.0f, 1.0f, 0.0f, 1.0f, /* specular */
yes, 30.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct masscenter_mat1 = {"masscenter_mat1", yes, no,
yes, 0.4f, 0.2f, 0.0f, 1.0f, /* ambient */
yes, 0.22f, 0.18f, 0.05f, 1.0f, /* diffuse */
yes, 0.8f, 0.8f, 0.4f, 1.0f, /* specular */
yes, 93.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct masscenter_mat2 = {"masscenter_mat2", yes, no,
yes, 0.2f, 0.2f, 0.2f, 1.0f, /* ambient */
yes, 0.05f, 0.05f, 0.05f, 1.0f, /* diffuse */
yes, 1.0f, 1.0f, 1.0f, 1.0f, /* specular */
yes, 95.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};
static MaterialStruct foot_print_mat = {"foot_print_mat", yes, no,
yes, 0.2f, 0.2f, 0.2f, 1.0f, /* ambient */
yes, 0.05f, 0.05f, 0.05f, 1.0f, /* diffuse */
yes, 0.8f, 0.8f, 0.4f, 1.0f, /* specular */
yes, 95.0f, /* shininess */
yes, 0.0f, 0.0f, 0.0f, 0.0f, /* emission */
no, /* alpha defined */
-1, -1 /* GL list numbers */
};


/*************** EXTERNED VARIABLES (declared in another file) ****************/


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
#if ! ENGINE
static FileReturnCode load_texture_file(PolyhedronStruct* ph, char filename[]);
static FileReturnCode load_texture_coordinates(PolyhedronStruct* ph, char filename[]);
#endif


#if ! ENGINE
void init_global_lighting()
{
   /* Initialize the global lighting parameters. The specific
    * values of the light source parameters need to be set in
    * each model window, not here.
    */
   glShadeModel(GL_FLAT);
   glEnable(GL_FRONT);
}
#endif

void init_materials(ModelStruct* ms)
{
   if (ms != NULL)
   {
      int i;
      ModelMaterials* mat = &ms->dis.mat;

      mat->num_materials = 0;
      mat->material_array_size = DEFAULT_ARRAY_INCREMENT;
      mat->materials = (MaterialStruct*)simm_malloc(mat->material_array_size * sizeof(MaterialStruct));

      for (i = 0; i < mat->material_array_size; i++)
      {
         mat->materials[i].name = NULL;
         mat->materials[i].defined_yet = no;
         mat->materials[i].defined_in_file = no;
      }

      mat->null_material = define_material(ms, &null_mat);
      mat->default_bone_material = define_material(ms, &def_bone);
      mat->default_world_object_material = define_material(ms, &def_world_object);
      mat->default_muscle_min_material = define_material(ms, &def_min_muscle);
      mat->default_muscle_max_material = define_material(ms, &def_max_muscle);
      mat->def_muscle_point_material = define_material(ms, &def_muscle_point);
      mat->sel_muscle_point_material = define_material(ms, &sel_muscle_point);
      mat->bone_outline_material = define_material(ms, &bone_outline);
      mat->highlighted1_polygon_material = define_material(ms, &highlighted1_polygon);
      mat->highlighted2_polygon_material = define_material(ms, &highlighted2_polygon);
      mat->marker_material = define_material(ms, &marker_mat);
      mat->sel_marker_material = define_material(ms, &sel_marker_mat);

      mat->co_current_active_material = define_material(ms, &co_current_active);
      mat->co_current_inactive_material = define_material(ms, &co_current_inactive);
      mat->co_back_active_material = define_material(ms, &co_back_active);
      mat->co_back_inactive_material = define_material(ms, &co_back_inactive);

      mat->cp_current_sel_ok_material = define_material(ms, &cp_sel_ok);              /*  */
      mat->cp_current_sel_broken_material = define_material(ms, &cp_sel_broken);              /*  */
      mat->cp_current_ok_material = define_material(ms, &cp_current_ok);              /*  */
      mat->cp_current_broken_material = define_material(ms, &cp_current_broken);              /*  */
      mat->cp_current_inactive_material = ms->dis.mat.co_current_inactive_material;
      mat->cp_back_ok_material = ms->dis.mat.co_back_active_material;
      mat->cp_back_broken_material = define_material(ms, &cp_back_broken);
      mat->cp_back_inactive_material = ms->dis.mat.co_back_inactive_material;

      mat->masscenter_material1 = define_material(ms, &masscenter_mat1);
      mat->masscenter_material2 = define_material(ms, &masscenter_mat2);

      define_material(ms, &arrow);
      define_material(ms, &joint_vector);
      define_material(ms, &foot_print_mat);
   }

}

#if ! ENGINE
void init_model_lighting()
{
   /* These parameters need to be set in each model window */
   glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
   glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

#if INCLUDE_2ND_LIGHT_SOURCE || 0
   glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
   glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
#endif
   glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_model);
}


void make_mat_display_list(MaterialStruct* mat)
{

   float polygonBackColor[4];

#if 0 /* this color is now specified on a per model basis */
   polygonBackColor[0] = root.color.cmap[MISC_MODEL_WINDOW_BACKGROUND].rgb[0];
   polygonBackColor[1] = root.color.cmap[MISC_MODEL_WINDOW_BACKGROUND].rgb[1];
   polygonBackColor[2] = root.color.cmap[MISC_MODEL_WINDOW_BACKGROUND].rgb[2];
   polygonBackColor[3] = 1.0;
#else
   polygonBackColor[0] = 0.0;
   polygonBackColor[1] = 0.0;
   polygonBackColor[2] = 0.0;
   polygonBackColor[3] = 1.0;
#endif

   if (mat->normal_list)
      glDeleteLists(mat->normal_list, 1);

   mat->normal_list = glGenLists(1);

   if (mat->normal_list == 0)
   {
      fprintf(stderr, "Unable to allocate GL display list.\n");
      mat->normal_list = -1;
      return;
   }

   glNewList(mat->normal_list, GL_COMPILE);

   if (mat->diffuse[3] < 1.0)
   {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   }
   else
     glDisable(GL_BLEND);

   glMaterialfv(GL_BACK, GL_DIFFUSE, polygonBackColor);

   if (mat->ambient_defined)
      glMaterialfv(GL_FRONT, GL_AMBIENT, mat->ambient);
   else
      glMaterialfv(GL_FRONT, GL_AMBIENT, null_mat.ambient);

   if (mat->diffuse_defined)
      glMaterialfv(GL_FRONT, GL_DIFFUSE, mat->diffuse);
   else
      glMaterialfv(GL_FRONT, GL_DIFFUSE, null_mat.diffuse);

   if (mat->specular_defined)
      glMaterialfv(GL_FRONT, GL_SPECULAR, mat->specular);
   else
      glMaterialfv(GL_FRONT, GL_SPECULAR, null_mat.specular);

   if (mat->emission_defined)
      glMaterialfv(GL_FRONT, GL_EMISSION, mat->emission);
   else
      glMaterialfv(GL_FRONT, GL_EMISSION, null_mat.emission);

   if (mat->shininess_defined)
      glMaterialf(GL_FRONT, GL_SHININESS, mat->shininess);
   else
      glMaterialf(GL_FRONT, GL_SHININESS, null_mat.shininess);

   glEndList();

}


void make_highlight_mat_display_list(MaterialStruct* mat)
{

   GLfloat new_col[3], new_col2[3];
   float polygonBackColor[4];

#if 0 /* this color is now specified on a per model basis */
   polygonBackColor[0] = root.color.cmap[MISC_MODEL_WINDOW_BACKGROUND].rgb[0];
   polygonBackColor[1] = root.color.cmap[MISC_MODEL_WINDOW_BACKGROUND].rgb[1];
   polygonBackColor[2] = root.color.cmap[MISC_MODEL_WINDOW_BACKGROUND].rgb[2];
   polygonBackColor[3] = 1.0;
#else
   polygonBackColor[0] = 0.0;
   polygonBackColor[1] = 0.0;
   polygonBackColor[2] = 0.0;
   polygonBackColor[3] = 1.0;
#endif

   if (mat->highlighted_list)
      glDeleteLists(mat->highlighted_list, 1);

   mat->highlighted_list = glGenLists(1);

   if (mat->highlighted_list == 0)
   {
      fprintf(stderr, "Unable to allocate GL display list.\n");
      return;
   }

   glNewList(mat->highlighted_list, GL_COMPILE);
 
   if (mat->diffuse[3] < 1.0)
   {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   }
   else
     glDisable(GL_BLEND);

   glMaterialfv(GL_BACK, GL_DIFFUSE, polygonBackColor);

   make_highlight_color(mat->diffuse, new_col);

   if (mat->diffuse_defined)
      glMaterialfv(GL_FRONT, GL_DIFFUSE, new_col);

   if (mat->ambient_defined)
   {
      make_ambient_color(new_col, new_col2);
      glMaterialfv(GL_FRONT, GL_AMBIENT, new_col2);
   }

   if (mat->specular_defined)
   {
      make_specular_color(new_col,new_col2);
      glMaterialfv(GL_FRONT, GL_SPECULAR, new_col2);
   }

   if (mat->emission_defined)
      glMaterialfv(GL_FRONT, GL_EMISSION, mat->emission);

   if (mat->shininess_defined)
      glMaterialf(GL_FRONT, GL_SHININESS, mat->shininess);

   glEndList();

}
#endif /* ENGINE */


ReturnCode read_material(ModelStruct* ms, FILE* fp)
{
   int i, ac = 0;
   MaterialStruct mat;
   char name[CHARBUFFER];

   if (fscanf(fp, "%s", name) != 1)
   { 
      error(abort_action, "Error reading name in material definition");
      return code_bad;
   }

   memset(&mat, 0, sizeof(MaterialStruct));

   mat.defined_in_file = yes;
   mat.shininess = 10.0;
   /* the alpha component is stored in diffuse[3] */
   mat.diffuse[3] = 1.0;

   while (1)
   {
      if (read_string(fp,buffer) == EOF)
      {
         error(abort_action, "Unexpected EOF found reading material definition");
         return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "endmaterial"))
      {
         mstrcpy(&mat.name,name);
         if (define_material(ms, &mat) == -1)
         {
            FREE_IFNOTNULL(mat.name);
            return code_bad;
         }
         break;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "ambient"))
      {
         if (mat.ambient_defined == yes)
            error(none, "Ignoring redefinition of ambient in material definition");
         else
         {
            if (fscanf(fp, "%f %f %f", &mat.ambient[0], &mat.ambient[1],
               &mat.ambient[2]) != 3)
            {
               error(abort_action, "Error reading ambient in material definition");
               return code_bad;
            }
            mat.ambient[3] = 1.0;
            ac += 3;
            mat.ambient_defined = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "diffuse"))
      {
         if (mat.diffuse_defined == yes)
            error(none, "Ignoring redefinition of diffuse in material definition");
         else
         {
            if (fscanf(fp, "%f %f %f", &mat.diffuse[0], &mat.diffuse[1],
               &mat.diffuse[2]) != 3)
            {
               error(abort_action, "Error reading diffuse in material definition");
               return code_bad;
            }
            ac += 3;
            mat.diffuse_defined = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "emission"))
      {
         if (mat.emission_defined == yes)
            error(none, "Ignoring redefinition of emission in material definition");
         else
         {
            if (fscanf(fp, "%f %f %f", &mat.emission[0], &mat.emission[1],
               &mat.emission[2]) != 3)
            {
               error(abort_action, "Error reading emission in material definition");
               return code_bad;
            }
            mat.emission[3] = 1.0;
            ac += 3;
            mat.emission_defined = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "specular"))
      {
         if (mat.specular_defined == yes)
            error(none, "Ignoring redefinition of specular in material definition");
         else
         {
            if (fscanf(fp, "%f %f %f", &mat.specular[0], &mat.specular[1],
               &mat.specular[2]) != 3)
            {
               error(abort_action, "Error reading specular in material definition");
               return code_bad;
            }
            mat.specular[3] = 1.0;
            ac += 3;
            mat.specular_defined = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "shininess"))
      {
         if (mat.shininess_defined == yes)
            error(none, "Ignoring redefinition of shininess in material definition");
         else
         {
            if (fscanf(fp, "%f", &mat.shininess) != 1)
            {
               error(abort_action, "Error reading shininess in material definition");
               return code_bad;
            }
            ac++;
            mat.shininess_defined = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "alpha"))
      {
         if (mat.alpha_defined == yes)
            error(none, "Ignoring redefinition of alpha in material definition");
         else
         {
            if (fscanf(fp, "%f", &mat.diffuse[3]) != 1)
            {
               error(abort_action, "Error reading alpha in material definition");
               return code_bad;
            }
            ac++;
            mat.alpha_defined = yes;
         }
      }
      else
      {
         (void)sprintf(errorbuffer, "Unknown string (%s) in definition of material", buffer);
         error(recover, errorbuffer);
      }
   }

   FREE_IFNOTNULL(mat.name);

   return code_fine;
}


public void copy_material(MaterialStruct* src, MaterialStruct* dst)
{
   int i;

   FREE_IFNOTNULL(dst->name);

   memcpy(dst, src, sizeof(MaterialStruct));

   mstrcpy(&dst->name, src->name);

   dst->normal_list = 0;
   dst->highlighted_list = 0;
   dst->defined_yet = yes;
}


int define_material(ModelStruct* ms, MaterialStruct* mat)
{
   int m;

   if (ms == NULL)
      return -1;

   m = enter_material(ms, mat->name, defining_element);

   /* If there was an error entering the material in the array, return the
    * null material.
    */

   if (m == -1)
      return ms->dis.mat.null_material;

   if (ms->dis.mat.materials[m].defined_in_file == yes)
   {
      (void)sprintf(errorbuffer,"Error: redefinition of material %s", mat->name);
      error(abort_action,errorbuffer);
      return -1;
   }

   copy_material(mat, &ms->dis.mat.materials[m]);

   return m;
}


int enter_material(ModelStruct* ms, const char name[], EnterMode emode)
{
   int i, m;
   ReturnCode rc = code_fine;

   if (ms == NULL)
      return -1;

   for (i=0; i<ms->dis.mat.num_materials; i++)
      if (STRINGS_ARE_EQUAL(name,ms->dis.mat.materials[i].name))
         return i;

   m = i;

   /* If you are just checking to see if the name has already been defined (or declared),
    * and you reach the end of the array without finding it, return an error.
    */

   if (m == ms->dis.mat.num_materials && emode == just_checking_element)
      return -1;

   if (ms->dis.mat.num_materials >= ms->dis.mat.material_array_size)
   {
      ms->dis.mat.material_array_size += DEFAULT_ARRAY_INCREMENT;
      ms->dis.mat.materials = (MaterialStruct*)simm_realloc(ms->dis.mat.materials,
         ms->dis.mat.material_array_size*sizeof(MaterialStruct),&rc);
      if (rc == code_bad)
      {
         ms->dis.mat.material_array_size -= DEFAULT_ARRAY_INCREMENT;
         return ZERO;
      }
      for (i=ms->dis.mat.num_materials; i<ms->dis.mat.material_array_size; i++)
      {
         ms->dis.mat.materials[i].name = NULL;
         ms->dis.mat.materials[i].defined_yet = no;
         ms->dis.mat.materials[i].defined_in_file = no;
      }
   }
   mstrcpy(&ms->dis.mat.materials[m].name,name);
   ms->dis.mat.num_materials++;

   return m;
}

#if ! ENGINE
void apply_material(ModelStruct* ms, int mat, SBoolean highlight)
{

   if (highlight == yes)
      glCallList(ms->dis.mat.materials[mat].highlighted_list);
   else
      glCallList(ms->dis.mat.materials[mat].normal_list);

}

static FileReturnCode load_texture_file(PolyhedronStruct* ph, char filename[])
{
   int width, height;
   unsigned int* data;

   if (read_tiff_image(filename, &width, &height, &data) == file_missing)
   {
      ph->texture = -1;
      return file_missing;
   }

   // allocate a texture name
   glGenTextures(1, &ph->texture);

   // select our current texture
   glBindTexture(GL_TEXTURE_2D, ph->texture);

   // select modulate to mix texture with color for shading
   glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

   // when texture area is small, bilinear filter the closest mipmap
   //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
   // when texture area is large, bilinear filter the first mipmap
   //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

   // if wrap is true, the texture wraps over at the edges (repeat)
   //       ... false, the texture ends at the edges (clamp)
   //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap ? GL_REPEAT : GL_CLAMP);
   //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap ? GL_REPEAT : GL_CLAMP);

   // build our texture mipmaps
   //gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);

   glTexImage2D(GL_TEXTURE_2D, 0, 4, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

   // free buffer
   _TIFFfree(data);

   return file_good;
}

//TODO5.0: right now textures are supported only in world objects.
// This function must be called when the model's window is the current
// GL window.
//TODO5.0: what if the model is in two windows?
void load_model_textures(ModelStruct* model)
{
   int i;

   for (i=0; i<model->numworldobjects; i++)
   {
      if (model->worldobj[i].texture_filename && model->worldobj[i].wobj)
      {
            FileReturnCode frc = lookup_texture_file(model->worldobj[i].wobj, model->worldobj[i].texture_filename, model);

            if (frc == file_missing)
         {
                (void)sprintf(errorbuffer, "Unable to locate texture file %s", model->worldobj[i].texture_filename);
                error(none, errorbuffer);
                model->worldobj[i].wobj->texture = -1;
         }
         else if (frc == file_bad)
            {
                (void)sprintf(errorbuffer, "Unable to read texture from file %s", model->worldobj[i].texture_filename);
                error(none, errorbuffer);
                model->worldobj[i].wobj->texture = -1;
            }
      }
   }
}
/* -------------------------------------------------------------------------
   lookup_texture_file - this routine tries to read the specified texture file
     by first checking the current directory, and then checking the
     standard SIMM directory for bones.
     TODO5.0: If one of the paths to search starts with a drive letter that
     does not exist, Vista will display an annoying error dialog (other versions
     of Windows quietly move on). Find some way to check for a valid drive
     letter before performing the action that causes the error.
---------------------------------------------------------------------------- */
FileReturnCode lookup_texture_file(PolyhedronStruct* ph, char filename[], ModelStruct* ms)
{
   int i;
   char* jointpath = NULL;
   char fullpath[CHARBUFFER], tmppath[CHARBUFFER];
   FileReturnCode rc;

   /* (0) strip the joint file name from ms->jointfilename to get just
    * the path to the joint file.
    */
   if (ms && ms->jointfilename)
   {
      get_pure_path_from_path(ms->jointfilename, &jointpath);
   }

   /* (1) First check the bone folder specified in the joint file.
    * If this is an absolute path, use it as is. If it is a relative
    * path, it is assumed to be relative to the joint file's folder.
    */
   if (ms && ms->bonepathname)
   {
      if (ms->jointfilename)
         build_full_path(jointpath, ms->bonepathname, tmppath);
      else
         build_full_path(NULL, ms->bonepathname, tmppath);
      build_full_path(tmppath, filename, fullpath);
      rc = load_texture_file(ph, fullpath);
      if (rc == file_good || rc == file_bad)
      {
         FREE_IFNOTNULL(jointpath);
         return rc;
      }
   }

   /* (2) Next check the folder "bones" under the joint file's
    * folder (PC only).
    */
   if (ms->jointfilename)
      strcat3(tmppath, jointpath, DIR_SEP_STRING, "bones", CHARBUFFER);
   else
      strcpy(tmppath, "bones");
   strcat3(fullpath, tmppath, DIR_SEP_STRING, filename, CHARBUFFER);
   rc = load_texture_file(ph, fullpath);
   if (rc == file_good || rc == file_bad)
   {
      FREE_IFNOTNULL(jointpath);
      return rc;
   }

   /* (3) Next check the joint file's folder itself (PC only). */
   if (ms->jointfilename)
      strcpy(tmppath, jointpath);
   else
      strcpy(tmppath, ".");
   FREE_IFNOTNULL(jointpath);
   strcat3(fullpath, tmppath, DIR_SEP_STRING, filename, CHARBUFFER);
   rc = load_texture_file(ph, fullpath);
   if (rc == file_good || rc == file_bad)
      return rc;

   /* (4) check the global bones folder. */
   build_full_path(get_bones_folder(), filename, fullpath);
   rc = load_texture_file(ph, fullpath);
   if (rc == file_good || rc == file_bad)
      return rc;

   /* (5) check the mocap bones folder. */
   sprintf(fullpath, "%s%s%s%s", get_mocap_folder(), "bones", DIR_SEP_STRING, filename);
   rc = load_texture_file(ph, fullpath);
   if (rc == file_good || rc == file_bad)
      return rc;

   /* You only make it to here if the file was not found in
    * any of the folders.
    */
   return file_missing;
}


static FileReturnCode load_texture_coordinates(PolyhedronStruct* ph, char filename[])
{
   FileReturnCode frc = file_good;
   FILE* fp = fopen(filename, "r");

   if (fp == NULL)
   {
      frc = file_missing;
   }
   else
   {
      int i;

      for (i=0; i<ph->num_vertices; i++)
      {
         if (fscanf(fp, "%f %f", &ph->vertex[i].texture[0], &ph->vertex[i].texture[1]) != 2)
         {
            frc = file_bad;
            fclose(fp);
            return frc;
         }
      }

      fclose(fp);
   }

   return frc;
}


/* -------------------------------------------------------------------------
   lookup_texture_coord_file - this routine tries to read the specified file
     of texture coordinates by first checking the current directory, and then
     checking the standard SIMM directory for bones.
     TODO5.0: If one of the paths to search starts with a drive letter that
     does not exist, Vista will display an annoying error dialog (other versions
     of Windows quietly move on). Find some way to check for a valid drive
     letter before performing the action that causes the error.
---------------------------------------------------------------------------- */
FileReturnCode lookup_texture_coord_file(PolyhedronStruct* ph, char filename[], ModelStruct* ms)
{
   int i;
   char* jointpath = NULL;
   char fullpath[CHARBUFFER], tmppath[CHARBUFFER];
   FileReturnCode rc;

   /* (0) strip the joint file name from ms->jointfilename to get just
    * the path to the joint file.
    */
   if (ms && ms->jointfilename)
   {
      get_pure_path_from_path(ms->jointfilename, &jointpath);
   }

   /* (1) First check the bone folder specified in the joint file.
    * If this is an absolute path, use it as is. If it is a relative
    * path, it is assumed to be relative to the joint file's folder.
    */
   if (ms && ms->bonepathname)
   {
      if (ms->jointfilename)
         build_full_path(jointpath, ms->bonepathname, tmppath);
      else
         build_full_path(NULL, ms->bonepathname, tmppath);
      build_full_path(tmppath, filename, fullpath);
      rc = load_texture_coordinates(ph, fullpath);
      if (rc == file_good || rc == file_bad)
      {
         FREE_IFNOTNULL(jointpath);
         return rc;
      }
   }

   /* (2) Next check the folder "bones" under the joint file's
    * folder (PC only).
    */
   if (ms->jointfilename)
      strcat3(tmppath, jointpath, DIR_SEP_STRING, "bones", CHARBUFFER);
   else
      strcpy(tmppath, "bones");
   strcat3(fullpath, tmppath, DIR_SEP_STRING, filename, CHARBUFFER);
   rc = load_texture_coordinates(ph, fullpath);
   if (rc == file_good || rc == file_bad)
   {
      FREE_IFNOTNULL(jointpath);
      return rc;
   }

   /* (3) Next check the joint file's folder itself (PC only). */
   if (ms->jointfilename)
      strcpy(tmppath, jointpath);
   else
      strcpy(tmppath, ".");
   FREE_IFNOTNULL(jointpath);
   strcat3(fullpath, tmppath, DIR_SEP_STRING, filename, CHARBUFFER);
   rc = load_texture_coordinates(ph, fullpath);
   if (rc == file_good || rc == file_bad)
      return rc;

   /* (4) check the global bones folder. */
   build_full_path(get_bones_folder(), filename, fullpath);
   rc = load_texture_coordinates(ph, fullpath);
   if (rc == file_good || rc == file_bad)
      return rc;

   /* (5) check the mocap bones folder. */
   sprintf(fullpath, "%s%s%s%s", get_mocap_folder(), "bones", DIR_SEP_STRING, filename);
   rc = load_texture_coordinates(ph, fullpath);
   if (rc == file_good || rc == file_bad)
      return rc;

   /* You only make it to here if the file was not found in
    * any of the folders.
    */
   return file_missing;
}

/* -------------------------------------------------------------------------
   read_tiff_image - read the specified TIFF file into memory as a 32-bit
       RGBA image.
       
   NOTE: if 'image' is NULL, then the image is not actually read, only the
       width and height of the image are returned.
       
   NOTE: it is the caller's responsibility to deallocated the image when it
       finished with it.
---------------------------------------------------------------------------- */
FileReturnCode read_tiff_image(const char* filename, int* width, int* height, unsigned** image)
{
   TIFF* tif;
   FILE* fp = simm_fopen(filename, "r");

   if (fp)
   {
      fclose(fp);
   }
   else
   {
      return file_missing;
   }

   tif = TIFFOpen(filename, "r");

   if (tif)
   {
      TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, width);
      TIFFGetField(tif, TIFFTAG_IMAGELENGTH, height);

      if (image)
      {
         *image = (unsigned*) _TIFFmalloc(*width * *height * sizeof(unsigned));

         if (*image)
            TIFFReadRGBAImage(tif, *width, *height, (unsigned long*) *image, 0);
      }
      TIFFClose(tif);
   }

   return file_good;
}

#endif
