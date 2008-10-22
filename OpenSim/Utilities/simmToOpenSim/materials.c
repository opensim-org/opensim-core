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




/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/

#ifndef ENGINE
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
   }

}

#ifndef ENGINE
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

   if (mat->normal_list != -1)
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

   if (mat->highlighted_list != -1)
      glDeleteLists(mat->highlighted_list, 1);

   mat->highlighted_list = glGenLists(1);

   if (mat->highlighted_list == 0)
   {
      fprintf(stderr, "Unable to allocate GL display list.\n");
      mat->highlighted_list = -1;
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


ReturnCode read_material(int mod, FILE** fp)
{

   int i, ac = 0;
   MaterialStruct mat;
   char name[CHARBUFFER];


   if (fscanf(*fp,"%s", name) != 1)
   { 
      error(abort_action,"Error reading name in material definition");
      return (code_bad);
   }

   mstrcpy(&mat.name,name);

   mat.defined_in_file = yes;
   mat.ambient_defined = no;
   mat.diffuse_defined = no;
   mat.specular_defined = no;
   mat.emission_defined = no;
   mat.shininess_defined = no;
   mat.alpha_defined = no;
   mat.shininess = 10.0;
   for (i=0; i<4; i++)
   {
      mat.ambient[i] = 0.0;
      mat.diffuse[i] = 0.0;
      mat.specular[i] = 0.0;
      mat.emission[i] = 0.0;
   }
   /* the alpha component is stored in diffuse[3] */
   mat.diffuse[3] = 1.0;

   while (1)
   {
      if (read_string(fp,buffer) == EOF)
      {
         error(abort_action,"Unexpected EOF found reading material definition");
         return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"endmaterial"))
      {
         if (define_material(model[mod],&mat) == -1)
            return code_bad;
         break;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"ambient"))
      {
         if (mat.ambient_defined == yes)
            error(none,"Ignoring redefinition of ambient in material definition");
         else
         {
            if (fscanf(*fp,"%f %f %f", &mat.ambient[0], &mat.ambient[1],
               &mat.ambient[2]) != 3)
            {
               error(abort_action,"Error reading ambient in material definition");
               return (code_bad);
            }
            mat.ambient[3] = 1.0;
            ac += 3;
            mat.ambient_defined = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"diffuse"))
      {
         if (mat.diffuse_defined == yes)
            error(none,"Ignoring redefinition of diffuse in material definition");
         else
         {
            if (fscanf(*fp,"%f %f %f", &mat.diffuse[0], &mat.diffuse[1],
               &mat.diffuse[2]) != 3)
            {
               error(abort_action,"Error reading diffuse in material definition");
               return (code_bad);
            }
            ac += 3;
            mat.diffuse_defined = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"emission"))
      {
         if (mat.emission_defined == yes)
            error(none,"Ignoring redefinition of emission in material definition");
         else
         {
            if (fscanf(*fp,"%f %f %f", &mat.emission[0], &mat.emission[1],
               &mat.emission[2]) != 3)
            {
               error(abort_action,"Error reading emission in material definition");
               return (code_bad);
            }
            mat.emission[3] = 1.0;
            ac += 3;
            mat.emission_defined = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"specular"))
      {
         if (mat.specular_defined == yes)
            error(none,"Ignoring redefinition of specular in material definition");
         else
         {
            if (fscanf(*fp,"%f %f %f", &mat.specular[0], &mat.specular[1],
               &mat.specular[2]) != 3)
            {
               error(abort_action,"Error reading specular in material definition");
               return (code_bad);
            }
            mat.specular[3] = 1.0;
            ac += 3;
            mat.specular_defined = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"shininess"))
      {
         if (mat.shininess_defined == yes)
            error(none,"Ignoring redefinition of shininess in material definition");
         else
         {
            if (fscanf(*fp,"%f", &mat.shininess) != 1)
            {
               error(abort_action,"Error reading shininess in material definition");
               return (code_bad);
            }
            ac++;
            mat.shininess_defined = yes;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"alpha"))
      {
         if (mat.alpha_defined == yes)
            error(none,"Ignoring redefinition of alpha in material definition");
         else
         {
            if (fscanf(*fp,"%f", &mat.diffuse[3]) != 1)
            {
               error(abort_action,"Error reading alpha in material definition");
               return (code_bad);
            }
            ac++;
            mat.alpha_defined = yes;
         }
      }
      else
      {
         (void)sprintf(errorbuffer, "Unknown string (%s) in definition of material", buffer);
         error(recover,errorbuffer);
      }
   }

   return (code_fine);

}


public void copy_material(MaterialStruct* src, MaterialStruct* dst)
{
   int i;

   FREE_IFNOTNULL(dst->name);

   memcpy(dst, src, sizeof(MaterialStruct));

   mstrcpy(&dst->name, src->name);

   dst->normal_list = -1;
   dst->highlighted_list = -1;
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


int enter_material(ModelStruct* ms, char name[], EnterMode emode)
{

   int i, m;
   ReturnCode rc;

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

#ifndef ENGINE
void apply_material(ModelStruct* ms, int mat, SBoolean highlight)
{

   if (highlight == yes)
      glCallList(ms->dis.mat.materials[mat].highlighted_list);
   else
      glCallList(ms->dis.mat.materials[mat].normal_list);

}
#endif
