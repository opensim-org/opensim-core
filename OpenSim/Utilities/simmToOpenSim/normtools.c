/*******************************************************************************

   NORMTOOLS.C

   Author: Peter Loan

   Date: 12-SEP-96

   Copyright (c) 1996 MusculoGraphics, Inc.
   All rights reserved.

*******************************************************************************/
#include "universal.h"
#include "functions.h"
#include "globals.h"

#include "normio.h"
#include "normtools.h"


/*************** DEFINES (for this file only) *********************************/
#define MAX_HOLE_EDGES 5000
#define MAX_POLY_EDGES 500
#define SKIP_ME -9999
#define NORM_EPSILON 0.0000000001
#define CONVEX_EPSILON 0.000001
#define CONVEX_EPSILON_SQR 0.00000000001
#define COLLINEAR_EPSILON 0.001
#define ANGLE_EPSILON 0.001


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static char* axis_names[] = {"X","Y","Z"};


/*************** GLOBAL VARIABLES (used in only a few files) ******************/
SBoolean verbose = no;


/*************** EXTERNED VARIABLES (declared in another .c file) *************/


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
/* rest in normtools.h */



void norm(PolyhedronStruct* ph_in, NormOptions* opt, int* num_output,
          PolyhedronStruct** ph_out)
{

   int i;
   PolyhedronStruct *ph, *ph2;

   if (check_polyhedron(ph_in) == code_bad)
   {
      fprintf(stderr,"Unable to norm bone.");
      *num_output = 0;
      *ph_out = NULL;
   }

   verbose = opt->verbose_output;

   /* For testing */
   verbose = opt->verbose_output = yes;

   remove_duplicate_vertices(ph_in, opt);

   *num_output = subdivide_input_polyhedron(ph_in, ph_out);

   for (i=0; i<*num_output; i++)
   {
      ph = &(*ph_out)[i];

      remove_degenerate_polygons(ph);

      fill_holes(ph,opt->fill_holes);

      check_edge_lengths(ph,opt->max_edge_length);

      convexify_polygons(ph,opt);

      triangulate_polygons(ph,opt->triangulate);

      compress_polyhedron(ph);

      find_vertex_normals(ph,opt);

      find_bounding_cube(ph,&ph->bc);
   }

   /* If opt.write_separate_polyhedra is yes, then just return
    * because you've already got separate bones. If not, then
    * you need to merge all of the polyhedra into one.
    */
   if (opt->write_separate_polyhedra == no && *num_output > 1)
   {
      ph = combine_polyhedra(ph_out,*num_output);
      for (i=0; i<*num_output; i++)
      {
         ph2 = &(*ph_out)[i];
         free_polyhedron(ph2, no, NULL);
      }
      free(*ph_out);
      *ph_out = ph;
      *num_output = 1;
   }
}


PolyhedronStruct* combine_polyhedra(PolyhedronStruct** phs, int num_phs)
{
   int i, j, k, count;
   PolyhedronStruct *ph, *ph2;

   ph = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
   preread_init_polyhedron(ph);

   /* Copy the name and display parameters from the first polyhedron to the combined one. */
   if ((*phs)[0].name)
      mstrcpy(&ph->name, (*phs)[0].name);
   ph->drawmode = (*phs)[0].drawmode;
   ph->material = (*phs)[0].material;
   ph->show_normals = (*phs)[0].show_normals;

   for (i=0; i<num_phs; i++)
   {
      ph2 = &(*phs)[i];
      ph->num_vertices += (ph2->num_vertices - ph2->num_removed_vertices);
      ph->num_polygons += (ph2->num_polygons - ph2->num_removed_polygons);
   }

   ph->vertex = (VertexStruct*)simm_malloc(ph->num_vertices*sizeof(VertexStruct));
   ph->polygon = (PolygonStruct*)simm_malloc(ph->num_polygons*sizeof(PolygonStruct));

   for (i=0, count=0; i<num_phs; i++)
   {
      ph2 = &(*phs)[i];
      for (j=0; j<ph2->num_vertices; j++)
      {
         if (ph2->vertex[j].thrown_out == no)
         {
            copy_vertex_lite(&ph2->vertex[j],&ph->vertex[count]);
            ph2->vertex[j].new_index = count++;
         }
      }
   }

   for (i=0, count=0; i<num_phs; i++)
   {
      ph2 = &(*phs)[i];
      for (j=0; j<ph2->num_polygons; j++)
      {
         if (ph2->polygon[j].thrown_out == no)
         {
            copy_polygon_lite(&ph2->polygon[j],&ph->polygon[count]);
            for (k=0; k<ph->polygon[count].num_vertices; k++)
               ph->polygon[count].vertex_index[k] =
               ph2->vertex[ph2->polygon[j].vertex_index[k]].new_index;
            count++;
         }
      }
   }

   make_vert_poly_lists(ph);

   for (i=0; i<num_phs; i++)
   {
      ph2 = &(*phs)[i];
      if (ph2->bc.x1 < ph->bc.x1)
         ph->bc.x1 = ph2->bc.x1;
      if (ph2->bc.y1 < ph->bc.y1)
         ph->bc.y1 = ph2->bc.y1;
      if (ph2->bc.z1 < ph->bc.z1)
         ph->bc.z1 = ph2->bc.z1;
      if (ph2->bc.x2 > ph->bc.x2)
         ph->bc.x2 = ph2->bc.x2;
      if (ph2->bc.y2 > ph->bc.y2)
         ph->bc.y2 = ph2->bc.y2;
      if (ph2->bc.z2 > ph->bc.z2)
         ph->bc.z2 = ph2->bc.z2;
   }

   return ph;
}


ReturnCode check_polyhedron(PolyhedronStruct* ph)
{
   int i, j;

   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].num_vertices <= 0)
         ph->polygon[i].thrown_out = yes;
      for (j=0; j<ph->polygon[i].num_vertices; j++)
      {
          if (ph->polygon[i].vertex_index[j] >= ph->num_vertices ||
             ph->polygon[i].vertex_index[j] < 0)
          {
             fprintf(stderr,"Bad polyhedron: vertex index out of range.\n");
             return code_bad;
          }
      }
   }

   return code_fine;
}


void preread_init_polygon(PolygonStruct* p)
{
   memset(p, 0, sizeof(PolygonStruct));

   p->normal_computed = no_proc;
   p->selected = no;
#if POLYGON_DISPLAY_HACK
   p->displayed = yes;
#endif
   p->old = no;
}


void preread_init_vertex(VertexStruct* v, int index)
{
   memset(v, 0, sizeof(VertexStruct));

   v->thrown_out = no;
   v->old = no;
   v->new_index = index;
}


void preread_init_polyhedron(PolyhedronStruct* ph)
{
   memset(ph, 0, sizeof(PolyhedronStruct));

   ph->selected = no;
   ph->selected_vertex = -1;
   ph->selected_polygon = -1;
   ph->selected_edge = -1;

   ph->drawmode = gouraud_shading;
   ph->material = -1;
   ph->show_normals = no;
   ph->bc.x1 = MAXMDOUBLE;
   ph->bc.x2 = MINMDOUBLE;
   ph->bc.y1 = MAXMDOUBLE;
   ph->bc.y2 = MINMDOUBLE;
   ph->bc.z1 = MAXMDOUBLE;
   ph->bc.z2 = MINMDOUBLE;

   reset_4x4matrix(ph->transform_matrix);
   ph->tx = 0.0;
   ph->ty = 0.0;
   ph->tz = 0.0;
   ph->sx = 1.0;
   ph->sy = 1.0;
   ph->sz = 1.0;
}


/* This function assumes that the polyhedron has just been loaded from a file,
 * or is otherwise completely uninitialized. When you are done, every member
 * variable is correctly initialized except for polygon and vertex normals,
 * which cannot be calculated until you subdivide the polyhedron (the polygon
 * normals and d values are calculated, but may have the wrong sign).
 */
void postread_init_polyhedron(PolyhedronStruct* ph, SBoolean full_init)
{
   int i;

   for (i=0; i<ph->num_polygons; i++)
   {
      calc_polygon_normal(ph, &ph->polygon[i]);
      calc_polygon_bounding_cube(ph, &ph->polygon[i]);
   }

   make_vert_poly_lists(ph);

   if (full_init == yes)
      calc_polyhedron_bounding_cube(ph);
}


PolyhedronStruct* clone_polyhedron(PolyhedronStruct* from)
{
   int i, j;
   PolyhedronStruct* to;

   to = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));

   mstrcpy(&to->name,from->name);

   to->num_vertices = from->num_vertices;
   to->vertex = (VertexStruct*)simm_malloc(to->num_vertices*sizeof(VertexStruct));

   to->num_polygons = from->num_polygons;
   to->polygon = (PolygonStruct*)simm_malloc(to->num_polygons*sizeof(PolygonStruct));

   for (i=0; i<to->num_vertices; i++)
      copy_vertex(&from->vertex[i],&to->vertex[i]);

   for (i=0; i<to->num_polygons; i++)
      copy_polygon(&from->polygon[i],&to->polygon[i]);

   to->num_removed_vertices = from->num_removed_vertices;
   to->num_removed_polygons = from->num_removed_polygons;

   to->tx = from->tx;
   to->ty = from->ty;
   to->tz = from->tz;

   to->sx = from->sx;
   to->sy = from->sy;
   to->sz = from->sz;

   for (i=0; i<4; i++)
      for (j=0; j<4; j++)
     to->transform_matrix[i][j] = from->transform_matrix[i][j];

   to->selected = from->selected;
   to->selected_vertex = from->selected_vertex;
   to->selected_polygon = from->selected_polygon;
   to->selected_edge = from->selected_edge;

   to->drawmode = from->drawmode;
   to->material = from->material;
   to->show_normals = from->show_normals;

   to->bc.x1 = from->bc.x1;
   to->bc.x2 = from->bc.x2;
   to->bc.y1 = from->bc.y1;
   to->bc.y2 = from->bc.y2;
   to->bc.z1 = from->bc.z1;
   to->bc.z2 = from->bc.z2;

   return to;
}


void copy_polyhedron(PolyhedronStruct* from, PolyhedronStruct* to)
{
   int i, j;

   mstrcpy(&to->name, from->name);

   to->num_vertices = from->num_vertices;
   to->vertex = (VertexStruct*)simm_malloc(to->num_vertices*sizeof(VertexStruct));

   to->num_polygons = from->num_polygons;
   to->polygon = (PolygonStruct*)simm_malloc(to->num_polygons*sizeof(PolygonStruct));

   for (i=0; i<to->num_vertices; i++)
      copy_vertex(&from->vertex[i],&to->vertex[i]);

   for (i=0; i<to->num_polygons; i++)
      copy_polygon(&from->polygon[i],&to->polygon[i]);

   to->num_removed_vertices = from->num_removed_vertices;
   to->num_removed_polygons = from->num_removed_polygons;

   to->tx = from->tx;
   to->ty = from->ty;
   to->tz = from->tz;

   to->sx = from->sx;
   to->sy = from->sy;
   to->sz = from->sz;

   for (i=0; i<4; i++)
      for (j=0; j<4; j++)
     to->transform_matrix[i][j] = from->transform_matrix[i][j];

   to->selected = from->selected;
   to->selected_vertex = from->selected_vertex;
   to->selected_polygon = from->selected_polygon;
   to->selected_edge = from->selected_edge;
   to->drawmode = from->drawmode;
   to->material = from->material;
   to->show_normals = from->show_normals;
   to->gl_display = from->gl_display;

   to->bc.x1 = from->bc.x1;
   to->bc.x2 = from->bc.x2;
   to->bc.y1 = from->bc.y1;
   to->bc.y2 = from->bc.y2;
   to->bc.z1 = from->bc.z1;
   to->bc.z2 = from->bc.z2;
}


void copy_vertex(VertexStruct* from, VertexStruct* to)
{
   memcpy(to, from, sizeof(VertexStruct));

   if (to->polygon_count > 0)
   {
      int i;

      to->polygons = (int*)simm_malloc(to->polygon_count * sizeof(int));
      for (i=0; i<to->polygon_count; i++)
         to->polygons[i] = from->polygons[i];
   }
   else
   {
      to->polygons = NULL;
   }
}


void copy_vertex_lite(VertexStruct* from, VertexStruct* to)
{
   memcpy(to, from, sizeof(VertexStruct));
   to->polygon_count = 0;
   to->polygons = NULL;
}


void copy_pintersegmentstruct(PInterSegmentStruct* from, PInterSegmentStruct* to)
{
   to->ptseg[0][0] = from->ptseg[0][0];
   to->ptseg[0][1] = from->ptseg[0][1];
   to->ptseg[0][2] = from->ptseg[0][2];
   to->ptseg[1][0] = from->ptseg[1][0];
   to->ptseg[1][1] = from->ptseg[1][1];
   to->ptseg[1][2] = from->ptseg[1][2];
   to->vert_edge[0] = from->vert_edge[0];
   to->vert_edge[1] = from->vert_edge[1];
   to->vertex_index[0] = from->vertex_index[0];
   to->vertex_index[1] = from->vertex_index[1];
   to->vindices[0][0] = from->vindices[0][0];
   to->vindices[0][1] = from->vindices[0][1];
   to->vindices[1][0] = from->vindices[1][0];
   to->vindices[1][1] = from->vindices[1][1];
   to->poly_index = from->poly_index;
   to->marked = from->marked;
}


void copy_pintersegmentliststruct(PInterSegmentListStruct* from,
                  PInterSegmentListStruct* to)
{
   int i;

   to->num_inter_seg = from->num_inter_seg;
   if (to->num_inter_seg > 0)
   {
      to->seg = (PInterSegmentStruct*)simm_malloc(to->num_inter_seg*sizeof(PInterSegmentStruct));
      for (i=0; i<to->num_inter_seg; i++)
         copy_pintersegmentstruct(&from->seg[i],&to->seg[i]);
   }
   else
   {
      to->seg = NULL;
   }
   to->segmaxx = NULL; /* TODO: should this point to the last element? */
}


void copy_polygon(PolygonStruct* from, PolygonStruct* to)
{
   int i;

   to->num_vertices = from->num_vertices;
   to->vertex_index = (int*)simm_malloc(to->num_vertices*sizeof(int));
   for (i=0; i<to->num_vertices; i++)
      to->vertex_index[i] = from->vertex_index[i];
   to->normal[0] = from->normal[0];
   to->normal[1] = from->normal[1];
   to->normal[2] = from->normal[2];
   to->d = from->d;
   to->normal_computed = from->normal_computed;
   to->selected = from->selected;
#if POLYGON_DISPLAY_HACK
   to->displayed = from->displayed;
#endif
   to->polyhedron_number = from->polyhedron_number;
   to->thrown_out = from->thrown_out;
   to->old = from->old;
   to->ordering_value = from->ordering_value;
   to->bc.x1 = from->bc.x1;
   to->bc.x2 = from->bc.x2;
   to->bc.y1 = from->bc.y1;
   to->bc.y2 = from->bc.y2;
   to->bc.z1 = from->bc.z1;
   to->bc.z2 = from->bc.z2;
   to->boolcode.coplanar_flag = from->boolcode.coplanar_flag;
   to->boolcode.polygon_mark = from->boolcode.polygon_mark;
   to->boolcode.poly_output = from->boolcode.poly_output;
   to->boolcode.poly_adjpush = from->boolcode.poly_adjpush;
   to->boolcode.num_inters = from->boolcode.num_inters;
   for (i=0; i<to->boolcode.num_inters; i++)
      to->boolcode.inters[i] = from->boolcode.inters[i];
   copy_pintersegmentliststruct(&from->boolcode.seg_list,&to->boolcode.seg_list);
   to->boolcode.seglst_num = from->boolcode.seglst_num;
   if (to->boolcode.seglst_num > 0)
   {
      to->boolcode.seglst = (PInterSegmentListStruct*)simm_malloc(
         to->boolcode.seglst_num*sizeof(PInterSegmentListStruct));
      for (i=0; i<to->boolcode.seglst_num; i++)
         copy_pintersegmentliststruct(&from->boolcode.seglst[i],&to->boolcode.seglst[i]);
   }
   else
   {
      to->boolcode.seglst = NULL;
   }
}


void copy_polygon_lite(PolygonStruct* from, PolygonStruct* to)
{
   int i;

   to->num_vertices = from->num_vertices;
   to->vertex_index = (int*)simm_malloc(to->num_vertices*sizeof(int));
   for (i=0; i<to->num_vertices; i++)
      to->vertex_index[i] = from->vertex_index[i];
   to->normal[0] = from->normal[0];
   to->normal[1] = from->normal[1];
   to->normal[2] = from->normal[2];
   to->d = from->d;
   to->normal_computed = from->normal_computed;
   to->selected = from->selected;
#if POLYGON_DISPLAY_HACK
   to->displayed = from->displayed;
#endif
   to->polyhedron_number = from->polyhedron_number;
   to->thrown_out = from->thrown_out;
   to->old = from->old;
   to->ordering_value = from->ordering_value;

   to->bc.x1 = from->bc.x1;
   to->bc.x2 = from->bc.x2;
   to->bc.y1 = from->bc.y1;
   to->bc.y2 = from->bc.y2;
   to->bc.z1 = from->bc.z1;
   to->bc.z2 = from->bc.z2;

   to->boolcode.coplanar_flag = 0;
   to->boolcode.polygon_mark = 0;
   to->boolcode.poly_output = 0;
   to->boolcode.poly_adjpush = 0;
   to->boolcode.num_inters = 0;
   to->boolcode.seg_list.num_inter_seg = 0;
   to->boolcode.seglst_num = 0;
   to->boolcode.seglst = NULL;
//dkb sept 2009
   to->boolcode.seg_list.seg = NULL;
   to->boolcode.seg_list.segmaxx = NULL;
}


void calc_polygon_bounding_cube(PolyhedronStruct* ph, PolygonStruct* p)
{

   int i;
   VertexStruct* v;

   p->bc.x1 = MAXMDOUBLE;
   p->bc.x2 = MINMDOUBLE;
   p->bc.y1 = MAXMDOUBLE;
   p->bc.y2 = MINMDOUBLE;
   p->bc.z1 = MAXMDOUBLE;
   p->bc.z2 = MINMDOUBLE;

   for (i=0; i<p->num_vertices; i++)
   {
      v = &ph->vertex[p->vertex_index[i]];
      if (v->coord[0] < p->bc.x1)
     p->bc.x1 = v->coord[0];
      if (v->coord[0] > p->bc.x2)
     p->bc.x2 = v->coord[0];
      if (v->coord[1] < p->bc.y1)
     p->bc.y1 = v->coord[1];
      if (v->coord[1] > p->bc.y2)
     p->bc.y2 = v->coord[1];
      if (v->coord[2] < p->bc.z1)
     p->bc.z1 = v->coord[2];
      if (v->coord[2] > p->bc.z2)
     p->bc.z2 = v->coord[2];
   }

}


void calc_polyhedron_bounding_cube(PolyhedronStruct* ph)
{

   int i;

   ph->bc.x1 = MAXMDOUBLE;
   ph->bc.x2 = MINMDOUBLE;
   ph->bc.y1 = MAXMDOUBLE;
   ph->bc.y2 = MINMDOUBLE;
   ph->bc.z1 = MAXMDOUBLE;
   ph->bc.z2 = MINMDOUBLE;

   for (i=0; i<ph->num_vertices; i++)
   {
      if (ph->vertex[i].coord[0] < ph->bc.x1)
     ph->bc.x1 = ph->vertex[i].coord[0];
      if (ph->vertex[i].coord[0] > ph->bc.x2)
     ph->bc.x2 = ph->vertex[i].coord[0];

      if (ph->vertex[i].coord[1] < ph->bc.y1)
     ph->bc.y1 = ph->vertex[i].coord[1];
      if (ph->vertex[i].coord[1] > ph->bc.y2)
     ph->bc.y2 = ph->vertex[i].coord[1];

      if (ph->vertex[i].coord[2] < ph->bc.z1)
     ph->bc.z1 = ph->vertex[i].coord[2];
      if (ph->vertex[i].coord[2] > ph->bc.z2)
     ph->bc.z2 = ph->vertex[i].coord[2];
   }

}


/* This routine finds all the polygons that each vertex is a member of.
 * It's not terribly efficient right now. First it scans all the polygons
 * to find out how much space to malloc for each vertex, then it scans
 * them again to fill in the polygons[] lists.
 */

void make_vert_poly_lists(PolyhedronStruct* ph)
{
   int i, j;
   VertexStruct* v;

   for (i=0; i<ph->num_vertices; i++)
   {
      FREE_IFNOTNULL(ph->vertex[i].polygons);
      ph->vertex[i].polygon_count = 0;
   }

   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == yes)
         continue;
      for (j=0; j<ph->polygon[i].num_vertices; j++)
         ph->vertex[ph->polygon[i].vertex_index[j]].polygon_count++;
   }

   for (i=0; i<ph->num_vertices; i++)
   {
      if (ph->vertex[i].thrown_out == yes)
         continue;
      if (ph->vertex[i].polygon_count == 0)
      {
         throw_out_vertex(ph,i);
         ph->vertex[i].polygons = NULL;
      }
      else
      {
         ph->vertex[i].polygons = (int*)simm_malloc(ph->vertex[i].polygon_count*sizeof(int));
         ph->vertex[i].polygon_count = 0;
      }
   }

   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == yes)
         continue;
      for (j=0; j<ph->polygon[i].num_vertices; j++)
      {
         v = &ph->vertex[ph->polygon[i].vertex_index[j]];
         v->polygons[v->polygon_count++] = i;
      }
   }
}


/* This routine finds the other polygon (not p1) that shares the
 * edge from vertices v1 to v2.
 */

int find_other_polygon(PolyhedronStruct* ph, int p1, int v1, int v2)
{

   int i, j;
   VertexStruct *vs1, *vs2;

   vs1 = &ph->vertex[v1];
   vs2 = &ph->vertex[v2];

   for (i=0; i<vs1->polygon_count; i++)
   {
      if (vs1->polygons[i] == p1)
     continue;
      for (j=0; j<vs2->polygon_count; j++)
      {
     if ((vs1->polygons[i] == vs2->polygons[j]) &&
         (ph->polygon[vs1->polygons[i]].thrown_out == no))
        return (vs1->polygons[i]);
      }
   }

   return (-1);

}


int subdivide_input_polyhedron(PolyhedronStruct* ip, PolyhedronStruct** phs)
{
   int i, j, ph_num, polys_used_so_far, verts_used_so_far, num_polyhedra;
   int read_pos, write_pos, *poly_list;
   VertexStruct *v_new, *v_old;
   PolyhedronStruct *ph;
   PolygonStruct *p_new, *p_old;
   ReturnCode rc;

   if (verbose == yes)
   {
      printf("Separating out polyhedra... ");
      fflush(stdout);
   }

   /* So that you don't have to keep checking for thrown_out polygons,
    * go thru the polygon list once and give all thrown_out polygons a
    * polyhedron_number of SKIP_ME.
    */

   for (i=0; i<ip->num_polygons; i++)
   {
      if (ip->polygon[i].thrown_out == yes)
          ip->polygon[i].polyhedron_number = SKIP_ME;
      else
          ip->polygon[i].polyhedron_number = 0;
   }

   for (i=0; i<ip->num_vertices; i++)
      ip->vertex[i].polyhedron_number = 0;

   poly_list = (int*)simm_malloc(ip->num_polygons*sizeof(int));

   num_polyhedra = 0;
   polys_used_so_far = 0;
   verts_used_so_far = ip->num_removed_vertices;

   while (1)
   {
      /* Find a polygon whose polyhedron_number is 0 (unprocessed) */

      for (i=0; i<ip->num_polygons; i++)
      {
         if (ip->polygon[i].polyhedron_number == 0)
            break;
      }

      if (i == ip->num_polygons)
         break;

      ph_num = ++num_polyhedra;
      ip->polygon[i].polyhedron_number = ph_num;
      poly_list[0] = i;
      read_pos = 0;
      write_pos = 1;

      if (num_polyhedra == 1)
          (*phs) = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
      else
          (*phs) = (PolyhedronStruct*)simm_realloc((*phs),num_polyhedra*sizeof(PolyhedronStruct),&rc);

      ph = &(*phs)[ph_num-1];

      preread_init_polyhedron(ph);
      mstrcpy(&ph->name,ip->name);
      ph->drawmode = ip->drawmode;
      ph->material = ip->material;
      ph->show_normals = ip->show_normals;

      ph->polygon = (PolygonStruct*)simm_malloc((ip->num_polygons-polys_used_so_far)*
                        sizeof(PolygonStruct));
      ph->vertex = (VertexStruct*)simm_malloc((ip->num_vertices-verts_used_so_far)*
                        sizeof(VertexStruct));

      for (i=0; i<ip->num_polygons-polys_used_so_far; i++)
          preread_init_polygon(&ph->polygon[i]);
      for (i=0; i<ip->num_vertices-verts_used_so_far; i++)
          preread_init_vertex(&ph->vertex[i],i);

      while (read_pos < write_pos)
      {
          p_old = &ip->polygon[poly_list[read_pos++]];

          /* For each vertex in this polygon,
           *   1. copy the vertex info to the new polyhedron structure
           *   2. put all the polygons that use this vertex into poly_list.
           */
          for (i=0; i<p_old->num_vertices; i++)
          {
             v_old = &ip->vertex[p_old->vertex_index[i]];
             if (v_old->polyhedron_number != 0)
             {
                /* This vertex has already been added to the new polyhedron,
                  * so just copy its new index into the old polygon (so that
                  * it gets copied into the new polygon later).
                  */
                p_old->vertex_index[i] = v_old->new_index;
                   continue;
             }
             v_old->polyhedron_number = ph_num;
             v_new = &ph->vertex[ph->num_vertices];
             copy_vertex(v_old,v_new);
             v_new->polyhedron_number = 0;
             v_new->new_index = ph->num_vertices;
             v_old->new_index = ph->num_vertices;
             p_old->vertex_index[i] = ph->num_vertices;
             for (j=0; j<v_old->polygon_count; j++)
             {
                if (ip->polygon[v_old->polygons[j]].polyhedron_number == 0) // crash here when applying two deform objects and norming in BE
                {
                    poly_list[write_pos++] = v_old->polygons[j];
                    ip->polygon[v_old->polygons[j]].polyhedron_number = ph_num;
                }
             }
             ph->num_vertices++;
          }

          /* Now copy the polygon to the new polyhedron structure */
          p_new = &ph->polygon[ph->num_polygons++];
          copy_polygon(p_old,p_new);
          p_new->polyhedron_number = 0;
      }

      /* Now that you're done making this polyhedron, realloc the polygon and vertex arrays. */
      ph->polygon = (PolygonStruct*)simm_realloc(ph->polygon,ph->num_polygons*sizeof(PolygonStruct),&rc);
      ph->vertex = (VertexStruct*)simm_realloc(ph->vertex,ph->num_vertices*sizeof(VertexStruct),&rc);

      make_vert_poly_lists(ph);
      calc_polyhedron_bounding_cube(ph);
      ph->num_removed_vertices = 0;
      ph->num_removed_polygons = 0;

      polys_used_so_far += ph->num_polygons;
      verts_used_so_far += ph->num_vertices;
   }

   free(poly_list);

   if (verbose == yes)
      printf("Done.\n");

   return num_polyhedra;
}


SBoolean vertex_in_polygon(PolygonStruct* polygon, int v_num)
{
   int i;

   for (i=0; i<polygon->num_vertices; i++)
      if (polygon->vertex_index[i] == v_num)
         return yes;

   return no;
}


/* This function assumes that the polygon normals have already been calculated */

void find_vertex_normals(PolyhedronStruct* ph, NormOptions* opt)
{

   int i, poly_num;

   if (ph->num_polygons < 4 || ph->num_vertices < 4)
   {
      if (verbose == yes)
      {
/*
     printf("Bad polyhedron... ");
     print_polyhedron(ph,"");
*/
      }
      for (i=0; i<ph->num_vertices; i++)
      {
     if (ph->vertex[i].thrown_out == no && ph->vertex[i].polygon_count > 0)
     {
        poly_num = ph->vertex[i].polygons[0];
        ph->vertex[i].normal[0] = ph->polygon[poly_num].normal[0];
        ph->vertex[i].normal[1] = ph->polygon[poly_num].normal[1];
        ph->vertex[i].normal[2] = ph->polygon[poly_num].normal[2];
     }
     else
     {
        ph->vertex[i].normal[0] = 0.0;
        ph->vertex[i].normal[1] = 0.0;
        ph->vertex[i].normal[2] = 1.0;
     }
      }
   }

   if (opt->vertex_order == clockwise)
   {
      if (verbose == yes)
     printf("   Assuming vertices  have clockwise ordering within each polygon.\n");
      for (i=0; i<ph->num_polygons; i++)
     reverse_vertex_order(&ph->polygon[i]);
   }
   else if (opt->vertex_order == counterclockwise)
   {
      /* order is already correct */
      if (verbose == yes)
     printf("   Assuming vertices have counterclockwise ordering within each polygon.\n");
   }
   else
   {
      make_polygons_consistent(ph,opt);
   }

   /* Now that the vertex ordering is consistent for every polygon,
    * use the polygon normals to find the vertex normals.
    */

   calc_vertex_normals(ph,opt);

}


void make_polygons_consistent(PolyhedronStruct* ph, NormOptions* opt)
{

   int top_vertex, shallowest_vertex, num_tries;
   int polygon1, polygon2, poly_num;
   static double yaxis[] = {0.0,1.0,0.0};
   double angle1, angle2, b_angle1, b_angle2, ref_mag;
   double norm_edge1[3], norm_edge2[3], proj_yaxis[3];
   double proj_norm_edge1[3], proj_norm_edge2[3];
   PlaneStruct plane;
   double rot_matrix[3][3];

   num_tries = 0;
   reset_3x3matrix(rot_matrix);

   while (1)
   {
      top_vertex = find_top_vertex(ph->vertex,ph->num_vertices);

      shallowest_vertex = find_shallowest_vertex(ph,top_vertex);

      find_polygons_sharing_edge(ph,top_vertex,shallowest_vertex,
                 &polygon1,&polygon2);

      if (polygon1 != -1 && polygon2 != -1)
      {
          break;
      }
      else if (num_tries > 21)
      {
          /* You just tried to rotate the polyhedron 20 times to find
           * a convex section (two polygons sharing the shallowest edge
           * descending from the topmost vertex), and failed.
           */
          if (verbose == yes)
             printf("   Couldn't find two faces sharing shallowest edge (%d %d).\n",
                 top_vertex+1, shallowest_vertex+1);
          unorient_polyhedron(ph,rot_matrix);
          if (polygon1 == -1 && polygon2 == -1)
          {
             if (verbose == yes)
             {
                printf("   ...badly formed polyhedron.\n");
                printf("      Assuming polygons are already in counterclockwise order.\n");
                printf("      Use \"-cw\" if they are clockwise.\n");
             }
          }
          else
          {
             ref_mag = VECTOR_MAGNITUDE(opt->reference_normal);
             poly_num = (polygon1 == -1) ? polygon2 : polygon1;
             if (EQUAL_WITHIN_ERROR(ref_mag,0.0))
             {
                if (verbose == yes)
                {
                    printf("   ...assuming that polygon %d faces up (+Y).\n", poly_num);
                    printf("      use \"-rn\" to specify other direction.\n");
                }
                opt->reference_normal[0] = 0.0;
                opt->reference_normal[1] = 1.0;
                opt->reference_normal[2] = 0.0;
             }
             else
             {
                if (verbose == yes)
                    printf("   ...using reference normal to orient polygon %d.\n", poly_num);
             }
             if (DOT_VECTORS(ph->polygon[poly_num].normal,opt->reference_normal) < 0.0)
                reverse_vertex_order(&ph->polygon[poly_num]);
             order_all_vertex_lists(ph,poly_num);
          }
          return;
      }
      else
      {
          reorient_polyhedron(ph,rot_matrix);
          num_tries++;
      }
   }

   match_vertex_orders(&ph->polygon[polygon1],&ph->polygon[polygon2],
               top_vertex,shallowest_vertex);

   angle1 = compute_angle_between_vectors(ph->polygon[polygon1].normal,yaxis);
   angle2 = compute_angle_between_vectors(ph->polygon[polygon2].normal,yaxis);

   if (angle1 <= M_PI_2 && angle2 <= M_PI_2)
   {
      order_all_vertex_lists(ph,polygon1);
   }
   else if (angle1 > M_PI_2 && angle2 > M_PI_2)
   {
      reverse_vertex_order(&ph->polygon[polygon1]);
      order_all_vertex_lists(ph,polygon1);
   }
   else
   {
      find_other_edge(ph,polygon1,top_vertex,shallowest_vertex,norm_edge1);
      find_other_edge(ph,polygon2,top_vertex,shallowest_vertex,norm_edge2);
      find_plane_normal_to_line(&plane,ph->vertex[top_vertex].coord,
                ph->vertex[shallowest_vertex].coord);
      plane.d = 0.0;
      project_point_onto_plane(yaxis,&plane,proj_yaxis);
      project_point_onto_plane(norm_edge1,&plane,proj_norm_edge1);
      project_point_onto_plane(norm_edge2,&plane,proj_norm_edge2);
      b_angle1 = compute_angle_between_vectors(proj_norm_edge1,proj_yaxis);
      b_angle2 = compute_angle_between_vectors(proj_norm_edge2,proj_yaxis);
      if ((b_angle1 < b_angle2 && angle1 <= M_PI_2) ||
      (b_angle1 >= b_angle2 && angle1 > M_PI_2))
      {
     order_all_vertex_lists(ph,polygon1);
      }
      else
      {
     reverse_vertex_order(&ph->polygon[polygon1]);
     order_all_vertex_lists(ph,polygon1);
      }
   }

   /* Now that you have ordered all the polygons, rotate the vertices
    * back to where they started from.
    */

   if (num_tries > 0)
      unorient_polyhedron(ph,rot_matrix);

}


void calc_vertex_normals(PolyhedronStruct* ph, NormOptions* opt)
{

   int i, j, vertex;
   double magnitude;

   /* A vertex normal is the average of the polygon normals for all polygons
    * containing that vertex. Scan the polygon lists, and add that polygon's
    * normal to all the vertices that are in the polygon. You don't need to
    * keep track of how many you're adding since you're going to normalize
    * the result anyway. This code assumes that the polygon normals have
    * already been computed and made consistent with each other.
    */

   for (i=0; i<ph->num_vertices; i++)
   {
      ph->vertex[i].normal[0] = 0.0;
      ph->vertex[i].normal[1] = 0.0;
      ph->vertex[i].normal[2] = 0.0;
   }

   for (i=0; i<ph->num_polygons; i++)
   {
      for (j=0; j<ph->polygon[i].num_vertices; j++)
      {
     vertex = ph->polygon[i].vertex_index[j];
     ph->vertex[vertex].normal[0] += ph->polygon[i].normal[0];
     ph->vertex[vertex].normal[1] += ph->polygon[i].normal[1];
     ph->vertex[vertex].normal[2] += ph->polygon[i].normal[2];
      }
   }

   for (i=0; i<ph->num_vertices; i++)
   {
      magnitude = normalize_vector(ph->vertex[i].normal,ph->vertex[i].normal);
      if (EQUAL_WITHIN_ERROR(magnitude,0.0))
      {
     ph->vertex[i].normal[0] = opt->reference_normal[0];
     ph->vertex[i].normal[1] = opt->reference_normal[1];
     ph->vertex[i].normal[2] = opt->reference_normal[2];
      }
   }

}


void order_all_vertex_lists(PolyhedronStruct* ph, int polygon)
{

   int i, start_polygon, vertex2, p2;
   PolygonStruct* poly;
   VertexOrder order;

   ph->polygon[polygon].normal_computed = yes_proc;

   while (1)
   {
      for (i=0; i<ph->num_polygons; i++)
      {
         if (ph->polygon[i].normal_computed == yes_proc)
     {
        start_polygon = i;
        break;
     }
      }

      if (i == ph->num_polygons)
         break;

      poly = &ph->polygon[start_polygon];
      for (i=0; i<poly->num_vertices; i++)
      {
     if (i == poly->num_vertices-1)
        vertex2 = poly->vertex_index[0];
     else
        vertex2 = poly->vertex_index[i+1];
     p2 = find_other_polygon(ph,start_polygon,
                 poly->vertex_index[i],vertex2);
     if (p2 != -1 && ph->polygon[p2].normal_computed == no_proc)
     {
        order = polygon_contains_edge(&ph->polygon[p2],poly->vertex_index[i],vertex2);
        if (order == clockwise)
           reverse_vertex_order(&ph->polygon[p2]);
        ph->polygon[p2].normal_computed = yes_proc;
     }
      }
      poly->normal_computed = yes_and_propagated;
   }

}


void calc_polygon_normal(PolyhedronStruct* ph, PolygonStruct* p)
{
   int i, v1, v2, v3, nv;
   double vec1[3], vec2[3];
   double mag;

   // Find the first three vertices which are not collinear,
   // and use them to find the surface normal.
   nv = p->num_vertices;

   for (i=0; i<nv; i++)
   {
      v1 = p->vertex_index[i];
      v2 = p->vertex_index[(i+1)%nv];
      v3 = p->vertex_index[(i+2)%nv];

      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v3].coord,vec1);
      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v1].coord,vec2);

      normalize_vector(vec1,vec1);
      normalize_vector(vec2,vec2);
      cross_vectors(vec1,vec2,p->normal);
      mag = normalize_vector(p->normal,p->normal);
      if (mag >= COLLINEAR_EPSILON)
         break;
   }

   if (i == nv)
   {
      p->normal[0] = 1.0;
      p->normal[1] = 0.0;
      p->normal[2] = 0.0;
      p->d = 0.0;
   }
   else
   {
      p->d = - ph->vertex[v1].coord[0] * p->normal[0] -
         ph->vertex[v1].coord[1] * p->normal[1] -
         ph->vertex[v1].coord[2] * p->normal[2];
   }
}


/* This routine makes consistent (matches) the vertex orders of the two polygons
 * which share the edge between vertex1 and vertex2.
 */

void match_vertex_orders(PolygonStruct* polygon1, PolygonStruct* polygon2,
             int vertex1, int vertex2)
{

   int order1, order2;

   order1 = polygon_contains_edge(polygon1,vertex1,vertex2);
   order2 = polygon_contains_edge(polygon2,vertex2,vertex1);

   if (order1 != order2)
      reverse_vertex_order(polygon2);

}


void reverse_vertex_order(PolygonStruct* p)
{

   int i, j, tmpvertex;

   for (i=0, j=p->num_vertices-1; i<j; i++, j--)
   {
      tmpvertex = p->vertex_index[i];
      p->vertex_index[i] = p->vertex_index[j];
      p->vertex_index[j] = tmpvertex;
   }

   /* Assuming that the polygon's normal has been computed, it needs to
    * be reversed as well.
    */

   p->normal[0] = -p->normal[0];
   p->normal[1] = -p->normal[1];
   p->normal[2] = -p->normal[2];
   p->d = -p->d;

}


void find_polygons_sharing_edge(PolyhedronStruct* ph, int vertex1, int vertex2,
                int* polygon1, int* polygon2)
{

   int i, j, polygons_found_so_far = 0;

   *polygon1 = *polygon2 = -1;

   for (i=0; i<ph->vertex[vertex1].polygon_count; i++)
   {
      for (j=0; j<ph->vertex[vertex2].polygon_count; j++)
      {
     if (ph->vertex[vertex1].polygons[i] == ph->vertex[vertex2].polygons[j])
     {
        if (polygons_found_so_far == 0)
        {
           *polygon1 = ph->vertex[vertex1].polygons[i];
           polygons_found_so_far = 1;
        }
        else
        {
           *polygon2 = ph->vertex[vertex1].polygons[i];
           return;
        }
     }
      }
   }

}


VertexOrder polygon_contains_edge(PolygonStruct* p, int vertex1, int vertex2)
{
   int i;

   for (i=0; i<p->num_vertices; i++)
      if (p->vertex_index[i] == vertex1)
         break;

   if (i == p->num_vertices)
      return unspecified_order;

   if (i == 0)
   {
      if (p->vertex_index[1] == vertex2)
         return clockwise;
      if (p->vertex_index[p->num_vertices-1] == vertex2)
         return counterclockwise;
      return unspecified_order;
   }

   if (i == p->num_vertices-1)
   {
      if (p->vertex_index[0] == vertex2)
     return clockwise;
      if (p->vertex_index[i-1] == vertex2)
     return counterclockwise;
      return unspecified_order;
   }

   if (p->vertex_index[i+1] == vertex2)
      return clockwise;
   if (p->vertex_index[i-1] == vertex2)
      return counterclockwise;
   return unspecified_order;
}


int find_top_vertex(VertexStruct* vertex, int num_vertices)
{
   int i, index_of_top_vertex;
   double max_y_value;

   max_y_value = vertex[0].coord[YY];
   index_of_top_vertex = 0;

   for (i=1; i<num_vertices; i++)
   {
      if (vertex[i].thrown_out == yes)
         continue;
      if (vertex[i].coord[YY] > max_y_value)
      {
         max_y_value = vertex[i].coord[YY];
         index_of_top_vertex = i;
      }
   }

   return index_of_top_vertex;
}


int find_shallowest_vertex(PolyhedronStruct* ph, int top_vertex)
{
   int i, shallowest_vertex = 0, previous_vertex, next_vertex;
   double x_length, y_length, z_length, y_slope, min_slope = MAXMDOUBLE;

   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == yes)
         continue;
      if (get_adjacent_vertices(&ph->polygon[i], top_vertex, &previous_vertex, &next_vertex) == found_none)
         continue;

      x_length = ph->vertex[top_vertex].coord[XX] - ph->vertex[previous_vertex].coord[XX];
      y_length = ph->vertex[top_vertex].coord[YY] - ph->vertex[previous_vertex].coord[YY];
      z_length = ph->vertex[top_vertex].coord[ZZ] - ph->vertex[previous_vertex].coord[ZZ];

      y_slope = y_length/sqrt((x_length)*(x_length)+(z_length)*(z_length));
      if (y_slope < min_slope)
      {
         min_slope = y_slope;
         shallowest_vertex = previous_vertex;
      }

      x_length = ph->vertex[top_vertex].coord[XX] - ph->vertex[next_vertex].coord[XX];
      y_length = ph->vertex[top_vertex].coord[YY] - ph->vertex[next_vertex].coord[YY];
      z_length = ph->vertex[top_vertex].coord[ZZ] - ph->vertex[next_vertex].coord[ZZ];

      y_slope = y_length / sqrt((x_length)*(x_length) + (z_length)*(z_length));
      if (y_slope < min_slope)
      {
         min_slope = y_slope;
         shallowest_vertex = next_vertex;
      }
   }

   return shallowest_vertex;
}


VerticesFound get_adjacent_vertices(PolygonStruct* p, int seed_vertex, int* previous_vertex, int* next_vertex)
{
   int i;

   for (i=0; i<p->num_vertices; i++)
      if (p->vertex_index[i] == seed_vertex)
     break;

   if (i == p->num_vertices)
      return (found_none);

   if (i == 0)
      *previous_vertex = p->vertex_index[p->num_vertices-1];
   else
      *previous_vertex = p->vertex_index[i-1];

   *next_vertex = p->vertex_index[(i+1)%p->num_vertices];

   return (found_some);

}


void find_other_edge(PolyhedronStruct* ph, int poly, int start_vertex,
             int adj_vertex, double other_edge[])
{

   int i, v1, v2, other_vertex, first_vertex, second_vertex;
   double the_edge[3];
   PolygonStruct* p;

   p = &ph->polygon[poly];

   for (i=0; i<p->num_vertices; i++)
      if (p->vertex_index[i] == start_vertex)
     break;

   if (i == p->num_vertices)
      return;

   if (i == 0)
   {
      v1 = 1;
      v2 = p->num_vertices - 1;
   }
   else if (i == p->num_vertices - 1)
   {
      v1 = 0;
      v2 = i - 1;
   }
   else
   {
      v1 = i + 1;
      v2 = i - 1;
   }

   /* v1 and v2 are the vertices on either side of start_vertex. Find the one that
    * is not equal to adj_vertex and set other_vertex equal to it.
    */

   if (p->vertex_index[v1] == adj_vertex)
      other_vertex = v2;
   else
      other_vertex = v1;

   /* The edge will be from start_vertex (i) to other_vertex. */

   first_vertex = p->vertex_index[other_vertex];
   second_vertex = p->vertex_index[i];

   /* Now subtract second from first to form the edge */

   the_edge[XX] = ph->vertex[first_vertex].coord[XX] -
      ph->vertex[second_vertex].coord[XX];
   the_edge[YY] = ph->vertex[first_vertex].coord[YY] -
      ph->vertex[second_vertex].coord[YY];
   the_edge[ZZ] = ph->vertex[first_vertex].coord[ZZ] -
      ph->vertex[second_vertex].coord[ZZ];

   normalize_vector(the_edge,other_edge);

}


int compare_dist(PListStruct* item1, PListStruct* item2)
{

   if (item1->distance < item2->distance)
      return (-1);

   if (item1->distance > item2->distance)
      return (1);

   return (0);

}


void find_zero_area_polygons(PolyhedronStruct* ph)
{

   int i, v1, v2, v3;
   double vec1[3], vec2[3], vec3[3], cross_vec[3];
   double mag1, mag2, edge_length[3];

   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == yes)
     continue;

      if (ph->polygon[i].num_vertices > 3)
      {
     find_zero_area_nontriangle(ph,i);
     continue;
      }

      v1 = ph->polygon[i].vertex_index[0];
      v2 = ph->polygon[i].vertex_index[1];
      v3 = ph->polygon[i].vertex_index[2];

      MAKE_3DVECTOR(ph->vertex[v1].coord,ph->vertex[v2].coord,vec1);
      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v3].coord,vec2);
      MAKE_3DVECTOR(ph->vertex[v3].coord,ph->vertex[v1].coord,vec3);

      edge_length[0] = normalize_vector(vec1,vec1);
      edge_length[1] = normalize_vector(vec2,vec2);
      edge_length[2] = normalize_vector(vec3,vec3);

      /* If two edges are collinear, then all three are. */
      cross_vectors(vec1,vec2,cross_vec);
      mag1 = normalize_vector(cross_vec,cross_vec);
      if ((mag1 > COLLINEAR_EPSILON) && (mag1 < 2.0 - COLLINEAR_EPSILON))
     continue;

      cross_vectors(vec2,vec3,cross_vec);
      mag2 = normalize_vector(cross_vec,cross_vec);
      if ((mag2 > COLLINEAR_EPSILON) && (mag2 < 2.0 - COLLINEAR_EPSILON))
     continue;
/*
      printf("polygon[%d] (%d %d %d) has zero area (%.6lf, %.6lf) (old=%d)\n",
         i, ph->polygon[i].vertex_index[0],
         ph->polygon[i].vertex_index[1], ph->polygon[i].vertex_index[2], mag1,
         mag2, ph->polygon[i].old);
*/
   }

}


void remove_zero_area_polygons(PolyhedronStruct* ph)
{

   int i, v1, v2, v3, e1, e2, v, p;
   double vec1[3], vec2[3], vec3[3], cross_vec[3];
   double mag, edge_length[3];

   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == yes)
     continue;

      if (ph->polygon[i].num_vertices > 3)
      {
     remove_zero_area_nontriangle(ph,i);
     continue;
      }

      v1 = ph->polygon[i].vertex_index[0];
      v2 = ph->polygon[i].vertex_index[1];
      v3 = ph->polygon[i].vertex_index[2];

      MAKE_3DVECTOR(ph->vertex[v1].coord,ph->vertex[v2].coord,vec1);
      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v3].coord,vec2);
      MAKE_3DVECTOR(ph->vertex[v3].coord,ph->vertex[v1].coord,vec3);

      edge_length[0] = normalize_vector(vec1,vec1);
      edge_length[1] = normalize_vector(vec2,vec2);
      edge_length[2] = normalize_vector(vec3,vec3);

      /* If two edges are collinear, then all three are. */
      cross_vectors(vec1,vec2,cross_vec);
      mag = normalize_vector(cross_vec,cross_vec);
      if ((mag > COLLINEAR_EPSILON) && (mag < 2.0 - COLLINEAR_EPSILON))
     continue;

      cross_vectors(vec2,vec3,cross_vec);
      mag = normalize_vector(cross_vec,cross_vec);
      if ((mag > COLLINEAR_EPSILON) && (mag < 2.0 - COLLINEAR_EPSILON))
     continue;

      /* If you make it to here, the triangle has [near]-zero area, so
       * find the vertex that is between the other two.
       */
      if (edge_length[0] > edge_length[1])
      {
     if (edge_length[0] > edge_length[2])
     {
        v = v3;
        e1 = v1;
        e2 = v2;
     }
     else
     {
        v = v2;
        e1 = v1;
        e2 = v3;
     }
      }
      else
      {
     if (edge_length[1] > edge_length[2])
     {
        v = v1;
        e1 = v2;
        e2 = v3;
     }
     else
     {
        v = v2;
        e1 = v1;
        e2 = v3;
     }
      }

      p = find_other_polygon(ph,i,e1,e2);
      if (p == -1)
      {
     printf("zero area: no other polygon (%d) with %d %d\n", i, e1, e2);
      }
      else
      {
     add_vert_to_poly(ph,p,e1,e2,v);
      }
      throw_out_polygon(ph,i);
   }

}


void find_zero_area_nontriangle(PolyhedronStruct* ph, int pl)
{

   int i, v1, v2, v3;
   double vec1[3], vec2[3];
   double *v_angle;
   PolygonStruct* ps;

   ps = &ph->polygon[pl];
   v_angle = (double*)simm_malloc(ps->num_vertices*sizeof(double));

   for (i=0; i<ps->num_vertices; i++)
   {
      v1 = ps->vertex_index[(i+ps->num_vertices-1)%ps->num_vertices];
      v2 = ps->vertex_index[i];
      v3 = ps->vertex_index[(i+1)%ps->num_vertices];

      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v1].coord,vec1);
      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v3].coord,vec2);

      v_angle[i] = compute_angle_between_vectors(vec1,vec2);

      if (v_angle[i] > ANGLE_EPSILON && v_angle[i] < M_PI - ANGLE_EPSILON)
      {
     free(v_angle);
     return;
      }
   }
/*
   printf("polygon[%d] has zero area ", pl);
   for (i=0; i<ps->num_vertices; i++)
      printf("%d ", ps->vertex_index[i]);
   printf("angles: ");
   for (i=0; i<ps->num_vertices; i++)
      printf("%5.1lf ", v_angle[i]*RTOD);
   printf("(old=%d)\n", ps->old);
*/
}


void remove_zero_area_nontriangle(PolyhedronStruct* ph, int pl)
{

   int i, j, count, index, e1, e2, v1, v2, v3, p, edge[2];
   double vec1[3], vec2[3];
   double *v_angle;
   PolygonStruct* ps;
   PListStruct* plist;

   ps = &ph->polygon[pl];
   v_angle = (double*)simm_malloc(ps->num_vertices*sizeof(double));

   for (i=0; i<ps->num_vertices; i++)
   {
      v1 = ps->vertex_index[(i+ps->num_vertices-1)%ps->num_vertices];
      v2 = ps->vertex_index[i];
      v3 = ps->vertex_index[(i+1)%ps->num_vertices];

      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v1].coord,vec1);
      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v3].coord,vec2);

      v_angle[i] = compute_angle_between_vectors(vec1,vec2);

      if (v_angle[i] > ANGLE_EPSILON && v_angle[i] < M_PI - ANGLE_EPSILON)
      {
     free(v_angle);
     return;
      }
   }
/*
   printf("polygon[%d] has zero area ", pl);
   for (i=0; i<ps->num_vertices; i++)
      printf("%d ", ps->vertex_index[i]);
   printf("angles: ");
   for (i=0; i<ps->num_vertices; i++)
      printf("%5.1lf ", v_angle[i]*RTOD);
   printf("(old=%d)\n", ps->old);
*/
   /* If you make it to here, the polygon must have [near]-zero area because
    * all of its internal angles are either 0.0 or 180.0. So now you want to
    * find the two vertices with an angle of 0.0, and sort the other vertices
    * between these two.
    */

   for (i=0, count=0; i<ps->num_vertices; i++)
   {
      if (ANGLES_APX_EQ(v_angle[i],0.0))
      {
     if (count > 1)
        printf("Too many 0.0 angles in polygon %d\n", pl);
     else
        edge[count++] = i;
      }
   }

   if (count < 2)
   {
      printf("Could not find two 0.0 angles in polygon %d\n", pl);
      free(v_angle);
      return;
   }

   plist = (PListStruct*)simm_malloc(ps->num_vertices*sizeof(PListStruct));

   plist[0].v_num = ps->vertex_index[edge[0]];
   plist[0].distance = 0.0;
   plist[0].side = 2;

   plist[ps->num_vertices-1].v_num = ps->vertex_index[edge[1]];
   plist[ps->num_vertices-1].distance = MAXMDOUBLE;
   plist[ps->num_vertices-1].side = 2;

   /* plist[0] and plist[nv-1] define the min and max points of the collinear
    * vertices. Move along one side of the polygon (side 0) from plist[0] to
    * plist[nv-1] and fill in the plist structure. Then you will move along the
    * other side (side 1) and fill in the rest of the plist structures, keeping
    * them sorted by distance from plist[0].
    */

   index = 1;
   for (i=edge[0]+1; i<edge[1]; i++)
   {
      plist[index].v_num = ps->vertex_index[i];
      plist[index].distance =
     distancesqr_between_vertices(ph->vertex[plist[0].v_num].coord,
                      ph->vertex[plist[index].v_num].coord);
      plist[index++].side = 0;
   }

   for (i=edge[1]+1; i<ps->num_vertices; i++)
   {
      plist[index].v_num = ps->vertex_index[i];
      plist[index].distance =
     distancesqr_between_vertices(ph->vertex[plist[0].v_num].coord,
                      ph->vertex[plist[index].v_num].coord);
      plist[index++].side = 1;
   }

   for (i=0; i<edge[0]; i++)
   {
      plist[index].v_num = ps->vertex_index[i];
      plist[index].distance =
     distancesqr_between_vertices(ph->vertex[plist[0].v_num].coord,
                      ph->vertex[plist[index].v_num].coord);
      plist[index++].side = 1;
   }

   qsort(plist,ps->num_vertices,sizeof(PListStruct),
     (int(*)(const void*,const void*))compare_dist);
/*
   for (i=0; i<ps->num_vertices; i++)
      printf("plist[%d] = %d, %lf, %d\n", i, plist[i].v_num,
         plist[i].distance, plist[i].side);
*/
   /* Now that you have a sorted list of vertices between plist[0] and plist[nv-1],
    * use them to add vertices to the polygons that border this zero-area one.
    * First scan the array looking for side1 vertices between side0 vertices, and
    * then scan it again looking for side0 vertices between side1 vertices.
    */

   for (i=1; i<ps->num_vertices-1; i++)
   {
      if (plist[i].side != 1)
     continue;
      e1 = plist[i-1].v_num;
      /* Find the first vertex after this one (plist[i]) not on side1 */
      for (j=i+1; j<ps->num_vertices; j++)
     if (plist[j].side != 1)
        break;
      e2 = plist[j].v_num;
      p = find_other_polygon(ph,pl,e1,e2);
      if (p == -1)
     printf("zero area: no other polygon (%d) with %d %d\n", pl, e1, e2);
      else
      {
/*
     printf("adding %d between %d and %d in polygon %d\n", plist[i].v_num,
        e1, e2, p);
*/
     add_vert_to_poly(ph,p,e1,e2,plist[i].v_num);
      }
   }

   for (i=1; i<ps->num_vertices-1; i++)
   {
      if (plist[i].side != 0)
     continue;
      e1 = plist[i-1].v_num;
      /* Find the first vertex after this one (plist[i]) not on side0 */
      for (j=i+1; j<ps->num_vertices; j++)
     if (plist[j].side != 0)
        break;
      e2 = plist[j].v_num;
      p = find_other_polygon(ph,pl,e1,e2);
      if (p == -1)
     printf("zero area: no other polygon (%d) with %d %d\n", pl, e1, e2);
      else
      {
/*
     printf("adding %d between %d and %d in polygon %d\n", plist[i].v_num,
        e1, e2, p);
*/
     add_vert_to_poly(ph,p,e1,e2,plist[i].v_num);
      }
   }

   throw_out_polygon(ph,pl);

   free(plist);
   free(v_angle);

}


void remove_duplicate_vertices(PolyhedronStruct* ph, NormOptions* opt)
{
   int i, j, k, vertices_removed;
   double distance;

   if (EQUAL_WITHIN_ERROR(0.0,opt->vertex_tolerance))
      return;

   if (opt->verbose_output == yes)
   {
      printf("Removing duplicate vertices (tol = %.10lf)\n", opt->vertex_tolerance);
      fflush(stdout);
   }

   for (i=0; i<ph->num_vertices; i++)
      ph->vertex[i].new_index = i;

   distance = opt->vertex_tolerance * opt->vertex_tolerance;

   for (i=0, vertices_removed=0; i<ph->num_vertices; i++)
   {
      if (ph->vertex[i].thrown_out == yes)
         continue;
      for (j=i+1; j<ph->num_vertices; j++)
      {
         if (ph->vertex[j].thrown_out == yes)
            continue;
         if (distancesqr_between_vertices(ph->vertex[i].coord, ph->vertex[j].coord) < distance)
         {
            // In the polygons that use vertex j, replace j with i.
            // TODO: for now, assume that the normals of these polygons
            // do not change much, so don't recalculate them. Also,
            // assume that vertex[i].normal does not change much.
            for (k=0; k<ph->vertex[j].polygon_count; k++)
               replace_vertex_index(&ph->polygon[ph->vertex[j].polygons[k]], j, i);
            // Now add all of these polygons to the poly_list for vertex i.
            add_polygons_to_vert_poly_list(&ph->vertex[i], ph->vertex[j].polygons, ph->vertex[j].polygon_count);
            throw_out_vertex(ph, j);
            vertices_removed++;
         }
      }
   }

   if (opt->verbose_output == yes)
      printf("   %d duplicate vertices removed.\n", vertices_removed);
}


void add_polygons_to_vert_poly_list(VertexStruct* v, int* polygons, int polygon_count)
{
   int i, j, max_polys;
   ReturnCode rc;

   if (v->thrown_out == yes)
      printf("OOPS: adding to v[deleted].polygons\n");

   max_polys = v->polygon_count + polygon_count;

   v->polygons = (int*)simm_realloc(v->polygons,max_polys*sizeof(int),&rc);

   for (i=0; i<polygon_count; i++)
   {
      for (j=0; j<v->polygon_count; j++)
      {
         if (v->polygons[j] == polygons[i])
            break;
      }
      if (j == v->polygon_count)
         v->polygons[v->polygon_count++] = polygons[i];
   }

   if (v->polygon_count < max_polys)
      v->polygons = (int*)simm_realloc(v->polygons,v->polygon_count*sizeof(int),&rc);
}


void add_polygon_to_vert_poly_list(VertexStruct* v, int poly_num)
{
   int i;
   ReturnCode rc;

   for (i=0; i<v->polygon_count; i++)
      if (v->polygons[i] == poly_num)
         return;

   if (v->thrown_out == yes)
      printf("OOPS: adding to v[deleted].polygons\n");

   v->polygon_count++;

   if (v->polygon_count == 1)
      v->polygons = (int*)simm_malloc(sizeof(int));
   else
      v->polygons = (int*)simm_realloc(v->polygons,v->polygon_count*sizeof(int),&rc);

   v->polygons[v->polygon_count-1] = poly_num;
}


void remove_polygon_from_vert_poly_list(VertexStruct* v, int poly_num)
{

   int i, index;

   for (i=0; i<v->polygon_count; i++)
      if (v->polygons[i] == poly_num)
     break;

   if (i == v->polygon_count)
      return;

   index = i;

   for (i=index; i<v->polygon_count-1; i++)
      v->polygons[i] = v->polygons[i+1];

   v->polygon_count--;

   /* TODO: you might want to realloc the array here */

}


void replace_vertex_index(PolygonStruct* p, int old_index, int new_index)
{

   int i;

   for (i=0; i<p->num_vertices; i++)
   {
      if (p->vertex_index[i] == old_index)
     p->vertex_index[i] = new_index;
   }

}


void remove_degenerate_polygons(PolyhedronStruct* ph)
{

   int i, j, k, num_removed=0;

   /* Remove all degenerate polygons. A degenerate polygon is one
    * which has identical indices for two or more of its vertices
    * (e.g., a polygon with a vertex list of: 2 6 7 6). This is a really
    * hard problem for polygons with 4 or more vertices, because you need
    * to break the polygon into two or more pieces, then remove the degenerate
    * pieces. For now, just remove the duplicate indices in the polygon, and
    * if there are less than 3 unique indices left, throw out the polygon
    * (hoping that the hole will get filled in later).
    */

    for (i=0; i<ph->num_polygons; i++)
    {
        if (ph->polygon[i].thrown_out == yes || ph->polygon[i].old == yes)
            continue;
        for (j=0; j<ph->polygon[i].num_vertices; j++)
        {
            for (k=j+1; k<ph->polygon[i].num_vertices; k++)
            {
                if (ph->polygon[i].vertex_index[j] == ph->polygon[i].vertex_index[k])
                {
                    /*
                    printf("polygon %d is degenerate ", i);
                    for (m=0; m<ph->polygon[i].num_vertices; m++)
                    printf("%d ", ph->polygon[i].vertex_index[m]);
                    printf("\n");
                    */
                    if ((ph->polygon[i].num_vertices < 4) || (fix_polygon(ph,i) == -1))
                    {
                        throw_out_polygon(ph,i);
                        num_removed++;
                    }
                    /* major hack (setting j) to break out of both loops */
                    j = ph->polygon[i].num_vertices;
                    break;
                }
            }
        }
    }

   if (verbose == yes)
      printf("   %d degenerate polygons removed.\n", num_removed);

}


void remove_overlapping_polygons(PolyhedronStruct* ph)
{
   int i, j, num_removed = 0;

   /* Remove all overlapping polygons. A polygon is overlapping if
     * its vertex list is identical to some other polygon's list
     * (if the order of the vertices is different the polygon is
     * still overlapping).
    */

    for (i = 0; i < ph->num_polygons; i++)
    {
        if (ph->polygon[i].thrown_out == yes || ph->polygon[i].old == yes)
            continue;

        for (j = i+1; j < ph->num_polygons; j++)
        {
            if (ph->polygon[j].thrown_out == yes || ph->polygon[j].old == yes)
                continue;

            if (vertex_lists_identical(ph, i, j))
            {
                throw_out_polygon(ph, j);
                num_removed++;
            }
        }
    }

   if (verbose == yes)
      printf("   %d overlapping polygons removed.\n", num_removed);
}


int vertex_lists_identical(PolyhedronStruct* ph, int poly1, int poly2)
{
    int i, j;

    if (ph->polygon[poly1].num_vertices != ph->polygon[poly2].num_vertices)
        return 0;

    for (i = 0; i < ph->polygon[poly1].num_vertices; i++)
    {
        for (j = 0; j < ph->polygon[poly2].num_vertices; j++)
        {
            if (ph->polygon[poly1].vertex_index[i] == ph->polygon[poly2].vertex_index[j])
                break;
        }
        if (j == ph->polygon[poly2].num_vertices)
            return 0;
    }

    return 1;
}


void remove_extra_polygons(PolyhedronStruct* ph)
{
   int i, j, k, num_removed = 0;
    int v1, v2, num_brothers, num_triplets;

   /* Remove all extra polygons. A polygon is extra if two or
     * more of its edges are shared by 2 or more other polygons.
    */

    for (i = 0; i < ph->num_polygons; i++)
    {
        if (ph->polygon[i].thrown_out == yes || ph->polygon[i].old == yes)
            continue;

        num_triplets = 0;

        if (i == 838)
        {
            num_triplets = 0;
        }

        for (j = 0; j < ph->polygon[i].num_vertices; j++)
        {
            v1 = ph->polygon[i].vertex_index[j];
            v2 = ph->polygon[i].vertex_index[(j+1) % ph->polygon[i].num_vertices];
            num_brothers = 0;

            for (k = 0; k < ph->num_polygons; k++)
            {
                if (k != i && ph->polygon[k].thrown_out == no && ph->polygon[k].old == no)
                {
                    if (polygon_contains_edge(&ph->polygon[k], v1, v2) > 0)
                    {
                        if (++num_brothers > 1)
                            break;
                    }
                }
            }

            if (num_brothers > 1)
                num_triplets++;

            //printf("p %d e %d: brothers = %d\n", i, j, num_brothers);
        }

        if (num_triplets > 1)
        {
            throw_out_polygon(ph, i);
            num_removed++;
        }
    }

   if (verbose == yes)
      printf("   %d extra polygons removed.\n", num_removed);
}


/* Remove all duplicates from the vertex_index list. If there
 * are 3 or more vertices left standing, save the fixed polygon.
 */

int fix_polygon(PolyhedronStruct* ph, int pl)
{

   int i, j, count=0;
   int* vlist;
   PolygonStruct* ps;

   ps = &ph->polygon[pl];

   vlist = (int*)simm_malloc(ps->num_vertices*sizeof(int));

   for (i=0; i<ps->num_vertices; i++)
   {
      for (j=0; j<count; j++)
     if (ps->vertex_index[i] == vlist[j])
        break;
      if (j == count)
     vlist[count++] = ps->vertex_index[i];
   }

   /* If there are less than 3 vertices, the attempted fix failed */
   if (count < 3)
      return (-1);

   ps->vertex_index = vlist;
   ps->num_vertices = count;
/*
   printf("fix_polygon succeeded: ");
   for (i=0; i<count; i++)
      printf("%d ", vlist[i]);
   printf("\n");
*/
   return (1);

}


void order_polygons(PolyhedronStruct* polyhedron, int order_format)
{

   int i, j, axis, index_of_min;
   double min_so_far;

   if (order_format == no_ordering)
      return;

   axis = order_format;

   if (verbose == yes)
      printf("Ordering polygons with respect to the %s axis... ", axis_names[axis]);

   /* First order the vertices. This is not necessary, but it causes
    * the vertices in each polygon to be closer in number.
    */

   order_vertices(polyhedron,order_format);

   /* First find the average value of the vertices, with respect
    * to the axis used for ordering.
    */

   for (i=0; i<polyhedron->num_polygons; i++)
   {
      if (polyhedron->polygon[i].thrown_out == yes)
     continue;
      polyhedron->polygon[i].ordering_value = 0.0;
      for (j=0; j<polyhedron->polygon[i].num_vertices; j++)
     polyhedron->polygon[i].ordering_value +=
        polyhedron->vertex[polyhedron->polygon[i].vertex_index[j]].coord[axis];
      polyhedron->polygon[i].ordering_value /= polyhedron->polygon[i].num_vertices;
   }

   for (i=0; i<polyhedron->num_polygons; i++)
   {
      if (polyhedron->polygon[i].thrown_out == yes)
     continue;
      min_so_far = polyhedron->polygon[i].ordering_value;
      index_of_min = i;
      for (j=i+1; j<polyhedron->num_polygons; j++)
      {
     if (polyhedron->polygon[j].thrown_out == yes)
        continue;
     if (polyhedron->polygon[j].ordering_value < min_so_far)
     {
        min_so_far = polyhedron->polygon[j].ordering_value;
        index_of_min = j;
     }
      }
      if (index_of_min != i)
     swap_polygons(polyhedron,i,index_of_min);
   }

   if (verbose == yes)
      printf("Done.\n");

}


void swap_polygons(PolyhedronStruct* ph, int index1, int index2)
{

   PolygonStruct p;

   copy_polygon(&ph->polygon[index1],&p);
   copy_polygon(&ph->polygon[index2],&ph->polygon[index1]);
   copy_polygon(&p,&ph->polygon[index2]);

}


void order_vertices(PolyhedronStruct* polyhedron, int order_format)
{

   int i, j, axis, index_of_min;
   double min_so_far;

   if (order_format == no_ordering)
      return;

   axis = order_format;

   for (i=0; i<polyhedron->num_vertices; i++)
   {
      if (polyhedron->vertex[i].thrown_out == yes)
     continue;
      min_so_far = polyhedron->vertex[i].coord[axis];
      index_of_min = i;
      for (j=i+1; j<polyhedron->num_vertices; j++)
      {
     if (polyhedron->vertex[j].coord[axis] < min_so_far)
     {
        min_so_far = polyhedron->vertex[j].coord[axis];
        index_of_min = j;
     }
      }
      if (index_of_min != i)
     swap_vertex_indices(polyhedron,i,index_of_min);
   }

}


void swap_vertex_indices(PolyhedronStruct* ph, int index1, int index2)
{

   int i, j;
   VertexStruct v;

   copy_vertex(&ph->vertex[index1],&v);
   copy_vertex(&ph->vertex[index2],&ph->vertex[index1]);
   copy_vertex(&v,&ph->vertex[index2]);

   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == yes)
     continue;
      for (j=0; j<ph->polygon[i].num_vertices; j++)
      {
     if (ph->polygon[i].vertex_index[j] == index1)
        ph->polygon[i].vertex_index[j] = index2;
     else if (ph->polygon[i].vertex_index[j] == index2)
        ph->polygon[i].vertex_index[j] = index1;
      }
   }

}


/* This function finds all of the edges that appear in just one polygon, and attempts
 * to string them together to form hole-filling polygons. It assumes that there are
 * not more that MAX_HOLE_EDGES loose edges, and not more that MAX_POLY_EDGES edges
 * in any one hole.
 */

void fill_holes(PolyhedronStruct* ph, SBoolean fill_holes)
{

   int i, j, v1, v2, nv, count=0, num_edges, start_v, other_end;
   int edgelist[MAX_HOLE_EDGES][2], edgeused[MAX_HOLE_EDGES], num_holes_filled=0;
   int new_p, e_num, v_num, vertex_index[MAX_POLY_EDGES];
   double normal[3];

   if (fill_holes == no)
      return;

   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == yes)
     continue;
      for (j=0; j<ph->polygon[i].num_vertices; j++)
      {
     v1 = ph->polygon[i].vertex_index[j];
     if (j == ph->polygon[i].num_vertices-1)
        v2 = ph->polygon[i].vertex_index[0];
     else
        v2 = ph->polygon[i].vertex_index[j+1];
     if (find_other_polygon(ph,i,v1,v2) == -1)
     {
        edgelist[count][0] = v1;
        edgelist[count++][1] = v2;
     }
     if (count >= MAX_HOLE_EDGES)
     {
        i = ph->num_polygons;   /* hack to break out of both loops */
        break;
     }
      }
   }

   num_edges = count;

   for (i=0; i<MAX_HOLE_EDGES; i++)
      edgeused[i] = 0;

   while (count > 0)
   {
      for (i=0; i<num_edges; i++)
     if (edgeused[i] == 0)
        break;
      if (i == num_edges)
     break;
      e_num = i;

      start_v = vertex_index[0] = edgelist[e_num][0];
      other_end = vertex_index[1] = edgelist[e_num][1];
      v_num = 2;
      edgeused[e_num] = 1;

      while (1)
      {
     for (i=0; i<num_edges; i++)
     {
        if (edgeused[i] == 0)
        {
           if (edgelist[i][0] == other_end)
           {
          other_end = edgelist[i][1];
          edgeused[i] = 1;
          break;
           }
           else if (edgelist[i][1] == other_end)
           {
          other_end = edgelist[i][0];
          edgeused[i] = 1;
          break;
           }
        }
     }
     if (i == num_edges)
     {
/*      printf("couldn't navigate around hole\n");*/
        /* Got a partial polygon (open loop of edges), just treat it
         * like a complete polygon for now.
         */
        if (v_num >= 3)
        {
           new_p = make_new_polygon(ph,v_num,vertex_index,0,normal,0.0,yes,yes);
           num_holes_filled++;
        }
        break;
     }
     if (other_end == start_v)
     {
        /* Got a complete polygon (closed loop of edges) */
        new_p = make_new_polygon(ph,v_num,vertex_index,0,normal,0.0,yes,yes);
        num_holes_filled++;
/*
        printf("filled hole with (%d): ", new_p);
        for (j=0; j<v_num; j++)
           printf("%d ", vertex_index[j]);
        printf("\n");
*/
        break;
     }
     else
     {
        /* See if the new edge forms a loop at some vertex other than
         * the first one (e.g., a figure 8 hole).
         */
        for (j=1; j<v_num; j++)
        {
           if (other_end == vertex_index[j])
          break;
        }
        if (j == v_num)
        {
           /* Didn't form a loop: add the new edge to the developing polygon */
           vertex_index[v_num++] = other_end;
        }
        else
        {
           /* Make a new polygon from this loop */
           nv = v_num - j;
           new_p = make_new_polygon(ph,nv,&vertex_index[j],0,normal,0.0,yes,yes);
           num_holes_filled++;
/*
           printf("filled hole with (%d): ", new_p);
           for (k=j; k<v_num; k++)
          printf("%d ", vertex_index[k]);
           printf("\n");
*/
           /* Now set v_num to effectively clip the loop out of vertex_index[] */
           v_num = j + 1;
        }
     }
      }
   }

   if (verbose == yes)
      printf("   %d holes filled.\n", num_holes_filled);

}


void check_edge_usage(PolyhedronStruct* ph)
{

   int j, k, v1, v2;

   for (j=0; j<ph->num_polygons; j++)
   {
      if (ph->polygon[j].thrown_out == yes)
     continue;
      for (k=0; k<ph->polygon[j].num_vertices; k++)
      {
     v1 = ph->polygon[j].vertex_index[k];
     if (k == ph->polygon[j].num_vertices-1)
        v2 = ph->polygon[j].vertex_index[0];
     else
        v2 = ph->polygon[j].vertex_index[k+1];
     if (find_other_polygon(ph,j,v1,v2) == -1)
        printf("edge %d %d is unshared (%d only)\n", v1, v2, j);
      }
   }

}


void check_edge_lengths(PolyhedronStruct* ph, double max_edge_length)
{

   int i, j, v1, v2, vi1, vi2, v_new, num_edges_split=0;
   double len, vec[3], vert[3];
   PolygonStruct* p;

   if (max_edge_length < 0.0)
      return;

   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == yes)
     continue;
      p = &ph->polygon[i];
      for (j=0; j<p->num_vertices; j++)
      {
     vi1 = j;
     vi2 = (j+1)%p->num_vertices;
     v1 = ph->polygon[i].vertex_index[vi1];
     v2 = ph->polygon[i].vertex_index[vi2];
     vec[0] = ph->vertex[v1].coord[0] - ph->vertex[v2].coord[0];
     vec[1] = ph->vertex[v1].coord[1] - ph->vertex[v2].coord[1];
     vec[2] = ph->vertex[v1].coord[2] - ph->vertex[v2].coord[2];
     len = VECTOR_MAGNITUDE(vec);
     if (len > max_edge_length)
     {
        vert[0] = (ph->vertex[v1].coord[0] + ph->vertex[v2].coord[0]) * 0.5;
        vert[1] = (ph->vertex[v1].coord[1] + ph->vertex[v2].coord[1]) * 0.5;
        vert[2] = (ph->vertex[v1].coord[2] + ph->vertex[v2].coord[2]) * 0.5;
        v_new = make_new_vertex(ph,vert,ph->vertex[v1].polyhedron_number,no,yes);
        add_vertex(ph,i,vi1,vi2,v_new);
        i--;
        num_edges_split++;
        break;
     }
      }
   }

   if (verbose == yes)
      printf("   %d edges split.\n", num_edges_split);

}


void triangulate_polygons(PolyhedronStruct* ph, TriangulateOption triangulate)
{

   int i, polys_to_add, verts_to_add, v_split, num_orig_polys, v_index[3];
   ReturnCode rc;

   if (triangulate == no_tri)
      return;

   /* First malloc space for the new polygons */

   if (triangulate == simple_tri)
   {
/*
      if (verbose == yes)
      {
     printf("Triangulating polygons (simple technique)... ");
     fflush(stdout);
      }
*/
      for (i=0, polys_to_add=0, verts_to_add=0; i<ph->num_polygons; i++)
      {
     if (ph->polygon[i].thrown_out == yes)
        continue;
     if (ph->polygon[i].num_vertices == 3)
        continue;
     if (ph->polygon[i].num_vertices == 4)
        polys_to_add++;
     else if (ph->polygon[i].num_vertices > 6)
     {
        polys_to_add += (ph->polygon[i].num_vertices - 1);
        verts_to_add++;
     }
     else
     {
        polys_to_add += (ph->polygon[i].num_vertices - 3);
     }
      }
   }
   else if (triangulate == complex_tri)
   {
/*
      if (verbose == yes)
      {
     printf("Triangulating polygons (complex technique)... ");
     fflush(stdout);
      }
*/
      for (i=0, polys_to_add=0, verts_to_add=0; i<ph->num_polygons; i++)
      {
     if (ph->polygon[i].thrown_out == yes)
        continue;
     if (ph->polygon[i].num_vertices == 3)
        continue;
     if (ph->polygon[i].num_vertices == 4)
     {
        polys_to_add += 3;
        verts_to_add++;
     }
     else if (ph->polygon[i].num_vertices > 6)
     {
        polys_to_add += (ph->polygon[i].num_vertices - 1);
        verts_to_add++;
     }
     else
     {
        polys_to_add += (ph->polygon[i].num_vertices - 3);
     }
      }
   }

   num_orig_polys = ph->num_polygons;

   if (polys_to_add > 0)
      ph->polygon = (PolygonStruct*)simm_realloc(ph->polygon,
            (ph->num_polygons+polys_to_add)*sizeof(PolygonStruct),&rc);

   if (verts_to_add > 0)
      ph->vertex = (VertexStruct*)simm_realloc(ph->vertex,
             (ph->num_vertices+verts_to_add)*sizeof(VertexStruct),&rc);

   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == yes)
     continue;

      if (ph->polygon[i].num_vertices == 3)
     continue;

      if (ph->polygon[i].num_vertices == 4)
      {
     if (triangulate == simple_tri)
     {
        v_split = find_largest_angle(ph,i);
        v_index[0] = ph->polygon[i].vertex_index[v_split];
        v_index[1] = ph->polygon[i].vertex_index[(v_split+2)%4];
        v_index[2] = ph->polygon[i].vertex_index[(v_split+3)%4];
        make_new_polygon(ph,3,v_index,ph->polygon[i].polyhedron_number,
                 ph->polygon[i].normal,ph->polygon[i].d,no,no);
        v_index[0] = ph->polygon[i].vertex_index[v_split];
        v_index[1] = ph->polygon[i].vertex_index[(v_split+1)%4];
        v_index[2] = ph->polygon[i].vertex_index[(v_split+2)%4];
        change_vertex_indices(ph,i,v_index,3);
     }
     else
     {
        quarter_quad(ph,i);
     }
      }
      else if (ph->polygon[i].num_vertices == 5)
      {
     trisect_fiver(ph,i);
      }
      else if (ph->polygon[i].num_vertices == 6)
      {
     bisect_sixer(ph,i);
     i--;
      }
      else
      {
     split_large_polygon(ph,i,no,no);
      }
   }

   if (verbose == yes)
      printf("   added %d polygons by triangulation.\n", polys_to_add);

}


void quarter_quad(PolyhedronStruct* ph, int poly_num)
{

   int v1, v2, v3, v4, new_v, v_index[3];
   double pt[3];
   PolygonStruct* p;

   p = &ph->polygon[poly_num];

   v1 = p->vertex_index[0];
   v2 = p->vertex_index[1];
   v3 = p->vertex_index[2];
   v4 = p->vertex_index[3];

   pt[0] = (ph->vertex[v1].coord[0] + ph->vertex[v2].coord[0] +
        ph->vertex[v3].coord[0] + ph->vertex[v4].coord[0]) / 4.0;
   pt[1] = (ph->vertex[v1].coord[1] + ph->vertex[v2].coord[1] +
        ph->vertex[v3].coord[1] + ph->vertex[v4].coord[1]) / 4.0;
   pt[2] = (ph->vertex[v1].coord[2] + ph->vertex[v2].coord[2] +
        ph->vertex[v3].coord[2] + ph->vertex[v4].coord[2]) / 4.0;

   new_v = make_new_vertex(ph,pt,ph->vertex[v1].polyhedron_number,no,no);

   v_index[0] = v1;
   v_index[1] = v2;
   v_index[2] = new_v;
   change_vertex_indices(ph,poly_num,v_index,3);

   v_index[0] = v2;
   v_index[1] = v3;
   v_index[2] = new_v;
   make_new_polygon(ph,3,v_index,p->polyhedron_number,p->normal,p->d,no,no);

   v_index[0] = v3;
   v_index[1] = v4;
   v_index[2] = new_v;
   make_new_polygon(ph,3,v_index,p->polyhedron_number,p->normal,p->d,no,no);

   v_index[0] = v4;
   v_index[1] = v1;
   v_index[2] = new_v;
   make_new_polygon(ph,3,v_index,p->polyhedron_number,p->normal,p->d,no,no);

}


void split_triangle(PolyhedronStruct* ph, int poly_num)
{

   int v1, v2, v3, v_index[3], new_v;
   double pt[3];
   PolygonStruct* p;

   p = &ph->polygon[poly_num];

   v1 = p->vertex_index[0];
   v2 = p->vertex_index[1];
   v3 = p->vertex_index[2];

   pt[0] = (ph->vertex[v1].coord[0] + ph->vertex[v2].coord[0] +
        ph->vertex[v3].coord[0]) / 3.0;
   pt[1] = (ph->vertex[v1].coord[1] + ph->vertex[v2].coord[1] +
        ph->vertex[v3].coord[1]) / 3.0;
   pt[2] = (ph->vertex[v1].coord[2] + ph->vertex[v2].coord[2] +
        ph->vertex[v3].coord[2]) / 3.0;

   new_v = make_new_vertex(ph,pt,ph->vertex[v1].polyhedron_number,no,no);

   v_index[0] = v1;
   v_index[1] = v2;
   v_index[2] = new_v;
   change_vertex_indices(ph,poly_num,v_index,3);

   v_index[0] = v2;
   v_index[1] = v3;
   v_index[2] = new_v;
   make_new_polygon(ph,3,v_index,p->polyhedron_number,p->normal,p->d,no,no);

   v_index[0] = v3;
   v_index[1] = v1;
   v_index[2] = new_v;
   make_new_polygon(ph,3,v_index,p->polyhedron_number,p->normal,p->d,no,no);

}


SBoolean polygon_very_concave(PolyhedronStruct* ph, int poly_num)
{

   int i, v1, v2, v3;
   double vec1[3], vec2[3], dot_product, angle_sum, convex_sum;
   double offset, ext_angle;

   for (i=0, angle_sum=0.0; i<ph->polygon[poly_num].num_vertices; i++)
   {
      v2 = ph->polygon[poly_num].vertex_index[i];
      if (i == 0)
     v1 = ph->polygon[poly_num].vertex_index[ph->polygon[poly_num].num_vertices-1];
      else
     v1 = ph->polygon[poly_num].vertex_index[i-1];
      if (i == ph->polygon[poly_num].num_vertices-1)
     v3 = ph->polygon[poly_num].vertex_index[0];
      else
     v3 = ph->polygon[poly_num].vertex_index[i+1];

      vec1[0] = ph->vertex[v1].coord[0] - ph->vertex[v2].coord[0];
      vec1[1] = ph->vertex[v1].coord[1] - ph->vertex[v2].coord[1];
      vec1[2] = ph->vertex[v1].coord[2] - ph->vertex[v2].coord[2];

      vec2[0] = ph->vertex[v3].coord[0] - ph->vertex[v2].coord[0];
      vec2[1] = ph->vertex[v3].coord[1] - ph->vertex[v2].coord[1];
      vec2[2] = ph->vertex[v3].coord[2] - ph->vertex[v2].coord[2];

      normalize_vector(vec1,vec1);
      normalize_vector(vec2,vec2);

      dot_product = vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];

      if (dot_product < -1.0)
     angle_sum += 180.0;
      else if (dot_product > 1.0)
     angle_sum += 0.0;
      else
     angle_sum += RTOD*acos(dot_product);
   }

   ext_angle = 360.0/ph->polygon[poly_num].num_vertices;
   convex_sum = ph->polygon[poly_num].num_vertices * (180.0 - ext_angle);
   offset = (convex_sum - angle_sum) * 0.5;

   if ((offset > 45.0) && (offset > convex_sum*0.05))
      return (yes);

   return (no);

}


void print_internal_angles(PolyhedronStruct* ph, int pl)
{

   int i;
   int count=0, v1, v2, v3;
   double v_angle, vec1[3], vec2[3];
   PolygonStruct* ps;

   ps = &ph->polygon[pl];

   printf("polygon %d ", pl);
   for (i=0; i<ps->num_vertices; i++)
      printf("%d ", ps->vertex_index[i]);
   printf(", internal angles:\n");

   for (i=0; i<ps->num_vertices; i++)
   {
      v1 = ps->vertex_index[(i+ps->num_vertices-1)%ps->num_vertices];
      v2 = ps->vertex_index[i];
      v3 = ps->vertex_index[(i+1)%ps->num_vertices];

      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v1].coord,vec1);
      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v3].coord,vec2);

      v_angle = compute_angle_between_vectors(vec1,vec2);
      printf("%5.1lf ", v_angle*RTOD);
   }
   printf("\n");

}


void trisect_fiver(PolyhedronStruct* ph, int old_polygon)
{

   int i, v_index[3], v_split, max_index;
   int count=0, v1, v2, v3, col[5];
   double v_angle[5], vec1[3], vec2[3], max_angle=MINMDOUBLE;
   PolygonStruct* p;

   p = &ph->polygon[old_polygon];

   /* In order to find the best trisection point, first find the internal
    * angles at the vertices. col[] holds the indices of vertices whose
    * internal angle is 180.0, and count is number of such vertices.
    */

   for (i=0; i<p->num_vertices; i++)
   {
      v1 = p->vertex_index[(i+p->num_vertices-1)%p->num_vertices];
      v2 = p->vertex_index[i];
      v3 = p->vertex_index[(i+1)%p->num_vertices];

      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v1].coord,vec1);
      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v3].coord,vec2);

      v_angle[i] = compute_angle_between_vectors(vec1,vec2);

      if (v_angle[i] > M_PI - ANGLE_EPSILON)
     col[count++] = i;
      if (v_angle[i] > max_angle)
      {
     max_angle = v_angle[i];
     max_index = i;
      }
   }

   if (count < 2)
   {
      v_split = max_index;
   }
   else if (count == 2)
   {
      if (col[0] == 0 && col[1] == p->num_vertices-1)
      {
     v_split = 2;
      }
      else if ((col[1] - col[0]) == 1)
      {
     v_split = (col[0] + 3) % p->num_vertices;
      }
      else
      {
     v_split = col[0];
      }
   }
   else
   {
      printf("Zero-area fiver found (%d)\n", old_polygon);
   }

   v_index[0] = p->vertex_index[v_split];
   v_index[1] = p->vertex_index[(v_split+2)%5];
   v_index[2] = p->vertex_index[(v_split+3)%5];
   make_new_polygon(ph,3,v_index,p->polyhedron_number,p->normal,p->d,no,no);

   v_index[0] = p->vertex_index[v_split];
   v_index[1] = p->vertex_index[(v_split+3)%5];
   v_index[2] = p->vertex_index[(v_split+4)%5];
   make_new_polygon(ph,3,v_index,p->polyhedron_number,p->normal,p->d,no,no);

   v_index[0] = p->vertex_index[v_split];
   v_index[1] = p->vertex_index[(v_split+1)%5];
   v_index[2] = p->vertex_index[(v_split+2)%5];
   change_vertex_indices(ph,old_polygon,v_index,3);

}


void bisect_sixer(PolyhedronStruct* ph, int old_polygon)
{

   int i, v_index[4], v_split, max_index;
   int count=0, v1, v2, v3, col[6];
   double v_angle[6], vec1[3], vec2[3], max_angle=MINMDOUBLE;
   PolygonStruct* p;

   p = &ph->polygon[old_polygon];

   /* In order to find the best bisection point, first find the internal
    * angles at the vertices. col[] holds the indices of vertices whose
    * internal angle is 180.0, and count is number of such vertices.
    */

   for (i=0; i<p->num_vertices; i++)
   {
      v1 = p->vertex_index[(i+p->num_vertices-1)%p->num_vertices];
      v2 = p->vertex_index[i];
      v3 = p->vertex_index[(i+1)%p->num_vertices];

      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v1].coord,vec1);
      MAKE_3DVECTOR(ph->vertex[v2].coord,ph->vertex[v3].coord,vec2);

      v_angle[i] = compute_angle_between_vectors(vec1,vec2);

      if (v_angle[i] > M_PI - ANGLE_EPSILON)
     col[count++] = i;
      if (v_angle[i] > max_angle)
      {
     max_angle = v_angle[i];
     max_index = i;
      }
   }

   if (count < 3)
   {
      v_split = max_index;
   }
   else if (count == 3)
   {
      if ((col[0] == 0) && (col[1] == 1) && (col[2] == p->num_vertices-1))
      {
     v_split = 3;
      }
      else if ((col[0] == 0) && (col[1] == p->num_vertices-2) &&
           (col[2] == p->num_vertices-1))
      {
     v_split = 2;
      }
      else if ((col[0] == col[1] - 1) && (col[1] == col[2] - 1))
      {
     v_split = (col[1] + 3) % p->num_vertices;
      }
      else
      {
     v_split = max_index;
      }
   }
   else
   {
      printf("Zero-area sixer found (%d)\n", old_polygon);
   }

   v_index[0] = p->vertex_index[v_split];
   v_index[1] = p->vertex_index[(v_split+3)%6];
   v_index[2] = p->vertex_index[(v_split+4)%6];
   v_index[3] = p->vertex_index[(v_split+5)%6];
   make_new_polygon(ph,4,v_index,p->polyhedron_number,p->normal,p->d,no,no);

   v_index[0] = ph->polygon[old_polygon].vertex_index[v_split];
   v_index[1] = ph->polygon[old_polygon].vertex_index[(v_split+1)%6];
   v_index[2] = ph->polygon[old_polygon].vertex_index[(v_split+2)%6];
   v_index[3] = ph->polygon[old_polygon].vertex_index[(v_split+3)%6];
   change_vertex_indices(ph,old_polygon,v_index,4);

}


/* This routine finds the largest internal angle of the polygon
 * and returns the vertex number where it occurs.
 */

int find_largest_angle(PolyhedronStruct* ph, int poly_num)
{

   int i, v1, v2, v3, max_v;
   double angle[20], vec1[3], vec2[3], dot_product, max_angle;

   for (i=0; i<ph->polygon[poly_num].num_vertices; i++)
   {
      v2 = ph->polygon[poly_num].vertex_index[i];
      if (i == 0)
     v1 = ph->polygon[poly_num].vertex_index[ph->polygon[poly_num].num_vertices-1];
      else
     v1 = ph->polygon[poly_num].vertex_index[i-1];
      if (i == ph->polygon[poly_num].num_vertices-1)
     v3 = ph->polygon[poly_num].vertex_index[0];
      else
     v3 = ph->polygon[poly_num].vertex_index[i+1];

      vec1[0] = ph->vertex[v1].coord[0] - ph->vertex[v2].coord[0];
      vec1[1] = ph->vertex[v1].coord[1] - ph->vertex[v2].coord[1];
      vec1[2] = ph->vertex[v1].coord[2] - ph->vertex[v2].coord[2];

      vec2[0] = ph->vertex[v3].coord[0] - ph->vertex[v2].coord[0];
      vec2[1] = ph->vertex[v3].coord[1] - ph->vertex[v2].coord[1];
      vec2[2] = ph->vertex[v3].coord[2] - ph->vertex[v2].coord[2];

      normalize_vector(vec1,vec1);
      normalize_vector(vec2,vec2);

      dot_product = vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];

      if (dot_product < -1.0)
     angle[i] = M_PI;
      else if (dot_product > 1.0)
     angle[i] = 0.0;
      else
     angle[i] = acos(dot_product);
   }

   for (i=0, max_angle=MINMDOUBLE; i<ph->polygon[poly_num].num_vertices; i++)
   {
      if (angle[i] > max_angle)
      {
     max_angle = angle[i];
     max_v = i;
      }
   }

/*   printf("largest angle is %lf at %d\n", max_angle*RTOD, max_v);*/
   return (max_v);

}


void change_vertex_indices(PolyhedronStruct* ph, int p_num, int v_index[], int num_v)
{

   int i, j;
   ReturnCode rc;
   PolygonStruct* p;

   p = &ph->polygon[p_num];

   for (i=0; i<p->num_vertices; i++)
   {
      for (j=0; j<num_v; j++)
      {
     if (p->vertex_index[i] == v_index[j])
        break;
      }
      if (j == num_v)
     remove_polygon_from_vert_poly_list(&ph->vertex[p->vertex_index[i]],p_num);
   }

   p->vertex_index = (int*)simm_realloc(p->vertex_index,num_v*sizeof(int),&rc);
   p->num_vertices = num_v;

   for (i=0; i<num_v; i++)
   {
      p->vertex_index[i] = v_index[i];
      add_polygon_to_vert_poly_list(&ph->vertex[v_index[i]],p_num);
   }

   calc_polygon_bounding_cube(ph,p);

}


void split_large_polygon(PolyhedronStruct* ph, int poly_num, SBoolean realloc_vertices,
             SBoolean realloc_polygons)
{

   int i, ph_num, v_index[3], new_v;
   double pt[3], normal[3];
   PolygonStruct* p;

   p = &ph->polygon[poly_num];
   ph_num = ph->vertex[p->vertex_index[0]].polyhedron_number;

   pt[0] = pt[1] = pt[2] = 0.0;
   for (i=0; i<p->num_vertices; i++)
   {
      pt[0] += ph->vertex[p->vertex_index[i]].coord[0];
      pt[1] += ph->vertex[p->vertex_index[i]].coord[1];
      pt[2] += ph->vertex[p->vertex_index[i]].coord[2];
   }

   pt[0] /= p->num_vertices;
   pt[1] /= p->num_vertices;
   pt[2] /= p->num_vertices;

   new_v = make_new_vertex(ph,pt,ph_num,no,realloc_vertices);
   ph->vertex[new_v].normal[0] = ph->polygon[poly_num].normal[0];
   ph->vertex[new_v].normal[1] = ph->polygon[poly_num].normal[1];
   ph->vertex[new_v].normal[2] = ph->polygon[poly_num].normal[2];

   normal[0] = p->normal[0];
   normal[1] = p->normal[1];
   normal[2] = p->normal[2];
      
   for (i=0; i<p->num_vertices-1; i++)
   {
      v_index[0] = p->vertex_index[i];
      v_index[1] = p->vertex_index[i+1];
      v_index[2] = new_v;
      
      make_new_polygon(ph,3,v_index,p->polyhedron_number,
               normal,p->d,realloc_polygons,no);
      
      p = &ph->polygon[poly_num];
   }

   v_index[0] = p->vertex_index[0];
   v_index[1] = new_v;
   v_index[2] = p->vertex_index[p->num_vertices-1];
   change_vertex_indices(ph,poly_num,v_index,3);

}


void reorient_polyhedron(PolyhedronStruct* ph, double rot_mat[][3])
{

   int i;
   double new_rot1[3][3], new_rot2[3][3], res_mat[3][3], res_mat2[3][3];
   static double x_axis[] = {1.0, 0.0, 0.0};
   static double y_axis[] = {0.0, 1.0, 0.0};

   make_3x3dircos_matrix(30.0,x_axis,new_rot1);
   make_3x3dircos_matrix(30.0,y_axis,new_rot2);

   mult_3x3matrices(new_rot1,new_rot2,res_mat);

   for (i=0; i<ph->num_vertices; i++)
      mult_3x3_by_vector(res_mat,ph->vertex[i].coord);

   mult_3x3matrices(res_mat,rot_mat,res_mat2);
   copy_3x3matrix(res_mat2,rot_mat);

}


void unorient_polyhedron(PolyhedronStruct* ph, double rot_mat[][3])
{

   int i;
   double inverse_mat[3][3];

   transpose_3x3matrix(rot_mat,inverse_mat);

   for (i=0; i<ph->num_vertices; i++)
      mult_3x3_by_vector(inverse_mat,ph->vertex[i].coord);

}


int make_new_vertex(PolyhedronStruct* ph, double pt[], int ph_num,
            SBoolean check_existing, SBoolean realloc_array)
{

   int i, v_num;
   double tmp;
   VertexStruct* vs;

   /* Check to see if the vertex is already in the list. If so, just return. */

   if (check_existing == yes)
   {
      for (i=ph->num_vertices-1; i>=0; i--)
      {
     tmp = pt[0] - ph->vertex[i].coord[0];
     if (tmp > BOOL_EPSILON || tmp < -BOOL_EPSILON)
        continue;
     tmp = pt[1] - ph->vertex[i].coord[1];
     if (tmp > BOOL_EPSILON || tmp < -BOOL_EPSILON)
        continue;
     tmp = pt[2] - ph->vertex[i].coord[2];
     if (tmp > BOOL_EPSILON || tmp < -BOOL_EPSILON)
        continue;
/*   printf("make_new_vertex(ph = %p, matched %d)\n", ph, i);*/
     return (i);
      }
   }

   v_num = ph->num_vertices++;

   if (realloc_array == yes)
   {
      ph->vertex = (VertexStruct*)realloc(ph->vertex,ph->num_vertices*sizeof(VertexStruct));
   }

   vs = &ph->vertex[v_num];

   preread_init_vertex(vs,v_num);
   vs->coord[0] = pt[0];
   vs->coord[1] = pt[1];
   vs->coord[2] = pt[2];
   vs->polyhedron_number = ph_num;
/*
   printf("just made vertex %d\n", v_num);
   print_vertex(vs);
*/
   return (v_num);

}


int make_new_polygon(PolyhedronStruct* ph, int num_vertices, int v_index[],
             int ph_num, double normal[], double d, SBoolean realloc_array,
             SBoolean recalc_normal)
{

   int i, poly_num;
   ReturnCode rc;
   PolygonStruct* p;

   poly_num = ph->num_polygons++;

   if (realloc_array == yes)
   {
      if (ph->num_polygons == 1)
     ph->polygon = (PolygonStruct*)simm_malloc(sizeof(PolygonStruct));
      else
     ph->polygon = (PolygonStruct*)simm_realloc(ph->polygon,
                ph->num_polygons*sizeof(PolygonStruct),&rc);
   }

   p = &ph->polygon[poly_num];

   preread_init_polygon(p);
/*
   printf("make_new_polygon ");
   for (i=0; i<num_vertices; i++)
      printf("%d ", v_index[i]);
   printf("\n");
*/
   p->num_vertices = num_vertices;
   p->vertex_index = (int*)simm_malloc(num_vertices*sizeof(int));
   for (i=0; i<num_vertices; i++)
      p->vertex_index[i] = v_index[i];

   p->polyhedron_number = ph_num;
   p->old = no;

   calc_polygon_bounding_cube(ph,p);

   if (recalc_normal == yes)
   {
      calc_polygon_normal(ph,&ph->polygon[poly_num]);
   }
   else
   {
      COPY_1X3VECTOR(normal,p->normal);
      p->d = d;
   }

   /* Add this polygon to the right vert_poly lists */
   add_poly_to_vert_poly_lists(ph,p,poly_num);

   return (poly_num);

}


void add_poly_to_vert_poly_lists(PolyhedronStruct* ph, PolygonStruct* p, int poly_num)
{

   int i, j;
   ReturnCode rc;
   VertexStruct* v;

   for (i=0; i<p->num_vertices; i++)
   {
      v = &ph->vertex[p->vertex_index[i]];
      if (v->thrown_out == yes)
     printf("OOPS: adding to v[deleted].polygons\n");
      for (j=0; j<v->polygon_count; j++)
      {
     if (v->polygons[j] == poly_num)
     {
/*      printf("add_poly_to_vert_poly_lists: already there!\n");*/
        break;
     }
      }
      if (j == v->polygon_count)
      {
     v->polygon_count++;
     v->polygons = (int*)simm_realloc(v->polygons,v->polygon_count*sizeof(int),&rc);
     v->polygons[v->polygon_count-1] = poly_num;
      }
   }

}


void convexify_polygons(PolyhedronStruct* ph, NormOptions* opt)
{

   int i, j, e1, e2, v1, v2, v_new, v_from, num_concave = 0;
   double p_bisect[3];

   if (opt->convexify_polygons == no && opt->triangulate == no_tri)
      return;
/*
   if (verbose == yes)
   {
      printf("Making all polygons convex... ");
      fflush(stdout);
   }
*/
   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].num_vertices < 4)
     continue;
      if (ph->polygon[i].thrown_out == yes)
     continue;
      else
      {
     if ((v1 = polygon_is_concave(ph,i)) != -1)
     {
        if (ph->polygon[i].num_vertices == 4)
        {
           v2 = (v1+2)%ph->polygon[i].num_vertices;
           bisect_polygon(ph,i,v1,v2);
        }
        else
        {
           /* Make a vertex (on the other side of the polygon) to form
        * the splitting edge with.  This new vertex (v_new) gets inserted
        * right after v2 in the polygon's vertex list.
        */
           v2 = find_bisection_point(ph,i,v1,p_bisect);
           if (v2 == -1)
          continue;

           v_from = ph->polygon[i].vertex_index[v1];
           v_new = make_new_vertex(ph,p_bisect,ph->vertex[v2].polyhedron_number,no,yes);

           /* The edge you want to split (by putting v_new in the middle of it)
        * is defined by v2 and the next vertex after v2. Store these indices
        * in e1 and e2.
        */
           e1 = v2;
           e2 = (v2+1)%(ph->polygon[i].num_vertices);
           add_vertex(ph,i,e1,e2,v_new);

           /* The bisection vertices may have moved when you added v_new to
        * the polygon, so now you want to find their new indices and put
        * them in e1 and e2.
        */
           for (j=0; j<ph->polygon[i].num_vertices; j++)
          if (ph->polygon[i].vertex_index[j] == v_from)
             break;
           e1 = j;
           for (j=0; j<ph->polygon[i].num_vertices; j++)
          if (ph->polygon[i].vertex_index[j] == v_new)
             break;
           e2 = j;

           /* Now do a simple bisection from e1 to e2 */
           bisect_polygon(ph,i,e1,e2);

           /* Backup the loop so you scan the original polygon again
        * (because it may still be concave in some other region).
        */
           i--;
        }
        num_concave++;
     }
      }
   }

   if (verbose == yes)
      printf("   made %d polygons convex.\n", num_concave);

}


int polygon_is_concave(PolyhedronStruct* ph, int poly)
{

   int i, sign_sum, min_index, axis;
   int* dot_product;
   double v1[3], v2[3], normal[3], ref_vector[3];
   PolygonStruct* p;
   double mag, vec_dot, min_so_far;

   p = &ph->polygon[poly];
   dot_product = (int*)malloc(p->num_vertices*sizeof(int));

   /* form a reference vector which will be compared to all subsequent
    * vectors to see if they point in the same direction. If one of them
    * does not, then the polygon is concave.
    */

   /* First find three adjacent, non-collinear vertices and use them to
    * find the reference vector.
    */
   for (i=0; i<p->num_vertices; i++)
   {
      MAKE_3DVECTOR(ph->vertex[p->vertex_index[i]].coord,
            ph->vertex[p->vertex_index[(i+1)%p->num_vertices]].coord,v2);
      MAKE_3DVECTOR(ph->vertex[p->vertex_index[i]].coord,
            ph->vertex[p->vertex_index[(i+p->num_vertices-1)%
                        p->num_vertices]].coord,v1);
      normalize_vector(v1,v1);
      normalize_vector(v2,v2);
      cross_vectors(v2,v1,ref_vector);
      mag = normalize_vector(ref_vector,ref_vector);
      if (mag > COLLINEAR_EPSILON)
     break;
   }

   /* Now form normal vectors for each vertex, and dot them with the reference
    * vector.
    */
   for (i=0, sign_sum=0; i<p->num_vertices; i++)
   {
      MAKE_3DVECTOR(ph->vertex[p->vertex_index[i]].coord,
            ph->vertex[p->vertex_index[(i+1)%p->num_vertices]].coord,v2);
      MAKE_3DVECTOR(ph->vertex[p->vertex_index[i]].coord,
            ph->vertex[p->vertex_index[(i+p->num_vertices-1)%
                        p->num_vertices]].coord,v1);
      normalize_vector(v1,v1);
      normalize_vector(v2,v2);
      cross_vectors(v2,v1,normal);
      mag = normalize_vector(normal,normal);
      if (EQUAL_WITHIN_ERROR(mag,0.0))
      {
     dot_product[i] = 1;
      }
      else
      {
     vec_dot = DOT_VECTORS(normal,ref_vector);
     dot_product[i] = SIGN(vec_dot);
      }
      sign_sum += dot_product[i];
   }

   /* If each of the dot product signs equals 1, then the polygon is convex */
   if (sign_sum == p->num_vertices)
      return (-1);

   /* If you reach here, the polygon is concave, so now you want to find a
    * vertex where concavity occurs. You do this by finding a vertex where
    * convexity occurs and then checking its dot_product value to see if it
    * is positive or negative. If it is positive, then all the vertices that
    * have a negative dot_product are vertices of concavity, and vice versa.
    * To find a vertex of convexity, find the vertex with the smallest coordinate
    * along either the X, Y, or Z axis. To make sure that the polygon is not
    * perpendicular to the axis, look at the reference vector and pick the
    * coordinate that is closest to zero.
    */
   axis = 0;
   min_so_far = DABS(ref_vector[0]);
   for (i=1; i<3; i++)
   {
      if (DABS(ref_vector[i]) < min_so_far)
      {
     min_so_far = DABS(ref_vector[i]);
     axis = i;
      }
   }

   min_index = 0;
   min_so_far = ph->vertex[p->vertex_index[0]].coord[axis];
   for (i=1; i<p->num_vertices; i++)
   {
      if (ph->vertex[p->vertex_index[i]].coord[axis] < min_so_far)
      {
     min_so_far = ph->vertex[p->vertex_index[i]].coord[axis];
     min_index = i;
      }
   }

   /* Now you want to store the polygon normal in the polygon structure,
    * because you'll need it later if you have to bisect the polygon
    * in the middle of a string of collinear vertices. The normal is
    * in ref_vector[], but you also need to know which way it is pointing.
    * To determine that, look at the min_index vertex (which you know
    * is convex). If its dot_product is positive, then the vertices are
    * ordered counterclockwise, so the ref_vector is correct. If it is
    * negative, then store the negative of the ref_vector in the polygon's
    * normal.
    */

   ph->polygon[poly].normal[0] = ref_vector[0] * dot_product[min_index];
   ph->polygon[poly].normal[1] = ref_vector[1] * dot_product[min_index];
   ph->polygon[poly].normal[2] = ref_vector[2] * dot_product[min_index];

   /* Now find the first concave vertex, which is the first vertex whose
    * dot_product does not match the sign of dot_product[min_index].
    */
/*
   printf("polygon %d is concave ", poly);
   for (i=0; i<p->num_vertices; i++)
      printf("%d ", p->vertex_index[i]);
   printf("\n");
*/
   if (dot_product[min_index] > 0)
   {
      for (i=0; i<p->num_vertices; i++)
     if (dot_product[i] == -1)
        return (i);
   }
   else
   {
      for (i=0; i<p->num_vertices; i++)
     if (dot_product[i] == 1)
        return (i);
   }

   return (-1);

}


void split_edge(PolyhedronStruct* bone, int poly_num, int edge_num)
{

   int newv, lv1, lv2, v1, v2, poly2;
   double pt[3];

   if (poly_num == -1 || edge_num == -1)
      return;

   lv1 = bone->selected_edge;
   lv2 = (bone->selected_edge+1)%bone->polygon[poly_num].num_vertices;

   v1 = bone->polygon[poly_num].vertex_index[lv1];
   v2 = bone->polygon[poly_num].vertex_index[lv2];

   poly2 = find_other_polygon(bone,poly_num,v1,v2);

   pt[0] = (bone->vertex[v1].coord[0] + bone->vertex[v2].coord[0]) * 0.5;
   pt[1] = (bone->vertex[v1].coord[1] + bone->vertex[v2].coord[1]) * 0.5;
   pt[2] = (bone->vertex[v1].coord[2] + bone->vertex[v2].coord[2]) * 0.5;

   newv = make_new_vertex(bone,pt,0,no,yes);

   if (poly2 != -1)
   {
      bone->vertex[newv].normal[0] = (bone->polygon[poly_num].normal[0] +
                      bone->polygon[poly2].normal[0]) * 0.5;
      bone->vertex[newv].normal[1] = (bone->polygon[poly_num].normal[1] +
                      bone->polygon[poly2].normal[1]) * 0.5;
      bone->vertex[newv].normal[2] = (bone->polygon[poly_num].normal[2] +
                      bone->polygon[poly2].normal[2]) * 0.5;
      normalize_vector(bone->vertex[newv].normal,bone->vertex[newv].normal);
   }

   add_vertex(bone,poly_num,lv1,lv2,newv);

}

int find_nth_polygon(PolyhedronStruct* bone, int vertex_num, int* n)
{

   int i, j, count;

   for (i=0, count=0; i<bone->num_polygons; i++)
   {
      for (j=0; j<bone->polygon[i].num_vertices; j++)
      {
     if (bone->polygon[i].vertex_index[j] == vertex_num)
        count++;
     if (count == *n)
     {
        (*n)++;
        return (i);
     }
      }
   }

   /* If you've reached the end of the array without finding the
    * nth occurrence, go back and find the first, then set n = 2.
    */

   for (i=0; i<bone->num_polygons; i++)
   {
      for (j=0; j<bone->polygon[i].num_vertices; j++)
      {
     if (bone->polygon[i].vertex_index[j] == vertex_num)
     {
        *n = 2;
        return (i);
     }
      }
   }

   return (-1);

}


/* This routine adds a vertex to the polygons that share the edge from e1 to e2
 * 
 */
void add_vertex(PolyhedronStruct* ph, int poly, int e1, int e2, int v_new)
{

   int i, v1, v2, other_p;
   PolygonStruct* p;
   ReturnCode rc;

   p = &ph->polygon[poly];

   /* First find and save the global indices of the two vertices */
   v1 = p->vertex_index[e1];
   v2 = p->vertex_index[e2];

   /* Put the new vertex in the vertex list right after e1. */
   p->num_vertices++;
   p->vertex_index = (int*)simm_realloc(p->vertex_index,p->num_vertices*sizeof(int),&rc);

   for (i=p->num_vertices-1; i>e1+1; i--)
      p->vertex_index[i] = p->vertex_index[i-1];
   p->vertex_index[e1+1] = v_new;
   p->old = no;

   /* Now find the other polygon which shares the edge from v1 to v2 */

   other_p = find_other_polygon(ph,poly,v1,v2);

   if (other_p != -1)
      add_vert_to_poly(ph,other_p,v1,v2,v_new);

   add_polygon_to_vert_poly_list(&ph->vertex[v_new],poly);

}


/* This routine adds a vertex to the polygon's vertex list,
 * between vert1 and vert2.
 */
void add_vert_to_poly(PolyhedronStruct* ph, int poly, int vert1, int vert2, int v_new)
{
   int i, j, index2;
   PolygonStruct* p;
   ReturnCode rc;

   p = &ph->polygon[poly];

   if (p->thrown_out == yes)
      printf("OOPS: adding to p[deleted].vertex_index\n");

   for (i=0; i<p->num_vertices; i++)
   {
      if (i == p->num_vertices-1)
     index2 = 0;
      else
     index2 = i+1;
      if ((p->vertex_index[i] == vert1 && p->vertex_index[index2] == vert2) ||
      (p->vertex_index[index2] == vert1 && p->vertex_index[i] == vert2))
     break;
   }

   if (i == p->num_vertices)
      return;

   p->num_vertices++;
   p->vertex_index = (int*)simm_realloc(p->vertex_index,p->num_vertices*sizeof(int),&rc);

   for (j=p->num_vertices-1; j>i+1; j--)
      p->vertex_index[j] = p->vertex_index[j-1];

   p->vertex_index[i+1] = v_new;

   p->old = no;

   /* Now update the vertex's polygon list */

   add_polygon_to_vert_poly_list(&ph->vertex[v_new],poly);
}


/* This routine bisects a polygon by adding an edge from v1 to v2,
 * cutting across the polygon.
 */
void bisect_polygon(PolyhedronStruct* ph, int poly, int v1, int v2)
{
   int i, j, n1, n2, old_num_vert, num_v;
   int* v_index;
   PolygonStruct* p;

   if (v1 == v2 || v1 >= ph->polygon[poly].num_vertices ||
       v2 >= ph->polygon[poly].num_vertices)
   {
      printf("error trying to bisect polygon %d (v1=%d, v2=%d)\n", poly, v1, v2);
      return;
   }

   p = &ph->polygon[poly];

   old_num_vert = p->num_vertices;

   /* It's easier to split the vertex list if you know which index
    * is smaller, so copy the vertex indices into n1 and n2. Then
    * one half of the polygon will be the vertices from n1 to n2,
    * and the other half will be from n2 to n1, wrapping around thru 0.
    */

   if (v1 > v2)
   {
      n1 = v2;
      n2 = v1;
   }
   else
   {
      n1 = v1;
      n2 = v2;
   }

   v_index = (int*)simm_malloc(old_num_vert*sizeof(int));

   num_v = old_num_vert - n2 + n1 + 1;
   for (i=n2,j=0; i<old_num_vert; i++,j++)
      v_index[j] = p->vertex_index[i];
   for (i=0; i<=n1; i++,j++)
      v_index[j] = p->vertex_index[i];

   /* recalculate the normal because you may be splitting a concave polygon,
    * which may mean that the original normal calculation was backwards.
    */
   make_new_polygon(ph,num_v,v_index,p->polyhedron_number,NULL,0.0,yes,yes);

   num_v = n2 - n1 + 1;
   for (i=n1, j=0; i<=n2; i++,j++)
      v_index[j] = ph->polygon[poly].vertex_index[i];
   change_vertex_indices(ph,poly,v_index,num_v);
   calc_polygon_normal(ph,&ph->polygon[poly]);

   free(v_index);
}


/* This routine determines how to bisect a concave polygon, given one
 * end of the bisecting edge (v1). It projects a ray (ray) from v1 to
 * the other side of the polygon, in the direction that bisects the
 * polygon angle at v1.
 */
int find_bisection_point(PolyhedronStruct* ph, int poly, int v1, double p_bisect[])
{
   int i, v2, p1, p2;
   double t, t1, t2;
   double ray[3], ray1[3], ray2[3], v1vec[3], v2vec[3], p1vec[3];
   double p2vec[3], p1p2[3], ptemp1[3], ptemp2[3];
   double mag, mag1, mag2, minT= MAXMDOUBLE;
   PolygonStruct* p;

   p = &ph->polygon[poly];
   v2 = -1;

   /* Make the ray that you'll project to the other side of the polygon */
   MAKE_3DVECTOR(ph->vertex[p->vertex_index[(v1+p->num_vertices-1)%p->num_vertices]].coord,
         ph->vertex[p->vertex_index[v1]].coord,ray1);
   MAKE_3DVECTOR(ph->vertex[p->vertex_index[(v1+1)%p->num_vertices]].coord,
         ph->vertex[p->vertex_index[v1]].coord,ray2);
   mag1 = normalize_vector(ray1,ray1);
   mag2 = normalize_vector(ray2,ray2);
   ray[0] = ray1[0] + ray2[0];
   ray[1] = ray1[1] + ray2[1];
   ray[2] = ray1[2] + ray2[2];
   mag = normalize_vector(ray,ray);

   /* If ray1 and ray2 are parallel, you'll get zero when you add them.
    * If this happens, cross the polygon normal with ray1, which should
    * create a ray that is perpendicular to ray1 and that points into
    * the polygon.
    */
   if (mag < 0.1)
   {
      cross_vectors(ph->polygon[poly].normal,ray1,ray);
      normalize_vector(ray,ray);
   }

   COPY_1X3VECTOR(ph->vertex[p->vertex_index[v1]].coord,v1vec);
   v2vec[0] = v1vec[0] + ray[0];
   v2vec[1] = v1vec[1] + ray[1];
   v2vec[2] = v1vec[2] + ray[2];

   for (i=0; i<p->num_vertices; i++)
   {
      p1 = i;
      if (p1 == (p->num_vertices - 1))
         p2 = 0;
      else
         p2 = i + 1;

      if ((p1 != v1) && (p2 != v1)) 
      {
         MAKE_3DVECTOR(ph->vertex[p->vertex_index[p1]].coord,
            ph->vertex[p->vertex_index[p2]].coord,p1p2); 
         COPY_1X3VECTOR(ph->vertex[p->vertex_index[p1]].coord,p1vec);
         COPY_1X3VECTOR(ph->vertex[p->vertex_index[p2]].coord,p2vec);

         /* check if the point the ray is shot from is not on the line
         * from p1 to p2.
         */
         if (get_distsqr_point_line(v1vec,p1vec,p1p2) > CONVEX_EPSILON_SQR)
         {
            /*
            get_nearest_2points_from_line_line_old(v1vec,ray,p1vec,p1p2,ptemp1,&t1,ptemp2,&t2);
            */
            intersect_lines(v1vec,v2vec,p1vec,p2vec,ptemp1,&t1,ptemp2,&t2);
            /*
            get_nearest_2points_from_line_line(v1vec,v2vec,p1vec,p2vec,ptemp1,&t1,ptemp2,&t2);
            */
            t = distancesqr_between_vertices(ptemp1,ptemp2);
            /* TODO: handle non-planar polygons (t is fairly large) */
            if ((t > -(0.000025) && t < 0.000025) &&
               t1 > CONVEX_EPSILON && t1 < minT &&
               (t2 <= 1.0 || EQUAL_WITHIN_ERROR(t2,1.0)) &&
               (t2 >= 0.0 || EQUAL_WITHIN_ERROR(t2,0.0)))
            {
               v2 = p1;
               minT = t1;
               p_bisect[0] = ptemp1[0];
               p_bisect[1] = ptemp1[1];
               p_bisect[2] = ptemp1[2];
            }
         }
      }
   }

/*
   if (v2 == -1)
   {
      printf("\nHmmm... couldn't find bisection point.\n");
      printf("tried to bisect at %d\n", ph->polygon[poly].vertex_index[v1]+1);
      printf("mag = %lf\n", mag);
      printf("ray1= %lf %lf %lf\n", ray1[0], ray1[1], ray1[2]);
      printf("ray2= %lf %lf %lf\n", ray2[0], ray2[1], ray2[2]);
      printf("ray= %lf %lf %lf\n", ray[0], ray[1], ray[2]);
      printf("%lf %lf %lf segment ground\n", ph->vertex[p->vertex_index[v1]].coord[0],
         ph->vertex[p->vertex_index[v1]].coord[1],
         ph->vertex[p->vertex_index[v1]].coord[2]);
      printf("%lf %lf %lf segment ground\n\n",
         ray1[0]+ph->vertex[p->vertex_index[v1]].coord[0],
         ray1[1]+ph->vertex[p->vertex_index[v1]].coord[1],
         ray1[2]+ph->vertex[p->vertex_index[v1]].coord[2]);
      printf("%lf %lf %lf segment ground\n", ph->vertex[p->vertex_index[v1]].coord[0],
         ph->vertex[p->vertex_index[v1]].coord[1],
         ph->vertex[p->vertex_index[v1]].coord[2]);
      printf("%lf %lf %lf segment ground\n\n",
         ray2[0]+ph->vertex[p->vertex_index[v1]].coord[0],
         ray2[1]+ph->vertex[p->vertex_index[v1]].coord[1],
         ray2[2]+ph->vertex[p->vertex_index[v1]].coord[2]);
      printf("%lf %lf %lf segment ground\n", ph->vertex[p->vertex_index[v1]].coord[0],
         ph->vertex[p->vertex_index[v1]].coord[1],
         ph->vertex[p->vertex_index[v1]].coord[2]);
      printf("%lf %lf %lf segment ground\n\n",
         5.0*ray[0]+ph->vertex[p->vertex_index[v1]].coord[0],
         5.0*ray[1]+ph->vertex[p->vertex_index[v1]].coord[1],
         5.0*ray[2]+ph->vertex[p->vertex_index[v1]].coord[2]);
      for (i=0; i<ph->polygon[poly].num_vertices; i++)
      {
     printf("[%d] %lf %lf %lf\n", ph->polygon[poly].vertex_index[i]+1,
        ph->vertex[ph->polygon[poly].vertex_index[i]].coord[0],
        ph->vertex[ph->polygon[poly].vertex_index[i]].coord[1],
        ph->vertex[ph->polygon[poly].vertex_index[i]].coord[2]);
      }
      fflush(stdout);
   }
*/

   return v2;
}


int polygon_contains_edges(PolygonStruct* p, int v1, int v2, int v3)
{
   int i;

   for (i=0; i<p->num_vertices; i++)
   {
      if (p->vertex_index[i] == v2)
         break;
   }

   if (i == p->num_vertices)
      return -1;

   if (p->vertex_index[(i+p->num_vertices-1)%p->num_vertices] == v1 &&
      p->vertex_index[(i+1)%p->num_vertices] == v3)
      return i;

   if (p->vertex_index[(i+p->num_vertices-1)%p->num_vertices] == v3 &&
      p->vertex_index[(i+1)%p->num_vertices] == v1)
      return i;

   return -1;
}


/* This routine removes thrown_out vertices and polygons from the structure */

void compress_polyhedron(PolyhedronStruct* ph)
{
   int i, j, count, num_verts, num_polys;
   VertexStruct* vert;
   PolygonStruct* poly;

   if (ph->num_removed_vertices == 0 && ph->num_removed_polygons == 0)
      return;

   num_verts = ph->num_vertices - ph->num_removed_vertices;
   num_polys = ph->num_polygons - ph->num_removed_polygons;

   vert = (VertexStruct*)simm_malloc(num_verts*sizeof(VertexStruct));
   for (i=0, count=0; i<ph->num_vertices; i++)
   {
      if (ph->vertex[i].thrown_out == no)
      {
         copy_vertex(&ph->vertex[i],&vert[count]);
         vert[count].polygon_count = 0;
         vert[count].polygons = NULL;
         ph->vertex[i].new_index = count++;
      }
   }

   poly = (PolygonStruct*)simm_malloc(num_polys*sizeof(PolygonStruct));
   for (i=0, count=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == no)
      {
         copy_polygon(&ph->polygon[i],&poly[count]);
         for (j=0; j<poly[count].num_vertices; j++)
            poly[count].vertex_index[j] =
            ph->vertex[ph->polygon[i].vertex_index[j]].new_index;
         count++;
      }
   }

   free(ph->vertex);
   free(ph->polygon);

   ph->vertex = vert;
   ph->polygon = poly;
   ph->num_vertices = num_verts;
   ph->num_removed_vertices = 0;
   ph->num_polygons = num_polys;
   ph->num_removed_polygons = 0;

   make_vert_poly_lists(ph);
}

#ifdef unfinished_new_code
void compress_polyhedron(PolyhedronStruct* ph)
{
   int i, j, far_end, count, num_verts, num_polys;
   VertexStruct* vert;
   PolygonStruct* poly;
   ReturnCode rc;

   num_verts = ph->num_vertices - ph->num_removed_vertices;
   num_polys = ph->num_polygons - ph->num_removed_polygons;
   far_end = ph->num_polygons - 1;

   /* Compress the polygon list by filling in thrown-out entries with
   * polygons from the end of the array.
   */
   for (i=0; i<num_polys; i++)
   {
      if (ph->polygon[i].thrown_out == yes)
      {
         for (j=far_end; j>i; j--)
         {
            if (ph->polygon[j].thrown_out == no)
            {
               copy_polygon(&ph->polygon[j],&ph->polygon[i]);
               break;
            }
         }
         far_end = j-1;
      }
   }

   vert = (VertexStruct*)simm_malloc(num_verts*sizeof(VertexStruct));
   for (i=0, count=0; i<ph->num_vertices; i++)
   {
      if (ph->vertex[i].thrown_out == no)
      {
         copy_vertex(&ph->vertex[i],&vert[count]);
         vert[count].polygon_count = 0;
         vert[count].polygons = NULL;
         ph->vertex[i].new_index = count++;
      }
   }

   poly = (PolygonStruct*)simm_malloc(num_polys*sizeof(PolygonStruct));
   for (i=0, count=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == no)
      {
         copy_polygon(&ph->polygon[i],&poly[count]);
         for (j=0; j<poly[count].num_vertices; j++)
            poly[count].vertex_index[j] =
            ph->vertex[ph->polygon[i].vertex_index[j]].new_index;
         count++;
      }
   }

   ph->num_vertices = num_verts;
   ph->num_removed_vertices = 0;
   ph->num_polygons = num_polys;
   ph->num_removed_polygons = 0;

   ph->polygon = (PolygonStruct*)simm_realloc(ph->polygon,
      ph->num_polygons*sizeof(PolygonStruct),&rc);

   ph->vertex = (VertexStruct*)simm_realloc(ph->vertex,
      ph->num_vertices*sizeof(VertexStruct),&rc);


   make_vert_poly_lists(ph);
}
#endif


void throw_out_vertex(PolyhedronStruct* ph, int v)
{
   if (ph->vertex[v].thrown_out == no)
   {
      FREE_IFNOTNULL(ph->vertex[v].polygons);
      ph->vertex[v].thrown_out = yes;
      ph->num_removed_vertices++;
   }
}


void throw_out_polygon(PolyhedronStruct* ph, int p)
{
   int i;

    if (ph->polygon[p].thrown_out == no)
    {
        for (i=0; i<ph->polygon[p].num_vertices; i++)
            remove_polygon_from_vert_poly_list(&ph->vertex[ph->polygon[p].vertex_index[i]],p);
        FREE_IFNOTNULL(ph->polygon[p].vertex_index);
        ph->polygon[p].thrown_out = yes;
        ph->num_removed_polygons++;
    }
}


void print_polyhedron(PolyhedronStruct* ph, char pname[])
{
   int i;

   printf("POLYHEDRON (%s)**************************\n", pname);
   printf("vertex= 0x%p\n", ph->vertex);
   printf("num_vertices= %d\n", ph->num_vertices);
   printf("polygon= 0x%p\n", ph->polygon);
   printf("num_polygons= %d\n", ph->num_polygons);
   printf("num_removed_vertices= %d\n", ph->num_removed_vertices);
   printf("num_removed_polygons= %d\n", ph->num_removed_polygons);
#ifdef TIM
   printf("bc= %lf %lf    %lf %lf    %lf %lf\n", ph->bc.x1, ph->bc.x2,
      ph->bc.y1, ph->bc.y2, ph->bc.z1, ph->bc.z2);
   printf("poly_list= 0x%p\n", ph->poly_list);
   printf("init_flag= %d\n", ph->init_flag);
#endif

   if (ph->vertex != NULL)
   {
      for (i=0; i<ph->num_vertices; i++)
      {
         printf("  VERTEX[%d]:\n", i);
         print_vertex(&ph->vertex[i]);
      }
   }

   if (ph->polygon != NULL)
   {
      for (i=0; i<ph->num_polygons; i++)
      {
         printf("  POLYGON[%d]:\n", i);
         print_polygon(&ph->polygon[i]);
      }
   }

   printf("*******************************************\n");
   fflush(stdout);
}


void print_vertex(VertexStruct* v)
{
   int i;

   printf("    coord= %lf %lf %lf\n", v->coord[0], v->coord[1], v->coord[2]);
   printf("    normal= %lf %lf %lf\n", v->normal[0], v->normal[1], v->normal[2]);
   printf("    polyhedron_number= %d\n", v->polyhedron_number);
   printf("    thrown_out= %d,  new_index= %d\n", v->thrown_out, v->new_index);
   printf("    polygon_count= %d, old = %d\n", v->polygon_count, v->old);
   printf("    polygons= 0x%p (", v->polygons);
   if (v->polygons != NULL)
   {
      for (i=0; i<v->polygon_count; i++)
         printf("%d ", v->polygons[i]);
   }
   printf(")\n");
}


void print_polygon(PolygonStruct* p)
{
   int i;

   printf("    num_vertices= %d, old = %d\n", p->num_vertices, p->old);
   printf("    vertex_index= 0x%p  (", p->vertex_index);
   if (p->vertex_index != NULL)
   {
      for (i=0; i<p->num_vertices; i++)
         printf("%d ", p->vertex_index[i]);
   }
   printf(")\n");

   printf("    normal= %lf %lf %lf\n", p->normal[0], p->normal[1], p->normal[2]);
   printf("    normal_computed= %d,  ", p->normal_computed);
   printf("    polyhedron_number= %d\n", p->polyhedron_number);
   printf("    thrown_out= %d\n", p->thrown_out);
/*
   printf("    ordering_value= %lf,  ", p->ordering_value);
   printf("    d= %lf, old = %d\n", p->d, p->old);
   printf("    bc= %.4lf %.4lf    %.4lf %.4lf    %.4lf %.4lf\n", p->bc.x1, p->bc.x2,
      p->bc.y1, p->bc.y2, p->bc.z1, p->bc.z2);
   printf("    coplanar_flag= %d,  polygon_mark= %d, poly_output= %d,  poly_adjpush= %d\n",
      p->boolcode.coplanar_flag, p->boolcode.polygon_mark, p->boolcode.poly_output, p->boolcode.poly_adjpush);
   printf("    seg_list.num_inter_seg= %d\n", p->boolcode.seg_list.num_inter_seg);
   printf("    seg_list.seg= 0x%p,  seg_list.segmaxx= 0x%p\n", p->boolcode.seg_list.seg,
      p->boolcode.seg_list.segmaxx);
   printf("    seglst_num= %d, seglst= 0x%p\n", p->boolcode.seglst_num, p->boolcode.seglst);
*/
}


void print_polyhedron_simple(PolyhedronStruct* ph, char pname[])
{
   int i, j;

   printf("%s\n", pname);

   for (i=0; i<ph->num_vertices; i++)
   {
      if (ph->vertex[i].thrown_out == yes)
      {
         printf("[%d] thrown out\n", i);
      }
      else
      {
         printf("[%d] %lf %lf %lf\n", i, ph->vertex[i].coord[0],
            ph->vertex[i].coord[1],ph->vertex[i].coord[2]);
      }
      fflush(stdout);
   }

   for (i=0; i<ph->num_polygons; i++)
   {
      if (ph->polygon[i].thrown_out == yes)
      {
         printf("[%d] thrown out\n", i);
      }
      else
      {
         printf("[%d] %d ", i, ph->polygon[i].num_vertices);
         for (j=0; j<ph->polygon[i].num_vertices; j++)
            printf("%d ", ph->polygon[i].vertex_index[j]);
         printf("\n");
      }
      fflush(stdout);
   }
}

void free_polyhedron(PolyhedronStruct* ph, SBoolean free_ph, ModelStruct* ms)
{
   int i;

   if (!ph)
      return;

   FREE_IFNOTNULL(ph->name);

   for (i=0; i<ph->num_vertices; i++)
      FREE_IFNOTNULL(ph->vertex[i].polygons);

   FREE_IFNOTNULL(ph->vertex);

   for (i=0; i<ph->num_polygons; i++)
   {
      FREE_IFNOTNULL(ph->polygon[i].vertex_index);
      FREE_IFNOTNULL(ph->polygon[i].boolcode.seglst);
//dkb crash sept 15, 2009      
      FREE_IFNOTNULL(ph->polygon[i].boolcode.seg_list.seg);
   }

   FREE_IFNOTNULL(ph->polygon);

#if ! NORM && ! ENGINE && ! OPENSMAC
   delete_polyhedron_display_list(ph, ms);
#endif

   if (free_ph == yes)
      free(ph);
}

void find_bounding_cube(PolyhedronStruct* polyhedron, BoundingCube* bc)
{
   int i;

   bc->x1 = bc->y1 = bc->z1 = MAXMDOUBLE;
   bc->x2 = bc->y2 = bc->z2 = MINMDOUBLE;

   for (i=0; i<polyhedron->num_vertices; i++)
   {
      if (polyhedron->vertex[i].thrown_out == yes)
         continue;
      if (polyhedron->vertex[i].coord[0] < bc->x1)
         bc->x1 = polyhedron->vertex[i].coord[0];
      if (polyhedron->vertex[i].coord[0] > bc->x2)
         bc->x2 = polyhedron->vertex[i].coord[0];
      if (polyhedron->vertex[i].coord[1] < bc->y1)
         bc->y1 = polyhedron->vertex[i].coord[1];
      if (polyhedron->vertex[i].coord[1] > bc->y2)
         bc->y2 = polyhedron->vertex[i].coord[1];
      if (polyhedron->vertex[i].coord[2] < bc->z1)
         bc->z1 = polyhedron->vertex[i].coord[2];
      if (polyhedron->vertex[i].coord[2] > bc->z2)
         bc->z2 = polyhedron->vertex[i].coord[2];
   }
}

/* Unscale the bones in a model, using the scale factors
 * that were stored in each segment when the model was scaled.
 * This is useful when you want to export the bones of a model
 * (e.g., to OpenSim).
 */
void unscale_bones(ModelStruct* model)
{
   int i, j, k;

   for (i = 0; i < model->numsegments; i++)
   {
      for (j = 0; j < model->segment[i].numBones; j++)
      {
         PolyhedronStruct* ph = &model->segment[i].bone[j];

         for (k = 0; k < ph->num_vertices; k++)
         {
            ph->vertex[k].coord[XX] /= model->segment[i].bone_scale[XX];
            ph->vertex[k].coord[YY] /= model->segment[i].bone_scale[YY];
            ph->vertex[k].coord[ZZ] /= model->segment[i].bone_scale[ZZ];
         }
         ph->bc.x1 /= model->segment[i].bone_scale[XX];
         ph->bc.x2 /= model->segment[i].bone_scale[XX];
         ph->bc.y1 /= model->segment[i].bone_scale[YY];
         ph->bc.y2 /= model->segment[i].bone_scale[YY];
         ph->bc.z1 /= model->segment[i].bone_scale[ZZ];
         ph->bc.z2 /= model->segment[i].bone_scale[ZZ];
      }
   }
}
