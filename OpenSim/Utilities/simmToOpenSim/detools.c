/*******************************************************************************

   DETOOLS.C

   Author: Kenny Smith

   Date: 22-OCT-98

   Copyright (c) 1992-8 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/
#include <float.h>
#include <assert.h>
#include <ctype.h>

#include "universal.h"
#include "deformeditor.h"

#include "globals.h"
#include "functions.h"
#include "wefunctions.h"
#include "defunctions.h"

/*************** DEFINES (for this file only) *********************************/
#define CHECK_MIN_VALUE 0
#define CHECK_MAX_VALUE 1


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static float* sBoneScale = NULL;


/*************** EXTERNED VARIABLES (declared in another .c file) *************/
#if ! SIMM_VIEWER && ! OPENSMAC
extern DeformEditorStruct* de;
extern WindowParams*     de_win_params;
extern WinUnion*         de_win_union;
#endif

/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/

/* -------------------------------------------------------------------------
   init_deform - 
---------------------------------------------------------------------------- */
void init_deform (DeformObject* dfm)
{
   dfm->name      = NULL;
   dfm->segment   = -1;
   dfm->active    = yes;
   dfm->visible   = yes;
   dfm->autoReset = no;
   dfm->translationOnly = no;
   dfm->innerMin.xyz[0] = dfm->innerMin.xyz[1] = dfm->innerMin.xyz[2] = -0.03;
   dfm->innerMax.xyz[0] = dfm->innerMax.xyz[1] = dfm->innerMax.xyz[2] =  0.03;
   dfm->outerMin.xyz[0] = dfm->outerMin.xyz[1] = dfm->outerMin.xyz[2] = -0.04;
   dfm->outerMax.xyz[0] = dfm->outerMax.xyz[1] = dfm->outerMax.xyz[2] =  0.04;
   
   dfm->position.rotation_axis.xyz[0] = 1.0;
   dfm->position.rotation_axis.xyz[1] = 0.0;
   dfm->position.rotation_axis.xyz[2] = 0.0;
   dfm->position.rotation_angle = 0.0;
   dfm->position.translation.xyz[0] = 0.0;
   dfm->position.translation.xyz[1] = 0.0;
   dfm->position.translation.xyz[2] = 0.0;
   dfm->position.xforms_valid = no;
   identity_matrix(dfm->position.from_local_xform);
   identity_matrix(dfm->position.to_local_xform);
   
   dfm->deform_start = dfm->deform_end = dfm->position;
   
   dfm->deform_factor = 0.0;
   
   identity_matrix(dfm->delta_xform);
   identity_matrix(dfm->delta_xform2);
   identity_matrix(dfm->delta_xform3);
   
   dfm->innerBox = dfm->innerBoxUndeformed = NULL;
   dfm->outerBox = dfm->outerBoxUndeformed = NULL;

#if ! ENGINE
   init_deform_box_verts(dfm);
#endif
}

/* -------------------------------------------------------------------------
   init_deformity - 
---------------------------------------------------------------------------- */
void init_deformity (Deformity* dty)
{
   dty->name = NULL;
   dty->value = dty->default_value = MINMDOUBLE;
   dty->range.start = 0.0;
   dty->range.end = 100.0;
   dty->keys[0] = null_key;
   dty->keys[1] = null_key;
   dty->num_deforms = 0;
   dty->deform = NULL;
}


/* -------------------------------------------------------------------------
   pt_in_box - return true if the specified 3d point is inside the specified
      3d axis-aligned box.
---------------------------------------------------------------------------- */
static SBoolean pt_in_box (const double pt[3], const dpCoord3D* min, const dpCoord3D* max)
{
   return (SBoolean) (pt[0] > min->xyz[0] && pt[0] < max->xyz[0] &&
                      pt[1] > min->xyz[1] && pt[1] < max->xyz[1] &&
                      pt[2] > min->xyz[2] && pt[2] < max->xyz[2]);
}

/* -------------------------------------------------------------------------
   calc_transition_factor - return a "transition factor" value that smoothly
      ranges from 1.0 to 0.0 as the specified 3d point moves from inside
      the deform's inner box to outside the deform's outer box.
---------------------------------------------------------------------------- */
static double calc_transition_factor (DeformObject* dfm, const double pt[3])
{
   double factors[3], factor = 0.0;
   int    i, n = 0;
   
   if (pt[0] >= dfm->outerMin.xyz[0] && pt[0] <= dfm->innerMin.xyz[0])
      factors[n++] = (pt[0] - dfm->innerMin.xyz[0]) / (dfm->outerMin.xyz[0] - dfm->innerMin.xyz[0]);
   else if (pt[0] <= dfm->outerMax.xyz[0] && pt[0] >= dfm->innerMax.xyz[0])
      factors[n++] = (pt[0] - dfm->innerMax.xyz[0]) / (dfm->outerMax.xyz[0] - dfm->innerMax.xyz[0]);
   
   if (pt[1] >= dfm->outerMin.xyz[1] && pt[1] <= dfm->innerMin.xyz[1])
      factors[n++] = (pt[1] - dfm->innerMin.xyz[1]) / (dfm->outerMin.xyz[1] - dfm->innerMin.xyz[1]);
   else if (pt[1] <= dfm->outerMax.xyz[1] && pt[1] >= dfm->innerMax.xyz[1])
      factors[n++] = (pt[1] - dfm->innerMax.xyz[1]) / (dfm->outerMax.xyz[1] - dfm->innerMax.xyz[1]);
   
   if (pt[2] >= dfm->outerMin.xyz[2] && pt[2] <= dfm->innerMin.xyz[2])
      factors[n++] = (pt[2] - dfm->innerMin.xyz[2]) / (dfm->outerMin.xyz[2] - dfm->innerMin.xyz[2]);
   else if (pt[2] <= dfm->outerMax.xyz[2] && pt[2] >= dfm->innerMax.xyz[2])
      factors[n++] = (pt[2] - dfm->innerMax.xyz[2]) / (dfm->outerMax.xyz[2] - dfm->innerMax.xyz[2]);
   
   for (i = 0; i < n; i++)
      if (factor < factors[i])
         factor = factors[i];

   return 1.0 - factor;
}

#if ! ENGINE
#if ! OPENSMAC

/* -------------------------------------------------------------------------
   deform_mesh - apply the specified deformation to the specified polygon mesh.
---------------------------------------------------------------------------- */
static void deform_mesh (DeformObject* dfm, PolyhedronStruct* mesh)
{
   int i;
   
   if ( ! dfm->active)
      return;
   
   for (i = 0; i < mesh->num_vertices; i++)
      deform_vert(dfm, mesh->undeformed[i].coord, mesh->vertex[i].coord, NULL);
}


/* -------------------------------------------------------------------------
   deform_deform_boxes - 
---------------------------------------------------------------------------- */
static void deform_deform_boxes (ModelStruct* ms, DeformObject* dfm, DeformObject* deformer)
{
   
   int i, n = 3 * VERTS_PER_BOX;
   
   if ( ! deformer->active)
      return;
   
   for (i = 0; i < n; i += 3)
   {
      double pt[3];
#if 0
      _COPY_VERT(pt, &dfm->innerBox[i]);
      transform_pt(dfm->deform_start.from_local_xform, pt);
      transform_pt(deformer->delta_xform, pt);
      transform_pt(dfm->deform_start.to_local_xform, pt);
      _COPY_VERT(&dfm->innerBox[i], pt);
#else
      double   undeformedPt[3];
      SBoolean inInnerBox, inOuterBox;
      XForm*   xform = &dfm->deform_start;

      _COPY_VERT(undeformedPt, &dfm->innerBoxUndeformed[i]);
      transform_pt(xform->from_local_xform, undeformedPt);
      transform_pt(deformer->position.to_local_xform, undeformedPt);
      
      inInnerBox = pt_in_box(undeformedPt, &deformer->innerMin, &deformer->innerMax);
      inOuterBox = pt_in_box(undeformedPt, &deformer->outerMin, &deformer->outerMax);
      
      if (inInnerBox || inOuterBox)
      {
         _COPY_VERT(pt, &dfm->innerBox[i]);
         transform_pt(xform->from_local_xform, pt);

         if (inInnerBox)
            transform_pt(deformer->delta_xform, pt);
         else
         {
            double pt2[3], delta[3], t = calc_transition_factor(deformer, undeformedPt);
            
            _COPY_VERT(pt2, pt);
            transform_pt(deformer->delta_xform, pt2);
            
            _MAKE_VEC(delta, pt2, pt);
            _SCALE_VEC(delta, t);
            _ADD_VEC(pt, delta);
         }
         transform_pt(xform->to_local_xform, pt);
         _COPY_VERT(&dfm->innerBox[i], pt);
      }
#endif
   }
}

#if ! SIMM_DEMO_VERSION && ! SIMM_VIEWER

/* -------------------------------------------------------------------------
   de_track_cb - 
---------------------------------------------------------------------------- */
void de_track_cb (void* data, SimmEvent se)
{
   int i, count, de_vals[DE_MAX_DEVS];   
   DeformEditorTracker* tracker = (DeformEditorTracker*) data;
   DEModelOptions* deop = NULL;
   Scene* scene = NULL;
   ModelStruct* ms = NULL;
   SegmentStruct*  seg;
   DeformObject*   dfm;
   XForm*          xform;
   SBoolean redraw = no;
   double   z_dist, tmp_matrix[4][4], inv_view_matrix[4][4];
   double   wpt[4], wpt2[4], owpt[4], owpt2[4];
   int      bpan_mx_new, bpan_my_new;
   double   bpan_wx_new, bpan_wy_new, bpan_wz_new;
   int      mx_new, my_new;
   double   wx_new, wy_new, wz_new;
   double   angle, origin[4], new_origin[4], naxis[4], axis[4], x_percent, cursor_movement;

   getDev(de->numdevs, de->devs, de_vals);

   for (count = 0, i = 0; i < de->numdevs; i++)
      if (de_vals[i] == 1)
     count++;

   if (count == 0)
   {
      REMOVE_TRACKER();
      return;
   }

   scene = tracker->scene;
   ms = tracker->model;
   deop = &de->deop[ms->modelnum];
   seg = &ms->segment[deop->segment];
   dfm = &seg->deform[deop->deform];
   
   if (!dfm)
      return;
   xform = get_deform_xform(dfm, deop->deformMode);
   
   recalc_deform_xforms(seg, dfm);

   if (de_vals[DE_MOVE_DEFORM_KEY] && deop->deformMode <= DE_DEFORM_END_MODE)
   {
       mx_new = se.mouse_x;
       my_new = se.mouse_y;

       if (deop->trackball_rotation)
       {
           if (de_vals[DE_TRACKBALL_KEY])
           {
               z_dist = scene->tz;

               find_world_coords(scene, mx_new, my_new,
                                 z_dist, &wx_new, &wy_new, &wz_new);
     
               if (tracker->mx_old == -1 || tracker->my_old == -1)
               {
                   tracker->mx_old = mx_new;
                   tracker->my_old = my_new;
                   tracker->wx_old = wx_new;
                   tracker->wy_old = wy_new;
                   tracker->wz_old = wz_new;
               }
               else if (tracker->mx_old != mx_new || tracker->my_old != my_new)
               {
                   origin[0] = origin[1] = origin[2] = 0.0;
                   origin[3] = 1.0;

                   axis[0] = tracker->wy_old - wy_new;
                   axis[1] = wx_new - tracker->wx_old;
                   axis[2] = 0.0;

                   normalize_vector(axis, naxis);
                   naxis[3] = 1.0;

                   invert_4x4transform(scene->transform_matrix,tmp_matrix);
                   mult_4x4matrix_by_vector(tmp_matrix,naxis,axis);
                   mult_4x4matrix_by_vector(tmp_matrix,origin,new_origin);

                   axis[0] -= new_origin[0];
                   axis[1] -= new_origin[1];
                   axis[2] -= new_origin[2];

                   normalize_vector(axis,naxis);

                   naxis[0] -= origin[0];
                   naxis[1] -= origin[1];
                   naxis[2] -= origin[2];

                   normalize_vector(naxis, naxis);
                   convert_vector(ms, naxis, ms->ground_segment, dfm->segment);
                   normalize_vector(naxis, naxis);

                   /* if cursor moves a full screen width, rotate by 90 degrees */
                   cursor_movement = sqrt((mx_new-tracker->mx_old)*(mx_new-tracker->mx_old) +
                                          (my_new-tracker->my_old)*(my_new-tracker->my_old));

                   x_percent = cursor_movement / (double)(scene->viewport[2]);

                   angle = x_percent * 90.0;

                   if (angle >= 0.0 && angle <= 180.0)
                   {
                       DMatrix m;

                       identity_matrix(m);
                       rotate_matrix_axis_angle(m, xform->rotation_axis.xyz,
                                                xform->rotation_angle);
                       rotate_matrix_axis_angle(m, naxis, angle * DTOR);
               
                       extract_rotation(m, &xform->rotation_axis, &xform->rotation_angle);

                       redraw = yes;

                       tracker->mx_old = mx_new;
                       tracker->my_old = my_new;
                       tracker->wx_old = wx_new;
                       tracker->wy_old = wy_new;
                       tracker->wz_old = wz_new;
                   }
               }
           }
           else
           {
               tracker->wx_old = tracker->wy_old = tracker->wz_old = -1.0;
               tracker->mx_old = tracker->my_old = -1;
           }

           if (de_vals[DE_PAN_DEFORM_KEY])
           {
               wpt[0] = wpt[1] = wpt[2] = 0.0;
               transform_pt(xform->from_local_xform, wpt);
               wpt[3] = 1.0;
       
               convert(ms, wpt, dfm->segment, ms->ground_segment);
               mult_4x4matrix_by_vector(scene->transform_matrix, wpt, wpt2);
       
               z_dist = wpt2[2];
               bpan_mx_new = se.mouse_x;
               bpan_my_new = se.mouse_y;
       
               find_world_coords(scene, bpan_mx_new, bpan_my_new,
                                 z_dist, &bpan_wx_new, &bpan_wy_new, &bpan_wz_new);
       
               if (tracker->bpan_mx_old == -1 || tracker->bpan_my_old == -1)
               {
                   tracker->bpan_mx_old = bpan_mx_new;
                   tracker->bpan_my_old = bpan_my_new;
                   tracker->bpan_wx_old = bpan_wx_new;
                   tracker->bpan_wy_old = bpan_wy_new;
                   tracker->bpan_wz_old = bpan_wz_new;
               }
               else if (tracker->bpan_mx_old != bpan_mx_new ||
                        tracker->bpan_my_old != bpan_my_new)
               {
                   wpt[0] = bpan_wx_new;
                   wpt[1] = bpan_wy_new;
                   wpt[2] = bpan_wz_new;
                   wpt[3] = 1.0;

                   invert_4x4transform(scene->transform_matrix,inv_view_matrix);

#if !STEADYCAM
                   if (scene->camera_segment >= 0)
                      copy_4x4matrix(*get_ground_conversion(ms->modelnum,
                                     scene->camera_segment, to_ground), tmp_matrix);
                   else
                      reset_4x4matrix(tmp_matrix);
                   append_4x4matrix(inv_view_matrix,tmp_matrix);
#endif
                   mult_4x4matrix_by_vector(inv_view_matrix,wpt,wpt2);
                   convert(ms, wpt2, ms->ground_segment, dfm->segment);

                   owpt[0] = tracker->bpan_wx_old;
                   owpt[1] = tracker->bpan_wy_old;
                   owpt[2] = tracker->bpan_wz_old;
                   owpt[3] = 1.0;

                   mult_4x4matrix_by_vector(inv_view_matrix,owpt,owpt2);
                   convert(ms, owpt2, ms->ground_segment, dfm->segment);

                   xform->translation.xyz[0] += (wpt2[XX] - owpt2[XX]);
                   xform->translation.xyz[1] += (wpt2[YY] - owpt2[YY]);
                   xform->translation.xyz[2] += (wpt2[ZZ] - owpt2[ZZ]);

                   tracker->bpan_mx_old = bpan_mx_new;
                   tracker->bpan_my_old = bpan_my_new;
                   tracker->bpan_wx_old = bpan_wx_new;
                   tracker->bpan_wy_old = bpan_wy_new;
                   tracker->bpan_wz_old = bpan_wz_new;
                   redraw = yes;
               }
           }
           else
           {
               tracker->bpan_wx_old = tracker->bpan_wy_old = tracker->bpan_wz_old = -1.0;
               tracker->bpan_mx_old = tracker->bpan_my_old = -1;
           }

           if (de_vals[DE_ZOOM_DEFORM_KEY])
           {
          if (tracker->zoom_mx_old == -1 || tracker->zoom_my_old == -1)
          {
             tracker->zoom_mx_old = mx_new;
             tracker->zoom_my_old = my_new;
          
                 wpt[0] = wpt[1] = 0.0;
                 wpt[2] = wpt[3] = 1.0;

                 invert_4x4transform(scene->transform_matrix,tmp_matrix);

                 mult_4x4matrix_by_vector(tmp_matrix, wpt, wpt2);
                 convert(ms, wpt2, ms->ground_segment, dfm->segment);
                 
                 tracker->zoom_vec.xyz[0] = wpt2[0];
                 tracker->zoom_vec.xyz[1] = wpt2[1];
                 tracker->zoom_vec.xyz[2] = wpt2[2];

                 wpt[0] = wpt[1] = wpt[2] = 0.0;
                 wpt[3] = 1.0;

                 mult_4x4matrix_by_vector(tmp_matrix, wpt, wpt2);
                 convert(ms, wpt2, ms->ground_segment, dfm->segment);
                 
                 tracker->zoom_vec.xyz[0] -= wpt2[0];
                 tracker->zoom_vec.xyz[1] -= wpt2[1];
                 tracker->zoom_vec.xyz[2] -= wpt2[2];
          }
              else if (tracker->zoom_mx_old != mx_new || tracker->zoom_my_old != my_new)
          {
             double netmove = (mx_new - tracker->zoom_mx_old) * 0.002;
             
                 xform->translation.xyz[0] += netmove * tracker->zoom_vec.xyz[0];
                 xform->translation.xyz[1] += netmove * tracker->zoom_vec.xyz[1];
                 xform->translation.xyz[2] += netmove * tracker->zoom_vec.xyz[2];

             redraw = yes;
             
             tracker->zoom_mx_old = mx_new;
             tracker->zoom_my_old = my_new;
          }
           }
           else
              tracker->zoom_mx_old = tracker->zoom_my_old = -1;
       }
       else /* trackball turned off, dfm x,y,z rotation */
       {
           double  new_rot_angle;
           DMatrix m;

           identity_matrix(m);
           rotate_matrix_axis_angle(m, xform->rotation_axis.xyz, xform->rotation_angle);

           if (de_vals[DE_ROTATE_X_KEY])
           {
               if (CURSOR_IN_VIEWPORT(mx_new, my_new, scene->viewport))
               {
                   new_rot_angle = DISTANCE_FROM_MIDPOINT(mx_new, scene->viewport) * 0.1;
                   
                   if (deop->xform_frame == DE_LOCAL_FRAME)
                       x_rotate_matrix_bodyfixed(m, new_rot_angle * DTOR);
                   else
                       x_rotate_matrix_spacefixed(m, new_rot_angle * DTOR);
                   
                   redraw = yes;
               }
           }
           if (de_vals[DE_ROTATE_Y_KEY])
           {
               if (CURSOR_IN_VIEWPORT(mx_new, my_new, scene->viewport))
               {
                   new_rot_angle = DISTANCE_FROM_MIDPOINT(mx_new, scene->viewport) * 0.1;
                   
                   if (deop->xform_frame == DE_LOCAL_FRAME)
                       y_rotate_matrix_bodyfixed(m, new_rot_angle * DTOR);
                   else
                       y_rotate_matrix_spacefixed(m, new_rot_angle * DTOR);
                   
                   redraw = yes;
               }
           }
           if (de_vals[DE_ROTATE_Z_KEY])
           {
               if (CURSOR_IN_VIEWPORT(mx_new, my_new, scene->viewport))
               {
                   new_rot_angle = DISTANCE_FROM_MIDPOINT(mx_new, scene->viewport) * 0.1;
                   
                   if (deop->xform_frame == DE_LOCAL_FRAME)
                       z_rotate_matrix_bodyfixed(m, new_rot_angle * DTOR);
                   else
                       z_rotate_matrix_spacefixed(m, new_rot_angle * DTOR);
                   
                   redraw = yes;
               }
           }
           if (redraw)
              extract_rotation(m, &xform->rotation_axis, &xform->rotation_angle);
       }
   }
   
   if (redraw == yes)
   {
      if (deop->deformMode == DE_POSITION_MODE)
      {
         dfm->deform_start = dfm->deform_end = dfm->position;
         
         de->deform_slider.sl[0].value = dfm->deform_factor = 0.0;
      }
      xform->xforms_valid = no;
      recalc_deform_xforms(seg, dfm);
      deform_segment(NULL, -1);
      queue_model_redraw(ms);
      if (ms == de->model)
         display_deformeditor(de_win_params, de_win_union);
   }
} /* de_track_cb */

#endif /* SIMM_DEMO_VERSION && ! SIMM_VIEWER */

#define FOG_DEFORM_BOXES 0

/* -------------------------------------------------------------------------
   set_fog_near_and_far - 
---------------------------------------------------------------------------- */
static void set_fog_near_and_far(const float* box)
{
   double fogNear = FLT_MAX, fogFar = 0.0;
   int j,k,l;
   DMatrix m, n;
   
   glGetDoublev(GL_MODELVIEW_MATRIX, (double*) m);
   invert_4x4transform(m, n);
   
      for (j = 0, l = 0; j < 12; j++)
      {
         for (k = 0; k < VERTS_PER_BOX_EDGE; k++)
         {
            double pt[3];
            
            pt[0] = box[l++];
            pt[1] = box[l++];
            pt[2] = box[l++];
            
            transform_pt(n, pt);
            
            if (pt[2] > 0.0 && fogNear > pt[2])
               fogNear = pt[2];
            
            if (fogFar < pt[2])
               fogFar = pt[2];
         }
      }
   
   if (fogNear == FLT_MAX)
      fogNear = 0.0;
   
   glFogf(GL_FOG_START, fogNear);
   glFogf(GL_FOG_END,   fogFar + 0.5 * (fogFar - fogNear));
   
#if 1
   fprintf(stderr, "%f  %f\n", fogNear, fogFar);
#endif
}

/* -------------------------------------------------------------------------
   draw_deform_objects - 
---------------------------------------------------------------------------- */
void draw_deform_objects(ModelStruct* model, SegmentStruct* seg, int segment_index, ModelDrawOptions* mdo)
{
#if ! SIMM_VIEWER

#define IS_CUR_DEFORM(DFM) (model == de->model && DFM == deop->deform)

   int i, j, k, l;
   static const float active_dfm_color[3]   = { 1.0, 0.5, 1.0 };
   static const float inactive_dfm_color[3] = { 0.0, 0.0, 1.0 };
   DEModelOptions* deop;

   if (de == NULL)
      return;

   deop = &de->deop[model->modelnum];

   if (seg != &model->segment[deop->segment])
      return;

   if (mdo->mode == GL_RENDER)
   {
      glPushAttrib(GL_ENABLE_BIT);
      glDisable(GL_LIGHTING);

#if FOG_DEFORM_BOXES
      glEnable(GL_FOG);
      glFogi(GL_FOG_MODE, GL_LINEAR);
      glFogfv(GL_FOG_COLOR, root.color.cmap[MISC_MODEL_WINDOW_BACKGROUND].rgb);
      glHint(GL_FOG_HINT, GL_FASTEST);  /* or try GL_NICEST */
#endif
   }

   for (i = 0; i < seg->num_deforms; i++)
   {
      DeformObject* dfm = &seg->deform[i];

      if ( ! dfm->visible)
         continue;

      recalc_deform_xforms(seg, dfm);

      // Draw the deform's outer box.
      glPushMatrix();
      glMultMatrixd((double*) dfm->position.from_local_xform);

      if (mdo->mode == GL_RENDER)
      {
         glColor3fv(IS_CUR_DEFORM(i) ? active_dfm_color : inactive_dfm_color);
#if FOG_DEFORM_BOXES
         set_fog_near_and_far(dfm->outerBox);
#endif
      }
      else
      {
         PickIndex color_value;
         GLubyte color[3];

         glShadeModel(GL_FLAT);
         color_value = (DEFORM_OBJECT << OBJECT_BITS) + ((PickIndex)segment_index << SUBELEMENT_BITS) + ((PickIndex)i << 3) + (PickIndex)0; // 0 is for outer box
         pack_int_into_color(color_value, color);
         glColor3ubv(color);
         {
            PickIndex obj_num, object_type;
            int seg_num, deform_index, component;
            get_object_type_and_number(color_value, &object_type, &obj_num);
            seg_num = obj_num >> SUBELEMENT_BITS;
            deform_index = obj_num & SUBELEMENT_MASK;
            component = obj_num & 0x7;
            printf("outer box: %d %d, %d %d %d %d\n", (int)obj_num, (int)object_type, seg_num, deform_index, component);
         }
      }

      for (j = 0, l = 0; j < 12; j++)
      {
         glBegin(GL_LINE_STRIP);

         for (k = 0; k < VERTS_PER_BOX_EDGE; k++)
         {
            glVertex3fv(&dfm->outerBox[l]);

            l += 3;
         }
         glEnd();
      }
      glPopMatrix();

      // Draw the deform's inner box.
      glPushMatrix();
      glMultMatrixd((double*) get_deform_xform3(dfm, deop->deformMode, dfm->deform_factor)->from_local_xform);

      if (IS_CUR_DEFORM(i))
      {
         if (deop->deformMode == DE_POSITION_MODE || dfm->deform_factor == 0.0 || dfm->deform_factor == 1.0)
         {
            glLineWidth(2.0);
         }
      }

      if (mdo->mode == GL_RENDER)
      {
         glColor3fv(IS_CUR_DEFORM(i) ? active_dfm_color : inactive_dfm_color);
#if FOG_DEFORM_BOXES
         set_fog_near_and_far(dfm->innerBox);
#endif
      }
      else
      {
         PickIndex color_value;
         GLubyte color[3];

         glShadeModel(GL_FLAT);
         color_value = (DEFORM_OBJECT << OBJECT_BITS) + ((PickIndex)segment_index << SUBELEMENT_BITS) + ((PickIndex)i << 3) + (PickIndex)1; // 1 is for inner box
         pack_int_into_color(color_value, color);
         glColor3ubv(color);
         {
            PickIndex obj_num, object_type;
            int seg_num, deform_index, component;
            get_object_type_and_number(color_value, &object_type, &obj_num);
            seg_num = obj_num >> SUBELEMENT_BITS;
            deform_index = obj_num & SUBELEMENT_MASK;
            component = obj_num & 0x7;
            printf("inner box: %d %d, %d %d %d %d\n", (int)obj_num, (int)object_type, seg_num, deform_index, component);
         }
      }

      for (j = 0, l = 0; j < 12; j++)
      {
         glBegin(GL_LINE_STRIP);

         for (k = 0; k < VERTS_PER_BOX_EDGE; k++)
         {
            glVertex3fv(&dfm->innerBox[l]);

            l += 3;
         }
         glEnd();
      }

      // Draw the deform's axes (current one only).
      if (IS_CUR_DEFORM(i))
      {
         glBegin(GL_LINES);
         if (mdo->mode == GL_RENDER)
         {
            simm_color(RED);
         }
         else
         {
            PickIndex color_value;
            GLubyte color[3];

            glShadeModel(GL_FLAT);
            color_value = (DEFORM_OBJECT << OBJECT_BITS) + ((PickIndex)segment_index << SUBELEMENT_BITS) + ((PickIndex)i << 3) + (PickIndex)2; // 2 is for X axis
            pack_int_into_color(color_value, color);
            glColor3ubv(color);
         }
         glVertex3d(0, 0, 0);
         glVertex3d(1.5 * fabs(dfm->innerMax.xyz[0]), 0, 0);
         if (mdo->mode == GL_RENDER)
         {
            simm_color(GREEN);
         }
         else
         {
            PickIndex color_value;
            GLubyte color[3];

            glShadeModel(GL_FLAT);
            color_value = (DEFORM_OBJECT << OBJECT_BITS) + ((PickIndex)segment_index << SUBELEMENT_BITS) + ((PickIndex)i << 3) + (PickIndex)3; // 3 is for Y axis
            pack_int_into_color(color_value, color);
            glColor3ubv(color);
         }
         glVertex3d(0, 0, 0);
         glVertex3d(0, 1.5 * fabs(dfm->innerMax.xyz[1]), 0);
         if (mdo->mode == GL_RENDER)
         {
            simm_color(BLUE);
         }
         else
         {
            PickIndex color_value;
            GLubyte color[3];

            glShadeModel(GL_FLAT);
            color_value = (DEFORM_OBJECT << OBJECT_BITS) + ((PickIndex)segment_index << SUBELEMENT_BITS) + ((PickIndex)i << 3) + (PickIndex)4; // 4 is for Z axis
            pack_int_into_color(color_value, color);
            glColor3ubv(color);
         }
         glVertex3d(0, 0, 0);
         glVertex3d(0, 0, 1.5 * fabs(dfm->innerMax.xyz[2]));
         glEnd();
      }

      glLineWidth(1.0);
      glPopMatrix();
   }

   if (mdo->mode == GL_RENDER)
      glPopAttrib();
#endif /* ! SIMM_VIEWER */
}


/* -------------------------------------------------------------------------
   get_deform_xform - 
---------------------------------------------------------------------------- */
XForm* get_deform_xform (DeformObject* dfm, int deformMode)
{
   switch (deformMode)
   {
      case DE_POSITION_MODE:
         return &dfm->position;
      case DE_DEFORM_START_MODE:
         return &dfm->deform_start;
      case DE_DEFORM_END_MODE:
         return &dfm->deform_end;
   }
/*   assert(0);*/
   
   return NULL;
}

/* -------------------------------------------------------------------------
   get_deform_xform2 - 
---------------------------------------------------------------------------- */
XForm* get_deform_xform2 (DeformObject* dfm, int deformMode, double factor)
{
   switch (deformMode)
   {
      case DE_POSITION_MODE:
         return &dfm->position;
         
      default:
         if (EQUAL_WITHIN_ERROR(factor,0.0))
            return &dfm->deform_start;
         else if (EQUAL_WITHIN_ERROR(factor,1.0))
            return &dfm->deform_end;
         else if (deformMode == DE_DEFORM_START_MODE)
            return &dfm->deform_start;
         else if (deformMode == DE_DEFORM_END_MODE)
            return &dfm->deform_end;
   }
/*   assert(0);*/
   
   return NULL;
}

/* -------------------------------------------------------------------------
   get_deform_xform3 - 
---------------------------------------------------------------------------- */
XForm* get_deform_xform3 (DeformObject* dfm, int deformMode, double factor)
{
   switch (deformMode)
   {
      //dkb - removed this because it messed up display of object using slider
 //     case DE_POSITION_MODE:
 //        return &dfm->position;
         
      default:
#if 0
         if (factor == 0.0)
            return &dfm->deform_start;
         else if (factor == 1.0)
            return &dfm->deform_end;
         else
#endif
         {
            static XForm xform;
            
            copy_4x4matrix(dfm->delta_xform3, xform.from_local_xform);

            return &xform;
         }
   }
   /*
   assert(0);
   return NULL;
   */
}

/* -------------------------------------------------------------------------
   deform_segment - 
---------------------------------------------------------------------------- */
void deform_segment (ModelStruct* ms, int segment)
{
   SegmentStruct* seg;
   int i,j,k;

   /* if 'ms' or 'segment' are unspecified, then use the deform editor's
    * current model and/or segment
    */
#if ! SIMM_VIEWER
   if (ms == NULL)
      ms = de->model;
#endif

   if (ms == NULL)
      return;
   
#if ! SIMM_VIEWER
   if (segment < 0)
       segment = de->deop[ms->modelnum].segment;
#endif

   if (segment < 0)
      return;
   
   seg = &ms->segment[segment];
   
   if (seg->num_deforms <= 0)
      return;

#if 0   
   /* prepare any auto-reset transforms */
   calc_auto_reset_deform(seg, NULL);
#endif
   
   /* deform all bones in the segment */
   for (i = 0; i < seg->numBones; i++)
   {
      PolyhedronStruct* bone = &seg->bone[i];
      
      int nBytes = bone->num_vertices * sizeof(VertexStruct);
      
      if (bone->undeformed == NULL)
      {
         bone->undeformed = (VertexStruct*) simm_malloc(nBytes);
         
         if (bone->undeformed)
            memcpy(bone->undeformed, bone->vertex, nBytes);
      }
      if (bone->undeformed)
      {
         memcpy(bone->vertex, bone->undeformed, nBytes);
#if NOSCALE
         if (NOT_EQUAL_WITHIN_ERROR(seg->bone_scale[0], 1.0) ||
             NOT_EQUAL_WITHIN_ERROR(seg->bone_scale[1], 1.0) ||
             NOT_EQUAL_WITHIN_ERROR(seg->bone_scale[2], 1.0))
         {
            sBoneScale = seg->bone_scale;
         }
#else
     sBoneScale = NULL;
#endif
         /*if (deop->deformMode != DE_POSITION_MODE)*/
            for (j = 0; j < seg->num_deforms; j++)
               deform_mesh(&seg->deform[j], bone);
         
         sBoneScale = NULL;
      }
   }

   /* deform the muscle points attached to the segment */
   for (i = 0; i < ms->nummuscles; i++)
   {
      dpMuscleStruct* msc = ms->muscle[i];
      
      for (j = 0; j < msc->path->num_orig_points; j++)
      {
         dpMusclePoint* mp = &msc->path->mp_orig[j];
         
         if (mp->segment == segment)
         {
            _COPY_VERT(mp->point, mp->undeformed_point);
            
            /*if (deop->deformMode != DE_POSITION_MODE)*/
               for (k = 0; k < seg->num_deforms; k++)
                  if (seg->deform[k].active)
                     deform_vert(&seg->deform[k],
                                 mp->undeformed_point,
                                 mp->point, NULL);
         }
      }
   }
   
   /* deform any muscle wrapping objects attached to the segment */
   for (i = 0; i < ms->num_wrap_objects; i++)
   {
      if (ms->wrapobj[i]->segment == segment)
      {
         dpWrapObject* wo = ms->wrapobj[i];
         
         _COPY_VERT(wo->translation.xyz, wo->undeformed_translation.xyz);
         
         /*if (deop->deformMode != DE_POSITION_MODE)*/
            for (k = 0; k < seg->num_deforms; k++)
               if (seg->deform[k].active)
                  deform_vert(&seg->deform[k],
                              wo->undeformed_translation.xyz,
                              wo->translation.xyz, NULL);
         
         wo->xforms_valid = no;
         recalc_xforms(wo);
         inval_model_wrapping(ms, ms->wrapobj[i]);
      }
   }
   
   /* deform any constraint objects attached to the segment */
   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      if (ms->constraintobj[i].segment == segment)
      {
         ConstraintObject* co = &ms->constraintobj[i];
         
         _COPY_VERT(co->translation.xyz, co->undeformed_translation.xyz);
         
         /*if (deop->deformMode != DE_POSITION_MODE)*/
            for (k = 0; k < seg->num_deforms; k++)
               if (seg->deform[k].active)
                  deform_vert(&seg->deform[k],
                              co->undeformed_translation.xyz,
                              co->translation.xyz, NULL);
         
         co->xforms_valid = no;
         recalc_constraint_xforms(co);
         ce_recalc_loops_and_constraints(ms, i, no);
      }
   }

   /* deform (invalidate actually) any joints emanating from the segment */
   for (i = 0; i < ms->numjoints; i++)
   {
      if (ms->joint[i].from == segment)
      {
         double pt[3], jointOrigin[3] = { 0,0,0 };
         SBoolean didDeform = no;

         /* determine the joint's undeformed origin in the segment's frame */
         ms->joint[i].pretransform_active = no;
         ms->joint[i].pretransform_condition = invalid;
         invalidate_joint_matrix(ms, &ms->joint[i]);
#if 0
         if (deop->deformMode == DE_POSITION_MODE)
            continue;
#endif
         convert(ms, jointOrigin, ms->joint[i].to, ms->joint[i].from);
         _COPY_VERT(pt, jointOrigin);
         
         /* determine the joint's deformed origin */
         for (k = 0; k < seg->num_deforms; k++)
            if (seg->deform[k].active)
               deform_vert(&seg->deform[k], jointOrigin, pt, &didDeform);
         
         /* if the joint is subject to deformation, then activate its pretransform */
         if (didDeform)
         {
            ms->joint[i].pretransform_active = yes;
            ms->joint[i].pretransform_condition = invalid;
            invalidate_joint_matrix(ms, &ms->joint[i]);
         }
      }
   }
#if 0
   /* deform the deform boxes in the segment */
   for (i = 0; i < seg->num_deforms; i++)
   {
      DeformObject* dfm = &seg->deform[i];
      int nBytes = VERTS_PER_BOX * 3 * sizeof(float);
      
      memcpy(dfm->innerBox, dfm->innerBoxUndeformed, nBytes);
      memcpy(dfm->outerBox, dfm->outerBoxUndeformed, nBytes);

#if 0
      deform_deform_boxes(ms, dfm, dfm);
#elif 0         
      /*if (deop->deformMode != DE_POSITION_MODE)*/
         for (j = 0; j < seg->num_deforms; j++)
            deform_deform_boxes(ms, dfm, &seg->deform[j]);
#endif
   }
#endif

   delete_segment_display_lists(seg, ms);
   queue_model_redraw(ms);
}
#endif /* ! OPENSMAC */
#endif /* ! ENGINE */


/* ------------------------------------------------------------------------
   reorthogonalize_matrix - given a 4x4 matrix before and after deformation,
      remove any shearing added to the upper 3x3 submatrix during deformation.
      
      The rows of the before and after matrices are treated as three mutually
      orthogonal vectors.  The dot-product between each before and after vector
      is computed to determine which vector has changed the most under deformation.

      The vector that has changed the *most* under deformation remains unchanged.
      The other two vectors are rebuilt via cross-products to create a pure
      rotation matrix that tracks the strongest deformation gradient.
--------------------------------------------------------------------------- */
static void reorthogonalize_matrix (double before[][4], double after[][4])
{
#define _SWAP(I, J) ( \
   map[3].dot = map[I].dot, map[I].dot = map[J].dot, map[J].dot = map[3].dot, \
   map[3].idx = map[I].idx, map[I].idx = map[J].idx, map[J].idx = map[3].idx)

#define _LEAST_DEFORMED  (map[0].idx)
#define _MIDDLE_DEFORMED (map[1].idx)
#define _MOST_DEFORMED   (map[2].idx)

   struct {
      double after[3];
      double dot;
      int    idx;
      }   map[4];  /* NOTE: the 4th slot is used as a temporary by _SWAP() */
      int i;

   /* compute and save the dot products of the before and after matrices */
   for (i = 0; i < 3; i++)
   {
      map[i].after[XX] = after[i][XX];
      map[i].after[YY] = after[i][YY];
      map[i].after[ZZ] = after[i][ZZ];
      
      normalize_vector(map[i].after, map[i].after);
      normalize_vector(before[i], before[i]);
      
      map[i].dot = 1.0 - DOT_VECTORS(before[i], map[i].after);
      map[i].idx = i;
   }
   /* sort the dot products (and their associated indexes) into ascending order */
   if (map[0].dot > map[1].dot) _SWAP(0, 1);
   if (map[1].dot > map[2].dot) _SWAP(1, 2);
   if (map[0].dot > map[1].dot) _SWAP(0, 1);

   /* rebuild the least-deformed vector by crossing the two most-deformed vectors */
   cross_vectors(after[_MOST_DEFORMED], after[_MIDDLE_DEFORMED], after[_LEAST_DEFORMED]);

   /* rebuild the 2nd-least-deformed vector */
   cross_vectors(after[_LEAST_DEFORMED], after[_MOST_DEFORMED], after[_MIDDLE_DEFORMED]);

   /* get the direction right on the rebuilt least-deformed vector */
   if (DOT_VECTORS(after[_LEAST_DEFORMED], before[_LEAST_DEFORMED]) < 0.0)
   {
      after[_LEAST_DEFORMED][XX] = - after[_LEAST_DEFORMED][XX];
      after[_LEAST_DEFORMED][YY] = - after[_LEAST_DEFORMED][YY];
      after[_LEAST_DEFORMED][ZZ] = - after[_LEAST_DEFORMED][ZZ];
   }
   
   /* normalize the reorthogonalized vectors */
   for (i = 0; i < 3; i++)
      normalize_vector(after[i], after[i]);
   
#undef _MOST_DEFORMED
#undef _MIDDLE_DEFORMED
#undef _LEAST_DEFORMED
#undef _SWAP
} /* reorthogonalize_matrix */

/* -------------------------------------------------------------------------
   calc_joint_pretransform - compute a "pretransform" matrix for the specified
     joint that contains the current deformation at the joint origin.
   
   NOTE: We create the joint pretransform matrix by deforming a "mini reference
     frame" whose major axis are each 0.1 mm long.
---------------------------------------------------------------------------- */
void calc_joint_pretransform (ModelStruct* ms, JointStruct* jnt)
{
   DMatrix n = {
      { 1.0, 0.0, 0.0, 0.0 },            /* undeformed reference frame */
      { 0.0, 1.0, 0.0, 0.0 },
      { 0.0, 0.0, 1.0, 0.0 },
      { 0.0, 0.0, 0.0, 1.0 }};
   
   DMatrix m = {
      { 0.0001, 0.0000, 0.0000, 0.0 },   /* mini reference frame */
      { 0.0000, 0.0001, 0.0000, 0.0 },
      { 0.0000, 0.0000, 0.0001, 0.0 },
      { 0.0000, 0.0000, 0.0000, 1.0 }};
   
   jnt->pretransform_active = no;
   jnt->pretransform_condition = invalid;
   
   if (deform_joint_pt(ms, jnt, m[3]))
   {
      int i;
      
      for (i = 0; i < 3; i++)
      {
         deform_joint_pt(ms, jnt, m[i]);
         _MAKE_VEC(m[i], m[i], m[3]);
         normalize_vector(m[i], m[i]);
      }
      
      /* NOTE:  the deformed reference frame may have lost its orthogonality
       *   at this point.  We straighten it out by calling reorthogonalize_matrix():
       */
      reorthogonalize_matrix(n, m);
      
      copy_4x4matrix(m, jnt->pretransform);
      invert_4x4transform(jnt->pretransform, jnt->pretransform_inverse);
   }
   else
   {
      identity_matrix(jnt->pretransform);
      identity_matrix(jnt->pretransform_inverse);
   }
   jnt->pretransform_active = yes;
}


/* -------------------------------------------------------------------------
   deform_vert - apply the specified deformation to the specified 3d point.
      The point must be specified in the reference frame of the deform's
      parent segment.
   
   Parameters:
      dfm           The deform to be applied.
      
      undeformed    This is the point in its completely undeformed
                    position.  This value is necessary to test for inclusion
                    in the specified deform's inner & outer boxes.
      
      pt_to_deform  This is the point in its current (possibly already
                    deformed) position.  This point may get (further)
                    deformed by the specified deform.
      
      didDeform     A boolean flag that is set to TRUE if the point lies
                    inside the deform's inner or outer boxes.  NOTE: this
                    flag is only set to TRUE.  If the point is not effected
                    by this deform, then the flag is not modified.
---------------------------------------------------------------------------- */
void deform_vert (DeformObject* dfm, const double undeformed[3], double pt_to_deform[3], SBoolean* didDeform)
{
   double   undeformedPt[3];
   SBoolean inInnerBox, inOuterBox;

   _COPY_VERT(undeformedPt, undeformed);
   
   if (sBoneScale)
   {
      undeformedPt[0] *= sBoneScale[0];
      undeformedPt[1] *= sBoneScale[1];
      undeformedPt[2] *= sBoneScale[2];
   }
   transform_pt(dfm->position.to_local_xform, undeformedPt);
   
   inInnerBox = pt_in_box(undeformedPt, &dfm->innerMin, &dfm->innerMax);
   inOuterBox = pt_in_box(undeformedPt, &dfm->outerMin, &dfm->outerMax);
   
   if (inInnerBox || inOuterBox)
   {
      if (sBoneScale)
      {
         pt_to_deform[0] *= sBoneScale[0];
         pt_to_deform[1] *= sBoneScale[1];
         pt_to_deform[2] *= sBoneScale[2];
      }
      
      if (inInnerBox)
         transform_pt(dfm->delta_xform, pt_to_deform);
      else
      {
         double pt[3], delta[3], t = calc_transition_factor(dfm, undeformedPt);
      
         _COPY_VERT(pt, pt_to_deform);
         transform_pt(dfm->delta_xform, pt);
         
         _MAKE_VEC(delta, pt, pt_to_deform);
         _SCALE_VEC(delta, t);
         _ADD_VEC(pt_to_deform, delta);
      }
      if (didDeform)
         *didDeform = yes;
      
      if (sBoneScale)
      {
         pt_to_deform[0] /= sBoneScale[0];
         pt_to_deform[1] /= sBoneScale[1];
         pt_to_deform[2] /= sBoneScale[2];
      }
   }
}


/* -------------------------------------------------------------------------
   deform_frame_pt - 
---------------------------------------------------------------------------- */
static SBoolean deform_frame_pt (SegmentStruct* seg, double* pt, DeformObject* ignoreDfm)
{
   double undeformedPt[3];
   int i;
   SBoolean didDeform = no;
   
   _COPY_VERT(undeformedPt, pt);
   
   for (i = 0; i < seg->num_deforms; i++)
      if (seg->deform[i].active && &seg->deform[i] != ignoreDfm)
         deform_vert(&seg->deform[i], undeformedPt, pt, &didDeform);
   
   return didDeform;
}


/* -------------------------------------------------------------------------
   deform_joint_pt - 
---------------------------------------------------------------------------- */
static SBoolean deform_joint_pt (ModelStruct* ms, JointStruct* jnt, double* pt)
{
   SegmentStruct* seg = &ms->segment[jnt->from];
   double undeformedPt[3];
   int i;
   SBoolean didDeform = no;

   convert(ms, pt, jnt->to, jnt->from);
   
   _COPY_VERT(undeformedPt, pt);
   
   for (i = 0; i < seg->num_deforms; i++)
      if (seg->deform[i].active)
         deform_vert(&seg->deform[i], undeformedPt, pt, &didDeform);
   
   convert(ms, pt, jnt->from, jnt->to);
   
   return didDeform;
}
/* -------------------------------------------------------------------------
   deform_frame_pt2 - 
---------------------------------------------------------------------------- */
static SBoolean deform_frame_pt2 (DeformObject* dfm, const double* undeformed, double* pt)
{
   SBoolean didDeform = no;
   
   if (dfm->active)
      deform_vert(dfm, undeformed, pt, &didDeform);
   
   return didDeform;
}

/* -------------------------------------------------------------------------
   deform_deform_reference_frame - this routine applies all deformations
      preceeding 'dfm' to the reference frame of the 'dfm' deform.
---------------------------------------------------------------------------- */
static void deform_deform_reference_frame (SegmentStruct* seg, DeformObject* dfm, double result[][4], SBoolean includeAutoResetDfm)
{
   DMatrix m1 = {
      { 0.0005, 0.0000, 0.0000, 0.0 },   /* mini reference frame - UNDEFORMED */
      { 0.0000, 0.0005, 0.0000, 0.0 },
      { 0.0000, 0.0000, 0.0005, 0.0 },
      { 0.0000, 0.0000, 0.0000, 1.0 }};
      
   DMatrix m2 = {
      { 0.0005, 0.0000, 0.0000, 0.0 },   /* mini reference frame */
      { 0.0000, 0.0005, 0.0000, 0.0 },
      { 0.0000, 0.0000, 0.0005, 0.0 },
      { 0.0000, 0.0000, 0.0000, 1.0 }};
      
   int i,j;
   
   for (i = 0; i < seg->num_deforms; i++)
   {
      DeformObject* deformer = &seg->deform[i];
      
      if (deformer == dfm)
         break;
      
      if (deform_frame_pt2(deformer, m1[3], m2[3]))
         for (j = 0; j < 3; j++)
            deform_frame_pt2(deformer, m1[j], m2[j]);
   }
   
   if (includeAutoResetDfm)
   {
      DeformObject* autoDfm = NULL;
      
      for (i = 0; i < seg->num_deforms; i++)
         if (seg->deform[i].autoReset)
            break;
   
      if (i < seg->num_deforms)
         autoDfm = &seg->deform[i];
      
      if (autoDfm)
         if (deform_frame_pt2(autoDfm, m1[3], m2[3]))
            for (j = 0; j < 3; j++)
               deform_frame_pt2(autoDfm, m1[j], m2[j]);
   }

   for (j = 0; j < 3; j++)
   {
      _MAKE_VEC(m2[j], m2[j], m2[3]);
      normalize_vector(m2[j], m2[j]);
   }
   
   copy_4x4matrix(m2, result);
}

/* -------------------------------------------------------------------------
   calc_auto_reset_deform - check for an auto-reset deform in the
     specified segment.  If one is found, then compute its deforming
     transform here.
---------------------------------------------------------------------------- */
static void calc_auto_reset_deform (SegmentStruct* seg, DeformObject* autoDfm)
{
   int i;
   
   if (autoDfm == NULL)
   {
      for (i = 0; i < seg->num_deforms; i++)
         if (seg->deform[i].autoReset)
            break;
   
      if (i < seg->num_deforms)
         autoDfm = &seg->deform[i];
   }
   
   if (autoDfm)
   {
      DMatrix m = {
         { 0.0001, 0.0000, 0.0000, 0.0 },   /* mini reference frame */
         { 0.0000, 0.0001, 0.0000, 0.0 },
         { 0.0000, 0.0000, 0.0001, 0.0 },
         { 0.0000, 0.0000, 0.0000, 1.0 }};
      
      if (deform_frame_pt(seg, m[3], autoDfm))
      {
         for (i = 0; i < 3; i++)
         {
            if ( ! autoDfm->translationOnly)
            {
              deform_frame_pt(seg, m[i], autoDfm);
              _MAKE_VEC(m[i], m[i], m[3]);
            }
            
            normalize_vector(m[i], m[i]);
         }
      
         /* NOTE:  the deformed reference frame may have lost its orthogonality
          *   at this point.  We should write a routine to straighten it out:
          *
          * reorthogonalize_matrix(m);
          */
         
         invert_4x4transform(m, autoDfm->delta_xform);
      }
      else
         identity_matrix(autoDfm->delta_xform);
   }
}

/* -------------------------------------------------------------------------
   lookup_deform - 
---------------------------------------------------------------------------- */
DeformObject* lookup_deform (ModelStruct* ms, const char* deformName)
{
   int i,j;
   
   for (i = 0; i < ms->numsegments; i++)
      
      for (j = 0; j < ms->segment[i].num_deforms; j++)
         
         if (ms->segment[i].deform[j].name && STRINGS_ARE_EQUAL(deformName, ms->segment[i].deform[j].name))
            
            return &ms->segment[i].deform[j];

   return NULL;
}


/* -------------------------------------------------------------------------
   recalc_deform_xforms - recalculate the specified deform's various matrices.
---------------------------------------------------------------------------- */
void recalc_deform_xforms (SegmentStruct* seg, DeformObject* dfm)
{
   if ( ! dfm->position.xforms_valid)
   {
      /* calculate the deform placement transform */
      identity_matrix(dfm->position.from_local_xform);

      if (dfm->position.rotation_angle != 0.0)
         rotate_matrix_axis_angle(dfm->position.from_local_xform, dfm->position.rotation_axis.xyz, dfm->position.rotation_angle);
   
      translate_matrix(dfm->position.from_local_xform, dfm->position.translation.xyz);
   
      invert_4x4transform(dfm->position.from_local_xform, dfm->position.to_local_xform);
   
      dfm->position.xforms_valid = yes;
   }

#if 0   
   /* SPECIAL-CASE: auto-reset deforms can be used to automatically deform
    *  a segment's origin back to its original (undeformed) position.  There
    *  should only be one auto-reset deform per segment, and it should be
    *  the last deform for the segment.  We don't calculate the auto-deform's
    *  deforming transforms in this routine.  This calculation is deferred
    *  until vertices are actually being deformed (see calc_auto_reset_deform()).
    */
   if (dfm->autoReset)
   {
      return;
   }
#endif
   
   if ( ! dfm->deform_start.xforms_valid && ! dfm->autoReset)
   {
      /* calculate the deform start transform */
      identity_matrix(dfm->deform_start.from_local_xform);

      if (dfm->deform_start.rotation_angle != 0.0)
         rotate_matrix_axis_angle(dfm->deform_start.from_local_xform, dfm->deform_start.rotation_axis.xyz, dfm->deform_start.rotation_angle);
   
      translate_matrix(dfm->deform_start.from_local_xform, dfm->deform_start.translation.xyz);
   
      invert_4x4transform(dfm->deform_start.from_local_xform, dfm->deform_start.to_local_xform);
   
      dfm->deform_start.xforms_valid = yes;
   }
   
   if ( ! dfm->deform_end.xforms_valid && ! dfm->autoReset)
   {
      /* calculate the deform end transform */
      identity_matrix(dfm->deform_end.from_local_xform);

      if (dfm->deform_end.rotation_angle != 0.0)
         rotate_matrix_axis_angle(dfm->deform_end.from_local_xform, dfm->deform_end.rotation_axis.xyz, dfm->deform_end.rotation_angle);
   
      translate_matrix(dfm->deform_end.from_local_xform, dfm->deform_end.translation.xyz);
   
      invert_4x4transform(dfm->deform_end.from_local_xform, dfm->deform_end.to_local_xform);
   
      dfm->deform_end.xforms_valid = yes;
   }

   /* calculate the current deforming transform */
   {
      dpCoord3D axis, translation;
      double  angle;
      DMatrix ref_frame_xform, ref_frame_xform_inv;
      
      if (dfm->deform_factor == 0.0)
      {
         axis        = dfm->deform_start.rotation_axis;
         angle       = dfm->deform_start.rotation_angle;
         translation = dfm->deform_start.translation;
      }
      else if (dfm->deform_factor == 1.0)
      {
         axis        = dfm->deform_end.rotation_axis;
         angle       = dfm->deform_end.rotation_angle;
         translation = dfm->deform_end.translation;
      }
      //dkb
      else if (dfm->deform_factor == -1.0)
      {
         axis        = dfm->position.rotation_axis;
         angle       = dfm->position.rotation_angle;
         translation = dfm->position.translation;
      }
      else
      {
         lerp_pt(dfm->deform_start.translation.xyz,
                 dfm->deform_end.translation.xyz,
                 dfm->deform_factor, translation.xyz);
   
         slerp(&dfm->deform_start.rotation_axis, dfm->deform_start.rotation_angle,
               &dfm->deform_end.rotation_axis,   dfm->deform_end.rotation_angle,
               dfm->deform_factor, &axis, &angle);
      }
      
      /* NOTE: the code below to build 'delta_xform' and 'delta_xform2' is
       *  not that clear, and unfortunately not that well understood by its
       *  author (me, Kenny).  I arrived at it by partial understanding and
       *  a fair amount of trial and error.
       */

      /* compute ref_frame_xform & ref_frame_xform_inv */
      deform_deform_reference_frame(seg, dfm, ref_frame_xform, no);
      
      invert_4x4transform(ref_frame_xform, ref_frame_xform_inv);
      
      /* compute delta_xform */
      if (dfm->autoReset)
         calc_auto_reset_deform(seg, dfm);
      else
      {
         identity_matrix(dfm->delta_xform);

         append_matrix(dfm->delta_xform, ref_frame_xform_inv);
      
         append_matrix(dfm->delta_xform, dfm->position.to_local_xform);
      
         if (angle != 0.0)
            rotate_matrix_axis_angle(dfm->delta_xform, axis.xyz, angle);
      
         translate_matrix(dfm->delta_xform, translation.xyz);

         append_matrix(dfm->delta_xform, ref_frame_xform);
      }

   /* NOTE: since a change to a deform can affect all deforms that follow,
    *  we must recursively call this routine again for every deform in the
    *  segment's list that follows this one.
    */
   {
      static SBoolean sRecursionBlock = no;
      
      if ( ! sRecursionBlock)
      {
         int i;
         
         sRecursionBlock = yes;
         
         for (i = 0; i < seg->num_deforms; i++)
            if (&seg->deform[i] == dfm)
               break;
         
         for (i++; i < seg->num_deforms; i++)
            recalc_deform_xforms(seg, &seg->deform[i]);
      
         /* compute delta_xform2 and delta_xform3 */
         identity_matrix(dfm->delta_xform2);
      
         if (angle != 0.0)
            rotate_matrix_axis_angle(dfm->delta_xform2, axis.xyz, angle);
      
         translate_matrix(dfm->delta_xform2, translation.xyz);
      
         copy_4x4matrix(dfm->delta_xform2, dfm->delta_xform3);
      
         deform_deform_reference_frame(seg, dfm, ref_frame_xform, yes);
      
         append_matrix(dfm->delta_xform3, ref_frame_xform);
         
         sRecursionBlock = no;
      }
   }
   }
}

/* ==============================================================================
 * ==== DEFORM INNER/OUTER BOX REPRESENTATION
 */

/* -------------------------------------------------------------------------
   build_deform_box_edge - 
---------------------------------------------------------------------------- */
static void build_deform_box_edge (const double* c0, const double* c1,
                                   float** vert, int** edge, int* nVerts)
{
   double delta[3];
   int i,j;
   
   delta[0] = (c1[0] - c0[0]) / SEGMENTS_PER_BOX_EDGE;
   delta[1] = (c1[1] - c0[1]) / SEGMENTS_PER_BOX_EDGE;
   delta[2] = (c1[2] - c0[2]) / SEGMENTS_PER_BOX_EDGE;
   
   for (i = 0; i <= SEGMENTS_PER_BOX_EDGE; i++)
   {
      for (j = 0; j < 3; j++)
      {
         *(*vert)++ = c0[j] + i * delta[j];
         
         if (edge)
            *(*edge)++ = (*nVerts)++;
      }
   }
}

/* -------------------------------------------------------------------------
   init_deform_box_verts - 
---------------------------------------------------------------------------- */
void init_deform_box_verts (DeformObject* dfm)
{
   enum { MINX, MINY, MINZ, MAXX, MAXY, MAXZ };
   
   static const char m[12][6] = { { MINX, MINY, MINZ,  MAXX, MINY, MINZ },
                                  { MINX, MAXY, MINZ,  MAXX, MAXY, MINZ },
                                  { MINX, MAXY, MAXZ,  MAXX, MAXY, MAXZ },
                                  { MINX, MINY, MAXZ,  MAXX, MINY, MAXZ },
                                  { MAXX, MINY, MINZ,  MAXX, MAXY, MINZ },
                                  { MAXX, MINY, MAXZ,  MAXX, MAXY, MAXZ },
                                  { MINX, MINY, MAXZ,  MINX, MAXY, MAXZ },
                                  { MINX, MINY, MINZ,  MINX, MAXY, MINZ },
                                  { MAXX, MINY, MINZ,  MAXX, MINY, MAXZ },
                                  { MAXX, MAXY, MAXZ,  MAXX, MAXY, MINZ },
                                  { MINX, MAXY, MAXZ,  MINX, MAXY, MINZ },
                                  { MINX, MINY, MINZ,  MINX, MINY, MAXZ } };
   
   double *corner, start[3], end[3];
   float  *vert;
   int    i, nBytes = VERTS_PER_BOX * 3 * sizeof(float);
   
   /* alloc (if necessary) and init inner box vertices */
   if (dfm->innerBox == NULL)
      dfm->innerBox = (float*) simm_malloc(nBytes);
   
   if (dfm->innerBoxUndeformed == NULL)
      dfm->innerBoxUndeformed = (float*) simm_malloc(nBytes);
   
   if (dfm->innerBox && dfm->innerBoxUndeformed)
   {
      corner = &dfm->innerMin.xyz[0];
      vert   = dfm->innerBoxUndeformed;
    
      for (i = 0; i < 12; i++)
      {
         _SET_VERT(start, corner[m[i][0]], corner[m[i][1]], corner[m[i][2]]);
         _SET_VERT(end,   corner[m[i][3]], corner[m[i][4]], corner[m[i][5]]);
         
         build_deform_box_edge(start, end, &vert, NULL, NULL);
      }
      memcpy(dfm->innerBox, dfm->innerBoxUndeformed, nBytes);
   }
   
   /* alloc (if necessary) and init outer box vertices */
   if (dfm->outerBox == NULL)
      dfm->outerBox = (float*) simm_malloc(nBytes);
   
   if (dfm->outerBoxUndeformed == NULL)
      dfm->outerBoxUndeformed = (float*) simm_malloc(nBytes);
   
   if (dfm->outerBox && dfm->outerBoxUndeformed)
   {
      corner = &dfm->outerMin.xyz[0];
      vert   = dfm->outerBoxUndeformed;
    
      for (i = 0; i < 12; i++)
      {
         _SET_VERT(start, corner[m[i][0]], corner[m[i][1]], corner[m[i][2]]);
         _SET_VERT(end,   corner[m[i][3]], corner[m[i][4]], corner[m[i][5]]);
         
         build_deform_box_edge(start, end, &vert, NULL, NULL);
      }
      memcpy(dfm->outerBox, dfm->outerBoxUndeformed, nBytes);
   }
}

