/*******************************************************************************

   CONVERT.C

   Authors: Peter Loan, Scott Delp

   Date: 6-APR-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.


   Description: This file contains routines which define the transformations
      between model segments. That is, they make and control
      the 4x4 matrices needed to travel across each joint.

      See the Programmer's Manual for a description of how body-fixed
      transformations work. Basically, when you do a transformation (rotation
      or translation), you move the coordinate system along with the
      object, so the next transformation is done with respect to the new
      coordinate system.
      In SIMM a joint is defined by one translation and three rotations.
      The user is free to do these transformations in any order he chooses,
      so the final joint transformation matrix is made by multiplying together
      the four transformation matrices in the user-defined order. When a given
      rotation matrix is made, the X, Y, and Z axes are rotated too, so the
      next transformation matrix can be built properly. The translation
      matrix is a bit special, because it typically moves the object without
      moving the coordinate system.
      Consider a square centered on the origin of its local coordinate system.
      If you wanted to move it along the X axis, and then spin it around its
      Z axis (its own center), the appropriate GL transformation commands would be
      translate(x,0,0) and then rotate(angle,'Z'). You could then change the
      value of "angle" to spin the object about its center. If you tried to
      do the rotation first, it would rotate the X axis, so the subsequent
      translation would be along this new axis, not the original X axis.
      If you tried to duplicate the desired transformation by creating your own
      4x4 matrix and putting it on the stack (i.e. without using the GL
      commands translate() and rotate()), you would encounter some problems.
      If you generated the translation matrix, and then multiplied it by the
      rotation matrix, the object would seem to spin around the original
      (world) Z axis, not the object's local Z axis. This is because the
      translation matrix effectively moves the points in the object away
      from its coordianate system, so the subsequent rotation is different
      (than what you want). When you want to rotate the object, what you
      need to do is translate the object back to the origin, do the rotation,
      then translate back to where you were. So your matrices look like
      [T] [INV(T) R T*], instead of just T [R]. The translation after the
      rotation is T* because it is a translation within the newly rotated
      coordinate system, not just the negative of the previous translation.
      If you were to do more than one rotation, your matrix string might be:
      [T] [INV(T) R1 T*] [INV(T*) R2 T**], which is equal to [R1] [R2] [T**].
      Thus to do the final translation, you need to use T**, the translation
      in the twice-rotated coordinate system. In this way, all the transformations
      are body-fixed. Equivalently, you could make the final translation not
      body-fixed so you do not need to compute T**. Just use T instead.
      So you always want to do the translation last. But if the translation is
      after some rotations (in the user-defined order), you need to translate
      along a rotated axis, so you want to compute the translation matrix in
      order, but then save it until the end to add it onto the transformation
      stack.

   Routines:
      convert         : converts a 3-D point from one ref. frame to another
      get_conversion  : finds the matrix needed to travel across a joint
      make_conversion : governs the making of the transformation matrices
      make_body_fixed_transform : makes a body-fixed transformation matrix

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"

#include "defunctions.h"

#if INCLUDE_SKIN_EDITOR
  #include "skfunctions.h"
#endif

/*************** DEFINES (for this file only) *********************************/



/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static double world_x[] = {1.0, 0.0, 0.0, 0.0};
static double world_y[] = {0.0, 1.0, 0.0, 0.0};
static double world_z[] = {0.0, 0.0, 1.0, 0.0};


/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void make_joint_conversion(ModelStruct* ms, int joint);

/* CONVERT: */

int convert(ModelStruct* ms, double p[], int n1, int n2)
{
   int i;
   int* path;
   double result[3];
   DMatrix* mat;

   if (n1 == n2)
      return (0);

   if (n1 == ms->ground_segment)
   {
      mat = get_ground_conversion(ms,n2,from_ground);
      result[0] = p[0]*(*mat)[0][0] + p[1]*(*mat)[1][0] +
         p[2]*(*mat)[2][0] + (*mat)[3][0];
      result[1] = p[0]*(*mat)[0][1] + p[1]*(*mat)[1][1] +
         p[2]*(*mat)[2][1] + (*mat)[3][1];
      result[2] = p[0]*(*mat)[0][2] + p[1]*(*mat)[1][2] +
         p[2]*(*mat)[2][2] + (*mat)[3][2];
      p[0] = result[0];
      p[1] = result[1];
      p[2] = result[2];
      return (0);
   }
   else if (n2 == ms->ground_segment)
   {
      mat = get_ground_conversion(ms,n1,to_ground);
      result[0] = p[0]*(*mat)[0][0] + p[1]*(*mat)[1][0] +
         p[2]*(*mat)[2][0] + (*mat)[3][0];
      result[1] = p[0]*(*mat)[0][1] + p[1]*(*mat)[1][1] +
         p[2]*(*mat)[2][1] + (*mat)[3][1];
      result[2] = p[0]*(*mat)[0][2] + p[1]*(*mat)[1][2] +
         p[2]*(*mat)[2][2] + (*mat)[3][2];
      p[0] = result[0];
      p[1] = result[1];
      p[2] = result[2];
      return (0);
   }

   // Performance enhancements for See System 9/8/93:
   // This routine used to copy p[] to a 1x4 vector, so that it can be multiplied
   // by the 4x4 transformation matrices.  Since the last column of the matrices
   // is always [0 0 0 1], the code was changed to use the original 1x3 vector
   // and ignore the last row and column of the 4x4 transformation matrices.

   path = GET_PATH(ms, n1, n2);

   for (i=0; path[i] != ms->numjoints+1; i++)
   {
      /* If you want to convert a point in A to B, and the joint is defined A -> B,
      * then you need to use the inverse transformation matrix.
      */

      if (path[i] > 0)
         mat = get_conversion(ms,path[i]-1,INVERSE);
      else
         mat = get_conversion(ms,-path[i]-1,FORWARD);

      result[0] = p[0]*(*mat)[0][0] + p[1]*(*mat)[1][0] +
         p[2]*(*mat)[2][0] + (*mat)[3][0];
      result[1] = p[0]*(*mat)[0][1] + p[1]*(*mat)[1][1] +
         p[2]*(*mat)[2][1] + (*mat)[3][1];
      result[2] = p[0]*(*mat)[0][2] + p[1]*(*mat)[1][2] +
         p[2]*(*mat)[2][2] + (*mat)[3][2];

      p[0] = result[0];
      p[1] = result[1];
      p[2] = result[2];
   }

   return (i);
}

int convertNEW(ModelStruct* ms, double p[], int path[], int len)
{
   int i;
   double result[3];
   DMatrix* mat;

   for (i=0; i < len; i++)
   {
      if (path[i] > 0)
         mat = get_conversion(ms, path[i]-1, INVERSE);
      else
         mat = get_conversion(ms, -path[i]-1, FORWARD);

      result[0] = p[0]*(*mat)[0][0] + p[1]*(*mat)[1][0] +
         p[2]*(*mat)[2][0] + (*mat)[3][0];
      result[1] = p[0]*(*mat)[0][1] + p[1]*(*mat)[1][1] +
         p[2]*(*mat)[2][1] + (*mat)[3][1];
      result[2] = p[0]*(*mat)[0][2] + p[1]*(*mat)[1][2] +
         p[2]*(*mat)[2][2] + (*mat)[3][2];

      p[0] = result[0];
      p[1] = result[1];
      p[2] = result[2];
   }

   return i;
}



/* Converts a vector from one reference frame to another */

int convert_vector(ModelStruct* ms, double p[], int n1, int n2)
{
   int i;
   int* path;
   double origin[3], result[3], new_origin[3];
   DMatrix* mat;

   if (n1 == n2)
      return (0);

   origin[0] = origin[1] = origin[2] = 0.0;

   if (n1 == ms->ground_segment)
   {
      mat = get_ground_conversion(ms,n2,from_ground);
      result[0] = p[0]*(*mat)[0][0] + p[1]*(*mat)[1][0] +
              p[2]*(*mat)[2][0] + (*mat)[3][0];
      result[1] = p[0]*(*mat)[0][1] + p[1]*(*mat)[1][1] +
              p[2]*(*mat)[2][1] + (*mat)[3][1];
      result[2] = p[0]*(*mat)[0][2] + p[1]*(*mat)[1][2] +
              p[2]*(*mat)[2][2] + (*mat)[3][2];
      new_origin[0] = origin[0]*(*mat)[0][0] + origin[1]*(*mat)[1][0] +
                  origin[2]*(*mat)[2][0] + (*mat)[3][0];
      new_origin[1] = origin[0]*(*mat)[0][1] + origin[1]*(*mat)[1][1] +
                  origin[2]*(*mat)[2][1] + (*mat)[3][1];
      new_origin[2] = origin[0]*(*mat)[0][2] + origin[1]*(*mat)[1][2] +
                  origin[2]*(*mat)[2][2] + (*mat)[3][2];
      p[0] = result[0] - new_origin[0];
      p[1] = result[1] - new_origin[1];
      p[2] = result[2] - new_origin[2];
      return (0);
   }
   else if (n2 == ms->ground_segment)
   {
      mat = get_ground_conversion(ms,n1,to_ground);
      result[0] = p[0]*(*mat)[0][0] + p[1]*(*mat)[1][0] +
              p[2]*(*mat)[2][0] + (*mat)[3][0];
      result[1] = p[0]*(*mat)[0][1] + p[1]*(*mat)[1][1] +
              p[2]*(*mat)[2][1] + (*mat)[3][1];
      result[2] = p[0]*(*mat)[0][2] + p[1]*(*mat)[1][2] +
              p[2]*(*mat)[2][2] + (*mat)[3][2];
      new_origin[0] = origin[0]*(*mat)[0][0] + origin[1]*(*mat)[1][0] +
                  origin[2]*(*mat)[2][0] + (*mat)[3][0];
      new_origin[1] = origin[0]*(*mat)[0][1] + origin[1]*(*mat)[1][1] +
                  origin[2]*(*mat)[2][1] + (*mat)[3][1];
      new_origin[2] = origin[0]*(*mat)[0][2] + origin[1]*(*mat)[1][2] +
                  origin[2]*(*mat)[2][2] + (*mat)[3][2];
      p[0] = result[0] - new_origin[0];
      p[1] = result[1] - new_origin[1];
      p[2] = result[2] - new_origin[2];
      return (0);
   }

   /* Performance enhancements for See System 9/8/93:
    * This routine used to copy p[] to a 1x4 vector, so that it can be multiplied
    * by the 4x4 transformation matrices.  Since the last column of the matrices
    * is always [0 0 0 1], the code was changed to use the original 1x3 vector
    * and ignore the last row and column of the 4x4 transformation matrices.
    */

   path = GET_PATH(ms,n1,n2);

   for (i=0; path[i] != ms->numjoints+1; i++)
   {
      /* If you want to convert a point in A to B, and the joint is defined A -> B,
       * then you need to use the inverse transformation matrix.
       */

      if (path[i] > 0)
     mat = get_conversion(ms,path[i]-1,INVERSE);
      else
     mat = get_conversion(ms,-path[i]-1,FORWARD);

      result[0] = p[0]*(*mat)[0][0] + p[1]*(*mat)[1][0] +
              p[2]*(*mat)[2][0] + (*mat)[3][0];
      result[1] = p[0]*(*mat)[0][1] + p[1]*(*mat)[1][1] +
              p[2]*(*mat)[2][1] + (*mat)[3][1];
      result[2] = p[0]*(*mat)[0][2] + p[1]*(*mat)[1][2] +
              p[2]*(*mat)[2][2] + (*mat)[3][2];
      new_origin[0] = origin[0]*(*mat)[0][0] + origin[1]*(*mat)[1][0] +
                  origin[2]*(*mat)[2][0] + (*mat)[3][0];
      new_origin[1] = origin[0]*(*mat)[0][1] + origin[1]*(*mat)[1][1] +
                  origin[2]*(*mat)[2][1] + (*mat)[3][1];
      new_origin[2] = origin[0]*(*mat)[0][2] + origin[1]*(*mat)[1][2] +
                  origin[2]*(*mat)[2][2] + (*mat)[3][2];
      p[0] = result[0] - new_origin[0];
      p[1] = result[1] - new_origin[1];
      p[2] = result[2] - new_origin[2];
   }

   return (i);

}


/* GET_TRANSFORM_BETWEEN_SEGS: This routine returns the 4x4 transform
 * matrix between segment n1 and n2.
 */

void get_transform_between_segs(ModelStruct* ms, double tmat[][4], int n1, int n2)
{

   if (n1 == n2)
   {
       reset_4x4matrix(tmat);
   }
   else if (n1 == ms->ground_segment)
   {
       memcpy(tmat, *get_ground_conversion(ms, n2, from_ground), 16 * sizeof(double));
   }
   else if (n2 == ms->ground_segment)
   {
       memcpy(tmat, *get_ground_conversion(ms, n1, to_ground), 16 * sizeof(double));
   }
   else
   {
       memcpy(tmat, *get_ground_conversion(ms, n1, to_ground), 16 * sizeof(double));

       append_matrix(tmat, *get_ground_conversion(ms, n2, from_ground));
   }

}


void invalidate_joint_matrix(ModelStruct* model, JointStruct* joint)
{
   int i;

   // Make the joint matrix invalid.
   joint->conversion.condition = invalid;

   // Now invalidate the ground-to-segment transforms of every segment that
   // uses this joint to get to ground.
   for (i=0; i<model->numsegments; i++)
   {
#if INCLUDE_SKIN_EDITOR
      sk_invalidate_bone_xform(model, i);
#endif
      if (joint->in_seg_ground_path[i] == yes)
         model->segment[i].ground_condition = invalid;
   }

   // Now set all of the muscle wrap flags to invalid (wrap_calced = no)
   // so that the wrapping points (implicit and explicit) will be updated.
#if ! ENGINE
   for (i=0; i<model->nummuscles; i++)
      model->muscle[i]->wrap_calced = no;
#endif
}


void invalidate_joints_using_func(ModelStruct* ms, dpFunction* function)
{
   int i, j;

   for (i=0; i<ms->numjoints; i++)
   {
      for (j=0; j<6; j++)
      {
         if (ms->joint[i].dofs[j].type == function_dof && ms->joint[i].dofs[j].function == function)
         {
            invalidate_joint_matrix(ms, &ms->joint[i]);
            break;
         }
      }
   }
}


/* GET_CONVERSION: this routine returns the 4x4 transformation matrix needed
 * to travel down the specified joint in the specified direction.
 */
DMatrix* get_conversion(ModelStruct* ms, int joint, Direction dir)
{
   if (joint > ms->numjoints)
   {
      (void)sprintf(errorbuffer, "Fatal error: joint %d not valid for model %s", joint, ms->name);
      error(exit_program, errorbuffer);
   }

   make_conversion(ms,joint);
   
   if (dir == FORWARD)
      return (&ms->joint[joint].conversion.forward);
   else /* if (dir == INVERSE) */
      return (&ms->joint[joint].conversion.inverse);
}

#if ! ENGINE
GLfloat* get_float_conversion(ModelStruct* ms, int joint, Direction dir)
{
   if (joint > ms->numjoints)
   {
      (void)sprintf(errorbuffer, "Fatal error: joint %d not valid for model %s", joint, ms->name);
      error(exit_program, errorbuffer);
   }

   make_conversion(ms,joint);

   if (dir == FORWARD)
      return (ms->joint[joint].conversion.gl_forward);
   else /* if (dir == INVERSE) */
      return (ms->joint[joint].conversion.gl_inverse);
}
#endif


/* GET_GROUND_CONVERSION: this routine returns the 4x4 transformation matrix needed
 * to travel between the specified segment and the ground segment.
 */

DMatrix* get_ground_conversion(ModelStruct* ms, int seg, GroundDirection gd)
{
   if (ms->segment[seg].ground_condition == invalid)
      make_ground_conversion(ms,seg);

   if (gd == from_ground)
      return (&ms->segment[seg].from_ground); 
   else /* if (gd == to_ground) */
      return (&ms->segment[seg].to_ground);

}



/* GET_FLOAT_GROUND_CONVERSION: this routine returns the 4x4 transformation
 * matrix needed to travel between the specified segment and the ground segment.
 */
#if ! ENGINE
GLfloat* get_float_ground_conversion(ModelStruct* ms, int seg, GroundDirection gd)
{
   if (ms->segment[seg].ground_condition == invalid)
      make_ground_conversion(ms,seg);

   if (gd == from_ground)
      return (ms->segment[seg].float_from_ground); 
   else /* if (gd == to_ground) */
      return (ms->segment[seg].float_to_ground);

}
#endif


void make_ground_conversion(ModelStruct* ms, int seg)
{

   int i, j, index;

   calc_transformation(ms,ms->ground_segment, seg, ms->segment[seg].from_ground);
   invert_4x4transform(ms->segment[seg].from_ground, ms->segment[seg].to_ground);

#if ! ENGINE
   for (i=0,index=0; i<4; i++)
   {
      for (j=0; j<4; j++)
      {
         ms->segment[seg].float_from_ground[index] = ms->segment[seg].from_ground[i][j];
         ms->segment[seg].float_to_ground[index++] = ms->segment[seg].to_ground[i][j];
      }
   }
#endif

   ms->segment[seg].ground_condition = valid;

}



/* MAKECONVERSION: this guy forms the conversion matrices which
 * define the transformation between the segments in a joint. Each joint
 * has a forward and an inverse matrix, so you can "travel" both ways
 * across a joint.
 */

void make_conversion(ModelStruct* ms, int joint)
{
   JointStruct* jnt = &ms->joint[joint];
   int i, j, index;

   if (jnt->conversion.condition == valid &&
       (jnt->pretransform_active == no || jnt->pretransform_condition == valid))
   {
      return;
   }
   
   /* if the kinematic transform is invalid and the pretransform is active,
    * then we need to invalidate the pretransform as well because the change
    * in kinematic transform may move the joint origin to a new location,
    * thereby affecting the amount that it is deformed.
    */
   if (jnt->conversion.condition == invalid && jnt->pretransform_active)
      jnt->pretransform_condition = invalid;

   if (jnt->conversion.condition == invalid)
   {
      make_joint_conversion(ms, joint);

      if (jnt->pretransform_active)
      {
         /* if this segment has any deformed ancestors, the prepend their
          * deforming transforms, if any:
          */
         DMatrix m;
      
         if (jnt->pretransform_condition == invalid)
         {
            calc_joint_pretransform(ms, jnt);
            
            jnt->pretransform_condition = valid;
         }
   
         copy_4x4matrix(jnt->conversion.forward, m);
         mult_4x4matrices(jnt->pretransform, m, jnt->conversion.forward);
   
         copy_4x4matrix(jnt->conversion.inverse, m);
         mult_4x4matrices(m, jnt->pretransform_inverse, jnt->conversion.inverse);
      }
   }
   /* Now copy the forward and inverse matrices to float versions so they can
    * be sent down the graphics pipeline via glMultMatrixf(). TODO: double
    * check this code. OpenGL wants column-major matrices and IRIS GL wanted
    * row-major.
    */
#if ! ENGINE
   for (i=0, index=0; i<4; i++)
      for (j=0; j<4; j++)
      {
         ms->joint[joint].conversion.gl_forward[index] = 
            ms->joint[joint].conversion.forward[i][j];
         ms->joint[joint].conversion.gl_inverse[index++] = 
            ms->joint[joint].conversion.inverse[i][j];
      }
#endif
}


static void make_joint_conversion(ModelStruct* ms, int joint)
{
   int i;
   int order[4];
   double dofvalue[6];
   double x[4], y[4], z[4], ra1[4], ra2[4], ra3[4];
   
   /* calculate the values of the 6 dof variables for this joint */
   for (i=0; i<6; i++)
      dofvalue[i] = evaluate_dof(ms,&ms->joint[joint].dofs[i]);

   /* initialize the [parent] x, y, and z axes */
   COPY_1X4VECTOR(world_x,ms->joint[joint].parentframe[XX]);
   COPY_1X4VECTOR(world_y,ms->joint[joint].parentframe[YY]);
   COPY_1X4VECTOR(world_z,ms->joint[joint].parentframe[ZZ]);

   /* call calc_joint_transform() to make the matrices */

   /***************** FORWARD **********************/

   /* set the order, copy the parent frame and parent axes to other vectors */
   order[TRANS] = ms->joint[joint].order[TRANS]+1;
   order[ROT1] = ms->joint[joint].order[ROT1]+1;
   order[ROT2] = ms->joint[joint].order[ROT2]+1;
   order[ROT3] = ms->joint[joint].order[ROT3]+1;
   COPY_1X4VECTOR(ms->joint[joint].parentframe[XX],x);
   COPY_1X4VECTOR(ms->joint[joint].parentframe[YY],y);
   COPY_1X4VECTOR(ms->joint[joint].parentframe[ZZ],z);
   COPY_1X4VECTOR(ms->joint[joint].parentrotaxes[R1],ra1);
   normalize_vector(ra1, ra1);
   COPY_1X4VECTOR(ms->joint[joint].parentrotaxes[R2],ra2);
   normalize_vector(ra2, ra2);
   COPY_1X4VECTOR(ms->joint[joint].parentrotaxes[R3],ra3);
   normalize_vector(ra3, ra3);

   calc_joint_transform(order,dofvalue,ms->joint[joint].conversion.forward,
                        x,y,z,ra1,ra2,ra3,BF,FORWARD,&ms->joint[joint]);

   /* When the axes come back, they are transformed into the child axes,
    * so store them in the child elements in the joint structure.
    */
   COPY_1X4VECTOR(x,ms->joint[joint].childframe[XX]);
   COPY_1X4VECTOR(y,ms->joint[joint].childframe[YY]);
   COPY_1X4VECTOR(z,ms->joint[joint].childframe[ZZ]);
   COPY_1X4VECTOR(ra1,ms->joint[joint].childrotaxes[R1]);
   COPY_1X4VECTOR(ra2,ms->joint[joint].childrotaxes[R2]);
   COPY_1X4VECTOR(ra3,ms->joint[joint].childrotaxes[R3]);


   /***************** INVERSE **********************/

   /* You cannot just invert the forward matrix to get the inverse matrix
    * because you also need to compute the intermediate axes to store in
    * the child portion of the joint structure (so the partial velocity
    * routines can get them).
    */
   order[TRANS] = ms->joint[joint].order[TRANS]-4;
   order[ROT1] = ms->joint[joint].order[ROT1]-4;
   order[ROT2] = ms->joint[joint].order[ROT2]-4;
   order[ROT3] = ms->joint[joint].order[ROT3]-4;

   calc_joint_transform(order,dofvalue,ms->joint[joint].conversion.inverse,
                        x,y,z,ra1,ra2,ra3,BF,INVERSE,&ms->joint[joint]);

   /* and now the condition of the conversion matrices is valid */
   ms->joint[joint].conversion.condition = valid;
}



/* CALC_JOINT_TRANSFORM: Makes a joint transformation matrix by concatenating four transformation
 * matrices, in either a body-fixed format or space-fixed.
 */

void calc_joint_transform(int order[], double dofs[], double rmat[][4], double xaxis[],
                            double yaxis[], double zaxis[], double axis1[], double axis2[],
                            double axis3[], int mode, Direction dir, JointStruct* joint)
{
   int i;
   int absorder[4], ordersigns[4];
   double cl, sl, omc, rvec[4];
   double axis[4];
   double mat[4][4], wmat[4][4], trans_mat[4][4];

   absorder[0] = ABS(order[0]) - 1;
   absorder[1] = ABS(order[1]) - 1;
   absorder[2] = ABS(order[2]) - 1;
   absorder[3] = ABS(order[3]) - 1;

   ordersigns[TRANS] = SIGN(order[TRANS]);
   ordersigns[ROT1] = SIGN(order[ROT1]);
   ordersigns[ROT2] = SIGN(order[ROT2]);
   ordersigns[ROT3] = SIGN(order[ROT3]);

   MAKE_IDENTITY_MATRIX(rmat);
   MAKE_IDENTITY_MATRIX(mat);

   /* We do this loop four times, once for each transformation (one translation,
    * three rotations).
    */
   for (i=0; i<4; i++)
   {
      MAKE_IDENTITY_MATRIX(mat);

      if (i == absorder[TRANS])
      {
         mat[3][0] = dofs[TX]*ordersigns[TRANS]*xaxis[0] +
                     dofs[TY]*ordersigns[TRANS]*yaxis[0] +
                     dofs[TZ]*ordersigns[TRANS]*zaxis[0];
         mat[3][1] = dofs[TX]*ordersigns[TRANS]*xaxis[1] +
                     dofs[TY]*ordersigns[TRANS]*yaxis[1] +
                     dofs[TZ]*ordersigns[TRANS]*zaxis[1];
         mat[3][2] = dofs[TX]*ordersigns[TRANS]*xaxis[2] +
                     dofs[TY]*ordersigns[TRANS]*yaxis[2] +
                     dofs[TZ]*ordersigns[TRANS]*zaxis[2];
      }
      else
      {
         if (i == absorder[ROT1])
         {
            cl = cos(DTOR*dofs[R1]*ordersigns[ROT1]);
            sl = sin(DTOR*dofs[R1]*ordersigns[ROT1]);
            axis[0] = axis1[0];
            axis[1] = axis1[1];
            axis[2] = axis1[2];
            axis[3] = 0.0;
         }
         else if (i == absorder[ROT2])
         {
            cl = cos(DTOR*dofs[R2]*ordersigns[ROT2]);
            sl = sin(DTOR*dofs[R2]*ordersigns[ROT2]);
            axis[0] = axis2[0];
            axis[1] = axis2[1];
            axis[2] = axis2[2];
            axis[3] = 0.0;
         }
         else if (i == absorder[ROT3])
         {
            cl = cos(DTOR*dofs[R3]*ordersigns[ROT3]);
            sl = sin(DTOR*dofs[R3]*ordersigns[ROT3]);
            axis[0] = axis3[0];
            axis[1] = axis3[1];
            axis[2] = axis3[2];
            axis[3] = 0.0;
         }

          /* the following matrix is taken from Kane's 'Spacecraft Dynamics,' pp 6-7 */
         omc = 1.0 - cl;
         mat[0][0] = cl + axis[0]*axis[0]*omc;
         mat[1][0] = -axis[2]*sl + axis[0]*axis[1]*omc;
         mat[2][0] = axis[1]*sl + axis[2]*axis[0]*omc;
         mat[0][1] = axis[2]*sl + axis[0]*axis[1]*omc;
         mat[1][1] = cl + axis[1]*axis[1]*omc;
         mat[2][1] = -axis[0]*sl + axis[1]*axis[2]*omc;
         mat[0][2] = -axis[1]*sl + axis[2]*axis[0]*omc;
         mat[1][2] = axis[0]*sl + axis[1]*axis[2]*omc;
         mat[2][2] = cl + axis[2]*axis[2]*omc;
      }

      /* If the transformation is the translation, do not multiply in into
       * the current working matrix (i.e. do not add it onto the stack of
       * transformation matrices). Since subsequent rotations should be
       * done with respect to the non-translated coordinate system, save
       * the translation matrix for last, so that it is added onto the
       * stack after all the rotations have been added. You want to compute
       * the translation matrix in the correct order, though, so that it
       * uses the "new" axes after any previous rotations. So compute it
       * in order, but add it onto the end of the transformation stack.
       */
      if (i != absorder[TRANS])
      {
         mult_4x4matrices(rmat,mat,wmat);
         copy_4x4matrix(wmat,rmat);
         /* save rmat in right rotation structure (e.g., parentinterrotmat) */
      }
      else
      {
         copy_4x4matrix(mat,trans_mat);
      }

      if (mode == NBF)
         continue;

      /* Now that we've done another transformation, transform the X, Y, Z, and
       * lambda (rotation) axes so that the next transformation uses the new
       * transformed axes (so transformations are body-fixed).
       */
      mult_4x4matrix_by_vector(mat,xaxis,rvec);
      COPY_1X4VECTOR(rvec,xaxis);
      mult_4x4matrix_by_vector(mat,yaxis,rvec);
      COPY_1X4VECTOR(rvec,yaxis);
      mult_4x4matrix_by_vector(mat,zaxis,rvec);
      COPY_1X4VECTOR(rvec,zaxis);
      mult_4x4matrix_by_vector(mat,axis1,rvec);
      COPY_1X4VECTOR(rvec,axis1);
      mult_4x4matrix_by_vector(mat,axis2,rvec);
      COPY_1X4VECTOR(rvec,axis2);
      mult_4x4matrix_by_vector(mat,axis3,rvec);
      COPY_1X4VECTOR(rvec,axis3);

      /* If we did a translation in this iteration, save the X, Y, and Z axes along
       * which we translated. Save them in the joint structure for which we are
       * computing the transformation matrices. If we are computing a forward matrix,
       * save them in the parentinterframe element of the structure. If an inverse
       * matrix, save them in the childinterframe element.
       * If we did a rotation, save the lambda axis about which we just rotated.
       * If we are computing a forward matrix, save it in the parentinterrotaxes
       * element in the joint structure. If an inverse matrix, save it in the
       * childinterrotaxes element.
       */
      if (dir == FORWARD)
      {
         if (i == absorder[TRANS])
         {
            COPY_1X4VECTOR(xaxis,joint->parentinterframe[0]);
            COPY_1X4VECTOR(yaxis,joint->parentinterframe[1]);
            COPY_1X4VECTOR(zaxis,joint->parentinterframe[2]);
         }
         else if (i == absorder[ROT1])
            COPY_1X4VECTOR(axis1,joint->parentinterrotaxes[0])
         else if (i == absorder[ROT2])
            COPY_1X4VECTOR(axis2,joint->parentinterrotaxes[1])
         else if (i == absorder[ROT3])
            COPY_1X4VECTOR(axis3,joint->parentinterrotaxes[2])

      }
      else if (dir == INVERSE)
      {
         if (i == absorder[TRANS])
         {
            COPY_1X4VECTOR(xaxis,joint->childinterframe[0]);
            COPY_1X4VECTOR(yaxis,joint->childinterframe[1]);
            COPY_1X4VECTOR(zaxis,joint->childinterframe[2]);
         } 
         else if (i == absorder[ROT1])
            COPY_1X4VECTOR(axis1,joint->childinterrotaxes[0])
         else if (i == absorder[ROT2])
            COPY_1X4VECTOR(axis2,joint->childinterrotaxes[1])
         else if (i == absorder[ROT3])
            COPY_1X4VECTOR(axis3,joint->childinterrotaxes[2])
      }
   }

   /* You can now add the translation matrix onto the end (or beginning if INVERSE) */
   if (dir == FORWARD)
      mult_4x4matrices(rmat,trans_mat,wmat);
   else
      mult_4x4matrices(trans_mat,rmat,wmat);

   copy_4x4matrix(wmat,rmat);
}



void calc_transformation(ModelStruct* ms, int from, int to, DMatrix mat)
{

   int i;
   int* path;
   DMatrix* jmat;
   DMatrix workmat1, workmat2;

   path = GET_PATH(ms->modelnum,from,to);

   reset_4x4matrix(workmat1);

   for (i=0; path[i] != ms->numjoints+1; i++)
   {
      if (path[i] > 0)
         jmat = get_conversion(ms,path[i]-1,INVERSE);
      else
         jmat = get_conversion(ms,-path[i]-1,FORWARD);
      mult_4x4matrices(workmat1,*jmat,workmat2);
      copy_4x4matrix(workmat2,workmat1);
   }

   copy_4x4matrix(workmat2,mat);

}
