/*******************************************************************************

   CEFUNCTIONS.H - this header declares the Constraint Editor's public functions
      
   Author: Krystyne Blaikie (based on wefunctions.h by Kenny Smith)

   Date: 8-APR-02

   Copyright (c) 1998 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef CEFUNCTIONS_H
#define CEFUNCTIONS_H

void   makeconstrainteditormenus(void);
void   ce_entervalue(SimmEvent se);
void   update_ce_forms(void);
void   move_ce_help_text(int dummy_int, double slider_value, double delta);

void   ce_track_cb(void* data, SimmEvent se);

void   ce_recalc_loops_and_constraints(ModelStruct* ms, int constraint_object, 
                             SBoolean displayErrorMsg);


int    is_current_constraint_object(ModelStruct*, ConstraintObject*);
int    is_current_constraint_point(ModelStruct* ms, ConstraintObject* co, ConstraintPoint *pt);
void   select_constraint_object(int constraintobj, SBoolean redisplay);
void   select_constraint_point(int coIndex, int ptIndex, SBoolean redisplay);

void   convert_to_constraint_object_frame(ConstraintObject*, double* pt);
void   convert_from_constraint_object_frame(ConstraintObject*, double* pt);

void   reset_constraintobj_xform();

void   apply_xform_to_constraintobj(double factor);
void   clear_ce_xform_form();
void   make_constraint_cylinder (ConstraintObject* co);
void   save_all_constraint_objects(ModelStruct* ms);
int    save_constraint_points(ModelStruct* ms, int constraint_obj);
void   restore_all_constraint_objects(ModelStruct* ms);
int    restore_constraint_points(ModelStruct* ms);
void   delete_constraint_object(ModelStruct* ms, int constraintobj, SBoolean queue_event);
void   delete_constraint_point(ModelStruct* ms, int constraintpt, SBoolean queue_event);

void   update_ce_win_status();

double calc_distance_to_ellipsoid(double p1[], double radii[], double projpt[]);

#endif /* CEFUNCTIONS_H */
