/*******************************************************************************

   DP420.H

   Author: Peter Loan

   Copyright (c) 2004-5 MusculoGraphics, Inc, a division of Motion Analysis Corp.
   All rights reserved.

   This header file defines the data structures used to hold a musculoskeletal
   model, as well as the interface between SIMM the DLLs made by the Dynamics
   Pipeline. It is the version of the header file from SIMM 4.2 (and 4.2.1, since
   there were no changes made in 4.2.1). It is included in later versions of SIMM
   to support the loading of DLLs made by SIMM 4.2 and 4.2.1 into SIMM 5.0 and later.
   This copy of the header has been modified as follows:
     1. data structures that did not change between 4.2 and 5.0 have been removed
        (so that they do not need to be renamed)
     2. data structures that did change have been renamed

*******************************************************************************/

#ifndef DP420_H
#define DP420_H

#ifdef _WINDOWS
    #define DLL __declspec(dllexport)
#else
    #define DLL
#endif

#include "dp.h"


typedef struct
{
   dpFunctionType type;              /* step, linear, gcv, natural cubic */
   double cutoff_frequency;          /* used only for gcv_spline */
   int numpoints;                    /* number of points in the function */
   int coefficient_array_size;       /* current size of x,y,b,c,d arrays */
   int usernum;                      /* user-supplied number for this function */
   double* x;                        /* list of x coordinates */
   double* y;                        /* list of y coordinates */
   double* b;                        /* list of b coefficients for spline fit */
   double* c;                        /* list of c coefficients for spline fit */
   double* d;                        /* list of d coefficients for spline fit */
   dpBoolean defined;                /* is function defined in joint file? */
} dp420SplineFunction;               /* description of a cubic-spline function*/


typedef struct
{
   int genc;                         /* gencoord number */
   double start;                     /* starting value of range-of-motion */
   double end;                       /* ending value of range-of-motion */
} dp420PointRange;                   /* muscle point range (wrapping pts) */


typedef struct
{
   int segment;                      /* body segment this point is fixed to */
   int refpt;                        /* corresponding point in saved muscle */
   dpBoolean selected;               /* whether or not point has been selected */
   double point[3];                  /* xyz coordinates of the point */
   int numranges;                    /* number of ranges where point is active */
   dp420PointRange* ranges;          /* array of ranges */
   dpBoolean is_auto_wrap_point;     /* was this point calc-ed by auto wrapper? */
   double wrap_distance;             /* stores distance of wrap over object surface */
   double* wrap_pts;                 /* points wrapping over surface */
   int num_wrap_pts;                 /* number of points wrapping over surface */
} dp420MusclePoint;                  /* properties of a muscle point */


typedef struct
{
   char* name;                       /* name of wrap object */
   dpWrapObjectType wrap_type;       /* type of wrap object */
   dpBoolean active;                 /* is this wrap object active? */
   int wrap_algorithm;               /* default wrap algorithm for wrap object */
   int segment;                      /* body segment object is fixed to */
   double radius[3];                 /* wrap object radius */
   double height;                    /* wrap object height (cylinder only) */
   int wrap_axis;                    /* which axis to wrap over X=0, Y=1, Z=2 */
   int  wrap_sign;                   /* which side of wrap axis to use (0 for both) */
   double from_local_xform[4][4];    /* parent-to-wrapobj transform matrix */
   double to_local_xform[4][4];      /* wrapobj-to-parent transform matrix */
} dp420WrapObject;


typedef struct
{
   dp420WrapObject* wrap_object;     /* muscle is auto-wrapped over this object */
   int wrap_algorithm;               /* wrap algorithm for this muscle */
   int startPoint;                   /* wrap only those muscle segments between startPoint */
   int endPoint;                     /* and endPoint */
   double c[3];                      /* previous c point */
   double r1[3];                     /* previous r1 (tangent) point */
   double r2[3];                     /* previous r2 (tangent) point */
   dp420MusclePoint mp_wrap[2];      /* the two muscle points created when the muscle wraps */
} dp420MuscleWrapStruct;


typedef struct
{
   char* name;                       /* name of muscle */
   dpBoolean display;                /* whether or not to display this muscle */
   dpBoolean output;                 /* write muscle values to output file? */
   dpBoolean selected;               /* whether or not this muscle is selected */
   dpBoolean has_wrapping_points;    /* does this muscle have wrapping pts? */
   dpBoolean has_force_points;       /* does this muscle have force-dependent pts? */
   int* num_orig_points;             /* number of muscle points */
   int num_points;                   /* number of muscle points */
   dp420MusclePoint* mp_orig;           /* list of muscle points from muscle file */
   dp420MusclePoint** mp;               /* list of muscle points after auto wrapping */
   int mp_array_size;                /* current size of muscle point array */
   double* max_isometric_force;      /* maximum isometric force */
   double* pennation_angle;          /* pennation angle of muscle fibers */
   double* optimal_fiber_length;     /* muscle fiber length */
   double* resting_tendon_length;    /* resting length of tendon */
   double* max_contraction_vel;      /* maximum contraction velocity */
   double muscle_tendon_length;      /* musculotendon length */
   double fiber_length;              /* muscle fiber length */
   double tendon_length;             /* tendon length */
   double muscle_tendon_vel;         /* musculotendon velocity */
   double fiber_velocity;            /* muscle velocity */
   double tendon_velocity;           /* tendon velocity */
   double force;                     /* force in musculotendon unit */
   double applied_power;             /* power this muscle applies to body segments */
   double heat_energy;               /* heat energy generated by this muscle */
   double mechanical_energy;         /* energy this muscle gives to body segments */
   double energy;                    /* energy this muscle gives to body segments */
   int nummomentarms;                /* number of moment arms (= # of gencoords) */
   double* momentarms;               /* list of moment arm values */
   dp420SplineFunction* tendon_force_len_curve; /* tendon force-length curve */
   dp420SplineFunction* active_force_len_curve; /* muscle active force-length curve */
   dp420SplineFunction* passive_force_len_curve; /* muscle passive force-length curve */
   dp420SplineFunction* force_vel_curve;/* muscle force-velocity curve */
   int num_dynamic_params;           /* size of dynamic_params array */
   char** dynamic_param_names;       /* list of dynamic parameter names */
   double** dynamic_params;          /* array of dynamic (muscle model) parameters */
   double dynamic_activation;        /* dynamic value of muscle activation */
   dpFunctionType* excitation_format; /* format for interpolating excitation data. */
   int excitation_abscissa;          /* excit. is func of this gencoord (-1 = time) */
   double excitation_level;          /* current level of excitation */
   dp420SplineFunction* excitation;     /* excitation (activation) sequence */
   int* muscle_model_index;          /* index for deriv, init, & assign func arrays */
   dpBoolean wrap_calced;            /* has wrapping been calculated/updated? */
   int numWrapStructs;               /* number of wrap objects used for this muscle */
   dp420MuscleWrapStruct** wrapStruct;  /* holds information about the wrap objects */
   int numStateParams;               /* number of initial state values stored in dllparams.txt */
   double* stateParams;              /* array of initial state values in dllparams.txt */
} dp420MuscleStruct;                    /* properties of a musculotendon unit */


typedef struct
{
   char* name;
   dpQType type;
   int joint;
   int axis;
   int q_ind;
   int constraint_num;
   double initial_value;
   double initial_velocity;
   double conversion;
   double range_start;
   double range_end;
   dp420SplineFunction* restraint_func;
   dp420SplineFunction* min_restraint_func;
   dp420SplineFunction* max_restraint_func;
   dp420SplineFunction* constraint_func;
   dpBoolean function_active;
   dpBoolean output;
   double torque;
    double pd_stiffness;
} dp420QStruct;


typedef struct
{
   char* name;
   int nq;
   int nu;
   int neq;
   int num_muscles;
   int num_body_segments;
   int num_gencoords;
   int num_joints;
   int num_closed_loops;
   int num_constraints;
   int num_user_constraints;
   int num_dynamic_params;           /* size of dynamic_param_names array */
   char** dynamic_param_names;       /* names of dynamic muscle parameters */
   int num_contacts;
   int num_bilat_contacts;
   int contacts_size;
   int bilat_contacts_size;
   dpContactInfo* contacts;
   dpContactInfo* bilat_contacts;
   int num_springs;             /* number of spring elements for spring-based contact */
   int num_spring_floors;       /* number of floor elements for spring-based contact */
   int spring_array_size;
   int spring_floor_array_size;
   dpSpringStruct* spring;      /* spring points for use with spring-based contact */
   dpSpringFloor* spring_floor; /* spring floors for use with spring-based contact */
   int num_force_mattes;        /* number of force mattes for applying ground-based forces */
   int force_matte_array_size;
   dpForceMatte* force_matte;   /* force mattes used for applying ground-based forces */
   dp420QStruct* q;
   dpBodyStruct* body_segment;
   dp420MuscleStruct* muscles;
   dp420MuscleStruct default_muscle;
   dpJointStruct* joint;
   int num_wrap_objects;
   dp420WrapObject* wrap_object;
   int num_constraint_objects;
   dpConstraintObject* constraint_object;
   int num_constraint_functions;
   dp420SplineFunction* constraint_function;
   double gravity[3];
   dpSimmModelID simmModel;
   int enforce_constraints;    /* enforce constraint objects while integrating? */
    int newInverseSimulation;   /* new-style inverse with corrective torques? */
} dp420ModelStruct;

#endif /*DP420_H*/
