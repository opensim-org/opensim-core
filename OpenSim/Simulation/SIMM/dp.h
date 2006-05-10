#ifndef _dp_h_
#define _dp_h_

// dp.h
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifdef _WINDOWS
    #define DLL __declspec(dllexport)
#else
    #define DLL
#endif


typedef void* dpSimmModelID;


typedef enum
{
   dpNo,
   dpYes
} dpBoolean;

typedef enum
{
   dpSetup,
   dpFrame,
   dpMessage
} dpDataType;

typedef enum
{
   dpModelWarning,
   dpModelError,
   dpSimWarning,
   dpSimError,
   dpInitialized,
   dpFinished,
   dpOverwritable,
   dpNormal
} dpMessageID;

typedef enum
{
   dpNoNormalization = 0,
   dpNormalizeToOne,
   dpNormalizeToOneHundred
} dpNormalizeOptions;


typedef struct
{
   char* name;
   double xCoord;
} dpMotionEvent;


typedef struct
{
   char* motionName;
   int numElements;
   char** elementNames;
   double startTime;
   double endTime;
   int numFrames;
   int numEvents;
   dpMotionEvent* event;
   float eventColor[3];
} dpDataSetup;


typedef struct
{
   int numElements;
   double* elements;
} dpDataFrame;


typedef struct
{
   dpMessageID id;
   char text[500];
   void* data;
} dpMessageStruct;


typedef struct
{
   dpBoolean muscleActivations;
   dpBoolean muscleLengths;
   dpBoolean muscleFiberLengths;
   dpBoolean muscleFiberVelocities;
   dpBoolean muscleForces;
   dpBoolean contactForces;
   dpBoolean massCenterPositions;
   dpBoolean massCenterVelocities;
   dpBoolean systemEnergy;
   dpBoolean jointReactionForces;
   dpBoolean jointReactionTorques;
   dpBoolean jointTorques;
   dpBoolean springForces;
   int numSpringForces;
   dpBoolean muscleMomentArms;
   dpBoolean muscleJointTorques;
   dpBoolean totalMuscleJointTorques;
   dpBoolean optimizedMuscleActivations;
   dpNormalizeOptions normalization;
	dpBoolean gencoordValues;
	dpBoolean traditionalInverse;
} dpOutputOptions;


typedef struct
{
   double startTime;
   double endTime;
   double stepSize;
   char* workingDir;
   char* inputMotionFile;
   char* outputMotionFile;
   char* boneDir;
   dpOutputOptions outputOptions;
   dpSimmModelID model;
} dpSimulationParameters;


typedef enum
{
	dpPin,
   dpReversePin,
	dpUniversal,
   dpReverseUniversal,
	dpGimbal,
   dpReverseGimbal,
	dpBall,
   dpReverseBall,
	dpSlider,
   dpReverseSlider,
	dpCylindrical,
   dpReverseCylindrical,
	dpPlanar,
   dpReversePlanar,
	dpBushing,
	dpReverseBushing,
	dpBearing,
	dpReverseBearing,
	dpFree,
	dpReverseFree,
	dpWeld,
   dpSkippable,
	dpUnknownJoint
} dpJointType;


typedef enum
{
   dpTranslational,
   dpRotational,
   dpNoDof
} dpDOFType;


typedef enum
{
   dpStepFunction,
   dpNaturalCubic,
   dpGCVSpline,
	dpLinear
} dpSplineType;


typedef enum
{
	dpUnknownContact,
	dpImpact,
	dpRestingContact,
	dpSeparatingContact
} dpCollisionTypes;

typedef struct
{
	int body1;				/* SDFast index of body1 */
	int body2;           /* SDFast index of body2 */
	double pt1[3];			/* contact point location in body1 frame from b1 com */
	double pt2[3];       /* contact point location in body2 frame from b2 com */
	double norm1[3];		/* contact normal on body1 in body1 frame */
	double norm2[3];     /* contact normal on body2 in body2 frame */
	double outward_norm[3];	/* from body1 to body2 in ground frame */
	double vel1[3];		/* velocity of contact point on body1 wrt ground */
	double vel2[3];      /* velocity of contact point on body2 wrt ground */
	double rel_vel[3];	/* relative velocity v2 - v1 */
	double cont_frc[3];	/* contact force or impulse */
	double acc;				/* magnitude of normal relative acceleration */
	double coef_rest;		/* coefficient of restitution */
	double mu_dynamic;	/* dynamic friction coefficient */
	double mu_static;		/* static friction coefficient */
	dpCollisionTypes contact_type;	/* IMPACT, CONTACT, or SEPARATING */
	char type1[30];		/* FACE, VERTEX, or EDGE */
	char type2[30];      /* FACE, VERTEX, or EDGE */
	double dist;			/* for ideal contacts along outward normal */
} dpContactInfo;		   /* contact node */


typedef struct
{
   double a, b, c, d;
} dpPlaneStruct;


typedef struct
{
   dpSplineType type;                /* step_function, natural_cubic, gcv_spline */
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
} dpSplineFunction;                  /* description of a cubic-spline function*/


typedef struct
{
   int genc;                         /* gencoord number */
   double start;                     /* starting value of range-of-motion */
   double end;                       /* ending value of range-of-motion */
} dpPointRange;                      /* muscle point range (wrapping pts) */


typedef struct
{
   int segment;                      /* body segment this point is fixed to */
   int refpt;                        /* corresponding point in saved muscle */
   dpBoolean selected;               /* whether or not point has been selected */
   double point[3];                  /* xyz coordinates of the point */
   int numranges;                    /* number of ranges where point is active */
   dpPointRange* ranges;             /* array of ranges */
   dpBoolean is_auto_wrap_point;     /* was this point calc-ed by auto wrapper? */
   double wrap_distance;             /* stores distance of wrap over object surface */
   double* wrap_pts;                 /* points wrapping over surface */
   int num_wrap_pts;                 /* number of points wrapping over surface */
} dpMusclePoint;                     /* properties of a muscle point */


typedef enum
{
   dpConstraintSphere = 0,
   dpConstraintCylinder,
   dpConstraintEllipsoid,
   dpConstraintPlane,
   dpConstraintNone
} dpConstraintObjectType;


typedef struct
{
   char *name;
   int segment;
   double offset[3];
   double weight;
   int constraint_num;
} dpConstraintPoint;


typedef struct
{
   char *name;                             /* name of constraint object */
   dpConstraintObjectType constraint_type; /* type of constraint object */
   dpBoolean active;                       /* is this constraint active? */
   int segment;                            /* body segment object is fixed to */
   double co_radius[3];                    /* constraint object radius */
   double height;                          /* constraint object height (cylinder only) */
   int constraint_axis;                    /* which axis to constrain over */
   int constraint_sign;                    /* which side of constraint object to use */
   int numPoints;                          /* number of constraint points */
   int ptIndex;                            /* index of first pt linked to this constraint */
   double from_local_xform[4][4];          /* parent-to-wrapobj transform matrix */
   dpPlaneStruct plane;                    /* plane parameters (plane only) */
   double to_local_xform[4][4];            /* wrapobj-to-parent transform matrix */
   int cp_array_size;                      /* size of array of constraint points */
   dpConstraintPoint *points;              /* constraint points */
} dpConstraintObject;


typedef struct
{
   int start;                        /* input:  wrap starting muscle point index */
   int end;                          /* input:  wrap ending muscle point index */
   double* wrap_pts;                 /* output: array of wrapping path points */
   int num_wrap_pts;                 /* output: size of 'wrap_pts' array */
   double wrap_path_length;          /* output: distance along curved r1->r2 path */
   double r1[4];                     /* output: wrap tangent point nearest to p1 */
   double r2[4];                     /* output: wrap tangent point nearest to p2 */
} dpWrapParams;


typedef enum
{
   dpWrapSphere,
   dpWrapCylinder,
   dpWrapEllipsoid,
   dpWrapTorus,
   dpWrapNone
} dpWrapObjectType;


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
} dpWrapObject;


typedef struct
{
   dpWrapObject* wrap_object;        /* muscle is auto-wrapped over this object */
   int wrap_algorithm;               /* wrap algorithm for this muscle */
   int startPoint;                   /* wrap only those muscle segments between startPoint */
   int endPoint;                     /* and endPoint */
   double c[3];                      /* previous c point */
   double r1[3];                     /* previous r1 (tangent) point */
   double r2[3];                     /* previous r2 (tangent) point */
   dpMusclePoint mp_wrap[2];         /* the two muscle points created when the muscle wraps */
} dpMuscleWrapStruct;


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
   dpMusclePoint* mp_orig;           /* list of muscle points from muscle file */
   dpMusclePoint** mp;               /* list of muscle points after auto wrapping */
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
   dpSplineFunction* tendon_force_len_curve; /* tendon force-length curve */
   dpSplineFunction* active_force_len_curve; /* muscle active force-length curve */
   dpSplineFunction* passive_force_len_curve; /* muscle passive force-length curve */
   dpSplineFunction* force_vel_curve;/* muscle force-velocity curve */
   int num_dynamic_params;           /* size of dynamic_params array */
   char** dynamic_param_names;       /* list of dynamic parameter names */
   double** dynamic_params;          /* array of dynamic (muscle model) parameters */
   double dynamic_activation;        /* dynamic value of muscle activation */
   dpSplineType* excitation_format;  /* format for interpolating excitation data. */
   int excitation_abscissa;          /* excit. is func of this gencoord (-1 = time) */
   double excitation_level;          /* current level of excitation */
   dpSplineFunction* excitation;     /* excitation (activation) sequence */
   int* muscle_model_index;          /* index for deriv, init, & assign func arrays */
   dpBoolean wrap_calced;            /* has wrapping been calculated/updated? */
   int numWrapStructs;               /* number of wrap objects used for this muscle */
   dpMuscleWrapStruct** wrapStruct;  /* holds information about the wrap objects */
   int numStateParams;               /* number of initial state values stored in dllparams.txt */
   double* stateParams;              /* array of initial state values in dllparams.txt */
} dpMuscleStruct;                    /* properties of a musculotendon unit */


typedef struct
{
   double x1;
   double x2;
   double y1;
   double y2;
   double z1;
   double z2;
} dpBoundingCube;


typedef struct
{
   int num_vertices;
   int *vertex_index;
   double normal[3];
   double d;
} dpPolygonStruct;


typedef struct
{
   double coord[3];
   double normal[3];
   int polygon_count;
   int* polygons;
} dpVertexStruct;


typedef struct
{
   char* name;
   int body_segment;
   dpBoundingCube bc;
   dpVertexStruct* vertex;
   int num_vertices;
   dpPolygonStruct* polygon;
   int num_polygons;
} dpPolyhedronStruct;


typedef enum
{
   dpUnconstrainedQ,  /* free gencoord */
   dpConstrainedQ,    /* constrained as a function of another gencoord */
   dpPrescribedQ,     /* gencoord with prescribed pos, vel, and acc */
   dpFixedQ           /* fixed gencoord, vel and acc = 0.0 */
} dpQType;


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
   dpSplineFunction* restraint_func;
   dpSplineFunction* min_restraint_func;
   dpSplineFunction* max_restraint_func;
   dpSplineFunction* constraint_func;
   dpBoolean function_active;
   dpBoolean output;
   double torque;
	double pd_stiffness;
} dpQStruct;


typedef struct
{
   int segment;                  /* SD/FAST segment floor associated with */
   char* name;                   /* name of spring_floor */
   dpPlaneStruct plane;          /* parameters of floor plane */
   dpPolyhedronStruct* ph;       /* polyhedron from bone file */
} dpSpringFloor;


typedef struct
{
   int segment;                  /* SD/FAST segment force matte associated with */
   char* name;                   /* name of force_matte */
   dpPolyhedronStruct* ph;       /* polyhedron from bone file */
} dpForceMatte;                  /* used for applying ground-based forces to other segments */


typedef struct
{
   int segment;                 /* SD/FAST segment spring is attached to */
   int floor;                   /* index of corresponding floor object */
   char* floor_name;            /* name of corresponding floor object */
   double point[3];             /* xyz coordinates of the spring point */
   double force[3];             /* spring force vector */
   double friction;             /* coefficient of friction with floor */
   double param_a;              /* spring parameters */
   double param_b;
   double param_c;
   double param_d;
   double param_e;
   double param_f;
} dpSpringStruct;


typedef struct
{
	double point[3];
	double vector[3];
} dpForceStruct;


typedef struct
{
   char* name;
   double mass;
   double inertia[3][3];
   double mass_center[3];
   double body_to_joint[3];
   double inboard_to_joint[3];
   dpBoolean output;
   dpBoolean contactable;
   int num_objects;
   dpPolyhedronStruct** object;
   double contact_force[3];
   double impact_force[3];
   double impact_point[3];
   dpBoolean* contact_joints;
} dpBodyStruct;


typedef struct
{
	int inboard_body;
	int outboard_body;
	int dof;
	int first_q;
	int quat;
	double axes[6][3];
	double force[3];
	double torque[3];
   dpDOFType dof_type[6];
	dpJointType jnt_type;
   dpBoolean loop_joint;
} dpJointStruct;


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
   dpQStruct* q;
   dpBodyStruct* body_segment;
   dpMuscleStruct* muscles;
   dpMuscleStruct default_muscle;
   dpJointStruct* joint;
   int num_wrap_objects;
   dpWrapObject* wrap_object;
   int num_constraint_objects;
   dpConstraintObject* constraint_object;
   int num_constraint_functions;
   dpSplineFunction* constraint_function;
   double gravity[3];
   dpSimmModelID SimmModel;
   int enforce_constraints;    /* enforce constraint objects while integrating? */
	int newInverseSimulation;   /* new-style inverse with corrective torques? */
} dpModelStruct;


DLL int dpInitSimulation(dpSimulationParameters* params);
DLL int dpSetModel(dpModelStruct* model);
DLL int dpRunSimulation(int);
DLL int dpPauseSimulation(int);
DLL int dpResetSimulation(int);
DLL int dpRegisterCallback(int (*dataCallback)(dpSimmModelID, dpDataType, void*));


typedef int (*dpInitSimulationFunc)(dpSimulationParameters* params);
typedef int (*dpSetModelFunc)(dpModelStruct* model);
typedef int (*dpRunSimulationFunc)(int);
typedef int (*dpPauseSimulationFunc)(int);
typedef int (*dpResetSimulationFunc)(int);
typedef int (*dpRegisterCallbackFunc)(int (*dataCallback)(dpSimmModelID, dpDataType, void*));


typedef struct
{
   dpInitSimulationFunc dpInitSimulation; 
   dpSetModelFunc dpSetModel;
   dpRunSimulationFunc dpRunSimulation;
   dpPauseSimulationFunc dpPauseSimulation;
   dpResetSimulationFunc dpResetSimulation;
   dpRegisterCallbackFunc dpRegisterCallback;
} dpSimFuncs;


#endif // __dp_h__

