/*******************************************************************************

   MODELPLOT.H

   Author: Peter Loan

   Date: 13-APR-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef MODELPLOT_H
#define MODELPLOT_H

#define MAX_GENCOORDS 5000
#define MAX_ELEMENTS 16384 // for pop-up menu tags

#define R1 0
#define R2 1
#define R3 2
#define TX 3
#define TY 4
#define TZ 5
#define TRANS 0
#define ROT1 1
#define ROT2 2
#define ROT3 3
#define AXIS1 0
#define AXIS2 1
#define AXIS3 2

#define DEFAULT_TITLE 0
#define USER_DEFINED_TITLE 1

/* For object picking */
#define OBJECT_BITS ((PickIndex)56)
#define MAX_OBJECTS ((PickIndex)2<<OBJECT_BITS)
#define ELEMENT_BITS ((PickIndex)28)
#define MAX_PICK_ELEMENTS ((PickIndex)2<<ELEMENT_BITS)
#define SUBELEMENT_BITS ((PickIndex)28)
#define MAX_PICK_SUBELEMENTS ((PickIndex)2<<SUBELEMENT_BITS)
#define OBJECT_MASK     ((PickIndex)0x00ffffffffffffff)
#define ELEMENT_MASK    ((PickIndex)0x00fffffff0000000)
#define SUBELEMENT_MASK ((PickIndex)0x000000000fffffff)
#define BONE_OBJECT ((PickIndex)0x01)
#define MUSCLE_OBJECT ((PickIndex)0x02)
#define WORLD_OBJECT ((PickIndex)0x03)
#define POINT_OBJECT ((PickIndex)0x04)
#define POLYGON_OBJECT ((PickIndex)0x05)
#define MARKER_OBJECT ((PickIndex)0x06)
#define WRAP_OBJECT ((PickIndex)0x07)
#define CONSTRAINT_OBJECT ((PickIndex)0x08)
#define DEFORM_OBJECT ((PickIndex)0x09)
#define JOINT_AXIS_OBJECT ((PickIndex)0x0a)
#define BODY_FRAME_OBJECT ((PickIndex)0x0b)
#define MOTION_OBJECT ((PickIndex)0x0c)
#define FORCE_MATTE_OBJECT ((PickIndex)0x0d)
#define CONTACT_OBJECT ((PickIndex)0x0e)
#define SPRING_FLOOR_OBJECT ((PickIndex)0x0f)
#define SPRING_OBJECT ((PickIndex)0x10)
#define MASS_OBJECT ((PickIndex)0x11)
#define INERTIA_OBJECT ((PickIndex)0x12)
#define SHADOW_OBJECT ((PickIndex)0x13)
#define CONSTRAINT_POINT_OBJECT ((PickIndex)0x14)
#define DRAW_ALL -1

#define BF 1
#define NBF 0

#define TOP 25
#define BOTTOM 44
#define RIGHT 20

#define LEFTMARGIN 8

#define KEYSTEP 20
#define CROSSHAIRPIXELS 12.0

#define XMARGIN 20.0
#define YMARGIN 20.0

#define PLOTMARGIN 15

#define GENCOORD_ARRAY_INCREMENT 10
#define MUSCLE_ARRAY_INCREMENT 15
#define MUSCGROUP_ARRAY_INCREMENT 10
#define MUSCINDEX_ARRAY_INCREMENT 20
#define MUSCLEPOINT_ARRAY_INCREMENT 12
#define WRAP_OBJECT_ARRAY_INCREMENT 10
#define DEFORMITY_ARRAY_INCREMENT 10
#define GENC_ARRAY_INCREMENT 16
#define SEGMENT_ARRAY_INCREMENT 10
#define JOINT_ARRAY_INCREMENT 10
#define FUNC_ARRAY_INCREMENT 20
#define CONSTRAINT_OBJECT_ARRAY_INCREMENT 10
#define CP_ARRAY_INCREMENT 10

#define NEW_MUSCLE_INDEX -6
#define DEFAULT_MUSCLE_INDEX -5

#define START_MATERIALS 50

#define MAXMOTIONCURVES (MAX_GENCOORDS * 10)
#define MAXSAVEDVIEWS 5
#define MAX_POLY_INTERS 20

// The following should really be equal to MAX_NUM_MOTIONS, but there is no such constant.
#define REALTIME_MENU_ITEM (MAX_GENCOORDS * 2)
#define NO_MOTION_OPTION (MAX_GENCOORDS * 2)

#define DEFAULT_MOTION_SPEED 100.0

#define MAX_WRAPS_PER_MUSCLE 5

#define DEFAULT_MARKER_RADIUS 0.01
#define DEFAULT_CONSTRAINT_TOLERANCE 1e-3
/* max value of RMS of all residuals.  If RMS > tolerance, loop is broken*/
#define DEFAULT_LOOP_TOLERANCE 1e-4
#define DEFAULT_LOOP_WEIGHT 10.0
#define DEFAULT_SOLVER_ACCURACY 1e-4

#define TIME ((void*)0xcafebeef)

#define BOOL_EPSILON        0.00001
#define BOOL_EPSILON_SQR    0.0000000001
#define BOOL_APX_EQ(x, y)              (DABS((x) - (y)) <= BOOL_EPSILON)

#define BOOL_EPSILON2         0.00001
#define BOOL_EPSILON2_SQR     0.0000000001
#define BOOL_APX_EQ2(x, y)               (DABS((x) - (y)) <= BOOL_EPSILON2)

#define BOOL_EPSILON3         0.002
#define BOOL_EPSILON3_SQR     0.000004
#define BOOL_APX_EQ3(x, y)               (DABS((x) - (y)) <= BOOL_EPSILON3)

#define ANGLES_APX_EQ(x, y)              (DABS((x) - (y)) <= ANGLE_EPSILON)

#define FALSE 0
#define TRUE 1
#define BOOL_INFINITY 1000000
#define ON_RAY 0
#define ABOVE_RAY 1
#define BELOW_RAY 2

#define GLOBAL_SEGMENT -1
#define UNDEFINED_SEGMENT -2
#define UNDEFINED_MARKER -2

#define FUNCTION_MENU_START 1000

ENUM {
   rtMocap,
   rtSimulation,
   rtNotConnected
} RTConnection;


ENUM {
   draw_skin_only,
   draw_bones_only,
   draw_skin_and_bones
} SkinMode;


ENUM {
   rotation_gencoord,
   translation_gencoord
} GencoordType;                      /* type of a gencoord */


ENUM {
   TFL_CURVE = 0,
   AFL_CURVE,
   PFL_CURVE,
   FV_CURVE,
   EXC_CURVE
} MuscleFunctionType;


ENUM {
   constant_dof,                     /* constant */
   function_dof                      /* function */
} DofType;                           /* type of elements in a dof definition */


ENUM {
   r1,                               /* rotation about axis 1 */
   r2,                               /* rotation about axis 2 */
   r3,                               /* rotation about axis 3 */
   tx,                               /* translation along the x axis */
   ty,                               /* translation along the y axis */
   tz,                               /* translation along the z axis */
   t                                 /* used by the Gait Loader */
} DofKey;                            /* the 6 dofs in every joint */


ENUM {
   MODEL_ADD,                        /* add a new model */
   MODEL_DELETE                      /* delete an existing model */
} ModelAction;                       /* possible actions on model windows */


ENUM {
   ASCII_FORMAT,                     /* columns of numbers (raw data) */
   POSTSCRIPT_FORMAT,                /* plain, Postscript format */
   ILLUSTRATOR_FORMAT                /* Illustrator-readable Postscript */
} PLOTFILEFORMAT;                    /* format for output plot file */


ENUM {
   found_none,
   found_some
} VerticesFound;


ENUM {
   unspecified_order,
   clockwise,
   counterclockwise
} VertexOrder;


ENUM {
   no_proc,
   yes_proc,
   yes_and_propagated
} PolygonProcess;


ENUM {
   old_ascii,
   new_ascii,
   binary,
    wavefront,
   stl_ascii,
   stl_binary,
   unknown,
   file_not_found
} FileType;

ENUM {
   file_good,
   file_bad,
   file_read_only,
   file_missing
} FileReturnCode;

ENUM {
   didNotSolveLoop,
   loopUnchanged,
   loopChanged,
   loopBroken,
   largeResidinLoop
} LoopStatus;

ENUM {
   didNotSolveConstraint,
   constraintUnchanged,
   constraintChanged,
   constraintBroken,
   gcOutOfRange
} ConstraintStatus;

enum {                               /* bit flags for display_animation_hz */
   DISPLAY_HZ_IN_WINDOW  = 0x01,
   DISPLAY_HZ_ON_CONSOLE = 0x02,
   DISPLAY_POLYS_PER_SEC = 0x04
};

STRUCT {
   double start;                     /* starting value of range */
   double end;                       /* ending value of range */
} Range;                             /* a generalized coordinate range */

ENUM {
   constraint_sphere = 0,
   constraint_cylinder,
   constraint_ellipsoid,
   constraint_plane,
   constraint_none
} ConstraintObjectType;

STRUCT {
   double a, b, c, d;
} PlaneStruct;


STRUCT {
   dpCoord3D  rotation_axis;           /* local-to-parent transform parameter */
   double   rotation_angle;          /* local-to-parent transform parameter */
   dpCoord3D  translation;             /* local-to-parent transform parameter */
   SBoolean xforms_valid;            /* do xform matrices need recalculation? */
   int      ref_count;               /* used to track a specific instance of an xform */
   DMatrix  from_local_xform;        /* wrapobj-to-parent transform matrix */
   DMatrix  to_local_xform;          /* parent-to-wrapobj transform matrix */
} XForm;

STRUCT {
   char*    name;                    /* name of deform object */
   int      segment;                 /* body segment object is fixed to */
   SBoolean active;                  /* is the deform object active? */
   SBoolean visible;                 /* is the deform object visible? */
   SBoolean autoReset;               /* special case -- auto-reset deformation */
   SBoolean translationOnly;         /* auto-reset translations only? */
   dpCoord3D  innerMin, innerMax;      /* inner deform box */
   dpCoord3D  outerMin, outerMax;      /* outer deform box */
   XForm    position;                /* outer box placement (in segment frame) */
   XForm    deform_start;            /* inner box starting placement (in segment frame) */
   XForm    deform_end;              /* inner box ending placement (in segment frame) */
   double   deform_factor;           /* deformation parameter (0.0 to 1.0) */
   DMatrix  delta_xform;             /* transform for deforming vertices */
   DMatrix  delta_xform2;            /* transform for displaying deform values in DE */
   DMatrix  delta_xform3;            /* transform for displaying deform boxes and axes */
   float*   innerBox;                /* inner box vertices */
   float*   innerBoxUndeformed;      /* inner box vertices (undeformed) */
   float*   outerBox;                /* outer box vertices */
   float*   outerBoxUndeformed;      /* outer box vertices (undeformed) */
} DeformObject;

STRUCT {
   char*          name;
   double         value;
   double         default_value;
   Range          range;
   int            keys[2];           /* keys to change deformity's value */
   int            num_deforms;
   DeformObject** deform;
   char**         deform_name;
} Deformity;

STRUCT {
   int segmentnum;                   /* number of the segment this polygon is in */
   int bonenum;                      /* number of the bone this polygon is in */
   int polynum;                      /* the polygon number */
} SelectedPolygon;                   /* properties of the highlighted polygon */


STRUCT {
   Condition condition;              /* condition (dirty,clean) of matrices */
   DMatrix forward;                  /* forward transform for ref frames */
   DMatrix inverse;                  /* backward transform for ref frames */
#if ! ENGINE
   GLfloat gl_forward[16];           /* float matrix for use in GL calls */
   GLfloat gl_inverse[16];           /* float matrix for use in GL calls */
#endif
} Conversion;                        /* joint transformation matrices */


STRUCT {
   int genc;                         /* gencoord number */
   double start;                     /* starting value of range-of-motion */
   double end;                       /* ending value of range-of-motion */
} PointRange;                        /* muscle point range (wrapping pts) */


STRUCT {
   char* name;                       /* name of muscle group */
   int num_muscles;                  /* number of muscles in group */
   int muscindex_array_size;         /* current size of muscle_index array */
   int* muscle_index;                /* list of muscles in group */
   int number_of_columns;            /* width of menu, in columns */
   int column_width;                 /* width of a column, in pixels */
   Menu menu;                        /* menu structure for this group */
} MuscleGroup;                       /* properties of a muscle group */


STRUCT {
   char* name;
   int   num_segments;
   int   seg_array_size;
   int*  segment;
   int   displaymodemenu;
} SegmentGroup;


STRUCT {
   int v_num;
   double distance;
   int side;
} PListStruct;


STRUCT {
    double ptseg[2][3];
    int vert_edge[2];
    int vertex_index[2]; /* vertices of edge (in pl1) if seg runs over an edge */
    int vindices[2][2];  /* vertices of edges (in pl2) that made this segment */
    int poly_index;
    int marked;          /* used in boolean extraction routines */
} PInterSegmentStruct;

STRUCT {
   int num_inter_seg;         /* segments in each polygon */
   PInterSegmentStruct *seg;
   PInterSegmentStruct *segmaxx;
} PInterSegmentListStruct;

STRUCT {
   int seglst_num;
   PInterSegmentListStruct *seglst; /* used for clipping loops */
} PTInterSegmentListStruct;

STRUCT {
   double key;
   int num_inter_seg;
   PInterSegmentListStruct *seglst; /* used for sorting loops */
} SortOpenStruct;

STRUCT {
   int num;              /* always be an even number */
   SortOpenStruct *sos;  /* used for sorting loops */
} SortOpenListStruct;

STRUCT {
   int coplanar_flag;
   int polygon_mark;     /* polygon used or has an intersection */
   int poly_output;      /* polygon is in output list */
   int poly_adjpush;     /* polygon is being pushed onto adj list */
   int num_inters;
   int inters[MAX_POLY_INTERS];
   PInterSegmentListStruct seg_list; /* segment lists from intersection */
   int seglst_num;
   PInterSegmentListStruct *seglst; /* used for clipping loops */
} BooleanInfo;


STRUCT {
   int num_vertices;
   int *vertex_index;
   double normal[3];
   int normal_computed;
   SBoolean selected;
#if POLYGON_DISPLAY_HACK
   SBoolean displayed;
#endif
   int polyhedron_number;
   int thrown_out;
   int old;
   double ordering_value;
   double d;
   BoundingCube bc;
   BooleanInfo boolcode;
} PolygonStruct;


STRUCT {
   double coord[3];
   double normal[3];
   float texture[2];
   int polyhedron_number;
   int thrown_out;
   int old;
   int new_index;
   int polygon_count;
   int* polygons;
} VertexStruct;


STRUCT {
   char* name;
   GLuint gl_display;
   BoundingCube bc;
   VertexStruct* vertex;
   int num_vertices;
   PolygonStruct* polygon;
   int num_polygons;
   int num_removed_vertices;
   int num_removed_polygons;
   float tx, ty, tz;
   float sx, sy, sz;
   double transform_matrix[4][4];
   SBoolean selected;
   int selected_vertex;
   int selected_polygon;
   int selected_edge;
   DrawingMode drawmode;
   int material;
   SBoolean show_normals;
   VertexStruct* undeformed;
   GLuint texture;
} PolyhedronStruct;


STRUCT {
   char* name;
   SBoolean defined_yet;
   SBoolean defined_in_file;
   SBoolean ambient_defined;
   GLfloat ambient[4];
   SBoolean diffuse_defined;
   GLfloat diffuse[4];
   SBoolean specular_defined;
   GLfloat specular[4];
   SBoolean shininess_defined;
   GLfloat shininess;
   SBoolean emission_defined;
   GLfloat emission[4];
   SBoolean alpha_defined;
   GLuint normal_list;
   GLuint highlighted_list;
} MaterialStruct;


STRUCT {
   char* name;
   char* filename;
   char* texture_filename;
   char* texture_coords_filename;
   PolyhedronStruct* wobj;
   DrawingMode drawmode;
   SBoolean selected;
   int material;
   DMatrix transform;
   long drawmodemenu;
} WorldObject;


STRUCT {
   char *name;                /* name of point */
   int segment;               /* index of segment point is on */
   int floorSegment;          /* index of segment floor is attached to */ 
   double point[3];           /* xyz coordinates of the point */
   double friction;           /* coefficient of friction with floor */
   double param_a;            /* parameters */
   double param_b;
   double param_c;
   double param_d;
   double param_e;
   double param_f;
   SBoolean visible;          /* is spring point visible? */   
   SBoolean active;           /* is spring point active? - not used yet */
   double radius;             /* radius used to display/draw point */
} SpringPoint;

STRUCT {
   char *name;                /* name of floor */
   char *filename;            /* filename for floor */
   PolyhedronStruct *poly;    /* polygons in floor */
   SBoolean visible;          /* is floor visible? */
   int numPoints;
   int pointArraySize;
   SpringPoint **points;      /* list of points used with floor */
} SpringFloor;

STRUCT {
   char* name;
   char* filename;
   PolyhedronStruct* poly;
   SBoolean visible;
} ContactObject;


STRUCT {
   char* body1;
   char* body2;
   double restitution;
   double mu_static;
   double mu_dynamic;
} ContactPair;


STRUCT {
   char* name;
   int numElements;
   char** element;
} ContactGroup;

STRUCT {
   char* name;
   double offset[3];
   double weight;
   SBoolean visible;
   SBoolean selected;
   SBoolean fixed;     /* is marker not solved for when using OrthoTrak algorithms? */
} Marker;

STRUCT {
   double accuracy;
   smSolverMethod method;
   int max_iterations;
   smBoolean joint_limits;
   smBoolean orient_body;
   smBoolean fg_contact;
} SolverOptions;

STRUCT {
   int mode;
   SBoolean draw_world_objects;
   SBoolean draw_bones;
   SBoolean draw_muscles;
   SBoolean draw_selection_box;
   SkinMode skin_mode;
   int field1;
   int field2;
} ModelDrawOptions;


STRUCT {
   char* name;                       /* name of segment */
   int numBones;                     /* number of bone files */
   int boneArraySize;                /* size of bone[] array */
   PolyhedronStruct* bone;           /* list of bones for this segment */
   SBoolean defined;                 /* is segment defined in joint file? */
   SBoolean shadow;                  /* draw shadow of this bone? */
   SBoolean draw_axes;               /* draw axes along with segment? */
   SBoolean shadow_color_spec;       /* was shadow color specified in file? */
   SBoolean show_masscenter;         /* draw masscenter along with bone */
   SBoolean show_inertia;             /* draw inertia along with bone */
   double axis_length;               /* length of each axis */
   long drawmodemenu;                /* pop-up menu for changing draw mode */
   float shadow_scale[3];            /* 1 0 1 means show y-axis shadow */
   GLfloat shadow_trans[3];          /* where to draw shadow */
   ColorRGB shadow_color;            /* color of the shadow */
   DrawingMode drawmode;             /* how to display this segment */
   double mc_radius;                 /* radius of sphere for mass display */
   int material;                     /* material this segment is made of */
   double mass;                      /* mass of this segment */
   double inertia[3][3];             /* moment of inertia of this segment */
   double masscenter[3];             /* location of mass center */
   SBoolean mass_specified;          /* was mass specified in joint file? */
   SBoolean inertia_specified;       /* was inertia specified in joint file? */
   SBoolean masscenter_specified;    /* was masscenter specified in joint file? */
   float bone_scale[3];              /* bone scale factor for patty */
   Condition ground_condition;       /* are matrices currently valid? */
   DMatrix from_ground;              /* transformation from ground to this seg */
   DMatrix to_ground;                /* transformation from this seg to ground */
#if ! ENGINE
   GLfloat float_from_ground[16];    /* transformation from ground to this seg */
   GLfloat float_to_ground[16];      /* transformation from this seg to ground */
#endif
   int numgroups;                    /* number of groups to which segment belongs*/
   int* group;                       /* list of segment group numbers */
   int numSpringPoints;
   int springPointArraySize;
   SpringPoint* springPoint;         /* spring points for use with spring-based contact det. */
   SpringFloor* springFloor;         /* spring floor */
   int numContactObjects;
   int contactObjectArraySize;
   ContactObject* contactObject;     /* array of [rigid-body] contact objects */
   ContactObject* forceMatte;        /* force matte (for applying ground forces in DP) */
   int numMarkers;
   int markerArraySize;
   Marker** marker;                  /* markers for Solver model */
   double htr_o[3];                  /* origin of htr segment in solver */
   double htr_x[3];                  /* direction of htr seg Xaxis in solver */
   double htr_y[3];                  /* direction of htr seg Yaxis in solver */
#if INCLUDE_BONE_EDITOR_EXTRAS
   char* pts_file;
   int num_raw;
   double* raw_vertices;
#endif
   int num_deforms;
   int deform_obj_array_size;
   DeformObject* deform;
#if INCLUDE_MOCAP_MODULE
   double  lengthstart[3];           /* starting point of reference for calcing [HTR] segment length */
   double  lengthend[3];             /* ending point of reference for calcing [HTR] segment length */
   SBoolean lengthstartend_specified; /* whether or not lengthstart and lengthend were specified in file */
   char*   gait_scale_segment;       /* name of segment in OrthoTrak model to compare length to */
   double  gait_scale_factor[3];     /* scale of this segment, is compared to size of gait_scale_segment */
   char*   mocap_segment;            /* associated mocap segment name */
   int     mocap_seg_index;          /* associated mocap segment index */
   int     mocap_scaling_method;     /* method used to scale this segment */
   char*   mocap_scale_chain_end1;   /* distal segment to use for scaling (SIMM or Mocap) */
   char*   mocap_scale_chain_end2;   /* distal segment to use for scaling (Mocap or SIMM) */
   DMatrix mocap_adjustment_xform;   /* SIMM-to-mocap segment transformation */
#endif
} SegmentStruct;                     /* properties of a segment */


enum { INHERIT_SCALE, SCALE_ONE_TO_ONE, SCALE_CHAIN, NUM_SCALING_METHODS };


STRUCT {
   char* name;                       /* name as dof appears in SD/FAST code */
   char* con_name;                   /* if constrained, additional name */
   double initial_value;             /* value of dof at start of simulation */
   SBoolean constrained;             /* is this dof constrained by a joint func? */
   SBoolean fixed;                   /* is this dof fixed (prescribed in SD/FAST)? */
   int state_number;                 /* element of state vector that holds this dof */
   int error_number;                 /* if constrained, index into error array */
   int joint;                        /* SD/FAST joint which contains this dof */
   int axis;                         /* axis in SD/FAST joint which cor. to this dof*/
   double conversion;                /* for printing (RTOD for rots, 1.0 for trans) */
   double conversion_sign;           /* set to -1 if DOF has - slope function */
} SDDof;                             /* holds stuff for SD/FAST */


STRUCT {
   char* name;                       /* name of gencoord */
   double value;                     /* current value of this gencoord */
   double velocity;                  /* current velocity of this gencoord */
   GencoordType type;                /* type of gencoord (rotation, translation) */
   SBoolean wrap;                    /* can values wrap across range boundaries? */
   double tolerance;                 /* tolerance for changing gencoord value */
   SBoolean clamped;                 /* clamp gencoord within range of motion? */
   SBoolean clamped_save;            /* saved value of clamped state */
   SBoolean locked;                  /* lock gencoord at its current value */
   SBoolean locked_save;             /* saved value of locked state */
   int numjoints;                    /* number of joints this gencoord is in */
   int* jointnum;                    /* list of joints this gencoord is in */
   int keys[2];                      /* keys to change gencoord's value */
   Range range;                      /* range of acceptable values */
   SBoolean defined;                 /* is gencoord defined in joint file? */
   SBoolean find_moment_arm;         /* used when calculating moment arms in PM */
   double default_value;             /* initial value from joints file */
   SBoolean default_value_specified; /* was default_value specified in joint file? */
   SBoolean used_in_model;           /* is this gencoord used in the model? */
   dpFunction* restraint_function;   /* torque func that restrains gc throughout ROM */
   int restraint_sdcode_num;         /* func number in SD/FAST code */
   SBoolean restraintFuncActive;     /* is restraint function active? */
   dpFunction* min_restraint_function; /* torque func that restrains gc at min ROM */
   int min_restraint_sdcode_num;     /* func number in SD/FAST code */
   dpFunction* max_restraint_function; /* torque func that restrains gc at max ROM */
   int max_restraint_sdcode_num;     /* func number in SD/FAST code */
   int numgroups;                    /* number of groups to which gencoord belongs*/
   int* group;                       /* list of gencoord group numbers */
#if INCLUDE_MOCAP_MODULE
   char*  mocap_segment;             /* associated mocap segment name */
   int    mocap_seg_index;           /* associated mocap segment index */
   int    mocap_column;              /* associated mocap channel index */
   double mocap_adjust;              /* mocap base position value */
#endif
   SBoolean used_in_loop;            /* is the gencoord used in a loop? */
   SBoolean used_in_constraint;      /* is the gencoord used in a constraint? */
   SBoolean slider_visible;          /* is slider visible in the Model Viewer? */
    double pd_stiffness;              /* stiffness of gc for inverse simulation */
} GeneralizedCoord;                  /* properties of a generalized coordinate*/


STRUCT {
   char* name;
   int num_gencoords;
   int genc_array_size;
   GeneralizedCoord** gencoord;
} GencoordGroup;


STRUCT {
   DofType type;                     /* type (constant,function) of dof */
   DofKey key;                       /* one of: tx, ty, tz, r1, r2, r3 */
   double value;                     /* current value of this dof */
   dpFunction* function;             /* if type=function, pointer to constraint function */
   GeneralizedCoord* gencoord;       /* if type=function, pointer to gencoord */
   SDDof sd;                         /* holds stuff for SD/FAST */
} DofStruct;                         /* description of a dof */


STRUCT {
   char* name;                       /* name of joint */
   char* sd_name;                    /* name of joint in SD/FAST code */
   int sd_num;                       /* number of joint in SD/FAST code */
   dpJointType type;                 /* type of joint, used in SD/FAST code */
   SBoolean defined;                 /* has this joint been defined in joint file? */
   char* solverType;                 /* type of joint, used in Solver code */
   SBoolean user_loop;               /* loop joint defined by user */
   SBoolean loop_joint;              /* does this joint close a loop? */
   int from;                         /* starting segment of joint (parent) */
   int to;                           /* ending segment of joint (child) */
   SBoolean show_axes[3];            /* draw rotation axes in model window? */
   double axis_length[3];            /* length of rotation axis if displayed */
   int order[4];                     /* order to perform transformations */
   double parentframe[3][4];         /* reference frame of parent (x,y,z) */
   double parentrotaxes[3][4];       /* rotation axes in parent frame */
   double parentinterframe[3][4];    /* parent axes after a translation */
   double parentinterrotaxes[3][4];  /* rotation axes (parent) after rotations*/
   double childframe[3][4];          /* child axes expressed in parent frame */
   double childrotaxes[3][4];        /* rotation axes expressed in child frame*/
   double childinterframe[3][4];     /* child axes after a translation */
   double childinterrotaxes[3][4];   /* rotation axes (child) after rotations */
   DofStruct dofs[6];                /* equations defining the 6 dofs */
   Conversion conversion;            /* transformation matrices for joint */
   double reaction_force[3];         /* joint reaction force */
   int jrf_segment;                  /* ref frame that reaction force is in */
   SBoolean* in_seg_ground_path;     /* is this joint in the path from seg to ground? */
   SBoolean  pretransform_active;
   Condition pretransform_condition;
   DMatrix   pretransform;           /* optional joint pretransformation */
   DMatrix   pretransform_inverse;   /* optional joint pretransformation inverse */
#if INCLUDE_MOCAP_MODULE
   char*  mocap_segment;             /* associated mocap segment name */
   int    mocap_seg_index;           /* associated mocap segment index */
#endif
} JointStruct;                       /* properties of a joint */


STRUCT {
   char *name;                            /* name of constraint point */
   int segment;                           /* segment the point is attached to */
   double offset[3];                      /* point's offset in segment frame from segment origin */
   double weight;                        /* weight of the point */
   double tolerance;                     /* tolerance to error between point and surface of constraint */
  // SBoolean visible;                      /* is the point visible */
   //SBoolean active;                       /* is the point active */
   SBoolean broken;                       /* is the point/surface constraint good or broken */
} ConstraintPoint;


STRUCT {
   char *name;                            /* name of constraint object */
   ConstraintObjectType constraintType;   /* type of constraint object */
   int      segment;                      /* body segment object is fixed to */
   SBoolean active;                       /* is the constraint object active? */
   SBoolean visible;                      /* is the constraint object visible? */
   dpCoord3D  radius;                       /* constraint object radius */
   double   height;                       /* constraint object height (cylinder only) */
   PlaneStruct plane;                     /* plane parameters (plane only) */
   int      constraintAxis;               /* which axis to constrain over */
   int      constraintSign;               /* which side of constraint axis to use */
   dpCoord3D  rotationAxis;                 /* local-to-parent transform parameter */
   double   rotationAngle;                /* local-to-parent transform parameter */
   dpCoord3D  translation;                  /* local-to-parent transform parameter */
   SBoolean xforms_valid;                 /* do xform matrices need recalculation? */
   DMatrix  from_local_xform;             /* consobj-to-parent transform matrix */
   DMatrix  to_local_xform;               /* parent-to-consobj transform matrix */
   dpCoord3D  undeformed_translation;
   int cp_array_size;                     /* size of array of constraint points */
   int numPoints;                         /* number of constraint points */
   ConstraintPoint *points;               /* constraint points */
   int num_qs;                           /* number of qs affected by constraint */
   int num_jnts;                         /* number of joints affected by constraint */
   int *joints;                          /* list of joints affected by constraint */
   GeneralizedCoord** qs;                /* list of gencoords affected by constraint */
   long     display_list;                 /* only used for cylinder */
   SBoolean display_list_is_stale;
} ConstraintObject;


STRUCT {
   char* name;                       /* name of ligament */
   SBoolean display;                 /* whether or not to display this ligament */
   SBoolean selected;                /* whether or not this ligament is selected */
   double activation;                /* for color display */
   int numlines;                     /* number of ligament lines */
   int line_array_size;              /* current size of ligament line array */
   dpMusclePathStruct* line;         /* array of lines which define ligament border */
   DrawingMode drawmode;             /* how to display this ligament */
   SBoolean show_normals;            /* display surface normal vectors? */
   SBoolean wrap_calced;             /* has wrapping been calculated? */
} LigamentStruct;                    /* properties of a musculotendon unit */


STRUCT{
    double num1[3];
    double num2[3];
    double vec[3];
    SBoolean flag;
    char* musc_name;
    char* segment_name;
} DglValueStruct;                  


STRUCT {                             /* ---- motion object record: */
   char* name;                       /* the motion object's name */
   char* filename;                   /* the motion object's file name */
   SBoolean defined_in_file;
   double startingPosition[3];
   double startingScale[3];
   double startingXYZRotation[3];
   DrawingMode drawmode;             /* the motion object's draw mode */
   int vector_axis;                  /* axis to use for vx,vy,vz vector animation */
   char* materialname;
   int material;                     /* the motion object's starting material */
   PolyhedronStruct shape;           /* the motion object's polyhedron */
} MotionObject;

enum {                               /* ---- motion object channel componant ids: */
   MO_TX, MO_TY, MO_TZ,              /* translation or position */
   MO_VX, MO_VY, MO_VZ,              /* vector */
   MO_SX, MO_SY, MO_SZ,              /* scale */
   MO_CR, MO_CG, MO_CB,              /* color */
   MO_X, MO_Y, MO_Z                  /* axis */
};

ENUM {
   UnknownMotionObject = 0,
   MarkerMotionObject,
   ForceMotionObject,
    FootPrintObject,
    ForcePlateObject
} MotionObjectType;

STRUCT {                             /* ---- motion object animation channel record: */
   int component;                    /* animated componant (see componant ids above) */
   int column;                       /* motion column index */
} MotionObjectChannel;


STRUCT {                             /* ---- motion object instance record: */
   char* name;
   MotionObjectType type;            /* marker, force, etc. */
   int object;                       /* the motion object's definition */
   int segment;                      /* the body segment to which the motion object is attached */
   int num_channels;                 /* the number of animated channels for this motion object */
   MotionObjectChannel* channels;    /* the motion object's channel descriptions */
   DMatrix currentXform;             /* the motion object's current parent-to-local xform */
   MaterialStruct currentMaterial;   /* the motion object's current material */
   double current_value;             /* */
   DrawingMode drawmode;             /* the motion object's current draw mode */
   SBoolean visible;
   GLuint aux_display_obj;
} MotionObjectInstance;


STRUCT {
   double current_value;             /*  */
   int current_frame;                /*  */
   double** gencoords;               /*  */
   double** genc_velocities;         /*  */
   int num_motion_object_instances;
   MotionObjectInstance** motion_object_instance;
   double** muscles;                 /* for activation levels of muscles */
   double** ligaments;               /* for activation levels of ligaments */
   double** other_data;              /*  */
   long curvemenu;                   /* parent menu for data columns (plot curves) */
   long gencoordmenu;                /* popup menu of gencoord data columns */
   long musclemenu;                  /* popup menu of muscle activations */
   long forceplatemenu;              /* popup menu of force plate components */
   long segmentforcemenu;            /* popup menu of segment forces */
   long markermenu;                  /* popup menu of markers */
   long motionobjectmenu;            /* popup menu of motion objects */
   long othermenu;                   /* popup menu of other data columns */
   SBoolean show_markers;            /*  */
   SBoolean show_marker_trails;      /*  */
   SBoolean show_forces;             /*  */
   SBoolean show_force_trails;       /*  */
   SBoolean show_foot_prints;        /*  */
    SBoolean show_force_plates;
} MotionModelOptions;                /*  */


STRUCT {
   char* name;                       /* name of motion */
   SBoolean used;                    /* is this motion structure being used? */
   SBoolean wrap;                    /* can values wrap across motion range? */
   SBoolean is_realtime;             /* should this motion receive realtime data? */
   SBoolean sliding_time_scale;      /* scroll the time scale during realtime import? */
   SBoolean calc_derivatives;        /* should SIMM calculate genc_vels? */
   SBoolean enforce_loops;           /* enforce loops while playing motion? */
   SBoolean enforce_constraints;     /* enforce constraints while playing motion? */
   char** deriv_names;               /* names of SIMM-calculated derivs */
   double** deriv_data;              /* SIMM-caluclated derivs of motion data */
   double time_step;                 /* time between steps in motion */
   int keys[2];                      /* keys to move model through motion */
   int number_of_datarows;           /* number of rows of data */
   int number_of_datacolumns;        /* number of columns of data */
   double min_value;                 /* min value (time step) of motion range */
   double max_value;                 /* max value (time step) of motion range */
   char* units;                      /* units of motion range */
   SBoolean show_cursor;             /* track motion with cursor in plot windows? */
   GLfloat cursor_color[3];          /* color of cursor in plot windows */
   MotionModelOptions mopt;          /* some properties that are tied to a specific model */
   char** columnname;                /* names of the columns of data */
   double** motiondata;              /* the actual motion data */
   double** data_std_dev;            /* standard deviations */
   int num_events;                   /* number of motion events */
   smMotionEvent* event;             /* array of motion events */
   GLfloat event_color[3];           /* color for event lines and text in plots */
   int realtime_circular_index;      /* magical index used to continuously add realtime samples */
   int simulation_index;             /* index of latest results from a simulation DLL */
   smAxes walk_direction;            /* for gait motions */
   smAxes vertical_direction;        /* primarily for gait motions */
   int time_column;                  /* index of column that contains time values */
   int x_column;                     /* index of column that serves as X variable (time, percent, etc.) */
} MotionSequence;                    /* holds a motion sequence, from a file */


ENUM {
   XVAR_GENCOORD_TYPE,               /* gencoord, a model's degree of freedom */
   XVAR_MOTION_CURVE_TYPE,           /* a single data column from a motion sequence */
   XVAR_MOTION_TYPE                  /* motion (gencoords taken from motion sequence) */
} XvarType;                          /* type of x-var used in plotting */


ENUM {
   YVAR_MOMENT_TYPE,                 /* y-var is moment or moment arm */
   YVAR_MOTION_CURVE_TYPE,           /* a single data column from a motion sequence */
   YVAR_OTHER_TYPE                   /* y-var is anything else */
} YvarType;                          /* type of y-var used in plotting */


STRUCT {
   XvarType type;                    /* type of xvar (GENCOORD_TYPE, MOTION_TYPE) */
   double start;                     /* starting value of x-variable for curve */
   double end;                       /* ending value of x-variable for curve */
   MotionSequence* motion;           /* if xvar is a motion, pointer to the motion */
   struct ModelStruct* ms;           /*    and model structures are stored here. */
   GeneralizedCoord* gencoord;       /* gencoord number, if xvar is a gencoord */
   int motion_column;                /* index of data column, if xvar is a motion curve */
   char name[150];                   /* name of x variable */
} XvarStruct;                        /* contains plotmaker xvar information */


STRUCT {
   YvarType type;                    /* type of yvar (MOMENT_TYPE, OTHER_TYPE) */
   int yvariable;                    /* the y variable chosen for plotting */
   GeneralizedCoord* gencoord;       /* if MOMENT_TYPE, the gencoord to use */
   MotionSequence* motion;           /* if yvar is a motion curve, pointer to the motion */
   int motion_column;                /* if a motion variable, the motion column */
   double min;                       /* minimum value for clipping */
   double max;                       /* maximum value for clipping */
   char name[150];                   /* name of y variable */
   int* musc;                        /* list of muscle states (on or off) */
   int* musc_index;
   int nummuscles;                   /* number of muscles in list */
   int muscle_array_size;
   double activation;                /* activation level for all muscles */
   double stepsize;                  /* number of degrees between data points */
   SBoolean active;                  /* include active force? */
   SBoolean passive;                 /* include passive force? */
   SBoolean sum;                     /* whether or not to sum muscle curves */
   SBoolean rectify;                 /* whether or not to rectify curve data */
   SBoolean override_activation;     /* whether or not to use global activation */
   double scale;                     /* scale Y-values by this amount */
   double offset;                    /* offset Y-values by this amount */
   int change_segment;               /* segment for muscle orientations */
   SBoolean isometric;               /* compute only isometric force? */
} YvarStruct;                        /* contains plotmaker yvar information */


STRUCT {
   char* name;                       /*  */
   int color;                        /* color of this curve */
   XvarStruct xvar;                  /* contains xvar information */
   YvarStruct yvar;                  /* contains yvar information */
   void *modelPtr;                   /* model used to make this curve */
   int numvalues;                    /*  */
   void* xvalues;                    /* was double* */
   void* yvalues;                    /* was double* */
   void* y_std_dev;                  /* standard deviations (was double*) */
   SBoolean selected;                /*  */
   void* musc_values;                /* was DglValueStruct* */
   int realtime_circular_index;      /* index used to circularize curve arrays */
   int realtime_update_index;        /* index used to update curves in realtime */
   int simulation_index;             /* index of latest frame from simulation DLL */
   int num_events;                   /* number of events in array */
   smMotionEvent* event;             /* array of motion events, if curve is from a motion */
   GLfloat event_color[3];           /* color for displaying event lines and text */
} CurveStruct;                       /*  */


STRUCT {
   char* name;
   double x_coord;
   GLfloat color[3];
} PlotFileEvent;


STRUCT {
   int plotnum;                      /*  */
   int numcurves;                    /*  */
   SBoolean zoomed_yet;              /*  */
   struct pks* plotkey;              /*  */
   CurveStruct* curve[MAX_PLOT_CURVES];  /*  */
   IntBox datavp;                    /*  */
   Ortho dataortho;                  /*  */
   Ortho savedataortho;              /*  */
   double xmin;                      /*  */
   double xmax;                      /*  */
   double ymin;                      /*  */
   double ymax;                      /*  */
   char xformat[6];                  /*  */
   char yformat[6];                  /*  */
   char *title;                      /*  */
   char *xname;                      /*  */
   char *yname;                      /*  */
   int title_len;                    /*  */
   int xname_len;                    /*  */
   int yname_len;                    /*  */
   int numxticks;                    /*  */
   int numyticks;                    /*  */
   double xinc;                      /*  */
   double yinc;                      /*  */
   double xstart;                    /*  */
   double ystart;                    /*  */
   int numpoints;                    /*  */
   XYCoord pt[30];                   /*  */
   int key_window_x;                 /* x loc. of plot key when plot is iconified */
   int key_window_y;                 /* y loc. of plot key when plot is iconified */
   double cursor_x;                  /* x location of cursor for tracking motion */
   GLfloat cursor_color[3];          /* color of cursor */
   MotionSequence* cursor_motion;    /* motion that is tied to this cursor */
   void *cursor_modelPtr;            /* model whose motion is tied to this cursor */
   SBoolean show_cursor;             /*  */
   SBoolean show_events;             /*  */
   SBoolean needs_bounding;
   int num_file_events;              /* number of plot file events */
   PlotFileEvent* file_event;        /* events read in from a plot file */
} PlotStruct;                        /*  */


STRUCT pks {
   int plotnum;                      /*  */
   int numcurves;                    /*  */
   PlotStruct* plot;                 /*  */
} PlotKeyStruct;    


STRUCT {
   int null_material;                /*  */
   int default_bone_material;        /*  */
   int default_world_object_material;/*  */
   int default_muscle_min_material;  /*  */
   int default_muscle_max_material;  /*  */
   int def_muscle_point_material;    /*  */
   int sel_muscle_point_material;    /*  */
   int bone_outline_material;        /*  */
   int highlighted1_polygon_material; /*  */
   int highlighted2_polygon_material; /*  */
   int marker_material;              /*  */
   int sel_marker_material;          /*  */
   int cp_current_sel_ok_material;              /*  */
   int cp_current_sel_broken_material;              /*  */
   int cp_current_ok_material;              /*  */
   int cp_current_broken_material;              /*  */
   int cp_current_inactive_material;
   int cp_back_ok_material;
   int cp_back_broken_material;
   int cp_back_inactive_material;
   int co_current_active_material;
   int co_current_inactive_material;
   int co_back_active_material;
   int co_back_inactive_material;
//   int co_broken_material;
   int masscenter_material1;
   int masscenter_material2;
   int num_materials;                /*  */
   int material_array_size;          /*  */
   MaterialStruct* materials;        /*  */
} ModelMaterials;


STRUCT {
   int default_view;                 /* which of the saved views is the default */
   int num_file_views;               /* number of views defined in joints file */
   double saved_view[MAXSAVEDVIEWS][4][4]; /* array of saved view transformations */
   char* view_name[MAXSAVEDVIEWS];        /* names of saved views */
   SBoolean view_used[MAXSAVEDVIEWS];     /* has this view been saved by user? */
   long view_menu;
   long maindrawmodemenu;            /*  */
   long allsegsdrawmodemenu;         /*  */
   long allligsdrawmodemenu;         /*  */
   long allworlddrawmodemenu;        /*  */
   long alldrawmodemenu;             /*  */
   long eachsegdrawmodemenu;         /*  */
   SBoolean continuous_motion;       /* animate the model continuously? */
   unsigned display_animation_hz;    /* bit flags for displaying animation hz */
   clock_t time_of_last_frame;       /* time that previous frame of motion was displayed */
   SBoolean display_motion_info;     /* display info about current motion? */
   MotionSequence* applied_motion;   /* motion that is currently applied to model */
   MotionSequence* current_motion;   /* motion linked to hot keys, continuous motion */
   int muscle_array_size;            /*  */
   int nummuscleson;                 /*  */
   int* muscleson;                   /*  */
   MuscleMenu mgroup[GROUPBUFFER];   /*  */
   int menucolumns[COLUMNBUFFER];    /*  */
   long muscle_cylinder_id;          /*  */
#if INCLUDE_MSL_LENGTH_COLOR
   double muscle_color_factor;       /* controls muscle length colorization */
#endif
   double motion_speed;              /*  */
   SBoolean show_selected_coords;    /*  */
   SBoolean show_all_muscpts;        /*  */
   SBoolean fast_muscle_drawing;
   SBoolean show_highlighted_polygon;/*  */
   SBoolean show_shadow;             /*  */
   GLfloat background_color[3];      /*  */
   GLfloat crosshairs_color[3];      /*  */
   GLfloat vertex_label_color[3];    /*  */
   GLfloat rotation_axes_color[3];   /*  */
   SBoolean background_color_spec;   /*  */
   SBoolean crosshairs_color_spec;   /*  */
   SBoolean vertex_label_color_spec; /*  */
   SBoolean rotation_axes_color_spec;/*  */
   SelectedPolygon hpoly;            /*  */
   float muscle_point_radius;        /*  */
   long muscle_point_id;             /*  */
   int numdevs;                      /*  */
   int* devs;                        /* button nums to control joint movement */
   int* dev_values;                  /*  */
   ModelMaterials mat;
} ModelDisplayStruct;

enum {
   SNAPSHOT_INACTIVE,
   SNAPSHOT_INSTANT,
   SNAPSHOT_CONTINUOUS,
   SNAPSHOT_MOTION
};

enum {
   MAKEMOVIE_INACTIVE,
   MAKEMOVIE_CONTINUOUS,
   MAKEMOVIE_MOTION
};

STRUCT {
   int shadem;                       /*  */
   float tx, ty, tz;                 /*  */
   short rx, ry, rz;                 /*  */
   int nummuscleson;                 /*  */
   int musclepts;                    /*  */
   int showpoly;                     /*  */
   SelectedPolygon hpoly;            /*  */
   SBoolean marker_visibility;
} SaveDisplay;                       /*  */


STRUCT {
   char* name;                       /* joint name, not used yet */
   int order[4];                     /* transformation order */
   DofStruct dofs[6];                /* the six dofs (rx, ry, rz, tx, ty, tz) */
   double parentrotaxes[3][4];       /* rotation axes in parent frame */
} SaveJoints;                        /* holds a copy of a joint */

STRUCT {
   char *name;                       /*  */
   double mass;                      /* mass of this segment */
   double inertia[3][3];             /* moment of inertia of this segment */
   double masscenter[3];             /* location of mass center */
   SBoolean mass_specified;          /* was mass specified in joint file? */
   SBoolean inertia_specified;       /* was inertia specified in joint file? */
   SBoolean masscenter_specified;    /* was masscenter specified in joint file? */
   SBoolean show_masscenter;
   SBoolean show_inertia;
   int numSpringPoints;
   SpringPoint* springPoint;         /* spring points for use with spring-based contact det. */
   SpringFloor* springFloor;         /* spring floor */
   int numContactObjects;
   ContactObject* contactObject;     /* array of [rigid-body] contact objects */
   ContactObject* forceMatte;        /* force matte (for applying ground forces in DP) */
} SaveSegments;                      /* holds a copy of a segment */

STRUCT {
   double value;
   double velocity;
   double tolerance;
   double pd_stiffness;
   double default_value;
//   SBoolean default_value_specified;
   SBoolean restraintFuncActive;
   SBoolean clamped;
   SBoolean locked;
   Range range;
   dpFunction* restraint_function;
   SBoolean used_in_model;
} SaveGencoords;


STRUCT {
   char* name;
   int segment;
   double offset[3];
   double weight;
   SBoolean visible;
   SBoolean selected;
} SaveMarker;

STRUCT {
   int muscle;
   int musc_wrap_index;
   dpWrapObject* wrap_object;
   int start_pt;
   int end_pt;
   int wrap_algorithm;
} MuscWrapAssoc;

STRUCT {
   int constraint_obj;
   int numPoints;
   ConstraintPoint *savedPoints;
} SaveConstraintPointAssoc;

STRUCT {
   int numsavedjnts;                 /*  */
   int numsavedgencs;                /*  */
   int numsavedbones;                /*  */
   int numsavedsegments;
   int numsavedmuscgroups;
   SaveSegments *segment;
   SaveJoints* joint;                /*  */
   SaveGencoords *gencoord;
   dpFunction** function;            /*  */
   SaveDisplay disp;                 /*  */
   MuscleGroup *muscgroup;
   int num_wrap_objects;
   dpWrapObject* wrap_object;
   int num_muscwrap_associations;
   MuscWrapAssoc* muscwrap_associations;
   int num_deforms;
   DeformObject* deform;
   int num_markers;                  /*  */
   SaveMarker* marker;               /*  */
   int num_conspt_associations;
   SaveConstraintPointAssoc *conspt_associations;
   int num_constraint_objects;
   ConstraintObject *constraintobj;
} ModelSave;                         /*  */


STRUCT {
   char* name;                       /*  */
   char* forceUnits;                 /* units of muscle force */
   char* lengthUnits;                /* length units of model */
   char* HTRtranslationUnits;        /* units for HTR output, used in Solver only */
   SBoolean is_demo_model;           /* is this the demo model? */
   SBoolean useIK;                   /*  */
   SBoolean defaultGCApproved;       /* has user approved current default GC values? */
   SBoolean defaultLoopsOK;          /* are the loops closed in the default configuration? */
   SBoolean defaultConstraintsOK;    /* are the constraints satisfied for the default values? */
   SBoolean constraintsOK;           /* are the constraints satisfied in current configuration? */
   SBoolean loopsOK;                 /* are the loops closed in the current configuration? */
   int modelnum;                     /*  */
   int numgencoords;                 /*  */
   int numunusedgencoords;           /*  */
   int nummuscles;                   /* number of muscles in this model */
   int numligaments;                 /* number of ligaments in this model */
   int numjoints;                    /*  */
   int numsegments;                  /*  */
   int numgroups;                    /* number of muscle groups */
   int numseggroups;                 /*  */
   int numgencgroups;                /*  */
   int numworldobjects;              /*  */
   int numclosedloops;               /*  */
//   int dis_muscle_array_size;
   int muscle_array_size;            /* current size of dpMuscleStruct array */
   int ligament_array_size;          /* current size of LigamentStruct array */
   int muscgroup_array_size;         /* current size of MuscleGroup array */
   int seggroup_array_size;          /* current size of SegmentGroup array */
   int gencgroup_array_size;         /* current size of GencoordGroup array */
   int genc_array_size;              /* current size of GeneralizedCoord array */
   int segment_array_size;           /* current size of SegmentStruct array */
   int joint_array_size;             /* current size of JointStruct array */
   int func_array_size;              /* current size of dpFunction array */
   int currentframe;                 /*  */
   int pushedframe;                  /*  */
   int longest_genc_name;            /* length, in pixels, of longest gencoord name */
   int longest_dynparam_name;        /* length, in pixels, of longest gencoord name */
   int** pathptrs;                   /*  */
   int ground_segment;               /* index of the ground segment */
   int initial_ground_segment;       /* index of the segment which starts as ground */
   int* segment_drawing_order;       /* order in which to draw the segments */
   SBoolean specified_min_thickness; /* did user specify min_thickness of def musc? */
   SBoolean specified_max_thickness; /* did user specify max_thickness of def musc? */
   long musclegroupmenu;             /* pop-up muscle group menu */
   long jointmenu;                   /* pop-up joint menu */
   long xvarmenu;                    /* pop-up x-var menu (gencoords and motions) */
   long gencoordmenu;                /* pop-up gencoord menu */
   long gencoordmenu2;               /* pop-up gencoord menu */
   long gencoord_group_menu;         /* pop-up gencoord group menu */
   long momentgencmenu;              /* gencoord menu used for moment y-var */
   long momentarmgencmenu;           /* gencoord menu used for moment arm y-var */
   long momentarmnumgencmenu;        /* gencoord menu used for moment arm y-var */
   long maxmomentgencmenu;           /* gencoord menu used for maxmoment y-var */
   long segmentmenu;                 /* pop-up menu of segment names */
   long motionplotmenu;              /* pop-up menu of motions linked to this model */
   long motionwithrealtimemenu;      /* pop-up menu of motions, with RT option, no column data */
   long motionmenu;                  /* pop-up menu of motions, no column data */
   long material_menu;               /* pop-up menu of materials */
   long markerMenu;                  /* pop-up menu of segment-based marker links */
   long functionMenu;                /* pop-up menu of functions */
   double max_dimension;             /* size of largest dimension of model (segments only) */
   double max_dimension2;            /* size of largest dimension of model (segments + world objects) */
   double max_diagonal;              /* diagonal dimension of scene bounding box (segments + world objects) */
   SBoolean max_diagonal_needs_recalc;/* do we need to recompute the model's max diagonal? */
   SliderArray gencslider;           /* array of gencoord sliders */
   Form gencform;                    /* form for gencoords and current values */
   CheckBoxPanel gc_chpanel;         /* checkbox panel for gencoord clamp/unclamp */
   CheckBoxPanel gc_lockPanel;       /* checkbox panel for gc lock/unlock */
   Form dynparamsform;
   int dynparamsSize;
   char* jointfilename;              /* file used to generate joints, segments */
   char* bonepathname;               /* directory used to find bones (optional) */
   char* musclefilename;             /* file used to generate the muscles */
   char* motionfilename[100];        /* motion files specified within joints file */
   char* mocap_dir;                  /* folder with static pose that scaled this model */
   int num_motion_files;             /* number of motion files named in joints file */
   WorldObject* worldobj;            /*  */
   int world_array_size;             /*  */
   JointStruct* joint;               /*  */
   SegmentStruct* segment;           /*  */
   MuscleGroup* muscgroup;           /* array of muscle group menus */
   SegmentGroup* seggroup;           /* array of segment groups */
   GencoordGroup* gencgroup;         /* array of gencoord groups */
   GeneralizedCoord** gencoord;      /*  */
   dpFunction** function;            /* functions used for dofs and gencoords and muscle points */
   dpMuscleStruct** muscle;          /* array of pointers to muscles */
   dpMuscleStruct* default_muscle;   /*  */
   LigamentStruct* ligament;         /* array of ligament structures */
   ModelDisplayStruct dis;           /*  */
   ModelSave save;                   /*  */
   TextLine genc_help[2*GENBUFFER];  /* help text for controlling gencs with keys*/
   SBoolean dynamics_ready;          /* were all required dynamics params specified? */
   int numContactPairs;              /*  */
   int contactPairArraySize;         /*  */
   ContactPair* contactPair;         /*  */
   int numContactGroups;             /*  */
   int contactGroupArraySize;        /*  */
   ContactGroup* contactGroup;       /*  */
   int num_motion_objects;
   MotionObject* motion_objects;
   int num_wrap_objects;
   int wrap_object_array_size;
   dpWrapObject** wrapobj;
   int num_deformities;
   int deformity_array_size;
   Deformity* deformity;
   struct loopstruct* loop;
   SBoolean GEFuncOK;
   SBoolean marker_visibility;
   double marker_radius;
   int constraint_object_array_size;
   int num_constraint_objects;
   ConstraintObject *constraintobj;
//   double constraint_tolerance;
   double loop_tolerance;
   double loop_weight;
   SolverOptions solver;
   smAxes gravity;
   SBoolean global_show_masscenter;
   SBoolean global_show_inertia;
   int num_motions;
   int motion_array_size;
   MotionSequence** motion;
#if ! ENGINE
   glut_mutex_t modelLock;
#endif
   SBoolean realtimeState;
} ModelStruct;

STRUCT {
   int window_index;                 /* index into array of all SIMM windows */
   int window_glut_id;               /* identifier returned by glueOpenWindow() */
   int scene_num;
   int num_models;
   ModelStruct** model;              /* models in this scene */
   int windowX1, windowY1;           /* coordinates of lower left corner of model window */
   int windowHeight, windowWidth;    /* height and width of model window */
   float tx, ty, tz;                 /*  */
   float start_tx;                   /*  */
   float start_ty;                   /*  */
   float start_tz;                   /*  */
   short rx, ry, rz;                 /*  */
   ModelStruct* camera_segment_model;
   int camera_segment;               /* the segment the camera is attached to (or -1) */
   double max_diagonal;
   double transform_matrix[4][4];    /*  */
   GLfloat z_vector[3];              /* world coords of vector out of screen */
   GLint viewport[4];                /* viewport as returned by OpenGL */
   GLdouble modelview_matrix[16];    /* model view matrix as returned by OpenGL */
   GLdouble projection_matrix[16];   /* projection matrix as returned by OpenGL */
   GLdouble fov_angle;               /*  */
   GLdouble aspect_ratio;            /*  */
   GLdouble near_clip_plane;         /*  */
   GLdouble far_clip_plane;          /*  */
   double x_zoom_constant;           /*  */
   double y_zoom_constant;           /*  */
   double model_move_increment;      /*  */
   SBoolean show_crosshairs;         /*  */
   SBoolean trackball;               /*  */
   GLfloat background_color[3];      /*  */
   GLfloat crosshairs_color[3];      /*  */
   int      snapshot_mode;
   int      snapshot_counter;
   char*    snapshot_file_suffix;
   SBoolean snapshot_include_depth;
   int movie_mode;
   char* movie_file;
} Scene;

typedef struct loopstruct {
   ModelStruct *ms;
   int model_num;
   GeneralizedCoord* locked_q;
   double q_value;
   int num_qs;
   int num_resids;
   GeneralizedCoord** qs;
   int *segs;
   int *joints;
   int num_segs;
   int num_jnts;
   SBoolean first_iter;
} LoopStruct;

STRUCT {
   ModelStruct *ms;
   SBoolean *loopUsed;
   GeneralizedCoord** gencoord_list;
   GeneralizedCoord* controlled_gc;
   double controlled_value;
   SBoolean first_iter;
} IKStruct;

STRUCT {
   ModelStruct *model;
   GeneralizedCoord** gencoord_list;
   GeneralizedCoord* controlled_gc;
   double controlled_value;
   SBoolean first_iter;
   SBoolean test;
   double tolerance;
   SBoolean *consUsed;
} ConstraintSolverStruct;

STRUCT {
   ModelStruct *model;
   SBoolean first_iter;
   GeneralizedCoord* controlled_gc;
   double controlled_value;
   int numQs;
   GeneralizedCoord** gencoord_list;
   IKStruct *loopInfo;
   ConstraintSolverStruct *constraintInfo;
   int numConstraintQs;
   int numConstraintResids;
   int numLoopQs;
   int numLoopResids;
   SBoolean largeGenCoordResids;
   SBoolean loopClosed;
   SBoolean constraintClosed;
} LCStruct;


ENUM {
   no_tri,
   simple_tri,
   complex_tri,
   hyper_complex_tri
} TriangulateOption;


ENUM {
   x_ordering,
   y_ordering,
   z_ordering,
   no_ordering
} OrderFormat;


STRUCT {
   SBoolean verbose_output;
   FileType output_format;
   const char* output_dir;
   SBoolean write_separate_polyhedra;
   SBoolean convexify_polygons;
   SBoolean remove_collinear;
   SBoolean clip_vertices;
   SBoolean tol_box_set;
   SBoolean fill_holes;
   TriangulateOption triangulate;
   VertexOrder vertex_order;
   int vertex_offset;
   double vertex_tolerance;
   double max_edge_length;
   double tol_x_min;
   double tol_x_max;
   double tol_y_min;
   double tol_y_max;
   double tol_z_min;
   double tol_z_max;
   double clip_x_min;
   double clip_x_max;
   double clip_y_min;
   double clip_y_max;
   double clip_z_min;
   double clip_z_max;
   double reference_normal[3];
} NormOptions;

STRUCT {
   char* name;
   double mass;
   double mass_center[3];
   double inertia[3][3];
   double body_to_joint[3];
   double inboard_to_joint[3];
   int simm_segment;
} SDSegment;

STRUCT {
   SBoolean used;
   Direction dir;
   char* inbname;
   char* outbname;
   SBoolean closes_loop;
} JointSDF;

STRUCT {
   SBoolean used;
   int times_split;
   double mass_factor;
} SegmentSDF;

STRUCT {
   SBoolean preserveMassDist;
} ScaleModelOptions;

STRUCT {
   Scene* scene;
   ModelStruct* model;
   int pan_mx_old;
    int pan_my_old;
   double pan_wx_old;
    double pan_wy_old;
    double pan_wz_old;
} MoveObjectTracker;

STRUCT {
   Scene* scene;
   ModelStruct* model;
   PickIndex object;
} SelectedObject;

#endif /*MODELPLOT_H*/

