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

#define MAX_GENCOORDS 500

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

#define NO_REGION 0x00000000
#define ACL_ATTACHMENT 0x00000001
#define ACL_ISOMETRY_CHECK 0x00000002
#define PATELLAR_INTERSECT 0x00000004
#define PATELLAR_KINEMATICS 0x00000008
#define ANY_REGION 0xffffffff

/* For object picking */
#define OBJECT_BITS 28
#define MAX_OBJECTS 268435446
#define ELEMENT_BITS 14
#define MAX_ELEMENTS 16384
#define SUBELEMENT_BITS 14
#define MAX_SUBELEMENTS 16384
#define OBJECT_MASK 0x0fffffff
#define ELEMENT_MASK 0x0fffc000
#define SUBELEMENT_MASK 0x00003fff
#define BONE_OBJECT 0x1
#define MUSCLE_OBJECT 0x2
#define WORLD_OBJECT 0x3
#define POINT_OBJECT 0x4
#define POLYGON_OBJECT 0x5
#define DRAW_ALL -1

#define BF 1
#define NBF 0

#define TOP 25
#define BOTTOM 44
#define RIGHT 20
#define XAXIS 0
#define YAXIS 2

#define LEFTMARGIN 8

#define KEYSTEP 20
#define CROSSHAIRPIXELS 12.0

#define XMARGIN 20.0
#define YMARGIN 20.0

#define PLOTMARGIN 15

#define TITLETOP 863
#define TITLEBOT 835

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

#define START_MATERIALS 50

#define MAXMOTIONCURVES 5000
#define MAXSAVEDVIEWS 5
#define MAX_POLY_INTERS 20
#define REALTIME_MENU_ITEM 1000
#define NO_MOTION_OPTION 1000

#define MAX_WRAPS_PER_MUSCLE 5

#define DEFAULT_MARKER_RADIUS 0.01
#define DEFAULT_CONSTRAINT_TOLERANCE 1e-3
/* max value of RMS of all residuals.  If RMS > tolerance, loop is broken*/
#define DEFAULT_LOOP_TOLERANCE 1e-4
#define DEFAULT_LOOP_WEIGHT 10.0
#define DEFAULT_SOLVER_ACCURACY 1e-4

#define MAX_EVENTS 12

#define TIME -1

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
   wrap_sphere,
   wrap_cylinder,
   wrap_ellipsoid,
   wrap_torus,
   wrap_none
} WrapObjectType;                    /* objects that muscles can wrap over */

ENUM {
   constraint_sphere = 0,
   constraint_cylinder,
   constraint_ellipsoid,
   constraint_plane,
   constraint_none
} ConstraintObjectType;

STRUCT {
   int     start;            /* input:  wrap starting muscle point index */
   int     end;              /* input:  wrap ending muscle point index */
   double* wrap_pts;         /* output: array of wrapping path points */
   int     num_wrap_pts;     /* output: size of 'wrap_pts' array */
   double  wrap_path_length; /* output: distance along curved r1->r2 path */
   double  r1[3];            /* output: wrap tangent point nearest to p1 */
   double  r2[3];            /* output: wrap tangent point nearest to p2 */
} WrapParams;


STRUCT {
   SBoolean     isLine;
   double       p1[3];
   double       p2[3];
   float        radius;
   const char*  name1;
   const char*  name2;
   const float* color;
} DebugGlyph;

#define MAX_DEBUG_GLYPHS 300

STRUCT {
   char*    name;                    /* name of wrap object */
   WrapObjectType wrap_type;         /* type of wrap object */
   int      wrap_algorithm;          /* default wrap algorithm for wrap object */
   int      segment;                 /* body segment object is fixed to */
   long     display_list;            /* only used for cylinder */
   SBoolean display_list_is_stale;
   SBoolean active;                  /* is the wrap object active? */
   SBoolean visible;                 /* is the wrap object visible? */
   SBoolean show_wrap_pts;           /* draw wrap point xyz coordinates? */
   Coord3D  radius;                  /* wrap object radius */
   double   height;                  /* wrap object height (cylinder only) */
   int      wrap_axis;               /* which axis to wrap over */
   int      wrap_sign;               /* which side of wrap axis to use */
   Coord3D  rotation_axis;           /* local-to-parent transform parameter */
   double   rotation_angle;          /* local-to-parent transform parameter */
   Coord3D  translation;             /* local-to-parent transform parameter */
   SBoolean xforms_valid;            /* do xform matrices need recalculation? */
   DMatrix  from_local_xform;        /* wrapobj-to-parent transform matrix */
   DMatrix  to_local_xform;          /* parent-to-wrapobj transform matrix */
   Coord3D  undeformed_translation;
#if 1
   int      num_debug_glyphs;
   DebugGlyph glyph[MAX_DEBUG_GLYPHS];
#endif
} WrapObject;

STRUCT {
   double a, b, c, d;
} PlaneStruct;


STRUCT {
   char *name;
   int segment;
   double offset[3];
   double weight;
   double tolerance;
   SBoolean visible;
   SBoolean active;
   SBoolean broken;
} ConstraintPoint;


STRUCT {
   char *name;                            /* name of constraint object */
   ConstraintObjectType constraintType;   /* type of constraint object */
   int      segment;                      /* body segment object is fixed to */
   SBoolean active;                       /* is the constraint object active? */
   SBoolean visible;                      /* is the constraint object visible? */
   Coord3D  radius;                       /* constraint object radius */
   double   height;                       /* constraint object height (cylinder only) */
   PlaneStruct plane;                     /* plane parameters (plane only) */
   int      constraintAxis;               /* which axis to constrain over */
   int      constraintSign;               /* which side of constraint axis to use */
   Coord3D  rotationAxis;                 /* local-to-parent transform parameter */
   double   rotationAngle;                /* local-to-parent transform parameter */
   Coord3D  translation;                  /* local-to-parent transform parameter */
   SBoolean xforms_valid;                 /* do xform matrices need recalculation? */
   DMatrix  from_local_xform;             /* consobj-to-parent transform matrix */
   DMatrix  to_local_xform;               /* parent-to-consobj transform matrix */
   Coord3D  undeformed_translation;
   int cp_array_size;                     /* size of array of constraint points */
   int numPoints;                         /* number of constraint points */
   ConstraintPoint *points;               /* constraint points */
   int num_qs;
   int num_jnts;
   int *joints;
   int *qs;
   long     display_list;                 /* only used for cylinder */
   SBoolean display_list_is_stale;
//   SBoolean broken;
} ConstraintObject;


STRUCT {
   Coord3D  rotation_axis;           /* local-to-parent transform parameter */
   double   rotation_angle;          /* local-to-parent transform parameter */
   Coord3D  translation;             /* local-to-parent transform parameter */
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
   Coord3D  innerMin, innerMax;      /* inner deform box */
   Coord3D  outerMin, outerMax;      /* outer deform box */
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
#ifndef ENGINE
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
   int segment;                      /* segment this muscle point is fixed to */
   SBoolean selected;                /* whether or not point has been selected */
   double point[3];                  /* xyz coordinates of the point */
   double ground_pt[3];              /* coords of point in ground frame */
   int fcn_index[3];                 /* index of function for moving point (3 coordinates - 3 possible functions) */
   int gencoord[3];                  /* index of gencoord for moving point (3 coordinates - 3 possible gencoords) */
   SBoolean isMovingPoint;           /* whether the point has non-constant coordinates */
   SBoolean isVia;                   /* is the point a via point? */
   PointRange viaRange;              /* the range of the via point */
   SBoolean is_auto_wrap_point;      /* was this point calc-ed by auto wrapper? */
   double wrap_distance;             /* stores distance of wrap over object surface */
   double* wrap_pts;                 /* points wrapping over surface */
   int num_wrap_pts;                 /* number of points wrapping over surface */
   double undeformed_point[3];       /* 'point' prior to deformation */
} MusclePoint;                       /* properties of a muscle point */


STRUCT {
   int segment;                      /* segment this muscle point is fixed to */
   int state;                        /* is this point on or off */
   double point[3];                  /* xyz coordinates of the point */
   double ground_pt[3];              /* coords of point in ground frame */
   int fcn_index[3];                 /* index of function for moving point */
   int gencoord[3];                  /* index ofgencoord for moving point */
   int normal_count;                 /* number of polygons used for this normal */
   float normal[3];                  /* point normal used for ligaments */
   int numranges;                    /* number of ranges where point is active */
   PointRange* ranges;               /* array of ranges */
   double undeformed_point[3];       /* 'point' prior to deformation */
} LigamentPoint;                     /* properties of a muscle point */


STRUCT {
   char* name;                       /* name of muscle group */
   int number_of_muscles;            /* number of muscles in group */
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
} SegmentGroup;


STRUCT {
   char* name;
   int   num_gencoords;
   int   genc_array_size;
   int*  gencoord;
} GencoordGroup;


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
   PolyhedronStruct* wobj;
   DrawingMode drawmode;
   SBoolean selected;
   int material;
   float origin_x;
   float origin_y;
   float origin_z;
   float scale_factor[3];
   long drawmodemenu;
} WorldObject;


STRUCT {
   char *name;                /* name of point */
   int segment;               /* index of segment point is on */
   int floorSegment;          /* index of segment floor is attached to */ 
   char* floorName;           /* name of corresponding floor object */
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
#ifndef ENGINE
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
   Marker* marker;                  /* markers for Solver model */
   double htr_o[3];                 /* origin of htr segment in solver */
   double htr_x[3];                 /* direction of htr seg Xaxis in solver */
   double htr_y[3];                 /* direction of htr seg Yaxis in solver */
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
   DofType type;                     /* type (constant,function) of dof */
   DofKey key;                       /* one of: tx, ty, tz, r1, r2, r3 */
   double value;                     /* current value of this dof */
   int funcnum;                      /* if type=function, function number */
   int gencoord;                     /* if type=function, gencoord number */
   SDDof sd;                         /* holds stuff for SD/FAST */
} DofStruct;                         /* description of a dof */


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
   int restraint_func_num;           /* torque func that restrains gc throughout ROM */
   int restraint_sdcode_num;         /* func number in SD/FAST code */
   int restraint_user_num;           /* func number as user put in joint file */
   SBoolean restraintFuncActive;     /* is restraint function active? */
   int min_restraint_func_num;       /* torque func that restrains gc at min ROM */
   int min_restraint_sdcode_num;     /* func number in SD/FAST code */
   int min_restraint_user_num;       /* func number as user put in joint file */
   int max_restraint_func_num;       /* torque func that restrains gc at max ROM */
   int max_restraint_sdcode_num;     /* func number in SD/FAST code */
   int max_restraint_user_num;       /* func number as user put in joint file */
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
   int wrap_object;                  /* muscle is auto-wrapped over this object */
   int wrap_algorithm;               /* wrapping algorithm (see constants in wefunctions.h) */
   int startPoint;                   /* wrap only those muscle segments between startPoint */
   int endPoint;                     /* and endPoint */
   double c[3];                      /* previous c point */
   double r1[3];                     /* previous r1 (tangent) point */
   double r2[3];                     /* previous r2 (tangent) point */
   MusclePoint mp_wrap[2];           /* the two muscle points created when the muscle wraps */
} MuscleWrapStruct;                  /* a wrap object instance for a specific muscle */

STRUCT SaveMusclePath SaveMusclePath;

STRUCT {
   int num_orig_points;             /* number of user-defined muscle points */
   int num_points;                   /* total number of muscle points */
   MusclePoint* mp_orig;             /* list of user-defined muscle points */
   MusclePoint** mp;                 /* list of muscle points after auto wrapping */
   int mp_orig_array_size;           /* current size of mp_orig[] */
   int mp_array_size;                /* current size of mp[] */
   SaveMusclePath *saved_copy;
} MusclePathStruct;

STRUCT {
   char* name;                       /* name of muscle */
   SBoolean display;                 /* whether or not to display this muscle */
   SBoolean output;                  /* used only in the Dynamics Pipeline */
   SBoolean selected;                /* whether or not this muscle is selected */
   MusclePathStruct *musclepoints; /* pointer to the musclepoints */
   int numgroups;                    /* number of groups to which musc belongs*/
   int* group;                       /* list of muscle group numbers */
   double* max_isometric_force;      /* maximum isometric force */
   double* pennation_angle;          /* pennation angle of muscle fibers */
   double* optimal_fiber_length;     /* muscle fiber length */
   double* resting_tendon_length;    /* resting length of tendon */
   double* min_thickness;            /* minimum thickness of muscle line */
   double* max_thickness;            /* maximum thickness of muscle line */
   int* min_material;                /* material to draw with for activation = 0.0 */
   int* max_material;                /* material to draw with for activation = 1.0 */
   double activation;                /* activation level of muscle */
   double initial_activation;        /* default/initial activation level of muscle */
   double muscle_tendon_length;      /* musculotendon length */
   double fiber_length;              /* muscle fiber length */
   double tendon_length;             /* tendon length */
   double muscle_tendon_vel;         /* musculotendon velocity */
   double fiber_velocity;            /* muscle velocity */
   double tendon_velocity;           /* tendon velocity */
   double force;                     /* force in musculotendon unit */
   double generated_power;           /* power generated by this muscle */
   double applied_power;             /* power this muscle applies to body segments */
   int nummomentarms;                /* number of moment arms (= # of gencoords) */
   double* momentarms;               /* list of moment arm values */
   SplineFunction* tendon_force_len_curve; /* tendon force-length curve */
   SplineFunction* active_force_len_curve; /* muscle active force-length curve */
   SplineFunction* passive_force_len_curve; /* muscle passive force-length curve */
   double* max_contraction_vel;      /* maximum contraction velocity */
   SplineFunction* force_vel_curve;  /* muscle force-velocity curve */
   int num_dynamic_params;           /* size of dynamic_params array */
   char** dynamic_param_names;       /* list of dynamic parameter names */
   double** dynamic_params;          /* array of dynamic (muscle model) parameters */
   double dynamic_activation;        /* dynamic value of muscle activation */
   double energy;                    /* energy muscle has given to body segments */
   SplineType* excitation_format;    /* format for excitation function */
   int excitation_abscissa;          /* excit. is func of this gencoord (-1 = time) */
   double excitation_level;          /* current level of excitation */
   int excitation_index;             /* excitation data point used last time */
   SplineFunction* excitation;       /* excitation (activation) sequence */
   int* muscle_model_index;          /* index for deriv, init, & assign func arrays */
   SBoolean wrap_calced;             /* has wrapping been calculated/updated? */
   int numWrapStructs;               /* number of wrap objects used for this muscle */
   MuscleWrapStruct** wrapStruct;    /* array of wrap objects to auto-wrap over */
} SaveMuscle;                      /* properties of a musculotendon unit */

STRUCT {
   char* name;                       /* name of muscle */
   SBoolean display;                 /* whether or not to display this muscle */
   SBoolean output;                  /* used only in the Dynamics Pipeline */
   SBoolean selected;                /* whether or not this muscle is selected */
   MusclePathStruct *musclepoints;  /* information about muscle points */
   int numgroups;                    /* number of groups to which musc belongs*/
   int* group;                       /* list of muscle group numbers */
   double* max_isometric_force;      /* maximum isometric force */
   double* pennation_angle;          /* pennation angle of muscle fibers */
   double* optimal_fiber_length;     /* muscle fiber length */
   double* resting_tendon_length;    /* resting length of tendon */
   double* min_thickness;            /* minimum thickness of muscle line */
   double* max_thickness;            /* maximum thickness of muscle line */
   int* min_material;                /* material to draw with for activation = 0.0 */
   int* max_material;                /* material to draw with for activation = 1.0 */
   double activation;                /* activation level of muscle */
   double initial_activation;        /* default/initial activation level of muscle */
   double muscle_tendon_length;      /* musculotendon length */
   double fiber_length;              /* muscle fiber length */
   double tendon_length;             /* tendon length */
   double muscle_tendon_vel;         /* musculotendon velocity */
   double fiber_velocity;            /* muscle velocity */
   double tendon_velocity;           /* tendon velocity */
   double force;                     /* force in musculotendon unit */
   double generated_power;           /* power generated by this muscle */
   double applied_power;             /* power this muscle applies to body segments */
   int nummomentarms;                /* number of moment arms (= # of gencoords) */
   double* momentarms;               /* list of moment arm values */
   SplineFunction* tendon_force_len_curve; /* tendon force-length curve */
   SplineFunction* active_force_len_curve; /* muscle active force-length curve */
   SplineFunction* passive_force_len_curve; /* muscle passive force-length curve */
   double* max_contraction_vel;      /* maximum contraction velocity */
   SplineFunction* force_vel_curve;  /* muscle force-velocity curve */
   int num_dynamic_params;           /* size of dynamic_params array */
   char** dynamic_param_names;       /* list of dynamic parameter names */
   double** dynamic_params;          /* array of dynamic (muscle model) parameters */
   double dynamic_activation;        /* dynamic value of muscle activation */
   double energy;                    /* energy muscle has given to body segments */
   SplineType* excitation_format;    /* format for excitation function */
   int excitation_abscissa;          /* excit. is func of this gencoord (-1 = time) */
   double excitation_level;          /* current level of excitation */
   int excitation_index;             /* excitation data point used last time */
   SplineFunction* excitation;       /* excitation (activation) sequence */
   int* muscle_model_index;          /* index for deriv, init, & assign func arrays */
   SBoolean wrap_calced;             /* has wrapping been calculated/updated? */
   int numWrapStructs;               /* number of wrap objects used for this muscle */
   MuscleWrapStruct** wrapStruct;    /* array of wrap objects to auto-wrap over */
   SaveMuscle *saved_copy;           /* pointer to the saved version of the muscle */
} MuscleStruct;                      /* properties of a musculotendon unit */


STRUCT {
   int numpoints;                    /* number of attachment points */
   LigamentPoint* pt;                /* the list of attachment points */
   int pt_array_size;                /* current size of attachment point array */
   SBoolean has_wrapping_points;     /* does this line have wrapping pts? */
   int first_on_point;               /* number of the first active attachment pt */
   int num_on_points;                /* number of currently active attachment pts */
} LigamentLine;


STRUCT {
   char* name;                       /* name of ligament */
   SBoolean display;                 /* whether or not to display this ligament */
   SBoolean selected;                /* whether or not this ligament is selected */
   double activation;                /* for color display */
   int numlines;                     /* number of ligament lines */
   int line_array_size;              /* current size of ligament line array */
   LigamentLine* line;               /* array of lines which define ligament border */
   DrawingMode drawmode;             /* how to display this ligament */
   SBoolean show_normals;            /* display surface normal vectors? */
} LigamentStruct;                    /* properties of a musculotendon unit */


STRUCT{
    double num1[3];
    double num2[3];
    double vec[3];
    SBoolean flag;
    char* musc_name;
    char* segment_name;
} DglValueStruct;                  


STRUCT {                        /* ---- motion object record: */
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

enum
 {                          /* ---- motion object channel componant ids: */
   MO_TX, MO_TY, MO_TZ,              /* translation or position */
   MO_VX, MO_VY, MO_VZ,              /* vector */
   MO_SX, MO_SY, MO_SZ,              /* scale */
   MO_CR, MO_CG, MO_CB,              /* color */
   MO_X, MO_Y, MO_Z                  /* axis */
};


STRUCT {                        /* ---- motion object animation channel record: */
   int component;                    /* animated componant (see componant ids above) */
   int column;                       /* motion column index */
} MotionObjectChannel;


STRUCT {                        /* ---- motion object instance record: */
   int object;                       /* the motion object's definition */
   int segment;                      /* the body segment to which the motion object is attached */
   int num_channels;                 /* the number of animated channels for this motion object */
   MotionObjectChannel* channels;    /* the motion object's channel descriptions */
   DMatrix currentXform;             /* the motion object's current parent-to-local xform */
   MaterialStruct currentMaterial;   /* the motion object's current material */
   double current_value;             /* */
   DrawingMode drawmode;             /* the motion object's current draw mode */
   SBoolean visible;
} MotionObjectInstance;


STRUCT {
   double current_value;             /*  */
   int current_frame;                /*  */
   double** gencoords;               /*  */
   double** genc_velocities;         /*  */
   int num_motion_object_instances;
   MotionObjectInstance* motion_object_instances;
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
} MotionModelOptions;                /*  */


STRUCT {
   char* name;
   double x_coord;
} MotionEvent;


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
   MotionEvent* event;               /* array of motion events */
   GLfloat event_color[3];           /* color for event lines and text in plots */
   int realtime_circular_index;      /* magical index used to continuously add realtime samples */
   int simulation_index;             /* index of latest results from a simulation DLL */
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
   int genc;                         /* gencoord number, if xvar is a gencoord */
   int motion_column;                /* index of data column, if xvar is a motion curve */
   char name[150];                   /* name of x variable */
} XvarStruct;                        /* contains plotmaker xvar information */


STRUCT {
   YvarType type;                    /* type of yvar (MOMENT_TYPE, OTHER_TYPE) */
   int yvariable;                    /* the y variable chosen for plotting */
   int genc;                         /* if MOMENT_TYPE, the gencoord to use */
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
   MotionEvent* event;               /* array of motion events, if curve is from a motion */
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
   int numFileEvents;                /* number of plot file events */
   PlotFileEvent *fileEvent;         /* events (originally from motions) read in from a plot file */
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
   int windowX1, windowY1;           /* coordinates of lower left corner of model window */
   int windowHeight, windowWidth;    /* height and width of model window */
   float tx, ty, tz;                 /*  */
   float start_tx;                   /*  */
   float start_ty;                   /*  */
   float start_tz;                   /*  */
   short rx, ry, rz;                 /*  */
   int camera_segment;               /* the segment the camera is attached to (or -1) */
   double transform_matrix[4][4];    /*  */
   int default_view;                 /* which of the saved views is the default */
   GLfloat z_vector[3];              /* world coords of vector out of screen */
   GLint viewport[4];                /* viewport as returned by OpenGL */
   GLdouble modelview_matrix[16];    /* model view matrix as returned by OpenGL */
   GLdouble projection_matrix[16];   /* projection matrix as returned by OpenGL */
   long view_menu;
   int num_file_views;               /* number of views defined in joints file */
   double saved_view[MAXSAVEDVIEWS][4][4]; /* array of saved view transformations */
   char* view_name[MAXSAVEDVIEWS];        /* names of saved views */
   SBoolean view_used[MAXSAVEDVIEWS];     /* has this view been saved by user? */
   long maindrawmodemenu;            /*  */
   long allsegsdrawmodemenu;         /*  */
   long allligsdrawmodemenu;         /*  */
   long allworlddrawmodemenu;        /*  */
   long alldrawmodemenu;             /*  */
   GLdouble fov_angle;               /*  */
   GLdouble aspect_ratio;            /*  */
   GLdouble near_clip_plane;         /*  */
   GLdouble far_clip_plane;          /*  */
   double x_zoom_constant;           /*  */
   double y_zoom_constant;           /*  */
   double model_move_increment;      /*  */
   SBoolean continuous_motion;       /* animate the model continuously? */
   SBoolean display_motion_info;     /* display info about current motion? */
   unsigned display_animation_hz;    /* bit flags for displaying animation hz */
   MotionSequence* applied_motion;   /* motion that is currently applied to model */
   MotionSequence* current_motion;   /* motion linked to hot keys, continuous motion */
   double motion_value;              /* value of current motion */
   int muscle_array_size;            /*  */
   int nummuscleson;                 /*  */
   int* muscleson;                   /*  */
   MuscleMenu mgroup[GROUPBUFFER];   /*  */
   int menucolumns[COLUMNBUFFER];    /*  */
   long muscle_cylinder_id;          /*  */
#if INCLUDE_MSL_LENGTH_COLOR
   double muscle_color_factor;       /* controls muscle length colorization */
#endif
   SBoolean show_selected_coords;    /*  */
   SBoolean show_crosshairs;         /*  */
   SBoolean show_all_muscpts;        /*  */
   SBoolean fast_muscle_drawing;
   SBoolean show_highlighted_polygon;/*  */
   SBoolean show_shadow;             /*  */
   SBoolean trackball;               /*  */
   GLfloat background_color[3];      /*  */
   GLfloat segment_axes_color[3];    /*  */
   GLfloat vertex_label_color[3];    /*  */
   GLfloat rotation_axes_color[3];   /*  */
   GLfloat crosshairs_color[3];      /*  */
   SBoolean background_color_spec;   /*  */
   SBoolean segment_axes_color_spec; /*  */
   SBoolean vertex_label_color_spec; /*  */
   SBoolean rotation_axes_color_spec;/*  */
   SBoolean crosshairs_color_spec;   /*  */
   double current_gear;              /*  */
   SelectedPolygon hpoly;            /*  */
   float muscle_point_radius;        /*  */
   long muscle_point_id;             /*  */
   int numdevs;                      /*  */
   int* devs;                        /* button nums to control joint movement */
   int* dev_values;                  /*  */
#if INCLUDE_SNAPSHOT
   int      snapshot_mode;
   int      snapshot_counter;
   char*    snapshot_file_base_name;
   char*    snapshot_file_suffix;
   SBoolean snapshot_include_depth;
#endif
   ModelMaterials mat;
} DisplayStruct;                     /*  */

#if INCLUDE_SNAPSHOT
enum {
   SNAPSHOT_INACTIVE,
   SNAPSHOT_INSTANT,
   SNAPSHOT_CONTINUOUS,
   SNAPSHOT_MOTION
};
#endif

STRUCT {
   int shadem;                       /*  */
   float tx, ty, tz;                 /*  */
   short rx, ry, rz;                 /*  */
   int nummuscleson;                 /*  */
   int* muscleson;                   /*  */
   int musclepts;                    /*  */
   int showpoly;                     /*  */
   SelectedPolygon hpoly;            /*  */
   SBoolean marker_visibility;
} SaveDisplay;                       /*  */


STRUCT {
   char *name;                       /* joint name, not used yet */
   int order[4];                     /* transformation order, not used yet */
   DofStruct dofs[6];                /* the six dofs (rx,ry,rz,tx,ty,tz) */
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
   int restraint_func_num;
   int restraint_user_num;
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
   int wrapobj;
   int start_pt;
   int end_pt;
   int wrap_algorithm;
} MuscWrapAssoc;

STRUCT {
   int constraint_obj;
   int numPoints;
   ConstraintPoint *savedPoints;
} SaveConstraintPointAssoc;

struct SaveMusclePath {
   int num_orig_points;             /* number of user-defined muscle points auto-wrap pts */
   MusclePoint* mp_orig;             /* list of user-defined muscle points */
   int mp_orig_array_size;           /* current size of mp_orig[] */
   MuscleStruct *owner;
   int temp_index;
};                       /* saved muscle point array */


STRUCT {
   int numsavedmuscs;                /*  */
   int numsavedpaths;                /*  */
   int numsavedjnts;                 /*  */
   int numsavedgencs;                /*  */
   int numsavedbones;                /*  */
   int numsavedsegments;
   int numsavedmuscgroups;
   MuscleStruct default_muscle;      /*  */
   SaveMuscle* muscle;             /*  */
   SaveMusclePath* musclepath;
   SaveSegments *segment;
   SaveJoints* joint;                /*  */
   SaveGencoords *gencoord;
   SplineFunction* function;         /*  */
   SaveDisplay disp;                 /*  */
   MuscleGroup *muscgroup;
   int            num_wrap_objects;
   WrapObject*    wrapobj;
   char**         wrapobjnames;
   int            num_muscwrap_associations;
   MuscWrapAssoc* muscwrap_associations;
   int           num_deforms;
   DeformObject* deform;
   int num_markers;                  /*  */
   SaveMarker* marker;               /*  */
   int num_conspt_associations;
   SaveConstraintPointAssoc *conspt_associations;
   int num_constraint_objects;
   ConstraintObject *constraintobj;
   char**         constraintobjnames;
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
   int muscle_array_size;            /* current size of MuscleStruct array */
   int ligament_array_size;          /* current size of LigamentStruct array */
   int muscgroup_array_size;         /* current size of MuscleGroup array */
   int seggroup_array_size;          /* current size of SegmentGroup array */
   int gencgroup_array_size;         /* current size of GencoordGroup array */
   int genc_array_size;              /* current size of GeneralizedCoord array */
   int segment_array_size;           /* current size of SegmentStruct array */
   int joint_array_size;             /* current size of JointStruct array */
   int func_array_size;              /* current size of SplineFunction array */
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
   long musclegroupcascademenu;      /* pop-up muscle group cascade menu */
   long *musclegroupsubmenu;        /* */
   long jointmenu;                   /* pop-up joint menu */
   long jointmenu2;                  /* pop-up joint menu for jrf x */
   long jointmenu3;                  /* pop-up joint menu for jrf y */
   long jointmenu4;                  /* pop-up joint menu for jrf z */
   long jointmenu5;                  /* pop-up joint menu for jrf magnitude */
   long xvarmenu;                    /* pop-up x-var menu (gencoords and motions) */
   long gencoordmenu;                /* pop-up gencoord menu */
   long gencoordmenu2;                /* pop-up gencoord menu */
   long gencoord_group_menu;         /* pop-up gencoord group menu */
   long momentgencmenu;              /* gencoord menu used for moment y-var */
   long momentarmgencmenu;           /* gencoord menu used for moment arm y-var */
   long momentarmnumgencmenu;        /* gencoord menu used for moment arm y-var */
   long maxmomentgencmenu;           /* gencoord menu used for maxmoment y-var */
   long doftypemenu;                 /* jointeditor menu for selecting dof types */
   long segmentmenu;                 /* pop-up menu of segment names */
   long motionplotmenu;              /* pop-up menu of motions linked to this model */
   long motionmenu;                  /* pop-up menu of motions, no column data */
   long material_menu;               /* pop-up menu of materials */
   long markerMenu;                  /* pop-up menu of segment-based marker links */
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
   GeneralizedCoord* gencoord;       /*  */
   SplineFunction* function;         /* functions used for dofs and gencoords and muscle points */
   MuscleStruct* muscle;             /*  */
   MuscleStruct default_muscle;      /*  */
   LigamentStruct* ligament;         /* array of ligament structures */
   DisplayStruct dis;                /*  */
   ModelSave save;                   /*  */
   TextLine genc_help[2*GENBUFFER];  /* help text for controlling gencs with keys*/
   int num_dynamic_params;           /* size of dynamic_param_names array */
   char** dynamic_param_names;       /* names of dynamic muscle parameters */
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
   WrapObject* wrapobj;
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
#ifndef ENGINE
   glut_mutex_t modelLock;
#endif
   SBoolean realtimeState;
} ModelStruct;

typedef struct loopstruct {
   ModelStruct *ms;
   int model_num;
   int locked_q;
   double q_value;
   int num_qs;
   int num_resids;
   int *qs;
   int *segs;
   int *joints;
   int num_segs;
   int num_jnts;
   SBoolean first_iter;
} LoopStruct;

STRUCT {
   ModelStruct *ms;
   SBoolean *loopUsed;
   int *gencoord_list;
   int controlled_gc;
   double controlled_value;
   SBoolean first_iter;
} IKStruct;

STRUCT {
   ModelStruct *model;
   int *gencoord_list;
   int controlled_gc;
   double controlled_value;
   SBoolean first_iter;
   SBoolean test;
   double tolerance;
   SBoolean *consUsed;
} ConstraintSolverStruct;

STRUCT {
   ModelStruct *model;
   SBoolean first_iter;
   int controlled_gc;
   double controlled_value;
   int numQs;
   int *gencoord_list;
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
   SBoolean loop;
   int joint;
   int body1;
   int body2;
   double coords1[3];
   double coords2[3];
} MarkerStruct;

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

#endif /*MODELPLOT_H*/

