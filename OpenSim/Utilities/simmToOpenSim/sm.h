/*******************************************************************************

  SM.H

  (c) Copyright 2001-9, MusculoGraphics, Inc.
      a division of Motion Analysis Corp.

  This version of the Solver header file is for use with Solver 1.4.0,
  released April 8, 2009.

  Authors: Peter Loan, Krystyne Blaikie

  This file contains definitions of enums and structures that are used in
  the interface between the Solver library and its clients. The functions
  comprising the interface have prototypes at the end of this file, and
  are defined in client.cpp. All of the code in this file is C (not C++),
  and the prototypes are within an "extern C" scope. Thus the Solver library
  can be called from a C or a C++ client.

  Terminology:
     DOF: not really a degree of freedom, but rather a "potential" degree
          of freedom. Every joint has six DOFs, three translational and
          three rotational. In a hinge joint, 5 of the DOFs are constants,
          and one of the rotational DOFs is a function of a GenCoord.
     GenCoord: (short for generalized coordinate) a variable with an
          operating range that controls one or more DOFs. In the simplest
          case, each GenCoord controls a single DOF, and the value of the
          DOF equals the value of the GenCoord. For example, in a hinge
          joint, one rotational DOF would be a linear function (with
          slope = 1.0) of a GenCoord, and that GenCoord would not be used
          in any other joints. In the general case, a GenCoord is an
          abstract value that can control any number of DOFs. In the SIMM
          knee for example, the GenCoord "knee_angle" controls not only
          the flexion angle, but five other DOFs in two separate joints
          are non-linear functions of knee_angle.
      Q: another name for a GenCoord.
*******************************************************************************/

#ifndef _SM_H_
#define _SM_H_

#ifdef _WINDOWS
    #define DLL __declspec(dllexport)
#else
    #define DLL
#endif

#define SMDEBUG 0

#define UNDEFINED_DOUBLE 99999.99

typedef double smMatrix4x4[4][4];
typedef double smPoint3[3];
typedef void* smModelID;

typedef enum { smNo = 0, smYes = 1 } smBoolean;

typedef enum { smNoAxis = 0, smX, smNegX, smY, smNegY, smZ, smNegZ, smNoAlign } smAxes;

typedef enum {  smXYZ = 1, smXZY, smYXZ, smYZX, smZXY, smZYX } smEulerRotationOrder;

typedef enum { PFX, PFY, PFZ, PPX, PPY, PMZ, NUM_TYPE1_CHANNELS } smForcePlate1Channels;

typedef enum { FX, FY, FZ, MX, MY, MZ, NUM_TYPE24_CHANNELS } smForcePlate24Channels;

typedef enum { FX12, FX34, FY14, FY23, FZ1, FZ2, FZ3, FZ4, NUM_TYPE3_CHANNELS } smForcePlate3Channels;

typedef enum { FZC, FZD, FZA, FZB, FYAC, FXDC, FXAB, FYBD, NUM_TYPE5_CHANNELS } smForcePlate5Channels;

typedef enum { smMale, smFemale, smNoGender } smGender;

typedef enum { smTRBFile, smTRCFile, smC3DFile, smNoFileType } smMotionFileType;

typedef enum { smFPForcePosition = 1, smFPSixChannel, smFPEightChannel, smFPCalSixChannel, smFPSixEightChannel, smKyowaDengyo } smForcePlateType;

typedef enum { smUnsignedShort, smSignedShort } smIntegerFormat;

typedef enum { smOwnedBySIMM, smOwnedByEVaRT, smOwnedBySolver, smOwnedByOther } smOwner;

typedef enum { smSIMMFile, smMODFile, smOtherFile} smSource;

typedef enum { smStepFunction, smLinearFunction, smNaturalCubicSpline, smGCVSpline} smFunctionType;

typedef enum {
   VSacral = 0,
   RASIS, LASIS, RPSIS, LPSIS,
   RKneeLat, LKneeLat,
   RAnkleLat, LAnkleLat,
   RHeel, LHeel,
   RToe, LToe,
   RGreaterTroc, LGreaterTroc,
   RKneeMed, LKneeMed,
   RAnkleMed, LAnkleMed,
   RShoulder, LShoulder,
   RElbowLat, LElbowLat,
   RWristLat, LWristLat, RWristFront, LWristFront,
   RElbowMed, LElbowMed,
   RWristMed, LWristMed, RWristBack, LWristBack,
   HeadRear, HeadTop, HeadFront,
   HeadFrontRight, HeadFrontLeft, HeadBackRight, HeadBackLeft,
   RMiddleFinger, LMiddleFinger,
   RThumb, LThumb,
   NumMarkers
} smOrthoTrakMarkerSet;

typedef enum {
   F1M1 = 0, F1M2, F1M3,
   F2M1, F2M2, F2M3,
   F3M1, F3M2, F3M3,
   F4M1, F4M2, F4M3,
   F5M1, F5M2, F5M3,
   NumFingerMarkers
} smFingerMarkerSet; // used for both right and left hands

typedef enum {
   UNSPECIFIED_ANALOG_CHANNEL,
   ELAPSED_TIME,
   FORCE_PLATE_FORCE,
   FORCE_PLATE_MOMENT,
   FORCE_LOCATION,
   EMG,
   FORCE_PLATE_FREE_TORQUE,
   OTHER_DATA
} smAnalogChannelType;

typedef enum {
   smPointGroup,
   smAnalogGroup,
   smForcePlateGroup,
   smSubjectGroup,
   smEventGroup,
   smOtherGroup
} smC3DGroup;

typedef enum {
   smRadians = 0,
   smDegrees,
   smMillimeters,
   smCentimeters,
   smMeters,
   smNewtons,
   smNewtonMeters,
   smNewtonCentimeters,
   smNewtonMillimeters,
   smVolts,
   smMillivolts,
   smOtherUnits
} smUnit;

typedef enum { smUndefinedMarker = 0, smCriticalMarker, smSemiCriticalMarker, smOptionalMarker, smFixedMarker } smMarkerType;

typedef enum
{
   smNoError = 0,
   smNullPointer,
   smImproperInput = 10,
   smMaxIter = 15,
   smUncontrolledGenCoord = 19,
   smZeroFrames = 22,
   smReadError = 24,
   smInputError = 25,
   smModelError,
   smJNTFileError,
   smFileError,
   smFormatError,
   smLeastSquaresSolverFailure,
   smZeroMarkers,
   smUncontrolledGCWarning,
   smFileWarning
} smReturnCode;

typedef enum
{
   smLevenbergMarquart,   /* the default method, which is more robust */
   smGaussNewton          /* the faster method, not guaranteed to converge */
} smSolverMethod;

typedef enum
{
   smHinge,             /* one rotational DOF */
   smUniversal,         /* two rotational DOFs */
   smSpherical,         /* three rotational DOFs, axes = X, Y, Z */
   smGimbal,            /* three rotational DOFs, user-specified axes */
   smSimm,              /* joint as defined in SIMM */
   smSixdof,            /* three translational, three rotational DOFs */
   smFixed,             /* no DOFs, all translations and rotations are fixed */
   smSlider,            /* one translational DOF */
   smPlanar,            /* two translational, one rotational DOF */
   smSlaveHinge,        /* slave, one rotational DOF */
   smSlaveUniversal,    /* slave, two rotational DOFs */
   smSlaveSpherical,    /* slave, three rotational DOFs, axes = X, Y, Z */
   smSlaveSlider,       /* slave, one translational DOF */
   smPoint,             /* two rotational, one translational DOF */
   smTranslational,     /* three translational DOFs */
   smMixed,             /* joint with three rotations that can be slaves or unique DOFs */
   smOtherJoint         /* unknown type */
} smJointType;

typedef enum
{
   smOrderR1 = 0,       /* where in the order the R1 rotation is */
   smOrderR2,           /* where in the order the R2 rotation is */
   smOrderR3,           /* where in the order the R3 rotation is */
   smOrderT             /* where in the order the translation rotation is */
} smTransformOrder;     /* order of transforms in a SIMM joint */

typedef enum
{
   smConstant = 0,      /* constant, number should be specified */
   smFunction           /* function of a generalized coordinate */
} smDOFType;

typedef enum
{
   smSolveMe,     /* solve this element (Q, segment) in the normal way */
   smDontSolveMe  /* do not solve this element (point, translational segments) */
} smSolveType;

typedef enum
{
   smLocalAxes,   /* axes are specified in local reference frame */
   smGlobalAxes   /* axes are specified in global reference frame */
} smAxisFrame;

typedef enum
{
   smDofT1,     /* for gencoords used only in a single TX component of a joint */
   smDofT2,     /* for gencoords used only in a single TY component of a joint */
   smDofT3,     /* for gencoords used only in a single TZ component of a joint */
   smDofR1,     /* for gencoords used only in a single R1 component of a joint */
   smDofR2,     /* for gencoords used only in a single R2 component of a joint */
   smDofR3,     /* for gencoords used only in a single R3 component of a joint */
   smDofOther   /* for [pre-defined] gencoords which may be used in more than one joint/DOF */
} smDOFElement; /* which DOF[s] a certain gencoord is used in */

typedef enum
{
   smGCGenerated,    /* created by Solver; used in only one DOF in one joint */
   smGCSIMM,         /* defined in smModel.gencoord[] array, used in SIMM joints */
   smGCShared,       /* created by Solver; used in master/slave joints */
   smGCOther         /* other; is an error if gencoord remains this type */
} smGenCoordType;

typedef enum
{
   smGlobalPose,   /* "global" pose, like htr2 files */
   smInitPose,     /* the pose from the static init TRC file */
   smFirstFrame    /* the first frame of data from the motion TRC file */
} smBasePosition;

typedef enum 
{
   smConstraintPlane, 
   smConstraintEllipsoid, 
   smConstraintSphere, 
   smConstraintCylinder,
   smOtherConstraint
} smConstraintObjectType;

typedef struct {
   smPoint3 normal;
   double d;
} smPlaneStruct;


typedef struct
{
   int type;                   /* type not currently used */
   int numMarkers;             /* number of markers in markerCoordList */
   int frameNum;               /* client-defined frame number */
   double time;                /* client-defined time of frame */
   smPoint3 *markerCoordList;  /* array of 3D marker coordinates in global space */
   smUnit units;               /* units of marker coordinates */
} smTRCFrame;

typedef struct
{
   int type;                   /* type not currently used */
   double translation[3];      /* translation of segment */
   double rotation[3];         /* rotation of segment */
   double scale;               /* scale of segment; needed for smPoints */
} smHTRSegmentTransform;

typedef struct
{
   int type;                   /* type not currently used */
   int frameNum;               /* frame number; copied from input TRC frame */
   double time;                /* time of frame: copied from input TRC frame */
   int numSegments;            /* number of segments in this frame of data */
   smHTRSegmentTransform* segmentTransformList;  /* array of segment transforms */
} smHTRFrame;

typedef struct
{
   int type;                           /* type not currently used */
   int frameNum;                       /* frame number; copied from input TRC frame */
   double time;                        /* time of frame: copied from input TRC frame */
   int numSegments;                    /* number of segments in this frame of data */
   smMatrix4x4 *segmentTransformList;  /* array of segment transforms */
} smMatrixFrame;

typedef struct
{
   int type;                            /* type not currently used */
   double accuracy;                     /* desired accuracy of solution */
   smSolverMethod method;               /* which method to use when solving */
   int maxIterations;                   /* per frame, Solver will stop at this number */
   smEulerRotationOrder rotationOrder;  /* rotation order for HTR output */
   smUnit htrRotationUnits;             /* degrees or radians for HTR output */
   smUnit htrTranslationUnits;          /* length units for HTR output */
   smBasePosition basePosition;         /* which base position to use in HTR file */
   smAxes autoAlign;                    /* auto-align option when processing init pose */
   smAxisFrame axisFrame;               /* in which frame are rotation axes specified */
   smBoolean orientBodyToFrame;         /* do you want to orient the body before solving? */
   smBoolean jointLimitsOn;             /* do you want to enforce joint limits? */
   double globalScale;                  /* global scale factor that was used to make this model */
   smBoolean loopsOn;                   /* do you want to enforce closed loops */
   smBoolean fgContactOn;              /* do you want to turn on foot_ground contact? */
} smOptions;

typedef struct
{
   int type;                    /* type not currently used */
   double axis[3][3];           /* the three rotation axes for this joint */
   double range[6][2];          /* range limits for the six dofs */
   double stiffness[6][2];      /* stiffness to use at each range limit */
} smSixDofJoint;

typedef struct
{
   int type;                    /* type not currently used */
   double axis[3][3];           /* the three rotation axes for this joint */
} smFixedJoint;

typedef struct
{
   int type;                    /* type not currently used */
   double axis[3];              /* the translation axis for this joint */
   double range[2];             /* range limits for the dof */
   double stiffness[2];         /* stiffness to use at the range limit */
} smSliderJoint;

typedef struct
{
   int type;                    /* type not currently used */
   double axis[3];              /* rotation axis (translations are orthogonal to it) */
   double range[3][2];          /* range limits for the three dofs */
   double stiffness[3][2];      /* stiffness to use at the range limits */
} smPlanarJoint;

typedef struct
{
   int type;                    /* type not currently used */
   double axis[3];              /* the axis of the hinge */
   double range[2];             /* range limits for the dof */
   double stiffness[2];         /* stiffness to use at the range limit */
} smHingeJoint;

typedef struct
{
   int type;                    /* type not currently used */
   double axis[2][3];           /* the two rotation axes */
   double range[2][2];          /* range limits for the two dofs */
   double stiffness[2][2];      /* stiffness to use at the range limits */
} smUniversalJoint;

typedef struct
{
   int type;                    /* type not currently used */
   double range[3][2];          /* range limits for the three dofs */
   double stiffness[3][2];      /* stiffness to use at the range limits */
} smSphericalJoint;

typedef struct
{
   int type;                    /* type not currently used */
   double axis[3][3];           /* the three rotation axes */
   double range[3][2];          /* range limits for the three dofs */
   double stiffness[3][2];      /* stiffness to use at the range limits */
} smGimbalJoint;

typedef struct
{
   smDOFType type;    /* constant or function */
   smPoint3 axis;     /* axis of rotation or translation */
   int functionNum;   /* index of function that controls this DOF */
   int gencoordNum;   /* index of gencoord that controls this DOF */
   double value;      /* current value in user units */
} smDOF;              /* a "potential" degree of freedom */

typedef struct
{
   int type;            /* type not currently used */
   smTransformOrder transformOrder[4];  /* order of T, R1, R2, R3 */
   smDOF tx;            /* the X-translation DOF */
   smDOF ty;            /* the Y-translation DOF */
   smDOF tz;            /* the Z-translation DOF */
   smDOF r1;            /* a rotation DOF, not necessarily performed first */
   smDOF r2;            /* a rotation DOF, not necessarily performed second */
   smDOF r3;            /* a rotation DOF, not necessarily performed third */
   smJointType subtype; /* which type (e.g., slider) this SIMM joint represents */
} smSIMMJoint;

typedef struct
{
   int type;            /* type not currently used */
   int master;          /* index of the joint which controls this one */
   double ratio[3];     /* ratio slave gets from master DOFs (if 0, master DOF not part of slave) percentage that this slave gets, % = ratio / total */
   double axis[3];      /* the axis of the hinge */
} smSlaveHingeJoint;

typedef struct
{
   int type;            /* type not currently used */
   int master;          /* index of the joint which controls this one */
   double ratio[3];     /* ratio slave gets from master DOFs (if 0, master DOF not part of slave) percentages that this slave gets, % = ratio / total */
   double axis[2][3];   /* the two rotation axes */
} smSlaveUniversalJoint;

typedef struct
{
   int type;            /* type not currently used */
   int master;          /* index of the joint which controls this one */
   double ratio[3];     /* ratio slave gets from master DOFs (if 0, master DOF not part of slave) percentage that this slave gets, % = ratio / total */
   double axis[3];      /* the translation axis */
} smSlaveSliderJoint;

typedef struct
{
   int type;            /* type not currently used */
   int master;          /* index of the joint which controls this one */
   double ratio[3];     /* ratio slave gets from master DOFs (if 0, master DOF not part of slave) percentages that this slave gets, % = ratio / total */
} smSlaveSphericalJoint;

typedef struct
{
   int type;            /* type not currently used */
   int master;          /* index of the joint which controls this one */
   double ratio[3];     /* ratio slave gets from master DOFs (if 0, master DOF not part of slave) percentages that this slave gets, % = ratio / total */
   double range[3][2];          /* range limits for the three dofs */
   double stiffness[3][2];      /* stiffness to use at the range limits */
//   double axis[3][3];           /* the three rotation axes */
} smMixedJoint;

typedef struct
{
   int type;            /* type not currently used */
} smPointJoint;

typedef struct
{
   int type;            /* type not currently used */
} smTranslationalJoint;

typedef union
{
   smHingeJoint hinge;                      /* see smJointType for a description */
   smUniversalJoint universal;              /* of each of these joints */
   smSphericalJoint spherical;
   smGimbalJoint gimbal;
   smSIMMJoint simm;
   smSixDofJoint sixdof;
   smFixedJoint fixed;
   smSliderJoint slider;
   smPlanarJoint planar;
   smSlaveHingeJoint slaveHinge;
   smSlaveUniversalJoint slaveUniversal;
   smSlaveSliderJoint slaveSlider;
   smSlaveSphericalJoint slaveSpherical;
   smPointJoint point;
   smTranslationalJoint translational;
   smMixedJoint mixed;
} smJointUnion;

typedef struct
{
   smMarkerType type;      /* critical, optional, fixed, etc. */
   int index;              /* index of marker in global marker array (smModel.marker[]) */
   double weight;          /* marker weight for each segment */
   double localOffset[3];  /* offset in segment's local reference frame */ 
} smMarkerLink;

typedef struct
{
   int type;               /* type not currently used */
   char *name;             /* client-defined name of marker */
} smMarker;

typedef struct
{
   smFunctionType type;    /* step, linear, natural cubic, GCV */
   int userNum;            /* client-defined number of this function */
   int numPoints;          /* number of control points */
   double *x;              /* array of X coordinates of control points */
   double *y;              /* array of Y coordinates of control points */
} smConstraintFunction;

typedef struct
{
   char *name;             /* name of constraint point */
   int segmentIndex;       /* index of segment on which it is defined */
   smPoint3 offset;        /* offset of point from segment origin */
   double weight;          /* weight used to calculate residual */
   double tolerance;       /* tolerance of error in point distance from object - not for solving */
   smBoolean visible;      /* is point visible */
   smBoolean active;       /* is point active */
} smConstraintPoint;

typedef struct
{
   char *name;                               /* name of constraint object */
   smConstraintObjectType constraintType;    /* type of constraint object */
   int segmentIndex;                         /* index of the segment the object is attached to */
   smBoolean active;                         /* is the constraint object active? */
   smBoolean visible;                        /* is the constraint object visible? */
   smPoint3 radius;                          /* constraint object radius */
   double height;                            /* constraint object height (cylinder only) */
   smPlaneStruct plane;                      /* plane parameters (plane only) */
   smAxes constraintAxis;                    /* which axis to constrain over */
   smPoint3 rotationAxis;                    /* local-to-parent transform parameter */
   double rotationAngle;                     /* local-to-parent transform parameter */
   smPoint3 translation;                     /* local-to-parent transform parameter */
   smMatrix4x4 from_local_xform;             /* consobj-to-parent transform matrix */
   smMatrix4x4 to_local_xform;               /* parent-to-consobj transform matrix */
   int numPoints;                            /* number of constraint points */
   smConstraintPoint *points;                /* constraint points */
   int numQs;                                /* not used */
   int numJnts;                              /* not used */
   int *joints;                              /* not used */
   int *Qs;                                  /* not used */
} smConstraintObject;

typedef struct
{
   int type;                /* type not currently used */
   char *name;              /* name of this gencoord */
   double rangeStart;       /* minimum gencoord value */
   double rangeEnd;         /* maximum gencoord value */
   double stiffness1;       /* stiffness when gencoord goes below rangeStart */
   double stiffness2;       /* stiffness when gencoord goes above rangeEnd */
   double tolerance;        /* used to make setting the value more efficient */
   smBoolean locked;        /* gencoord cannot be changed from default value */
   smBoolean clamped;       /* gencoord constrained to remain within specified range (restraint func not used) */
   double defaultValue;     /* more for SIMM, but used here to set initial value */
   smUnit units;            /* units of this gencoord */
   int restraintFuncNum;    /* index of the constraint/restraint function */
   smBoolean restraintFuncActive;   /* is the restraintFunction active */
   smBoolean solve;         /* is Solver used to find value, or is it calculated after? */
} smGenCoord;

typedef struct
{
   int type;                     /* type not currently used */
   char *name;                   /* name of this segment */
   int parent;                   /* index of segment with joint to this segment */
   int child;                    /* index of child segment in joint - unless loop or reverse, equal to index of current segment */
   int loopSeg;                  /* index of segment that, when jointed to current seg, closes a loop */
   double length;                /* used only in HTR output */
   smJointType jointType;        /* type of joint from parent to this segment */
   smJointUnion joint;           /* joint from parent to this segment */
   double translation[3];        /* To specify the orientation of the segment with */
   double angle;                 /*    respect to its parent, use translation, angle, */
   smPoint3 axis;                /*    and axis */
   int beginMarker;              /* The orientation of the segment can also be */
   smPoint3 beginOffset;         /*    specified using beginMarker, endMarker, and */
   int endMarker;                /*    frontVector. In this case, an init pose file is */
   smPoint3 endOffset;           /*    needed as well. jointPositionMarker is just like */
   smPoint3 frontVector;         /*    beginMarker- they function the same, and only one */
   int jointPositionMarker;      /*    of them should be used. If both are specified, */
   smPoint3 jointPosition;       /*    jointPositionMarker will be used.  */
   int numMarkerLinks;           /* number of markers attached to this segment */
   smMarkerLink *markerLinkList; /* list of markers attached to this segment */
   smAxisFrame localAxis;        /* are parent->segment axes specified locally or globally? */
   smPoint3 scaling;             /* stored, but not used by Solver */
   smPoint3 endPoint;            /* stored, but not used by Solver */
   char *gaitScaleSegment;       /* name of the segment on which to base scaling */
   smPoint3 gaitScaleFactor;     /* scale factor btwn simm and static trc pose */
   smPoint3 htr_o;               /* origin of HTR segment */
   smPoint3 htr_x;               /* direction of X axis of HTR segment */
   smPoint3 htr_y;               /* direction of Y axis of HTR segment */
} smSegment;

typedef struct
{
   char *name;                   /* the name of the FG marker */
   int segmentIndex;             /* the index of the SolverModel segment it is attached to */
   int markerIndex;              /* the index of the SolverModel marker */
   int associatedMarkerIndex;    /* the index of its associated marker (that has the same name without the fg. */
   int associatedMarkerTRCIndex; /* the TRC index of the associated marker */
   double threshold;             /* the threshold for turning on the contact */
   smPoint3 staticPos;           /* the original position of the marker */
} fgMarkerStruct;

typedef struct
{
   int type;                                     /* type not currently used */ 
   smOwner owner;                                /* which program created the model: SIMM, EVaRT, Solver or Other? */
   smSource source;                              /* what format is the model derived from SIMM joint file or MOD file? */
   char *name;                                   /* name of this model */
   int numSegments;                              /* number of segments in this model */
   smSegment *segmentList;                       /* list of segments in this model */
   int numMarkers;                               /* number of markers in this model */
   smMarker *markerList;                         /* list of markers in this model */
   int numConstraintFunctions;                   /* used by joint files, not mod files */
   smConstraintFunction *constraintFunctionList; /* used by joint files, not mod files */
   int numSIMMGenCoords;                         /* used by joint files, not mod files */
   smGenCoord *SIMMGenCoordList;                 /* used by joint files, not mod files */
   smUnit rotationUnits;                         /* units that model is defined in */
   smUnit translationUnits;                      /* units that model is defined in */
   smOptions options;                            /* list of Solver options for this model */
   int numConstraintObjects;                     /* number of constraint objects in this model */
   smConstraintObject *constraintObjectList;     /* constraint objects */
   smAxes gravity;                               /* direction of gravity */
   smPlaneStruct groundPlane;                    /* description of the ground Plane */
   smBoolean gait;                               /* used for display in SIMM - set to true if using an OT model */
   /* used for SIMM display */
   char *bonefile;                               /* used for display in SIMM */
   char *bonepath;                               /* used for display in SIMM */
   char *musclefilename;                         /* used for display in SIMM */
   char *motionfilename;                         /* used for display in SIMM */
   double markerRadius;                          /* used for display in SIMM */
   double mvGear;                                /* used for display in SIMM */
   char *forceUnits;                             /* used for display in SIMM */
   char *lengthUnits;                            /* used for display in SIMM */
   smBoolean markerVisibility;                   /* used for display in SIMM */
   double loopTolerance;                         /* used to check status of loops */
   double loopWeight;                            /* used to calculate loop residuals when solving */
   smBoolean solveLoops;                         /* to turn loop solving on and off */
   int numFGMarkers;                             /* number of foot ground contact markers in the model */
   fgMarkerStruct *FGMarkerList;                 /* list of foot ground contact markers */
} smModel;

typedef struct
{
   int type;               /* not currently used */
   int numGenCoords;       /* the number of gencoords in the joint to this segment */
   int genCoordIndex[6];   /* the indices of these gencoords in the output Q array */
} smSegmentMapping;        /* for interpreting segment-based output */

typedef struct
{
   smGenCoordType type;    /* type of gencoord: SIMM, Generated, Shared */
   int segment;            /* segment whose joint this gencoord is in */
   smDOFElement dof;       /* which dof this gencoord controls */
   double axis[3];         /* the instantaneous axis of rotation/translation */
} smGenCoordMapping;       /* for interpreting Q-based output */

typedef struct
{
   int type;               /* type not currently used */
   int numGenCoords;       /* number of gencoords in the Q array */
   char **nameList;        /* names of gencoords as they appear in the Q array */
} smGCNames;

typedef struct
{
   int type;                            /* type not currently used */
   char *name;                          /* name of this segment */
   int numFrames;                       /* number of frames in this segment's HTR struct */
   smHTRSegmentTransform *htrDataList;  /* list of frames of HTR data */
} smHTRSegmentData;

typedef struct
{
   int type;                            /* type not currently used */
   int numSegments;                     /* number of segments in this HTR model */
   int *parentList;                     /* parent indices for each segment */
   int *frameNumList;                   /* frame numbers for each frame of data */
   char **segmentNameList;              /* names of segments in this HTR model */
   smOptions options;                   /* file output options */
   double dataFrameRate;                /* frame rate of data */
   int framesSolved;                    /* number of frames of HTR data */
   smHTRSegmentData *segmentDataList;   /* list of segments' data frames */
   smHTRFrame HTRBasePosition;          /* base position from which frames are measured */
} smHTRData;

typedef struct
{
   double dataRate;           /* data rate */
   double cameraRate;         /* camera rate */
   double origDataRate;       /* original data rate */
   int origStartFrame;        /* original starting frame */
   int origNumFrames;         /* original number of frames */
   int numFrames;             /* number of frames in file */
   int firstFrameNum;         /* user-defined number of first frame in file */
   int numMarkers;            /* number of named markers */
   smUnit units;              /* units of marker coordinates */
   char **markerNameList;     /* list of marker names */
} smTRCHeaderInfo;

typedef struct
{
   int type;                  /* type not currently used */
   char *filename;            /* name of file this TRC data came from */
   smTRCFrame* frameList;     /* list of TRC frames */
   smTRCHeaderInfo header;    /* information from file header */
} smTRCStruct;

typedef struct
{
   int numMarkersSolved;      /* number of markers used to solve frame */
   double rmsTotal;           /* total rms value of solved frame */
   int iterations;            /* number of iterations needed to solve frame */
   int totalCalls;            /* total number of calls to calculateResiduals */
   int inputFrameNum;         /* actual frame number */
   int numQs;                 /* number of Qs to solve */
   double *qerr;              /* Q error propagation for frame */
} smSolveInfoStruct;

typedef struct
{
   int firstFrameToSolve;     /* index of the first frame to solve */
   int lastFrameToSolve;      /* index of the last frame to solve */
   int frameIncrement;        /* amount to increment when solving */
   smBoolean cropEnds;        /* crop ends of TRC file when not all markers present? */
   smBoolean markerNamesFromDescriptions; /* used only for C3D files */
   smBoolean fgContactOn;      /* is foot ground contact on or off */ 
} smTRCOptions;


typedef struct
{
   smPoint3 translation;
   double angle;
   smPoint3 axis;
} smTransform;

typedef enum {
   smUnknownEventType = 0,
   smHeelStrikeEvent,
   smToeOffEvent
} smEventType;

typedef enum {
   smUnknownSegmentEvent = 0,
   smRightFootEvent,
   smLeftFootEvent,
   smOtherSegmentEvent
} smEventSegment;

typedef enum {
   smUnknownEvent = 0,
   smUserEvent,
   smForceEvent,
   smMarkerEvent
} smEventSource;

typedef struct {
   char* name;
   double xCoord;
   smEventType type;
   smEventSegment segment;
   smEventSource source;
   smBoolean displayFlag;
} smMotionEvent;

typedef struct {
   smForcePlateType type;
   double scaleFactor;
   double origin[3];              // raw origin[] values from c3d file
   double length;
   double width;
   int baselineRange[2];
   double calibrationMatrix[64];
   double transformMatrix[4][4];  // force plate reference frame w.r.t. lab frame
   int channels[9];
} smForcePlateSpec;

typedef struct {
   int index;
   int component;
   double baseline;
} smFPChannel;

typedef struct {
   int numMuscles;
   char** muscleNames;
   int* muscleIndices;
   double specifiedMVC;
   double actualMVC;
} smEMGChannel;

typedef struct {
   char* name;
   int index;
} smOtherChannel;

typedef struct {
   char* name;
   char* description;
   smUnit units;
   double frequency;
   double range;
   smAnalogChannelType type;
   union {
      smFPChannel force;
      smEMGChannel emg;
      smOtherChannel other;
   } u;
   int numMotionColumns;
   int* motionColumn;
} smAnalogChannel;

typedef struct {
   int numChannels;
   int numRows;
   smAnalogChannel *channelList;
   double* data;
   int numForceEvents;
   smMotionEvent* forceEvents;
} smAnalogStruct;

typedef struct {
   char* name;
   int number;
   smGender gender;
   float height;
   float weight;
   char* dateOfBirth;
   char* projectName;
} smSubjectStruct;

typedef smTRCStruct smMotionStruct;

typedef struct {
   unsigned char firstParameterBlock;
   unsigned char key;
   float scaleFactor;
   int numAnalogSamples;
   short firstDataBlock;
   short fourCharSupport;
   double analogGlobalScale;
   double* analogChannelScale;
   int* analogChannelOffset;
   smBoolean calMatricesSpecified;
   int numAnalogChannels;
   smIntegerFormat analogOffsetFormat;
} ExtraHeaderStuff;

typedef struct {
   smMotionStruct* motionData;
   smAnalogStruct* analogData;
   smSubjectStruct* subjectData;
   int numHeaderEvents;           // number of events specified in the header
   int numEvents;                 // total number of events specified
   smMotionEvent* eventList;
   int numForcePlates;
   smForcePlateSpec* forcePlateList;
   int numOtherData;
   smOtherChannel* otherDataList;
   ExtraHeaderStuff* extraHeaderStuff;
} smC3DStruct;

typedef struct {
   int num3DPoints;
   double pointFrameRate;
   smUnit pointUnits;
   int numAnalogChannels;
   double analogFrameRate;
   smUnit analogUnits;
   int firstFrame;
   int lastFrame;
   int maxInterpolationGap;
   smSubjectStruct subject;
} smC3DHeader;                  /* used for display, not for referencing data in file */


typedef struct {
   smBoolean readAnalogData;
   smBoolean readOtherData;
   smBoolean markerNamesFromDescriptions;
   smBoolean leaveAnalogDataUnscaled;
} smReadC3DOptions;

typedef struct {
   smPoint3 X;
   smPoint3 Y;
   smPoint3 Z;
   smMatrix4x4 dirCos;
   smMatrix4x4 transp;
} smCoordSysStruct;

typedef struct {
   smPoint3 ASISmid, ShoulderMid;
   smPoint3 LHipJC, RHipJC, LKneeJC, RKneeJC, LAnkleJC, RAnkleJC, PelvisCenter;
   smPoint3 RShoulderJC, LShoulderJC, RElbowJC, LElbowJC, RWristJC, LWristJC;
} smJointCenterStruct;

typedef struct {
   smCoordSysStruct PelvisAxis, RThighAxis, LThighAxis, TrunkAxis;
   smCoordSysStruct RShankAxis, LShankAxis, RFootAxis, LFootAxis;
   smCoordSysStruct RUpArmAxis, LUpArmAxis, RLowArmAxis, LLowArmAxis, HeadAxis;
} smCoordSystemStruct;

typedef struct {
   smBoolean lowerBody;
   smBoolean upperBody;
   smBoolean rightHandFull;
   smBoolean leftHandFull;
   smBoolean rightArmOnly;
   smBoolean leftArmOnly;
   smBoolean pelvisPresent;
   smBoolean headPresent;
    smBoolean torsoPresent;
   smBoolean armsUp;
   int rightFingerMarkers[NumFingerMarkers];
   int leftFingerMarkers[NumFingerMarkers];
   int upAxis;
   int index[NumMarkers];
   smPoint3 markerOffsets[NumMarkers];
   double RThighLength;
   double RShankLength;
   double RFootLength;
   double LThighLength;
   double LShankLength;
   double LFootLength;
   double PelvisLength;
   double PelvisDepth;
   double PelvisHeight;
   double TrunkLength;
   double TrunkWidth;
   double RUpperArmLength;
   double RLowerArmLength;
   double LUpperArmLength;
   double LLowerArmLength;
   double HeadLength;
   double RHandLength;
   double LHandLength;
   double RThumbLength;
   double LThumbLength;
   double RIndexFingerLength;
   double LIndexFingerLength;
   double RMiddleFingerLength;
   double LMiddleFingerLength;
   double RRingFingerLength;
   double LRingFingerLength;
   double RPinkyLength;
   double LPinkyLength;
   smJointCenterStruct globalJC;
   smCoordSystemStruct coordSys;
   smTRCStruct* trcInfo;
   int numOtherData;
   smOtherChannel* otherDataList;
   double subjectMass;
} smOrthoTrakModel;

/**************************** Function Prototypes *****************************/

#ifdef __cplusplus
extern "C" {
#endif

/******************************** create model ********************************/
DLL smReturnCode smCreateModel(smModel *model, smModelID *modelID);
DLL smReturnCode smCreateModelFromSIMMJointFile(smModel *model, const char filename[],
                                                smModelID *modelID);
DLL smReturnCode smCreateModelFromMODFile(smModel *model, const char filename[],
                                          smModelID *modelID,
                                          smTRCOptions staticOptions);
DLL smReturnCode smCreateScaledModel(smModel *model, smModelID *modelID, 
                                     const char modelFilename[],
                                     const char staticPoseFilename[],
                                     const char personalDataFilename[],
                                     smTRCOptions staticOptions);
DLL smReturnCode smCreateOrthoTrakModel(const char staticPoseFilename[],
                                        const char personalDataFilename[],
                                        smTRCOptions options,
                                        smOrthoTrakModel *ortho);

/******************************** delete structs ******************************/
DLL smReturnCode smDeleteModel(smModelID modelID);
DLL smReturnCode smDeletesmModel(smModel *model);
DLL void smFreeTRCStruct(smTRCStruct *trc);
DLL void smFreeTRCHeader(smTRCHeaderInfo *header);
DLL void smFreeOrthoTrakModel(smOrthoTrakModel *ortho);
DLL void smFreeC3DStruct(smC3DStruct* c3d, smBoolean freeMotionData, smBoolean freeAnalogData,
                         smBoolean freeSubjectData, smBoolean freeEventData, smBoolean freeForcePlateData,
                         smBoolean freeOtherData);
DLL void smFreeC3DHeader(smC3DHeader* c3dHeader);
DLL void smFreeGenCoordNames(smGCNames *gcNames);

/******************************** solve frames ********************************/
DLL smReturnCode smSolveFrame(smModelID modelID, smTRCFrame *trcData, int nq,
                              double q[], smTRCFrame *trcPredicted, smSolveInfoStruct *info);
DLL smReturnCode smSolveFrameHTR(smModelID modelID, smTRCFrame *trcData,
                                 int nq, double q[], smTRCFrame *trcPredicted,
                                 smHTRFrame *htr, smSolveInfoStruct *info);
DLL smReturnCode smSolveFrameMatrix(smModelID modelID, smTRCFrame *trcData,
                                    int nq, double q[], smTRCFrame *trcPredicted,
                                    smMatrixFrame *mat, smSolveInfoStruct *info);
#if 0
DLL smReturnCode smOptimizeFrame(smModelID modelID, smTRCFrame *trcData,
                          int nq, double q[], smTRCFrame *trcPredicted,
                          smSolveInfoStruct *info);
#endif
/******************************** read functions ******************************/
DLL smReturnCode smReadTrackedFile(const char filename[], smTRCStruct *trc);
DLL smReturnCode smCropMotionEnds(smC3DStruct *c3d, int *firstFrame, int *lastFrame);
DLL smReturnCode smReadTrackedFileHeader(const char filename[], smTRCHeaderInfo *header);
DLL smReturnCode smReadC3DFile(const char filename[], smC3DStruct* c3d, smReadC3DOptions* options);
DLL smReturnCode smScanC3DHeader(const char filename[], smC3DHeader* head);

/******************************** write functions *****************************/
DLL smReturnCode smWriteMODFile(smModelID modelID, const char filename[]);
DLL smReturnCode smWriteJointFile(smModelID modelID, const char filename[]);   
DLL smReturnCode smWriteHTRFile(const char filename[], smHTRData* data);
DLL smReturnCode smSetupMotionFile(smModelID modelID, const char filename[],
                                   int first, int last, const char trcfilename[]);
DLL smReturnCode smSetupMotionWithTRCFile(smModelID modelID, const char filename[],
                                          int first, int last, int numMarkers,
                                          const char trcfilename[],
                                          smTRCHeaderInfo *trcHeader);
DLL smReturnCode smWriteFrame(smModelID modelID);
DLL smReturnCode smWriteFrameWithTRC(smModelID modelID, smTRCFrame *trc,
                                     smTRCFrame *predicted, smSolveInfoStruct *info);
DLL smReturnCode smCloseMotionFile(smModelID modelID, const char filename[]);
DLL smReturnCode smCloseMotionWithTRCFile(smModelID modelID, const char filename[]);

/**************************** utility functions ***************************/
DLL void smSetWorkingDirectory(const char path[]);
DLL int smGetVersionNumber(void);
DLL const char *smGetVersionString(void);
DLL const char *smGetWorkingDirectory(void);
DLL const char *smGetErrorMessage(smReturnCode code);
DLL const char *smGetMessage(smModelID modelID);
DLL const char *smGetErrorBuffer(smModelID modelID);
DLL void smClearMessage(smModelID modelID);
DLL void smClearErrorBuffer(smModelID modelID);
DLL void smUseGlobalMessages(smBoolean flag);
DLL const char *smGetGlobalMessage(void);
DLL const char *smGetGlobalErrorBuffer(void);
DLL void smSetStandAlone(void);
DLL double smGetConversion(smUnit from, smUnit to);
DLL void smSetCriticalMarkerNames(int numNames, const char *names[]);

/**************************** set model parameters ****************************/
DLL smReturnCode smResetModel(smModelID modelID);
DLL smReturnCode smSetPoseGenCoords(smModelID modelID, int numGenCoords, double gencoords[]);
DLL smReturnCode smSetPoseTransforms(smModelID modelID, int numSegments, smTransform segmentTransforms[]);
DLL smReturnCode smSetMarkerOrder(smModelID modelID, int numMarkers, char *markerNameList[]);
DLL smReturnCode smSetOptions(smModelID modelID, smOptions *options);
DLL smReturnCode smSetSolverMethod(smModelID modelID, smSolverMethod method);
DLL smReturnCode smSetMarkerWeight(smModelID modelID, const char segName[], 
                                   const char markerName[], double weight);
DLL smReturnCode smSetGravityDirection(smModelID modelID, smModel *smmodel, smAxes gravity);
DLL smReturnCode smSetAllSegmentSolveStates(smModelID modelID, int numSegments, smBoolean segmentStates[]);
DLL smReturnCode smSetSegmentSolveState(smModelID modelID, char *segmentName, smBoolean state);
DLL smReturnCode smSetSegmentSolveStateIndex(smModelID modelID, int index, smBoolean state);
DLL smReturnCode smSetAllGencoordSolveStates(smModelID modelID, int numGCs, smBoolean gcStates[]);
DLL smReturnCode smSetGencoordSolveState(smModelID modelID, char *gcName, smBoolean state);
DLL smReturnCode smSetGencoordSolveStateIndex(smModelID modelID, int index, smBoolean state);

/**************************** get model parameters ****************************/
DLL int smGetNumGenCoords(smModelID modelID);
DLL int smGetNumMarkers(smModelID modelID);
DLL int smGetNumMarkerLinks(smModelID modelID);
DLL smReturnCode smGetGenCoordMapping(smModelID modelID, smGenCoordMapping gcMap[]);
DLL smReturnCode smGetSegmentMapping(smModelID modelID, smSegmentMapping segMap[]);
DLL smReturnCode smGetOptions(smModelID, smOptions *options);
DLL smReturnCode smGetTRC(smModelID modelID, smTRCFrame *trc);
DLL smReturnCode smGetGlobalMarkerPositions(smModelID modelID, char **segmentNames, char **markerNames, smTRCFrame *trc);
DLL smReturnCode smGetGenCoordNames(smModelID modelID, smGCNames *gcNames);
DLL smReturnCode smGetHTRBasePosition(smModelID modelID, smHTRFrame *htr);
DLL smReturnCode smGetHTRPose(smModelID modelID, int nq, double q[], smHTRFrame *htr);
DLL smReturnCode smGetMatrixBasePosition(smModelID modelID, smMatrixFrame *matrix);
DLL smReturnCode smGetMatrixPose(smModelID modelID, int nq, double q[],
                                 smMatrixFrame *matrix);
DLL smReturnCode smRemoveExtraMarkers(smModel *model, int numMarkersUsed, char *markerNamesUsed[]);
DLL smReturnCode smConvertPointFromGround(smModelID modelID, int smIndex,
                                          smPoint3 *pt1, smPoint3 *pt2);
DLL smReturnCode smConvertPoint(smModelID modelID, const char fromName[], const char toName[],
                                smPoint3 *pt);
DLL smReturnCode smConvertVec(smModelID modelID, char fromName[], char toName[],
                                smPoint3 *vec);
DLL smReturnCode smGetGroundPlane(smModelID modelID, smPlaneStruct *plane);
DLL smReturnCode smSetGroundPlane(smModelID modelID, smPlaneStruct *plane);
DLL smReturnCode smGetPoseGenCoords(smModelID modelID, int numGenCoords, double gencoords[]);
DLL smReturnCode smGetPoseTransforms(smModelID modelID, int numSegments, smTransform segmentTransforms[]);
DLL int smGetNumSegments(smModelID modelID);
DLL smReturnCode smGetAllSegmentSolveStates(smModelID modelID, int numSegments, smBoolean segmentStates[]);
DLL smReturnCode smGetSegmentSolveState(smModelID modelID, char * segmentName, smBoolean * segmentState);
DLL smReturnCode smGetSegmentSolveStateIndex(smModelID modelID, int index, smBoolean * segmentState);
DLL smReturnCode smGetAllGencoordSolveStates(smModelID modelID, int numGCs, smBoolean gcStates[]);
DLL smReturnCode smGetGencoordSolveState(smModelID modelID, char * gcName, smBoolean * gcState);
DLL smReturnCode smGetGencoordSolveStateIndex(smModelID modelID, int index, smBoolean * gcState);

#if SMDEBUG
/*** debug ***/
DLL void displayModFileInfo(smModel model);
DLL smReturnCode writeModFileInfo(smModel model, char filename[], char modFilename[]);
DLL smReturnCode printsmModel(smModel *mod, char filename[]);
DLL smReturnCode smWriteFrameWithQs(smModelID modelID, int nq, double q[], smTRCFrame *trc,
                                    smTRCFrame *predicted, smSolveInfoStruct *info);
#endif

#ifdef __cplusplus
}
#endif

#endif
