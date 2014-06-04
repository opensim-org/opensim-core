/*******************************************************************************

   MOCAP.H - public api for the mocap to SIMM tranlator.  This translator
      currently handles Motion Analysis HTR & HTR2 files as well as realtime
      motion received via. network connection to EVa Realtime.  In addition
      this translator can import analog data that may accompany an HTR or HTR2
      file in the form of ANB, ANC, or OrthoTrak XLS file(s).

   Author: Kenny Smith

   Date: 25-JUN-99

   Copyright (c) 1999 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/
#ifndef MOCAP_H
#define MOCAP_H

/* ---------------------------------------------------------------------------
   PUBLIC MOCAP IMPORT API:
------------------------------------------------------------------------------ */
#if ! OPENSIM_BUILD
public ReturnCode mocap_to_simm(glutMocapOptions*, int* modelIndex);
#endif

public ReturnCode save_pose_file(ModelStruct*, const char* fileName);

/* ---------------------------------------------------------------------------
   PUBLIC REALTIME MOCAP IMPORT API:
------------------------------------------------------------------------------ */
public void start_stop_realtime_mocap_stream(ModelStruct*);
public void stop_realtime_mocap_stream();
public void realtime_exit();
public void refresh_realtime_display();

public const char* get_realtime_hz();

/* ===========================================================================
   PRIVATE API - the following declarations are included in this header simply
      so that the mocap importer implementation can be divided into seperate
      modules (currently mocap.c and analoc.c).  Client code elsewhere in SIMM
      should not make use of any of the following code.
------------------------------------------------------------------------------ */

#define REALTIME_MOTION_NAME "realtime"

enum {                       /* htr & htr2 column identifiers */
   _TX, _TY, _TZ,
   _RX, _RY, _RZ,
   _BONE_SCALE,
   HTR_FRAME_NUM_COLS,
   
   _HTR2_BONE_LEN = _BONE_SCALE
};

STRUCT {                     /* ---- mocap model segment record: */
   char    name[128];              /* mocap segment name */
   char    parentName[128];        /* mocap parent segment name */
   int     parentIndex;            /* mocap parent segment index */
   dpCoord3D offset;                 /* base position: starting translation */
   dpCoord3D rotation;               /* base position: starting rotation */
   DMatrix base_transform;         /* base position: as a transform matrix */
   double  boneLength;             /* mocap segment length */
   double  htr2TranslationParam;   /* htr2 translation parameter */
} MocapSeg;

enum { XYZ, ZYX };           /* eulerRotationOrder */

enum { MM };                 /* calibrationUnits */

enum { DEGREES, RADIANS };   /* rotationUnits */

STRUCT {                     /* ---- mocap file info record: */
   int       fileVersion;         /* mocap file version number */
   int       numSegments;         /* number of body segments */
   int       numFrames;           /* number of mocap frames to follow */
   int       dataFrameRate;       /* mocap frames per second */
   int       eulerRotationOrder;  /* XYZ, or ZYX */
   int       calibrationUnits;    /* MM */
   int       rotationUnits;       /* DEGREES or RADIANS */
   int       globalAxisOfGravity; /* up direction (XX, YY, or ZZ) */
   int       boneLengthAxis;      /* local long axis for mocap segments */
   int       analogSyncFrameIndex;/* index to sync motion & analog data */
   double    scaleFactor;         /* ?? */
   MocapSeg* segments;            /* array of mocap body segments */
   int       rootSegment;         /* index of root mocap segment */
   long      byteOffsetToFrames;  /* offset to beginning of motion frame data */
   ModelStruct* model;            /* SIMM model that is created from this mocap data */
} MocapInfo;


#if OLD_ANALOG

enum { FX, FY, FZ, MX, MY, MZ, NUM_AMTI_BERTEC_CHANNELS };

enum { FX12, FX34, FY14, FY23, FZ1, FZ2, FZ3, FZ4, NUM_KISTLER_CHANNELS };

enum {                       /* ---- analog channel type */
   UNSPECIFIED_ANALOG_CHANNEL,
   ELAPSED_TIME,
   FORCE_PLATE_FORCE,
   FORCE_PLATE_MOMENT,
   FORCE_LOCATION,
   EMG,
   OTHER_DATA
};
   
STRUCT {                     /* ---- forceplate channel info: */
   int index;                     /* forceplate index (zero based) */
   int component;                 /* forceplate channel componant (see enums above) */
   double baseline;
} FPChannel;

STRUCT {                     /* ---- emg channel info: */
   int    numMuscles;
   char** muscleNames;            /* array of SIMM muscle names */
   int*   muscleIndices;          /* array of SIMM muscle indices */
   double maxVoluntaryContraction;/* user-specified value for scaling this emg channel */
   double maxSample;              /* observed maximum value for this channel */
} EMGMuscles;

STRUCT {                     /* ---- other data channel info: */
   char*  name;                   /* name to use for SIMM motion column title */
} OtherData;

STRUCT {                     /* ---- import variable info: */
   char*  name;                   /* imported variable name */
   int    type;                   /* force plate, emg, or other data */
   int    frequency;              /* sample rate, Hz */
   int    range;                  /* channel range (ie. maximum?) */
   int    numMotionColumns;       /* number of motion data columns channel maps to */
   int*   motionColumn;           /* columns of motion data that this channel maps to */
   union {
      FPChannel  forcePlate;
      EMGMuscles emg;
      OtherData  otherData;
   }     u;
} AnalogChannel;

typedef struct {             /* ---- gait events (read from OrthoTrak XLS file) */
   char   name[32];               /* event name */
   double time;                   /* event frame number */
} GaitEvent;

STRUCT {                     /* ---- analog data record */
   int            numChannels;
   int            numRows;
   AnalogChannel* channels;
   double*        data;
   int            numGaitEvents;
   GaitEvent*     gaitEvents;
} AnalogData;

STRUCT {                          /* ---- forceplate record: */
   SBoolean inited;                    /* has this record been initialized? */
   int      manufacturer;              /* manufacturer id (see enum above) */
   double   scaleFactor;               /* overall channel scale factor? */
   double   length;                    /* forceplate length (optional) */
   double   width;                     /* forceplate width (optional) */
   double   calibrationMatrix[64];     /* forceplate calibration matrix */
   DMatrix  localToWorldXform;         /* forceplate placement matrix */
   
   struct {                            /* -- Kistler forceplate parameters: */
     double   a, b, az0;                 /* transducer placement: a, b, az0 */
     double   p[6];                      /* COP correction coefficients (optional) */
     SBoolean hasCOPCorrectionValues;    /* are COP correction coefficients inited? */
   }        kistler;
} ForcePlateSpec;

#endif

#if ! OPENSIM_BUILD
smAnalogStruct* _init_channel_mapping(SBoolean includeMuscles, const char analogFile[]);
smC3DStruct* read_analog_data_files(int numFiles, char* const* files, const MocapInfo*, const glutMocapOptions*);
SBoolean _map_channel(const char* name, smAnalogChannel* channel, const smAnalogStruct* m);
void _rectify_emg_samples(smAnalogStruct* ad);
void _scale_emg_samples(smAnalogStruct* ad);
int _init_forceplates(smForcePlateSpec fp[], int n, const char analogFile[], ModelStruct* ms);
void _nuke_forceplate_samples(smAnalogStruct* ad);
void _autocalibrate_forceplate_channel(smAnalogStruct* ad, int col);
void free_analog_data(smAnalogStruct*);
void post_process_c3d_analog_data(smC3DStruct* c3d, const MocapInfo* mi, const glutMocapOptions* mo);
void read_critical_marker_names(char* names[], const char dataFile[]);
#endif

#endif /* MOCAP_H */
