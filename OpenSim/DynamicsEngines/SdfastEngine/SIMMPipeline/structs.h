/*******************************************************************************

   STRUCTS.H

   Authors: Peter Loan
            Krystyne Blaikie

   Copyright (c) 1996-2004 MusculoGraphics, a division of Motion Analysis Corp.
   All rights reserved.

   Description: This file contains definitions of all structures that are
      used by the Dynamics Pipeline.

*******************************************************************************/

#ifndef STRUCTS_H
#define STRUCTS_H


STRUCT {
   double xyz[3];                    /* coordinates of a point in 3space */
} Coord3D;                           /* 3D point for passing to v3d() routine */


STRUCT {
   int segment;                      /* body segment to which force is applied */
   dpSplineFunction appl_point[3];   /* pnt of application of force (local coords) */
   dpSplineFunction force_vec[3];    /* force vector */
} ForceStruct;                       /* contains one external force */


STRUCT {
   int segment;                      /* body segment to which torque is applied */
   dpSplineFunction torque_vec[3];   /* torque vector (in coords of body segment) */
} TorqueStruct;                      /* contains one external torque */


STRUCT {
   char* name;                         /* name of this motion */
   FILE* fp;                           /* file that these data read from or written to */
   int num_frames;                     /* number of [time] frames of data */
   int num_otherdata;                  /* number of "other" data values per frame */
   int num_elements;                   /* number of data values per frame */
   int num_musc_excitations;           /* number of muscle excitations */
   int num_forces;                     /* number of external forces */
   int num_torques;                    /* number of external torques */
   int current_frame;                  /* current data frame in simulation */
   double range_start;                 /* start of X range */
   double range_end;                   /* end of X range */
   double step_size;                   /* step size between adjacent frames of data */
   int time_index;                     /* column index of time data, if any */
   char** elementnames;                /* names of the data elements */
   double** motiondata;                /* the actual motion data */
   dpSplineFunction** q_data;          /* pointers to the q (gencoord) data */
   dpSplineFunction** u_data;          /* pointers to the u (gencoord vels) data */
   dpSplineFunction** udot_data;       /* pointers to the udot (gencoord accels) */
   dpSplineFunction** q_torques;       /* pointers to the torques applied to the qs */
   dpSplineFunction** musc_excitations;/* pointers to the muscle excitations */
   ForceStruct** forces;               /* external forces (e.g. ground-reaction) */
   TorqueStruct** torques;             /* external torques (e.g. ground-reaction) */
   int num_events;                     /* number of events in array */
   dpMotionEvent* event;               /* array of motion events (e.g., heelstrike) */
   float event_color[3];               /* color used to display events in SIMM plots */
   int enforce_constraints;            /* enforce constraints while prescribing motion? */
} MotionData;                          /* holds motion data for inverse dynamics */


STRUCT {
   double mass;
   double gravity[3];
   double mass_center[3];
   double inertia[3][3];
   double kinetic_energy;
   double potential_energy;
   double linear_momentum[3];
   double angular_momentum[3];
   double system_energy;
   double initial_system_energy;
} SystemInfo;


#endif /* STRUCTS_H */
