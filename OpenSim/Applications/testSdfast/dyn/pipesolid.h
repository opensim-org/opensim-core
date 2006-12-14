/*******************************************************************************

   PIPESOLID.H

   Authors: Peter Loan
            Krystyne Blaikie

   Copyright (c) 2000-2001 MusculoGraphics, a division of Motion Analysis Corp.
   All rights reserved.

   Description: This file contains structures and function prototypes that are
      used by the SOLID contact detection code.

*******************************************************************************/

#ifndef _PIPESOLID_H_
#define _PIPESOLID_H_

typedef enum
{
   ONE_POINT,
   ALL_POINTS
} ResponseType;

typedef enum
{
   VERTEX,
   EDGE,
   FACE
} ContactType;

typedef struct
{
   double p1[3];
   double normal1[3];
   ContactType type1;
   double p2[3];
   double normal2[3];
   ContactType type2;
   double contact_normal[3];
   int contact_body;
} ContactReport;

ENUM
{
   PAIR_OBJECT,
   PAIR_GROUP
} PairElementType;

STRUCT
{
   char* name;
   int num_objects;
   int* object_num;
   dpBoolean defined;
} ObjectGroup;

struct cp
{
   char* name1;
   int element1;
   PairElementType type1;
   char* name2;
   int element2;
   PairElementType type2;
   double restitution;
   double static_friction;
   double dynamic_friction;
   void (*callback)(dpPolyhedronStruct*, dpPolyhedronStruct*,
		    struct cp*, int, ContactReport*);
};
typedef struct cp ContactPair;

STRUCT
{
   char* name;
   dpBoolean defined;
   dpPolyhedronStruct* ph;
} ContactObject;

STRUCT
{
   int num_objects;
   ContactObject* object;
   int num_groups;
   ObjectGroup* group;
   int num_pairs;
   ContactPair* pair;
} ObjectInfo;

#ifdef __cplusplus
extern "C" {
#endif

void solid_pre_init(void);
void solid_post_init(void);
void make_solid_object(dpPolyhedronStruct*);
void make_contact_response(dpPolyhedronStruct* ph1, dpPolyhedronStruct* ph2,
			     ContactPair* cp, ResponseType type);
void transform_solid_object(dpPolyhedronStruct* ph, double vec[], double quat[]);
void calc_solid_contacts(void);

#ifdef __cplusplus
}
#endif

#endif
