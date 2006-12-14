/*******************************************************************************

   OBJECTS.H

   Authors: Peter Loan
            Krystyne Blaikie

   Copyright (c) 2000-2001 MusculoGraphics, a division of Motion Analysis Corp.
   All rights reserved.

*******************************************************************************/

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

STRUCT
{
   char* name1;
   int element1;
   PairElementType type1;
   char* name2;
   int element2;
   PairElementType type2;
/*   double coef_rest;
   double mu_dynamic;
   double mu_static;*/
} ContactPair;

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


