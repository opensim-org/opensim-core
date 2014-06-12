/*******************************************************************************

   ORTHOTRAK.H

   Author: Krystyne Blaikie
           Peter Loan

   Date: 14-Aug-02

   Copyright (c) 2002-4 MusculoGraphics, Inc.
   All rights reserved.

*******************************************************************************/

STRUCT
{
   int numNames;
   char **markerNames;
} MarkerNameStruct;

STRUCT 
{
   double inferior;
   double lateral;
   double posterior;
} HJCOffsetStruct;

STRUCT
{
   HJCOffsetStruct ROffset;
   HJCOffsetStruct LOffset;
   double RFootLength;
   double LFootLength;
   double RKneeDiameter;
   double LKneeDiameter;
   double RAnkleDiameter;
   double LAnkleDiameter;
   double subjectMass;
} PersonalDataStruct;


void subPoints(smPoint3 a, smPoint3 b, smPoint3 c);
ReturnCode checkMarkerMovement(double movement, smUnit units, double threshold);
void initializeOrthoTrakStruct(smOrthoTrakModel *data);
ReturnCode createsmModel(ModelStruct *simmModel, smModel *smMod);
void deletesmModel(smModel *model);
