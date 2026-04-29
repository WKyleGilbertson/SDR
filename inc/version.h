#ifndef VERSION_H
#define VERSION_H

#define MAJOR_VERSION 0
#define MINOR_VERSION 1
#define PATCH_VERSION 0

typedef struct
{
   unsigned short int Major, Minor, Patch;
   unsigned int   GitTag;
   char GitCI[41], BuildDate[20], Name[50];
} SWV;

#endif