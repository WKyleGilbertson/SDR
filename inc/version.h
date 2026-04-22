typedef struct
{
   unsigned short int Major, Minor, Patch;
   unsigned int  GitTag;
   char GitCI[41], BuildDate[17], Name[50];
} SWV;
