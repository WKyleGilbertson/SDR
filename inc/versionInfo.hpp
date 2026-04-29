#pragma once
#include <cstdio>
#include <cstdint>
#include <cstring>

typedef struct
{
   unsigned short int Major, Minor, Patch;
   unsigned int  GitTag;
   char GitCI[41], BuildDate[17], Name[50];
} SWV;

class versionInfo {
   private:
      uint8_t Major, Minor, Patch;
      uint32_t GitTag;
      char GitCI[41], BuildDate[17], Name[50];
   public:
      versionInfo();
      SWV getVersion();
      void printVersion();
};