#include "versionInfo.hpp"

versionInfo::versionInfo()
{
  Major = MAJOR_VERSION;
  Minor = MINOR_VERSION;
  Patch = PATCH_VERSION;
  sscanf(CURRENT_HASH, "%x", &GitTag);
  strncpy(GitCI, CURRENT_HASH, 40);
  GitCI[40] = '\0'; // Ensure null-termination
  strncpy(BuildDate, CURRENT_DATE, sizeof(BuildDate) - 1);
  BuildDate[sizeof(BuildDate) - 1] = '\0'; // Ensure null-termination
  strncpy(Name, APP_NAME, sizeof(Name) - 1);
  Name[sizeof(Name) - 1] = '\0'; // Ensure null-termination
}

SWV versionInfo::getVersion() {
  SWV v;
  v.Major = Major;
  v.Minor = Minor;
  v.Patch = Patch;
  v.GitTag = GitTag;
  strncpy(v.GitCI, GitCI, 40);
  v.GitCI[40] = '\0';
  strncpy(v.BuildDate, BuildDate, 16);
  v.BuildDate[16] = '\0';
  strncpy(v.Name, Name, 49);
  v.Name[49] = '\0';
  return v;
}

void versionInfo::printVersion() {
fprintf(stdout, "%s GitCI:%s %s v%.1d.%.1d.%.1d\n",
        Name, GitCI, BuildDate, Major, Minor, Patch);
}