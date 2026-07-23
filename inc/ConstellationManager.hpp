#pragma once
#include <unordered_map>
#include <mutex>
#include <cstdio>
#include "Ephemeris.hpp"

class ConstellationManager
{
public:
  static ConstellationManager &getInstance()
  {
    static ConstellationManager instance;
    return instance;
  }

  void commitEphemeris(int prn, const Ephemeris &ephem)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    _ephemerisDB[prn] = ephem;
  }

  bool hasValidEphemeris(int prn)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    return _ephemerisDB.count(prn) && _ephemerisDB[prn].isValid;
  }

  Ephemeris getEphemeris(int prn)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    return _ephemerisDB[prn]; // Will return empty struct if not found
  }

  void ConstellationManager::printEphemerisSanityCheck(int prn)
  {
    std::lock_guard<std::mutex> lock(_mtx);
    if (_ephemerisDB.find(prn) == _ephemerisDB.end() || !_ephemerisDB[prn].isValid)
    {
      printf("\n[DB] PRN %d: No valid ephemeris available to print.\n", prn);
      return;
    }

    Ephemeris e = _ephemerisDB[prn];

    printf("\n====================================================\n");
    printf("         EPHEMERIS SANITY CHECK: PRN %02d\n", e.prn);
    printf("====================================================\n");
    printf(" [CLOCK]\n");
    printf("   Week Number : %u\n", e.weekNumber);
    printf("   TOC (Time)  : %u sec\n", e.toc);
    printf("   af0 (Bias)  : %e sec\n", e.af0);
    printf("   af1 (Drift) : %e sec/sec\n", e.af1);

    printf("\n [ORBIT]\n");
    printf("   Eccentricity (ecc) : %f  <-- Should be ~0.01\n", e.ecc);
    printf("   Sqrt(A)            : %f  <-- Should be ~5153\n", e.sqrta);
    printf("   Inclination (i0)   : %f rad <-- Should be ~0.95\n", e.i0);
    printf("   Long. Node (Omega0): %f rad\n", e.omega0);
    printf("   Arg. Perigee (omega): %e rad\n", e.omega);
    printf("   Mean Anomaly (M0)  : %e rad\n", e.m0);
    printf("====================================================\n");
  }

private:
  ConstellationManager() = default;
  std::unordered_map<int, Ephemeris> _ephemerisDB;
  std::mutex _mtx;
};