#pragma once
#include <stdint.h>

// UTC calendar <-> day-number conversion - pure, header-only, shared by
// gnss.cpp and the MiniSEED record builder (and unit-tested on the host).

// Howard Hinnant's days_from_civil algorithm (public domain) - avoids any
// mktime()/timezone dependency for a straightforward UTC-to-epoch
// conversion.
inline int64_t daysFromCivilUTC(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  int64_t era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int64_t)doe - 719468;
}

// Howard Hinnant's civil_from_days algorithm (public domain) - inverse of
// daysFromCivilUTC() above.
inline void civilFromDaysUTC(int64_t z, int *y, unsigned *m, unsigned *d) {
  z += 719468;
  int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  unsigned doe = (unsigned)(z - era * 146097);                         // [0, 146096]
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365; // [0, 399]
  int64_t yy = (int64_t)yoe + era * 400;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
  unsigned mp = (5 * doy + 2) / 153;                      // [0, 11]
  unsigned dd = doy - (153 * mp + 2) / 5 + 1;             // [1, 31]
  unsigned mm = mp < 10 ? mp + 3 : mp - 9;                // [1, 12]
  *y = (int)(yy + (mm <= 2 ? 1 : 0));
  *m = mm;
  *d = dd;
}
