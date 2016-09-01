#include "util/Time.h"
#include <stddef.h>
#include <sys/time.h>

/* Returns current time in milliseconds. */
unsigned long long getTime() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
