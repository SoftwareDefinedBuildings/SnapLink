#include <stddef.h>
#include <sys/time.h>
#include "util/Time.h"

/* Returns current time in milliseconds. */
long getTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
