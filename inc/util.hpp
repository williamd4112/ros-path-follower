#ifndef _UTIL_HPP_
#define _UTIL_HPP_

#include <unistd.h>

#define sign_int(x) (((x) == 0) ? 0 : ((x) < 0) ? -1 : 1)
#define sign_float(x) (((x) == 0) ? 0 : ((x) < 0.0f) ? -1.0f : 1.0f)
#define bool2str(b) ((b) ? "True" : "False")
#define normalize(offset, range) ((float)(offset) / (float)(range))


#endif
