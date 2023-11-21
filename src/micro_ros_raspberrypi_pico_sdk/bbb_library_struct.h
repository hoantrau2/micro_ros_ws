#ifndef BBB_LIBRARY_STRUCT_H_
#define BBB_LIBRARY_STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct bbb_Float64
{
  double data;
} bbb_Float64;

// Struct for a sequence of std_msgs__msg__Float64.
typedef struct bbb_Float64__Sequence
{
  bbb_Float64 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bbb_Float64__Sequence;

#ifdef __cplusplus
}
#endif
#endif  //BBB_LIBRARY_STRUCT_H_