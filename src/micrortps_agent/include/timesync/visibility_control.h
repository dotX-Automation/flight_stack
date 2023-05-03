#ifndef TIMESYNC__VISIBILITY_CONTROL_H_
#define TIMESYNC__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TIMESYNC_EXPORT __attribute__ ((dllexport))
    #define TIMESYNC_IMPORT __attribute__ ((dllimport))
  #else
    #define TIMESYNC_EXPORT __declspec(dllexport)
    #define TIMESYNC_IMPORT __declspec(dllimport)
  #endif
  #ifdef TIMESYNC_BUILDING_LIBRARY
    #define TIMESYNC_PUBLIC POSE_EXPORT
  #else
    #define TIMESYNC_PUBLIC POSE_IMPORT
  #endif
  #define TIMESYNC_PUBLIC_TYPE POSE_PUBLIC
  #define TIMESYNC_LOCAL
#else
  #define TIMESYNC_EXPORT __attribute__ ((visibility("default")))
  #define TIMESYNC_IMPORT
  #if __GNUC__ >= 4
    #define TIMESYNC_PUBLIC __attribute__ ((visibility("default")))
    #define TIMESYNC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TIMESYNC_PUBLIC
    #define TIMESYNC_LOCAL
  #endif
  #define TIMESYNC_PUBLIC_TYPE
#endif

#endif  // TIMESYNC__VISIBILITY_CONTROL_H_
