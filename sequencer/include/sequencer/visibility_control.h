#ifndef SEQUENCER__VISIBILITY_CONTROL_H_
#define SEQUENCER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SEQUENCER_EXPORT __attribute__ ((dllexport))
    #define SEQUENCER_IMPORT __attribute__ ((dllimport))
  #else
    #define SEQUENCER_EXPORT __declspec(dllexport)
    #define SEQUENCER_IMPORT __declspec(dllimport)
  #endif
  #ifdef SEQUENCER_BUILDING_LIBRARY
    #define SEQUENCER_PUBLIC SEQUENCER_EXPORT
  #else
    #define SEQUENCER_PUBLIC SEQUENCER_IMPORT
  #endif
  #define SEQUENCER_PUBLIC_TYPE SEQUENCER_PUBLIC
  #define SEQUENCER_LOCAL
#else
  #define SEQUENCER_EXPORT __attribute__ ((visibility("default")))
  #define SEQUENCER_IMPORT
  #if __GNUC__ >= 4
    #define SEQUENCER_PUBLIC __attribute__ ((visibility("default")))
    #define SEQUENCER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SEQUENCER_PUBLIC
    #define SEQUENCER_LOCAL
  #endif
  #define SEQUENCER_PUBLIC_TYPE
#endif

#endif  // SEQUENCER__VISIBILITY_CONTROL_H_
