// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INJECTION_PARAM_CALCULATOR__VISIBILITY_H_
#define INJECTION_PARAM_CALCULATOR__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define INJECTION_PARAM_CALCULATOR_EXPORT __attribute__ ((dllexport))
    #define INJECTION_PARAM_CALCULATOR_IMPORT __attribute__ ((dllimport))
  #else
    #define INJECTION_PARAM_CALCULATOR_EXPORT __declspec(dllexport)
    #define INJECTION_PARAM_CALCULATOR_IMPORT __declspec(dllimport)
  #endif

  #ifdef INJECTION_PARAM_CALCULATOR_DLL
    #define INJECTION_PARAM_CALCULATOR_PUBLIC INJECTION_PARAM_CALCULATOR_EXPORT
  #else
    #define INJECTION_PARAM_CALCULATOR_PUBLIC INJECTION_PARAM_CALCULATOR_IMPORT
  #endif

  #define INJECTION_PARAM_CALCULATOR_PUBLIC_TYPE INJECTION_PARAM_CALCULATOR_PUBLIC

  #define INJECTION_PARAM_CALCULATOR_LOCAL

#else

  #define INJECTION_PARAM_CALCULATOR_EXPORT __attribute__ ((visibility("default")))
  #define INJECTION_PARAM_CALCULATOR_IMPORT

  #if __GNUC__ >= 4
    #define INJECTION_PARAM_CALCULATOR_PUBLIC __attribute__ ((visibility("default")))
    #define INJECTION_PARAM_CALCULATOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INJECTION_PARAM_CALCULATOR_PUBLIC
    #define INJECTION_PARAM_CALCULATOR_LOCAL
  #endif

  #define INJECTION_PARAM_CALCULATOR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // INJECTION_PARAM_CALCULATOR__VISIBILITY_H_
