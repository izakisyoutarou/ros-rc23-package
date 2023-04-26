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

#ifndef CONTROLLER_INTERFACE_ER__VISIBILITY_H_
#define CONTROLLER_INTERFACE_ER__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define CONTROLLER_INTERFACE_ER_EXPORT __attribute__ ((dllexport))
    #define CONTROLLER_INTERFACE_ER_IMPORT __attribute__ ((dllimport))
  #else
    #define CONTROLLER_INTERFACE_ER_EXPORT __declspec(dllexport)
    #define CONTROLLER_INTERFACE_ER_IMPORT __declspec(dllimport)
  #endif

  #ifdef CONTROLLER_INTERFACE_ER_DLL
    #define CONTROLLER_INTERFACE_ER_PUBLIC CONTROLLER_INTERFACE_ER_EXPORT
  #else
    #define CONTROLLER_INTERFACE_ER_PUBLIC CONTROLLER_INTERFACE_ER_IMPORT
  #endif

  #define CONTROLLER_INTERFACE_ER_PUBLIC_TYPE CONTROLLER_INTERFACE_ER_PUBLIC

  #define CONTROLLER_INTERFACE_ER_LOCAL

#else

  #define CONTROLLER_INTERFACE_ER_EXPORT __attribute__ ((visibility("default")))
  #define CONTROLLER_INTERFACE_ER_IMPORT

  #if __GNUC__ >= 4
    #define CONTROLLER_INTERFACE_ER_PUBLIC __attribute__ ((visibility("default")))
    #define CONTROLLER_INTERFACE_ER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONTROLLER_INTERFACE_ER_PUBLIC
    #define CONTROLLER_INTERFACE_ER_LOCAL
  #endif

  #define CONTROLLER_INTERFACE_ER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER_INTERFACE_ER__VISIBILITY_H_
