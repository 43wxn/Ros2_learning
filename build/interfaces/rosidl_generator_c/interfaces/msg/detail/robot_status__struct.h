// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/RobotStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/robot_status.h"


#ifndef INTERFACES__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
#define INTERFACES__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'STATUS_MOVEING'.
enum
{
  interfaces__msg__RobotStatus__STATUS_MOVEING = 1ul
};

/// Constant 'STATUS_STOP'.
enum
{
  interfaces__msg__RobotStatus__STATUS_STOP = 2ul
};

/// Struct defined in msg/RobotStatus in the package interfaces.
typedef struct interfaces__msg__RobotStatus
{
  uint32_t status;
  float pose;
} interfaces__msg__RobotStatus;

// Struct for a sequence of interfaces__msg__RobotStatus.
typedef struct interfaces__msg__RobotStatus__Sequence
{
  interfaces__msg__RobotStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__RobotStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
