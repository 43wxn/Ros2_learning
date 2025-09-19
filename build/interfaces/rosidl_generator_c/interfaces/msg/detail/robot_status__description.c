// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from interfaces:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#include "interfaces/msg/detail/robot_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_interfaces
const rosidl_type_hash_t *
interfaces__msg__RobotStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc2, 0xd3, 0xfa, 0x07, 0xc9, 0x2b, 0x51, 0x3e,
      0x02, 0x66, 0x56, 0xf8, 0xa8, 0xd3, 0x6f, 0xe1,
      0xa9, 0xae, 0x41, 0x4a, 0xb2, 0x04, 0xba, 0x56,
      0xfd, 0x96, 0xda, 0x9e, 0x85, 0x33, 0xbb, 0x01,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char interfaces__msg__RobotStatus__TYPE_NAME[] = "interfaces/msg/RobotStatus";

// Define type names, field names, and default values
static char interfaces__msg__RobotStatus__FIELD_NAME__status[] = "status";
static char interfaces__msg__RobotStatus__FIELD_NAME__pose[] = "pose";

static rosidl_runtime_c__type_description__Field interfaces__msg__RobotStatus__FIELDS[] = {
  {
    {interfaces__msg__RobotStatus__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__RobotStatus__FIELD_NAME__pose, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interfaces__msg__RobotStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interfaces__msg__RobotStatus__TYPE_NAME, 26, 26},
      {interfaces__msg__RobotStatus__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 STATUS_MOVEING = 1\n"
  "uint32 STATUS_STOP = 2\n"
  "uint32  status\n"
  "float32 pose";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
interfaces__msg__RobotStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interfaces__msg__RobotStatus__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 76, 76},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interfaces__msg__RobotStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interfaces__msg__RobotStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
