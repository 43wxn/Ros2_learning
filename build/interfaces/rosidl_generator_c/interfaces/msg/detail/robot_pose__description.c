// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from interfaces:msg/RobotPose.idl
// generated code does not contain a copyright notice

#include "interfaces/msg/detail/robot_pose__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_interfaces
const rosidl_type_hash_t *
interfaces__msg__RobotPose__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x12, 0xd5, 0xac, 0x2c, 0x55, 0xac, 0x31, 0xab,
      0x92, 0x1a, 0x6a, 0xaf, 0x09, 0x62, 0x3d, 0x97,
      0xd4, 0x3d, 0xb4, 0x47, 0x05, 0x50, 0x64, 0xa2,
      0x35, 0x36, 0x1c, 0x52, 0xc4, 0x63, 0x4d, 0xa0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/point__functions.h"
#include "geometry_msgs/msg/detail/pose__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Pose__EXPECTED_HASH = {1, {
    0xd5, 0x01, 0x95, 0x4e, 0x94, 0x76, 0xce, 0xa2,
    0x99, 0x69, 0x84, 0xe8, 0x12, 0x05, 0x4b, 0x68,
    0x02, 0x6a, 0xe0, 0xbf, 0xae, 0x78, 0x9d, 0x9a,
    0x10, 0xb2, 0x3d, 0xaf, 0x35, 0xcc, 0x90, 0xfa,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
#endif

static char interfaces__msg__RobotPose__TYPE_NAME[] = "interfaces/msg/RobotPose";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Pose__TYPE_NAME[] = "geometry_msgs/msg/Pose";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";

// Define type names, field names, and default values
static char interfaces__msg__RobotPose__FIELD_NAME__status[] = "status";
static char interfaces__msg__RobotPose__FIELD_NAME__pose[] = "pose";

static rosidl_runtime_c__type_description__Field interfaces__msg__RobotPose__FIELDS[] = {
  {
    {interfaces__msg__RobotPose__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__RobotPose__FIELD_NAME__pose, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription interfaces__msg__RobotPose__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interfaces__msg__RobotPose__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interfaces__msg__RobotPose__TYPE_NAME, 24, 24},
      {interfaces__msg__RobotPose__FIELDS, 2, 2},
    },
    {interfaces__msg__RobotPose__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 STATUS_MOVEING = 1\n"
  "uint32 STATUS_STOP = 2\n"
  "uint32  status\n"
  "geometry_msgs/Pose pose\n"
  "\n"
  "#\\xe5\\x89\\x8d\\xe4\\xb8\\xa4\\xe8\\xa1\\x8c\\xe6\\x98\\xaf\\xe5\\xb8\\xb8\\xe9\\x87\\x8f\\xe5\\xae\\x9a\\xe4\\xb9\\x89 \\xe7\\x9b\\xb8\\xe5\\xbd\\x93\\xe4\\xba\\x8e\\xe5\\xae\\x9a\\xe4\\xb9\\x89\\xe4\\xba\\x86\\xe4\\xb8\\x80\\xe4\\xb8\\xaa\\xe5\\xb8\\xb8\\xe9\\x87\\x8fSTAUS_MOVEING\\xe5\\x80\\xbc\\xe4\\xb8\\xba1\\xef\\xbc\\x8c\\xe5\\x90\\x8e\\xe4\\xb8\\xa4\\xe8\\xa1\\x8c\\xe6\\x98\\xaf\\xe5\\x8f\\x98\\xe9\\x87\\x8f\\xe5\\xae\\x9a\\xe4\\xb9\\x89\n"
  "#\\xe5\\x9c\\xa8\\xe4\\xbd\\xbf\\xe7\\x94\\xa8\\xe7\\x9a\\x84\\xe6\\x97\\xb6\\xe5\\x80\\x99\\xe5\\x8f\\xaf\\xe4\\xbb\\xa5\\xe7\\x9b\\xb4\\xe6\\x8e\\xa5\\xe4\\xbd\\xbf\\xe7\\x94\\xa8STATUS_MOVEING\\xe4\\xbb\\xa3\\xe6\\x9b\\xbf1 ...";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
interfaces__msg__RobotPose__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interfaces__msg__RobotPose__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 169, 169},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interfaces__msg__RobotPose__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interfaces__msg__RobotPose__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
