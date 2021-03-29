#ifndef CONVERTERS_H_L7OWNAZ8
#define CONVERTERS_H_L7OWNAZ8

// TODO
// should break this up by package,
// or even msg type -- otherwise this file will grow
// unwieldy, and also might affect compilation times

#include <memory>

#include <pybind11/pybind11.h>

#include <ros/duration.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Header.h>

static inline bool is_ros_msg_type(pybind11::handle src,
                                   const std::string &msg_type_name)
{
  namespace py = pybind11;
  if (!py::hasattr(src, "_type"))
  {
    return false;
  }
  std::string msg_type(src.attr("_type").cast<std::string>());
  if (msg_type != msg_type_name)
  {
    return false;
  }
  return true;
}

namespace pybind11
{
namespace detail
{

template<>
struct type_caster<ros::Duration>
{
 public:
 PYBIND11_TYPE_CASTER(ros::Duration, _("Duration"));

  bool load(handle src, bool)
  {
    value.sec = src.attr("sec").cast<int>();
    value.nsec = src.attr("nsec").cast<int>();
    return true;
  }

  static handle cast(ros::Duration cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("rospy");
    object MsgType = mod.attr("Duration");
    object msg = MsgType();
    msg.attr("secs") = pybind11::cast(cpp_msg.sec);
    msg.attr("nsecs") = pybind11::cast(cpp_msg.nsec);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<ros::Time>
{
 public:
 PYBIND11_TYPE_CASTER(ros::Time, _("ros::Time"));

  // python -> cpp
  bool load(handle src, bool)
  {
    PyObject *obj(src.ptr());
    if (!PyObject_HasAttrString(obj, "secs"))
    {
      return false;
    }
    if (!PyObject_HasAttrString(obj, "nsecs"))
    {
      return false;
    }

    value.sec = src.attr("secs").cast<uint32_t>();
    value.nsec = src.attr("nsecs").cast<uint32_t>();
    return true;
  }

  // cpp -> python
  static handle cast(ros::Time src, return_value_policy policy, handle parent)
  {
    object rospy = module::import("rospy");
    object TimeType = rospy.attr("Time");
    object pyts = TimeType();
    pyts.attr("secs") = pybind11::cast(src.sec);
    pyts.attr("nsecs") = pybind11::cast(src.nsec);
    pyts.inc_ref();
    return pyts;
  }
};

template<>
struct type_caster<std_msgs::Header>
{
 public:
 PYBIND11_TYPE_CASTER(std_msgs::Header, _("std_msgs::Header"));

  // python -> cpp
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "std_msgs/Header"))
    {
      return false;
    }
    value.seq = src.attr("seq").cast<uint32_t>();
    value.stamp = src.attr("stamp").cast<ros::Time>();
    value.frame_id = src.attr("frame_id").cast<std::string>();
    return true;
  }

  // cpp -> python
  static handle cast(std_msgs::Header header,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("std_msgs.msg._Header");
    object MsgType = mod.attr("Header");
    object msg = MsgType();
    msg.attr("seq") = pybind11::cast(header.seq);
    msg.attr("stamp") = pybind11::cast(header.stamp);
    msg.attr("frame_id") = pybind11::cast(header.frame_id);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<std_msgs::ColorRGBA>
{
 public:
 PYBIND11_TYPE_CASTER(std_msgs::ColorRGBA, _("std_msgs::ColorRGBA"));

  // python -> cpp
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "std_msgs/ColorRGBA"))
    {
      return false;
    }
    value.r = src.attr("r").cast<float>();
    value.g = src.attr("g").cast<float>();
    value.b = src.attr("b").cast<float>();
    value.a = src.attr("a").cast<float>();
    return true;
  }

  // cpp -> python
  static handle cast(std_msgs::ColorRGBA header,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("std_msgs.msg._ColorRGBA");
    object MsgType = mod.attr("ColorRGBA");
    object msg = MsgType();
    msg.attr("r") = pybind11::cast(header.r);
    msg.attr("g") = pybind11::cast(header.g);
    msg.attr("b") = pybind11::cast(header.b);
    msg.attr("a") = pybind11::cast(header.a);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<geometry_msgs::Point>
{
 public:
 PYBIND11_TYPE_CASTER(geometry_msgs::Point, _("geometry_msgs::Point"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Point"))
    {
      return false;
    }
    value.x = src.attr("x").cast<double>();
    value.y = src.attr("y").cast<double>();
    value.z = src.attr("z").cast<double>();
    return true;
  }

  static handle cast(geometry_msgs::Point pt,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Point");
    object MsgType = mod.attr("Point");
    object msg = MsgType();
    msg.attr("x") = pybind11::cast(pt.x);
    msg.attr("y") = pybind11::cast(pt.y);
    msg.attr("z") = pybind11::cast(pt.z);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<geometry_msgs::Vector3>
{
 public:
 PYBIND11_TYPE_CASTER(geometry_msgs::Vector3, _("geometry_msgs::Vector3"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Vector3"))
    {
      return false;
    }
    value.x = src.attr("x").cast<double>();
    value.y = src.attr("y").cast<double>();
    value.z = src.attr("z").cast<double>();
    return true;
  }

  static handle cast(geometry_msgs::Vector3 cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Vector3");
    object MsgType = mod.attr("Vector3");
    object msg = MsgType();
    msg.attr("x") = pybind11::cast(cpp_msg.x);
    msg.attr("y") = pybind11::cast(cpp_msg.y);
    msg.attr("z") = pybind11::cast(cpp_msg.z);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<geometry_msgs::Quaternion>
{
 public:
 PYBIND11_TYPE_CASTER(geometry_msgs::Quaternion,
                      _("geometry_msgs::Quaternion"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Quaternion"))
    {
      return false;
    }
    value.x = src.attr("x").cast<double>();
    value.y = src.attr("y").cast<double>();
    value.z = src.attr("z").cast<double>();
    value.w = src.attr("w").cast<double>();
    return true;
  }

  static handle cast(geometry_msgs::Quaternion cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Quaternion");
    object MsgType = mod.attr("Quaternion");
    object msg = MsgType();
    msg.attr("x") = pybind11::cast(cpp_msg.x);
    msg.attr("y") = pybind11::cast(cpp_msg.y);
    msg.attr("z") = pybind11::cast(cpp_msg.z);
    msg.attr("w") = pybind11::cast(cpp_msg.w);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<geometry_msgs::Transform>
{
 public:
 PYBIND11_TYPE_CASTER(geometry_msgs::Transform, _("geometry_msgs::Transform"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Transform"))
    {
      return false;
    }
    value.translation = src.attr("translation").cast<geometry_msgs::Vector3>();
    value.rotation = src.attr("rotation").cast<geometry_msgs::Quaternion>();
    return true;
  }

  static handle cast(geometry_msgs::Transform cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Transform");
    object MsgType = mod.attr("Transform");
    object msg = MsgType();
    msg.attr("translation") = pybind11::cast(cpp_msg.translation);
    msg.attr("rotation") = pybind11::cast(cpp_msg.rotation);
    msg.inc_ref();
    return msg;
  }
};


template<>
struct type_caster<geometry_msgs::Twist>
{
 public:
 PYBIND11_TYPE_CASTER(geometry_msgs::Twist, _("geometry_msgs::Twist"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Twist"))
    {
      return false;
    }
    value.linear = src.attr("linear").cast<geometry_msgs::Vector3>();
    value.angular = src.attr("angular").cast<geometry_msgs::Vector3>();
    return true;
  }

  static handle cast(geometry_msgs::Twist cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Twist");
    object MsgType = mod.attr("Twist");
    object msg = MsgType();
    msg.attr("linear") = pybind11::cast(cpp_msg.linear);
    msg.attr("angular") = pybind11::cast(cpp_msg.angular);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<geometry_msgs::TransformStamped>
{
 public:
 PYBIND11_TYPE_CASTER(geometry_msgs::TransformStamped,
                      _("geometry_msgs::TransformStamped"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/TransformStamped"))
    {
      return false;
    }
    value.header = src.attr("header").cast<std_msgs::Header>();
    value.child_frame_id = src.attr("child_frame_id").cast<std::string>();
    value.transform = src.attr("transform").cast<geometry_msgs::Transform>();
    return true;
  }

  static handle cast(geometry_msgs::TransformStamped cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._TransformStamped");
    object MsgType = mod.attr("TransformStamped");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("child_frame_id") = pybind11::cast(cpp_msg.child_frame_id);
    msg.attr("transform") = pybind11::cast(cpp_msg.transform);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<sensor_msgs::PointField>
{
 public:
 PYBIND11_TYPE_CASTER(sensor_msgs::PointField, _("sensor_msgs::PointField"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "sensor_msgs/PointField"))
    {
      return false;
    }
    value.name = src.attr("name").cast<std::string>();
    value.offset = src.attr("offset").cast<uint32_t>();
    value.datatype = src.attr("datatype").cast<uint8_t>();
    value.count = src.attr("count").cast<uint32_t>();
    return true;
  }

  static handle cast(sensor_msgs::PointField cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("sensor_msgs.msg._PointField");
    object MsgType = mod.attr("PointField");
    object msg = MsgType();
    // avoid !!python/unicode problem.
    // msg.attr("name") = pybind11::cast(cpp_msg.name);
    // msg.attr("name") = PyString_FromString(cpp_msg.name.c_str());
    msg.attr("name") = pybind11::cast(cpp_msg.name);
    msg.attr("offset") = pybind11::cast(cpp_msg.offset);
    msg.attr("datatype") = pybind11::cast(cpp_msg.datatype);
    msg.attr("count") = pybind11::cast(cpp_msg.count);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<sensor_msgs::PointCloud2>
{
 public:
 PYBIND11_TYPE_CASTER(sensor_msgs::PointCloud2, _("sensor_msgs::PointCloud2"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "sensor_msgs/PointCloud2"))
    {
      return false;
    }
    value.header = src.attr("header").cast<std_msgs::Header>();
    value.height = src.attr("height").cast<uint32_t>();
    value.width = src.attr("width").cast<uint32_t>();
    pybind11::list field_lst = src.attr("fields").cast<pybind11::list>();
    for (int i = 0; i < pybind11::len(field_lst); ++i)
    {
      sensor_msgs::PointField pf(
          (field_lst[i]).cast<sensor_msgs::PointField>());
      value.fields.push_back(pf);
    }
    value.is_bigendian = src.attr("is_bigendian").cast<bool>();
    value.point_step = src.attr("point_step").cast<uint32_t>();
    value.row_step = src.attr("row_step").cast<uint32_t>();
    std::string data_str = src.attr("data").cast<std::string>();
    value.data.insert(value.data.end(),
                      data_str.c_str(),
                      data_str.c_str() + data_str.length());
    value.is_dense = src.attr("is_dense").cast<bool>();
    return true;
  }

  static handle cast(sensor_msgs::PointCloud2 cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("sensor_msgs.msg._PointCloud2");
    object MsgType = mod.attr("PointCloud2");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("height") = pybind11::cast(cpp_msg.height);
    msg.attr("width") = pybind11::cast(cpp_msg.width);
    pybind11::list field_lst;
    for (size_t i = 0; i < cpp_msg.fields.size(); ++i)
    {
      const sensor_msgs::PointField &pf(cpp_msg.fields[i]);
      field_lst.append(pybind11::cast(pf));
    }
    msg.attr("fields") = field_lst;
    msg.attr("is_bigendian") = pybind11::cast(cpp_msg.is_bigendian);
    msg.attr("point_step") = pybind11::cast(cpp_msg.point_step);
    msg.attr("row_step") = pybind11::cast(cpp_msg.row_step);
    msg.attr("data") = pybind11::bytes(
        reinterpret_cast<const char *>(&cpp_msg.data[0]), cpp_msg.data.size());
    msg.attr("is_dense") = pybind11::cast(cpp_msg.is_dense);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<sensor_msgs::Image>
{
 public:
 PYBIND11_TYPE_CASTER(sensor_msgs::Image, _("sensor_msgs::Image"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "sensor_msgs/Image"))
    {
      return false;
    }
    value.header = src.attr("header").cast<std_msgs::Header>();
    value.height = src.attr("height").cast<uint32_t>();
    value.width = src.attr("width").cast<uint32_t>();
    value.encoding = src.attr("encoding").cast<std::string>();
    value.is_bigendian = src.attr("is_bigendian").cast<int>();
    value.step = src.attr("step").cast<uint32_t>();
    std::string data_str = src.attr("data").cast<std::string>();
    value.data.insert(value.data.end(),
                      data_str.c_str(),
                      data_str.c_str() + data_str.length());
    return true;
  }

  static handle cast(sensor_msgs::Image cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("sensor_msgs.msg._Image");
    object MsgType = mod.attr("Image");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("height") = pybind11::cast(cpp_msg.height);
    msg.attr("width") = pybind11::cast(cpp_msg.width);
    msg.attr("encoding") = pybind11::bytes(cpp_msg.encoding);
    msg.attr("is_bigendian") = pybind11::cast(cpp_msg.is_bigendian);
    msg.attr("step") = pybind11::cast(cpp_msg.step);
    msg.attr("data") = pybind11::bytes(reinterpret_cast<const char *>(&cpp_msg.data[0]), cpp_msg.data.size());
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<sensor_msgs::CameraInfo>
{
 public:
 PYBIND11_TYPE_CASTER(sensor_msgs::CameraInfo, _("sensor_msgs::CameraInfo"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "sensor_msgs/CameraInfo"))
    {
      return false;
    }

    value.height = src.attr("height").cast<uint32_t>();
    value.width = src.attr("width").cast<uint32_t>();
    value.distortion_model = src.attr("distortion_model").cast<std::string>();

    {
      for (auto item : src.attr("D"))
      {
        value.D.push_back(item.cast<double>());
      }
    }
    {
      int i = 0;
      for (auto item : src.attr("K"))
      {
        value.K[i] = item.cast<double>();
        ++i;
      }
    }
    {
      int i = 0;
      for (auto item : src.attr("R"))
      {
        value.R[i] = item.cast<double>();
        ++i;
      }
    }
    {
      int i = 0;
      for (auto item : src.attr("P"))
      {
        value.P[i] = item.cast<double>();
        ++i;
      }
    }

    value.header = src.attr("header").cast<std_msgs::Header>();

    return true;
  }

  static handle cast(sensor_msgs::CameraInfo cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("sensor_msgs.msg._CameraInfo");
    object MsgType = mod.attr("CameraInfo");
    object msg = MsgType();

    // TODO untested

    msg.attr("height") = cpp_msg.height;
    msg.attr("width") = cpp_msg.width;

    msg.attr("distortion_model") = cpp_msg.distortion_model;

    for (size_t i = 0; i < cpp_msg.D.size(); ++i)
    {
      pybind11::list D = msg.attr("D");
      D[i] = cpp_msg.K[i];
    }

    for (size_t i = 0; i < cpp_msg.K.size(); ++i)
    {
      pybind11::list K = msg.attr("K");
      K[i] = cpp_msg.K[i];
    }

    for (size_t i = 0; i < cpp_msg.R.size(); ++i)
    {
      pybind11::list R = msg.attr("R");
      R[i] = cpp_msg.K[i];
    }

    for (size_t i = 0; i < cpp_msg.P.size(); ++i)
    {
      pybind11::list P = msg.attr("P");
      P[i] = cpp_msg.P[i];
    }

    msg.attr("header") = pybind11::cast(cpp_msg.header);

    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<geometry_msgs::Pose>
{
 public:
 PYBIND11_TYPE_CASTER(geometry_msgs::Pose, _("geometry_msgs::Pose"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Pose"))
    {
      return false;
    }
    value.position = src.attr("position").cast<geometry_msgs::Point>();
    value.orientation =
        (src.attr("orientation")).cast<geometry_msgs::Quaternion>();
    return true;
  }

  static handle cast(geometry_msgs::Pose cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Pose");
    object MsgType = mod.attr("Pose");
    object msg = MsgType();
    msg.attr("position") = pybind11::cast(cpp_msg.position);
    msg.attr("orientation") = pybind11::cast(cpp_msg.orientation);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<trajectory_msgs::JointTrajectoryPoint>
{
 public:
 PYBIND11_TYPE_CASTER(trajectory_msgs::JointTrajectoryPoint, _("trajectory_msgs::JointTrajectoryPoint"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "trajectory_msgs/JointTrajectoryPoint"))
    {
      return false;
    }
    value.time_from_start = src.attr("time_from_start").cast<ros::Duration>();
    value.positions = src.attr("positions").cast<std::vector<double>>();
    value.velocities = src.attr("velocities").cast<std::vector<double>>();
    value.accelerations = src.attr("accelerations").cast<std::vector<double>>();
    value.effort = src.attr("effort").cast<std::vector<double>>();
    return true;
  }

  static handle cast(trajectory_msgs::JointTrajectoryPoint cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("trajectory_msgs.msg._JointTrajectoryPoint");
    object MsgType = mod.attr("JointTrajectoryPoint");
    object msg = MsgType();
    msg.attr("positions") = pybind11::cast(cpp_msg.positions);
    msg.attr("velocities") = pybind11::cast(cpp_msg.velocities);
    msg.attr("accelerations") = pybind11::cast(cpp_msg.accelerations);
    msg.attr("effort") = pybind11::cast(cpp_msg.effort);
    msg.attr("time_from_start") = pybind11::cast(cpp_msg.time_from_start);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<trajectory_msgs::MultiDOFJointTrajectoryPoint>
{
 public:
 PYBIND11_TYPE_CASTER(trajectory_msgs::MultiDOFJointTrajectoryPoint,
                      _("trajectory_msgs::MultiDOFJointTrajectoryPoint"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "trajectory_msgs/MultiDOFJointTrajectoryPoint"))
    {
      return false;
    }
    value.time_from_start = src.attr("time_from_start").cast<ros::Duration>();
    value.transforms = src.attr("transforms").cast<std::vector<geometry_msgs::Transform>>();
    value.velocities = src.attr("velocities").cast<std::vector<geometry_msgs::Twist>>();
    value.accelerations = src.attr("accelerations").cast<std::vector<geometry_msgs::Twist>>();
    return true;
  }

  static handle cast(trajectory_msgs::MultiDOFJointTrajectoryPoint cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("trajectory_msgs.msg._MultiDOFTrajectoryPoint");
    object MsgType = mod.attr("MultiJointTrajectoryPoint");
    object msg = MsgType();
    msg.attr("transforms") = pybind11::cast(cpp_msg.transforms);
    msg.attr("velocities") = pybind11::cast(cpp_msg.velocities);
    msg.attr("accelerations") = pybind11::cast(cpp_msg.accelerations);
    msg.attr("time_from_start") = pybind11::cast(cpp_msg.time_from_start);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<trajectory_msgs::JointTrajectory>
{
 public:
 PYBIND11_TYPE_CASTER(trajectory_msgs::JointTrajectory, _("trajectory_msgs::JointTrajectory"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "trajectory_msgs/JointTrajectory"))
    {
      return false;
    }
    value.header = src.attr("header").cast<std_msgs::Header>();
    value.joint_names = src.attr("joint_names").cast<std::vector<std::string>>();
    value.points = src.attr("points").cast<std::vector<trajectory_msgs::JointTrajectoryPoint>>();
    return true;
  }

  static handle cast(trajectory_msgs::JointTrajectory cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("trajectory_msgs.msg._JointTrajectory");
    object MsgType = mod.attr("JointTrajectory");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("joint_names") = pybind11::cast(cpp_msg.joint_names);
    msg.attr("points") = pybind11::cast(cpp_msg.points);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<trajectory_msgs::MultiDOFJointTrajectory>
{
 public:
 PYBIND11_TYPE_CASTER(trajectory_msgs::MultiDOFJointTrajectory, _("trajectory_msgs::MultiDOFJointTrajectory"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "trajectory_msgs/MultiDOFJointTrajectory"))
    {
      return false;
    }
    value.header = src.attr("header").cast<std_msgs::Header>();
    value.joint_names = src.attr("joint_names").cast<std::vector<std::string>>();
    value.points = src.attr("points").cast<std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>>();
    return true;
  }

  static handle cast(trajectory_msgs::MultiDOFJointTrajectory cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("trajectory_msgs.msg._MultiDOFJointTrajectory");
    object MsgType = mod.attr("MultiDOFJointTrajectory");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("joint_names") = pybind11::cast(cpp_msg.joint_names);
    msg.attr("points") = pybind11::cast(cpp_msg.points);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<sensor_msgs::JointState>
{
 public:
 PYBIND11_TYPE_CASTER(sensor_msgs::JointState, _("sensor_msgs::JointState"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "sensor_msgs/JointState"))
    {
      return false;
    }
    value.header = src.attr("header").cast<std_msgs::Header>();
    value.name = src.attr("name").cast<std::vector<std::string>>();
    value.position = src.attr("position").cast<std::vector<double>>();
    value.velocity = src.attr("velocity").cast<std::vector<double>>();
    value.effort = src.attr("effort").cast<std::vector<double>>();
    return true;
  }

  static handle cast(sensor_msgs::JointState cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("sensor_msgs.msg._JointState");
    object MsgType = mod.attr("JointState");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("name") = pybind11::cast(cpp_msg.name);
    msg.attr("position") = pybind11::cast(cpp_msg.position);
    msg.attr("velocity") = pybind11::cast(cpp_msg.velocity);
    msg.attr("effort") = pybind11::cast(cpp_msg.effort);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<moveit_msgs::AttachedCollisionObject>
{
 public:
 PYBIND11_TYPE_CASTER(moveit_msgs::AttachedCollisionObject, _("moveit_msgs::AttachedCollisionObject"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "moveit_msgs/AttachedCollisionObject"))
    {
      return false;
    }
    value.link_name = src.attr("link_name").cast<std::string>();
    value.object = src.attr("object").cast<moveit_msgs::CollisionObject>();
    value.touch_links = src.attr("touch_links").cast<std::vector<std::string>>();
    value.detach_posture = src.attr("detach_posture").cast<trajectory_msgs::JointTrajectory>();
    value.weight = src.attr("weight").cast<double>();
    return true;
  }

  static handle cast(moveit_msgs::AttachedCollisionObject cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("moveit_msgs.msg._AttachedCollisionObject");
    object MsgType = mod.attr("AttachedCollisionObject");
    object msg = MsgType();
    msg.attr("link_name") = pybind11::cast(cpp_msg.link_name);
    msg.attr("object") = pybind11::cast(cpp_msg.object);
    msg.attr("touch_links") = pybind11::cast(cpp_msg.touch_links);
    msg.attr("detach_posture") = pybind11::cast(cpp_msg.detach_posture);
    msg.attr("weight") = pybind11::cast(cpp_msg.weight);
    msg.inc_ref();
    return msg;
  }
};


template<>
struct type_caster<moveit_msgs::RobotState>
{
 public:
 PYBIND11_TYPE_CASTER(moveit_msgs::RobotState, _("moveit_msgs::RobotState"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "moveit_msgs/RobotState"))
    {
      return false;
    }
    value.joint_state = src.attr("joint_state").cast<sensor_msgs::JointState>();
//    value.multi_dof_joint_state = src.attr("multi_dof_joint_state").cast<sensor_msgs::MultiDOFJointState>();
    value.attached_collision_objects = src.attr("attached_collision_objects").cast<std::vector<moveit_msgs::AttachedCollisionObject>>();
    value.is_diff = src.attr("is_diff").cast<bool>();
    return true;
  }

  static handle cast(moveit_msgs::RobotState cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("moveit_msgs.msg._RobotState");
    object MsgType = mod.attr("RobotState");
    object msg = MsgType();
    msg.attr("joint_state") = pybind11::cast(cpp_msg.joint_state);
//    msg.attr("multi_dof_joint_state") = pybind11::cast(cpp_msg.multi_dof_joint_state);
    msg.attr("attached_collision_objects") = pybind11::cast(cpp_msg.attached_collision_objects);
    msg.attr("is_diff") = pybind11::cast(cpp_msg.is_diff);
    msg.inc_ref();
    return msg;
  }
};


template<>
struct type_caster<moveit_msgs::RobotTrajectory>
{
 public:
 PYBIND11_TYPE_CASTER(moveit_msgs::RobotTrajectory, _("moveit_msgs::RobotTrajectory"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "moveit_msgs/RobotTrajectory"))
    {
      return false;
    }
    value.joint_trajectory = src.attr("joint_trajectory").cast<trajectory_msgs::JointTrajectory>();
    value.multi_dof_joint_trajectory = src.attr(
        "multi_dof_joint_trajectory").cast<trajectory_msgs::MultiDOFJointTrajectory>();
    return true;
  }

  static handle cast(moveit_msgs::RobotTrajectory cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("moveit_msgs.msg._RobotTrajectory");
    object MsgType = mod.attr("RobotTrajectory");
    object msg = MsgType();
    msg.attr("joint_trajectory") = pybind11::cast(cpp_msg.joint_trajectory);
    msg.attr("multi_dof_joint_trajectory") = pybind11::cast(cpp_msg.multi_dof_joint_trajectory);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<shape_msgs::SolidPrimitive>
{
 public:
 PYBIND11_TYPE_CASTER(shape_msgs::SolidPrimitive, _("shape_msgs::SolidPrimitive"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "shape_msgs/SolidPrimitive"))
    {
      return false;
    }
    value.dimensions = src.attr("dimensions").cast<std::vector<double>>();
    value.type = src.attr("type").cast<int>();
    return true;
  }

  static handle cast(shape_msgs::SolidPrimitive cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("shape_msgs.msg._SolidPrimitive");
    object MsgType = mod.attr("SolidPrimitive");
    object msg = MsgType();
    msg.attr("dimensions") = pybind11::cast(cpp_msg.dimensions);
    msg.attr("type") = pybind11::cast(cpp_msg.type);
    msg.inc_ref();
    return msg;
  }
};


template<>
struct type_caster<shape_msgs::MeshTriangle>
{
 public:
 PYBIND11_TYPE_CASTER(shape_msgs::MeshTriangle, _("shape_msgs::MeshTriangle"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "shape_msgs/MeshTriangle"))
    {
      return false;
    }
    value.vertex_indices = src.attr("vertex_indices").cast<boost::array<uint32_t, 3>>();
    return true;
  }

  static handle cast(shape_msgs::MeshTriangle cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("shape_msgs.msg._MeshTriangle");
    object MsgType = mod.attr("MeshTriangle");
    object msg = MsgType();
    msg.attr("vertex_indices") = pybind11::cast(cpp_msg.vertex_indices);
    msg.inc_ref();
    return msg;
  }
};


template<>
struct type_caster<shape_msgs::Mesh>
{
 public:
 PYBIND11_TYPE_CASTER(shape_msgs::Mesh, _("shape_msgs::Mesh"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "shape_msgs/Mesh"))
    {
      return false;
    }
    value.triangles = src.attr("triangles").cast<std::vector<shape_msgs::MeshTriangle>>();
    value.vertices = src.attr("vertices").cast<std::vector<geometry_msgs::Point>>();
    return true;
  }

  static handle cast(shape_msgs::Mesh cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("shape_msgs.msg._Mesh");
    object MsgType = mod.attr("Mesh");
    object msg = MsgType();
    msg.attr("triangles") = pybind11::cast(cpp_msg.triangles);
    msg.attr("vertices") = pybind11::cast(cpp_msg.vertices);
    msg.inc_ref();
    return msg;
  }
};


template<>
struct type_caster<moveit_msgs::CollisionObject>
{
 public:
 PYBIND11_TYPE_CASTER(moveit_msgs::CollisionObject, _("moveit_msgs::CollisionObject"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "moveit_msgs/CollisionObject"))
    {
      return false;
    }
    value.header = src.attr("header").cast<std_msgs::Header>();
    value.id = src.attr("id").cast<std::string>();
    value.primitives = src.attr("primitives").cast<std::vector<shape_msgs::SolidPrimitive>>();
    value.primitive_poses = src.attr("primitive_poses").cast<std::vector<geometry_msgs::Pose>>();
    value.meshes = src.attr("meshes").cast<std::vector<shape_msgs::Mesh>>();
    value.mesh_poses = src.attr("mesh_poses").cast<std::vector<geometry_msgs::Pose>>();
    value.operation = src.attr("operation").cast<int8_t>();
    return true;
  }

  static handle cast(moveit_msgs::CollisionObject cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("moveit_msgs.msg._CollisionObject");
    object MsgType = mod.attr("CollisionObject");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("id") = pybind11::cast(cpp_msg.id);
    msg.attr("primitives") = pybind11::cast(cpp_msg.primitives);
    msg.attr("primitive_poses") = pybind11::cast(cpp_msg.primitive_poses);
    msg.attr("operation") = pybind11::cast(cpp_msg.operation);
    msg.inc_ref();
    return msg;
  }
};


template<>
struct type_caster<octomap_msgs::Octomap>
{
 public:
 PYBIND11_TYPE_CASTER(octomap_msgs::Octomap, _("octomap_msgs::Octomap"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "octomap_msgs/Octomap"))
    {
      return false;
    }
    value.header = src.attr("header").cast<std_msgs::Header>();
    value.binary = src.attr("binary").cast<bool>();
    value.id = src.attr("id").cast<std::string>();
    value.resolution = src.attr("resolution").cast<double>();
    value.data = src.attr("data").cast<std::vector<int8_t>>();
    return true;
  }

  static handle cast(octomap_msgs::Octomap cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("octomap_msgs.msg._Octomap");
    object MsgType = mod.attr("Octomap");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("binary") = pybind11::cast(cpp_msg.binary);
    msg.attr("id") = pybind11::cast(cpp_msg.id);
    msg.attr("resolution") = pybind11::cast(cpp_msg.resolution);
    msg.attr("data") = pybind11::cast(cpp_msg.data);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<octomap_msgs::OctomapWithPose>
{
 public:
 PYBIND11_TYPE_CASTER(octomap_msgs::OctomapWithPose, _("octomap_msgs::OctomapWithPose"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "octomap_msgs/OctomapWithPose"))
    {
      return false;
    }
    value.header = src.attr("header").cast<std_msgs::Header>();
    value.origin = src.attr("origin").cast<geometry_msgs::Pose>();
    value.octomap = src.attr("octomap").cast<octomap_msgs::Octomap>();
    return true;
  }

  static handle cast(octomap_msgs::OctomapWithPose cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("octomap_msgs.msg._OctomapWithPose");
    object MsgType = mod.attr("OctomapWithPose");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("origin") = pybind11::cast(cpp_msg.origin);
    msg.attr("octomap") = pybind11::cast(cpp_msg.octomap);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<moveit_msgs::PlanningSceneWorld>
{
 public:
 PYBIND11_TYPE_CASTER(moveit_msgs::PlanningSceneWorld, _("moveit_msgs::PlanningSceneWorld"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "moveit_msgs/PlanningSceneWorld"))
    {
      return false;
    }
    value.collision_objects = src.attr("collision_objects").cast<std::vector<moveit_msgs::CollisionObject>>();
    value.octomap = src.attr("octomap").cast<octomap_msgs::OctomapWithPose>();
    return true;
  }

  static handle cast(moveit_msgs::PlanningSceneWorld cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("moveit_msgs.msg._PlanningSceneWorld");
    object MsgType = mod.attr("PlanningSceneWorld");
    object msg = MsgType();
    msg.attr("collision_objects") = pybind11::cast(cpp_msg.collision_objects);
    msg.attr("octomap") = pybind11::cast(cpp_msg.octomap);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<moveit_msgs::LinkScale>
{
 public:
 PYBIND11_TYPE_CASTER(moveit_msgs::LinkScale, _("moveit_msgs::LinkScale"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "moveit_msgs/LinkScale"))
    {
      return false;
    }
    value.link_name = src.attr("link_name").cast<std::string>();
    value.scale = src.attr("scale").cast<double>();
    return true;
  }

  static handle cast(moveit_msgs::LinkScale cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("moveit_msgs.msg._LinkScale");
    object MsgType = mod.attr("LinkScale");
    object msg = MsgType();
    msg.attr("link_name") = pybind11::cast(cpp_msg.link_name);
    msg.attr("scale") = pybind11::cast(cpp_msg.scale);
    msg.inc_ref();
    return msg;
  }
};


template<>
struct type_caster<moveit_msgs::ObjectColor>
{
 public:
 PYBIND11_TYPE_CASTER(moveit_msgs::ObjectColor, _("moveit_msgs::ObjectColor"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "moveit_msgs/ObjectColor"))
    {
      return false;
    }
    value.id = src.attr("id").cast<std::string>();
    value.color = src.attr("color").cast<std_msgs::ColorRGBA>();
    return true;
  }

  static handle cast(moveit_msgs::ObjectColor cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("moveit_msgs.msg._ObjectColor");
    object MsgType = mod.attr("ObjectColor");
    object msg = MsgType();
    msg.attr("id") = pybind11::cast(cpp_msg.id);
    msg.attr("color") = pybind11::cast(cpp_msg.color);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<moveit_msgs::LinkPadding>
{
 public:
 PYBIND11_TYPE_CASTER(moveit_msgs::LinkPadding, _("moveit_msgs::LinkPadding"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "moveit_msgs/LinkPadding"))
    {
      return false;
    }
    value.link_name = src.attr("link_name").cast<std::string>();
    value.padding = src.attr("padding").cast<double>();
    return true;
  }

  static handle cast(moveit_msgs::LinkPadding cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("moveit_msgs.msg._LinkPadding");
    object MsgType = mod.attr("LinkPadding");
    object msg = MsgType();
    msg.attr("link_name") = pybind11::cast(cpp_msg.link_name);
    msg.attr("padding") = pybind11::cast(cpp_msg.padding);
    msg.inc_ref();
    return msg;
  }
};


template<>
struct type_caster<moveit_msgs::AllowedCollisionEntry>
{
 public:
 PYBIND11_TYPE_CASTER(moveit_msgs::AllowedCollisionEntry, _("moveit_msgs::AllowedCollisionEntry"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "moveit_msgs/AllowedCollisionEntry"))
    {
      return false;
    }
    value.enabled = src.attr("enabled").cast<std::vector<uint8_t>>();
    return true;
  }

  static handle cast(moveit_msgs::AllowedCollisionEntry cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("moveit_msgs.msg._AllowedCollisionEntry");
    object MsgType = mod.attr("AllowedCollisionEntry");
    object msg = MsgType();
    msg.attr("enabled") = pybind11::cast(cpp_msg.enabled);
    msg.inc_ref();
    return msg;
  }
};

template<>
struct type_caster<moveit_msgs::AllowedCollisionMatrix>
{
 public:
 PYBIND11_TYPE_CASTER(moveit_msgs::AllowedCollisionMatrix, _("moveit_msgs::AllowedCollisionMatrix"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "moveit_msgs/AllowedCollisionMatrix"))
    {
      return false;
    }
    value.entry_names = src.attr("entry_names").cast<std::vector<std::string>>();
    value.entry_values = src.attr("entry_values").cast<std::vector<moveit_msgs::AllowedCollisionEntry>>();
    value.default_entry_names = src.attr("default_entry_names").cast<std::vector<std::string>>();
    value.default_entry_values = src.attr("default_entry_values").cast<std::vector<uint8_t>>();
    return true;
  }

  static handle cast(moveit_msgs::AllowedCollisionMatrix cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("moveit_msgs.msg._AllowedCollisionMatrix");
    object MsgType = mod.attr("AllowedCollisionMatrix");
    object msg = MsgType();
    msg.attr("entry_names") = pybind11::cast(cpp_msg.entry_names);
    msg.attr("entry_values") = pybind11::cast(cpp_msg.entry_values);
    msg.attr("default_entry_names") = pybind11::cast(cpp_msg.default_entry_names);
    msg.attr("default_entry_values") = pybind11::cast(cpp_msg.default_entry_values);
    msg.inc_ref();
    return msg;
  }
};


template<>
struct type_caster<moveit_msgs::PlanningScene>
{
 public:
 PYBIND11_TYPE_CASTER(moveit_msgs::PlanningScene, _("moveit_msgs::PlanningScene"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "moveit_msgs/PlanningScene"))
    {
      return false;
    }
    value.name = src.attr("name").cast<std::string>();
    value.robot_state = src.attr("robot_state").cast<moveit_msgs::RobotState>();
    value.robot_model_name = src.attr("robot_model_name").cast<std::string>();
    value.fixed_frame_transforms = src.attr(
        "fixed_frame_transforms").cast<std::vector<geometry_msgs::TransformStamped>>();
    value.allowed_collision_matrix = src.attr("allowed_collision_matrix").cast<moveit_msgs::AllowedCollisionMatrix>();
    value.link_padding = src.attr("link_padding").cast<std::vector<moveit_msgs::LinkPadding>>();
    value.link_scale = src.attr("link_scale").cast<std::vector<moveit_msgs::LinkScale>>();
    value.object_colors = src.attr("object_colors").cast<std::vector<moveit_msgs::ObjectColor>>();
    value.world = src.attr("world").cast<moveit_msgs::PlanningSceneWorld>();
    value.is_diff = src.attr("is_diff").cast<bool>();
    return true;
  }

  static handle cast(moveit_msgs::PlanningScene cpp_msg,
                     return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("moveit_msgs.msg._PlanningScene");
    object MsgType = mod.attr("PlanningScene");
    object msg = MsgType();
    msg.attr("name") = pybind11::cast(cpp_msg.name);
    msg.attr("robot_state") = pybind11::cast(cpp_msg.robot_state);
    msg.attr("robot_model_name") = pybind11::cast(cpp_msg.robot_model_name);
    msg.attr("fixed_frame_transforms") = pybind11::cast(cpp_msg.fixed_frame_transforms);
    msg.attr("allowed_collision_matrix") = pybind11::cast(cpp_msg.allowed_collision_matrix);
    msg.attr("link_padding") = pybind11::cast(cpp_msg.link_padding);
    msg.attr("link_scale") = pybind11::cast(cpp_msg.link_scale);
    msg.attr("object_colors") = pybind11::cast(cpp_msg.object_colors);
    msg.attr("world") = pybind11::cast(cpp_msg.world);
    msg.attr("is_diff") = pybind11::cast(cpp_msg.is_diff);
    msg.inc_ref();
    return msg;
  }
};

}
}

#endif /* end of include guard: CONVERTERS_H_L7OWNAZ8 */
