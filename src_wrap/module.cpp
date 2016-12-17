/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/


#include <memory>

#include <pybind11/pybind11.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

//#include "serialization.h"


static inline
bool is_ros_msg_type(pybind11::handle src, const std::string& msg_type_name) {
  namespace py = pybind11;
  if (!py::hasattr(src, "_type")) { return false; }
  std::string msg_type(src.attr("_type").cast<std::string>());
  if (msg_type != msg_type_name) { return false; }
  return true;
}

namespace pybind11 { namespace detail {

template <> struct type_caster<ros::Time> {
  public:
   PYBIND11_TYPE_CASTER(ros::Time, _("ros::Time"));

   // python -> cpp
   bool load(handle src, bool) {
     PyObject *obj(src.ptr());
     if (!PyObject_HasAttrString(obj, "secs")) { return false; }
     if (!PyObject_HasAttrString(obj, "nsecs")) { return false; }

     value.sec = (src.attr("secs")).cast<uint32_t>();
     value.nsec = (src.attr("nsecs")).cast<uint32_t>();
     return true;
   }

   // cpp -> python
   static handle cast(ros::Time src, return_value_policy policy, handle parent) {
     object rospy = module::import("rospy");
     object TimeType = rospy.attr("Time");
     object pyts = TimeType();
     pyts.attr("secs") = pybind11::cast(src.sec);
     pyts.attr("nsecs") = pybind11::cast(src.nsec);
     pyts.inc_ref();
     return pyts;
   }
};

template <> struct type_caster<std_msgs::Header> {
  public:
   PYBIND11_TYPE_CASTER(std_msgs::Header, _("std_msgs::Header"));

   // python -> cpp
   bool load(handle src, bool) {
     if (!is_ros_msg_type(src, "std_msgs/Header")) { return false; }
     value.seq = src.attr("seq").cast<uint32_t>();
     value.stamp = src.attr("stamp").cast<ros::Time>();
     value.frame_id = src.attr("frame_id").cast<std::string>();
     return true;
   }

   // cpp -> python
   static handle cast(std_msgs::Header header, return_value_policy policy, handle parent) {
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

template <> struct type_caster<geometry_msgs::Point> {
  public:
   PYBIND11_TYPE_CASTER(geometry_msgs::Point, _("geometry_msgs::Point"));

   bool load(handle src, bool) {
    if (!is_ros_msg_type(src, "geometry_msgs/Point")) { return false; }
    value.x = (src.attr("x")).cast<double>();
    value.y = (src.attr("y")).cast<double>();
    value.z = (src.attr("z")).cast<double>();
    return true;
  }

  static handle cast(geometry_msgs::Point pt, return_value_policy policy, handle parent) {
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

template <> struct type_caster<geometry_msgs::Vector3> {
  public:
  PYBIND11_TYPE_CASTER(geometry_msgs::Vector3, _("geometry_msgs::Vector3"));
  bool load(handle src, bool) {
    if (!is_ros_msg_type(src, "geometry_msgs/Vector3")) { return false; }
    value.x = (src.attr("x")).cast<double>();
    value.y = (src.attr("y")).cast<double>();
    value.z = (src.attr("z")).cast<double>();
    return true;
  }

  static handle cast(geometry_msgs::Vector3 cpp_msg, return_value_policy policy, handle parent) {
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

template <> struct type_caster<geometry_msgs::Quaternion> {
  public:
  PYBIND11_TYPE_CASTER(geometry_msgs::Quaternion, _("geometry_msgs::Quaternion"));
  bool load(handle src, bool) {
    if (!is_ros_msg_type(src, "geometry_msgs/Quaternion")) { return false; }
    value.x = (src.attr("x")).cast<double>();
    value.y = (src.attr("y")).cast<double>();
    value.z = (src.attr("z")).cast<double>();
    value.w = (src.attr("w")).cast<double>();
    return true;
  }

  static handle cast(geometry_msgs::Quaternion cpp_msg, return_value_policy policy, handle parent) {
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

template <> struct type_caster<geometry_msgs::Transform> {
  public:
  PYBIND11_TYPE_CASTER(geometry_msgs::Transform, _("geometry_msgs::Transform"));
  bool load(handle src, bool) {
    if (!is_ros_msg_type(src, "geometry_msgs/Transform")) { return false; }
    value.translation = (src.attr("translation")).cast<geometry_msgs::Vector3>();
    value.rotation = (src.attr("rotation")).cast<geometry_msgs::Quaternion>();
    return true;
  }

  static handle cast(geometry_msgs::Transform cpp_msg, return_value_policy policy, handle parent) {
    object mod = module::import("geometry_msgs.msg._Transform");
    object MsgType = mod.attr("Transform");
    object msg = MsgType();
    msg.attr("translation") = pybind11::cast(cpp_msg.translation);
    msg.attr("rotation") = pybind11::cast(cpp_msg.rotation);
    msg.inc_ref();
    return msg;
  }
};

template <> struct type_caster<geometry_msgs::TransformStamped> {
  public:
  PYBIND11_TYPE_CASTER(geometry_msgs::TransformStamped, _("geometry_msgs::TransformStamped"));
  bool load(handle src, bool) {
    if (!is_ros_msg_type(src, "geometry_msgs/TransformStamped")) { return false; }
    value.header = src.attr("header").cast<std_msgs::Header>();
    value.child_frame_id = src.attr("child_frame_id").cast<std::string>();
    value.transform = src.attr("transform").cast<geometry_msgs::Transform>();
    return true;
  }

  static handle cast(geometry_msgs::TransformStamped cpp_msg, return_value_policy policy, handle parent) {
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

template <> struct type_caster<sensor_msgs::PointField> {
  public:
  PYBIND11_TYPE_CASTER(sensor_msgs::PointField, _("sensor_msgs::PointField"));
  bool load(handle src, bool) {
    if (!is_ros_msg_type(src, "sensor_msgs/PointField")) { return false; }
    value.name = (src.attr("name")).cast<std::string>();
    value.offset = (src.attr("offset")).cast<uint32_t>();
    value.datatype = (src.attr("datatype")).cast<uint8_t>();
    value.count = (src.attr("count")).cast<uint32_t>();
    return true;
  }

  static handle cast(sensor_msgs::PointField cpp_msg, return_value_policy policy, handle parent) {
    object mod = module::import("sensor_msgs.msg._PointField");
    object MsgType = mod.attr("PointField");
    object msg = MsgType();
    msg.attr("name") = pybind11::cast(cpp_msg.name);
    msg.attr("offset") = pybind11::cast(cpp_msg.offset);
    msg.attr("datatype") = pybind11::cast(cpp_msg.datatype);
    msg.attr("count") = pybind11::cast(cpp_msg.count);
    msg.inc_ref();
    return msg;
  }
};

template <> struct type_caster<sensor_msgs::PointCloud2> {
  public:
  PYBIND11_TYPE_CASTER(sensor_msgs::PointCloud2, _("sensor_msgs::PointCloud2"));
  bool load(handle src, bool) {
    if (!is_ros_msg_type(src, "sensor_msgs/PointCloud2")) { return false; }
    value.header = (src.attr("header")).cast<std_msgs::Header>();
    value.height = (src.attr("height")).cast<uint32_t>();
    value.width = (src.attr("width")).cast<uint32_t>();
    pybind11::list field_lst = (src.attr("fields")).cast<pybind11::list>();
    for (int i=0; i < pybind11::len(field_lst); ++i) {
      sensor_msgs::PointField pf((field_lst[i]).cast<sensor_msgs::PointField>());
      value.fields.push_back(pf);
    }
    value.is_bigendian = (src.attr("is_bigendian")).cast<bool>();
    value.point_step = (src.attr("point_step")).cast<uint32_t>();
    value.row_step = (src.attr("row_step")).cast<uint32_t>();
    std::string data_str = (src.attr("data")).cast<std::string>();
    value.data.insert(value.data.end(), data_str.c_str(), data_str.c_str()+data_str.length());
    value.is_dense = (src.attr("is_dense")).cast<uint32_t>();
    return true;
  }

  static handle cast(sensor_msgs::PointCloud2 cpp_msg, return_value_policy policy, handle parent) {
    object mod = module::import("sensor_msgs.msg._PointCloud2");
    object MsgType = mod.attr("PointCloud2");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("height") = pybind11::cast(cpp_msg.height);
    msg.attr("width") = pybind11::cast(cpp_msg.width);
    //msg.attr("fields") = pybind11::cast(cpp_msg.fields);
    //pybind11::list field_lst = (msg.attr("fields")).cast<pybind11::list>();
    pybind11::list field_lst;
    for (size_t i=0; i < cpp_msg.fields.size(); ++i) {
      const sensor_msgs::PointField& pf(cpp_msg.fields[i]);
      field_lst.append(pybind11::cast(pf));
    }
    msg.attr("fields") = field_lst;
    msg.attr("is_bigendian") = pybind11::cast(cpp_msg.is_bigendian);
    msg.attr("point_step") = pybind11::cast(cpp_msg.point_step);
    msg.attr("row_step") = pybind11::cast(cpp_msg.row_step);
    std::string data_str(cpp_msg.data.begin(), cpp_msg.data.end());
    pybind11::bytes data_str2(data_str);
    msg.attr("data") = data_str2;
    msg.attr("is_dense") = pybind11::cast(cpp_msg.is_dense);

    msg.inc_ref();
    return msg;
  }
};


} }

#if 0
struct SensorMsgsPointCloud2 {

  static void * convertible(PyObject * obj_ptr) {
    return ConvertibleRosMessage(obj_ptr, "sensor_msgs/PointCloud2");
  }

  static void construct(PyObject * obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data) {
    namespace py = boost::python;
    typedef py::converter::rvalue_from_python_storage< sensor_msgs::PointCloud2 > StorageT;
    void* storage = reinterpret_cast< StorageT* >(data)->storage.bytes;
    new (storage) sensor_msgs::PointCloud2; // placement new
    data->convertible = storage;

    sensor_msgs::PointCloud2* pc = static_cast< sensor_msgs::PointCloud2* >(storage);
    // get borrowed reference
    py::handle<> handle(py::borrowed(obj_ptr));
    py::object o(handle);
    pc->header = py::extract<std_msgs::Header>(o.attr("header"));
    pc->height = py::extract<uint32_t>(o.attr("height"));
    pc->width = py::extract<uint32_t>(o.attr("width"));
    py::list field_lst = py::extract<py::list>(o.attr("fields"));
    //std::cerr << "py::len(field_lst) = " << py::len(field_lst) << std::endl;
    for (int i=0; i < py::len(field_lst); ++i) {
      sensor_msgs::PointField pf(py::extract< sensor_msgs::PointField >(field_lst[i]));
      pc->fields.push_back(pf);
    }
    pc->is_bigendian = py::extract<bool>(o.attr("is_bigendian"));
    pc->point_step = py::extract<uint32_t>(o.attr("point_step"));
    pc->row_step = py::extract<uint32_t>(o.attr("row_step"));
    std::string data_str = py::extract< std::string >(o.attr("data"));
    //pc->data = std::vector<uint8_t>(data_str.c_str(), data_str.c_str()+data_str.length());
    pc->data.insert(pc->data.end(), data_str.c_str(), data_str.c_str()+data_str.length());
    pc->is_dense = py::extract<uint32_t>(o.attr("is_dense"));
  }

  static PyObject * convert(const sensor_msgs::PointCloud2& pc) {
    namespace py = boost::python;
    //return py::incref(py::object( s.data().ptr() ));
    py::object module = py::import("sensor_msgs.msg._PointCloud2");
    py::object MsgType = module.attr("PointCloud2");
    py::object msg = MsgType();
    msg.attr("header") = pc.header;
    msg.attr("height") = pc.height;
    msg.attr("width") = pc.width;
    // create empty python list
    py::list field_lst = py::extract<py::list>(msg.attr("fields"));
    //field_lst.append(pc.fields[0]);
    //std::cerr << "py::len(field_lst) = " << py::len(field_lst) << std::endl;
    //std::cerr << "pc.fields.size() = " << pc.fields.size() << std::endl;
    for ( size_t i=0; i<pc.fields.size(); ++i ) {
      const sensor_msgs::PointField& pf(pc.fields[i]);
      //std::cerr << "pf.name = " << pf.name << std::endl;
      //py::object opf(pf);
      field_lst.append(pf);
      //std::cerr << "bla1" << std::endl;
    }
    //std::cerr << "py::len(field_lst) = " << py::len(field_lst) << std::endl;
    //std::cerr << "bla2" << std::endl;
    msg.attr("fields") = field_lst;
    msg.attr("is_bigendian") = pc.is_bigendian;
    msg.attr("point_step") = pc.point_step;
    msg.attr("row_step") = pc.row_step;
    std::string data_str(pc.data.begin(), pc.data.end());
    msg.attr("data") = data_str;
    msg.attr("is_dense") = pc.is_dense;
    //std::cerr << "bla" << std::endl;
    return py::incref(msg.ptr());
  }

  static void RegisterConverter() {
    namespace py = boost::python;
    //py::to_python_converter<sensor_msgs::PointCloud2,
    //SensorMsgsPointCloud2>();
    //std::cerr << "registering cpp pc from python pc converter" << std::endl;
    py::to_python_converter< sensor_msgs::PointCloud2, SensorMsgsPointCloud2 >();
    py::converter::registry::push_back(
        &SensorMsgsPointCloud2::convertible,
        &SensorMsgsPointCloud2::construct,
        py::type_id< sensor_msgs::PointCloud2 >());
  }

};

struct SensorMsgsImage {

  static void * convertible(PyObject * obj_ptr) {
    return ConvertibleRosMessage(obj_ptr, "sensor_msgs/Image");
  }

  static void construct(PyObject * obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data) {
    namespace py = boost::python;
    typedef py::converter::rvalue_from_python_storage< sensor_msgs::Image > StorageT;
    void* storage = reinterpret_cast< StorageT* >(data)->storage.bytes;
    new (storage) sensor_msgs::Image; // placement new
    data->convertible = storage;

    sensor_msgs::Image* img = static_cast< sensor_msgs::Image* >(storage);
    // get borrowed reference
    py::handle<> handle(py::borrowed(obj_ptr));
    py::object o(handle);
    img->header = py::extract<std_msgs::Header>(o.attr("header"));
    img->height = py::extract<uint32_t>(o.attr("height"));
    img->width = py::extract<uint32_t>(o.attr("width"));
    img->is_bigendian = py::extract<bool>(o.attr("is_bigendian"));
    img->step = py::extract<uint32_t>(o.attr("step"));
    std::string data_str = py::extract< std::string >(o.attr("data"));
    //img->data = std::vector<uint8_t>(data_str.c_str(), data_str.c_str()+data_str.length());
    img->data.insert(img->data.end(), data_str.c_str(), data_str.c_str()+data_str.length());
  }

  static PyObject * convert(const sensor_msgs::Image& img) {
    namespace py = boost::python;
    py::object module = py::import("sensor_msgs.msg._Image");
    py::object MsgType = module.attr("Image");
    py::object msg = MsgType();
    msg.attr("header") = img.header;
    msg.attr("height") = img.height;
    msg.attr("width") = img.width;
    // create empty python list
    msg.attr("is_bigendian") = img.is_bigendian;
    msg.attr("step") = img.step;
    std::string data_str(img.data.begin(), img.data.end());
    msg.attr("data") = data_str;
    return py::incref(msg.ptr());
  }

  static void RegisterConverter() {
    namespace py = boost::python;
    py::to_python_converter< sensor_msgs::Image, SensorMsgsImage >();
    py::converter::registry::push_back(
        &SensorMsgsImage::convertible,
        &SensorMsgsImage::construct,
        py::type_id< sensor_msgs::Image >());
  }
};


struct SensorMsgsCameraInfo {

  static void * convertible(PyObject * obj_ptr) {
    return ConvertibleRosMessage(obj_ptr, "sensor_msgs/CameraInfo");
  }

  static void construct(PyObject * obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data) {
    namespace py = boost::python;
    typedef py::converter::rvalue_from_python_storage< sensor_msgs::CameraInfo > StorageT;
    void* storage = reinterpret_cast< StorageT* >(data)->storage.bytes;
    new (storage) sensor_msgs::CameraInfo;
    data->convertible = storage;
    sensor_msgs::CameraInfo* msg = static_cast< sensor_msgs::CameraInfo* >(storage);
    py::handle<> handle(py::borrowed(obj_ptr));
    py::object o(handle);

    msg->height = py::extract< uint32_t >(o.attr("height"));
    msg->width = py::extract< uint32_t >(o.attr("width"));
    msg->distortion_model = py::extract< std::string >(o.attr("distortion_model"));
    // TODO tuple or list?
    //typedef py::tuple seq_type;
    //typedef py::list seq_type;
    typedef py::object seq_type;
    seq_type D_lst = py::extract<seq_type>(o.attr("D"));
    for (int i=0; i < py::len(D_lst); ++i) {
      double di(py::extract<double>(D_lst[i]));
      msg->D.push_back(di);
    }
    seq_type K_lst = py::extract<seq_type>(o.attr("K"));
    for (int i=0; i < py::len(K_lst); ++i) {
      double ki(py::extract<double>(K_lst[i]));
      msg->K[i] = ki;
    }
    seq_type R_lst = py::extract<seq_type>(o.attr("R"));
    for (int i=0; i < py::len(K_lst); ++i) {
      double Ri(py::extract<double>(K_lst[i]));
      msg->R[i] = Ri;
    }
    seq_type P_lst = py::extract<seq_type>(o.attr("P"));
    for (int i=0; i < py::len(P_lst); ++i) {
      double Pi(py::extract<double>(P_lst[i]));
      msg->P[i] = Pi;
    }
    msg->header = py::extract<std_msgs::Header>(o.attr("header"));
    // TODO roi, binning
  }

  static PyObject * convert(const sensor_msgs::CameraInfo & ci) {
    namespace py = boost::python;
    //return py::incref(py::object( s.data().ptr() ));
    py::object module = py::import("sensor_msgs.msg._CameraInfo");
    py::object MsgType = module.attr("CameraInfo");
    py::object msg = MsgType();
    msg.attr("header") = ci.header;
    msg.attr("height") = ci.height;
    msg.attr("width") = ci.width;
    msg.attr("distortion_model") = ci.distortion_model;
    py::list D_lst = py::extract<py::list>(msg.attr("D"));
    for ( size_t i=0; i<ci.D.size(); ++i ) {
      D_lst.append(ci.D[i]);
    }
    py::list K_lst = py::extract<py::list>(msg.attr("K"));
    for ( size_t i=0; i<9; ++i ) {
      K_lst.append(ci.K[i]);
    }
    py::list R_lst = py::extract<py::list>(msg.attr("R"));
    for ( size_t i=0; i<9; ++i ) {
      R_lst.append(ci.R[i]);
    }
    py::list P_lst = py::extract<py::list>(msg.attr("P"));
    for ( size_t i=0; i<12; ++i ) {
      R_lst.append(ci.P[i]);
    }
    // TODO roi,binning
    return py::incref(msg.ptr());
  }

  static void RegisterConverter() {
    namespace py = boost::python;
    py::to_python_converter< sensor_msgs::CameraInfo, SensorMsgsCameraInfo >();
    py::converter::registry::push_back(
        &SensorMsgsCameraInfo::convertible,
        &SensorMsgsCameraInfo::construct,
        py::type_id< sensor_msgs::CameraInfo >());
  }
};

void print_cam_info(const sensor_msgs::CameraInfo& ci) {
  std::cout << ci.distortion_model << "\n";
  std::cout << ci.K[8] << " " << ci.R[8] << " " << ci.P[8] << "\n";
}
#endif

// just a sanity check
void print_centroid(const sensor_msgs::PointCloud2& cloud) {
  double cx = 0., cy = 0., cz = 0.;
  for (size_t i=0; i < cloud.width; ++i) {
    float x = *reinterpret_cast<const float*>(&cloud.data[i*cloud.point_step]);
    float y = *reinterpret_cast<const float*>(&cloud.data[i*cloud.point_step + sizeof(float)]);
    float z = *reinterpret_cast<const float*>(&cloud.data[i*cloud.point_step + 2*sizeof(float)]);
    cx += x; cy += y; cz += z;
  }
  std::cerr << "cloud.width = " << cloud.width << std::endl;
  cx /= cloud.width;
  cy /= cloud.width;
  cz /= cloud.width;
  std::cout << "centroid = [" << cx << " " << cy << " " << cz << "]" << std::endl;
}

sensor_msgs::PointCloud2 make_pc2(int rows) {
  sensor_msgs::PointCloud2 pc;
  pc.width = rows;
  pc.height = 1;
  sensor_msgs::PointField pfx;
  pfx.name =  "x";
  pfx.offset = 0;
  pfx.datatype = sensor_msgs::PointField::FLOAT32;
  pfx.count = 1;
  pc.fields.push_back(pfx);
  pc.point_step = sizeof(float);
  float data[rows];
  for (int i=0; i < rows; ++i) { data[i] = 28.0; }
  pc.data.insert(pc.data.end(),
                 reinterpret_cast<uint8_t *>(data),
                 reinterpret_cast<uint8_t *>(data+rows));
  return pc;
}


ros::Time make_time() {
  ros::Time ts;
  ts.sec = 28;
  ts.nsec = 999;
  return ts;
}

void print_time(const ros::Time& ts) {
  std::cerr << "ts.sec = " << ts.sec << std::endl;
  std::cerr << "ts.nsec = " << ts.nsec << std::endl;
}

ros::Time increment_ts( const ros::Time& ts ) {
  std::cerr << "ts.sec = " << ts.sec << ", ts.nsec = " << ts.nsec << std::endl;
  ros::Time newts(ts.sec + 1, ts.nsec + 1);
  std::cerr << "newts.sec = " << newts.sec << ", newts.nsec = " << newts.nsec << std::endl;
  return newts;
}

std_msgs::Header make_header(int seq) {
  std_msgs::Header out;
  out.seq = seq;
  return out;
}

void print_header_seq(std_msgs::Header& header) {
  std::cerr << "header.seq = " << header.seq << std::endl;
}


PYBIND11_PLUGIN(libpymsg) {
  namespace py = pybind11;

#if 0
  py::def("print_cam_info", &print_cam_info);
#endif

  py::module m("libpymsg", "libpymsg plugin");
  m.def("print_centroid", &print_centroid);
  m.def("make_pc2", &make_pc2);
  m.def("print_time", &print_time, "print time");
  m.def("make_time", &make_time, "make time");

  m.def("make_header", &make_header);
  m.def("increment_ts", &increment_ts);
  m.def("print_header_seq", &print_header_seq);
  return m.ptr();
}
