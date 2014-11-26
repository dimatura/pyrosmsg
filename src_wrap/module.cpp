#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

static inline
void * ConvertibleRosMessage(PyObject * obj_ptr, const std::string& name) {
    namespace py = boost::python;
    if (!PyObject_HasAttrString(obj_ptr, "_type")) { return NULL; }
    py::handle<> handle(py::borrowed(obj_ptr));
    py::object o(handle);
    std::string msg_type(py::extract<std::string>(o.attr("_type")));
    if (msg_type != name) { return NULL; }
    return obj_ptr;
}

// TODO duration is pretty much same as time
struct RosTime {

  static void * convertible(PyObject * obj_ptr) {
    namespace py = boost::python;
    // TODO: do a check()?
    if (!PyObject_HasAttrString(obj_ptr, "secs")) { return NULL; }
    if (!PyObject_HasAttrString(obj_ptr, "nsecs")) { return NULL; }
    return obj_ptr;
  }

  static void construct(PyObject * obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data) {
    namespace py = boost::python;
    typedef py::converter::rvalue_from_python_storage< ros::Time > StorageT;
    void* storage = reinterpret_cast< StorageT* >(data)->storage.bytes;
    new (storage) ros::Time; // placement new
    data->convertible = storage;
    ros::Time* t = static_cast< ros::Time* >(storage);
    py::handle<> handle(py::borrowed(obj_ptr));
    py::object o(handle);
    t->sec = py::extract<uint32_t>(o.attr("secs"));
    t->nsec = py::extract<uint32_t>(o.attr("nsecs"));
  }

  // to-python converter
  static PyObject * convert(ros::Time const& ts) {
    namespace py = boost::python;
    //return py::incref(py::object( s.data().ptr() ));
    py::object rospy = py::import("rospy");
    //int info = py::extract<int>(rospy.attr("INFO"));
    py::object TimeType = rospy.attr("Time");
    py::object pyts = TimeType();
    pyts.attr("secs") = ts.sec;
    pyts.attr("nsecs") = ts.nsec;
    return py::incref(pyts.ptr());
  }

  static void RegisterConverter() {
    namespace py = boost::python;
    py::to_python_converter< ros::Time, RosTime >();
    py::converter::registry::push_back(
        &RosTime::convertible,
        &RosTime::construct,
        py::type_id< ros::Time >());
  }

};

struct StdMsgsHeader {

  static void * convertible(PyObject * obj_ptr) {
    return ConvertibleRosMessage(obj_ptr, "std_msgs/Header");
  }

  static void construct(PyObject * obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data) {
    namespace py = boost::python;
    typedef py::converter::rvalue_from_python_storage< std_msgs::Header > StorageT;
    void* storage = reinterpret_cast< StorageT* >(data)->storage.bytes;
    new (storage) std_msgs::Header; // placement new
    data->convertible = storage;
    std_msgs::Header* msg = static_cast< std_msgs::Header* >(storage);
    py::handle<> handle(py::borrowed(obj_ptr));
    py::object o(handle);
    msg->seq = py::extract<uint32_t>(o.attr("seq"));
    msg->stamp = py::extract<ros::Time>(o.attr("stamp"));
    msg->frame_id = py::extract<std::string>(o.attr("frame_id"));
  }

  static PyObject * convert(const std_msgs::Header& header) {
    namespace py = boost::python;
    //return py::incref(py::object( s.data().ptr() ));
    py::object module = py::import("std_msgs.msg._Header");
    py::object MsgType = module.attr("Header");
    py::object msg = MsgType();
    msg.attr("seq") = header.seq;
    msg.attr("stamp") = header.stamp;
    msg.attr("frame_id") = header.frame_id;
    return py::incref(msg.ptr());
  }

  static void RegisterConverter() {
    namespace py = boost::python;
    py::to_python_converter< std_msgs::Header, StdMsgsHeader >();
    py::converter::registry::push_back(
        &StdMsgsHeader::convertible,
        &StdMsgsHeader::construct,
        py::type_id< std_msgs::Header >());
  }

};

struct GeometryMsgsPoint {

  static void * convertible(PyObject * obj_ptr) {
    return ConvertibleRosMessage(obj_ptr, "geometry_msgs/Point");
  }

  static void construct(PyObject * obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data) {
    namespace py = boost::python;
    typedef py::converter::rvalue_from_python_storage< geometry_msgs::Point > StorageT;
    void* storage = reinterpret_cast< StorageT* >(data)->storage.bytes;
    new (storage) geometry_msgs::Point; // placement new
    data->convertible = storage;
    geometry_msgs::Point* msg = static_cast< geometry_msgs::Point* >(storage);
    py::handle<> handle(py::borrowed(obj_ptr));
    py::object o(handle);
    msg->x = py::extract<double>(o.attr("x"));
    msg->y = py::extract<double>(o.attr("y"));
    msg->z = py::extract<double>(o.attr("z"));
  }

  static PyObject * convert(const geometry_msgs::Point& pt) {
    namespace py = boost::python;
    //return py::incref(py::object( s.data().ptr() ));
    py::object module = py::import("geometry_msgs.msg._Point");
    py::object MsgType = module.attr("Point");
    py::object msg = MsgType();
    msg.attr("x") = pt.x;
    msg.attr("y") = pt.y;
    msg.attr("z") = pt.z;
    return py::incref(msg.ptr());
  }

  static void RegisterConverter() {
    namespace py = boost::python;
    py::to_python_converter< geometry_msgs::Point, GeometryMsgsPoint >();
    py::converter::registry::push_back(
        &GeometryMsgsPoint::convertible,
        &GeometryMsgsPoint::construct,
        py::type_id< geometry_msgs::Point >());
  }

};

struct GeometryMsgsQuaternion {

  static void * convertible(PyObject * obj_ptr) {
    return ConvertibleRosMessage(obj_ptr, "geometry_msgs/Quaternion");
  }

  static void construct(PyObject * obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data) {
    namespace py = boost::python;
    typedef py::converter::rvalue_from_python_storage< geometry_msgs::Quaternion > StorageT;
    void* storage = reinterpret_cast< StorageT* >(data)->storage.bytes;
    new (storage) geometry_msgs::Quaternion; // placement new
    data->convertible = storage;
    geometry_msgs::Quaternion* msg = static_cast< geometry_msgs::Quaternion* >(storage);
    py::handle<> handle(py::borrowed(obj_ptr));
    py::object o(handle);
    msg->x = py::extract<double>(o.attr("x"));
    msg->y = py::extract<double>(o.attr("y"));
    msg->z = py::extract<double>(o.attr("z"));
    msg->w = py::extract<double>(o.attr("w"));
  }

  static PyObject * convert(const geometry_msgs::Quaternion& q) {
    namespace py = boost::python;
    //return py::incref(py::object( s.data().ptr() ));
    py::object module = py::import("geometry_msgs.msg._Quaternion");
    py::object MsgType = module.attr("Quaternion");
    py::object msg = MsgType();
    msg.attr("x") = q.x;
    msg.attr("y") = q.y;
    msg.attr("z") = q.z;
    msg.attr("z") = q.w;
    return py::incref(msg.ptr());
  }

  static void RegisterConverter() {
    namespace py = boost::python;
    py::to_python_converter< geometry_msgs::Quaternion, GeometryMsgsQuaternion >();
    py::converter::registry::push_back(
        &GeometryMsgsQuaternion::convertible,
        &GeometryMsgsQuaternion::construct,
        py::type_id< geometry_msgs::Quaternion >());
  }

};

struct SensorMsgsPointField {

  static void * convertible(PyObject * obj_ptr) {
    return ConvertibleRosMessage(obj_ptr, "sensor_msgs/PointField");
  }

  static void construct(PyObject * obj_ptr,
                        boost::python::converter::rvalue_from_python_stage1_data* data) {
    namespace py = boost::python;
    typedef py::converter::rvalue_from_python_storage< sensor_msgs::PointField > StorageT;
    void* storage = reinterpret_cast< StorageT* >(data)->storage.bytes;
    new (storage) sensor_msgs::PointField;
    data->convertible = storage;
    sensor_msgs::PointField* msg = static_cast< sensor_msgs::PointField* >(storage);
    py::handle<> handle(py::borrowed(obj_ptr));
    py::object o(handle);
    msg->name = py::extract< std::string >(o.attr("name"));
    msg->offset = py::extract< uint32_t >(o.attr("offset"));
    msg->datatype = py::extract< uint8_t >(o.attr("datatype"));
    msg->count = py::extract< uint32_t >(o.attr("count"));
  }

  static PyObject * convert(sensor_msgs::PointField const & pf) {
    namespace py = boost::python;
    //return py::incref(py::object( s.data().ptr() ));
    py::object module = py::import("sensor_msgs.msg._PointField");
    py::object MsgType = module.attr("PointField");
    py::object msg = MsgType();
    msg.attr("name") = pf.name;
    msg.attr("offset") = pf.offset;
    msg.attr("datatype") = pf.datatype;
    msg.attr("count") = pf.count;
    return py::incref(msg.ptr());
  }

  static void RegisterConverter() {
    namespace py = boost::python;
    py::to_python_converter< sensor_msgs::PointField, SensorMsgsPointField >();
    py::converter::registry::push_back(
        &SensorMsgsPointField::convertible,
        &SensorMsgsPointField::construct,
        py::type_id< sensor_msgs::PointField >());
  }
};

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
    for (size_t i=0; i < py::len(field_lst); ++i) {
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
    py::list field_lst = py::extract<py::list>(msg.attr("fields"));
    std::cerr << "py::len(field_lst) = " << py::len(field_lst) << std::endl;
    std::cerr << "pc.fields.size() = " << pc.fields.size() << std::endl;
    field_lst.append(pc.fields[0]);
    std::cerr << "py::len(field_lst) = " << py::len(field_lst) << std::endl;
#if 0
    for ( size_t i=0; pc.fields.size(); ++i ) {
      sensor_msgs::PointField pf(pc.fields[i]);
      std::cerr << "pf.name = " << pf.name << std::endl;
      //py::object opf(pf);
      field_lst.append(pf);
      std::cerr << "bla1" << std::endl; }
#endif
    std::cerr << "bla2" << std::endl;
    msg.attr("is_bigendian") = pc.is_bigendian;
    msg.attr("point_step") = pc.point_step;
    msg.attr("row_step") = pc.row_step;
    std::string data_str(pc.data.begin(), pc.data.end());
    msg.attr("data") = data_str;
    msg.attr("is_dense") = pc.is_dense;
    std::cerr << "bla" << std::endl;
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


// just a sanity check
void print_centroid(const sensor_msgs::PointCloud2& cloud) {
  double cx = 0., cy = 0., cz = 0.;
  for (size_t i=0; i < cloud.width; ++i) {
    cx += ( *reinterpret_cast<const float*>(&cloud.data[i*cloud.point_step]) );
    cy += ( *reinterpret_cast<const float*>(&cloud.data[i*cloud.point_step + sizeof(float)]) );
    cz += ( *reinterpret_cast<const float*>(&cloud.data[i*cloud.point_step + 2*sizeof(float)]) );
  }
  cx /= cloud.width;
  cy /= cloud.width;
  cz /= cloud.width;
  std::cout << "centroid = [" << cx << " " << cy << " " << cz << "]" << std::endl;
}

ros::Time increment_ts( const ros::Time& ts ) {
  std::cerr << "ts.sec = " << ts.sec << ", ts.nsec = " << ts.nsec << std::endl;
  ros::Time newts(ts.sec + 1, ts.nsec + 1);
  std::cerr << "newts.sec = " << newts.sec << ", newts.nsec = " << newts.nsec << std::endl;
  return newts;
}

std_msgs::Header make_header( int seq ) {
  std_msgs::Header out;
  out.seq = seq;
  return out;
}

sensor_msgs::PointCloud2 make_pc2( int rows ) {
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
  pc.data.insert(pc.data.end(), reinterpret_cast<uint8_t *>(data), reinterpret_cast<uint8_t *>(data+rows));
  return pc;
}

BOOST_PYTHON_MODULE(libpymsg) {
  namespace py = boost::python;

  RosTime::RegisterConverter();
  StdMsgsHeader::RegisterConverter();
  SensorMsgsPointField::RegisterConverter();
  SensorMsgsPointCloud2::RegisterConverter();
  py::def("print_centroid", &print_centroid);
  py::def("make_header", &make_header);
  py::def("make_pc2", &make_pc2);
  py::def("increment_ts", &increment_ts);
}
