#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

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

struct CppTimeFromPyTime {

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

  static void RegisterConverter() {
    namespace py = boost::python;
    py::converter::registry::push_back(
        &CppTimeFromPyTime::convertible,
        &CppTimeFromPyTime::construct,
        py::type_id< ros::Time >());
  }
};

struct CppHeaderFromPyHeader {

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

  static void RegisterConverter() {
    namespace py = boost::python;
    py::converter::registry::push_back(
        &CppHeaderFromPyHeader::convertible,
        &CppHeaderFromPyHeader::construct,
        py::type_id< std_msgs::Header >());
  }

};

struct CppPointFieldFromPyPointField {

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

  static void RegisterConverter() {
    namespace py = boost::python;
    py::converter::registry::push_back(
        &CppPointFieldFromPyPointField::convertible,
        &CppPointFieldFromPyPointField::construct,
        py::type_id< sensor_msgs::PointField >());
  }
};

struct CppPointCloud2FromPyPointCloud2 {

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

  static void RegisterConverter() {
    namespace py = boost::python;
    //py::to_python_converter<sensor_msgs::PointCloud2,
    //CppPointCloud2FromPyPointCloud2>();
    //std::cerr << "registering cpp pc from python pc converter" << std::endl;
    py::converter::registry::push_back(
        &CppPointCloud2FromPyPointCloud2::convertible,
        &CppPointCloud2FromPyPointCloud2::construct,
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

BOOST_PYTHON_MODULE(libpymsg) {
  using namespace boost::python;
  CppTimeFromPyTime::RegisterConverter();
  CppHeaderFromPyHeader::RegisterConverter();
  CppPointFieldFromPyPointField::RegisterConverter();
  CppPointCloud2FromPyPointCloud2::RegisterConverter();
  def("print_centroid", &print_centroid);
}