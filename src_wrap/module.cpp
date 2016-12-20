/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/

#include <pymsg/converters.hpp>

void print_cam_info(const sensor_msgs::CameraInfo& ci) {
  std::cout << "distortion model\n";
  std::cout << ci.distortion_model << "\n";
  std::cout << "K R P [8]\n";
  std::cout << ci.K[8] << " " << ci.R[8] << " " << ci.P[8] << "\n";
  std::cout << "\n";
}

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

void print_img(const sensor_msgs::Image& img) {
  std::cerr << "img.width = " << img.width << std::endl;
  std::cerr << "img.height = " << img.height << std::endl;
  std::cerr << "img.step = " << img.step << std::endl;
  std::cerr << "[";
  for (int i=0; i < img.width*img.height; ++i) {
    std::cerr << (int)img.data[i] << ", ";
  }
  std::cerr << "]\n";
}

sensor_msgs::Image make_img(int width, int height) {
  sensor_msgs::Image msg;
  msg.width = width;
  msg.height = height;
  msg.encoding = "8UC1";
  msg.step = width;
  for (int i=0; i < height; ++i) {
    for (int j=0; j < width; ++j) {
      msg.data.push_back(width*i + j);
    }
  }
  return msg;
}


PYBIND11_PLUGIN(libpymsg) {
  namespace py = pybind11;

  py::module m("libpymsg", "libpymsg plugin");

  m.def("print_cam_info", &print_cam_info);

  m.def("print_centroid", &print_centroid);
  m.def("make_pc2", &make_pc2);
  m.def("print_time", &print_time, "print time");
  m.def("make_time", &make_time, "make time");

  m.def("make_header", &make_header);
  m.def("increment_ts", &increment_ts);
  m.def("print_header_seq", &print_header_seq);
  m.def("print_img", &print_img);
  m.def("make_img", &make_img);
  return m.ptr();
}
