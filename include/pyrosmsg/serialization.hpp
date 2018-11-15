/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/

#ifndef SERIALIZATION_H_QC30JWRG
#define SERIALIZATION_H_QC30JWRG

#include <memory>

#include <ros/ros.h>

namespace pyrosmsg {

// in python
// buf = StringIO.StringIO()
// msg.serialize(buf)
// s = msg.getvalue()
//
// from string
// msg = sensor_msgs.msg.PointCloud2()
// msg.deserialize(msg_str)

template <class MsgT>
std::string serialize(const MsgT& msg) {
  uint32_t serial_size = ros::serialization::serializationLength(msg);
  // we could directly use a string.c_str(), but not sure if
  // that could have ownership issues
  uint8_t* obuffer = new uint8_t[serial_size];
  ros::serialization::OStream os(obuffer, serial_size);
  ros::serialization::serialize(os, msg);
  // TODO do we need null-termination?
  std::string out(obuffer, obuffer + serial_size);
  delete[] obuffer;
  return out;
}

template <class MsgT>
void deserialize(const std::string s, MsgT& msg) {
  uint32_t serial_size = s.length();
  uint8_t* ibuffer = new uint8_t[serial_size]();
  std::copy(s.c_str(), s.c_str() + s.length(), &ibuffer[0]);
  ros::serialization::IStream istream(&ibuffer[0], serial_size);
  ros::serialization::deserialize(istream, msg);
  delete[] ibuffer;
}
}

#endif /* end of include guard: SERIALIZATION_H_QC30JWRG */
