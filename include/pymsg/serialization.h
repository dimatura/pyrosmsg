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

#include <boost/python.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include <ros/ros.h>

namespace ca { namespace pyvox {

template <class MsgT>
std::string MessageToString(const MsgT& msg) {
  uint32_t serial_size = ros::serialization::serializationLength(msg);
  // in practice we could directly use a string.c_str() but that is living dangerously
  boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
  ros::serialization::OStream os(obuffer.get(), serial_size);
  ros::serialization::serialize(os, msg);
  // TODO do we need null-termination?
  std::string out(obuffer.get(), obuffer.get() + serial_size);
  return out;
}

template <class MsgT>
void MessageFromString(MsgT& self, const std::string& s) {
  uint32_t serial_size = s.length();
  boost::shared_array<uint8_t> ibuffer(new uint8_t[serial_size]);
  std::copy(s.c_str(), s.c_str()+s.length(), ibuffer.get());
  ros::serialization::IStream istream(ibuffer.get(), serial_size);
  ros::serialization::deserialize(istream, self);
}

} }

#endif /* end of include guard: SERIALIZATION_H_QC30JWRG */
