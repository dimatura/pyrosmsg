# pyrosmsg

Daniel Maturana 2018, dimatura@gmail.com

# What

Bidirectional pybind11 converters for a few ROS messages.

The converters will accept Python messages and convert them to C++ messages or viceversa.

This library only really makes sense for C++ code wrapped with pybind11 for use with Python.

# Why

The two main languages ROS supports are C++ and Python. For any given message type, ROS
will generate Python and C++ versions of that message that are interoperable when sending them
over the network -- i.e., it's fine to send messages from python nodes to C++ nodes or viceversa.
However, if we have a Python node that uses wrapped C++ code (something I do a lot), an issue is that
(from the computer's point of view) the Python and C++ messages are actually completely unrelated objects.
So e.g., to use a Python PointCloud2 message in C++ code or viceversa, conversion is needed. 

One option is to use the built-in serialization capabilities of ROS - e.g., if we have a Python message,
we can serialize it to a string, pass it as a string to the C++ code (trivial with `pybind11`), and deserialize 
it there. This would be the equivalent of sending the a message over the network, except we're just copying
strings within the same process. This is actually implemented in this package (`serialization.h`). However,
I found that even without the network overhead, this method is fairly slow.

Another option is to use the converters in this package.
It uses `pybind11` to make this conversion as transparent as possible from both the C++ and Python
ends -- all supported message types are converted automatically in each direction.
I should note that whenever a message crosses the Python/C++ barrier with this method, the data is copied --
there is no memory shared between the Python and C++ messages. While this will incur in some overhead,
it is far less than the (de)serialization method described above.

One drawback is that the converters are (currently) written manually. It's a fairly mechanical process,
and in theory it could be automated, but I have not done so yet. I would welcome converters for other message types.

# How

This is a catkin ROS package, and as such should be installed in a catkin workspace like any other.

To use from the C++ side, simply `#include <pyrosmsg/pyrosmsg.h>`. This will register `pybind11` converters that
can be used to transparently accept and return the support ROS messages in your code.

To use from the Python side, simply `import pyrosmsg` before using wrapped functions.

So in your C++ code,

```cpp
// this code is in a pybind11-wrapped module called, say, 'mycloudlib'

#include <sensor_msgs/PointCloud2.h>
#include <pyrosmsg/pyrosmsg.h>

// etc

void process_pointcloud(const sensor_msgs::PointCloud2& cloud) {
    // do fast C++ stuff with your point cloud here
}

```

And in your python code, presumably a ROS node,

```python
from sensor_msgs.msg import PointCloud2
import pyrosmsg
import mycloudlib

def my_pc2_callback(pc2msg):
    # pc2msg is a *python* sensor_msgs.msg.PointCloud2
    # transparently pass the PointCloud2 to your C++ code.
    # under the hood, pyrosmsg will create a C++ copy of the message
    # and pass it to the wrapped mycloudlib function.
    mycloudlib.process_pointcloud(pc2msg)
```


# Currently supported messages

- `ros::Time`
- `std_msgs::Header`
- `geometry_msgs::Point`
- `geometry_msgs::Vector3`
- `geometry_msgs::Quaternion`
- `geometry_msgs::Transform`
- `geometry_msgs::TransformStamped`
- `sensor_msgs::PointField`
- `sensor_msgs::PointCloud2`
- `sensor_msgs::Image`


# Platforms

Only tested with Ubuntu 16.04, ROS Kinetic, and Python 2.7. Would probably work with other Linux and (Indigo and newer) ROS distros.
Python 3, probably not without minor modifications.

# TODO

It would be nice to avoid copying data, if possible. In my own experience copying image and point cloud data within a process has 
never been a bottleneck compared to whatever data processing I perform with the data -- after all, the copy is just
a `memcpy` of a contiguous chunk a memory. But for massive amounts of data unnecessary copying is a drag.
In theory, this should be possible -- the only difficulty is correct memory management across the Python and C++
border.


# LICENSE

BSD, see LICENSE.
