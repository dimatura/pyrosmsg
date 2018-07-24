# pyrosmsg

Daniel Maturana 2018, dimatura@gmail.com

# What

Bidirectional pybind11 converters for a few ROS messages.

The converters will accept Python messages and convert them to C++ messages or viceversa.

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

# LICENSE

BSD.
