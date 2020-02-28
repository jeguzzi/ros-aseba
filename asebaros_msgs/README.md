[asebaros] is a bridge between [Aseba] and [ROS].
It allows to load source code, inspect the network structure, read and write variables, and send and receive events from [ROS].
It maps the [Aseba] named events to [ROS] topics in a dynamic way, when loading source code.
[asebaros] will compile on platforms supported by [ROS].

Compilation
-----------

Get the [ethzasl_aseba] stack:
	git clone git://github.com/ethz-asl/ros-aseba.git ethzasl_aseba

Make sure that it is included in your `ROS_PACKAGE_PATH`:
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ethzasl_aseba

Then, to install the required development libraries and to compile [asebaros], execute:
	rosmake --rosdep-install asebaros

Information about the usage of [asebaros] is available on the [ROS] wiki at [http://www.ros.org/wiki/asebaros](http://www.ros.org/wiki/asebaros).

[Aseba]: http://aseba.wikidot.com
[ROS]: http://www.ros.org
[ethzasl_aseba]: http://www.ros.org/wiki/ethzasl_aseba
[asebaros]: http://www.ros.org/wiki/asebaros
