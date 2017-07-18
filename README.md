ROS-IGTL-Bridge
===============

Author:Tobias Frank

This ROS-Node provides an OpenIGTLink bridge to exchange data with ROS. 
It supports sending and receiving Transformations, Images, Strings, 
PolyData, Points and Pointclouds. 
For further information regarding the OpenIGTLink protocol please see:
- http://openigtlink.org/


----------------------------------------------------------------------------------------------------------------------------------------

Build Instruction
-----------------

[Install OpenIGTLink]

Please refer to http://openigtlink.org/. In the following instruction, we assume that the build directory for the OpenIGTLink library is located at: ~/igtl/OpenIGTLink-build

[Install ROS] (http://wiki.ros.org)
and follow the standard [ROS instructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to create your ROS workspace if necessary.

    $ mkdir -p ~/catkin_ws/src
    $ catkin_make
    $ source devel/setup.bash
	
Then download the ros_igtl_bridge package from GitHub:
	
    $ git clone https://github.com/openigtlink/ROS-IGTL-Bridge

and execute catkin_make in your workspace directory:

    $ cd ~/catkin_ws/
    $ catkin_make --cmake-args -DOpenIGTLink_DIR:PATH=~/igtl/OpenIGTLink-build


Launch the test procedure for communication with [3D Slicer] (https://www.slicer.org/):

    $ roslaunch ros_igtl_bridge test.launch
    
To simply run the bridge without sending test sample data, type:

    $ roslaunch ros_igtl_bridge bridge.launch

It is possible to edit the launch files and set your IP & Port in the file. Run the node as server or client by adjusting the parameter RIB_type.
Open the file and uncomment the lines:

	  $ <!--param name="RIB_server_ip" value="111.111.111.111" type="str"/-->
	  $ <!--param name="RIB_port" value="18944" type="int"/-->
      $ <!--param name="RIB_type" value="client" type="str"/-->

The node can be run as server or client. If you executed the test procedure, the node will send
a "ROS_IGTL_Test_Transform" with random translation, a random "ROS_IGTL_Test_Point", 
a random "ROS_IGTL_Test_Pointcloud" including 20 points, a "ROS_IGTL_Test_String" and a "ROS_IGTL_Test_PolyData", which is a rendered model 
of the 3D Slicer MRHead sample data. Any data received from 3D Slicer is published to a ROS topic by the bridge node and displayed by the test node.


References
----------
1. Frank T, Krieger A, Leonard S, Patel NA, Tokuda J. ROS-IGTL-Bridge: an open network interface for image-guided therapy using the ROS environment. Int J Comput Assist Radiol Surg. 2017 May 31. doi: 10.1007/s11548-017-1618-1. PubMed PMID: [28567563](https://www.ncbi.nlm.nih.gov/pubmed/?term=28567563).



    
    


