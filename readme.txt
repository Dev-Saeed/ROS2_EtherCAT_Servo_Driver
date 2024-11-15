Before building this package you have to install SOEM library:

go to home directory then:

    git clone https://github.com/OpenEtherCATsociety/SOEM.git
    cd SOEM
    mkdir build
    cmake ..
    make
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
    sudo make install


---------------------------------------------------------------------------

To give Permissions to the node:
    1) If you do not want to use password again:

        sudo visudo
        your_username ALL=(ALL) NOPASSWD:ALL

    2)) You have to grant specific capabilities to a ROS2 node executable,
        allowing it to perform network administration and raw socket operations 
        To do that:

       sudo setcap cap_net_admin,cap_net_raw=eip ~/path/to/the/node/executable
       ex:
       sudo setcap cap_net_admin,cap_net_raw=eip ~/HardwareInterface_ws/install/ethercat_servo_control/lib/ethercat_servo_control/motor_control_node

       Explanation of Permissions
            - cap_net_admin: Allows network administrative actions (e.g., configuring network interfaces).
            - cap_net_raw: Grants permission to use raw network sockets, needed for protocols like EtherCAT.
            - =eip: Ensures these capabilities are effective (e), inheritable (i), and permitted (p) for the executable.
    
    3))) You may have some error when you will run the node after that. So you will run this cmd just one time:

       echo "/opt/ros/humble/lib" | sudo tee /etc/ld.so.conf.d/ros2.conf
       echo "$HOME/Portable-ros2_ws/Servant_ws/install/servo_msgs/lib" | sudo tee /etc/ld.so.conf.d/servo_msgs.conf
       sudo ldconfig

       By adding the ROS 2 library path to the system’s library configuration and updating the cache, 
       you no longer need to set LD_LIBRARY_PATH manually or within scripts, as the system automatically 
       searches in /opt/ros/humble/lib for libraries like librclcpp.so. 

       using setcap causes the system to ignore LD_LIBRARY_PATH for security reasons, which then requires 
       you to configure the library paths in a way that doesn’t rely on environment variables.

       "I dont know how it take me all the week to figure out this but its ok"

       
