# ros_tutorials
Tutorials with ROS: Find videos on my channel https://www.youtube.com/channel/UC2aPsByptP3HXobjXgsXQsA?view_as=subscriber

------- Setup the ROS_MASTER_URI -----------
If ROS Master runs on the Raspberry Pi, you need to set 
export ROS_MASTER_URI=http://ubiquityrobot.local:11311

In case you are running ROS Master on your local host, set
export ROS_MASTER_URI=http://localhost:11311

To speedup the process you can edit ~.baschrc with:

function myip()
{
    ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\$
}

source ~/catkin_ws/devel/setup.bash
export ROS_IP=myip()

export ROS_MASTER_URI=http://ubiquityrobot.local:11311
or 
export ROS_MASTER_URI=http://localhost:11311
