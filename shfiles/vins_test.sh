gnome-terminal -x bash -c "roslaunch px4 indoor1.launch" & sleep 6

gnome-terminal -x bash -c "cd ~/DPDrone/catkin_package/Vins && source devel/setup.bash && rosrun vins vins_node src/VINS-Fusion-master/config/px4/px4_indoor1_stereo_imu_config.yaml " & sleep 3

gnome-terminal -x bash -c "cd ~/DPDrone/catkin_package/Vins && source devel/setup.bash && rosrun vins vins_transfer" & sleep 3

gnome-terminal -x bash -c "cd ~/DPDrone/DPdrone_ws && source devel/setup.bash && rosrun communication drone_communication" & sleep 3

gnome-terminal -x bash -c "cd ~/DPDrone/DPdrone_ws && source devel/setup.bash && rosrun control vins_test" & sleep 3
