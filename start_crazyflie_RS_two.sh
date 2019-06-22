echo "Running external_position_two_RS.launch"
gnome-terminal -e "roslaunch crazyflie_demo external_position_two_RS.launch"
sleep 10
echo "Running findMyBallPublish.py"
rosrun crazyflie_demo findMyBallPublish.py