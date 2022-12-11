# Useful commands

To launch localizer 

```
ros2 launch create3_localization apriltag_launch.py

ros2 launch create3_localization create3_joystick.py
```

## dependencies 

```
sudo apt install ros-galactic-usb-cam -y

sudo apt install ros-galactic-tf-transformations
sudo pip3 install transforms3d

```

## debug 

Qos
```
ros2 topic info /odom --verbose

```