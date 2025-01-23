# Long Horizon Robot Arm

TODOs:
- [ ] setup pyrealsense2 from source then add setup instructions (wget) in this readme
- [ ] Tune max_vel and max_accel of servos
- [ ] Add in parameter to Servo class for custom max_vel and max_accel fields (to be tweaked in ServoController.py). Reason? Lower joints may need lower maxes than later joints
- [ ] may need to add in additional delays, specificlaly worried about move_in_synced where motion profiles are reset after move_to_pos


## Setup Jetson
```
sudo apt update
sudo apt install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
sudo ufw allow ssh
ifconfig
```

## Setup GPIO Permissions
```
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER
sudo cp /usr/local/lib/python*/site-packages/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
> You'll need to log out and log back in for the group changes to take effect

## Setup on Laptop VSCode
```
ssh nvidia@<jetson-ip-from-ifconfig>
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```