# Long Horizon Robot Arm

TODOs:
- [ ] calibrate servo home positions and limits (most gruesome part of tomorrow) (should it be vertical? how do we allot it 270 range?)
- [ ] setup pyrealsense2 from source then add setup instructions (wget) in this readme
- [ ] Tune max_vel and max_accel of servos
- [ ] Add in parameter to Servo class for custom max_vel and max_accel fields (to be tweaked in ServoController.py). Reason? Lower joints may need lower maxes than later joints
- [ ] may need to add in additional delays, specificlaly worried about move_in_synced where motion profiles are reset after move_to_pos

[pybullet docs](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0#heading=h.2ye70wns7io3)


## Setup Jetson Nano
1. Download SD Card Image for "Jetson Nano Developer Kit" from https://developer.nvidia.com/embedded/jetpack-sdk-46
2. Flash ZIP Image onto Micro SD card using [Etcher](https://etcher.balena.io/)
3. Plug in peripherals, insert SD card, and plug in 5V 3A or 4A power supply. 
4. Boot and create/sign in with username: "nvidia"
5. Plug in Ethernet connection to computer
6. If using Mac, rearrange service order to put WiFi above LAN then enable "Internet Sharing"
7. Run these commands in the Ubuntu terminal:
```
sudo apt update
sudo apt install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
ifconfig
```
8. Copy inet IP address (should be similar to 192.168.2.2). This address will be used to SSH

### Setup GPIO Permissions
Next, we need to enable GPIO I/O
```
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER
sudo cp /usr/local/lib/python*/site-packages/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```
> You'll need to log out and log back in for the group changes to take effect

## Setup on Laptop VSCode
```
ssh nvidia@192.168.2.2
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```