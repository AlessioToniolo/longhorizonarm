# Long Horizon Robot Arm

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
wget https://github.com/IntelRealSense/librealsense/releases/download/v2.56.3/pyrealsense2-2.56.3.7838-cp310-cp310-manylinux1_x86_64_beta.whl
pip install pyrealsense2-2.56.3.7838-cp310-cp310-manylinux1_x86_64_beta.whl
pip install -r requirements.txt
```