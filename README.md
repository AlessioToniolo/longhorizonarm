# Long Horizon Robot Arm

## Setup on Jetson
```
sudo apt update
sudo apt install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
sudo ufw allow ssh
ifconfig
```

## Setup on Laptop VSCode
```
ssh nvidia@<jetson-ip-from-ifconfig>
python3 -m venv venv
source venv/bin/activate
wget https://github.com/IntelRealSense/librealsense/releases/download/v2.56.3/pyrealsense2-2.56.3.7838-cp310-cp310-manylinux1_x86_64_beta.whl
pip install pyrealsense2-2.56.3.7838-cp310-cp310-manylinux1_x86_64_beta.whl
pip install -r requirements.txt
```

> Note, librealsense package is for D400 camera. Need to find correct package for camera we are using