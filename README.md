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
pip install -r requirements.txt
```