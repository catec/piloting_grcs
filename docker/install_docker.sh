#/bin/bash

# Check if ubuntu18 or 20
OS_VERSION=$(lsb_release -r |cut -f2)

if [ $OS_VERSION != "18.04" ] && [ $OS_VERSION != "20.04" ] 
then
	echo "Only tested on Ubuntu LTS versions: 18.04 and 20.04. Current version:"$OS_VERSION
	exit 0
fi

# Check if "docker" is installed
REQUIRED_PKG="docker"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" != "$PKG_OK" ]; then
    echo "Docker already installed"
    exit 0
fi

# Setup 
sudo apt-get remove -y docker docker-engine docker.io containerd runc
sudo apt-get update
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install docker engine
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io

sudo groupadd docker
sudo usermod -aG docker $USER
sudo systemctl restart docker

sudo chmod 666 /var/run/docker.sock
echo "xhost +local:docker &> /dev/null" >> ~/.bashrc

# Used to reset terminal
bash
