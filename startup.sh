#!/bin/bash

cd /home/oversir/projects/unitree_sdk2_python \
&& pip3 install -e .

# Install system dependencies
sudo apt install -qqy lsb-release curl

# Set up robotpkg repository
sudo mkdir -p /etc/apt/keyrings

curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc

echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list

# Update and install pinocchio
sudo apt update
sudo apt install -qqy robotpkg-py3*-pinocchio

# Create a temporary file with the new environment variables
cat << EOF > /tmp/robotpkg_env
export PATH=/opt/openrobots/bin:\$PATH
export PATH=\$HOME/.local/bin:\$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:\$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH
EOF

# Check if the environment variables are already in .bashrc
if ! grep -q "# Robotpkg environment variables" ~/.bashrc; then
    echo -e "\n# Robotpkg environment variables" >> ~/.bashrc
    cat /tmp/robotpkg_env >> ~/.bashrc
fi

# Clean up
rm /tmp/robotpkg_env

echo "Installation complete! Please run 'source ~/.bashrc' or restart your terminal to apply the changes."