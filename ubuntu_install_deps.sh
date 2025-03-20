#########################################
# determine current shell 
cur_shell="$(readlink /proc/$$/exe | sed "s/.*\///")"
startup_file=~/.bashrc
# check if cur shell is bash or zsh
if [[ "${cur_shell}" == "bash" ]]
then
	startup_file=~/.bashrc
elif [[ "${cur_shell}" == "zsh" ]]
then
	startup_file=~/.zshrc
else
	echo "Sorry, i dont know your shell: $cur_shell"
	echo "Only bash and zsh are supported"
	exit 1
fi

echo "export PATH=\"\$PATH:$PWD/unitree_sdk2_python\"" >> $startup_file


#########################################
# update and install base dependencies
sudo apt update
sudo apt install -y \
      wget \
      curl \
      sudo \
      nano \
      python3 \
      python3-pip \
      xorg \
      x11-apps \
      xserver-xorg-core \
      git \
      lsb-release 


#########################################
# install micromamba if it is not installed
if ! command -v micromamba 2>&1 >/dev/null
then
    echo "Installing micromamba..."
    "${SHELL}" <(curl -L micro.mamba.pm/install.sh)
else
    "Micromamba found!"
fi

# set up micromamba environment
CONDA_ENV_NAME=unitree_sim_env
micromamba create -n $CONDA_ENV_NAME
micromamba activate $CONDA_ENV_NAME

# install base dependencies
micromamba install python==3.10
pip install \
    mujoco \
    pygame \
    casadi \
    meshcat \
    numpy \
    matplotlib \
    getch

# install unitree sdk
pushd unitree_sdk2_python
pip install -e .
popd >> /dev/null

#########################################
# Set up robotpkg repository
sudo mkdir -p /etc/apt/keyrings

curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc

echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list
      

#########################################
# update and install pinocchio
sudo apt update
sudo apt install -qqy "robotpkg-py3*-pinocchio"


#########################################
# Create a temporary file with the new environment variables
cat << EOF > /tmp/robotpkg_env
export PATH=/opt/openrobots/bin:\$PATH
export PATH=\$HOME/.local/bin:\$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:\$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH
echo "export PATH=\$PATH:$PWD/unitree_sdk2_python" >> $startup_file
EOF


#########################################
# Check if the environment variables are already in .bashrc
if ! grep -q "# Robotpkg environment variables" $startup_file; then
    echo -e "\n# Robotpkg environment variables" >> $startup_file
    cat /tmp/robotpkg_env >> $startup_file
fi


#########################################
# clena up
rm /tmp/robotpkg_env

echo "installation complete, restart terminal to apply changes"
