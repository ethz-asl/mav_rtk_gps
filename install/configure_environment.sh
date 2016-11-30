#!/bin/bash
GIT_REPO_LIBSBP=git@github.com:swift-nav/libsbp.git
REPO_TAG=v1.2.1 #version you want to chechout before installing

# Install libsbp in $HOME and compile it
mkdir  ~/piksi_rtk_lib
cd ~/piksi_rtk_lib
git clone $GIT_REPO_LIBSBP
cd ./libsbp
git checkout $REPO_TAG

# Install requirements.
sudo apt-get install pandoc
sudo pip install -r requirements.txt
# Build package.
make python

# Export PYTHONPATH and make sure it points to the python subdirectory of the repository
sh -c 'echo "export PYTHONPATH=\${PYTHONPATH}:~/piksi_rtk_lib/libsbp/python #add libsbp for RTK GPS Piksi devices" >> ~/.bashrc'
