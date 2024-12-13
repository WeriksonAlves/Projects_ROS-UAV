#!/usr/bin/env bash

#
# Copyright 2024 Gabriel Víctor <gabriel.munizt@ufv.br>
# 
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 2 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 51
# Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#

# This is a script that builds a ROS workspace and builds from source the bebop
# autonomy package for ROS Noetic. It was built with Ubuntu 20.04 LTS in mind.

# Updates the user repositories and installs some dependencies
sudo apt-get update
if [ $? -ne 0 ]; then 
    echo "Something went wrong while updating the package list. Exiting."
    exit 1 
fi

sudo apt-get install build-essential git python3 libavahi-client-dev ros-noetic-joy ros-noetic-joy-teleop ros-noetic-teleop-twist-joy -y
if [ $? -eq 0 ]; then 
    echo -e "Dependencies installed with success.\n"
else 
    echo "Something went wrong while fetching package dependencies. Exiting."
    exit 1 
fi

# Reads the user stdin
read -p "Enter a name for the ROS workspace: " ROSWORKSPACE

# Checks if there's something wrong with the workspace name
if [ -z "$ROSWORKSPACE" ]; then 
    echo "The workspace name can't be empty. Exiting."
    exit 1 
fi

if [[ "$ROSWORKSPACE" =~ [^a-zA-Z0-9_-] ]]; then 
    echo "The workspace name has invalid characters. Exiting."
    exit 1 
fi

# Verifies if there's already a directory using the $ROSWORKSPACE name
if [ -d "$ROSWORKSPACE" ]; then 
    read -p "Found a directory called $ROSWORKSPACE, do you wish to use it for this script? (Y/N)? " choice
    case "$choice" in 
        [Yy]* ) echo "Using the existing workspace.";;
        [Nn]* ) echo "Please choose a different workspace name."; exit 1;;
        * ) echo "Invalid input. Exiting."; exit 1;;
    esac
else
    # Creates a ROS workspace dir in case it doesn't exist
    mkdir -p "$ROSWORKSPACE/src"
    echo "Workspace directory created: $ROSWORKSPACE/src"
fi

# Downloading and compiling the parrot arsdk package
cd "$ROSWORKSPACE"
ROSWORKSPACEROOT=$(pwd)
cd src

git clone https://github.com/antonellabarisic/parrot_arsdk.git
cd parrot_arsdk
git checkout noetic_dev

# Tries to create a symlink between python3 and python2, because python2 is the default interpreter in this package
if ! sudo ln -sf /usr/bin/python3 /usr/bin/python; then 
    echo "Python2 found, this may lead to errors. If something happens, try removing it. Ignore if there's already a symlink from python3 to python."
fi

cd "$ROSWORKSPACEROOT"

# Tries to compile the package twice
catkin_make
if [ $? -ne 0 ]; then 
    catkin_make
    if [ $? -ne 0 ]; then 
        echo "Something went wrong while trying to compile parrot arsdk. Exiting."
        exit 1
    fi
fi

source devel/setup.bash

# Downloading and compiling bebop autonomy package
cd src
git clone https://github.com/AutonomyLab/bebop_autonomy.git
cd bebop_autonomy

# Changes the lines recommended by the git parrot_arsdk page
file="$ROSWORKSPACEROOT/src/bebop_autonomy/bebop_driver/src/bebop_video_decoder.cpp"

# Line 93 "AP_TRUNCATED" to "AV_CODEC_CAP_TRUNCATED"
sed -i '93s/CODEC_CAP_TRUNCATED/AV_CODEC_CAP_TRUNCATED/' "$file"

# Line 95 "CODEC_FLAG_TRUNCATED" to "AV_CODEC_FLAG_TRUNCATED"
sed -i '95s/CODEC_FLAG_TRUNCATED/AV_CODEC_FLAG_TRUNCATED/' "$file"

# Line 97 "CODEC_FLAG2_CHUNKS" to "AV_CODEC_FLAG2_CHUNKS"
sed -i '97s/CODEC_FLAG2_CHUNKS/AV_CODEC_FLAG2_CHUNKS/' "$file"

cd "$ROSWORKSPACEROOT"

source devel/setup.bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROSWORKSPACEROOT/devel/lib/parrot_arsdk

# Tries to compile the package twice
catkin_make
if [ $? -ne 0 ]; then 
    catkin_make
    if [ $? -ne 0 ]; then 
        echo "Something went wrong while trying to compile bebop autonomy. Exiting."
        exit 1
    fi
fi

# Writes to setup.bash a new library path var including the parrot arsdk library
if ! grep -q "export LD_LIBRARY_PATH=" devel/setup.bash; then 
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:"$ROSWORKSPACEROOT"/devel/lib/parrot_arsdk" >> devel/setup.bash
fi

echo "Instalation finished with sucess!"