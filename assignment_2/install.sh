#!/bin/sh

if [ ! -e .venv ]
then
    # install python3.8.10 if possible
    sudo apt update 
    sudo apt-get install -y virtualenv

    # generate virtual environment
    virtualenv .venv    
    .venv/bin/python -m pip install --upgrade pip    
fi    

#source .venv/bin/activate
#.venv/bin/pip install torch --extra-index-url https://download.pytorch.org/whl/cu113
.venv/bin/pip install torch
.venv/bin/pip install dill
.venv/bin/pip install gym
.venv/bin/pip install scikit-learn==1.3.1
.venv/bin/pip install shimmy==1.2.1

# for baselines
.venv/bin/pip install matplotlib==2.*
.venv/bin/pip install stable-baselines3
.venv/bin/pip install sb3-contrib

# for pybullet-gym
.venv/bin/pip install -e ./pybullet-gym

# Copy a target object to the pybullet_data folder
ver=$(python -c"import sys; print(sys.version_info.minor)")
cp ./urdf/block.urdf .venv/lib/python3."$ver"/site-packages/pybullet_data/
