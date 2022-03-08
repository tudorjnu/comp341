# mujoco-installation

## Install Ubuntu on Windows Subsystem for Linux (WSL)
Open a terminal on Windows (PowerShell) and execute the following:

```
wsl --install
```

Restart the computer


## Install MuJoCo on WSL

Within WSL and using bash, run the following:

```
# make mujoco directory
mkdir ~/.mujoco

# switch to mujoco dir
cd .mujoco/

# download and unzip mujoco
wget https://github.com/deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz
tar xzvf mujoco210-linux-x86_64.tar.gz
```

Now we have to add those to the PATH by opening the ```.bashrc```

```
echo 'export LD_LIBRARY_PATH=~/.mujoco/mujoco210/bin:${LD_LIBRARY_PATH}' >> ~/.bashrc 
echo 'export LD_LIBRARY_PATH=/usr/lib/nvidia:${LD_LIBRARY_PATH}' >> ~/.bashrc 
```

## Install Miniconda to not tinker with the system libraries (click yes when you are prompted)

```
# download 
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
# install it
bash Miniconda3-latest-Linux-x86_64.sh
```

## Install dependencies, create a virtual environment and install mujoco-py

```
# install dependencies
sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3

# create a virtual environment
# syntax goes as: conda create -n <env-name> <module>=<version>
conda create -n comp341 python=3.9

# lastly, activate the environment and install mujoco-py
# syntax is conda activate <env-name>
conda activate comp341
pip install mujoco-py

# to deactivate the environment simply run (not necessary right now as we will test it first 
conda deactivate
```

## Testing the system

```
# open python in the virual env
python

import mujoco_py
import os

# find mujoco path
mj_path = mujoco_py.utils.discover_mujoco()
xml_path = os.path.join(mj_path, 'model', 'humanoid.xml')

# load model
model = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(model)

# test if values match
print(sim.data.qpos)
# [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]

# test if values match
sim.step()
print(sim.data.qpos)
# [-2.09531783e-19  2.72130735e-05  6.14480786e-22 -3.45474715e-06
#   7.42993721e-06 -1.40711141e-04 -3.04253586e-04 -2.07559344e-04
#   8.50646247e-05 -3.45474715e-06  7.42993721e-06 -1.40711141e-04
#  -3.04253586e-04 -2.07559344e-04 -8.50646247e-05  1.11317030e-04
#  -7.03465386e-05 -2.22862221e-05 -1.11317030e-04  7.03465386e-05
#  -2.22862221e-05]
```

## Install OpenAI Gym

```
# make sure the virtual env is activated
pip install gym
```




