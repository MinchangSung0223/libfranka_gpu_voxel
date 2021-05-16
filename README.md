# gpu_voxel_panda_sim
conda activate ros
apt-get install ros-melodic-ompl
sudo apt-get install -y ros-melodic-moveit \
                       ros-melodic-industrial-core \
                       ros-melodic-moveit-visual-tools \
                       ros-melodic-joint-state-publisher-gui

sudo apt-get install -y ros-melodic-gazebo-ros-pkgs \
                       ros-melodic-gazebo-ros-control \
                       ros-melodic-joint-state-controller \
                       ros-melodic-effort-controllers \
                       ros-melodic-position-controllers \
                       ros-melodic-joint-trajectory-controller
                       
cmake . -D icl_core_DIR=~/workspace/gpu-voxels/build/packages/icl_core/ -D gpu_voxels_DIR=~/workspace/gpu-voxels/build/packages/gpu_voxels
# libfranka_gpu_voxel
