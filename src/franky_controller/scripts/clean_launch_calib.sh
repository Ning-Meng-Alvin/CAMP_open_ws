#!/bin/bash

# Step 1: Force deactivation of any active Conda environments to restore a clean system environment.
# First, source the conda init script to make the 'conda' command available.
# Note: Your path might be 'anaconda3', please adjust if necessary.
if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/anaconda3/etc/profile.d/conda.sh"
fi

# Execute deactivate to exit the virtual environment.
# It's run twice to handle potentially nested environments, with the second one's
# potential error output suppressed.
conda deactivate
conda deactivate &> /dev/null

# Step 2: After deactivating Conda, re-source the clean ROS system environment.
source /opt/ros/noetic/setup.bash
source ~/CampUsers/Pei/open_ws/devel/setup.bash

# Step 3: Now, execute the desired launch file in the clean environment.
roslaunch netft_calib check_netft_calib.launch