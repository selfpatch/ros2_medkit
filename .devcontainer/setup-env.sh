#!/bin/bash
set -e

# Add ROS2 setup to shell configs
grep -Fxq "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -Fxq "source /opt/ros/jazzy/setup.zsh" ~/.zshrc || echo "source /opt/ros/jazzy/setup.zsh" >> ~/.zshrc

# Source the current shell to apply changes immediately
if [ -n "$ZSH_VERSION" ]; then
    # Running in zsh
    source ~/.zshrc 2>/dev/null || true
elif [ -n "$BASH_VERSION" ]; then
    # Running in bash
    source ~/.bashrc 2>/dev/null || true
fi

# Initialize rosdep
sudo rosdep init
rosdep update

# Test installations
echo "Testing installations..."
echo "ROS2 Jazzy: $(test -f /opt/ros/jazzy/setup.bash && echo 'Installed' || echo 'Not found')"

echo "Environment setup complete!"