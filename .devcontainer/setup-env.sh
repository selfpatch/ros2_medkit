#!/bin/bash
set -e

# Add ROS2 setup to shell configs
grep -Fxq "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -Fxq "source /opt/ros/jazzy/setup.zsh" ~/.zshrc || echo "source /opt/ros/jazzy/setup.zsh" >> ~/.zshrc

# ccache configuration for PCH compatibility
grep -Fxq 'export CCACHE_SLOPPINESS=pch_defines,time_macros' ~/.bashrc || echo 'export CCACHE_SLOPPINESS=pch_defines,time_macros' >> ~/.bashrc
grep -Fxq 'export CCACHE_SLOPPINESS=pch_defines,time_macros' ~/.zshrc || echo 'export CCACHE_SLOPPINESS=pch_defines,time_macros' >> ~/.zshrc

# Source the current shell to apply changes immediately
if [ -n "$ZSH_VERSION" ]; then
    # Running in zsh
    # shellcheck source=/dev/null
    source ~/.zshrc 2>/dev/null || true
elif [ -n "$BASH_VERSION" ]; then
    # Running in bash
    # shellcheck source=/dev/null
    source ~/.bashrc 2>/dev/null || true
fi

# Initialize rosdep if not already initialized
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Install pre-commit hooks (pre-commit for local, pre-push for clang-tidy)
if command -v pre-commit &>/dev/null; then
  pre-commit install
  pre-commit install --hook-type pre-push
fi

# Test installations
echo "Testing installations..."
echo "ROS2 Jazzy: $(test -f /opt/ros/jazzy/setup.bash && echo 'Installed' || echo 'Not found')"

echo "Environment setup complete!"
