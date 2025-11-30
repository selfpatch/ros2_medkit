FROM ubuntu:24.04

# Environment setup
ENV ROS_DISTRO=jazzy
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV HADOLINT_VERSION=v2.14.0

# Args
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=1000

# System dependencies
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    locales \
    build-essential \
    cmake \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python-is-python3 \
    wget \
    curl \
    vim \
    nano \
    sudo \
    software-properties-common \
    lsb-release \
    gnupg2 \
    yq \
    ca-certificates \
    clang-format \
    clang-tidy \
    graphviz \
    plantuml \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository and install ROS 2 Jazzy
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python tools via pip
RUN pip install --break-system-packages \
    vcstool \
    setuptools \
    pytest \
    argcomplete \
    black \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    sphinx \
    sphinx-rtd-theme \
    sphinx-needs \
    sphinxcontrib-plantuml \
    sphinx-autobuild \
    myst-parser

# Install hadolint for Dockerfile linting
RUN wget -O /usr/local/bin/hadolint https://github.com/hadolint/hadolint/releases/download/${HADOLINT_VERSION}/hadolint-Linux-x86_64 && \
    chmod +x /usr/local/bin/hadolint

# Create non-root user
# Remove existing user/group with the same UID/GID if they exist
RUN if id -u ${USER_UID} &>/dev/null; then userdel -r "$(id -un ${USER_UID})" 2>/dev/null || true; fi && \
    if getent group ${USER_GID} &>/dev/null; then groupdel "$(getent group ${USER_GID} | cut -d: -f1)" 2>/dev/null || true; fi && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# Switch to user context
USER ${USERNAME}
WORKDIR /home/${USERNAME}
RUN mkdir -p /home/${USERNAME}/workspace
