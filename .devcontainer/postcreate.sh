# Update container
if ! sudo apt-get update; then
    echo "Failed to update"
    exit 1
fi
if ! sudo apt-get upgrade -y; then
    echo "Failed to upgrade"
    exit 1
fi

# Download depenendencies
mkdir -p ${USER_WORKSPACE}/src/deps
if ! vcs import --input ${USER_WORKSPACE}/src/daly_bms/common.repos.yaml ${USER_WORKSPACE}/src/deps; then
    echo "Failed to download dependencies"
fi

if ! local_deps.sh; then
    echo "Failed to install local dependencies"
    exit 1
fi

if ! rosdep update --rosdistro=${ROS_DISTRO}; then
    echo "Failed to update rosdep"
    exit 1
fi

# Install ROS packages
if ! rosdep install --from-paths ${USER_WORKSPACE}/src --ignore-src -y -r; then
    echo "Failed to install rosdep packages"
    exit 1
fi
