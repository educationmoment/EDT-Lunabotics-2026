export red="\e[31m"
export green="\e[32m"
export blue="\e[34m"
export reset_color="\e[0m"

trap 'echo -e "${red}An error occurred during installation. Exiting.${reset_color}"; exit 1' ERR

# cleanup_exit
# Check that ROS2 Sources list exists
check_ros_repos(){
    if [ -n $(ls /etc/apt/sources.list.d/ros2*.list 2>>/dev/null) ]
    then
        return 0
    else 
        return 1
    fi
}


print_status() {
    item=$1
    status=$2
    error_code=$3

    # No Previous command tested
    if [ -n "$item" ] && [ -n "$status" ] && [ -z "$error_code" ]
    then
        echo -ne "[ $green" "$1" " $reset_color ] - "
        echo -ne "$blue $status $reset_color"
        echo
    
    # Error in Previous Command
    elif [ -n "$item" ] && [ -n "$status" ] && [ "$error_code" -ne 0 ]
    then
        echo -ne "[ $green" "$1" " $reset_color ] - "
        echo -ne "$red $status $reset_color"
        echo  
        exit 1

    elif [ -n "$item" ] && [ -n "$status" ] && [ "$error_code" -eq 0 ]
    then
        echo -ne "[ $green" "$1" " $reset_color ] - "
        echo -ne "$green $status $reset_color"
        echo  
    fi
}

ros_install() {
    # Do First Full System Update
    print_status "System Update" "Starting..."
    apt-get update
    print_status "System Update" "Finished with Error Code $? (0 indicates Success)" "$?"

    # Do Full System Upgrade
    print_status "System Upgrade" "Starting..."
    apt-get upgrade -y
    print_status "System Upgrade" "Finished with Error Code $? (0 indicates Success)" "$?"

    # Install Software-Properties-Common
    print_status "System Install: software-properties-common" "Starting..."
    apt-get install software-properties-common -y
    print_status "System Install: software-properties-common" "Finished with Error Code $? (0 indicates Success)" "$?"
    
    # Add universe to apt repositories
    print_status "System Install: add-apt-repository"
    add-apt-repository universe
    print_status "System Install: add-apt-repository" "Finished with Error Code $? (0 indicates Success)" "$?"

    # ROS2 Apt Sources List Does Not Exist
    if [ check_ros_repos -ne 0 ]
    then
        apt-get update
        apt-get install curl -y
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    fi

    # Do Second Full System Update: ROS repo added, must update to fetch package lists
    print_status "System Update" "Starting..."
    apt-get update
    print_status "System Update" "Finished with Error Code $? (0 indicates Success)" "$?"

    # Prompt User (ros-humble-base OR ros-humble-core)
    echo "Please choose the ROS2 package to install:"
    echo "1) ros-humble-ros-base"
    echo "2) ros-humble-desktop"
    read -p "Enter the number of your choice: " choice

    # Install the chosen ROS2 package
    case $choice in
        1)
            package="ros-humble-ros-base"
            ;;
        2)
            package="ros-humble-desktop"
            ;;
        *)
            echo -e "${red}Invalid choice. Exiting.${reset_color}"
            exit 1
            ;;
    esac
    print_status "ROS2 Package Install: $package" "Starting..."
    apt-get install -y $package
    print_status "ROS2 Package Install: $package" "Finished with Error Code $? (0 indicates Success)" "$?"

    # Install ROS2 Development Tools
    print_status "ROS2 Development Tools Install" "Starting..."
    apt-get install -y python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pip
    apt-get install -y ros-dev-tools
    print_status "ROS2 Development Tools Install" "Finished with Error Code $? (0 indicates Success)" "$?"

    # Add Setup.bash to .bashrc
    print_status "ROS2 Setup" "Starting..."
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    print_status "ROS2 Setup" "Finished with Error Code $? (0 indicates Success)" "$?"
    echo -e "${green}ROS2 Humble installation completed successfully!${reset_color}"

    source ~/.bashrc
    return 0
}

ros_install_check() {
    # Check if ROS2 is installed
    if [ -d "/opt/ros/humble" ]
    then
        echo -e "${green}ROS2 Humble is already installed.${reset_color}"
        return 0
    else
        return 1
    fi
}

ros_install_package_dependencies() {
    # Check if rosdep is initialized
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]
    then
        print_status "ROS2 Dependency Install" "Starting..."
        rosdep init
        rosdep update
        print_status "ROS2 Dependency Install" "Finished with Error Code $? (0 indicates Success)" "$?"
    fi

    # Install dependencies for the ROS2 package
    print_status "ROS2 Package Dependencies Install" "Starting..."
    rosdep install --from-paths src --ignore-src -r -y
    print_status "ROS2 Package Dependencies Install" "Finished with Error Code $? (0 indicates Success)" "$?"

    # Install additional dependencies
    print_status "ROS2 Additional Dependencies Install: ros-humble-realsense2-camera" "Starting..."
    apt-get install -y ros-humble-realsense2-camera
    print_status "ROS2 Additional Dependencies Install: ros-humble-realsense2-camera" "Finished with Error Code $? (0 indicates Success)" "$?"
    
    # Install SparkCAN
    print_status "ROS2 Additional Dependencies Install: Install sparkcan" "Starting..."
    add-apt-repository ppa:graysonarendt/sparkcan
    apt-get update
    apt-get install -y sparkcan
    print_status "ROS2 Additional Dependencies Install: sparkcan" "Finished with Error Code $? (0 indicates Success)" "$?"

    # Install Madgwick Filter Node
    print_status "ROS2 Additional Dependencies Install: ros-humble-imu-filter-madgwick" "Starting..."
    apt-get install -y ros-humble-imu-filter-madgwick
    print_status "ROS2 Additional Dependencies Install: ros-humble-imu-filter-madgwick" "Finished with Error Code $? (0 indicates Success)" "$?"

    # Install Flask
    print_status "ROS2 Additional Dependencies Install: python3-pip" "Starting..."
    python3 -m pip install flask pyrealsense2 apriltag
    print_status "ROS2 Additional Dependencies Install: python3-pip" "Finished with Error Code $? (0 indicates Success)" "$?"

    source /opt/ros/humble/setup.bash

    print
    
}
# source /opt/ros/humble/setup.bash
ros_install
ros_install_check
ros_install_package_dependencies
