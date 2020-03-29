#!/bin/bash

function print_info {
    (>&2 printf "\x1b[36m[i] ${1}\x1b[0m\n")
}

function print_success {
    (>&2 printf "\x1b[32m[+] ${1}\x1b[0m\n")
}

function print_failure {
    (>&2 printf "\x1b[31m[!] ${1}\x1b[0m\n")
    exit 1
}

function generate_urdfs {
    xacro tools/xacro/asterix.xacro -o src/asterix/robot/asterix.urdf
    xacro tools/xacro/obelix.xacro -o src/obelix/robot/obelix.urdf
    if [[ $? -eq 0 ]]; then
        print_success "Generated URDF files"
        return 0
    else
        print_failure "Failled to generate URDFs"
        exit 1
    fi
}


if [ -n "$1" ]; then
    robot=$1
else
    read -p "Please enter the name of the robot to setup : " robot
fi


if [ $robot = "asterix" ] || [ $robot = "obelix" ]; then
    print_info "Setting up $robot"
    generate_urdfs && colcon build --symlink-install --cmake-args=" -DCMAKE_BUILD_TYPE=Release" --packages-skip assurancetourix strategix pharaon_msgs pharaon && print_success "Built packages for $robot" || print_failure "Packages build failed"

elif [ $robot = "assurancetourix" ]; then
    print_info "Setting up $robot"
    colcon build --symlink-install --cmake-args " -DMIPI_CAMERA=ON" --cmake-args=" -DCMAKE_BUILD_TYPE=Release" --packages-select transformix_msgs transformix strategix_msgs strategix pharaon_msgs pharaon assurancetourix && print_success "Built packages for $robot" || print_failure "Packages build failed"

elif [ $robot = "simulation" ]; then
    print_info "Setting up simulation environment"
    generate_urdfs && colcon build --symlink-install --cmake-args=" -DCMAKE_BUILD_TYPE=Release" --cmake-args=" -DSIMULATION=ON" --packages-skip sensors && print_success "Built packages for $robot" || print_failure "Packages build failed"

else
    print_failure "No such component"

fi
