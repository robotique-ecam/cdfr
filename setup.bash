#!/bin/bash

function print_info {
    (>&2 printf "\x1b[36m[i] ${1}\x1b[0m\n")
}

function print_success {
    (>&2 printf "\x1b[32m[+] ${1}\x1b[0m\n")
    exit 0
}

function print_failure {
    (>&2 printf "\x1b[31m[!] ${1}\x1b[0m\n")
    exit 1
}

echo "[38;5;004m                                 /
                               &
                              %
                 %%%%%%%%%%%%%%%%
             .%%%%%%%%%%%%%%%%%%%%%%/
            %%%%%%%    #%%%%%%%%%%%%%%(
          ,%%%%%%%%#   %%%%%%,   %%%%%%%
          #%%%%%%%%%%%%%%%%%%    %%%%%%%
               .#%%%%%%%%%%%%%%%%%%%%%%%
         (%%%#, &%%%%%%%%%%%%%%%%%%%%%%
 &%%%%%%%%%%%%%%%%%%%%%%%%%%#    (&%#
    ##%%%%%%%%%%%    *%%%%%%%%%%%%#.
       %%%%%%%%%%%.            (%%%%%
      %%%%%%%%%%%%%*          *%%%%%%%/
     %%%%%%%%%%%%%%%#        .%%%*(%%&
    &%%%%%%%%%%%%%%%%%       %%%(
   %%%%%%%%%%%%%%%%%%%%     %%%%
  (%%%%%%%%%%%%%%%%%%%%%   %%%%
     (%%%%%%%%%%%%%%%%%%% %%%%
     %%%%%%%%%%%%%%%%%%%%%%%%
     %%%%%%%%%%%%&         /
        %%%%%%%.


"

print_info "Welcome to Robot Setup Script"
read -p "Please enter the name of the robot to setup : " robot

if [ $robot = 'asterix' ] || [ $robot = 'obelix' ]; then

  print_info "Setting up $robot"
  xacro tools/xacro/$robot.xacro -o src/$robot/robot/$robot.urdf && colcon build --symlink-install --cmake-args=' -DCMAKE_BUILD_TYPE=Release' --packages-skip assurancetourix strategix pharaon_msgs pharaon && print_success "Built packages for $robot" || print_failure "Packages build failed"

elif [ $robot = 'assurancetourix' ]; then

  print_info "Setting up $robot"
  colcon build --symlink-install --cmake-args ' -DMIPI_CAMERA=ON' --cmake-args=' -DCMAKE_BUILD_TYPE=Release' --packages-select assurancetourix strategix_msgs strategix pharaon_msgs pharaon && print_success "Built packages for $robot" || print_failure "Packages build failed"

else

  print_failure "No such component"

fi
