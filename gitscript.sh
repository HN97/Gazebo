#!/bin/bash

NAM_FOLDER=namtest
REPO_PATH=https://github.com/HN97/Gazebo.git

echo "Remove folder ${NAM_FOLDER}"
rm -rf ${NAM_FOLDER}
mkdir -p ${NAM_FOLDER}
git -C ${NAM_FOLDER} clone ${REPO_PATH} ./
git -C ${NAM_FOLDER} checkout develop
git -C ${NAM_FOLDER} reset --hard HEAD

if [ -d ${NAM_FOLDER}/.git ];then
	rm -rf ${NAM_FOLDER}/Simulation ${NAM_FOLDER}/image ${NAM_FOLDER}/scripts ${NAM_FOLDER}/src/controlPID
	rm -rf ${NAM_FOLDER}/Tool ${NAM_FOLDER}/Command ${NAM_FOLDER}/src/aruco_detector
fi

cd ${NAM_FOLDER} && catkin build

echo "DONE"
echo -e "\x1B[31mLet open readme for the next step, please.\033[0m"
