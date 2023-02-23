#!/bin/bash

## Update these version information
REL_VER=08.06.00.00
EDGE_AI_VER=08_06_00
SRC_REPO="origin"
SRC_BRANCH="master"
REL_PREP_BRANCH="REL_prep_temp"
REL_REPO="release"
REL_BRANCH="REL.${REL_VER}"

# Add "release" repo if it does not exist
RELEASE_URL="ssh://git@bitbucket.itg.ti.com/processor-sdk-vision/jacinto_ros_perception.git"
CHECK_RELEASE_URL=$(git config --get remote.release.url)
if [ "$CHECK_RELEASE_URL" = "$RELEASE_URL" ]; then
	echo "\"release\" repo already exists."
else
	git remote add release $RELEASE_URL
fi

# Update remote repos
git remote update

## Work on temporary work branch $REL_PREP_BRANCH
git checkout $SRC_REPO/$SRC_BRANCH -b $REL_PREP_BRANCH

# Remove folders not to release
# Robotics SDK 0.5 RC4: Decided to remove ros2/nodes/ti_vl
echo "Remove folders not to release..."
Folders_remove=(
	ros2/nodes/ti_vl
	docs_internal
	scripts_internal
)
for Folder in ${Folders_remove[@]}; do
	git rm -r $Folder
done

# Remove files not to release
# Robotics SDK 0.5 RC4: Decided to remove ros2/nodes/**/bag_*_launch.py
echo "Remove files not to release..."
# Files_remove=(
# 	ros2/nodes/ti_vl/launch/bag_visloc_launch.py
# 	ros2/nodes/ti_estop/launch/bag_estop_launch.py
# 	ros2/nodes/ti_vision_cnn/launch/bag_semseg_cnn_launch.py
# 	ros2/nodes/ti_vision_cnn/launch/bag_objdet_cnn_launch.py
# 	ros2/nodes/ti_sde/launch/bag_sde_launch.py
# 	ros2/nodes/ti_sde/launch/bag_sde_pcl_launch.py
# 	cmake/platform_PC.cmake
# 	compatible.txt
# )
Files_remove=(
	ros2/nodes/ti_viz_nodes/launch/rviz_visloc_launch.py
	cmake/platform_PC.cmake
	compatible.txt
)
for File in ${Files_remove[@]}; do
	git rm $File
done

# make commit
now=$(date)
git commit -m"Release prep: $now"

## Merge to the target $REL_PREP_BRANCH (to be pushed to $REL_REPO)
# https://stackoverflow.com/questions/2862590/how-to-replace-master-branch-in-git-entirely-from-another-branch
git merge --allow-unrelated-histories -s ours $REL_REPO/master -m"Merged $REL_REPO/master branch"

# Defne variables for the commit message
REL_COMMIT_MSG="REL.$REL_VER. Compatible with: Linux SDK for Edge AI $EDGE_AI_VER"

# "git merge --squash" to hide commit history
git checkout $REL_REPO/master -b $REL_BRANCH
git merge --allow-unrelated-histories -X theirs $REL_PREP_BRANCH --squash
git add -u
git commit -m"$REL_COMMIT_MSG"
echo "$REL_BRANCH branch created. This branch is the release candidate."

# Remove the temporary work branch $REL_PREP_BRANCH
git branch -D $REL_PREP_BRANCH

echo "===> Instruction for mirroring to the git.ti.com repo"
echo "Please review carefully before pushing to the \"release\" repository:"
echo "  git push -u $REL_REPO $REL_BRANCH"

echo "## On \"jacinto_ros_perception\" repo"
echo "  git checkout master"
echo "  git pull"
echo "  git checkout --track origin/REL.$REL_VER"
echo "If everything looks fine, merger $REL_BRANCH to the master branch, and tag"
echo "  git checkout master"
echo "  git merge $REL_BRANCH"
echo "  git tag -a REL.$REL_VER -m \"REL.$REL_VER\""
echo "  git push origin master"
echo "  git push origin --tags"

echo "## tag \"j7_ros_perception\" the release branch"
echo "  git checkout $SRC_REPO/$SRC_BRANCH"
echo "  git tag -a REL.J7ROS.$REL_VER -m \"REL.J7ROS.$REL_VER\""
echo "  git push origin --tags"
