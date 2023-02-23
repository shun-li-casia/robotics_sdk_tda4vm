#!/bin/bash
# repleace CMakeLists_release.txt -> CMakeLists.txt

Nodes=(
	nodes/ti_sde
	nodes/ti_semseg_cnn
    nodes/ti_estop
    nodes_internal/ti_vl
)
if [ $# -eq 0 ]; then
    echo "CMakeLists_release.txt -> CMakeLists.txt"
    for Node in ${Nodes[@]}; do
        mv $Node/CMakeLists.txt $Node/CMakeLists_head.txt
        cp $Node/CMakeLists_release.txt $Node/CMakeLists.txt
    done
else
    echo "Undoing"
    for Node in ${Nodes[@]}; do
        rm $Node/CMakeLists.txt
        mv $Node/CMakeLists_head.txt $Node/CMakeLists.txt
    done
fi