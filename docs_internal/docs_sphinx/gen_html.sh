#!/bin/bash
# PROJECT_BASE=${HOME}/j7ros_home/ros_ws/src/robotics_sdk
PARENT_DIR=$(dirname `pwd`)
PROJECT_BASE=$(dirname $PARENT_DIR)
HTML_FOLDER="robotics_sdk_docs"

# soft-link the source, but exclude the current folder
rm -rf source
mkdir source
for d in $PROJECT_BASE/*; do
    if [[ "$d" != "$PARENT_DIR" ]]; then
        ln -s "$d" 'source/';
    fi
done

# generate with two different themes
opts=(
	# rtd
    ti
)
for opt in ${opts[@]}; do
    ln -snf conf_$opt.py conf.py
    # Sphinx version <= 1.8.5: expects contents.rst
    # ln -snf index.rst contents.rst
    rm -rf _build_$opt
    make clean
    make html
    mv _build/html _build/$HTML_FOLDER
    mv _build _build_$opt
    xdg-open _build_$opt/$HTML_FOLDER/index.html
done
rm -rf source

# zip
ZIP_FILE=${HTML_FOLDER}.zip
cd _build_ti
zip -q -r $ZIP_FILE robotics_sdk_docs
mv $ZIP_FILE ..
cd -
echo "===> $ZIP_FILE generated"
