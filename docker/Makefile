#!make

#  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Independent variables
# TI Vision Apps Lib version
TIVA_LIB_VER     = 7.3.0
# TIDL version
TIDL_VER         = 2.0.0.7
# ROS bag date
ROSBAG_DATE      = 2020_1109

## Dependent variables
WORK_DIR         = j7ros_home
REPO_NAME        = jacinto_ros_perception
DOCKER_BUILD_SH  = docker_build.sh
DOCKER_RUN_SH    = docker_run.sh
OPKG_REPO_DIR    = /opkg_repo
WORK_PATH_HOST   = $(HOME)/$(WORK_DIR)
REPO_PATH        = $(WORK_PATH_HOST)/catkin_ws/src/$(REPO_NAME)
DATA_DIR         = $(WORK_PATH_HOST)/data
DOCKER_DIR       = $(REPO_PATH)/docker
SCRIPTS_DIR      = $(REPO_PATH)/docker/scripts

# Environment variable list to pass to the Docker container
ENV_FILE         = env_list.txt
ENV_FILE_PATH    = $(DOCKER_DIR)/$(ENV_FILE)

# Tarball filenames
TIDL_MODEL_FILE  = tidl-semseg-model_$(TIDL_VER).tar.gz
ROSBAG_FILE      = ros-bag_$(ROSBAG_DATE).tar.gz

# Tarball URLs
TIDL_MODEL_URL   = https://software-dl.ti.com/jacinto7/esd/processor-sdk-rtos-jacinto7/ros_perception/$(TIDL_MODEL_FILE)
ROSBAG_URL   	 = https://software-dl.ti.com/jacinto7/esd/processor-sdk-rtos-jacinto7/ros_perception/$(ROSBAG_FILE)

# Semantic segmantation TVM-DLR model artifacts location
SEMSEG_MODEL     = tvmdlr_deeplabv3lite_mobilenetv2_tv_768x432_zed1hd
SEMSEG_ARTIFACTS_DIR = $(DATA_DIR)/tidl_semseg_model/$(SEMSEG_MODEL)/artifacts

# Visual localization related: temporary URL (to be updated)
VISUAL_LOC_URL   = https://tidrive.ext.ti.com/fss/public/link/public/stream/read/visual-localization_2021_0510.tar.gz?linkToken=y5RKIwGUSjzZlEoi&itemName=c7025289-8fa9-415f-8b25-14e0adbd40ea
VISUAL_LOS_VER   = 2021_0510
VISUAL_LOC_FILE  = visual-localization_$(VISUAL_LOS_VER).tar.gz

# IP address
J7_IP_ADDR       := $(shell ifconfig | grep -A 1 'eth0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)

## Set up environments on J7 host linux
opkg_config:
	$(SCRIPTS_DIR)/config_opkg_repo.sh

ipk_install:
	$(SCRIPTS_DIR)/install_tivision_apps_ipk.sh

ip_show:
	@echo "$(J7_IP_ADDR)"

## Docker
scripts: DOCKERFILE_NAME = Dockerfile.ros-melodic
scripts: DOCKER_TAG = j7-ros-melodic:0.4
scripts: docker_build_sh docker_run_sh

scripts_rosdep: DOCKERFILE_NAME = Dockerfile.ros-melodic.rosdep
scripts_rosdep: DOCKER_TAG = j7-ros-melodic:0.4_rosdep
scripts_rosdep: docker_build_sh docker_run_sh

docker_build_sh:
	@echo "#!/bin/bash"                                       >  $(DOCKER_BUILD_SH)
	@echo "DOCKER_TAG=$(DOCKER_TAG)"                          >> $(DOCKER_BUILD_SH)
	@echo "DOCKER_DIR=$(DOCKER_DIR)"                          >> $(DOCKER_BUILD_SH)
	@echo "# modify the server and proxy URLs as requied"     >> $(DOCKER_BUILD_SH)
	@echo "ping bitbucket.itg.ti.com -c 1 > /dev/null 2>&1"   >> $(DOCKER_BUILD_SH)
	@echo "if [ \"\$$?\" -eq \"0\" ]; then"                   >> $(DOCKER_BUILD_SH)
	@echo "    USE_PROXY=1"                                   >> $(DOCKER_BUILD_SH)
	@echo "    REPO_LOCATION=artifactory.itg.ti.com/docker-public-arm" >> $(DOCKER_BUILD_SH)
	@echo "    HTTP_PROXY=http://webproxy.ext.ti.com:80"      >> $(DOCKER_BUILD_SH)
	@echo "else"                                              >> $(DOCKER_BUILD_SH)
	@echo "    USE_PROXY=0"                                   >> $(DOCKER_BUILD_SH)
	@echo "    REPO_LOCATION=arm64v8"                         >> $(DOCKER_BUILD_SH)
	@echo "fi"                                                >> $(DOCKER_BUILD_SH)
	@echo "cd $(WORK_PATH_HOST)"                              >> $(DOCKER_BUILD_SH)
	@echo "docker build \\"                                   >> $(DOCKER_BUILD_SH)
	@echo "    -t \$$DOCKER_TAG \\"                           >> $(DOCKER_BUILD_SH)
	@echo "    --build-arg USE_PROXY=\$$USE_PROXY \\"         >> $(DOCKER_BUILD_SH)
	@echo "    --build-arg REPO_LOCATION=\$$REPO_LOCATION \\" >> $(DOCKER_BUILD_SH)
	@echo "    --build-arg HTTP_PROXY=\$$HTTP_PROXY \\"       >> $(DOCKER_BUILD_SH)
	@echo "    -f \$$DOCKER_DIR/$(DOCKERFILE_NAME) ."         >> $(DOCKER_BUILD_SH)
	@chmod +x $(DOCKER_BUILD_SH)
	@echo "$(DOCKER_BUILD_SH) is generated"

docker_run_sh:
	@echo "#!/bin/bash"                                       >  $(DOCKER_RUN_SH)
	@echo "DOCKER_TAG=$(DOCKER_TAG)"                          >> $(DOCKER_RUN_SH)
	@echo "DOCKER_DIR=$(DOCKER_DIR)"                          >> $(DOCKER_RUN_SH)
	@echo "IP_ADDR=\$$(ifconfig | grep -A 1 'eth0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)" >> $(DOCKER_RUN_SH)
	@echo "if [ \"\$$#\" -lt 1 ]; then"                       >> $(DOCKER_RUN_SH)
	@echo "    CMD=/bin/bash"                                 >> $(DOCKER_RUN_SH)
	@echo "else"                                              >> $(DOCKER_RUN_SH)
	@echo "    CMD=\"\$$@\""                                  >> $(DOCKER_RUN_SH)
	@echo "fi"                                                >> $(DOCKER_RUN_SH)
	@echo "# modify the server and proxy URLs as requied"     >> $(DOCKER_RUN_SH)
	@echo "ping bitbucket.itg.ti.com -c 1 > /dev/null 2>&1"   >> $(DOCKER_RUN_SH)
	@echo "if [ \"\$$?\" -eq \"0\" ]; then"                   >> $(DOCKER_RUN_SH)
	@echo "    USE_PROXY=1"                                   >> $(DOCKER_RUN_SH)
	@echo "else"                                              >> $(DOCKER_RUN_SH)
	@echo "    USE_PROXY=0"                                   >> $(DOCKER_RUN_SH)
	@echo "fi"                                                >> $(DOCKER_RUN_SH)
	@echo "docker run -it --rm \\"                            >> $(DOCKER_RUN_SH)
	@echo "    -v $(HOME)/$(WORK_DIR):/root/$(WORK_DIR) \\"   >> $(DOCKER_RUN_SH)
	@echo "    -v /usr:/host/usr:ro \\"                       >> $(DOCKER_RUN_SH)
	@echo "    -v /dev:/dev \\"                               >> $(DOCKER_RUN_SH)
	@echo "    --privileged \\"                               >> $(DOCKER_RUN_SH)
	@echo "    --network host \\"                             >> $(DOCKER_RUN_SH)
	@echo "    --device-cgroup-rule='c 238:* rmw' \\"         >> $(DOCKER_RUN_SH)
	@echo "    --env USE_PROXY=\$$USE_PROXY \\"               >> $(DOCKER_RUN_SH)
	@echo "    --env J7_IP_ADDR=\$$IP_ADDR \\"                >> $(DOCKER_RUN_SH)
	@echo "    --env TIVA_LIB_VER=$(TIVA_LIB_VER) \\"         >> $(DOCKER_RUN_SH)
	@echo "    --env-file \$$DOCKER_DIR/$(ENV_FILE) \\"       >> $(DOCKER_RUN_SH)
	@echo "    \$$DOCKER_TAG \$$CMD"                          >> $(DOCKER_RUN_SH)
	@chmod +x $(DOCKER_RUN_SH)
	@echo "$(DOCKER_RUN_SH) is generated"

## data files
data_download: tidl_net_download rosbag_download

model_download: check_data_dir
	wget -O $(DATA_DIR)/$(TIDL_MODEL_FILE) "$(TIDL_MODEL_URL)"
	rm -rf  $(DATA_DIR)/tidl_semseg_model
	tar xzf $(DATA_DIR)/$(TIDL_MODEL_FILE) -C $(DATA_DIR)
	rm -f $(DATA_DIR)/$(TIDL_MODEL_FILE)
	cd $(SEMSEG_ARTIFACTS_DIR); ln -s deploy_lib.so.j7 deploy_lib.so; cd -

rosbag_download: check_data_dir
	wget -O $(DATA_DIR)/$(ROSBAG_FILE) "$(ROSBAG_URL)"
	rm -rf  $(DATA_DIR)/ros_bag
	tar xzf $(DATA_DIR)/$(ROSBAG_FILE) -C $(DATA_DIR)
	rm -f $(DATA_DIR)/$(ROSBAG_FILE)

vl_download: check_data_dir
	wget -O $(DATA_DIR)/$(VISUAL_LOC_FILE) "$(VISUAL_LOC_URL)"
	rm -rf  $(DATA_DIR)/visual_localization
	tar xzf $(DATA_DIR)/$(VISUAL_LOC_FILE) -C $(DATA_DIR)
	rm -f $(DATA_DIR)/$(VISUAL_LOC_FILE)

check_data_dir:
	@if [ ! -d $(DATA_DIR) ]; then \
		mkdir -p $(DATA_DIR); \
		echo "mkdir -p $(DATA_DIR)"; \
	else \
		echo "Ok, $(DATA_DIR) exists"; \
	fi

data_clean:
	rm -rf $(DATA_DIR)

## .PHONY
.PHONY: docker_build_sh docker_run_sh ip_show \
        tidl_net_download rosbag_download check_data_dir
