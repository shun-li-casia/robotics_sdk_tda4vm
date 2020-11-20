#!make
# Dependency: J7 Processor SDK-RTOS 07_01_00_11

## Independent variables
# Dockerfile
DOCKERFILE_NAME  = Dockerfile
# Docker image tag
DOCKER_TAG       = j7ros
# TI Vision Apps Lib version
TIVA_LIB_VER     = 7.1.0
# TIDL version
TIDL_VER         = 1.3.0.11
# ROS bag date
ROSBAG_DATE      = 2020_0914

## Dependent variables
WORK_DIR         = j7ros_home
REPO_NAME        = jacinto_ros_perception
DOCKER_BUILD_SH  = docker_build.sh
DOCKER_RUN_SH    = docker_run.sh
OPKG_REPO_DIR    = /opkg_repo
REPO_PATH        = $(WORK_PATH_HOST)/catkin_ws/src/$(REPO_NAME)
WORK_PATH_HOST   = $(HOME)/$(WORK_DIR)
DATA_DIR         = $(WORK_PATH_HOST)/data
DOCKER_DIR       = $(REPO_PATH)/docker
SCRIPTS_DIR      = $(REPO_PATH)/docker/scripts

# Environment variable list to pass to the Docker container
ENV_FILE         = env_list.txt
ENV_FILE_PATH    = $(DOCKER_DIR)/$(ENV_FILE)

# Tarball URLs
TIDL_MODEL_URL  = https://tidrive.ext.ti.com/fss/public/link/public/stream/read/tidl-semseg-model_1.3.0.11.tar.gz?linkToken=xeW1RA_ltU_X5bVF&itemName=aabc76cb-ae69-451c-97b9-11bf0b959563
ROSBAG_URL   	= https://tidrive.ext.ti.com/fss/public/link/public/stream/read/ros-bag_2020_0914.tar.gz?linkToken=rRCsIGcQrCe_EKwh&itemName=3b002324-6c8b-43b4-b4b3-c797d3852ff3

# Tarball filenames
TIDL_MODEL_FILE  = tidl-semseg-model_$(TIDL_VER).tar.gz
ROSBAG_FILE      = ros-bag_$(ROSBAG_DATE).tar.gz

# IP address
J7_IP_ADDR       := $(shell ifconfig | grep -A 1 'eth0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)

ip_show:
	@echo "$(J7_IP_ADDR)"

## Docker
scripts: docker_build_sh docker_run_sh

docker_build_sh:
	@echo "#!/bin/bash"                                      >  $(DOCKER_BUILD_SH)
	@echo "DOCKER_TAG=$(DOCKER_TAG)"                         >> $(DOCKER_BUILD_SH)
	@echo "DOCKER_DIR=$(DOCKER_DIR)"                         >> $(DOCKER_BUILD_SH)
	@echo "cd $(WORK_PATH_HOST)"                             >> $(DOCKER_BUILD_SH)
	@echo "docker build -t \$$DOCKER_TAG -f \$$DOCKER_DIR/$(DOCKERFILE_NAME) ." >> $(DOCKER_BUILD_SH)
	@chmod +x $(DOCKER_BUILD_SH)
	@echo "$(DOCKER_BUILD_SH) is generated"

docker_run_sh:
	@echo "#!/bin/bash"                                      >  $(DOCKER_RUN_SH)
	@echo "DOCKER_TAG=$(DOCKER_TAG)"                         >> $(DOCKER_RUN_SH)
	@echo "DOCKER_DIR=$(DOCKER_DIR)"                         >> $(DOCKER_RUN_SH)
	@echo "IP_ADDR=\$$(ifconfig | grep -A 1 'eth0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)" >> $(DOCKER_RUN_SH)
	@echo "TIVA_LIB_VER=$(TIVA_LIB_VER)"                     >> $(DOCKER_RUN_SH)
	@echo "if [ \"\$$#\" -lt 1 ]; then"                      >> $(DOCKER_RUN_SH)
	@echo "    COMMAND=/bin/bash"                            >> $(DOCKER_RUN_SH)
	@echo "else"                                             >> $(DOCKER_RUN_SH)
	@echo "    COMMAND=\"\$$@\""                             >> $(DOCKER_RUN_SH)
	@echo "fi"                                               >> $(DOCKER_RUN_SH)
	@echo "docker run -it --rm \\"                           >> $(DOCKER_RUN_SH)
	@echo "    -v $(HOME)/$(WORK_DIR):/root/$(WORK_DIR) \\"  >> $(DOCKER_RUN_SH)
	@echo "    -v /usr:/host/usr:ro \\"                      >> $(DOCKER_RUN_SH)
	@echo "    -v /dev:/dev \\"                              >> $(DOCKER_RUN_SH)
	@echo "    --privileged \\"                              >> $(DOCKER_RUN_SH)
	@echo "    --network host \\"                            >> $(DOCKER_RUN_SH)
	@echo "    --device-cgroup-rule='c 238:* rmw' \\"        >> $(DOCKER_RUN_SH)
	@echo "    --env J7_IP_ADDR=\$$IP_ADDR \\"               >> $(DOCKER_RUN_SH)
	@echo "    --env TIVA_LIB_VER=\$$TIVA_LIB_VER \\"        >> $(DOCKER_RUN_SH)
	@echo "    --env-file \$$DOCKER_DIR/$(ENV_FILE) \\"      >> $(DOCKER_RUN_SH)
	@echo "      \$$DOCKER_TAG \$$COMMAND"                   >> $(DOCKER_RUN_SH)
	@chmod +x $(DOCKER_RUN_SH)
	@echo "$(DOCKER_RUN_SH) is generated"

## data files
data_download: tidl_net_download rosbag_download

tidl_net_download: check_data_dir
	wget -O $(DATA_DIR)/$(TIDL_MODEL_FILE) "$(TIDL_MODEL_URL)"
	rm -rf  $(DATA_DIR)/tidl_semseg_model
	tar xzf $(DATA_DIR)/$(TIDL_MODEL_FILE) -C $(DATA_DIR)
	rm -f $(DATA_DIR)/$(TIDL_MODEL_FILE)

rosbag_download: check_data_dir
	wget -O $(DATA_DIR)/$(ROSBAG_FILE) "$(ROSBAG_URL)"
	rm -rf  $(DATA_DIR)/ros_bag
	tar xzf $(DATA_DIR)/$(ROSBAG_FILE) -C $(DATA_DIR)
	rm -f $(DATA_DIR)/$(ROSBAG_FILE)

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
