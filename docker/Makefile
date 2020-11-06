#!make
## Independent variables
# Dockerfile
DOCKERFILE_NAME  = Dockerfile
# Docker image tag
DOCKER_TAG       = j7ros
# TI Vision Apps Lib version
LIB_VER          = 7.1.0-r0.0
# TIDL version
TIDL_VER         = 1.3.0.7
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

# Tarball URLs: tentative
OPKG_REPO_URL   = https://txn.box.com/shared/static/uav8998htorweblm6se4o0uveepejymr
TIDL_MODEL_URL  = https://txn.box.com/shared/static/2u7ypmbvy9mlk1hq1wq9ym88edk7rk9b
ROSBAG_URL   	= https://txn.box.com/shared/static/oyz3syp7up8rtfhif3akxiw1tac065ys

# Tarball filenames
OPKG_REPO_FILE   = opkg-repo_$(LIB_VER).tar.gz
TIDL_MODEL_FILE  = tidl-semseg-model_$(TIDL_VER).tar.gz
ROSBAG_FILE      = ros-bag_$(ROSBAG_DATE).tar.gz

# IP address
J7_IP_ADDR       := $(shell ifconfig | grep -A 1 'eth0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)

## Set up environments on J7 host linux
host_setup: opkg_config opkg_repo_download ipk_install

opkg_config:
	$(SCRIPTS_DIR)/config_opkg_repo.sh

ipk_install:
	$(SCRIPTS_DIR)/install_tivision_apps_ipk.sh

ip_show:
	@echo "$(J7_IP_ADDR)"

opkg_repo_download: check_data_dir
	wget -O $(DATA_DIR)/$(OPKG_REPO_FILE) $(OPKG_REPO_URL)
	rm -rf $(OPKG_REPO_DIR)
	tar xzf $(DATA_DIR)/$(OPKG_REPO_FILE) -C /

## Docker
scripts: docker_build_sh docker_run_sh

docker_build_sh:
	@echo "#!/bin/bash"                                      >  $(DOCKER_BUILD_SH)
	@echo "DOCKER_TAG=$(DOCKER_TAG)"                         >> $(DOCKER_BUILD_SH)
	@echo "DOCKER_DIR=$(DOCKER_DIR)"                         >> $(DOCKER_BUILD_SH)
	@echo "cd $(WORK_PATH_HOST)"                             >> $(DOCKER_BUILD_SH)
	@echo "docker build -t \$$DOCKER_TAG -f \$$DOCKER_DIR/$(DOCKERFILE_NAME) ." >> $(DOCKER_BUILD_SH)
	@chmod +x $(DOCKER_BUILD_SH)
	@cp $(DOCKER_BUILD_SH) $(DOCKER_DIR)/$(DOCKER_BUILD_SH)
	@echo "$(DOCKER_BUILD_SH) is generated"

docker_run_sh:
	@echo "#!/bin/bash"                                      >  $(DOCKER_RUN_SH)
	@echo "DOCKER_TAG=$(DOCKER_TAG)"                         >> $(DOCKER_RUN_SH)
	@echo "DOCKER_DIR=$(DOCKER_DIR)"                         >> $(DOCKER_RUN_SH)
	@echo "IP_ADDR=\$$(ifconfig | grep -A 1 'eth0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)" >> $(DOCKER_RUN_SH)
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
	@echo "    --env-file \$$DOCKER_DIR/$(ENV_FILE) \\"      >> $(DOCKER_RUN_SH)
	@echo "      \$$DOCKER_TAG \$$COMMAND"                   >> $(DOCKER_RUN_SH)
	@chmod +x $(DOCKER_RUN_SH)
	@cp $(DOCKER_RUN_SH) $(DOCKER_DIR)/$(DOCKER_RUN_SH)
	@echo "$(DOCKER_RUN_SH) is generated"

docker_build:
	./docker_build.sh

docker_run:
	./docker_run.sh

## data files
data_download: tidl_net_download rosbag_download

tidl_net_download: check_data_dir
	wget -O $(DATA_DIR)/$(TIDL_MODEL_FILE) $(TIDL_MODEL_URL)
	rm -rf  $(DATA_DIR)/tidl_semseg_model
	tar xzf $(DATA_DIR)/$(TIDL_MODEL_FILE) -C $(DATA_DIR)

rosbag_download: check_data_dir
	wget -O $(DATA_DIR)/$(ROSBAG_FILE) $(ROSBAG_URL)
	rm -rf  $(DATA_DIR)/ros_bag
	tar xzf $(DATA_DIR)/$(ROSBAG_FILE) -C $(DATA_DIR)

check_data_dir:
	@if [ ! -d $(DATA_DIR) ]; then \
		mkdir -p $(DATA_DIR); \
		echo "mkdir -p $(DATA_DIR)"; \
	else \
		echo "Ok,.$(DATA_DIR) exists"; \
	fi

data_clean:
	rm -rf $(DATA_DIR)

## .PHONY
.PHONY: docker_build_sh docker_run_sh opkg_config ipk_install ip_show \
        opkg_repo_download tidl_net_download rosbag_download check_data_dir
