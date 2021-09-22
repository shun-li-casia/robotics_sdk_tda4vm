#!make
MAKEFLAGS += --no-print-directory

# Defaults. These can be changed from command line
ROS_VER          ?= 2
TIVA_LIB_VER     ?= 8.0.0

# GPUS: if 'y', '--gpus all' argument is added in 'docker run' command
GPUS             ?= n

# Robotics SDK version
RSDK_VER          = 0.5

# Set the operating variables based on the ROS version specified
DOCKER_TAG_HOST   = pc-ros2-foxy:$(RSDK_VER)
DOCKER_TAG_TGT    = j7-ros2-foxy:$(RSDK_VER)
WORK_DIR          = j7ros_home
REPO_NAME         = jacinto_ros_perception
WS_NAME           = ros_ws
WS_NAME_TGT       = $(WS_NAME)
ROS_DISTRO        = foxy
ROS_TAG           = ros2
EXCLUDE           = ros1

ifeq ($(ROS_VER), 1)
	ROS_DISTRO      = melodic
	DOCKER_TAG_TGT  = j7-ros-melodic:$(RSDK_VER)
	DOCKER_TAG_HOST = pc-ros-melodic:$(RSDK_VER)
	ROS_TAG         = ros1
	EXCLUDE         = ros2
endif

# set PLATFORM based on "uname -m"
PLATFORM          = j7
ifeq ($(shell uname -m), x86_64)
	PLATFORM      = pc
endif

# ROS1 specific
ARCH_HOST         = x86_64
ARCH_TGT          = arm64v8
DOCKER_FILE_TGT   = Dockerfile.$(ARCH_TGT).$(ROS_DISTRO)
DOCKER_FILE_HOST  = Dockerfile.$(ARCH_HOST).$(ROS_DISTRO)

WORK_PATH_HOST    = $(HOME)/$(WORK_DIR)
REPO_PATH         = $(WORK_PATH_HOST)/ros_ws/src/$(REPO_NAME)
ROS_SRC_DIR       = $(REPO_PATH)
DATA_DIR          = $(WORK_PATH_HOST)/data
DOCKER_DIR        = $(ROS_SRC_DIR)/docker
SCRIPTS_DIR       = $(ROS_SRC_DIR)/docker/scripts
DLRT_DIR          = /opt/dl_runtime

# Environment variable list to pass to the Docker container
ENV_FILE          = env_list.txt
ENV_FILE_PATH     = $(DOCKER_DIR)/$(ENV_FILE)

# Tarball filenames
ROSBAG_FILE       = ros-bag_2020_1109.tar.gz
DLRT_LIBS_FILE    = dlrt-libs-aarch64-ubuntu18.04_8.0.tar.gz
SEMSEG_MODEL_FILE = tidl-semseg-model_8.0.0.11.tar.gz
VISUAL_LOC_FILE   = visual-localization_8.0.0.10.tar.gz
LIDAR2D_FILE      = lidar2d_2021_0521.tar.gz
CALIB_IMGS_FILE   = calib-images-example_2021_0422.tar.gz

# Tarball URLs
URL_BASE         = https://software-dl.ti.com/jacinto7/esd/robotics-sdk/data
ROSBAG_URL   	 = $(URL_BASE)/$(ROSBAG_FILE)
DLRT_LIBS_URL    = $(URL_BASE)/$(DLRT_LIBS_FILE)
SEMSEG_MODEL_URL = $(URL_BASE)/$(SEMSEG_MODEL_FILE)
VISUAL_LOC_URL   = $(URL_BASE)/$(VISUAL_LOC_FILE)
LIDAR2D_LOC_URL  = $(URL_BASE)/$(LIDAR2D_FILE)
CALIB_IMGS_URL   = $(URL_BASE)/$(CALIB_IMGS_FILE)

# Semantic segmantation TVM-DLR model artifacts location
SEMSEG_MODEL     = tvmdlr_deeplabv3lite_mobilenetv2_tv_768x432_qat-p2_onnx
SEMSEG_ARTIFACTS_DIR = $(DATA_DIR)/tidl_semseg_model/$(SEMSEG_MODEL)/artifacts

# IP address
J7_IP_ADDR       := $(shell ifconfig | grep -A 1 'eth0' | tail -1 | awk '{print $$2}')

## Set up environments on J7 host linux
opkg_config:
	$(SCRIPTS_DIR)/config_opkg_repo.sh

ipk_install:
	$(SCRIPTS_DIR)/install_tivision_apps_ipk.sh

ip_show:
	@echo "$(J7_IP_ADDR)"

info:
	$(info PLATFORM         = $(PLATFORM))
	$(info DOCKER_FILE_TGT  = $(DOCKER_FILE_TGT))
	$(info DOCKER_FILE_HOST = $(DOCKER_FILE_HOST))
	$(info DOCKER_TAG_TGT   = $(DOCKER_TAG_TGT))
	$(info DOCKER_TAG_HOST  = $(DOCKER_TAG_HOST))
	$(info WORK_DIR         = $(WORK_DIR))
	$(info REPO_NAME        = $(REPO_NAME))
	$(info WORK_PATH_HOST   = $(WORK_PATH_HOST))
	$(info REPO_PATH        = $(REPO_PATH))
	$(info ROS_SRC_DIR      = $(ROS_SRC_DIR))
	$(info DATA_DIR         = $(DATA_DIR))
	$(info DOCKER_DIR       = $(DOCKER_DIR))
	$(info SCRIPTS_DIR      = $(SCRIPTS_DIR))

## Docker
docker_build:
	@echo "#!/bin/bash"                                       >  $(TGT_NAME)
	@echo "DOCKER_TAG=$(DOCKER_TAG)"                          >> $(TGT_NAME)
	@echo "DOCKER_DIR=$(DOCKER_DIR)"                          >> $(TGT_NAME)
	@echo "# modify the server and proxy URLs as requied"     >> $(TGT_NAME)
	@echo "ping bitbucket.itg.ti.com -c 1 > /dev/null 2>&1"   >> $(TGT_NAME)
	@echo "if [ \"\$$?\" -eq \"0\" ]; then"                   >> $(TGT_NAME)
	@echo "    USE_PROXY=1"                                   >> $(TGT_NAME)
	@echo "    REPO_LOCATION=artifactory.itg.ti.com/docker-public-arm" >> $(TGT_NAME)
	@echo "    HTTP_PROXY=http://webproxy.ext.ti.com:80"      >> $(TGT_NAME)
	@echo "else"                                              >> $(TGT_NAME)
	@echo "    USE_PROXY=0"                                   >> $(TGT_NAME)
	@echo "    REPO_LOCATION=$(ARCH)"                         >> $(TGT_NAME)
	@echo "fi"                                                >> $(TGT_NAME)
	@echo "cd $(WORK_PATH_HOST)"                              >> $(TGT_NAME)
	@echo "docker build \\"                                   >> $(TGT_NAME)
	@echo "    -t \$$DOCKER_TAG \\"                           >> $(TGT_NAME)
	@echo "    --build-arg USE_PROXY=\$$USE_PROXY \\"         >> $(TGT_NAME)
	@echo "    --build-arg REPO_LOCATION=\$$REPO_LOCATION \\" >> $(TGT_NAME)
	@echo "    --build-arg HTTP_PROXY=\$$HTTP_PROXY \\"       >> $(TGT_NAME)
	@echo "    -f \$$DOCKER_DIR/$(DOCKER_FILE) ./ros_ws/src"  >> $(TGT_NAME)
	@chmod +x $(TGT_NAME)
	@echo "$(TGT_NAME) is generated"

docker_run:
	@echo "#!/bin/bash"                                       >  $(TGT_NAME)
	@echo "DOCKER_TAG=$(DOCKER_TAG)"                          >> $(TGT_NAME)
	@echo "DOCKER_DIR=$(DOCKER_DIR)"                          >> $(TGT_NAME)
	@echo "IP_ADDR=\$$(ifconfig | grep -A 1 'eth0' | tail -1 | awk '{print \$$2}')" >> $(TGT_NAME)
	@echo "if [ \"\$$#\" -lt 1 ]; then"                       >> $(TGT_NAME)
	@echo "    CMD=/bin/bash"                                 >> $(TGT_NAME)
	@echo "else"                                              >> $(TGT_NAME)
	@echo "    CMD=\"\$$@\""                                  >> $(TGT_NAME)
	@echo "fi"                                                >> $(TGT_NAME)
	@echo "# modify the server and proxy URLs as requied"     >> $(TGT_NAME)
	@echo "ping bitbucket.itg.ti.com -c 1 > /dev/null 2>&1"   >> $(TGT_NAME)
	@echo "if [ \"\$$?\" -eq \"0\" ]; then"                   >> $(TGT_NAME)
	@echo "    USE_PROXY=1"                                   >> $(TGT_NAME)
	@echo "else"                                              >> $(TGT_NAME)
	@echo "    USE_PROXY=0"                                   >> $(TGT_NAME)
	@echo "fi"                                                >> $(TGT_NAME)
ifeq ($(PLATFORM), pc)
	@echo "xhost +local:$$USER"                               >> $(TGT_NAME)
endif
	@echo "docker run -it --rm \\"                            >> $(TGT_NAME)
	@echo "    -v $(WORK_PATH_HOST):/root/$(WORK_DIR) \\"     >> $(TGT_NAME)
	@echo "    -v /root/$(WORK_DIR)/$(WS_NAME_TGT)/src/${REPO_NAME}/${EXCLUDE} \\" >> $(TGT_NAME)
ifeq ($(PLATFORM), j7)
	@echo "    -v $(WORK_PATH_HOST)/.ros:/root/.ros \\"       >> $(TGT_NAME)
	@echo "    -v /usr:/host/usr:ro \\"                       >> $(TGT_NAME)
	@echo "    -v /opt/edge_ai_apps:/opt/edge_ai_apps \\"     >> $(TGT_NAME)
	@echo "    -v /opt/dl_runtime:/opt/dl_runtime \\"         >> $(TGT_NAME)
	@echo "    -v /opt/dlpack:/opt/dlpack \\"                 >> $(TGT_NAME)
	@echo "    -v /opt/tensorflow:/opt/tensorflow \\"         >> $(TGT_NAME)
	@echo "    -v /opt/onnxruntime:/opt/onnxruntime \\"       >> $(TGT_NAME)
	@echo "    -v /opt/model_zoo:/opt/model_zoo \\"           >> $(TGT_NAME)
endif
	@echo "    -v /dev:/dev \\"                               >> $(TGT_NAME)
	@echo "    --privileged \\"                               >> $(TGT_NAME)
	@echo "    --network host \\"                             >> $(TGT_NAME)
	@echo "    --env USE_PROXY=\$$USE_PROXY \\"               >> $(TGT_NAME)
ifeq ($(PLATFORM), j7)
	@echo "    --env TIVA_LIB_VER=$(TIVA_LIB_VER) \\"         >> $(TGT_NAME)
	@echo "    --env J7_IP_ADDR=\$$IP_ADDR \\"                >> $(TGT_NAME)
	@echo "    --env-file \$$DOCKER_DIR/$(ENV_FILE) \\"       >> $(TGT_NAME)
	@echo "    --device-cgroup-rule='c 238:* rmw' \\"         >> $(TGT_NAME)
endif
ifeq ($(PLATFORM), pc)
ifeq ($(GPUS), y)
	@echo "    --gpus all \\"                                 >> $(TGT_NAME)
	@echo "    --env=NVIDIA_VISIBLE_DEVICES=all \\"           >> $(TGT_NAME)
	@echo "    --env=NVIDIA_DRIVER_CAPABILITIES=all \\"       >> $(TGT_NAME)
endif
	@echo "    --env='DISPLAY' \\"                            >> $(TGT_NAME)
	@echo "    --env='QT_X11_NO_MITSHM=1' \\"                 >> $(TGT_NAME)
	@echo "    --volume='/tmp/.X11-unix:/tmp/.X11-unix:rw' \\">> $(TGT_NAME)
endif
	@echo "      \$$DOCKER_TAG \$$CMD"                        >> $(TGT_NAME)
ifeq ($(PLATFORM), pc)
	@echo "xhost -local:$$USER"                               >> $(TGT_NAME)
endif
	@chmod +x $(TGT_NAME)
	@echo "$(TGT_NAME) is generated"

scripts: scripts_$(PLATFORM)
scripts_$(PLATFORM): docker_build_$(PLATFORM) docker_run_$(PLATFORM)

docker_build_j7:
	@$(MAKE) docker_build TGT_NAME=docker_build_$(ROS_TAG).sh \
	                      DOCKER_FILE=$(DOCKER_FILE_TGT) \
	                      DOCKER_TAG=$(DOCKER_TAG_TGT) \
	                      ARCH=$(ARCH_TGT)

docker_run_j7:
	@$(MAKE) docker_run TGT_NAME=docker_run_$(ROS_TAG).sh \
	                    DOCKER_TAG=$(DOCKER_TAG_TGT) \

docker_build_pc:
	@$(MAKE) docker_build TGT_NAME=docker_build_$(ROS_TAG).sh \
	                      DOCKER_FILE=$(DOCKER_FILE_HOST) \
	                      DOCKER_TAG=$(DOCKER_TAG_HOST) \
	                      ARCH=$(ARCH_HOST)

docker_run_pc:
	@$(MAKE) docker_run TGT_NAME=docker_run_$(ROS_TAG).sh \
	                    DOCKER_TAG=$(DOCKER_TAG_HOST) \

## data files
data_download: dlrt_download semseg_download rosbag_download \
               vl_download lidar2d_download calib_download

semseg_download: check_data_dir
	@wget -O $(DATA_DIR)/$(SEMSEG_MODEL_FILE) "$(SEMSEG_MODEL_URL)"
	@rm -rf  $(DATA_DIR)/tidl_semseg_model
	@tar xzf $(DATA_DIR)/$(SEMSEG_MODEL_FILE) -C $(DATA_DIR)
	@rm -f $(DATA_DIR)/$(SEMSEG_MODEL_FILE)
	@cd $(SEMSEG_ARTIFACTS_DIR); \
	ln -snf deploy_lib.so.j7 deploy_lib.so; \
	ln -snf deploy_params.params.j7 deploy_params.params; \
	ln -snf deploy_graph.json.j7 deploy_graph.json; cd -

rosbag_download: check_data_dir
	@wget -O $(DATA_DIR)/$(ROSBAG_FILE) "$(ROSBAG_URL)"
	@rm -rf  $(DATA_DIR)/ros_bag
	@tar xzf $(DATA_DIR)/$(ROSBAG_FILE) -C $(DATA_DIR)
	@rm -f $(DATA_DIR)/$(ROSBAG_FILE)

vl_download: check_data_dir
	@wget -O $(DATA_DIR)/$(VISUAL_LOC_FILE) "$(VISUAL_LOC_URL)"
	@rm -rf  $(DATA_DIR)/visual_localization
	@tar xzf $(DATA_DIR)/$(VISUAL_LOC_FILE) -C $(DATA_DIR)
	@rm -f $(DATA_DIR)/$(VISUAL_LOC_FILE)

lidar2d_download: check_data_dir
	@wget -O $(DATA_DIR)/$(LIDAR2D_FILE) "$(LIDAR2D_LOC_URL)"
	@rm -rf  $(DATA_DIR)/lidar_2d
	@tar xzf $(DATA_DIR)/$(LIDAR2D_FILE) -C $(DATA_DIR)
	@rm -f $(DATA_DIR)/$(LIDAR2D_FILE)

dlrt_download:
	@rm -rf $(DLRT_DIR)
	@mkdir -p $(DLRT_DIR)
	@wget -O $(DLRT_DIR)/$(DLRT_LIBS_FILE) "$(DLRT_LIBS_URL)"
	@tar xzf $(DLRT_DIR)/$(DLRT_LIBS_FILE) -C $(DLRT_DIR)
	@rm $(DLRT_DIR)/$(DLRT_LIBS_FILE)

calib_download: check_data_dir
	@wget -O $(DATA_DIR)/$(CALIB_IMGS_FILE) "$(CALIB_IMGS_URL)"
	@rm -rf  $(DATA_DIR)/calib_imgs_example
	@tar xzf $(DATA_DIR)/$(CALIB_IMGS_FILE) -C $(DATA_DIR)
	@rm -f $(DATA_DIR)/$(CALIB_IMGS_FILE)

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
.PHONY: ip_show \
        semseg_download \
		rosbag_download \
		vl_download \
		lidar2d_download \
		dlrt_download\
		check_data_dir
