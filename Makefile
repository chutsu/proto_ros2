DATA_DIR=/data
ROS2_WS=${HOME}/proto_ws
SHELL := /bin/bash

help: ## Help
	@echo -e "\033[1;34m[make targets]:\033[0m"
	@egrep -h '\s##\s' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-15s\033[0m %s\n", $$1, $$2}'

install_docker: ## Install Docker (Ubuntu only)
	@bash scripts/install_docker.bash

install_librealsense: ## Install librealsense (Ubuntu only)
	@bash scripts/install_librealsense.bash

build_docker:  ## Build proto_ros2 docker
	@cd docker && \
		docker build -t proto_ros2 \
			--build-arg USERNAME=${USER} \
			--build-arg USER_ID=`id -u` \
			--build-arg GROUP_ID=`id -g` .

run_docker: ## Run proto_ros2 docker
	@xhost +local:docker && docker run \
		-e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /tmp:/tmp \
		-v $(DATA_DIR):$(DATA_DIR) \
		-v $(PWD):/home/docker/proto_ros2 \
		-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' \
		-v /dev/shm:/dev/shm \
		-v /dev/dri:/dev/dri \
		-v $(HOME)/.cache:$(HOME)/.cache \
		--privileged \
		--ipc=host \
		--pid=host \
		--network="host" \
		-it --rm proto_ros2 /bin/bash

${HOME}/proto_ws/src/proto_ros2:
	@cd ${HOME} && mkdir -p ${HOME}/proto_ws/src && cd ${HOME}/proto_ws/src && ln -s ${PWD} .

build: ${HOME}/proto_ws/src/proto_ros2 ## Build proto_ws
	@cd ${ROS2_WS} && source /opt/ros/humble/setup.bash && colcon build --packages-select proto_ros2
