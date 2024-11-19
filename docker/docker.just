docker_img_ros_base := "ros:humble"
# docker_img_rosmicro := "aether/micro-ros"
docker_img_aether := "aether/aether"


[private]
docker-build-ros-base:
    docker pull {{docker_img_ros_base}}
    @echo "Docker image {{docker_img_ros_base}} pulled!"

# [private]
# docker-build-rosmicro: docker-build-ros-base
#     docker build -t {{docker_img_rosmicro}} \
#         --build-arg BASE_IMAGE={{docker_img_ros_base}} \
#         {{source_directory()}}/microros
#     @echo "Docker image {{docker_img_rosmicro}} built!"

[private]
docker-build-aether: docker-build-ros-base
    docker build -t {{docker_img_aether}} \
        --build-arg BASE_IMAGE={{docker_img_ros_base}} \
        {{source_directory()}}/aether
    @echo "Docker image {{docker_img_aether}} built!"
