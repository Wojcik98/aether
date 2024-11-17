docker_img_ros_base := "ros:humble"
docker_img_espidf := "aether/espidf"
docker_img_rosmicro := "aether/micro-ros"
docker_img_aether := "aether/aether"

espidf_version := "v5.2.3"
espidf_target := "esp32s3"

[private]
docker-build-ros-base:
    docker pull {{docker_img_ros_base}}
    @echo "Docker image {{docker_img_ros_base}} pulled!"

[private]
docker-build-espidf: docker-build-ros-base
    docker build -t {{docker_img_espidf}} \
        --build-arg BASE_IMAGE={{docker_img_ros_base}} \
        --build-arg IDF_CLONE_BRANCH_OR_TAG={{espidf_version}} \
        --build-arg IDF_INSTALL_TARGETS={{espidf_target}} \
        {{source_directory()}}/espidf
    @echo "Docker image {{docker_img_espidf}} built!"

[private]
docker-build-rosmicro: docker-build-espidf
    docker build -t {{docker_img_rosmicro}} \
        --build-arg BASE_IMAGE={{docker_img_espidf}} \
        {{source_directory()}}/microros
    @echo "Docker image {{docker_img_rosmicro}} built!"

[private]
docker-build-aether: docker-build-rosmicro
    docker build -t {{docker_img_aether}} \
        --build-arg BASE_IMAGE={{docker_img_rosmicro}} \
        {{source_directory()}}/aether
    @echo "Docker image {{docker_img_aether}} built!"
