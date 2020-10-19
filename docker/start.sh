
# Based on:
# http://wiki.ros.org/docker/Tutorials/GUI


IMAGE_NAME=infdistopt


# Build docker image if not exists
if [ `docker images $IMAGE_NAME | wc -l` = 1 ]
then
    docker build `dirname $0` -t $IMAGE_NAME --network host
fi



xhost +local:root > /dev/null

docker run -it --rm \
    --env="DISPLAY" \
    --workdir="/home/$USER" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/dev/:/dev/" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network host \
    $IMAGE_NAME \
    "$@"

xhost -local:root > /dev/null

