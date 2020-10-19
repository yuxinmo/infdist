IMAGE_NAME=infdistopt



docker build --no-cache `dirname $0` -t $IMAGE_NAME --network host --rm=false
