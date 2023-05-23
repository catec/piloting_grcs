#!/usr/bin/env bash

NVIDIA_DRIVERS="False"
usage() { echo "Usage: $0 [--nvidia_drivers=<True|False>]" 1>&2; exit 1; }

# --------------------------------------------------------------
function check_input_arguments()
{
    if [ $# -eq 0 ];
    then
        usage
        exit 0
    fi

    while [ $# -gt 0 ]; do
    case "$1" in
        --nvidia_drivers=*)
        NVIDIA_DRIVERS="${1#*=}"
        ;;
        *)
        echo "Error: Invalid arguments or format"
        usage
        exit 1
    esac
    shift
    done

    if [ $NVIDIA_DRIVERS != "True" ] && [ $NVIDIA_DRIVERS != "False" ];
    then
        echo "Invalid value defined to nvidia_drivers parameter"
        usage
        exit 1
    fi
}
# --------------------------------------------------------------

function configure_xserver()
{
    XAUTH=/tmp/.docker.xauth
    if [ ! -f $XAUTH ]
    then
        xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
        if [ ! -z "$xauth_list" ]
        then
            echo $xauth_list | xauth -f $XAUTH nmerge -
        else
            touch $XAUTH
        fi
        chmod a+r $XAUTH
    fi
}
# --------------------------------------------------------------

function main()
{
    check_input_arguments $@

    # If use nvidia drivers, check that nvidia-docker is installed.
    if [ $NVIDIA_DRIVERS == "True" ];
    then
        REQUIRED_PKG="nvidia-container-toolkit"
        PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
        echo Checking for $REQUIRED_PKG: $PKG_OK
        if [ "" = "$PKG_OK" ]; then
            echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
            bash -c "./install_nvidia-docker2.sh"
        fi
    fi

    # Check if container is already loaded
    CONTAINER_EXISTS=$(docker images | grep catecupia/piloting_grcs)
    if [ -z "$CONTAINER_EXISTS" ] 
    then
        echo "Loading docker image ..."
        docker load < piloting_grcs_latest.tar.gz
    fi

    # Make sure processes in the container can connect to the x server
    configure_xserver

    # Run docker image "catecupia/piloting_grcs" in container
    if [ $NVIDIA_DRIVERS == "True" ];
    then
        docker container run -it --rm \
        --gpus all \
        --security-opt apparmor:unconfined \
        --ipc host \
        --network host \
        --env="DISPLAY=$DISPLAY" \
        --env QT_X11_NO_MITSHM=1 \
        --env XAUTHORITY=$XAUTH \
        --volume "$XAUTH:$XAUTH" \
        --volume "/tmp/.X11-unix:/tmp/.X11-unix" \
        --volume "${HOME}/.pilotingGrcs:/root/.pilotingGrcs" \
        --volume "${HOME}/.config/FADA-CATEC:/root/.config/FADA-CATEC" \
        --volume "/etc/timezone:/etc/timezone:ro" \
        --volume "/etc/localtime:/etc/localtime:ro" \
        catecupia/piloting_grcs
    else
        docker container run -it --rm \
        --security-opt apparmor:unconfined \
        --ipc host \
        --network host \
        --env="DISPLAY=$DISPLAY" \
        --env QT_X11_NO_MITSHM=1 \
        --env XAUTHORITY=$XAUTH \
        --volume "$XAUTH:$XAUTH" \
        --volume "/tmp/.X11-unix:/tmp/.X11-unix" \
        --volume "${HOME}/.pilotingGrcs:/root/.pilotingGrcs" \
        --volume "${HOME}/.config/FADA-CATEC:/root/.config/FADA-CATEC" \
        --volume "/etc/timezone:/etc/timezone:ro" \
        --volume "/etc/localtime:/etc/localtime:ro" \
        catecupia/piloting_grcs
    fi
}
# --------------------------------------------------------------

main "$@"
