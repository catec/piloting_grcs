#!/bin/bash

DOCKER_BUILDKIT=1 docker build -t catecupia/piloting_grcs \
    --secret id=user,src=<( echo $USER ) \
    --secret id=password,src=<( echo $PASSWORD ) \
    .
