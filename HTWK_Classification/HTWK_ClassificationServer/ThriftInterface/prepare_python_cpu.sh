#!/bin/sh

####################################################################
## AADC 2017 deployment ############################################
####################################################################

# Setup python virtual environment for classification server (CPU)

#-------------------------------------------------------------------
#-- usage / help ---------------------------------------------------
usage()
{
    echo "Usage: prepare_python_cpu.sh TARGET_DIR"
    echo ""
    copyright
}

# show help and exit when running without parameters
if [ "${#}" -lt "1" ]; then
    usage
    exit 1
fi

#-------------------------------------------------------------------
#-- main -----------------------------------------------------------
echo ${1}

WORKING_DIR=`pwd`

cd "${1}"
virtualenv -p /usr/bin/python2.7 python_env
source python_env/bin/activate
pip install thrift==0.9.3
pip install opencv-python==3.2.0.8
pip install tensorflow
pip install keras
pip install pillow
pip install h5py
deactivate

cd "${WORKING_DIR}"

exit 0

