import os

script_dir = os.path.dirname(os.path.realpath(__file__))

CONFIG_HEADER = script_dir + '/../firmware/src/config.h'
HOST_COMM_HEADER = script_dir + '/../firmware/src/host_comm.h'
TARGET_COMM_HEADER = script_dir + '/../libdebug/src/include/libdebug/target_comm.h'
