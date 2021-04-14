#!/bin/bash
# set up TI Processor SDK environment
ln -snf /host/usr/lib/libtivision_apps.so.$TIVA_LIB_VER /usr/lib/libtivision_apps.so.$TIVA_LIB_VER
ln -snf /usr/lib/libtivision_apps.so.$TIVA_LIB_VER /usr/lib/libtivision_apps.so
ln -snf /host/usr/include/processor_sdk /usr/include/processor_sdk
ln -snf /host/usr/lib/libion.so.1.0.0 /usr/lib/libion.so.1.0.0
ln -snf /usr/lib/libion.so.1.0.0 /usr/lib/libion.so
ln -snf /host/usr/lib/libti_rpmsg_char.so.0.3.0 /usr/lib/libti_rpmsg_char.so.0.3.0
ln -snf /usr/lib/libti_rpmsg_char.so.0.3.0 /usr/lib/libti_rpmsg_char.so.0
ln -snf /usr/lib/libti_rpmsg_char.so.0.3.0 /usr/lib/libti_rpmsg_char.so
ln -snf /host/usr/lib/libvx_tidl_rt.so.1.0 /usr/lib/libvx_tidl_rt.so.1.0
ln -snf /usr/lib/libvx_tidl_rt.so.1.0 /usr/lib/libvx_tidl_rt.so
ln -snf /host/usr/lib/python3.8/site-packages/dlr/libdlr.so /usr/lib/libdlr.so
ln -snf /host/usr/include/dlr.h /usr/include/dlr.h
