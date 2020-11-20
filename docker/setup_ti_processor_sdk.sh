#!/bin/bash
# set up TI Processor SDK environment
ln -s /host/usr/lib/libtivision_apps.so.$TIVA_LIB_VER /usr/lib/libtivision_apps.so.$TIVA_LIB_VER
ln -s /usr/lib/libtivision_apps.so.$TIVA_LIB_VER /usr/lib/libtivision_apps.so
ln -s /host/usr/include/processor_sdk /usr/include/processor_sdk
ln -s /host/usr/lib/libion.so.1.0.0 /usr/lib/libion.so.1.0.0
ln -s /usr/lib/libion.so.1.0.0 /usr/lib/libion.so
ln -s /host/usr/lib/libgbm.so.1.0.0 /usr/lib/libgbm.so.1.0.0
ln -s /usr/lib/libgbm.so.1.0.0 /usr/lib/libgbm.so.1
ln -s /usr/lib/libgbm.so.1.0.0 /usr/lib/libgbm.so
ln -s /host/usr/lib/libti_rpmsg_char.so.0.2.0 /usr/lib/libti_rpmsg_char.so.0.2.0
ln -s /usr/lib/libti_rpmsg_char.so.0.2.0 /usr/lib/libti_rpmsg_char.so.0
ln -s /usr/lib/libti_rpmsg_char.so.0.2.0 /usr/lib/libti_rpmsg_char.so
