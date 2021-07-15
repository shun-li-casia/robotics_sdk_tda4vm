#!/bin/bash

#  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# set up TI Processor SDK environment
ln -snf /host/usr/lib/libtivision_apps.so.$TIVA_LIB_VER /usr/lib/libtivision_apps.so.$TIVA_LIB_VER
ln -snf /usr/lib/libtivision_apps.so.$TIVA_LIB_VER /usr/lib/libtivision_apps.so
ln -snf /host/usr/lib/libvx_tidl_rt.so /usr/lib/libvx_tidl_rt.so
ln -snf /host/usr/include/processor_sdk /usr/include/processor_sdk
ln -snf /host/usr/lib/libion.so /usr/lib/libion.so
ln -snf /host/usr/lib/libti_rpmsg_char.so.0 /usr/lib/libti_rpmsg_char.so.0
ln -snf /host/usr/lib/python3.8/site-packages/dlr/libdlr.so /usr/lib/libdlr.so
ln -snf /host/usr/include/dlr.h /usr/include/dlr.h
