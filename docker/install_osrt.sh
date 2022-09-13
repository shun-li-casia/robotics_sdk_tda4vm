#!/bin/bash
# Cut-down version of:
# https://github.com/TexasInstruments/edgeai-tidl-tools/blob/master/dockers/J721E/ubuntu_20.04/container_setup.sh

SCRIPTDIR=`pwd`

cd $HOME
if [ ! -d required_libs ];then
    mkdir required_libs
fi

if [ ! -d u_20_pywhl ];then
    mkdir u_20_pywhl
fi
cd u_20_pywhl

STR=`pip3 list | grep dlr`
SUB='dlr'
if [[ "$STR" != *"$SUB"* ]]; then
    wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/08_04_00_00/ubuntu20_04/pywhl/dlr-1.10.0-py3-none-any.whl
    pip3 install --upgrade --force-reinstall dlr-1.10.0-py3-none-any.whl
fi

STR=`pip3 list | grep onnxruntime-tidl`
SUB='onnxruntime-tidl'
if [[ "$STR" != *"$SUB"* ]]; then
    wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/08_04_00_00/ubuntu20_04/pywhl/onnxruntime_tidl-1.7.0-cp38-cp38-linux_aarch64.whl
    pip3 install onnxruntime_tidl-1.7.0-cp38-cp38-linux_aarch64.whl
fi

STR=`pip3 list | grep tflite-runtime`
SUB='tflite-runtime'
if [[ "$STR" != *"$SUB"* ]]; then
    wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/08_04_00_00/ubuntu20_04/pywhl/tflite_runtime-2.8.2-cp38-cp38-linux_aarch64.whl
    pip3 install --upgrade --force-reinstall tflite_runtime-2.8.2-cp38-cp38-linux_aarch64.whl
fi

cd $HOME
rm -r u_20_pywhl
if [  ! -d /usr/include/tensorflow ];then
    wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/08_04_00_00/ubuntu20_04/tflite_2.8_aragoj7.tar.gz
    tar xf tflite_2.8_aragoj7.tar.gz
    rm tflite_2.8_aragoj7.tar.gz
    mv tflite_2.8_aragoj7/tensorflow /usr/include
    mv tflite_2.8_aragoj7/tflite_2.8 /usr/lib/
    cp ~/tflite_2.8_aragoj7/libtensorflow-lite.a $HOME/required_libs/
    rm -r tflite_2.8_aragoj7
    cd $HOME
else
    echo "skipping tensorflow setup: found /usr/include/tensorflow"
    echo "To redo the setup delete: /usr/include/tensorflow and run this script again"
fi

if [  ! -d /usr/include/onnxruntime ];then
    wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/08_04_00_00/ubuntu20_04/onnx_1.7.0_u20.tar.gz
    tar xf onnx_1.7.0_u20.tar.gz
    rm onnx_1.7.0_u20.tar.gz
    cp -r  onnx_1.7.0_u20/libonnxruntime.so* $HOME/required_libs/
    cp -r  onnx_1.7.0_u20/libtidl_onnxrt_EP.so* $HOME/required_libs/
    mv onnx_1.7.0_u20/onnxruntime /usr/include/
    rm -r onnx_1.7.0_u20
    cd $HOME
else
    echo "skipping onnxruntime setup: found /usr/include/onnxruntime"
    echo "To redo the setup delete: /usr/include/onnxruntime and run this script again"
fi

if [  ! -d /usr/include/neo-ai-dlr ];then
    wget -q https://software-dl.ti.com/jacinto7/esd/tidl-tools/08_04_00_00/ubuntu20_04/dlr_1.10.0_u20.tar.gz
    tar xf dlr_1.10.0_u20.tar.gz
    rm dlr_1.10.0_u20.tar.gz
    cp -r  /usr/local/lib/python3.8/dist-packages/dlr/libdlr.so $HOME/required_libs/
    mv dlr_1.10.0_u20/neo-ai-dlr /usr/include/
    rm -r dlr_1.10.0_u20
    cd $HOME
else
    echo "skipping neo-ai-dlr setup: found /usr/include/neo-ai-dlr"
    echo "To redo the setup delete: /usr/include/neo-ai-dlr and run this script again"
fi

#symbolic link creation
cd /usr/lib/
if [  ! -f /usr/dlr/libdlr.so ];then
    mkdir /usr/dlr
    cp ~/required_libs/libdlr.so /usr/dlr/
fi

if [   -d $HOME/required_libs ];then
    cp -r $HOME/required_libs/* /usr/lib/
    ln -s /usr/lib/libonnxruntime.so /usr/lib/libonnxruntime.so.1.7.0
fi

rm -rf $HOME/required_libs

cd $SCRIPTDIR