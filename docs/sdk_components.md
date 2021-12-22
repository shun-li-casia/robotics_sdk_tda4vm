SDK Components
==============

## Processor SDK Linux
The Robotics SDK gets all the Linux kernel, filesystem, device-drivers and more from Processor SDK Linux

For more information visit [Processor SDK Linux Software Developer’s Guide](https://software-dl.ti.com/processor-sdk-rtos/esd/docs/latest/rtos/index_overview.html).

## Processor SDK RTOS
The Robotics SDK  gets all the HWA drivers, optimized libraries, OpenVx framework and more from Processor SDK RTOS

For more information visit [Processor SDK RTOS Getting Started Guide](https://software-dl.ti.com/processor-sdk-rtos/esd/docs/latest/rtos/index_overview.html).

In particular, the Robotics SDK has dependency on the following software components of the Processor SDK RTOS. These are deployed on TDA4 root filesystem with `/usr/lib/libtivision_apps.so` and header files under `/usr/include/processor_sdk`.

* [TI OpenVX](https://software-dl.ti.com/jacinto7/esd/processor-sdk-rtos-jacinto7/latest/exports/docs/tiovx/docs/user_guide/index.html)

* [Vision Apps Lib](https://software-dl.ti.com/jacinto7/esd/processor-sdk-rtos-jacinto7/latest/exports/docs/vision_apps/docs/user_guide/index.html)

* [Perception Tool Ki](https://software-dl.ti.com/jacinto7/esd/processor-sdk-rtos-jacinto7/latest/exports/docs/perception/docs/ptk_api_guide/index.html)

## Processor SDK Linux for Edge AI
The Robotics SDK gets deep-learning inference C++ framework and GStreamer plugins optimized on TDA4, and TI Edge AI Model Zoo.

For more information visit [Processor SDK Linux for Edge AI](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/latest/exports/docs/sdk_components.html)