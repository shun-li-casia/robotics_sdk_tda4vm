Semantic Segmentation Map Visualization
=======================================

This application subscribes to two topics, one is an input image topic, on which class dependent colors will be overlaid, and the other is a semantic segmentation CNN output tensor topic, which has class information for all pixels. It creates a color-coded semantic segmentation map from them, which is published as an output.


`ti_viz_semseg` directory structure is shown below:
```
ti_viz_semseg
.
├── CMakeLists.txt
├── launch
│   └── viz_semseg.launch
├── package.xml
├── README.md
└── src
    └── viz_semseg_node.cpp

```

We can run this application by launching viz_semseg.launch file, i.e.,
```
roslaunch ti_viz_semseg viz_semseg.launch
```
viz_semseg.launch file specifies the followings:
* Input image topic name to read input images.
* CNN tensor topic name to read output tensors from a CNN network.
* Output topic name to publish a color-coded semantic segmentation map, which can be displayed directly by RViz.

