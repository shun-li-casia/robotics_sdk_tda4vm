Disparity Map Visualization
===========================

This application subscribes to a raw disparity map topic, which is published by ti_sde, creates a color-coded disparity map and a confidence map from the raw disparity map and publish them.


`ti_viz_disparity` directory structure is shown below:
```
ti_viz_disparity
.
├── CMakeLists.txt
├── launch
│   └── viz_disparity.launch
├── package.xml
├── README.md
└── src
    └── viz_disparity_node.cpp

```

We can run this application by launching viz_disparity.launch file, i.e.,
```
roslaunch ti_viz_disparity viz_disparity.launch
```
viz_disparity.launch file specifies the followings:
* Input topic name to read a raw disparity map.
* Output topic names to publish a color-coded disparity map and confidence map, respectively, which can be dispalyed directly by RViz. 

