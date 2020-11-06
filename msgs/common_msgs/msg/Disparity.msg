std_msgs/Header header

int32 width              # disparity width
int32 height             # disparity height

int32 minDisp            # minimum disparity
int32 maxDisp            # maximum disparity

int32   step             # full row length in bytes
uint16[] data            # actual data 
