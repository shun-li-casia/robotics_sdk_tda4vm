std_msgs/Header header

int32 width      # disparity width
int32 height     # disparity height

int32 numObjects # number of objects
ObjectPos[]      objPos             # objects' pos (bounding box) array

DriveSpace       drvSpace           # drivable space

int32            fsBoundaryLength   # down-sample factor in detection algorithm
int32[]          fsBoundary         # freespace boundary
