# publish this on /perception_output

string name # reference name. Maybe use random strings for now
float64 confidence # 0.0 - 1.0
uint32 shape
geometry_msgs/PoseStamped pose
float64 width # size meters
float64 height
float64 depth
std_msgs/ColorRGBA color
string relativ_tf_tree # use 'map' for now

# Shape values
uint32 MISC=0
uint32 BOX=1 # we will use BOX for now
uint32 CYLINDER=2
uint32 SPHERE=3
uint32 CONE=4
uint32 PLANE=5
