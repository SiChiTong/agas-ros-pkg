string object_name
string object_type

sensor_msgs/Image image
int32 image_index
string image_name
Point2D[] outline
Point2D[] b_box
Point2D center
int32 bounding_box_index

uint32[] key_point_index_map
KeyPoint[] object_key_points

KeyPointMatch[] stage1_matches
KeyPointMatchArray[] stage2_matches
KeyPointMatch[] stage3_matches
KeyPoint[] scene_key_points_within_outline

float64[9] homography
