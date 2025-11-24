std_msgs/Header header
int32 id
float32[] keypoints   # [x0,y0,score0, ..., x16,y16,score16]
float32[] bbox        # [x,y,w,h] in pixels
float32 score         # detection score
