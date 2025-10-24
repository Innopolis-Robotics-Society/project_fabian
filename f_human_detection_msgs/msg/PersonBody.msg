std_msgs/Header header
int32 id
float32[] keypoints   # [x1,y1,score1, ..., x133,y133,score133]
float32[] bbox        # [x,y,w,h] в пикселях входного кадра
float32 score         # усреднённая уверенность по точкам
