std_msgs/Header header        # stamp = lidar_end_time
float64[3]  pos
float64[4]  rot               # quaternion x y z w
float64[3]  vel               # world frame
float64[3]  bg
float64[3]  ba
float64[3]  grav              # world frame
float64[4]  offset_r_l_i      # quaternion x y z w
float64[3]  offset_t_l_i
float64[529] cov              # 23x23 row-major, order: pos, rot, offR, offT, vel, bg, ba, grav(2)