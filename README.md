### RRT_cs_probability_field_sampling
RRT_cs is an algorithm combine RRT_connect and RRT_star. Probability field sampling is to sample the random point according to probability field. Probability field is generated by the environment (goal and obstacles). The nearer to the goal the point is, the more likely it is to be chosen. The nearer to the obstacles the point is, the less likely it is to be chosen. Probability field sampling is just like pheromone in the biological world. It is suitable in the dense environment.  
#### 2D world
Main file: rrtcs_pfs_2d/main.m

final_path.jpg

Magnification #1 when iteration = 1000.jpg

Magnification #2 when iteration = 1000.jpg

Magnification #1 when iteration = 2000.jpg

Magnification #2 when iteration = 2000.jpg

#### 3D world
Main file: rrtcs_pfs_3d/main.m

Path.jpg
#### path planning for UR robotic arm
Main file: rrtcs_pfs_2d/main.m
    
trajectory.jpg
It should be noticed that probability field sampling is not realized here.