### RRT_cs_probability_field_sampling
RRT_cs is an algorithm combine RRT_connect and RRT_star. Probability field sampling is to sample the random point according to probability field. Probability field is generated by the environment (goal and obstacles). The nearer to the goal the point is, the more likely it is to be chosen. The nearer to the obstacles the point is, the less likely it is to be chosen. Probability field sampling is just like pheromone in the biological world. It is suitable in the dense environment.  
#### 2D world
Main file: rrtcs_pfs_2d/main.m

![Image discription](https://github.com/YixiaoWang7/RRT_cs_probability_field_sampling/tree/master/rrtcs_pfs_2d/Magnification_#1_when_iteration=1000.jpg)

![Image discription](https://github.com/YixiaoWang7/RRT_cs_probability_field_sampling/tree/master/rrtcs_pfs_2d/Magnification #2 when iteration = 1000.jpg)

![Image discription](https://github.com/YixiaoWang7/RRT_cs_probability_field_sampling/tree/master/rrtcs_pfs_2d/Magnification #1 when iteration = 2000.jpg)

![Image discription](https://github.com/YixiaoWang7/RRT_cs_probability_field_sampling/tree/master/rrtcs_pfs_2d/Magnification #2 when iteration = 2000.jpg) 

#### 3D world
Main file: rrtcs_pfs_3d/main.m

![Image discription](https://github.com/YixiaoWang7/RRT_cs_probability_field_sampling/tree/master/rrtcs_pfs_3d/Path.jpg) 

#### path planning for UR robotic arm
Main file: rrtcs_pfs_2d/main.m

![Image discription](https://github.com/YixiaoWang7/RRT_cs_probability_field_sampling/tree/master/rrtcs_robotarm/trajectory.jpg) 

It should be noticed that probability field sampling is not realized here.
