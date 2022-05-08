# catkin
Original vins-fusion for validation some data. The related path has been revised to ourself path.
The robust function is important for the performance of VINS.
# Day 20210508 by xiwei: the VIO and Groundtruth are successfully aligned (the whampoaData-0521 validation) 
## 1. Evaluation of Whampoa data:UrbanNav-HK-Deep-Urban-1
### Left camera is used, set estimate_extrinsic: 1 (Have an initial guess about extrinsic parameters)
### The evaluation result as follows:
       max	15.405519
      mean	0.681131
    median	0.384471
       min	0.000118
      rmse	1.142427
       sse	2004.693594
       std	0.917169
- Trajectory
<p align="center">
  <img width="712pix" src="result/trajectoryLeft.png">
</p>

### Stereo camera is used, set estimate_extrinsic: 1,and set td: 0.05 
### Interestingly, we set the bag file start from second 1 as follows, then the data can initialize successfully, otherwise would fail
- rosbag  play -s 1 Whampoasensors.bag 
### The evaluation result as follows:
       max	2.372637
      mean	0.327725
    median	0.197800
       min	0.000059
      rmse	0.505415
       sse	392.107044
       std	0.384760
- Trajectory
<p align="center">
  <img width="712pix" src="result/trajectoryStereo.png">
</p>

## 2. Evaluation of TST data:UrbanNav-HK-Medium-Urban-1
### Stereo camera is used, set estimate_extrinsic: 1,and set td: 0.05 
### The evaluation result as follows:
       max	2.909680
      mean	0.480456
    median	0.314631
       min	0.000184
      rmse	0.726379
       sse	414.187215
       std	0.544783
- Trajectory
<p align="center">
  <img width="712pix" src="result/trajectoryTSTstereo.png">
</p>
