局部地图

1. 将VO匹配到的特征点放到地图中，并将当前帧与地图点进行匹配，计算位姿；

2. 每个帧为地图贡献一些信息，比如，添加新的特征点或更新旧特征点的位置估计，地图中的特征点位置往往是使 用世界坐标的；

3. 当前帧到来时，我们求它与地图之间的特征匹配与运动关系，即直接计算T_cw

4. 局部地图的一件麻烦事是维护它的规模。为了保证实时性，我们需要保证地图规模不至于太大(否者匹配会消耗大量的时间)


#usage: 
bin/run_vo config/default.yaml


                                                                                                                                                                                                                 