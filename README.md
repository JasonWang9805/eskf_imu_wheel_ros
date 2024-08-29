# eskf_imu_wheel

这是用来融合 imu 和 wheel 的 ros 包。目前只使用了数据集进行测试。

## 话题消息

从 config 中进行更改话题名称

```
iwo_sub_topics:
  imu_topic: "/camera/imu"
  wheel_topic: "/odometry/filtered"
iwo_pub_topics:
  odom_topic: "/odom"
iwo_frame_topics:
  frame_id: "world"
```

## 配置参数路径地址

配置参数路径使用的是绝对路径，你需要在程序中将 FILENAME 改为自己的路径。

## 感谢

基于 高博 的 SLAM in Autonomous Driving book，其中把 imu 和 wheel 融合的部分剥离出来，并集成到 ros 系统中。
