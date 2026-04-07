# ROS2 Publish

CARLAとRGLの独立したROS2 publishの仕組みと設定。

## 要点

- CARLA built-in ROS2: `enable_for_ros()`で有効化、`ProcessDataFromLidar`→`ConvertToRosFormat`(Y反転)
- RGL ROS2: `rgl_node_points_ros2_publish_with_qos`でRGLグラフ内から直接publish
- 両者は独立制御（同一トピックでも別トピックでも可）
- `enable_for_ros()`はストリーミングサーバーの`IsEnabledForROS(StreamId)`フラグで制御
- PointXYZIRCAEDT形式（32bytes/point）でAutoware互換

## 詳細

### CARLA ROS2 publish制御

`PostPhysTick`内で`IsEnabledForROS(StreamId)`をチェック:
- `enable_for_ros()`未呼出 → publishしない
- `--ros2`グローバルフラグだけでは不十分（以前のバグ：グローバルフラグのみチェックしていた）

### RGL ROS2グラフ分岐

```
ToSensorNode → Ue5ToRos2Node(Y反転) → FormatNode → Ros2PublishNode
             → YieldNode (CARLA DataStream用、並列)
```

### PointCloud2フォーマット定義（AWSIM互換）

| フォーマット | サイズ | フィールド |
|-------------|--------|-----------|
| minimal | 20B | xyz, distance, intensity |
| pcl24 | 24B | xyz, pad, intensity, ring_id, pad |
| pcl48 | 48B | pcl24 + azimuth, distance, return_type, pad, timestamp |
| PointXYZIRCAEDT | 32B | xyz, intensity_u8, return_type_u8, ring_id_u16, azimuth, elevation, distance, timestamp_u32 |

### Python APIオプション

| オプション | 説明 | デフォルト |
|-----------|------|-----------|
| `--ros_topic_name` | 共通トピック名 | `/sensing/lidar/top/pointcloud_raw_ex` |
| `--ros_frame_id` | 共通frame_id | `velodyne_top` |
| `--enable_carla_ros2` | CARLA ROS2有効化 | off |
| `--rgl_ros2_topic` | RGL専用トピック上書き | (ros_topic_nameを使用) |
| `--rgl_ros2_reliability` | QoS reliability | `best_effort` |
| `--rgl_ros2_durability` | QoS durability | `volatile` |
| `--rgl_ros2_history` | QoS history | `keep_last` |
| `--rgl_ros2_history_depth` | QoS depth | 5 |
| `--rgl_ros2_format` | pointcloud形式 | `PointXYZIRCAEDT` |
