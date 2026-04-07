# 座標変換

UE5とRGL間の行列変換で遭遇した問題と解決策。

## 要点

- UE5 FMatrixは**行ベクトル規約**（V' = V × M）、RGLは**列ベクトル規約**（V' = M × V）
- 変換時に3x3回転部分の**転置**が必要
- UE5の平行移動はM[3][0..2]（Row 3）、RGLはColumn 3（Out[0..2][3]）
- RGLのレイ方向はmatrixの**Column 2**（ローカルZ軸）
- Yaw 180度ズレ: CPU `ray_cast`は`- HorizontalFov/2`で角度中心化するが、RGL版で欠落していた

## 詳細

### ToRGL変換（FTransform → rgl_mat3x4f）

```cpp
// 転置 + 平行移動の正しい取得
Out.value[Row][0] = M.M[0][Row]; // Column 0 = UE5 X-axis
Out.value[Row][1] = M.M[1][Row]; // Column 1 = UE5 Y-axis
Out.value[Row][2] = M.M[2][Row]; // Column 2 = UE5 Z-axis
Out.value[Row][3] = M.M[3][Row] * 0.01f; // Translation (cm→m)
```

### FromPitchYaw（レイ方向行列の構築）

FRotationMatrixの直接コピーは不可（Column 2がZ軸=上方向になる）。代わりに方向ベクトルを直接計算してColumn 2に配置:

```cpp
Dir = (cos(P)*cos(Y), cos(P)*sin(Y), sin(P))  // レイ方向
Column 0 = Right, Column 1 = Up, Column 2 = Dir
```

### Yaw角の中心化

CPU ray_cast: `HorizAngle = fmod(...) - HorizontalFov / 2`
RGL版にも同じオフセットが必要（360°の場合180°ズレ）。

### Y軸反転（UE5→ROS2）

CARLAの`ConvertToRosFormat`はY軸を`*= -1.0f`で反転。RGL ROS2 publishでは`Ue5ToRos2Node`（Y=-1スケール変換ノード）をFormatNodeの前に挿入。

### 距離単位

- UE5内部: cm
- RGL内部: m
- ROS2 publish: m（Autoware標準）
- LidarData（CARLA DataStream経由）: m に統一済み
