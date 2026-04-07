# AWSIM互換

AWSIMのRGL統合実装との比較と互換設計。

## 要点

- AWSIMの`PointCloudFormatLibrary`をCARLA側に移植（pcl24, pcl48, PointXYZIRCAEDT, minimal）
- AWSIMのRglQos設定（reliability, durability, history, depth）をPython APIで設定可能に
- AWSIMはレイパターン静的、CARLAは動的（回転シミュレーション）→ キャッシュで最適化
- AWSIMはコリジョンメッシュ使用 → CARLAもChaosコリジョン優先に変更（同等の低ポリアプローチ）

## 詳細

### RGLグラフ構造比較

| ノード | AWSIM | CARLA |
|--------|-------|-------|
| RaysFromMat3x4f | ✓ | ✓ |
| RaysSetRange | ✓ | ✓ |
| RaysSetRingIds | ✓ | ✓ |
| RaysSetTimeOffsets | ✓ | なし |
| RaysTransform | ✓ | ✓ |
| GaussianNoise (ray/hitpoint/distance) | ✓ (optional) | なし |
| Raytrace | ✓ | ✓ |
| CompactByField | ✓ | ✓ |
| PointsTransform (to sensor) | ✓ | ✓ |
| FormatNode + Ros2Publish | 外部(Unity) | RGLグラフ内蔵 |

### シーン管理比較

| 項目 | AWSIM | CARLA |
|------|-------|-------|
| シーン更新 | 毎フレーム全スキャン | 30フレームごと + インクリメンタル |
| Transform更新 | 全エンティティ毎フレーム | 静的スキップ + 変更検出 |
| メッシュ共有 | InstanceID + 参照カウント | TMapキャッシュ |
| 距離カリング | なし | なし（一時実装→削除） |
| メッシュソース | コリジョンメッシュ | Chaosコリジョン + GPU readbackフォールバック |

### PointXYZIRCAEDT形式（32バイト、Autoware標準）

```
XYZ_VEC3_F32(12) + INTENSITY_U8(1) + RETURN_TYPE_U8(1) + RING_ID_U16(2)
+ AZIMUTH_F32(4) + ELEVATION_F32(4) + DISTANCE_F32(4) + TIME_STAMP_U32(4) = 32
```

CARLAの`pointcloud_raw_ex`トピックと同一レイアウト。
