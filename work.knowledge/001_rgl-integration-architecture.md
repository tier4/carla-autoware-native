# RGL統合アーキテクチャ

CarlaUE5にRobotecGPULidar(RGL)を統合するための全体設計と段階的実装。

## 要点

- `sensor.lidar.rgl`としてCARLAセンサーレジストリに登録（既存`ray_cast`と共存）
- ASensor直接継承（ARayCastSemanticLidarは継承しない）→ PIMPLパターンでRGL依存を隠蔽
- RGLグラフ: UseRays → SetRange → SetRingIds → RaysTransform → Raytrace → Compact → ToSensor → Yield（＋ROS2分岐）
- `#ifdef WITH_RGL`で条件コンパイル、UHTの制約によりUCLASSは`#ifdef`の外に配置

## 詳細

### フェーズ構成

| Phase | 内容 | 状態 |
|-------|------|------|
| 1 | GPU頂点バッファreadbackでレンダーメッシュ取得 | 完了 |
| 2 | RGL GPUレイトレース（Chaosコリジョン主体） | 完了 |
| 3 | RGL ROS2拡張で直接publish | 完了 |

### 新規ファイル構成

```
Source/Carla/
  Sensor/RGLLidar.h       # ARGLLidarクラス（UCLASS、PIMPLパターン）
  Sensor/RGLLidar.cpp      # 実装（SimulateLidar、CollectResults、グラフ構築）
  RGL/RGLSceneManager.h    # FRGLSceneManager（シーン同期シングルトン）
  RGL/RGLSceneManager.cpp  # メッシュ抽出、エンティティ管理、Transform最適化
  RGL/RGLCoordinateUtils.h # 座標変換、RGLLog、RGL_CHECKマクロ
```

### 既存ファイル変更（最小限）

- `SensorRegistry.h`: ARGLLidar登録（`#ifdef WITH_RGL`ガード）
- `ActorBlueprintFunctionLibrary.cpp`: "rgl"ブランチ追加
- `Carla.Build.cs`: libRobotecGPULidar.so動的ロード
- `.def`ファイル: WITH_RGL定義、インクルード/ライブラリパス

### PostPhysTick最適化

- `sensor_tick`間隔（0.1s）をDeltaSecondsとして使用（物理ステップ0.01sではなく）
- レイパターンは初回のみ生成・キャッシュ（毎tick同一のため）
- CollectResultsはクライアントリスナーまたはデバッグ描画時のみ実行
- SerializeAndSendもクライアント不在時はスキップ
