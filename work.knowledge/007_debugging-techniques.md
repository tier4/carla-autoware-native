# デバッグ手法

RGL LiDAR開発中に使用したデバッグ技法と落とし穴。

## 要点

- Shippingビルドでは`UE_LOG`も`carla::log_info`も出力されない → `RGLLog::Info`（std::cout）を使用
- `PersistentLineBatcher->DrawPoint`はShippingでも動作（`DrawDebugPoint`は無効化される）
- `sensor.listen()`はPython側からのコールバックでsegfault → C++側のDrawに変更
- `rgl_graph_run`失敗時はRGLの`rgl_get_last_error_string`でエラー内容を取得

## 詳細

### クラッシュ原因と対策

| 症状 | 原因 | 対策 |
|------|------|------|
| SIGSEGV at VertexPosition() | bAllowCPUAccess=falseでData pointer null | bAllowCPUAccessチェック + GPU readbackフォールバック |
| SIGSEGV at InitializeFromWorld | 一部メッシュのrendered data未初期化 | Stride==0チェック、index bounds検証 |
| rgl_graph_run失敗 | RING_ID_U16をFormatNodeで要求するがSetRingIdsノード未設定 | SetRingIdsノード追加 or FormatNodeからRING_IDを除去 |
| Python listen()でsegfault | ストリーミングスレッドからのCARLA API呼び出し | C++側のPersistentLineBatcherでの描画に変更 |

### ログ出力の選択フロー

1. 開発中はDevelopmentビルド + `RGLLog::Info`で十分
2. Shippingビルドではログなし（`#if !UE_BUILD_SHIPPING`ガード）
3. RGLライブラリ自体のログ（spdlog）は全ビルドで出力される

### 点群比較スクリプト

`compare_lidar_topics.py`: ros2 topic echo出力をファイルに保存し、タイムスタンプ50ms tolerance でマッチング比較。width/point_step/row_step/is_denseを自動比較。

### タイミング計測

`FPlatformTime::Seconds()`でrgl_graph_runとCollectResultsの所要時間を個別計測。`#if !UE_BUILD_SHIPPING`でShippingから除外。

### デバッグ点群表示

- `--rgl_show_lidar_points`: PersistentLineBatcher経由で描画（Shipping含む全ビルドで動作）
- `--rgl_lidar_draw_point_rate`: 表示点数の割合（0.0-1.0）
- `--rgl_lidar_draw_life_time`: 各点の表示秒数
