# Knowledge Summary

## ファイル一覧

| # | ファイル名 | トピック | 概要 |
|---|-----------|----------|------|
| 1 | 001_rgl-integration-architecture.md | RGL統合アーキテクチャ | CARLA UE5へのRobotecGPULidar統合の全体設計とフェーズ構成 |
| 2 | 002_coordinate-transform.md | 座標変換 | UE5↔RGL間の行列変換、行/列ベクトル規約、Yaw180度問題 |
| 3 | 003_mesh-extraction.md | メッシュ抽出 | GPU readback、Chaosコリジョン、レンダーメッシュの抽出戦略 |
| 4 | 004_ros2-publish.md | ROS2 publish | CARLA/RGL独立publish、PointXYZIRCAEDT形式、QoS設定 |
| 5 | 005_performance-optimization.md | 性能最適化 | レイキャッシュ、Transform最適化、ボトルネック計測結果 |
| 6 | 006_build-system.md | ビルドシステム | CMake/UBT統合、Shipping/Development、ログ出力方式 |
| 7 | 007_debugging-techniques.md | デバッグ手法 | クラッシュ対策、ログレベル、点群表示、比較スクリプト |
| 8 | 008_awsim-compatibility.md | AWSIM互換 | PointCloudFormatLibrary、RglQos、シーン管理の比較 |

## 全体概要

CarlaUE5にRobotecGPULidar(RGL)を統合し、GPUレイトレースによるLiDARセンサー模擬を実装した。Chaosコリジョンメッシュ＋GPU readbackフォールバックによるシーン構築、RGL ROS2拡張によるAutoware互換のpointcloud publish、Python APIによる柔軟な設定を実現。
