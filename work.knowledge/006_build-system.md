# ビルドシステム

CARLA UE5 + RGLのCMake/UBT統合とビルド構成。

## 要点

- `ENABLE_RGL` CMakeオプションでRGL有効化（Options.cmake）
- `WITH_RGL`プリプロセッサ定義はDefinitions.def経由でUBTに伝搬
- `libRobotecGPULidar.so`はBinaries/Linux/にコピーしてAddDynamicLibraryで登録
- Shipping/Developmentは別ターゲット（`package` / `package-development`）
- `UE_BUILD_SHIPPING`ガードでデバッグコードをShippingから除外

## 詳細

### ビルドターゲット

| ターゲット | 構成 | UE_LOG | RGLLog::Info | デバッグ計測 |
|-----------|------|--------|-------------|------------|
| `package` | Shipping | 全レベル無効 | 無効（空関数） | 無効 |
| `package-development` | Development | Warning以上（ファイル） | 有効（stdout） | 有効 |
| `launch` | Editor Development | 全レベル有効 | 有効 | 有効 |

### ログ出力方式

| 方式 | Shipping stdout | Development stdout | Development logファイル |
|------|----------------|-------------------|---------------------|
| UE_LOG | × | × | ○（Warning以上） |
| carla::log_warning | ○ (stderr) | ○ (stderr) | × |
| carla::log_info | × (NDEBUG) | × (NDEBUG) | × |
| RGLLog::Info (std::cout) | × (空関数) | ○ | ○ (tee経由) |
| RGL spdlog | ○ | ○ | × |

### ファイル構成

- `Definitions.def`: `WITH_RGL`定義（CMakeで自動生成 or 手動追加）
- `Libraries.def`: `libRobotecGPULidar.so`パス
- `Includes.def`: RGLヘッダ + ROS2拡張ヘッダのインクルードパス
- `Carla.Build.cs`: `WITH_RGL`検出 → AddDynamicLibrary + ROS2拡張インクルードパス追加
- `LibCarla/CMakeLists.txt`: `CARLA_RGL_DEFINITIONS`でcarla-serverにもWITH_RGL伝搬

### 注意点

- `.def`ファイルはBuild/Unreal/からPluginディレクトリにシンボリックリンク
- CMake `CARLA_WORKSPACE_PATH` = CarlaUE5/（RGLは親ディレクトリなので`../`が必要）
- UHTはUCLASS宣言を`#ifdef`内に配置できない → PIMPLパターンで回避
- Shipping/Developmentの同時ビルドは中間ファイル競合の可能性 → 順次実行
