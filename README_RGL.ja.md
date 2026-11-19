RobotecGPULidar (RGL) を使う CARLA
==================================

[English](README_RGL.md)

この文書は、[RobotecGPULidar (RGL)](https://github.com/RobotecAI/RobotecGPULidar)
を組み込んだ CARLA の build と実行の方法を説明する。RGL は、GPU で ray tracing を行う
LiDAR シミュレーションのライブラリである。手順は `RglSetup.sh` が進める。`RglSetup.sh`
は標準の `CarlaSetup.sh` の流れを補い、RGL とその依存を clone・build して組み込む手順を
受け持つ。

> [!NOTE]
> RGL の組み込みには、host に NVIDIA の GPU、CUDA、NVIDIA OptiX SDK 7.2 と
> ROS 2 (Ubuntu 22.04 では Humble、Ubuntu 24.04 では Jazzy) が要る。GPU LiDAR の pipeline は、
> `Unreal/CarlaUnreal/Plugins/CarlaRGL/ThirdParty/RclcppBridge` にある補助ライブラリ
> `RclcppBridge` も使う。CARLA と RGL の間で、rclcpp/DDS の domain の持ち主を調整するためである。

## 推奨の環境

* Intel i7 第 9〜11 世代 / Intel i9 第 9〜11 世代 / AMD Ryzen 7 / AMD Ryzen 9
* RAM 32 GB 以上
* NVIDIA RTX 3070 / 3080 / 3090 / RTX 4090 / RTX 5090 以上
* VRAM 16 GB 以上
* Ubuntu 22.04 または Ubuntu 24.04
* CUDA Toolkit 12.x (Ubuntu 24.04 では 12.9 で試した。そこでは RGL に 12.6 以上が要る)
* NVIDIA OptiX SDK 7.2
* ROS 2 Humble (Ubuntu 22.04) または ROS 2 Jazzy (Ubuntu 24.04)。加えて
  `ros-<distro>-radar-msgs`、`ros-<distro>-fastrtps`、
  `ros-<distro>-rmw-fastrtps-cpp`、`ros-<distro>-cyclonedds`、
  `ros-<distro>-rmw-cyclonedds-cpp`、`python3-colcon-common-extensions` と
  `patchelf`

> [!NOTE]
> Windows には、まだ `RglSetup.sh` が対応していない — **TBD**。

## 作業の流れ

RGL の組み込みは 3 段の流れで行う。2 回の `RglSetup.sh` の間に `CarlaSetup.sh` を流す。

```
1. bash RglSetup.sh prepare [OPTIONS]   # Check prerequisites, clone+build RGL
2. bash CarlaSetup.sh -i                # Standard CARLA setup
3. bash RglSetup.sh build  [OPTIONS]    # Reconfigure CARLA with RGL, rebuild
```

段 1 は、RGL のライブラリと、colcon で build する依存 (`agnocast`、`radar_msgs` など)
を用意する。段 2 は、標準の CARLA の setup (Unreal Engine の download、content、最初の
build) を行う。段 3 は、CMake を `ENABLE_RGL=ON` で構成し直して CARLA を build し直し、
CarlaRGL の plugin を link する。

## RGL を組み込んだ CARLA の build

### 段 1 — `prepare`

CARLA の root の dir (`CarlaUE5/`) で流す。

```sh
cd CarlaUE5
bash RglSetup.sh prepare --optix-dir=/path/to/NVIDIA-OptiX-SDK-7.2.0-linux64-x86_64
```

知っておくとよい option:

* `--ros-distro=humble|jazzy` は ROS 2 の distribution を選ぶ。既定は、ROS の環境を
  source してあれば `$ROS_DISTRO`、そうでなければ Ubuntu 22.04 で Humble、
  Ubuntu 24.04 で Jazzy。`RglSetup.sh build` にも同じ値を渡す。
* `--cuda-dir=PATH` は、`nvcc` が `PATH` に無い CUDA Toolkit を指す。例: home の dir の
  下に toolkit だけを入れた場合。
* `--no-pcl` は PCL の extension を飛ばし (CARLA の RGL plugin は使わない)、長い vcpkg の
  build を省く。

することは次のとおり。

1. CUDA、OptiX SDK、ROS 2 (選んだ distro) と `patchelf` があることを確かめる。
2. `RobotecGPULidar/` を `CarlaUE5/` の横に clone する (git の branch: `develop`)。
3. `/opt/ros/<distro>/setup.bash` を source し、RGL core・PCL・ROS 2・Agnocast の依存を
   `python3 setup.py --install-*-deps` で入れる。
4. 選んだ extension つきで RGL を build し、
   `RobotecGPULidar/build/lib/libRobotecGPULidar.so` を作る。

> [!NOTE]
> `--optix-dir=` を渡す代わりに、環境変数 `OptiX_INSTALL_DIR` を設定してもよい。

### 段 2 — `CarlaSetup.sh`

標準の CARLA の setup を流し、Unreal Engine 5 と content を download して、最初の
CARLA の build を作る。

```sh
bash CarlaSetup.sh -i
```

`CarlaSetup.sh` の使い方の全体は [README.md](README.md) を見る。

> [!IMPORTANT]
> ここで `CarlaSetup.sh -l` は使わない。RGL *なしで* editor を起動してしまう。
> RGL が有効になるのは段 3 の後である。

### 段 3 — `build`

CMake を `ENABLE_RGL=ON` で構成し直し、build し直す。

```sh
bash RglSetup.sh build                       # rebuild only
bash RglSetup.sh build --package=shipping    # + build Shipping package
bash RglSetup.sh build --package=development # + build Development package
bash RglSetup.sh build --package=launch      # + open UE5 editor with RGL
```

script は次を行う。

1. `Build/` の下の古い CMake の cache を見つけて消す
   ([古い cache の検出](#古い-cache-の検出) を見る)。
2. `cmake -G Ninja -S . -B Build --toolchain=$PWD/CMake/Toolchain.cmake
   -DENABLE_ROS2=ON -DENABLE_RGL=ON …` を流す。ROS 2 の middleware は server の起動時に
   選ぶ (既定は `-rmw=fastdds`、CycloneDDS なら `-rmw=cyclonedds`)。
3. `cmake --build Build` を呼び、頼まれていれば package の target も呼ぶ。

`RclcppBridge` は `Build/` の下の ExternalProject として、UE の toolchain でなく system の
compiler で build する。link する `rclcpp` は `$AMENT_PREFIX_PATH` から取る。無ければ
`/opt/ros/$ROS_DISTRO` から (`RglSetup.sh build` が `--ros-distro` から設定する)、それも
無ければ `/opt/ros/jazzy` と `/opt/ros/humble` のうち入っている方から取る。RGL を prepare
したときと同じ distro を使う。

## CARLA の build し直し (手で)

`RglSetup.sh build` が一度通った後は、標準の CMake の command で build し直せる。

```sh
export CARLA_UNREAL_ENGINE_PATH=$(realpath ../UnrealEngine5_carla)
cmake --build Build                              # rebuild
cmake --build Build --target package             # Shipping package
cmake --build Build --target package-development # Development package
cmake --build Build --target launch              # UE5 editor
```

## package した build を動かす

```sh
cd Build/Package/Carla-*-Linux-Shipping
./CarlaUnreal.sh --ros2
```

## command の一覧

### `prepare` の option

| Option | 説明 |
|---|---|
| `--optix-dir=PATH`       | OptiX SDK の path (または `OptiX_INSTALL_DIR` を設定する) |
| `--cuda-dir=PATH`        | `nvcc` が `PATH` に無いときの CUDA Toolkit の path |
| `--ros-distro=NAME`      | ROS 2 の distro、`humble` か `jazzy` (既定: `$ROS_DISTRO`、無ければ Ubuntu の版で決める) |
| `--rgl-branch=BRANCH`    | RGL の git の branch (既定: `develop`) |
| `--rgl-repo=URL`         | RGL の git の repository の URL |
| `--skip-rgl-build`       | RGL の build を飛ばす (build 済みのライブラリを使う) |
| `--no-pcl`               | PCL の extension を切る |
| `--no-ros2-standalone`   | ROS 2 standalone の extension を切る |
| `--no-agnocast`          | Agnocast の extension を切る |
| `--with-weather`         | weather の extension を入れる (private の repo) |
| `--with-udp`             | UDP の extension を入れる (private の repo) |
| `--ext-patch-dir=DIR`    | clone の後に `DIR/*.patch` を UDP の extension に当てる。`--with-udp` のときだけ ([UDP Raw Packet extension](#udp-raw-packet-extension-任意) を見る) |

### `build` の option

| Option | 説明 |
|---|---|
| `--package=shipping`    | build し直した後に、CARLA の Shipping の package を build する |
| `--package=development` | build し直した後に、CARLA の Development の package を build する |
| `--package=launch`      | build し直した後に、UE5 の editor を起動する |
| `--package=none`        | (既定) build し直すだけで、package の段は無い |
| `--ros-distro=NAME`     | RclcppBridge の build に使う ROS 2 の distro。RGL を prepare したときと同じものを使う |
| `--no-clear-ddc`        | `--package=launch` のときの、DDC の自動の削除を飛ばす |

## 古い cache の検出

`RglSetup.sh` は、workspace の名前を変えた・写した・CMake の `SOURCE_DIR` が変わった、の
どれかで生じる古い build の産物を、3 種類見つける。

| 検出器 | 条件 | 処置 |
|---|---|---|
| `verify_cmake_cache_path`              | 最上位の `CMAKE_HOME_DIRECTORY` が食い違う | `build_dir` を消す |
| `verify_cmake_subcache_sources_exist`  | ExternalProject の下位の cache が、無い dir を指す | その下位の build だけを消す |
| `verify_colcon_install_path`           | colcon の `setup.sh` に、前の workspace の path が焼き込まれている | `build/`、`install/`、`log/` を消す |

これらは自動で呼ばれる。

- `prepare`: `RobotecGPULidar/build/` と、`RobotecGPULidar/external/*/` と
  `RobotecGPULidar/extensions/*/` の下の colcon の workspace を全部調べる。
- `build`: `CarlaUE5/Build/` と、その下の ExternalProject の下位の cache を全部調べる。

flag は要らない。workspace を動かしても、そのまま扱える。

## package の圧縮

### 圧縮が走るとき

`package*` の target が、stage した build を圧縮するかどうかは、上流の option
`CARLA_UNREAL_PACKAGE_NO_COMPRESSION` が決める。その既定は build type で決まる
(上流の CARLA から受け継いだもので、RGL 固有ではない)。

| `CARLA_UNREAL_PACKAGE_BUILD_TYPE` | `NO_COMPRESSION` の既定 | 振る舞い |
|---|---|---|
| `Shipping`       | `OFF` | 圧縮が走る |
| その他 (Development、Debug、…) | `ON`  | 圧縮を飛ばす |

`-DCARLA_UNREAL_PACKAGE_NO_COMPRESSION=ON|OFF` で上書きする。

### 圧縮の方式

圧縮が走るときの方式は、`CARLA_PACKAGE_COMPRESSION` で選べる (既定: `pigz` — 並列の
gzip。CPU の core の数を自動で見つける)。

```sh
cmake ... -DCARLA_PACKAGE_COMPRESSION=pigz   # .tar.gz, parallel gzip (default)
cmake ... -DCARLA_PACKAGE_COMPRESSION=zstd   # .tar.zst, parallel zstd (pzstd)
cmake ... -DCARLA_PACKAGE_COMPRESSION=gzip   # .tar.gz, single-threaded (compatibility)
```

`pigz` と `zstd` は、どちらも CPU の core の数を自動で見つけて並列に動く。
`apt install pigz` か `apt install zstd` で入れる。

## 困ったとき

### `agnocastlib` が見つからない

```
Could not find a package configuration file provided by "agnocastlib" …
```

Agnocast の colcon の install が、別の workspace のものである。`prepare` がこれを見つけて
自動で消すので、`bash RglSetup.sh prepare` を流し直す。

### `CMakeCache.txt directory is different than the directory …`

build の dir が、別の workspace で作られている。`prepare` / `build` がこれを自動で見つけて
cache を消すので、同じ command を流し直す。

### CycloneDDS の build での iceoryx の link の error

この fork で、CycloneDDS の ExternalProject に `-DENABLE_SHM=OFF` を設定して直してある。
UE の toolchain は libc++ を使うが、CycloneDDS は、libstdc++ で build された system の
iceoryx を、もう拾わない。

### 起動時の editor の crash (Nanite / SkeletalMesh の assertion)

症状 — `cmake --build Build --target launch` の少し後に editor の process が abort し、
次のような message が出る。

```
Assertion failed: ExternalEdgeOffset == ExternalEdges.Num()
  [File:./Developer/NaniteBuilder/Private/ClusterDAG.cpp] [Line: 126]

  Stack: Nanite::BuildDAG ← FSkeletalMeshBuilder::Build
       ← USkeletalMesh::ExecutePostLoadInternal
```

根本の原因 — project の `DerivedDataCache/` (DDC) に、前の build が作った Nanite / asset の
産物が残っていて、その内部の配置が今のコードと合わない。上流との同期、branch の切り替え、
asset の post-load の経路に触る rebase の後によく起きる。DDC は内部の cache で、周りの
CARLA/UE5 のコードが変わっても、自分では無効にならない。

回復 — project の DDC を消し、次の起動で UE5 に作り直させる。

```sh
# 1. Kill any lingering editor / crash reporter
pkill -9 -f UnrealEditor
pkill -9 -f CrashReportClient

# 2. Remove the project DDC (typically <100 MB on a cold build)
rm -rf Unreal/CarlaUnreal/DerivedDataCache

# 3. Relaunch — initial start is slow (~5–10 min) while DDC is rebuilt
cmake --build Build --target launch
```

`RglSetup.sh build --package=launch` はこれを自動で行う。前回の起動の git の HEAD を
`Build/.last-launch-head` に覚えておき、HEAD が動いていれば DDC を消す。切り替えをまたいで
温まった cache を残したいなら、`--no-clear-ddc` で止める。この見立てが外れるとき (例:
asset の振る舞いを変える、commit していない手元の変更) は、上の手順で手で消す。

project の DDC を消しても crash が直らなければ、次も見る。

- engine で共有する DDC (まれ): `~/Documents/Unreal Engine/Common/DerivedDataCache/`
- engine の内部の DDC (まれ): `UnrealEngine5_carla/Engine/DerivedDataCache/`

どちらも消して安全である。作り直しは長くなる。

## Windows

**TBD** — `RglSetup.sh` は今は Bash の script で、Linux (Ubuntu 22.04) を対象にする。
Windows の相当品 (例: `RglSetup.bat` や PowerShell) は、まだ実装していない。

## 関連

- [README.md](README.md) — 標準の CARLA UE5 の build の手順。
- [RobotecGPULidar](https://github.com/RobotecAI/RobotecGPULidar) — 上流の RGL の
  repository。

## UDP Raw Packet extension (任意)

AWSIM 互換の UDP Raw Packet を出すには、private の `RobotecAI/RGL-extension-udp`
extension つきで RGL を build する。`RglSetup.sh` にはそのための `--with-udp` flag がある。
手順の全体、Python の recipe、属性の一覧、model ごとの return mode の whitelist は
[`Docs/rgl/udp_raw_packets.md`](Docs/rgl/udp_raw_packets.md) を見る。

CARLA 自体は、UDP の extension なしでも build できる。その場合、UDP に関わる機能は実行時に
警告の log を出して自動で無効になり、RGL の pipeline の残り (ROS 2 の publish、Yield) は
影響を受けない。

> [!IMPORTANT]
> UDP の extension には、RobotecAI の repository にまだ入っていない直しが要る。直しは、
> この公開の repository の外に patch として置いてある (patch には、private の extension の
> source の文脈の行が入るため)。`--with-udp` と一緒に、`--ext-patch-dir=DIR` で当てる。
>
> ```bash
> bash RglSetup.sh prepare --with-udp --ext-patch-dir=/path/to/udp-patches ...
> ```
>
> `--with-udp` だけでも、load できて packet を送るライブラリは build できる。ただし、その
> packet は下流で受け付けられないことがある (例: Autoware が engage しない)。
> `RglSetup.sh` は 2 つの build を見分けられないので、UDP の extension を build するときは
> 必ず patch の dir を渡す。RobotecAI が直しを出したら、patch は何もしなくなり (当て済みの
> patch は飛ばす)、`--ext-patch-dir` は外せる。`--with-udp` なしの `--ext-patch-dir` は、
> 警告を出して無視される。

## tier4/nebula での Phase 2 の検証

shipping の package を build し、load される `libRobotecGPULidar.so` に UDP の extension が
入ったら、Phase 2 の end-to-end の試験を流す。対応する 8 つの LiDAR の model を、本番の
ROS 2 の driver `tier4/nebula` で確かめる。

```bash
source /opt/ros/<distro>/setup.bash   # humble or jazzy
source /path/to/colcon_workspace/install/setup.bash

cd PythonAPI/rgl/tests
python3 test_phase2_nebula_e2e.py
```

試験は CARLA を自動で起動し、model ごとの Nebula の config の YAML を `/tmp` に作り、
model ごとに Nebula の process を 1 つずつ順に動かして、最後に `N/8 passed` の要約を出す。
前提、確かめる指標、return mode の対応、困ったとき (HesaiPandarQT 固有の
`enable_hesai_udp_sequence` + `ensure_hesai_pandar_driver_compat` の要件を含む) は
[`Docs/rgl/phase2_nebula_e2e.md`](Docs/rgl/phase2_nebula_e2e.md) を見る。

## スケルタルメッシュ (車体と歩行者)

2026-09 から、RGL の scene の同期は、pawn が持つ `USkeletalMeshComponent` (CARLA の車両と
歩行者) も登録する。rest pose の頂点、4 つの影響の bone の重み、bind pose の逆行列は、
(mesh, LOD) ごとに 1 回だけ上げる。毎 tick 送るのは bone の world の行列だけで
(`rgl_entity_set_pose_world`)、RGL が GPU で skinning する。

| CVar | 既定 | 効果 |
|---|---|---|
| `rgl.SkeletalMesh.Enable` | 1 | 0 = 旧来の static mesh だけの振る舞い (実行中に変えると、skeletal の entity を壊す)。 |
| `rgl.SkeletalMesh.AlwaysTickPose` | 1 | 登録した component に `AlwaysTickPoseAndRefreshBones` を強制し、URO を切る。実行中に効く。 |
| `rgl.SkeletalMesh.Scope` | 0 | 0 = pawn が持つものだけ、1 = 全部の skeletal の component。 |
| `rgl.SkeletalMesh.MaxEntities` | 64 | 予算。センサーに近いものが勝つ (登録解除の hysteresis で残っている entity も数える)。 |
| `rgl.SkeletalMesh.MinLOD` | 0 | extractor が使ってよい最も低い LOD の番号。上げると、精細さと引き換えに VRAM と skinning の費用が減る。 |

device の memory は、登録した skeletal の entity 1 つあたり、おおよそ **24 B x 頂点の数**
(RGL の SkeletonAnimator + GAS) である。例: 347k 頂点の LOD0 の車両で約 8 MB。
`MaxEntities` と `MinLOD` で抑える。

package の `Config/DefaultEngine.ini` の `[ConsoleVariables]` の節で設定する (Shipping を
含む全部の build の構成で効く)。Development/editor の build に限り、
`-ExecCmds="rgl.SkeletalMesh.Enable 0"` でもよい。

> [!IMPORTANT]
> `-ExecCmds=` と `-ini:` の command line での上書きは、Shipping の build では compile で
> 外される。Development/editor でしか効かない。package の Shipping の server では、
> `Config/DefaultEngine.ini` の CVar か、この branch で足した、Shipping でも効く専用の
> switch を使う: `-rgl-skeletal-mesh-enable=<0|1>`、`-rgl-skeletal-mesh-always-tick-pose=<0|1>`、
> `-rgl-skeletal-mesh-scope=<0|1>`、`-rgl-skeletal-mesh-max-entities=<n>`、
> `-rgl-skeletal-mesh-min-lod=<n>`。

既知の限界:

* morph/cloth/deformer の変位は無い。LOD による bone の評価で、遠くの NPC の末端の bone が
  古いまま残ることがある。見つけるまでの遅れは 1 s 以下 (scene の同期の周期)。
* **ego の自分の見え方**: ego の車両の skeletal の車体も登録される (この変更の前は、door の
  static mesh だけだった)。そのため、`lower_fov` が負の屋根の LiDAR は、屋根・hood・trunk の
  点を 0〜2 m に返す。下流がそれを想定していなければ、距離か ego の bounding box で除く。
* **ego の自分の遮蔽で返りが減る**: それらの ray は、以前は登録されていない ego の車体を
  通り抜け、その先の scene に当たっていた。今は車体で止まり、センサーの最小の距離より
  手前に返るので、報告されずに捨てられる。空の Odaiba で、試験用の ego 車両とその屋根の
  2 つの Pandar で測ると、skeletal の経路を入れると生の点群が 17.8% 減った (20 frame で
  1,186,458 -> 975,408 点)。自分の LiDAR を実際に遮る実車と同じだが、検出器の振る舞いが
  変わるほど大きい変化である。同じ scene で、82 m 先の路側の物体は、検出が 629 frame 中
  2 から 718 frame 中 658 に増えた。その物体自身の返りの数は変わらなかった (どちらも
  1 frame あたり約 30)。
* **kill switch の遅れ**: `rgl.SkeletalMesh.Enable 0` で pose はすぐ止まるが、既存の
  entity が消えるのは次の scene の同期 (1 s 以下) である。

試験: `Automation RunTests CarlaRGL.Skeletal` (editor -game)、
`PythonAPI/rgl/tests/check_dynamic_actor_visibility.py` (ROS2 が要る)。

Shipping の package で測った値 (`check_dynamic_actor_visibility.py`、同じ binary、
`rgl.SkeletalMesh.Enable=0` 対 `=1`):

| blueprint | OFF | ON |
|---|---|---|
| vehicle.ue4.audi.tt (Door=0) | 12 | 372 |
| vehicle.ue4.ford.mustang (Door=0) | 6 | 367 |
| vehicle.lincoln.mkz (doors) | 207 | 415 |
| walker.pedestrian.0016 | 0 | 78 |

tick の費用への影響 (歩行者 7 + 車両 3 + RGL の LiDAR 1、20 Hz の同期、world.tick() の
wall 時間): skeletal OFF に比べて中央値で +11.4% (予算 +20% の内)。

## World Partition の地図

Town15 などの World Partition の地図では、Unreal Engine は streaming source の周りの cell
単位で地図を読み込み、RGL の scene は読み込まれたものに従う。static mesh が RGL の scene に
入るのは、その cell が読み込まれている間 (かつ、mobility が Static のものは、どれかの
センサーの球の中にある間。Movable のものは距離によらず残す) だけで、cell が外されると
scene からも外れる。読み込まれていないものは RGL の scene に無い。
`ray_cast` の LiDAR が辿る physics の scene に無いのと同じである。これは設計どおりで、
ある区域を見るには、そこを読み込ませる (下を見る)。World Partition でない地図
(例: Town10HD_Opt) は全体が読み込まれ、影響を受けない。

> [!NOTE]
> Landscape (地形の高さ場) の component は、RGL の scene に登録しない。`ray_cast` の
> LiDAR はそれに当たる。地面が Landscape の所では、RGL の LiDAR に地面の返りが無い。

### 読み込まれる区域

cell は、World Partition の streaming source の周りで読み込まれる。World Partition の
地図では、CARLA は spawn した車両と歩行者のすべてに、球の streaming source を付ける。

| spawn した actor | その streaming source の半径 |
|---|---|
| `role_name` に `hero` か `ego_vehicle` を含む (大文字・小文字は問わない) 車両・歩行者 | spawn の時点の `tile_stream_distance` (既定 3000 m) |
| それ以外の車両・歩行者 (例: `role_name` = `ego`) | 200 m |

spectator (player controller の視点) も streaming source である。センサーと、CARLA が
spawn するその他の actor は、streaming source ではない。

* RGL の LiDAR は hero の車両に付ける。`ego` のような `role_name` では hero にならない。
  その車両の周りは 200 m しか読み込まれず、それより range の長いセンサーには、その外は
  他の streaming source が読み込んだものしか見えない。
* `tile_stream_distance` が効くのは hero だけで、それも hero の spawn の時だけである。
  半径は spawn の時点で決まり、後で設定を変えても変わらない。
* `actor_active_distance` が `tile_stream_distance` より大きいと、`tile_stream_distance`
  は同じ値に引き上げられる (CARLA は警告の log を出す)。`actor_active_distance` の既定は
  2000 m なので、2 km より小さい `tile_stream_distance` は、`actor_active_distance` も
  小さくしたときだけ効く。hero の spawn の前に、両方を設定する。

  ```python
  settings = world.get_settings()
  settings.tile_stream_distance = 500.0   # m: the hero's streaming radius
  settings.actor_active_distance = 500.0  # m: not larger than tile_stream_distance
  world.apply_settings(settings)
  # then spawn the hero (role_name='hero') and attach the lidars to it
  ```

* 路側のセンサー (何にも付けない、または streaming source の無い actor に付ける) の range
  は、他の streaming source が覆うときしか読み込まれない。hero の `tile_stream_distance`
  を、hero の球がセンサーの range を丸ごと含む (hero からセンサーまでの距離とセンサーの
  range の和が半径以下になる) まで上げるか、センサーを車両に載せる。

### 読み込みの警告

World Partition の地図では、static な scene の同期のたびに (周期的に、センサーが動けば
それより早く)、scene manager が、各センサーの range の球が読み込み済み (その World
Partition の cell が activated) かを確かめる。range が sim 時間で
5 s 続けて読み込み済みでないセンサーは、センサーの登録 1 回につき 1 回だけ次の行を出す。

```text
rgl_scene_stream_warning t=12.400 sensor=0x7f3a2c001230 range_m=200 parent=BP_Car_C_1 parent_stream_radius_m=150 hint=attach_the_sensor_to_a_hero_vehicle_or_raise_tile_stream_distance
```

| Key | 意味 |
|---|---|
| `t` | その行を出した確認の sim 時間 (s) |
| `sensor` | センサーの id (RGL の session の handle。`rgl_hit_dump` の行と同じ) |
| `range_m` | 確かめた球の半径: センサーの range (m) |
| `parent` | センサーの付け先を根まで辿った actor、または `none` |
| `parent_stream_radius_m` | その actor の streaming source の半径 (m)。無ければ `-1` |
| `hint` | すること: センサーを hero の車両に付けるか、`tile_stream_distance` を上げる |

* World Partition でない地図では出ない。親の streaming source が range の球を丸ごと含む
  間も出ない。それは読み込みの途中であり (既定の 3 km の hero の半径は、起動時の読み込みに
  5 s より長くかかることがある)、設定では直せないからである。
* 確認は同期のときだけ行う。そのため行が出るのは、range が読み込み済みでないと分かった
  最初の確認から 5 s 以上たった、最初の同期のときである。
* range が後で読み込まれ、また外れても、行はセンサーの登録 1 回につき 1 回だけである。
  5 s より長く tick しなかったセンサーは scene manager から外れ、また tick すると新しく
  登録されるので、行をもう一度出すことがある。

### 既定の 3 km の hero の半径での同期の費用

static な同期は、毎回、読み込まれた世界の actor を全部走査する。RGL の scene が残すのは
センサーの球の中のものだけだが、走査は全部に及ぶ。既定の `tile_stream_distance`
(3000 m) では、hero が 3 km の円盤を読み込ませたままにし、同期のたびにその全部を走査する。
Town15 で、走る hero に 200 m の LiDAR を 1 つ付けて測った値:

<!-- placeholder: the numbers come from the 3 km sync-cost measurement -->

| hero の streaming source の半径 (`tile_stream_distance`) | static な同期、p95 | static な同期、最大 |
|---|---|---|
| 3000 m (既定) | TBD | TBD |

読み込む区域と同期の時間を減らすには、両方の距離を、センサーに要る分まで下げる。
センサーの range に、hero の原点からセンサーまでの距離と、少しの余裕を足した値である。

### scene の同期の knob

`rgl.Scene.*` と `rgl.Debug.*` の knob は CVar である (CVar の設定の仕方は
[スケルタルメッシュ](#スケルタルメッシュ-車体と歩行者) を見る)。package の Shipping の
server は、2 列目の switch も受ける。例: `-rgl-scene-audit=10`。最初の 5 つの効いている
値は、起動時と、どれかが変わったときに、`rgl_scene_config` の行に出る。

<!-- placeholder: the last row becomes the sync clock and range-edge knobs -->

| CVar | Shipping の switch | 既定 | 効果 |
|---|---|---|---|
| `rgl.Scene.MaxMeshCreatesPerUpdate` | `-rgl-scene-max-mesh-creates-per-update=` | 8 | scene の Update 1 回あたりに作る、新しい RGL の mesh (または idle からの戻し) の上限。残りは近い順の queue で待つ。0 = 上限なし。 |
| `rgl.Scene.CreateBudgetMs` | `-rgl-scene-create-budget-ms=` | 4 | scene の Update 1 回あたりの、mesh の作成の wall 時間の予算 (ms)。0 = 上限なし。 |
| `rgl.Scene.UnloadDistanceFactor` | `-rgl-scene-unload-distance-factor=` | 1.0 | static mesh を外すのは、登録の半径 × factor + 20 m より遠くなってから。1 未満は 1 とみなす。 |
| `rgl.Scene.IdleMeshCacheMax` | `-rgl-scene-idle-mesh-cache-max=` | 0 | 最後の entity が外れた後も残しておく RGL の mesh の数。最も前に idle になったものから壊す。0 = すぐ壊す。 |
| `rgl.Scene.MaxIsmInstanceChangesPerUpdate` | `-rgl-scene-max-ism-instance-changes-per-update=` | 512 | 同期が ISM の群をセンサーの球と比べ直させるとき (同期の無い Update で行う)、scene の Update 1 回あたりに足し引きする Static の ISM の instance の上限。0 = 上限なし。 |
| `rgl.Scene.Audit` | `-rgl-scene-audit=` | 0 | 読むだけの検査: N >= 1 で、N frame ごとに `rgl_scene_audit` の行を出す。actor を全部走査するので、frame の時間を測るときは切っておく。 |
| `rgl.Debug.HitDumpDir` | `-rgl-debug-hit-dump=` | (空) | debug: dump した scan ごとに、当たった点の file を 1 つ受ける dir。空 = 切。 |
| `rgl.Debug.HitDumpEvery` | `-rgl-debug-hit-dump-every=` | 10 | debug: dump の dir を設定したとき、各センサーの最初の scan と、その後 N 回ごとの scan を dump する。 |
| TBD | TBD | TBD | sync の時計と range の縁の knob (これから足す)。 |

### log の行の ISM の見直しの key

別の行にある 2 つの key は、名前が似ていて、同じ queue を違う時点で数える。その queue は、
instance をセンサーの球と比べ直すのを待つ、Static の instanced static mesh (ISM) の群の列
である。同期のたびに、登録済みの Static の ISM の群を全部 queue に入れ直し、同期の無い
Update が `rgl.Scene.MaxIsmInstanceChangesPerUpdate` の上限の内で順に処理する。

| 行 | Key | 意味 |
|---|---|---|
| `rgl_scene_sync` | `ism_pending` | この同期が queue を入れ直したときに残っていた entry の数: 前の回が届かなかった群。0 でないのは、前の同期からの Update で回が終わらなかったとき: 上限で止まったか、同期が続いた (回は同期の無い Update でだけ進む)。 |
| `rgl_scene_audit` | `pending_ism` | 検査した frame の時点の queue の entry の数: まだ今のセンサーの球と比べていない群。 |

どちらの数にも、queue に入れた後に外された component の entry が入る。そのような entry は、
queue がそこに届いたときに捨てる。
