# 性能最適化

RGL LiDARセンサーの性能計測結果と適用した最適化。

## 要点

- `rgl_graph_run`自体は0.05-0.28ms（極めて高速）
- `CollectResults`（GPU→CPUコピー）は初回69ms、以降2ms
- ボトルネックはCARLAのsyncループ自体（ray_castでも同じ"behind schedule"）
- Chaosコリジョン（835K tri）はGPU readback（2.3M tri）より64%軽量で同等のヒット率
- 静的エンティティ（1274/1284）のTransform更新スキップが有効

## 詳細

### タイミング計測結果（RTX 5080）

| 処理 | 時間 | 備考 |
|------|------|------|
| rgl_graph_run | 0.05-0.28ms | GPUレイトレース（非同期、即return） |
| CollectResults（初回） | 69ms | GPU→CPUコピー + BVH初回構築 |
| CollectResults（以降） | 2ms | 3フィールド × 4800点のreadback |
| NeedResults=no時 | 0ms | GPU→CPUコピースキップ |

### 適用済み最適化

1. **レイパターンキャッシュ**: 初回のみ生成、以降は再利用（`bRayPatternCached`）
2. **リングIDキャッシュ**: レイパターンと同時にキャッシュ
3. **静的エンティティスキップ**: `Mobility == Static`かつ初期化済みならTransform更新不要
4. **動的エンティティ変更検出**: `FTransform::Equals`で変更なしならスキップ
5. **CollectResultsスキップ**: `AreClientsListening() || bRglShowLidarPoints`の場合のみ実行
6. **SerializeAndSendスキップ**: クライアントリスナー不在時はスキップ
7. **フレームスキップ**: `GFrameCounter`で同一フレーム内の重複Update防止
8. **1回転キャップ**: `AngleDistanceOfTick`を`HorizontalFov`以下に制限（6回転→1回転）

### publish周波数比較

ray_cast: 5.0Hz, RGL(CARLA publish): 5.1Hz, RGL(RGL publish): 5.1Hz
→ 差はCARLAのsyncループ制約が支配的でLiDARスキャン方式の差は微小

### AWSIM比較からの知見

- AWSIMはレイパターンを静的（設定変更時のみ更新）→ CARLAも同様にキャッシュ化
- AWSIMはResultsをコールバック委譲（CPU readbackなし）→ CARLAでもNeedResults=noでスキップ
- AWSIMはメッシュ共有（参照カウント）→ CARLAもTMapキャッシュで実装済み
