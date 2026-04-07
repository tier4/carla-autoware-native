# メッシュ抽出戦略

UE5のStaticMeshからRGLにジオメトリをアップロードする3つのパスと優先順位。

## 要点

- **優先順位**: Chaosコリジョン → レンダーデータ(CPU) → GPU readback
- Chaosコリジョンは低ポリ（835K tri）でAWSIM同等のアプローチ → レイトレース高速
- GPU readbackは高ポリ（2.3M tri）でレンダーメッシュ品質 → レイトレース低速
- `bAllowCPUAccess`はShipping/Developmentパッケージ両方でfalse → レンダーデータパスは実質使用不可
- Shippingビルドでの`VertexPosition()`呼び出しはSIGSEGVの原因（Data pointer null）

## 詳細

### 抽出パス

| パス | データソース | ポリゴン品質 | CPU access必要 | 実績 |
|------|-------------|-------------|---------------|------|
| ExtractFromChaosCollision | BodySetup→ChaosTriMeshes/ConvexElems/BoxElems | 簡略化 | 不要 | 469メッシュ |
| ExtractFromRenderData | LODResources→PositionVertexBuffer | 最高 | 必要 | 0メッシュ（パッケージビルドでは使用不可） |
| ExtractFromGPUBuffer | RHI LockBuffer→GPU readback | 最高 | 不要 | 16メッシュ（フォールバック） |

### GPU readback実装

```cpp
ENQUEUE_RENDER_COMMAND(LockVertexBuffer)([&](FRHICommandListImmediate& RHICmdList)
{
    LockedVB = RHICmdList.LockBuffer(VBRef, 0, VBSize, RLM_ReadOnly);
});
FlushRenderingCommands();
```

- ゲームスレッドから呼び出し、`FlushRenderingCommands()`で同期
- 頂点バッファ・インデックスバッファ両方をreadback
- 16bit/32bitインデックス自動判定

### Chaosコリジョン経路の詳細

1. `ChaosTriMeshes`（複合コリジョン）→ Particles + Elements(Large/Small index)
2. `ConvexElems`（凸包）→ VertexData + IndexData
3. `BoxElems`（箱型）→ 8頂点12三角形に変換

### NishishinjukuMapの結果

- Landscape: 0（地面はStaticMesh）
- WithStaticMesh: 1276アクター
- Chaosコリジョン: 835,949三角形（96%ヒット率）
- GPU readback: 2,346,277三角形（96%ヒット率だがレイトレース低速）
