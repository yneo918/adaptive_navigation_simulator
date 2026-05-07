# auto_runner — ヘッドレスバッチ実験ランナー

論文用のAN性能評価データを、RViz・GUIなしで自動取得するためのROS 2パッケージ。

各実験は、配置 → AN実行 → 終了判定 → NPZ保存 → 次実験 のサイクルで進む。
50件 (5モード × 10) の理想条件評価がデフォルトで定義されている。

## アーキテクチャ

```
┌─────────────────────── Terminal A (常駐) ────────────────────────┐
│  disaster_AN_headless.launch.py                                  │
│  ├─ cesium_sensor_field         (放射線場、起動時1回読み込み)    │
│  ├─ pioneer_5 (5台のsim_robot)  (差動駆動シミュ)                 │
│  ├─ cluster_controller          (フォーメーション制御)           │
│  ├─ adaptive_nav                (勾配計算、AN制御則)             │
│  └─ headless_an_controller      (GUIの代替)                      │
└──────────────────────────────────────────────────────────────────┘
        ↑ /rviz/pose2D, /auto/{config,start,stop}
        │
┌─────────────────────── Terminal B (実行毎) ──────────────────────┐
│  batch_orchestrator                                              │
│   for each experiment in YAML:                                   │
│     1. /auto/config 送信       (d, mode, z_des)                  │
│     2. trajectory_plotter_3d をsubprocess起動                    │
│     3. /rviz/pose2D で初期配置                                   │
│     4. /auto/start でナビ開始                                    │
│     5. 終了監視                                                  │
│     6. /auto/stop → SIGINT plotter → NPZ保存                     │
│     7. NPZ移動 → 次実験                                          │
└──────────────────────────────────────────────────────────────────┘
```

## 終了条件 (OR)

各実験は以下のいずれかで終了する：

| 条件 | 内容 | 適用 |
|---|---|---|
| max_steps | サンプル数到達 (default 5000) | 全モード |
| 収束 (plateau) | 直近200stepでleader位置span < 0.5 sim units | MAX/MIN/RIDGE/TRENCH |
| 領域外 (oob) | 5台全ロボットのz < 0.12 が50step連続 | 全モード |
| time_limit | 30分タイムアウト (default 1800s) | 全モード |

CROSSTRACKは**周回し続ける**ため、収束(plateau)判定を `disable_plateau: true` で無効化済み。

## 前提

- ROS 2 Jazzy (Ubuntu 24.04)
- Python 3.12 + numpy, scipy, sympy, geopandas, shapely, PyYAML
- ワークスペースで `colcon build` 済みであること

## 初回セットアップ

```bash
# ワークスペース直下で
cd ~/mlu/adaptive_navigation_simulator   # 自分の workspace に置き換え

# プル & クリーンビルド推奨
git pull
rm -rf build install log   # numpy header 問題等のクリア
colcon build --symlink-install
source install/setup.bash
```

## クイックスタート (単発テスト)

50件流す前に、まず1件で動作確認することを推奨。

### Terminal A: ヘッドレスシミュレータ起動

```bash
source /opt/ros/jazzy/setup.bash
source ~/mlu/adaptive_navigation_simulator/install/setup.bash

ros2 launch auto_runner disaster_AN_headless.launch.py time_scale:=10.0
```

`time_scale:=10.0` でシミュレーション時間を10倍速に（CPU余力次第で50〜100まで上げ可）。

起動完了の目安：`cesium_sensor_field` のフィールド読み込みログが落ち着き、
`headless_an_controller` の `ready` メッセージが出ること。

### Terminal B: G01のみ単発実行

```bash
source /opt/ros/jazzy/setup.bash
source ~/mlu/adaptive_navigation_simulator/install/setup.bash

ros2 run auto_runner batch_orchestrator \
    --config ~/mlu/adaptive_navigation_simulator/install/auto_runner/share/auto_runner/config/experiments_ideal.yaml \
    --output-root ~/git_repos/paper/data \
    --ids G01_MAX_d150
```

期待されるログ（20step毎に進捗）:

```
============================================================
[G01_MAX_d150] mode=MAX d=150.0
============================================================
[G01_MAX_d150] plotter spawned (pid=12345)
[G01_MAX_d150] warmup 5.0s ...
[G01_MAX_d150] initial pose published
[G01_MAX_d150] /auto/start sent
[G01_MAX_d150] step=  20 leader=(-1948.3, 1265.7) z_c=0.421 z_max=0.443 span=12.34 oob_streak=0
[G01_MAX_d150] step=  40 leader=(-1942.1, 1278.2) z_c=0.435 z_max=0.461 span=15.08 oob_streak=0
...
[G01_MAX_d150] terminating: plateau (span=0.348 < 0.5) (step=843)
[G01_MAX_d150] SIGINT plotter ...
[G01_MAX_d150] saved -> /home/neo/git_repos/paper/data/IDEAL/MAX/20260507_xxxxx_trajectory_data.npz
```

各カラム:
- `step`: 現在のサンプル数 (sample_interval=5sごと)
- `leader`: リーダー(p1)位置 [sim coord]
- `z_c`: 5台センサ値の平均
- `z_max`: 5台センサ値の最大（領域外判定基準）
- `span`: 直近200stepの位置span（収束基準: < 0.5 で収束）
- `oob_streak`: 領域外連続step（≥50で領域外終了）

## 全50件バッチ実行

長時間ジョブのため `tmux` を強く推奨。

### tmuxでの放置可能セットアップ

```bash
# 新規セッション
tmux new -s sim

# 縦分割: Ctrl+B → "
# 上ペイン (Terminal A)
source /opt/ros/jazzy/setup.bash
source ~/mlu/adaptive_navigation_simulator/install/setup.bash
ros2 launch auto_runner disaster_AN_headless.launch.py time_scale:=10.0

# Ctrl+B → ↓ で下ペインへ移動
# 下ペイン (Terminal B)
source /opt/ros/jazzy/setup.bash
source ~/mlu/adaptive_navigation_simulator/install/setup.bash
mkdir -p ~/git_repos/paper/data/IDEAL
ros2 run auto_runner batch_orchestrator \
    --config ~/mlu/adaptive_navigation_simulator/install/auto_runner/share/auto_runner/config/experiments_ideal.yaml \
    --output-root ~/git_repos/paper/data \
    2>&1 | tee ~/git_repos/paper/data/IDEAL/batch_$(date +%Y%m%d_%H%M%S).log

# detach: Ctrl+B → D
# 復帰:    tmux attach -t sim
# 終了:    tmux kill-session -t sim
```

### 想定所要時間

| time_scale | 1件平均 | 合計（50件） |
|---|---|---|
| 1.0 (実時間) | 1〜7時間 | **数日** |
| 10.0 (推奨初期値) | 6〜40分 | **5〜30時間** |
| 50.0 (高速) | 1〜8分 | **1〜6時間** |

実際の所要は収束タイミング次第。プラトー検出で早期終了するモードでは大幅に短縮される。

### 部分実行・再実行

```bash
# 特定モードのみ
ros2 run auto_runner batch_orchestrator --config ... --output-root ... --ids G01_MAX_d150,G02_MAX_d150,G03_MAX_d150

# モード単位で実行 (G/H/I/J/K プレフィックス)
ros2 run auto_runner batch_orchestrator --config ... --output-root ... \
    --ids "$(yq -r '.experiments[] | select(.id | startswith("G")) | .id' .../experiments_ideal.yaml | paste -sd,)"

# 計画確認のみ (実行しない)
ros2 run auto_runner batch_orchestrator --config ... --output-root ... --dry-run
```

## 出力

```
~/git_repos/paper/data/IDEAL/
├── MAX/
│   ├── 20260507_120100_trajectory_data.npz   # G01
│   ├── 20260507_122230_trajectory_data.npz   # G02
│   └── ...
├── MIN/
│   └── ...
├── CT/
│   └── ...
├── RIDGE/
│   └── ...
└── TRENCH/
    └── ...
```

各NPZの内容（`scripts/process_experiments.py` で処理可能）:
- `trajectories`: dict of `{p1: ndarray(N,3), p2:..., p5:...}`
- `terrain_points`, `terrain_elevations`: フィールドデータ
- `sensor_distance_scale`: 距離スケール (0.05)
- 他メタ情報

## 解析

データ収集後、論文用の図・テーブル生成は paper側スクリプトで：

```bash
cd ~/git_repos/paper
source venv/bin/activate

# scripts/process_experiments.py の EXPERIMENTS dict に新NPZパスを追加
# その後
python3 scripts/process_experiments.py
```

## 監視・操作

### 進捗確認

```bash
# tmuxでログを見ながら
tmux attach -t sim

# 別端末でログtail
tail -f ~/git_repos/paper/data/IDEAL/batch_*.log

# 完了済み実験数
ls ~/git_repos/paper/data/IDEAL/*/  | grep -c trajectory_data.npz
```

### 中断・停止

```bash
# Terminal Bでの中断 (現在の実験は途中終了、NPZは生成されない可能性)
Ctrl+C

# Terminal A自体を止めるとシミュレータ全停止
Ctrl+C
```

### 再開

途中中断した場合、未実行のIDを `--ids` で指定して再開：

```bash
# 完了済みのID一覧
ls ~/git_repos/paper/data/IDEAL/*/*.npz | xargs -n1 basename | head

# 残りを実行 (例: G05以降)
ros2 run auto_runner batch_orchestrator --config ... --output-root ... \
    --ids G05_MAX_d150,G06_MAX_d150,...,K10_TRENCH_d200
```

## カスタマイズ

### 新しい実験を追加

`config/experiments_ideal.yaml` の `experiments:` リストにエントリ追加：

```yaml
- id: NEW_EXPT
  mode: MAX                                # MAX/MIN/CROSSTRACK_CW/RIDGE_DOWN/TRENCH_UP 等
  d: 150.0                                 # GUI単位 (real_m = d * 20)
  start: {x: -1000.0, y: 500.0, theta: 0.0}
  z_des: 0.6                               # CROSSTRACK時のみ
  max_steps: 5000                          # オプション (default 5000)
  disable_plateau: true                    # CROSSTRACK時に必須
  output_subdir: IDEAL/CUSTOM
```

### 終了条件をいじる

defaults セクションを編集（または個別実験で上書き）：

```yaml
defaults:
  max_steps: 5000              # 全実験のステップ上限
  plateau_eps: 0.5             # 収束判定: 位置span閾値
  plateau_window_steps: 200    # 収束判定: 窓幅 (step)
  oob_threshold: 0.12          # 領域外判定: センサ値閾値 (z_norm)
  oob_window_steps: 50         # 領域外判定: 連続step
  time_limit_s: 1800           # ハードタイムアウト (秒)
  progress_every_steps: 20     # 進捗ログ間隔
```

### 新モードの追加

`adaptive_nav/ScalarGradient.py` の `ControlMode` enum に存在するモード名であれば、
YAML の `mode:` フィールドに指定するだけで動く。例：`CONTOUR_CW`, `RIDGE_UP` 等。

## トラブルシューティング

### `[Errno 2] No such file or directory: '...experiments_ideal.yaml'`

ビルド時に YAML が存在しなかった可能性。再ビルド：

```bash
colcon build --symlink-install --packages-select auto_runner
```

または src 側を直接指定：

```bash
ros2 run auto_runner batch_orchestrator \
    --config ~/mlu/adaptive_navigation_simulator/src/auto_runner/config/experiments_ideal.yaml \
    --output-root ~/git_repos/paper/data --dry-run
```

### `numpy/ndarrayobject.h: No such file or directory`

`rosidl_generator_py` が numpy ヘッダを見つけられない（Ubuntu 24.04で散発）。
クリーンビルドで解消：

```bash
rm -rf build install log
colcon build --symlink-install
```

### `[<expt_id>] ERROR: leader pose never received`

シミュレータ側で `/p1/pose2D` が出ていない。Terminal A のログを確認：
- `cesium_sensor_field` のフィールド読み込みが完了しているか
- `pioneer_5` の各 `sim_rover` が立ち上がっているか
- `headless_an_controller` の `ready` ログが出ているか

### `out-of-area` で頻発する

開始位置が低線量域にある可能性。`experiments_ideal.yaml` で `start` を高線量側にずらす。
あるいは `oob_threshold` を 0.12 → 0.10 に下げる、`oob_window_steps` を増やす。

### `plateau` で早期終了する

ロボットがすぐ動かなくなっている。考えられる原因：
- 開始位置がすでに極値の近く（MAX/MINで)
- `d` が大きすぎて勾配が消えている

`plateau_window_steps` を 200 → 400 に増やす、`plateau_eps` を 0.5 → 1.0 に緩める、
あるいは開始位置を遠ざける。

### 1件目が極端に長い

初回は `cesium_sensor_field` の terrain ingestion + 補間構築 に時間がかかる
（数十秒〜数分）。2件目以降はキャッシュされて高速化する。

### NPZ が生成されない

`trajectory_plotter_3d` の SIGINT 処理が間に合わない可能性。
`cooldown_s` を 2.0 → 5.0 に増やすか、 `stop_plotter()` の `save_timeout_s` を確認。

## ファイル構成

```
auto_runner/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md                              ← このファイル
├── auto_runner/
│   ├── __init__.py
│   ├── headless_an_controller.py          # GUI代替ノード
│   └── batch_orchestrator.py              # 実験ループ
├── launch/
│   └── disaster_AN_headless.launch.py     # ヘッドレスlaunch
├── config/
│   ├── experiments.yaml                   # 旧8件 (B1-F1)
│   └── experiments_ideal.yaml             # 新50件 (G01-K10)
├── resource/
│   └── auto_runner
└── test/
```

## 関連ドキュメント

- `~/git_repos/paper/CLAUDE.md` — 論文プロジェクト全体の方針
- `~/git_repos/paper/ADDITIONAL_EXPERIMENTS.md` — 旧8件 (B1-F1) の計画
- `~/git_repos/paper/scripts/process_experiments.py` — NPZ → 図・テーブル生成
