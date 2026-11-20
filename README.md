# robot-sim

フリッパー付きクローラロボットのシミュレータです。ラグ(ベルトの突起)を 1 つずつ計算するので、ラグが段差の角に引っかかって登るところまで再現できます。

- Windows の実行ファイル `robot-sim.exe` 1 つで動きます(Rust 製、外部ライブラリなし)。
- ブラウザの画面で、ロボットの寸法や地形を変えて走らせたり、キーボードで操縦したりできます。
- 走らせた様子は記録に残し、あとから 3D で再生できます。

## 動かす

必要なもの: Windows 10 / 11(64 ビット)とブラウザ。ほかに入れるものはありません。

1. [Releases](https://github.com/26m3005/crawler-sim/releases/latest) から `robot-sim-v0.1.0-windows-x64.zip` をダウンロードして、展開します。
2. 中の `robot-sim.exe` をダブルクリックすると、ブラウザに操作の画面が開きます。

署名のない exe なので、初めて開くと Windows が「PC が保護されました」と出すことがあります。「詳細情報」を押して「実行」を選ぶと開きます。

自分でビルドするときは、下の「詳しい説明」の「自分でビルドする」を見てください。

## 操作の画面(GUI)

左で値を選ぶと、右にロボットと地形がすぐ出ます。

- **ロボット**: 寸法、質量、ラグの形などを変える。パラメータの保存と読み込みもできます。
- **地形**: 段差、階段、坂などを選ぶ(下の「地形」)。
- **走らせる**: 決めた速さとフリッパーの角度で走らせ、終わると右で再生します。「記録を保存」で記録を保存できます。
- **手で操縦する**: 「操縦を始める」で、キーボードで操縦します(実際の時間と同じ速さで動きます)。

| キー | 動き |
|---|---|
| W / S(↑ / ↓) | 前へ / 後ろへ |
| A / D(← / →) | 左へ / 右へ回る |
| R / F | 前のフリッパーを上げる / 下げる |
| T / G | 後ろのフリッパーを上げる / 下げる |

終わるときは、画面の「終了」を押すか、exe の窓を閉じます。

## 設定ファイルで走らせる

画面を使わずに、設定ファイルで走らせることもできます。

```sh
robot-sim.exe run examples\course.txt
```

(展開したフォルダで実行します。) サンプルのロボットがサンプルのコースを走り、記録 `examples\course_run.json` を書きます。設定ファイルは 1 行に `key = value` を 1 つ書きます。たとえば `examples` に次のファイルを置くと、階段を走ります(パスは設定ファイルの場所から数えます)。

```
robot   = ../robots/simple_crawler/simple_crawler.urdf
terrain = stairs rise=0.1 steps=4
belt    = 0.5, 0.5          # 左右のベルトの速さ [m/s]
joints  = 40, 0, 40, 0      # フリッパーの角度 [度]
record  = run.json
```

設定ファイルを `robot-sim.exe` のアイコンにドロップしても走ります。

## 地形

`terrain = 名前 キー=値` で選びます(キーを書かなければ既定の値)。

| 名前 | 中身 |
|---|---|
| `flat` | 平らな地面 |
| `step h=0.1` | 段差 1 つ |
| `curb h=0.1` | 左半分だけの段差 |
| `stairs rise=0.08 tread=0.25 steps=8` | 階段 |
| `rails h=0.05` | 並んだレール |
| `holes h=0.1` | 穴の市松模様 |
| `random hmax=0.2 seed=0` | ランダムな高さのブロック |
| `slope deg=15` | なめらかな坂 |
| `side_slope deg=15` | 左半分だけの横の坂 |
| `course` | 上の地形を全部並べたサンプルのコース(31 m) |

## 記録を見る

`viewer/viewer.html` をブラウザで開き、記録(`.json`)をドロップします。スペースで再生と停止、スライダーで好きな時刻へ、ドラッグとホイールで視点を変えられます。

## ロボットのサンプル

`robots/simple_crawler/` に 1 台あります。20 kg、主クローラ 2 本とフリッパー 4 本、ラグは長方形の、見本のために決めた架空のクローラです。同じロボットを 2 通りの書き方で置いています。

- `simple_crawler.txt`: 寸法などのパラメータ(GUI で変えられるのはこれ)
- `simple_crawler.urdf`: 拡張 URDF(部品を 1 つずつ書く、細かく作りたいとき)

## 詳しい説明

<details>
<summary>自分でビルドする</summary>

[Rust](https://www.rust-lang.org/)(stable、MSVC)を入れて、このリポジトリで次を実行します。

```sh
cargo build --release
```

`target\release\robot-sim.exe` ができます。設定ファイルで走らせるときは `target\release\robot-sim.exe run examples\course.txt` のように実行します。

</details>

<details>
<summary>コマンドの一覧</summary>

| コマンド | 動き |
|---|---|
| `robot-sim` | 操作の画面を開く |
| `robot-sim run <設定ファイル>` | 設定のとおりに走らせ、記録を書く |
| `robot-sim info <ロボットのファイル>` | ロボットの名前、質量、関節、ベルト、モーターを JSON で表示する |
| `robot-sim urdf <パラメータ>` | パラメータから作ったロボットを、拡張 URDF で表示する |

ほかの PC へは、exe とロボットのファイルと設定ファイルを持っていけば動きます(Windows に最初から入っている DLL だけを使います)。

操作の画面は、`robot-sim.exe` がこの PC の中だけ(127.0.0.1)で開く小さなサーバーが出します。起動のたびに作る合言葉(アドレスの `t=`)がない要求には答えず、ファイルの読み書きはブラウザがします。手で操縦しているとき、画面を閉じるなどで指令が 3 秒届かないと止まります。設定ファイルをアイコンにドロップしたときは、終わっても Enter を押すまで窓を閉じません。

</details>

<details>
<summary>設定ファイルのキー</summary>

`#` から行末まではコメントです。書き間違えたキーや値は、行番号付きのエラーになります。パスは設定ファイルの場所から数えます。

| キー | 既定 | 意味 |
|---|---|---|
| `robot` | (必須) | ロボットのファイル(`.urdf` か、パラメータ) |
| `terrain` | `course` | 地形 |
| `friction` | `0.8` | 地形の摩擦係数。ロボットの面の摩擦と小さい方が使われる |
| `seconds` | `90` | この時間で止める。地形の終わりを過ぎたらその前でも止める |
| `belt` | `0.5, 0.5` | 左右のベルトの速さ [m/s]。正で前進 |
| `joints` | 全部 `0` | 関節の目標角度 [度]。ロボットのファイルに書いた関節の順 |
| `control_dt` | `0.05` | 指令を出す間隔 [s](中では 1 ms ごとに計算する) |
| `record` | (なし) | 記録を書くファイル |

ロボットは止まった状態で、x = 0 に +x を向いて置かれます。単位は m、kg、s、rad(設定ファイルの関節の角度だけは度)。座標は x が前、y が左、z が上です。

</details>

<details>
<summary>地形の細かいこと</summary>

地形は、正方形のマスに並べた柱です。柱の上面は水平か、マスごとに傾いた平面です(同じ傾きのマスが並ぶと、継ぎ目のない斜面になります)。

| 名前 | キー(既定) | 中身 |
|---|---|---|
| `step` | `h`(0.1) | 幅いっぱいの段差 1 つ。長さ 2 m |
| `curb` | `h`(0.1) | 左半分だけの段差。長さ 2 m |
| `rails` | `h`(0.05)、`every`(5) | 高さ `h` のレールを、0.1 m のマス `every` 個ごとに。長さ 4 m |
| `holes` | `h`(0.1)、`cell`(0.25) | 深さ `h` の穴の市松模様。長さ 4 m |
| `random` | `cell`(0.25)、`hmax`(0.2)、`seed`(0) | 高さ 0〜`hmax` のランダムなブロック。長さ 4 m。同じ `seed` なら同じ地形 |
| `stairs` | `rise`(0.08)、`tread`(0.25)、`steps`(8)、`skew_deg`(0) | 階段。`skew_deg` で段の縁を斜めにすると、片方のクローラが先に当たる |
| `slope` | `deg`(15) | 平らな地面から登る坂。長さ 3 m。登りきるとその高さの平らな地面が続く |
| `side_slope` | `deg`(15) | 左半分(y > 0)だけ左へ登る横の坂。長さ 3 m。手前と奥の縁は段差になる |
| `course` | `seed`(0) | rails、step、curb、holes、random、stairs、slope、side_slope を 1 m おきに並べたコース。だんだん登る |

- 地形は x = 1.2 m から始まり、幅は 3.5 m(y = 0 が中心)。手前は高さ 0、奥と横は端の高さのまま平らに続きます。
- 地形の終わりから 0.6 m 先を過ぎると、`run` は止まります。
- 1 つのマスの上面は平面なので、ねじれた面は表せません。

</details>

<details>
<summary>パラメータでロボットを作る</summary>

拡張子が `.urdf` でないロボットのファイルは、パラメータとして読みます。形は決まっていて(車体に主クローラ左右 2 本、フリッパー 4 本)、関節の位置などはパラメータから求めます。書かなかったキーはサンプルの値になります。全部を書いた例が `robots/simple_crawler/simple_crawler.txt` です。

| キー | サンプルの値 | 意味 |
|---|---|---|
| `name` | `simple_crawler` | ロボットの名前(英数字、`_`、`-`) |
| `chassis_mass`、`chassis_inertia` | 16、0.17, 0.45, 0.55 | 車体の質量 [kg] と慣性モーメント ixx, iyy, izz [kg·m²] |
| `chassis_friction` | 0.3 | 車体の摩擦 |
| `main_spacing`、`main_radius` | 0.5、0.06 | 主クローラの前後のプーリーの間隔と半径 [m] |
| `main_belt`、`main_belt_mass` | 0.04, 0.14、1.0 | ベルトの、中心面から内端と外端までの距離 [m] と質量 [kg] |
| `main_lugs` | 16 | ラグの数。0 でラグなし |
| `main_lug_shape` | `rect` | ラグの形: `rect`(長方形)、`trapezoid`(台形)、`ramp`(上面を内側へ下げた長方形) |
| `main_lug_height`、`main_lug_length` | 0.015、0.03 | ラグの高さと、根元の長さ [m] |
| `main_lug_top` | (根元と同じ) | `trapezoid` のときだけ: 先端の長さ [m] |
| `main_lug_ramp` | (なし) | `ramp` のときだけ: 下げ始める中心面からの距離と、内端の高さ [m] |
| `flipper_length`、`flipper_radius` | 0.24、0.06 | フリッパーの長さとプーリーの半径 [m] |
| `flipper_belt`、`flipper_belt_mass` | 0.145, 0.185、0.5 | フリッパーのベルトの内端と外端 [m]、質量 [kg] |
| `flipper_mass`、`flipper_inertia` | 1、0.001, 0.006, 0.006 | フリッパー 1 本の質量と慣性モーメント |
| `flipper_lugs`、`flipper_lug_*` | 10、主クローラと同じ | フリッパーのラグ(`main_` を `flipper_` にしたキー) |
| `joint_range` | -1.57, 1.57 | フリッパーの角度の範囲 [rad]。正で先端が上がる |
| `joint_velocity`、`joint_effort` | 1.5、60 | 関節の最大の速さ [rad/s] と力 [N·m] |
| `servo_gain` | 4 | 関節のサーボのゲイン |
| `drive_torque` | 30 | 片側 3 本のベルトを回すモーターの最大トルク [N·m] |

知らないキー、その形で使わないキー、重なるベルトはエラーになります。`robot-sim urdf <パラメータ>` で、作られる拡張 URDF を表示できます。

</details>

<details>
<summary>拡張 URDF の書き方</summary>

URDF にクローラの要素を足した XML です。全体の例は `robots/simple_crawler/simple_crawler.urdf`。読めない書き方(知らない要素や属性、計算しないもの)は行番号付きのエラーになり、`<visual>`、`<material>`、`<gazebo>`、`<transmission>` は読み飛ばします。

**普通の URDF の部分**

| 要素 | 読むもの | 制限 |
|---|---|---|
| `<link>` | `<inertial>`、`<collision>` の箱 | 慣性の非対角成分は使わない。`<collision>` は回転なしの箱だけ |
| `<joint>` | `type="revolute"`、`parent`、`child`、`origin xyz`、`axis xyz`、`limit` | 回転関節だけで、親は根のリンク。軸は y 方向 |

根のリンク(どの関節の子でもないリンク)が車体です。関節を書いた順が、設定ファイルの `joints` の順になります。

**クローラの要素**(`<robot>` の直下)

```xml
<track name="left_main" side="left" link="base_link">
  <pulley xyz="-0.25 0 0" radius="0.06"/>
  <pulley xyz="0.25 0 0" radius="0.06"/>
  <belt inner="0.04" outer="0.14" mass="1.0"/>
  <lugs count="16" spacing="even" inner="0.04" outer="0.14">
    <profile points="-0.015 0  0.015 0  0.015 0.015  -0.015 0.015"/>
  </lugs>
</track>
<drive name="left" max_torque="30" tracks="left_main left_front_flipper left_rear_flipper"/>
<plate track="left_main" y="0.14 0.145" inset="0.005"/>
<servo joint="front_left" gain="4"/>
<surface link="base_link" friction="0.3"/>
```

| 要素 | 意味 |
|---|---|
| `<track name side link>` | クローラ 1 本。`side` は `left` か `right`。`link` は車体かフリッパーのリンク |
| `<pulley xyz radius>` | ちょうど 2 個。軸の位置(x と z)とベルト面の半径 |
| `<belt inner outer mass>` | 中心面からベルトの内端と外端までの距離、ベルトの質量 |
| `<lugs count spacing inner outer>` | ラグ。1 周の数、並べ方(`even`)、幅の範囲。省略するとラグなし |
| `<profile points>` | ラグの断面の 4 隅(`u h` の組)。`u = 0` に対称な台形 |
| `<ramp from inner_height>` | ラグの上面を内側へ下げる(省略可、断面が長方形のときだけ) |
| `<drive name max_torque tracks>` | 同じ側のクローラを 1 つのモーターで回す。`<track>` の中に `<drive max_torque>` を書くと、そのクローラだけを回す |
| `<plate track y inset>` | ベルトと同じ輪郭の動かない板。ベルトの間などを埋める |
| `<servo joint gain>` | 関節の位置サーボ。関節ごとに 1 つ |
| `<surface link friction stiffness damping>` | 接触の性質。摩擦(省略で地形の摩擦)、剛性(1e5 N/m)、減衰(1500 N·s/m)。`<track>` の中にも書ける |

フリッパーは、フリッパーのリンク、車体からの回転関節(前は `axis="0 -1 0"`、後ろは `axis="0 1 0"` で、正の角度で先端が上がる)、`<servo>` を書き、`<track link="フリッパーのリンク">` でベルトを載せます。

</details>

<details>
<summary>Rust から使う(何台も並べて動かす)</summary>

`crates/sim-task` をライブラリとして使うと、何台も並べて 1 台ずつ違う指令で動かせます(学習などに)。`crates/robot-sim/src/main.rs` が短い使い方の例です。

| もの | 中身 |
|---|---|
| `robot::load(path)` | ロボットのファイルを読む |
| `world::World::new(&model, 台数, スレッドの数, control_dt)` | 何台も並べた世界 |
| `set_terrain(選ぶ台, 地形, 摩擦)` | 選んだ台に地形を与える |
| `reset(選ぶ台, 位置, 向き, 関節の角度)` | 選んだ台を置き直す |
| `step(ベルト, 関節)` | 全台を `control_dt` 秒進める |
| `envs[k].state` | k 番の台の状態(姿勢、速度、関節、ベルト、時間) |
| `record(k)`、`stop_recording()`、`recording_json(記録)` | 記録を取る、止める、JSON にする |
| `terrain::TerrainSpec::parse(文字列)` と `build()` | 地形の書き方から地形を作る |

同じ入力、同じ PC、同じビルドなら、スレッドの数によらず同じ結果になります(PC が違うと少し変わります)。

</details>

<details>
<summary>速さの目安</summary>

速さは、リアルタイム係数(RTF: シミュレーションの中で進んだ時間 ÷ 計算にかかった時間)で表します。1 より大きければ実際の時間より速く、1 なら同じ、1 より小さければ遅い、という意味です。`robot-sim run` の最後に出る `4.1 x real time` がこの値です。

ふつうのノートパソコンで、サンプルのロボットがサンプルのコースを走るとき:

| 指標 | 値 |
|---|---|
| RTF(1 台) | 約 4(ロボットが 68 秒かけて走るコースを約 17 秒で計算) |
| 物理のステップ(1 ms 刻み) | 1 秒あたり約 4,000 回 |
| 指令のステップ(`control_dt` = 0.05 s) | 1 秒あたり約 80 回 |
| RTF(`World` で何台も同時に) | 台数を合わせて、1 台のときの数倍 |

速さは地形とロボットの動きで変わります(接触が多いほど遅い)。

</details>

## ライセンス

MIT(`LICENSE`)
