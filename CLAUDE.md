# プロジェクト概要
wros
[WRS](https://github.com/wanweiwei07/wrs)（Wan et al., IEEE TRO 2021）で実装されたロボット動作・把持計画プランナーを利用するための、ROS Noeticノードのサンプルパッケージ。

## 1. ディレクトリ構造と実行環境
本プロジェクトは、NVIDIAコンテナランタイムを利用したROS NoeticベースのDocker環境（`wros_noetic_container`）で構成されます。
- **共有ディレクトリ**: ホストマシンの `./catkin_ws/noetic/src/wros_tutorials` がコンテナ内の `/catkin_ws/src/wros_tutorials` にマウントされます。
- **WRSライブラリ（gitサブモジュール）**: `wros_tutorials/wrs/` は外部リポジトリ（`https://github.com/takuya-ki/wrs.git`）のgitサブモジュールです。**本リポジトリ内では `wrs/` 配下のファイルを直接編集しないでください。**
- **Python環境**: ソースビルドされたPython 3.9.5をベースとし、`panda3d`, `open3d`, `trimesh`, `scikit-learn`, `networkx` などの高度な計算・可視化ライブラリがインストールされています。

## 2. 開発と運用の厳格なルール
### A. ビルドの制約
- 環境はUbuntu 22.04ホストおよびROS Noetic (Python3) を前提とします。
- ROSパッケージのビルド時には、コンテナ内で必ず `catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3` を実行し、Python 3インタープリタを明示すること。

### B. 動作計画の実行とパラメータ設定
- 把持計画のパラメータは `wros_tutorials/config/` 配下のYAMLファイル（例: `planner_params_robotiq140_single_example.yaml` 等）で定義されます。設定変更後は都度ビルド（`catkin build`）と `source devel/setup.bash` が必要です。
- コンテナ内での操作は `byobu` を活用したマルチウィンドウでの実行（LaunchとService Callの並行実行）が推奨されています。

## 3. エージェントのルーティングルール（PMへの指示）
タスクのフェーズに応じて、以下の専門エージェントに的確に委譲してください。
- **アーキテクチャ設計・プランナー連携設計**: `architect` に依頼
- **ROSパッケージ実装・YAML編集・WRS連携のコーディング**: `developer` に依頼
- **ビルド確認・Launch実行・Service Callテスト**: `tester` に依頼
- **Python依存関係エラー・X11表示エラー・プランニング失敗の解消**: `debugger` に依頼