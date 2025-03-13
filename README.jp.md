# リポジトリについて
このREADMEは日本語話者向けのREADMEです．英語版は[こちら](README.md)を参照してください．

このリポジトリは，xArmやLite6のエンドエフェクタに接続されたDynamixelサーボを制御するためのラッパーです．
# インストール方法
## リポジトリのクローン
```bash
git clone https://github.com/HoneyMack/xarm_eef_dynamixel_wrapper
```

## 実行に必要なパッケージのインストール
```bash
cd xarm_eef_dynamixel_wrapper
pip install .
```

# 使い方
このラッパーはハイレベルAPIとローレベルAPIを提供している．
## ハイレベルAPI
eefに接続されたDynamixelサーボを制御するために，ハイレベルAPI`EEFAPI`が使用できます．具体的な使い方は[example](examples/use_highlevel_api.py)を参照してください．

## ローレベルAPI
eefに接続されたDynamixelサーボを細かく制御するために，ローレベルAPI`EndEffectorPortHandler`と`DynamixelSDK`を使用することもできます．
eefに接続されたDynamixelとの通信方法はほとんど[DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/main/python)と同じです．

違いは，デフォルトの`PortHandler`の代わりに`EndEffectorPortHandler`(src/endeffector_port_handler.py)を用いることです．
この時の引数はxArmAPIで，これのインスタンス化の仕方は[xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK)と同じです．

このAPIの具体的な使い方は，[ping](examples/ping.py), [read_write](examples/read_write.py)などの例を参照してください．
