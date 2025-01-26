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

# 実行方法
接続したDynamixelとの通信手段はほとんど[DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/main/python)と同じ．
違いは，デフォルトの`PortHandler`の代わりに`EndEffectorPortHandler`(src/endeffector_port_handler.py)を用いれば良い．
この時の引数はxArmAPIで，これのインスタンス化の仕方は[xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK)と同じ．

## サンプルコードの実行
```bash
cd xarm_eef_dynamixel_wrapper
python sample/read_write.py
```