# ros-service-caller

`rosservice call`に渡す引数をエディタで編集するためのツール

## Installation

[Release](https://github.com/hiroebe/ros-service-caller/releases)ページから`ros-service-caller-*.zip`をダウンロードして，適当なディレクトリに展開します．

`~/.zshrc`に以下を追記します．

```
export PATH=$PATH:{path to ros-service-caller}/bin
```

zsh補完を有効にするには，以下を追記します．

```
source {path to ros-service-caller}/completion.zsh
```

今のところ，補完はzshのみ対応しています．

## Usage

```
$ ros-service-caller <service>
```

サービス名にはタブ補完が使えます．実行するとテキストエディタが開き，`rosservice call`に渡す引数を`yaml`ファイルとして編集することができます．保存してエディタを閉じると，それらの引数を用いて`rosservice call <service>`が実行されます．

また，引数を既存の`yaml`ファイルから読み込むこともできます．これには`--file, -f`オプションを使います．

```
$ ros-service-caller -f data.yaml <service>
```

エディタを変更するには，環境変数`$EDITOR`を設定します(デフォルトはvim)．

```
$ EDITOR=nano ros-service-caller <service>
```

または，`~/.zshrc`に次のように追記します．

```
export EDITOR=nano
```
