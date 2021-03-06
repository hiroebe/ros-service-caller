# ros-service-caller

`rosservice call`および`rostopic pub`に渡す引数をエディタで編集するためのツール

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
$ ros-service-caller service <service name>
$ ros-service-caller topic <topic name>
```

実行するとテキストエディタが開き，`rosservice call`または`rostopic pub`に渡す引数を`yaml`ファイルとして編集することができます．保存してエディタを閉じると，それらの引数を用いて`rosservice call <service>`または`rostopic pub <topic>`が実行されます．

また，引数を既存の`yaml`ファイルから読み込むこともできます．これには`--file, -f`オプションを使います．

```
$ ros-service-caller -f data.yaml <service>
```

`rostopic pub`に渡すオプションとして，`-1,--once`のみ対応しています．

```
$ ros-service-caller topic <topic> -1
```

## Configuration (Optional)

`~/.config/ros-service-caller/config.yaml`に設定を書きます．設定可能な項目は以下の3つです．

```yaml
editor: vim                              # 起動するエディタ(デフォルトはvim)
selectcmd: fzf                           # filterを行うコマンド
histfile: ~/.ros_service_caller_history  # historyを書き込むファイル
```

`selectcmd`は，次の2つで動作確認をしています．

- [fzf](https://github.com/junegunn/fzf)
- [peco](https://github.com/peco/peco)

これらのコマンドは，別途インストールしておく必要があります．`selectcmd`を指定していると，service名/topic名なしで実行できます．

```
$ ros-service-caller service
$ ros-service-caller topic
```

実行すると，`selectcmd`で指定したツールにより，service名/topic名を絞り込んで選択できます．

`histfile`を設定しておくと，ros-service-callerで実行したコマンドの履歴が記録され，次のコマンドで再度呼び出すことができます．

```
$ ros-service-caller history
```

`selectcmd`にオプションを指定することで，プレビューを表示することもできます．以下はその例です．

```yaml
selectcmd: fzf --preview="echo {}" --preview-window=up:wrap
```
