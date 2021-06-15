
Two dimensional optimal velocity robot
====

## Overview
2次元最適速度ロボット(2dOV Robot)とは，[2dOVモデル](http://traffic.phys.cs.is.nagoya-u.ac.jp/%7Emstf/pdf/mstf2016-11.pdf)
に基いてその行動（モーターの駆動）が生成される走行ロボットのことです．

2dOVモデルとは２次元平面における生物などの行動を数理モデル化したものです．
紐状の運動など，変化に富んだ集団行動を再現することが知られています．
このレポジトリでは，その2dOVモデルをアルゴリズム化し，２次元平面を走行するスキッドステアロボット[SSR2](https://github.com/HondaLab/SSR2)
に適用するPythonコードを提供します．

## Description
詳しくは
[このレポジトリのwiki](https://github.com/HondaLab/2DOVR/wiki)を参照してください．

## Requirement
  * [SSR2ロボット](https://github.com/HondaLab/SSR2)
  * Raspberry Pi 3 or 4
  * Parallax High speed continuous servo (x2)
  * 5V 3A バッテリー（18650 x2 + DC-DCコンバータ あるいはモバイルバッテリー)


## History
### 2021 6/1
 * tanhによる感覚運動写像とOVアルゴリズムの積によりモーター出力を決定する
 * スキッドステアリングのために，ゲイン self.d=1 --> 5と増加
 * SSR2ロボット３台により初めて紐状走行観測
 * 左右tofセンサーのgpioに注意．
### 2021 5/26
 * omega = atanh(vx/vy)
 * vL= v + r * omega
 * vR= v - r * omega





 

