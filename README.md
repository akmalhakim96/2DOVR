
Two dimensional optimal velocity robot
====

## Overview
2次元最適速度ロボット(2dOV Robot)とは，2dOVモデルに基いてその行動（モーターの駆動）が生成される
走行ロボットのことです．

2dOVモデルとは２次元平面における生物などの行動を数理モデル化したものです．
紐状の運動など，変化に富んだ集団行動を再現することが知られています．
このレポジトリでは，その2dOVモデルをアルゴリズム化し，２次元平面を走行するロボットに適用するプログラムを開発します．

## Description
詳しくは
[このレポジトリのwiki](https://github.com/HondaLab/2DOVR/wiki)を参照してください．

## Requirement
  * Raspberry Pi3
  * Parallax High speed continuous servo (x2)


## History
2021 10/19
2dovr.py : 実行するソースコード
past_srcというディレクトリを作成しました。
これは過去のソースコードを入れておくディレクトリです。
2dovr_multi.pyはOVとSMMを掛け算して出力を求めていたものです。
2dovr_211014は壁に沿う動きを創発したソースコードです。

2021 9/23
一斉スタート用のプログラムをparamiko_2dovr内部に置きました


2021 5/26
 * omega = atanh(vx/vy)
 * vL= v + r * omega
 * vR= v - r * omega





 

