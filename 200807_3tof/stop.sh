#!/usr/bin/python3
# CC BY-SA Yasushi Honda 2020 5/16

import modules.motor as mt # モーターを回転させるためのモジュール

mL=mt.Lmotor(17) # 左モーター(gpio17番)
mR=mt.Rmotor(18) # 右モーター(gpio18番)

mL.run(0)
mR.run(0)
