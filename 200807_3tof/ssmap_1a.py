#!/usr/bin/python3
# CC BY-SA Yasushi Honda 2020.8.7
# Double tanh関数を描画

import fview_1a as fv
from math import *

def ssmap(x):
    alpha=7
    alpha2=7
    beta=0.008
    beta2=10
    b=300
    c=4

    f=alpha*tanh(beta*(x-b))+alpha2*tanh(beta2*(x-b))+c
    return f



fv.fview(ssmap,0,800)
