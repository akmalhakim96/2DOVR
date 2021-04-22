import sympy
from sympy.plotting import plot

sympy.var('x')

alpha=1.0
beta=2.0
b=0.13
c=1.0

f=alpha*(sympy.tanh(beta*(x-b))+c)

plot(f,(x,0,1))
