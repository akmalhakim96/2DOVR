import sympy
from sympy.plotting import plot

sympy.var('x')

alpha=1
beta=5
b=1
c=0

f = alpha*(sympy.tanh(beta*(x-b))+c)

plot(f,(x,0,2*b))
