def fview(func,x1,x2):
    #%matplotlib inline
    import matplotlib.pyplot as plt
    px=[]
    py=[]
    dx=(x2-x1)/400
    x=x1
    while x<x2:
        px.append(x)
        py.append(func(x))
        x+=dx
        
    plt.grid()
    # 関数を描画
    plt.plot(px,py) 
    plt.savefig('tmp.png')

if __name__=="__main__":
    from math import *    
    def ovf(r):

        # 係数を与える
        alpha=7
        alpha2=7
        beta=0.008
        beta2=10
        b=300
        c=4

        # 関数を決める
        f= alpha*tanh(beta*(r-b)) + alpha2*tanh(beta2*(r-b)) + c
    
        return f
    
    fview(ovf,0,800)
