from bayes_opt import BayesianOptimization

import GPy
import GPyOpt
import numpy as np


def f(x):
    return x*x

def opt_gpyopt():
    bounds = [
        {'name': 'x', 'type': 'continuous', 'domain': (0,10)}
    ]
    myBopt = GPyOpt.methods.BayesianOptimization(f=f, domain=bounds, initial_design_numdata=5, maximize=True)

    myBopt.run_optimization(max_iter=30)

    print(f"f: {-myBopt.fx_opt} ")#gpyoptの仕様で最大値にマイナスがかかってる
    print(f"x: {myBopt.x_opt[0]} ")

def opt_bopt():
    pbounds = {
        'x': (0, 10)
    }

    optimizer = BayesianOptimization(f=f, pbounds=pbounds)

    optimizer.maximize(init_points=5, n_iter=30)

    print(f"f: {optimizer.max['target']} ")
    print(f"x: {optimizer.max['params']['x']} ")


opt_gpyopt()
opt_bopt()