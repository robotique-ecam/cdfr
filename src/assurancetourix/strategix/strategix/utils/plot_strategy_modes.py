#!/usr/bin/env python3

import matplotlib.pyplot as plt
from strategix.strategy_modes import strategies, get_time_coeff

strategy_mode = "NORMAL"

plt.axes()
plt.xlabel("Time")
plt.ylabel("Coefficient")
lw = len(strategies.get(strategy_mode))
for action, time_coeff in strategies.get(strategy_mode).items():
    x, y = [], []
    for i in range(101):
        x.append(i)
        y.append(get_time_coeff(i, action, strategy_mode))
    plt.plot(x, y, lw=lw, label=action)
    lw -= 1
plt.axis("scaled")
plt.legend()
plt.show()
