import codac as cd
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

from Interval_observer import IntervalObserver


Obs = IntervalObserver(np.array([cd.IntervalVector([[-0.5, 0.5], [-0.5, 0.5]])]),
                       np.array([cd.IntervalVector([[9.5, 10.5], [9.5, 10.5]])]),
                       np.array([cd.IntervalVector([[-2.5, -1.5], [-2.5, -1.5]])]))


current_self_pos = np.zeros((2,1))

print(Obs.self_pos)
print(Obs.other_pos_1)
print(Obs.other_pos_2)

print((Obs.self_pos[0,0]))
print((Obs.self_pos[0,1].lb()))
print((Obs.self_pos[0,0].lb(), Obs.self_pos[0,1].lb()))





ax = plt.gca()
ax.set_xlim([-4, 12])
ax.set_ylim([-4, 12])

Obs.plot(ax)
sleep(2)

measurements = {
    'other_pos_1': cd.Interval([14, 14.5]),
    'other_pos_2': cd.Interval([2.5, 3.0])
}
Obs.predict(pos=current_self_pos)
Obs.update(measurements)
Obs.plot(ax)
sleep(2)

Obs.predict(pos=current_self_pos)
measurements = {
    'other_pos_1': cd.Interval([14, 14.5]),
    'other_pos_2': cd.Interval([2.5, 3.0])
}
Obs.update(measurements)
Obs.plot(ax)


plt.show()