import numpy as np
from scipy.spatial.transform import Rotation as R

sequence = 'ZYX'
att1 = R.from_euler(seq=sequence, angles=[-30, 0, 0], degrees=True)
att2 = R.from_euler(seq=sequence, angles=[45, 180, 180], degrees=True)

rotation = att2 * att1.inv()
mnvr_angles = rotation.as_euler(seq=sequence, degrees=True)
print(mnvr_angles)
