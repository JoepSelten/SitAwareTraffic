import geopandas
from basic_functions import trans
import numpy as np
import math
import matplotlib.pyplot as plt

world = geopandas.read_file(geopandas.datasets.get_path('naturalearth_lowres'))
#print(type(world.geometry[0]))


pos = np.array([0,0])

lane = trans(pos, 0, 20, 20)
lane_id = 'asdf'

df = geopandas.GeoDataFrame(columns=['id', 'geometry'], geometry='geometry')
df.geometry = lane

print(df)

#print(lane)

plt.fill(*lane.exterior.xy)
#plt.plot(lane)
plt.show()