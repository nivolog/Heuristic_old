#!/usr/bin/env python
# coding: utf-8

# In[2]:


from PIL import Image
import numpy as np
import pandas as pd

num = '8'

im = Image.open('/home/vlad/Heuristic_old/maps/' + num + '/map.png')
karta = np.array(im)

for number in range (1, 100):
	f = open('/home/vlad/Heuristic_old/maps/' + num + '/res/' + str(number) + '.txt', 'r')
	path = np.loadtxt(f).astype(int)



	for i in range(path.shape[0]):
		y = path[i][0]
		x = path[i][1]
		karta[x-1][y-1][0] = 255
		karta[x-1][y-1][1] = 0
		karta[x-1][y-1][2] = 0

	im_p = Image.fromarray(karta)
	im_p.save('/home/vlad/Heuristic_old/maps/' + num + '/res/' + 'total_map_path.png', format = 'png')


# In[ ]:
