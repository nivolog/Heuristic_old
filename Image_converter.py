#!/usr/bin/env python
# coding: utf-8

# In[4]:


from PIL import Image
import numpy as np

num = '8'

im = Image.open('/home/what_is_love/Lab/maps/' + num + '/map.png')
p = np.array(im)
m = p[::,::,1]
m = m/255
m = abs(m-1)
m = m.astype(int)

f = open('/home/what_is_love/Lab/maps/' + num + '/map.txt', 'w')
f.write(str(m.shape[0]) + ' ' + str(m.shape[1]) + '\n')
f.write('150 425 \n325 100 \n')
for i in range(m.shape[0]):
    for j in range(m.shape[1]):
        f.write(str(j+1) + ' ' + str(i+1) + ' ' + str(m[i][j]) + '\n')


        
f.close()


# In[8]:


from PIL import Image
import numpy as np

num = '5'

im = Image.open('/home/what_is_love/Lab/maps/' + num + '/map.png')
p = np.array(im)
m = p[::,::,]

print(np.argwhere(m == [0, 0, 0]))



# In[ ]:




