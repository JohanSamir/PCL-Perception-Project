
# coding: utf-8

# # Velodyne Points Projected into Image

# In[123]:


import pykitti
import numpy as np
import cv2
import matplotlib.pyplot as plt
from open3d import *

'''Image and Sparse Laser Fusion'''
basedir = '/home/johan/Desktop/UAO Projects_2/KitiiDatabase'
date = '2011_09_26'
drive = '0005'
#drive = '0001'

dataset = pykitti.raw(basedir, date, drive)
#img = plt.imread("/home/johan/Desktop/Jackal/Files/Classification/Classification1/Images/img126.png")
#img = plt.imread("/home/johan/Desktop/Jackal/Files/Classification/Classification1/Images/img168.png")
img = plt.imread("/home/johan/Desktop/Jackal/Files/Classification/Classification1/Images/img111.png")
#img = img_o[0:420, 0:1280]

#Velopoints = read_point_cloud("/home/johan/Desktop/Jackal/Files/Classification/Classification1/Points/points168.pcd")
Velopoints = read_point_cloud("/home/johan/Desktop/Jackal/Files/Classification/Classification1/Points/points111.pcd")

#draw_geometries([Velopoints])

#print 'pcd',Velopoints.shape
Velopoints = np.asarray(Velopoints.points,np.float32)
#Velopoints = dataset.get_velo(15) #67, image 67

#Loading the calibration parameters
P_rect_20= dataset.calib.P_rect_20
R_rect_20= dataset.calib.R_rect_20
T_rect_20= dataset.calib.T_cam0_velo_unrect
print('P_rect_20')
P_rect_20 = np.matrix(P_rect_20)
print(P_rect_20, P_rect_20.shape, type(P_rect_20))
print('R_rect_20')
R_rect_20 = np.matrix(R_rect_20)
print(R_rect_20, R_rect_20.shape, type(R_rect_20))
print('T_rect_20')
T_rect_20 = np.matrix(T_rect_20)
print(T_rect_20, T_rect_20.shape, type(T_rect_20))


#A = np.identity(4)+0.1
#Trasl = np.array([0.22,0.06,-0.1103])
#A[:3,3] = Trasl
#T_rect_20 = A
#print('T_rect_20',T_rect_20, T_rect_20.shape, type(T_rect_20))

#T_rect_20 = np.array([[ 7.533745e-03, -9.999714e-01, -6.166020e-04, 0.22],
        #[ 1.480249e-02,  7.280733e-04, -9.998902e-01, 0.06],
        #[ 9.998621e-01,  7.523790e-03,  1.480755e-02, -0.1103],
        #[ 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00]])


Velopoints = np.asarray(Velopoints, np.float32)
print 'Velopoints\n',Velopoints,Velopoints.shape,type(Velopoints)

T0 = P_rect_20 * R_rect_20
T1= T0 * T_rect_20


# In[78]:


#Velopoints = Velopoints[::2]
idx = Velopoints[:,0]<0.2
Velopoints = np.delete(Velopoints, np.where(idx),0)

Velopoints3P = Velopoints[:,0:3]
Velopoints_getvalues = Velopoints3P
np.savetxt('Raw_PointCloud_Total'+'txt', Velopoints3P, delimiter=' ')

print(type(Velopoints3P),Velopoints3P.shape)
Velopoints3P_3D_Plot = Velopoints3P

pcd = PointCloud()
pcd.points = Vector3dVector(Velopoints3P_3D_Plot)
write_point_cloud("/home/johan/Desktop/Jackal/Files/Point_Cloud_Raw_Points.ply", pcd)
pcd_load = read_point_cloud("/home/johan/Desktop/Jackal/Files/Point_Cloud_Raw_Points.ply")
#draw_geometries([pcd_load])


# In[79]:


dim_M = T1.shape[0]
dim_N = T1.shape[1]
print('M',dim_M,' ','N',dim_N,'\n')

if Velopoints3P.shape[1] < dim_N:
    ones_vect_Velo3p = np.ones ((Velopoints3P.shape[0], 1),int)
    Velopoints3P = np.concatenate ((Velopoints3P, ones_vect_Velo3p), axis = 1)

Velopoints3P = np.matrix(Velopoints3P)
print'Velopoints3P or Velopoints3PP: \n',Velopoints3P,Velopoints3P.shape,type(Velopoints3P)
Velopoints3PP = Velopoints3P



# In[80]:


Velopoints3P = np.transpose(Velopoints3P)
print('Velopoints3P Transpose',' ', Velopoints3P.shape, '\n')

y = (T1 * Velopoints3P)
y = np.transpose(y)
print('y: ',y, y.shape, type(y),'\n')
y_final = y


# In[81]:


x_y = y[:,0:dim_M-1]
b_ones = np.ones((1, dim_M-1),int)
z = y[:,dim_M-1]
p_out = np.divide(x_y, np.multiply(z , b_ones))
print(p_out, p_out.shape , type(p_out))

idx_p_out = np.arange(0,p_out.shape[0], 1)
print(idx_p_out.shape)


# In[82]:


imgplot = plt.imshow(img)
plt.axis("off")
plt.show()
x = p_out[:,0]
y = p_out[:,1]+150 #Important Parameter
plt.plot(x,y,'r.',markersize = 0.5)


# In[83]:


img_x = img.shape[1]
img_y = img.shape[0]
print ('img_x',img_x,'img_y',img_y,'\n')

# Evaluation of the column x for identifying the values outside the image rank
pointxt = np.logical_or(x < 0, x > img_x)
idex = np.where(np.logical_not(pointxt))[0]
pout_ft = p_out[idex,:]

#Index of point clouds
idx_p_outt = idx_p_out[idex]
xa = pout_ft[:,0]
yb = pout_ft[:,1]
pointyt = np.logical_or(yb < 0,yb > img_y)

idexy = np.where(np.logical_not(pointyt))[0]
pout_fty =pout_ft[idexy,:]
print('pout_fty,',pout_fty, pout_fty.shape)
idx_p_outtt = idx_p_outt[idexy]
print('idx_p_outtt',idx_p_outtt, idx_p_outtt.shape)

xaa = pout_fty[:,0]
ybb = pout_fty[:,1]

fig = plt.figure()
ax1 = fig.add_subplot(211)
ax1.plot(xa,yb,'r.',markersize = 1)

ax2 = fig.add_subplot(212)
ax2.plot(xaa,ybb,'b.',markersize = 1)

pout_fty = np.matrix.round(pout_fty)
pout_fty = pout_fty.astype(int)
print('pout_fty_int',pout_fty, pout_fty.shape, type(pout_fty))


# In[84]:


import matplotlib.pyplot as plt
imgplot = plt.imshow(img)
plt.plot(xaa,ybb,'r.',markersize=1)

matrix_points = np.zeros((img.shape[0], img.shape[1]))
print(matrix_points.shape)

for i in range(0,pout_fty.shape[0]):
    x = pout_fty[i,0] 
    y = pout_fty[i,1]
    ppoints = Velopoints3PP[idx_p_outtt[i],0:3]
       
    matrix_points[y-1,x-1] = np.sqrt(np.power( ppoints[:,0], 2)+ np.power( ppoints[:,1], 2) + np.power(ppoints[:,2], 2))

#np.savetxt('Data_set_pointcloud'+'.csv', matrix_points, delimiter=',')
print('Matrix with point',matrix_points.shape)


# In[85]:


f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex='col', sharey='row')

#image_padded = np.zeros((img.shape[0] + 2, img.shape[1] + 2))
gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#np.savetxt('gray_image'+'.csv', gray_image, delimiter=',')
#image_padded[1:-1, 1:-1] = gray_image

plt.figure(5)
# RGB - Blue
blue = img[:, :, 0]
#np.savetxt('blue'+'.csv', blue, delimiter=',')

# RGB - Green
green = img[:, :, 1]
#np.savetxt('green'+'.csv', green, delimiter=',')

# RGB Red
red = img[:, :, 2]
#np.savetxt('red'+'.csv', red, delimiter=',')

ax1.imshow(img[:, :, 0], plt.cm.Blues_r)
ax2.imshow(img[:, :, 1], plt.cm.Greens_r, )
ax3.imshow(img[:, :, 2],plt.cm.Reds_r)
ax4.imshow(gray_image ,cmap=plt.cm.gray)
print 'Image Value Pixel, Grayscale: ', gray_image[1][1]


# # Image and Sparse Laser Fusion

# In[86]:


from skimage.segmentation import slic
from segraph import create_graph
import matplotlib.pyplot as plt
from skimage.segmentation import mark_boundaries

image = img
print 'image shape\n', image.shape
segments = slic(image, n_segments = 1000, enforce_connectivity = True)
#segments are groups of region which was divided according to the color pixeles

#np.savetxt('segments'+'.csv', segments, delimiter=',')
#vetices are the nodes
# Create graph of superpixels
vertices, edges = create_graph(segments)
print 'vertices:\n',vertices.shape, type(vertices)
#print('vertices',vertices)

# Compute centers:
gridx, gridy = np.mgrid[:segments.shape[0], :segments.shape[1]]
centers = dict()
for v in vertices:
    centers[v] = [round(gridy[segments == v].mean()), round(gridx[segments == v].mean())]


# In[87]:


'''
Getting the branches' values according to the pixel postision (close to the edges, corners, 
or inside the image).
Here the value of 0° is respresentes as 360°
'''

b_grafo = np.zeros(len(centers))
t = 0
orientacion = dict()
average_pixel = dict()
average_pix_blue = dict()
average_pix_red = dict()
average_pix_green = dict()

average_points = dict()
rt = 0
for k in range (0,len(centers)):
#for k in range (0,2):
    #rint 'k1',k
    j=0
    i=0
    a = 0
    p = 0
    pb = 0
    pr = 0
    pg = 0
    pe = 0
    bg = 0
    for j in range(0,gray_image.shape[0]):
        for i in range(0,gray_image.shape[1]):
            #segments[y][x]
            #gray_image.shape[0] ---> y (j)
            #gray_image.shape[1] ---> x (i)
            
            if segments[j][i] == k:
                #print 'k2',k
                if j == 0 and i == gray_image.shape[1]-1:
                    #Point_1: y = 0 & x = max
                    b_grafo[t] = 2
                    #orientacion[t] = [180, 270]
                    orientacion[t] = [180, -90]
                elif j == 0 and i == 0:
                    #Point_2: y = 0 & x = 0 
                    b_grafo[t] = 2
                    #orientacion[t] = [270,360]
                    orientacion[t] = [-90,0]
                    k = len(centers)+1
                elif j == gray_image.shape[0]-1 and i == 0:
                    #Point_3: y = max & x = 0 
                    b_grafo[t] = 2
                    #orientacion[t] = [90,360]
                    orientacion[t] = [90,0]
                    k = len(centers)+1   
                elif j == gray_image.shape[0]-1 and i == gray_image.shape[1]-1:
                    #Point_4: y = max & x = max
                    b_grafo[t] = 2
                    orientacion[t] = [90,180]
                    k = len(centers)+1 
                elif j == 0 and i > 0 and i < gray_image.shape[1]-1:
                    #Vertical Line_1
                    b_grafo[t] = 3
                    #orientacion[t] = [180,270,360] 
                    orientacion[t] = [180,-90,0] 
                elif j == gray_image.shape[0]-1 and i > 0 and i < gray_image.shape[1]-1:
                    #Vertical Line_2
                    b_grafo[t] = 3
                    #orientacion[t] = [90,180,360]
                    orientacion[t] = [90,180,0]
                elif i == 0 and j > 0 and j < gray_image.shape[0]-1:
                    #Vertical Line_1
                    b_grafo[t] = 3
                    #orientacion[t] = [90,270,360]
                    orientacion[t] = [90,-90,0] 
                elif i == gray_image.shape[1]-1 and j > 0 and j < gray_image.shape[0]-1:
                    #Vertical Line_1
                    b_grafo[t] = 3
                    #orientacion[t] = [90,180,270]
                    orientacion[t] = [90,180,-90]
                #print 'kf',k
                q = gray_image[j][i]
                q_blue = blue[j][i]
                q_red= green[j][i]
                q_gren= red[j][i]

                qe = matrix_points[j][i]
                p = p + q
                pb = pb + q_blue
                pr = pr + q_red
                pg = pg + q_gren
                if qe != 0:
                    pe = pe + qe
                    a = a + 1
                bg = bg + 1    
    #Getting the average values of color and point cloud.
    average_pixel[rt] = p/bg
    average_pix_blue[rt] = pb/bg
    average_pix_red[rt] = pr/bg
    average_pix_green[rt] = pg/bg

    if a == 0:
        average_points[rt] = 0
    else:
        average_points[rt] = pe/a
    rt = rt +1
     
    t = t+1  
    #print 't',t
print 'ready!'        


# In[88]:


#Average of the chanels RGB for each segment
aaverage_pix_blue = np.asarray(average_pix_blue.values())
aaverage_pix_red = np.asarray(average_pix_red.values())
aaverage_pix_green = np.asarray(average_pix_green.values())

RG= np.column_stack((aaverage_pix_red,aaverage_pix_green))
RGB = np.column_stack((RG,aaverage_pix_blue ))

print type(RGB),RGB.shape, RGB
#np.savetxt('RGB'+'.txt', RGB, delimiter=' ') 


# In[89]:


#print 'b_grafo',b_grafo.shape,'\n',b_grafo
for i in range (0,len(b_grafo)):
    if b_grafo[i] == 0:
        b_grafo[i] = 4
        orientacion[i] = [90,180, -90, 0]
#print 'orientacion\n', orientacion, len(orientacion)


# In[90]:


# overlay graph_Method 1:
plt.figure(4)
plt.figure(figsize=(15, 6), dpi=80)
plt.imshow(segments)
for edge in edges:
    plt.plot([centers[edge[0]][0],centers[edge[1]][0]],
             [centers[edge[0]][1],centers[edge[1]][1]])
plt.show()


# In[91]:


''' Image Graph '''
tunning_pa = 1
l=0
edges_pairs = {}
many_edges_pairs = {}
for i in range(0,len(centers)):
    t=0
    for edge in edges:
       
        if edge[0]== i or edge[1]== i:
            edges_pairs[l] = edge
            l = l+1
            t = t+1
    many_edges_pairs[i]= t     


# In[92]:


'''Calculating the atan2 for every branch in the main graph. Moreover, a data conversion
was made for getting the values between 0° and 360°'''

edges_pair_wise = {}
a = edges_pairs.values()
f_g = many_edges_pairs[0]
t = 0
f = 0
s = 0
r = 0
for edge in a:
    f_g = many_edges_pairs[f]
    #print 'f',f_g
    s = s + 1
    r_s = f_g - s
    #print 'r_s',r_s
    if r_s >= 0:
        if edge[0] == r:
            y = centers[edge[1]][1] - centers[edge[0]][1] 
            x = centers[edge[1]][0] - centers[edge[0]][0] 
            e = (np.arctan2(y,x) * 180 / np.pi)
            #To convert the angle according to righ coordenates.
            #-1 represents the inverse of the Y axle
            if e == 0:
                e = e 
            else:
                e = e*-1
            edges_pair_wise[t] = e
            
        else:
            y = centers[edge[0]][1] - centers[edge[1]][1]
            x = centers[edge[0]][0] - centers[edge[1]][0]           
            e = (np.arctan2(y, x) * 180 / np.pi)
            #To convert the angle according to righ coordenates.
            #-1 represents the inverse of the Y axle
            if e == 0:
                e = e 
            else:
                e = e*-1        
            edges_pair_wise[t] = e
            #print 'b'
        t = t + 1 
        if r_s == 0:
            f = f + 1
            s = 0
            r = r + 1


# In[93]:


'''
Getting the four neighboors for each pixel in the image.
'''
u = -1
a = 0
f = many_edges_pairs[0]
indx_angulos =[]
angu_test = dict()
angu_test_idx = dict()

for i in range (0,len(centers)):
    angulos = []
    angulos_idx = []
    orientacio_nu = orientacion[i]
    b_grafo_nu = int(b_grafo[i])
    many_edges_pairs_num = many_edges_pairs[i]

    for j in range(a,f):
        u = u + 1
        angulos = np.append(angulos,edges_pair_wise[j])
        angulos_idx = np.append(angulos_idx,u)
    l = np.array(angulos)
    
    angu_test[i] = l
    angu_test_idx[i] = angulos_idx
    a = many_edges_pairs_num + a
    if i < len(centers)-1:
        f = a + many_edges_pairs[i+1]


# In[94]:


graph = []
graph_dict_many = dict()
graph_dict = dict()
for i in range (0,len(centers)):
    many_edges_pairs_num = many_edges_pairs[i]
    orientacio_nu = orientacion[i]
    b_grafo_nu = int(b_grafo[i])
    u = []
    
    for z in range(0,b_grafo_nu):
        t = orientacio_nu[z]
        a = []
        for s in range (0,many_edges_pairs_num):
            #print 's',s
            if t == -90: 
                p = abs(t-angu_test[i][s])
                a = np.append(a,p)
            elif t == 0:
                p = abs(t-angu_test[i][s])
                a = np.append(a,p)
            elif t == 90:
                p = abs(t-angu_test[i][s])
                a = np.append(a,p)
            elif t == 180:
                p = abs(t-abs(angu_test[i][s]))
                a = np.append(a,p)
        min_value = np.argmin(a)
        min_value = angu_test_idx[i][min_value]
        coorde_min_value = edges_pairs[min_value]
        u = np.append(u, coorde_min_value)
        graph = np.append(graph,coorde_min_value)
    
    graph_dict[i] = u 
    u = len(u)
    graph_dict_many[i] = u
               
euler = np.zeros((len(graph),1))/2


# In[95]:


#Test
for i in range (0, len(graph_dict)):
     if graph_dict_many[i]>8:
        print 'ok'


# In[96]:


plt.figure(4)
plt.figure(figsize=(15, 6), dpi=80)
plt.imshow(segments)
for i in range (0,len(graph),2):
    a = graph[i]
    b = graph[i +1]
    
    plt.plot([centers[a][0],centers[b][0]],
             [centers[a][1],centers[b][1]])
plt.show()


# In[97]:


#Ploting the segments and the points projected on the image.
plt.figure(4)
plt.figure(figsize=(15, 6), dpi=80)
plt.imshow(segments)
for i in range (0,len(graph),2):
    a = graph[i]
    b = graph[i +1]
    
    plt.plot([centers[a][0],centers[b][0]],
             [centers[a][1],centers[b][1]])
plt.plot(xaa,ybb,'r.',markersize=2)    
plt.show()


# In[98]:


#Ploting the image and the points projected on it
plt.figure(4)
plt.figure(figsize=(15, 6), dpi=80)
mgplot = plt.imshow(img)
for i in range (0,len(graph),2):
    a = graph[i]
    b = graph[i +1]
    
    plt.plot([centers[a][0],centers[b][0]],
             [centers[a][1],centers[b][1]])
plt.plot(xaa,ybb,'w.',markersize=2)    
plt.show()


# # Cost function with Data Cost and Discontinuity Cost

# In[99]:


from random import randint

p = 0
r = 0
measure_confi = 1
diagonal = np.zeros((len(centers),1))
V_points = np.zeros((len(centers),1))

for i in range(0,len(centers)):
                
        V_points[i] = average_points[i]
        
        if average_points[i] > 0:
            diagonal[i] = measure_confi
        else:
            diagonal[r] = 0
                   
x = np.eye(diagonal.shape[0])
diagonal = x * diagonal
print 'Vector Points cloud: ',V_points.shape
print 'W Matrix: (Diagonal)',diagonal.shape
#np.savetxt('V_points'+'.csv', V_points, delimiter=',') 


# In[100]:


'''Adjencia Matrix'''
Ad_m = np.zeros((sum(graph_dict_many.values())/2,len(centers)), dtype = float)
d = 0
k = 0
euler = np.zeros((len(edges_pairs),1))
f=0

for i in range (0,len(centers)):
    for j in range (0,len(graph_dict[i]),2):
        a = j+1
        if graph_dict[i][j]== i:
            Ad_m[f][int(graph_dict[i][j])] = 1
            Ad_m[f][int(graph_dict[i][a])] = -1
            
                                
        elif graph_dict[i][a] == i:
            Ad_m[f][int(graph_dict[i][a])] = 1
            Ad_m[f][int(graph_dict[i][j])] = -1
        
        f=f+1 
print 'Ad_m :\n', Ad_m, Ad_m.shape,type(Ad_m)
#np.savetxt('Ad_m'+'.csv', Ad_m, delimiter=',')


# In[101]:


'''In this part of the code, we calculated S.
S = E * Ad_m where E = [It is a vector of differece in pixel appereance] and Ad_m = [it is a Adjacency matrix]
'''
e = []
tunning_pa = 0.02
for i in range (0,len(centers)):
    for j in range (0,len(graph_dict[i]),2):
        a = j+1
        if graph_dict[i][j]== i:
            m1 = np.power((average_pixel[graph_dict[i][j]]-average_pixel[graph_dict[i][a]]),2)
            m1 = (m1 / tunning_pa) * (-1)
            m1 = np.exp(m1)  
            e = np.append(e, m1)
                          
        elif graph_dict[i][a] == i:
            m1 = np.power((average_pixel[graph_dict[i][a]]-average_pixel[graph_dict[i][j]]),2)
            m1 = (m1 / tunning_pa) * (-1)
            m1 = np.exp(m1)                
            e = np.append(e, m1)              
                        
print 'e : ',e, e.shape    


# In[102]:


'''In this part of the code, we calculated S.
S = E * Ad_m where E = [It is a vector of differece in pixel appereance] and Ad_m = [it is a Adjacency matrix]
'''
s = []
p = 0

for i in range (0,Ad_m.shape[0]):
    for j in range (0,Ad_m.shape[1]):
        
        if Ad_m[i,j] == 1:
            Ad_m[i,j] = Ad_m[i][j]*e[p]    
        
        elif Ad_m [i][j] == -1:
            Ad_m [i][j] = Ad_m [i][j]*e[p]   
                
print 'Ad_m_with_euler: \n', Ad_m, Ad_m.shape               
#np.savetxt('Ad_m_with_euler'+'.csv', Ad_m, delimiter=',')      


# In[103]:


import scipy.sparse.linalg
from scipy.sparse.linalg import cgs

lamda_1 = 0.4
lamda_2 = 1

SS = np.transpose(Ad_m)
SS = np.matmul(SS,Ad_m)
print('Matrix S:\n',SS.shape)

WW = np.transpose(diagonal)
WW = np.matmul(WW,diagonal)
print('Matrix W:\n', WW.shape)

A = (((lamda_1*lamda_2)*SS)+((1-lamda_1)*WW))/(1-lamda_1)
print('Matrix A:\n',A.shape)

B = np.matmul(WW,V_points)
print('Matrix B:\n',B.shape)

x_values_inf = cgs(A, B)
print('Matrix x (Inference):\n',x_values_inf ,len(x_values_inf [0]),type(x_values_inf ))
#np.savetxt('x inference'+'.csv', x_values_inf [0], delimiter=',')   


# In[104]:


import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
import numpy as np
from mpl_toolkits.axes_grid1 import make_axes_locatable

fig, ax = plt.subplots()
imgplot = ax.imshow(img)
fig.set_size_inches(14, 10)
ybb = ybb +10
c = xaa.tolist()
d = ybb.tolist()
f = Velopoints3PP[:,0:1]
f = f[idx_p_outtt]
f = f.tolist()

image_new = img
img = ax.scatter(c,d, s= 3, marker=".",c=f, cmap=plt.cm.jet)
divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="1%", pad=0.05)
plt.colorbar(img, cax=cax)

plt.show()


# In[124]:


import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
import numpy as np
from mpl_toolkits.axes_grid1 import make_axes_locatable

fig, ax = plt.subplots()
imgplot = ax.imshow(img)
fig.set_size_inches(14, 10)
ybbb = np.copy(ybb)+130
xaaa = np.copy(xaa)+16
c = xaaa.tolist()
d = ybbb.tolist()
f = Velopoints3PP[:,0:1]
f = f[idx_p_outtt]
f = f.tolist()

image_new = img
img = ax.scatter(c,d, s= 3, marker=".",c=f, cmap=plt.cm.jet)
divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="1%", pad=0.05)
plt.colorbar(img, cax=cax)

plt.show()


# In[ ]:


import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
import numpy as np
from mpl_toolkits.axes_grid1 import make_axes_locatable

fig, ax = plt.subplots()
imgplot = ax.imshow(img)
fig.set_size_inches(14, 10)
yf = np.copy(ybb)+130
xf = np.copy(xaa)+100
c = xaa.tolist()
d = ybbb.tolist()
f = Velopoints3PP[:,0:1]
f = f[idx_p_outtt]
f = f.tolist()

image_new = img
img = ax.scatter(c,d, s= 3, marker=".",c=f, cmap=plt.cm.jet)
divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="1%", pad=0.05)
plt.colorbar(img, cax=cax)

plt.show()


# In[ ]:


f, (ax1, ax2) = plt.subplots(2, 1, sharey=True)
f.set_size_inches(14, 10)
imgplot = ax1.imshow(segments)
imgplot = ax2.imshow(segments)

r = []
t = []
k = []
r1 = []
t1 = []
k1 = []

for i in range (0, len(centers)):
    
    if V_points[i] != 0:
        c = int(centers[i][0])
        r = np.append(r, c)
        r = r.tolist()
        d = int(centers[i][1])
        t = np.append(t, d)
        t = t.tolist()
        f = V_points[i]
        k = np.append(k,f)
        k = k.tolist()
                
    c1 = int(centers[i][0])
    r1 = np.append(r1, c1)
    r1 = r1.tolist()
    d1 = int(centers[i][1])
    t1 = np.append(t1, d1)
    t1 = t1.tolist()
    f1 = x_values_inf[0][i]
    k1 = np.append(k1,f1)
    k1 = k1.tolist()    
        
img = ax1.scatter(r,t, s= 50, marker=".",c=k, cmap=plt.cm.jet)
divider = make_axes_locatable(ax1)
cax = divider.append_axes("right", size="1%", pad = 0.05)
plt.colorbar(img, cax=cax, label=' Depth')

img1 = ax2.scatter(r1,t1, s= 50, marker=".",c=k1, cmap=plt.cm.jet)
divider = make_axes_locatable(ax2)
cax2 = divider.append_axes("right", size="1%", pad = 0.05)
plt.colorbar(img1, cax=cax2, label=' Depth')


# In[ ]:


from skimage import exposure
f, (ax1, ax2) = plt.subplots(2, 1, sharey=True)
f.set_size_inches(14, 10)

img = image_new
gamma_corrected = exposure.adjust_gamma(img, 0.4)

ax1.imshow(mark_boundaries(gamma_corrected , segments,color=(0, 0, 0)))
ax2.imshow(mark_boundaries(gamma_corrected , segments,color=(0, 0, 0)))

r = []
t = []
k = []
r1 = []
t1 = []
k1 = []

for i in range (0, len(centers)):
    
    if V_points[i] != 0:
        c = int(centers[i][0])
        r = np.append(r, c)
        r = r.tolist()
        d = int(centers[i][1])
        t = np.append(t, d)
        t = t.tolist()
        f = V_points[i]
        k = np.append(k,f)
        k = k.tolist()
                
    c1 = int(centers[i][0])
    r1 = np.append(r1, c1)
    r1 = r1.tolist()
    d1 = int(centers[i][1])
    t1 = np.append(t1, d1)
    t1 = t1.tolist()
    f1 = x_values_inf[0][i]
    k1 = np.append(k1,f1)
    k1 = k1.tolist()    
        
img = ax1.scatter(r,t, s= 50, marker=".",c=k, cmap=plt.cm.jet)
divider = make_axes_locatable(ax1)
cax = divider.append_axes("right", size="1%", pad = 0.05)
plt.colorbar(img, cax=cax, label=' Depth')

img1 = ax2.scatter(r1,t1, s= 50, marker=".",c=k1, cmap=plt.cm.jet)
divider = make_axes_locatable(ax2)
cax2 = divider.append_axes("right", size="1%", pad = 0.05)
plt.colorbar(img1, cax=cax2, label=' Depth')


# # PointCloud Visualization

# In[ ]:


'''Point Cloud Raw Visualization'''
from open3d import *

pcd = PointCloud()
print 'Velopoints3P_3D_Plot',Velopoints3P_3D_Plot, type(Velopoints3P_3D_Plot),Velopoints3P_3D_Plot.shape

pcd.points = Vector3dVector(Velopoints3P_3D_Plot)
write_point_cloud("pointcloud.ply", pcd)

pcd_load = read_point_cloud("pointcloud.ply")
draw_geometries([pcd_load])


# In[ ]:


'''Point Cloud Super Pixels, Visualization of average_points on the image'''
print 'P_rect_20:',P_rect_20,'\n'

xc = P_rect_20[0,2]
yc =  P_rect_20[1,2]
f = P_rect_20[0,0]

print 'xc:',xc,'yc:',yc,'f:',f

#Centers's information
Y_fi = []
X_fi = []
Z_fi = []

for i in range (0,len(centers)):
    x_center = centers[i][0]
    y_center = centers[i][1]

    y = (y_center - yc)
    x = (x_center - xc)

    Y = (y * average_points[i])/((f**2 + y**2)**(1/2.0))
    Y_fi = np.append(Y_fi, Y)

    X = (x * average_points[i])/((f**2 + x**2)**(1/2.0))   
    X_fi = np.append(X_fi, X)

    Z = (f * Y) / y                            
    Z_fi = np.append(Z_fi, Z)
    
print 'X:',X_fi.shape,'Y:',Y_fi.shape,'Z:',Z_fi.shape
print 'X:',type(X_fi)   np.savetxt

Point_Cloud_Segmen = np.column_stack((X_fi, Y_fi))
Point_Cloud_Segmen = np.column_stack((Point_Cloud_Segmen, Z_fi))  
print 'Point_Cloud_Segmen', Point_Cloud_Segmen.shape
np.savetxt('XYZ_Raw_Points'+'.txt', Point_Cloud_Segmen  , delimiter=' ')

pcd = PointCloud()
pcd.points = Vector3dVector(Point_Cloud_Segmen)
write_point_cloud("Point_Cloud_Raw_Points.ply", pcd)
pcd_load = read_point_cloud("Point_Cloud_Raw_Points.ply")
a = draw_geometries([pcd_load])


# In[ ]:


'''Point Cloud Super Pixels, Visualization of estimated points on the image'''

xc = P_rect_20[0,2]
yc =  P_rect_20[1,2]
f = P_rect_20[0,0]

print 'xc:',xc,'yc:',yc,'f:',f
#y_values_inference
#Centers's information
Y_fi = []
X_fi = []
Z_fi = []

for i in range (0,len(centers)):
    x_center = centers[i][0]
    y_center = centers[i][1]

    y = (y_center - yc)
    x = (x_center - xc)
    
    Y = (y * x_values_inf[0][i])/((f**2 + y**2)**(1/2.0))
    Y_fi = np.append(Y_fi, Y)

    X = (x *  x_values_inf[0][i])/((f**2 + x**2)**(1/2.0))   
    X_fi = np.append(X_fi, X)

    Z = (f * Y) / y                            
    Z_fi = np.append(Z_fi, Z)
    
print 'X:',X_fi.shape,'Y:',Y_fi.shape,'Z:',Z_fi.shape
print 'X:',type(X_fi)  

PC_Segmen_in = np.column_stack((X_fi, Y_fi))
PC_Segmen_in = np.column_stack((PC_Segmen_in, Z_fi))  
print 'Point_Cloud_XYZ_inference', PC_Segmen_in.shape, type(PC_Segmen_in)
np.savetxt('Point_Cloud_XYZ_inference'+'.txt', PC_Segmen_in , delimiter=' ')


# In[ ]:


#For getting a XYZRGB we did the following:
RGB_255 = RGB*255
print 'RGB_255',RGB_255, RGB_255.shape
XYZRGB_inference_points = np.column_stack((PC_Segmen_in, RGB_255))

print 'XYZRGB_inference_points',XYZRGB_inference_points.shape, type(XYZRGB_inference_points)
np.savetxt('XYZRGB_inference_points'+'.txt', XYZRGB_inference_points, delimiter=' ')


# In[ ]:


#For putting color to every raw point we did the following:

RGB_average = np.zeros((len(centers),3))
for i in range(0,len(centers)):
    if Point_Cloud_Segmen[i,0]!=0 or Point_Cloud_Segmen[i,1]!=0 or Point_Cloud_Segmen[i,2]!=0:
        RGB_average[i] = RGB_255[i]
print 'RGB_255_average',RGB_average.shape
XYZRGB_rawpoints = np.column_stack((Point_Cloud_Segmen, RGB_average))
print 'XYZRGB_rawpoints',XYZRGB_rawpoints.shape, type(XYZRGB_rawpoints)
np.savetxt('XYZRGB_255_average'+'.txt', XYZRGB_rawpoints, delimiter=' ')

