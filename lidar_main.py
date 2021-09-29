import matplotlib.pyplot as plt
import numpy as np

# -------------------------------------------
# input parameters

f = open("lidar_input.txt", 'r')

f.readline()
camera_x = float(f.readline().strip());
f.readline()
camera_y = float(f.readline().strip());
f.readline()
camera_height = float(f.readline().strip());
f.readline()
road_xmin = float(f.readline().strip());
f.readline()
road_xmax = float(f.readline().strip());
f.readline()
bg_xmin = float(f.readline().strip());
f.readline()
bg_xmax = float(f.readline().strip());
f.readline()
bg_ymin = float(f.readline().strip());
f.readline()
bg_ymax = float(f.readline().strip());
f.readline()
blkice_xmin = float(f.readline().strip());
f.readline()
blkice_xmax = float(f.readline().strip());
f.readline()
blkice_ymin = float(f.readline().strip());
f.readline()
blkice_ymax = float(f.readline().strip());
f.readline()
hor_min = float(f.readline().strip());
f.readline()
hor_max = float(f.readline().strip());
f.readline()
hor_interval = float(f.readline().strip());
f.readline()
ver_min = float(f.readline().strip());
f.readline()
ver_max = float(f.readline().strip());
f.readline()
ver_interval = float(f.readline().strip());
f.readline()
range_noise_amp = float(f.readline().strip());
f.readline()
intensity_noise_amp = float(f.readline().strip());
f.readline()
reflection_rate = float(f.readline().strip());
f.readline()
ice_reduce = float(f.readline().strip());

f.close()


# ------------------------------------------------------------
# 수평(hor), 수직(ver) 각도   (radian, degree 별도 선언)
# 간격(interval)에 따라 배열로 표현함.
# 0.00001 넣은 이유 : float형이 정확한 소수값을 나타내지 않아 이에 따른 오차를 보정하기 위해 사용됨.

ho = np.arange(hor_min, hor_max+0.00001, hor_interval) * np.pi / 180
ho_deg = np.arange(hor_min, hor_max+0.00001, hor_interval)
ve = np.arange(ver_min, ver_max+0.00001, ver_interval) * np.pi / 180
ve_deg = np.arange(ver_min, ver_max+0.00001, ver_interval)
ho_len = len(ho)
ve_len = len(ve)


# -----------------------------------------------------------
# Point cloud 좌표 계산 + 거리 noise 추가
#
# A : x좌표
# B : y좌표
# C : z좌표(지면)

range_noise = range_noise_amp * np.random.randn(ve_len, ho_len)
A = camera_x + camera_height * np.tan(ve.reshape(ve_len, 1)).dot(np.sin(ho.reshape(1, ho_len))) + np.sin(ve.reshape(ve_len, 1)).dot(np.sin(ho.reshape(1, ho_len))) * range_noise
B = camera_y + camera_height * np.tan(ve.reshape(ve_len, 1)).dot(np.cos(ho.reshape(1, ho_len))) + np.sin(ve.reshape(ve_len, 1)).dot(np.cos(ho.reshape(1, ho_len))) * range_noise
C = 0 * np.ones((ve_len, ho_len)) + range_noise_amp * np.cos(ve.reshape(ve_len, 1)).dot(np.ones((1, ho_len))) * range_noise;


# ----------------------------------------------------------
# Intensity noise 추가

intensity_noise = intensity_noise_amp * np.random.randn(ve_len, ho_len)


# ----------------------------------------------------------
# 하늘, 배경, 도로, 블랙아이스 그리기

	# 하늘
t1 = plt.Polygon([[hor_min-hor_interval, 90], [hor_max+hor_interval, 90], [hor_max+hor_interval, 135-0.5*ver_min], [hor_min-hor_interval, 135-0.5*ver_min]], color='lightskyblue', zorder=0)
plt.gca().add_patch(t1)

	# 배경
t2 = plt.Polygon([[hor_min-hor_interval, ver_min-ver_interval], [hor_max+hor_interval, ver_min-ver_interval], [hor_max+hor_interval, 90], [hor_min-hor_interval, 90]], color='limegreen', zorder=0)
plt.gca().add_patch(t2)

	# 도로
road_horangle_min = np.ones(ve_len);
road_horangle_max = np.ones(ve_len);

for i in range(0, ve_len):
        road_horangle_min[i] = np.arcsin((road_xmin - camera_x)/camera_height/np.tan(ve[i]))
        road_horangle_max[i] = np.arcsin((road_xmax - camera_x)/camera_height/np.tan(ve[i]))

for i in range(0, ve_len-1):
	t3 = plt.Polygon([[road_horangle_min[i]*180/np.pi, ve_deg[i]], [road_horangle_max[i]*180/np.pi, ve_deg[i]], [road_horangle_max[i+1]*180/np.pi, ve_deg[i+1]], [road_horangle_min[i+1]*180/np.pi, ve_deg[i+1]]], color='gray', zorder=1)
	plt.gca().add_patch(t3)

t4 = plt.Polygon([[road_horangle_min[ve_len-1]*180/np.pi, ver_max], [road_horangle_max[ve_len-1]*180/np.pi, ver_max], [0, 90]], color='gray', zorder=1)
plt.gca().add_patch(t4)

	# 블랙아이스
AA = [blkice_xmin, blkice_xmax, blkice_xmax, blkice_xmin]
BB = [blkice_ymin, blkice_ymin, blkice_ymax, blkice_ymax]
CC = np.zeros(4);
DD = np.zeros(4);

for i in range(0, 4):
	CC[i] = np.arctan(AA[i]/BB[i])*180/np.pi
	DD[i] = np.arctan(np.sqrt(AA[i]**2+BB[i]**2)/camera_height)*180/np.pi

t5 = plt.Polygon([[CC[0], DD[0]], [CC[1], DD[1]], [CC[2], DD[2]], [CC[3], DD[3]]], color='black', zorder=2)
plt.gca().add_patch(t5)

# ------------------------------------------------------------
# 반사율 계산
# d : 거리, I : intensity, value : 반사율

d = np.zeros((ve_len, ho_len))
I = np.zeros((ve_len, ho_len))
value = np.zeros((ve_len, ho_len))

for i in range(0, ve_len):
	for j in range(0, ho_len):
		d[i][j] = camera_height / np.cos(ve[i]) + range_noise[i][j]
		if A[i][j] > road_xmin and A[i][j] < road_xmax:
			if B[i][j] > blkice_ymin and B[i][j] < blkice_ymax and A[i][j] > blkice_xmin and A[i][j] < blkice_xmax:
				I[i][j] = ice_reduce * reflection_rate * np.cos(ve[i]) / d[i][j]**2 + intensity_noise[i][j]
				value[i][j] = I[i][j] * d[i][j]**3 / camera_height
			else:
				I[i][j] = reflection_rate * np.cos(ve[i]) / d[i][j]**2 + intensity_noise[i][j]
				value[i][j] = I[i][j] * d[i][j]**3 / camera_height
		else:
			I[i][j] = reflection_rate * np.cos(ve[i]) / d[i][j]**2 + intensity_noise[i][j]
			value[i][j] = I[i][j] * d[i][j]**3 / camera_height
		
		if value[i][j] > 1:
			value[i][j] = 1
		if value[i][j] < 0:
			value[i][j] = 0

		plt.scatter(ho_deg[j], ve_deg[i], color=[value[i][j], value[i][j], value[i][j]], zorder=3, s=5)

# -------------------------------------------------------------
# plot
# x,y축 범위, 라벨링

from PIL import Image

low_im = Image.fromarray(I)
low_im.show()

plt.xlim(hor_min-hor_interval, hor_max+hor_interval)
plt.ylim(ver_min, 135-0.5*ver_min)
plt.xlabel('Horizontal Angle (Degree)')
plt.ylabel('Vertival Angle (Degree)')
plt.show()
