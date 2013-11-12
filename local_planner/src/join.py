import os
import math
from scipy.special import fresnel
from scipy.integrate import quad

maxKappa = 0.2
nIter = 100

class Pose:
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta

def sgn(x):
	if x > 0:
		return 1
	else:
		return -1

def dist(alpha):
	fresnelS, fresnelC = fresnel(math.sqrt(2 * alpha / math.pi))
	return math.cos(alpha) * fresnelC + math.sin(alpha) * fresnelS

def join(a, b):
	sigma = l = 0
	beta = math.atan((b.y - a.y) / (b.x - a.x))
	if ((a.theta - beta) == (beta - b.theta)):
		alpha = (b.theta - a.theta) / 2
		r = math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)
		sigma = 4 * math.pi * sgn(alpha) * (dist(abs(alpha)) ** 2) / (r ** 2)
		l = 2 * math.sqrt(2 * alpha / sigma)
	return sigma, l

def kappa(s, sigma, l):
	if 0 <= s and s < l / 2:
		return min(sigma * s, maxKappa)
	elif l / 2 <= s and s < l:
		return min(sigma * (l - s), maxKappa)
	else:
		return 0

def theta(s, sigma, l):
	val, error = quad(kappa, 0, s, args = (sigma, l))
	return val
	#if maxKappa < sigma * l / 2:
		#if 0 < s and s < maxKappa / sigma:
			#return sigma * s ** 2 / 2
		#elif maxKappa / sigma < s and s < l - maxKappa / sigma:
			#return maxKappa ** 2 / sigma / 2 + maxKappa * s
		#else:
			#return maxKappa * l + sigma * (s ** 2 / 4 - 5 * l * s / 4 + l ** 2 / 2)
	#else:
		#if 0 < s and s < l / 2:
			#return sigma * s ** 2 / 2
		#else:
			#return sigma * (l * (s - l / 2) - (s ** 2 / 2 - (l / 2) ** 2 / 2))

def integrandX(s, sigma, l):
	return math.cos(theta(s, sigma, l))

def integrandY(s, sigma, l):
	return math.sin(theta(s, sigma, l))

def generate(pose1, pose2, sigma, l):
	path = []
	for i in range(nIter):
		s = l * i / nIter
		x, errorX = quad(integrandX, 0, s, args = (sigma, l))
		y, errorY = quad(integrandY, 0, s, args = (sigma, l))
		t = theta(s, sigma, l)
		#print s, x, y, t
		path.append(Pose(pose1.x + x, pose1.y + y, pose1.theta + t))
	path.append(pose2)
	return path

pose1 = Pose(10., 10., 90. * math.pi / 180)
pose2 = Pose(20., 20., 0)

sigma, l = join(pose1, pose2)
print sigma, l

path = generate(pose1, pose2, sigma, l)
print len(path)
for pose in path:
	print pose.x, pose.y, pose.theta
