import math
from utils import *

angle = math.pi/2
vector1 = [1,0,0]
vector2 = body_to_stability(angle).rotate(vector1)

print('vector1: ' + str(vector1))
print('vector2: ' + str(vector2))
