import ctypes
import platform
import os

file_extension = '.so'
if platform.system() =='cli':
    file_extension = '.dll'
elif platform.system() =='Windows':
    file_extension = '.dll'
elif platform.system() == 'Darwin':
    file_extension = '.so'
else:
    file_extension = '.so'
path = os.path.join(os.path.dirname(__file__), 'cpsr' + file_extension)

intersect_in_c = ctypes.CDLL(path)

intersect_in_c.intersect.argtypes = (ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.ARRAY(ctypes.c_float,3)))

point = (ctypes.c_float * 3)()


inter = intersect_in_c.intersect