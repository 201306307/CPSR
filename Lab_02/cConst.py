import ctypes

intersect_in_c = ctypes.CDLL("cpsr.so")

intersect_in_c.intersect.argtypes = (ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.ARRAY(ctypes.c_float,2)))