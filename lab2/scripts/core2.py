from core import VecToso3, MatrixExp3
import numpy as np

rvec = np.array([[0.02551187], 
                 [-0.56615859], 
                 [0.00412808]])
tvec= np.array([[-108.5218783], 
                 [-97.22284861], 
                 [1515.76607254]])
def Rodrigues(rvec):
    rvec1=rvec.flatten()
    so3mat = VecToso3(rvec1)
    return MatrixExp3(so3mat)

print("RVec", rvec, "\n\n TVec", tvec, "\n\n Rotation Matrix", Rodrigues(rvec), "\n\n")