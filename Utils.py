import numpy as np

def checkLineCollision(line1: tuple, line2: tuple):
    p1, p2 = line1
    p3, p4 = line2
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
    return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

def euclideanDistance(point1: tuple, point2: tuple):
    return np.sqrt((point2[0] - point1[0])**2 + (point2[1]-point1[1])**2)