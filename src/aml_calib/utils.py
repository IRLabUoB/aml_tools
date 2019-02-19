
__author__ = 'Ermano Arruda'
__copyright__ = 'Copyright 2018, Ermano Arruda, University of Birmingham'
__license__ = 'MIT'
__version__ = '0.7.0'
__maintainer__ = 'Ermano Arruda'
__email__ = 'ermano.arruda@gmail.com'
__status__ = 'Development'

import numpy as np

def compute_log(q):
    v = q[1:4]
    norm_v = np.linalg.norm(v)
    if norm_v == 0:
        return np.zeros(3)
    else:
        return np.arccos(q[0])*v/norm_v

def compute_exp(w):
    norm_w = np.linalg.norm(w)
    if norm_w == 0:
        return np.zeros(4)
    else:
        return np.hstack([np.cos(norm_w), np.sin(norm_w)*w/norm_w])

def quat2rot(q):
    p = np.dot(q.T, q)

    if (p > 1):
        print 'Warning: quat2rot: quaternion greater than 1'

    w = np.sqrt(1 - p)  # % w = np.cos(theta / 2)

    R = np.eye(4)

    R[:3, :3] = 2 * np.dot(q, q.T) + 2 * w * skew(q) + np.eye(3) - 2 * diag([p, p, p])

    return R

def rot2quat(R):

    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = R[:3,:3].flat
    # Fill only lower half of symmetric matrix
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0              ],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
        ) / 3.0
    # Use Hermitian eigenvectors, values for speed
    vals, vecs = np.linalg.eigh(K)
    # Select largest eigenvector, reorder to w,x,y,z quaternion
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    # Prefer quaternion with positive w
    # (q * -1 corresponds to same rotation as q)
    if q[0] < 0:
        q *= -1
    return q[1:4].reshape(3,1)

def rot2quat3(R):
    tc = np.trace(R[:3, :3])
    w4 = 2*np.sqrt( 1 + trace(R[:3,:3]) )
    q = np.array([( R[3,2] - R[2,3] ) / w4,
                  (R[1,3] - R[3,1] ) / w4,
                  ( R[2,1] - R[1,2] ) / w4])

    return q

def skew(v):
    """
    
    :param V: 
    :return: skew symmetric matrix from 3d vector v
    """
    S = np.array([[0, -v[2], v[1]],
                  [v[2], 0, -v[0]],
                  [-v[1], v[0], 0]])

    return S

def diag(v):
    """
    
    :param v: 
    :return: diagonal elements of matrix v
    """
    d = np.eye(len(v))

    for i in range(len(v)):
        d[i][i] = v[i]

    return d

def transl(t):
    """
    
    :param t: 
    :return: 4x4 homogeneous transform representing translation
    """
    translation = np.eye(4)

    translation[:3, 3] = t.reshape(3)

    return translation

def posediff(A,B):

    t1 = A[:,3]
    t2 = B[:,3]

    dT = t2 - t1

    q1 = rot2quat(A)
    q2 = rot2quat(B)

    rotAngle1 = 2 * np.math.asin(np.linalg.norm(q1))		#% rotation angle around the axis
    q1 = q1/np.linalg.norm(q1)

    rotAngle2 = 2 * np.math.asin(np.linalg.norm(q2))
    q2 = q2/np.linalg.norm(q1)

    dAxisAngle = np.math.asin(np.linalg.norm(np.cross(q1.T,q2.T)))
    dRotAngle = rotAngle2 - rotAngle1

    return dT, dAxisAngle, dRotAngle

def posediff2(pa,qa,pb,qb):

    dT = pb - pa

    q1 = qa
    q2 = qb

    rotAngle1 = 2 * np.math.asin(np.linalg.norm(q1[:3]))		#% rotation angle around the axis
    q1 = q1/np.linalg.norm(q1)

    rotAngle2 = 2 * np.math.asin(np.linalg.norm(q2[:3]))
    q2 = q2/np.linalg.norm(q1)

    dAxisAngle = np.math.asin(np.linalg.norm(np.cross(q1[:3].T,q2[:3].T)))
    dRotAngle = rotAngle2 - rotAngle1

    return dT, dAxisAngle, dRotAngle