import numpy as np
import matplotlib.pyplot as plt
import math



def gen_plane_data(A, B, C, D, add_noise=False):
    # A, B, C, D = 1,2,3,1
    N = 50
    x_ = np.linspace(-2, 6, N)
    y_ = np.linspace(-2, 6, N)
    x, y = np.meshgrid(x_, y_)
    x = x.reshape(-1)
    y = y.reshape(-1)
    z = -(A*x + B*y +D)/C
    if add_noise:
        n = np.random.randn(x.shape[0],)*0.1
        x += n
        y += n
        z += n
    # z[20:23] += 1
    return x, y, z


def solve1(x, y, z):
    N = z.shape[0]
    A = np.zeros((N, 4))
    A[:, 0] = x
    A[:, 1] = y
    A[:, 2] = z
    A[:, 3] = 1
    # print(A)

    '''
    for i in range(N):
        A[i, :] = [x[i], y[i], z[i], 1]
    '''
    u, s, vt = np.linalg.svd(A, full_matrices=True)
    # print(vt[:, -1])
    # print(vt[-1, :])

    x_ = vt[-1, :]
    print('Solution: ', x_, np.linalg.norm(x_))
    print('Solution: ', x_*-4)
    print('b: ', A @ x_)

def solve2(x, y, z):
    xm = x.mean()
    ym = y.mean()
    zm = z.mean()
    xd  = x - xm
    yd  = y - ym
    zd  = z - zm

    N = z.shape[0]
    A = np.zeros((N, 3))
    A[:, 0] = xd
    A[:, 1] = yd
    A[:, 2] = zd
    # print(A)

    '''
    for i in range(N):
        A[i, :] = [x[i], y[i], z[i], 1]
    '''
    u, s, vt = np.linalg.svd(A, full_matrices=True)    
    print('Sigma: ', s)
    x_ = vt[-1, :]
    D_ = -(x_[0]*xm + x_[1]*ym + x_[2]*zm)
    print('Solution: ', x_, D_)

    # xo = np.zeros((3, 1))
    # xo[:, 0] = x_
    print('b: ', A @ x_)

    xo = np.zeros((4,))
    xo[:3] = x_
    xo[3] = D_
    return xo


def get_distance(a,b,c,d, x, y, z):
    d = abs(a*x + b*y + c*z + d)/math.sqrt(a**2 + b**2 + c**2)
    print(f'avgd: {d.mean():.3f}   dmax: {d.max():.3f}')

def get_distance2(plane_coeffs, x, y, z):
    a,b,c,d = plane_coeffs
    get_distance(a,b,c,d, x, y, z)


def plot_plane(x, y, z):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(x, y, z)
    ax.set_zlabel('z')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis('equal')
    plt.show()


# X = np.arange(-5, 5, 0.25)
# Y = np.arange(-5, 5, 0.25)
# X, Y = np.meshgrid(X, Y)
# R = np.sqrt(X**2 + Y**2)
# Z = np.sin(R)
# print(X.shape)

A, B, C, D = 1,2,3,1
A, B, C, D = 0,0,1,-2
x, y, z = gen_plane_data(A, B, C, D, add_noise=True)
plot_plane(x, y, z)

# solve1(x, y, z)
v_ = solve2(x, y, z)
get_distance2(v_, x, y, z)