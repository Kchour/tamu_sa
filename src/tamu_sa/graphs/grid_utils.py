''' Grid Utilities '''
import numpy as np
#import png

# def png_to_ogm(filename, normalized=False, origin='lower'):
#     """
#     Convert a png image to occupancy data.
#     :param filename: the image filename
#     :param normalized: whether the data should be normalised, i.e. to be in value range [0, 1]
#     :param origin:
#     :return:
#     """
#     r = png.Reader(filename)
#     img = r.read()
#     img_data = list(img[2])

#     out_img = []
#     bitdepth = img[3]['bitdepth']

#     for i in range(len(img_data)):

#         out_img_row = []

#         for j in range(len(img_data[0])):
#             if j % img[3]['planes'] == 0:
#                 if normalized:
#                     out_img_row.append(img_data[i][j] * 1.0 / (2**bitdepth))
#                 else:
#                     out_img_row.append(img_data[i][j])

#         out_img.append(out_img_row)

#     if origin == 'lower':
#         out_img.reverse()

#     return out_img

def filter_points(path, points, grid_dim):
    # for each point in 'points', return only points within boundaries
    x = points[:,0]
    y = points[:,1]
    thresh = points[ (grid_dim[0] <= x)*(x <= grid_dim[1])* (grid_dim[2] <= y)*(y <= grid_dim[3])]
    return thresh

def init_grid(grid_dim, grid_size, init_val):
    # Add 1 to even out world coordinates
    # Add np ceil to ensure actual grid size is not bigger than desired
    # since for some values, the grid dim wont divide evenly by grid size
    xvals = int(np.ceil((grid_dim[1] - grid_dim[0] + 1) / grid_size))
    yvals = int(np.ceil((grid_dim[3] - grid_dim[2] + 1) / grid_size))
    if init_val != 0:
        return init_val * np.ones((yvals, xvals))
    else:
        return np.zeros((yvals, xvals))


def get_index(x, y, grid_size, grid_dim):
    # Convert from world coordinates to index
    #indx = int((x - grid_dim[0])/grid_size)
    #indy = int((y - grid_dim[2])/grid_size)
    # Make sure x,y fall within the grid physical boundaries
    if np.any((grid_dim[0] <= x) *
              (x <= grid_dim[1]) == 0) and np.any((grid_dim[2] <= y) *
                                                  (y <= grid_dim[3]) == 0):
        raise NameError('(x,y) world coordinates must be within boundaries')
    indx = np.round((x - grid_dim[0]) / grid_size).astype(int)
    indy = np.round((y - grid_dim[2]) / grid_size).astype(int)
    return (indx, indy)


# A list of all the points in our grid
def mesh_grid_list(grid_dim, grid_size):
    # Make a mesh grid with the specified grid size/dim
    xvals = np.arange(grid_dim[0], grid_dim[1], grid_size)
    yvals = np.arange(grid_dim[2], grid_dim[3], grid_size)
    x, y = np.meshgrid(xvals, yvals)
    x, y = x.flatten(), y.flatten()
    points = np.vstack((x,y)).T
    return points

def get_world(indx, indy, grid_size, grid_dim):
    # Convert from index to world coordinates
    x = (indx) * grid_size + grid_dim[0]
    y = (indy) * grid_size + grid_dim[2]
    return (x, y)

''' courtesty of https://stackoverflow.com/questions/42463172/how-to-perform-max-mean-pooling-on-a-2d-array-using-numpy'''

def pooling(mat,ksize,method='max',pad=False):
    '''Non-overlapping pooling on 2D or 3D data.

    <mat>: ndarray, input array to pool.
    <ksize>: tuple of 2, kernel size in (ky, kx).
    <method>: str, 'max for max-pooling, 
                   'mean' for mean-pooling.
    <pad>: bool, pad <mat> or not. If no pad, output has size
           n//f, n being <mat> size, f being kernel size.
           if pad, output has size ceil(n/f).

    Return <result>: pooled matrix.
    '''

    m, n = mat.shape[:2]
    ky,kx=ksize

    _ceil=lambda x,y: int(np.ceil(x/float(y)))

    if pad:
        ny=_ceil(m,ky)
        nx=_ceil(n,kx)
        size=(ny*ky, nx*kx)+mat.shape[2:]
        mat_pad=np.full(size,np.nan)
        mat_pad[:m,:n,...]=mat
    else:
        ny=m//ky
        nx=n//kx
        mat_pad=mat[:ny*ky, :nx*kx, ...]

    new_shape=(ny,ky,nx,kx)+mat.shape[2:]

    if method=='max':
        result=np.nanmax(mat_pad.reshape(new_shape),axis=(1,3))
    else:
        result=np.nanmean(mat_pad.reshape(new_shape),axis=(1,3))

    return result