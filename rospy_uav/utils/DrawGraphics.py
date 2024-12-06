import numpy as np
from mpl_toolkits.mplot3d import Axes3D


class DrawGraphics:
    pass


def set_axes_equal(ax: Axes3D) -> Axes3D:
    """
    Define os eixos de um gráfico 3D como iguais para manter as proporções.

    :param ax: Objeto Axes3D do Matplotlib.
    :return: Objeto Axes3D com os eixos definidos como iguais.
    """
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])

    max_range = max(x_range, y_range, z_range)

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)

    ax.set_xlim3d([x_middle - max_range / 2, x_middle + max_range / 2])
    ax.set_ylim3d([y_middle - max_range / 2, y_middle + max_range / 2])
    ax.set_zlim3d([z_middle - max_range / 2, z_middle + max_range / 2])

    return ax
