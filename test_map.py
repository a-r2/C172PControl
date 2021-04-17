import cartopy.crs as ccrs
import cartopy.io.img_tiles as cimgt
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from mpl_toolkits.axes_grid1.inset_locator import InsetPosition
from settings import *

longitude = -4.853942
latitude = 37.834722
LONGITUDE_INIT = longitude
LATITUDE_INIT = latitude
LONGITUDE_WIDTH_INIT = 1
LATITUDE_WIDTH_INIT = 1
LONGITUDE_WIDTH = 0.05
LATITUDE_WIDTH = 0.05
TILE_EXTENT_INIT = (LONGITUDE_INIT-LONGITUDE_WIDTH_INIT, LONGITUDE_INIT+LONGITUDE_WIDTH_INIT, LATITUDE_INIT-LATITUDE_WIDTH_INIT, LATITUDE_INIT+LATITUDE_WIDTH_INIT)
TILE_EXTENT = (longitude-LONGITUDE_WIDTH, longitude+LONGITUDE_WIDTH, latitude-LATITUDE_WIDTH, latitude+LATITUDE_WIDTH)
x_pos = 0.45036
y_pos = 0.4460

plt.ion()
tile = cimgt.OSM()
fig = plt.figure('Navigation map', figsize=(6.4, 4.8))
ax = fig.add_subplot(1, 1, 1, projection=tile.crs, zorder=0)
fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
ax.axis('off')
ax.set_extent(TILE_EXTENT_INIT)
ax.add_image(tile, 12)
#plt.plot(longitude, latitude, marker='d', fillstyle='none', color='fuchsia', markersize=10, transform=ccrs.Geodetic())
icon = plt.imread('./Icons/uav_0.ico')
#newax = ax.inset_axes((0.5, 0.5, AC_ICON_SIZE[0], AC_ICON_SIZE[1]),)
#newax.imshow(icon)
#newax.axis('off')
inset_x = 0.5
inset_y = 0.5
inset_size = 0.1
ax2 = plt.axes((0, 0, 1, 1), projection=ccrs.Mercator(central_longitude=(TILE_EXTENT_INIT[0] + TILE_EXTENT_INIT[1]) / 2, min_latitude=TILE_EXTENT_INIT[2], max_latitude=TILE_EXTENT_INIT[3]), zorder=1)
ip = InsetPosition(ax, [inset_x - inset_size / 2,
                        inset_y - inset_size / 2,
                        inset_size,
                        inset_size])
ax2.set_axes_locator(ip)
ax2.imshow(icon)
ax2.axis('off')
def update_map(longitude, latitude):
    ax.set_extent(TILE_EXTENT)
    plt.draw()
def replot(fig):
    dummy_fig = plt.figure('Navigation map')
    new_manager = dummy_fig.canvas.manager
    new_manager.canvas.figure = fig
    fig.set_canvas(new_manager.canvas)
