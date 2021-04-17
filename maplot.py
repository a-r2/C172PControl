from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from mpl_toolkits.axes_grid1.inset_locator import InsetPosition
import cartopy.crs as ccrs
import cartopy.io.img_tiles as cimgt
import matplotlib.pyplot as plt
import time

from settings import *

TILE_EXTENT_INIT = (LONG_INIT-LONG_DELTA_INIT, LONG_INIT+LONG_DELTA_INIT, LAT_INIT-LAT_DELTA_INIT, LAT_INIT+LAT_DELTA_INIT)

plt.ion()

class NavMap():
    def __init__(self):
        self.fig = plt.figure(NAVMAP_TITLE, figsize=NAV_FIG_SIZE)
        self.fig.canvas.manager.window.wm_geometry("+%d+%d" % (1000, 0))
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        self.tile = cimgt.OSM()
        self.ax_map = self.fig.add_subplot(1, 1, 1, projection=self.tile.crs, zorder=0)
        self.ax_map.axis('off')
        self.ax_map.set_extent(TILE_EXTENT_INIT)
        self.ax_map.add_image(self.tile, 12)
        self.plot_mission()
        self.ax_aircraft = plt.axes((0, 0, 1, 1), projection=ccrs.Mercator(central_longitude=(TILE_EXTENT_INIT[0] + TILE_EXTENT_INIT[1]) / 2, min_latitude=TILE_EXTENT_INIT[2], max_latitude=TILE_EXTENT_INIT[3]), zorder=2)
        ax_aircraft_pos = InsetPosition(self.ax_map, (0.5 - AC_ICON_SIZE / 2, 0.5 - AC_ICON_SIZE / 2, AC_ICON_SIZE, AC_ICON_SIZE))
        self.ax_aircraft.set_axes_locator(ax_aircraft_pos)
        self.ax_aircraft.axis('off')
        self.read_icons()
        self.plot_icon(HEAD_INIT)
    def update_map(self, rxqueue):
        t1 = time.time()
        rx_telemetry = rxqueue.get()
        longitude = rx_telemetry[5]
        latitude = rx_telemetry[6]
        yaw_angle = rx_telemetry[11]
        print('longitude: ' + str(longitude))
        print('latitude: ' + str(latitude))
        print('yaw: ' + str(yaw_angle))
        TILE_EXTENT = (longitude-LONG_DELTA_UPDT, longitude+LONG_DELTA_UPDT, latitude-LAT_DELTA_UPDT, latitude+LAT_DELTA_UPDT)
        self.ax_map.set_extent(TILE_EXTENT)
        self.plot_icon(yaw_angle)
        plt.draw()
        t2 = time.time()
        time.sleep(max(NAVMAP_PERIOD - (t2 - t1), 0))
    def replot(self):
        dummy_fig = plt.figure(NAVMAP_TITLE, figsize=FIG_SIZE)
        new_manager = dummy_fig.canvas.manager
        new_manager.canvas.figure = self.fig
        self.fig.set_canvas(new_manager.canvas)
        self.fig.canvas.manager.window.wm_geometry("+%d+%d" % (1000, 0))
    def plot_icon(self, heading):
        heading_int = int(round(heading) - (round(heading) // 360) * 360)
        self.ax_aircraft.imshow(self.ac_icons[heading_int])
    def plot_mission(self):
        for (longitude, latitude)  in WP_TUPLE:
            plt.plot(longitude, latitude, marker='d', fillstyle='none', color='fuchsia', markersize=WP_ICON_SIZE, transform=ccrs.Geodetic(), zorder=1)
    def read_icons(self):
        self.ac_icons = list()
        for i in range(360):
            self.ac_icons.append(plt.imread('./Icons/ac_' + str(i) + '.ico'))
    def __del__(self):
        pass
