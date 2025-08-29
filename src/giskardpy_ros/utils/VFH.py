import math
import os
import time
from itertools import groupby
from operator import itemgetter

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
import rospy
import timeit
import tf

from giskardpy.utils.math import angle_between_vector

if 'GITHUB_WORKFLOW' not in os.environ:
    matplotlib.use('Qt5Agg')


class VectorFieldHistogram:
    def __init__(self,
                 max_range: float,
                 grid_size: float,
                 sector_angle: int,
                 obstacle_threshold: float,
                 s_max: int,
                 input_topic: str):

        """
        The Vector Field Histogram is a real-time collision avoidance algorithm,
        which is specifically designed to handle mid-maneuver path obstructions and
        the problem of autonomous maneuvering of narrow paths. This is done by creating
        a polar histogram which essentially cuts the HSRs LiDar scanner into 48 equally
        sized sectors and calculates the so-called Polar Obstacle Density(POD) for each of those sectors.
        If the POD overshoots a certain threshold, the sector in which the threshold has been found is marked as
        obstructed, whereas sectors in which the threshold hasn't been overshot are marked as free.

        :param max_range: maximal that the VFH should consider when doing its calculations
        :param grid_size: size of the histogram grid
        :param sector_angle: size of the sectors in degrees that the polar histogram is being constructed with
        :param obstacle_threshold: threshold to differentiate between obstructed and free sectors
        :param s_max: variable to decide whether a valley is wide or narrow, which changes the direction vector calculation
        :param input_topic: the topic the laser data is taken from
        """
        self.max_range = max_range
        self.grid_size = grid_size
        self.sector_angle = sector_angle
        self.obstacle_threshold = obstacle_threshold
        self.s_max = s_max
        # self.human_point = human_point
        self.direction_vector = None

        self.topic = input_topic  # "/hsrb/base_scan"
        self.robot_position = (0.0, 0.0)
        self.tf_listener = tf.TransformListener()
        self.distances = np.array([])
        self.angles = np.array([])
        self.polar_histogram = None
        self.theta_deg = None
        self.d_max = self.max_range
        self.obstacle_density = None

        self.sub = rospy.Subscriber(self.topic, LaserScan, self.laser_callback)
        self.rate = rospy.Rate(10)

        # used to limit interval of plot generation
        self.plotting = 0
        self.frequency = 50

    # ideas: check whether human_point is behind obstacle; use that as part of condition to start search algo
    # - make sure there are always more than one valleys in search stage; check which valley is closer to the direction of the human point(??)
    # TODO: Unit testing by checking steering angles

    def laser_callback(self, data: LaserScan):
        self.distances = np.array(data.ranges)
        self.angles = np.arange(len(self.distances)) * data.angle_increment

    def target_sim(self,
                   target_point=(1.0, 0.0, 0.0)):
        # start_time = time.perf_counter() - benchmark stuff
        # calculating sector that target is in
        target_point[2] = 0
        target_angle = angle_between_vector(v1=np.array([1, 0, 0]), v2=target_point) + 2.0944
        if target_point[1] > 0:
            target_angle = -target_angle

        # print('target_angle:', target_angle)
        target_angle_deg = np.rad2deg(target_angle) % 240
        # print('target_angle_deg:', target_angle_deg)
        target_sector = int(target_angle_deg // self.sector_angle)
        print(f"Target Sector:{target_sector}")
        # calculate sector that human_point is in and pass it to update to use as req for search algo

        # end_time = time.perf_counter()  # End timing
        # elapsed_time_ms = (end_time - start_time) * 1000
        # rospy.loginfo(f"target_sim took {elapsed_time_ms:.3f} ms") - benchmark stuff
        return target_sector, target_angle_deg, target_point

    def run(self,
            target_point):
        if len(self.distances) == 0:
            return
        # print(f'target_point: {target_point}')
        target_sector, target_angle_deg, target_point = self.target_sim(target_point=target_point)
        self.update_histogram(self.distances, self.angles, target_sector, target_angle_deg)
        if self.plotting >= self.frequency:
            self.histogram_plot(target_point)
            self.plotting = 0
        else:
            self.plotting += 1

    def update_histogram(self, distances, angles, target_sector, target_angle_deg):
        """
        The update_histogram method constitutes the core of the Vector Field Histogram.
        The method first constructs the histogram by cutting the scanners range up into
        48 5-degree sectors, after which the magnitudes of each sector are calculated.
        If the calculated magnitude is under a certain threshold, the sectors in which the threshold
        has been undershot are marked as free, whereas sector where the threshold has been overshot are
        marked as obstructed. Consecutive free sector are merged into lists, so-called "valleys";
        at which point the algorithm checks whether the direction of the target is within one of these valleys.
        Should the sector of the target direction coincide with a sector that's part of a valley,
        then the robot will be pushed into the direction of a target. Should the target be obstructed
        by an obstacle, the algorithm will search for the nearest available valley of a certain size to
        autonomously avoid said obstacle.

        :param distances: distances from the target sector
        :param angles: angles from the respective laser scanner reading
        :param target_sector: the sector of the histogram the target is in (from point of the robot)
        :param target_angle_deg: target angle in degrees
        """
        # start_time = time.perf_counter()
        theta_deg = None
        x_points = distances * np.cos(angles)
        y_points = distances * np.sin(angles)

        grid_dim = int((2 * self.max_range) / self.grid_size)
        histogram_grid = np.zeros((grid_dim, grid_dim))
        # map converted laser scanner ranges onto histogram grid
        for x, y in zip(x_points, y_points):
            gx = int((x + self.max_range) / self.grid_size)
            gy = int((y + self.max_range) / self.grid_size)
            if 0 <= gx < grid_dim and 0 <= gy < grid_dim:
                histogram_grid[gx, gy] = 1

        num_sectors = int(240 / self.sector_angle)
        polar_histogram = np.zeros(num_sectors)
        angles = angles - angles[0]
        # calculate sector magnitudes
        for sector in range(num_sectors):  # TODO: refactor to be faster
            start_angle = np.deg2rad(sector * self.sector_angle)
            end_angle = np.deg2rad((sector + 1) * self.sector_angle)
            mask = (angles >= start_angle) & (angles <= end_angle)
            sector_dists = distances[mask]
            # density = np.sum((self.max_range - sector_dists) / self.max_range)
            clipped_dists = np.clip(sector_dists, 0, self.d_max)
            magnitudes = 1 - (clipped_dists / self.d_max)
            polar_histogram[sector] = np.sum(magnitudes)
        polar_histogram = polar_histogram[::-1]  # flips histogram, because we have a right hand coord system
        # print(polar_histogram)
        # TODO: Look into smoothing the polar histogram
        # polar_histogram = self.smooth_polar_histogram(polar_histogram, l=5)
        # print(f'Magnitude Sum for Sector {sector}:{np.sum(magnitudes)}')
        free_sectors = np.where(polar_histogram < self.obstacle_threshold)[0]
        print(free_sectors)
        # valley calculation
        valleys = []
        for k, g in groupby(enumerate(free_sectors), lambda ix: ix[0] - ix[1]):
            valley = list(map(itemgetter(1), g))
            valleys.append(valley)
            print(f"Valley: {valley}")
            print(f"Valleys: {valleys}")
        # Find the valley containing the target sector
        selected_valley = None

        for valley in valleys:
            # check condition and how the valley is picked
            if min(valley) <= target_sector <= max(valley) and len(valley) >= 4:
                selected_valley = valley
                print(f"Selected Valley: {selected_valley}")
                break

        if selected_valley:##
            k_near = min(selected_valley)
            k_far = k_near + self.s_max
            prob = k_near + self.s_max
            if len(selected_valley) <= self.s_max:
                # print("Valley is narrow")
                k_far = max(selected_valley)
            if len(selected_valley) >= prob:
                k_far = max(selected_valley)
            theta = (k_near + k_far) / 2
            if len(selected_valley) >= prob and target_sector in selected_valley:
                theta = target_sector
            best_sector = int(theta)
            theta_deg = best_sector * self.sector_angle
            print(f"Target angle: {target_angle_deg:.2f}° => Sector {target_sector}")
            # print(f"Steering Valley: {selected_valley}")
            print(f"k_near: {k_near}, k_far: {k_far}, Steering direction sector: {theta}")
            print(f"Steering angle: {theta_deg:.2f}°")
        # Start searching for adjacent free valley
        else:
            total_sectors = len(polar_histogram)
            # TODO: look into which cases wrongfully trigger the search algo
            print("Target point is behind obstacle, choosing largest adjacent Valley")
            # check whether target sector is adjacent to any valleys, then check which of the two is larger
            nearby_sectors = set((target_sector + offset) % total_sectors for offset in range(-2, 2))
            # print(f"Nearby sectors: {nearby_sectors}")
            candidate_valleys = []
            for valley in valleys:
                if not set(valley).isdisjoint(nearby_sectors) and len(valley) >= 4:
                    candidate_valleys.append(valley)
            # candidate valley has been found in first iteration
            # check in what sector human_point is and pick valley dependent on which has the human_sector
            if candidate_valleys and len(candidate_valleys) >= 2:
                selected_valley = max(candidate_valleys, key=len)
                print(f"Selected adjacent Valley: {selected_valley}")
                k_near = min(selected_valley)
                k_far = k_near + self.s_max
                if len(selected_valley) <= self.s_max:
                    # print("Valley is narrow")
                    k_far = max(selected_valley)
                theta = (k_near + k_far) / 2
                best_sector = int(theta)
                theta_deg = best_sector * self.sector_angle
                # print(f"Target angle: {target_angle_deg:.2f}° => Sector {target_sector}")
                # print(f"Steering Valley: {selected_valley}")
                # print(f"k_near: {k_near}, k_far: {k_far}, Steering direction sector: {theta}")
                # print(f"Steering angle: {theta_deg:.2f}°")
            # Dynamically expand search radius if no candidate is found in first iteration
            else:
                # print("No immediately adjacent Valleys found, expanding search radius")
                found_valley = None
                max_search_expansion = total_sectors // 2
                for radius in range(2, max_search_expansion + 1):
                    nearby_sectors = set()
                    for offset in range(-radius, radius + 1):
                        candidate_sector = target_sector + offset
                        if 0 <= candidate_sector < total_sectors:
                            nearby_sectors.add(candidate_sector)
                    # print(f"Trying nearby sectors with radius {radius}: {sorted(nearby_sectors)}")

                    # check whether target sector is adjacent to any valleys, then check which of the two is larger
                    candidate_valleys = []
                    for valley in valleys:
                        if not set(valley).isdisjoint(nearby_sectors) and len(valley) >= 4:
                            candidate_valleys.append(valley)
                            # TODO: replace choosing bigger valley with minimum valley size
                    if candidate_valleys:
                        found_valley = max(candidate_valleys, key=len)
                        # print(f"Found valley at radius {radius}: {found_valley}")
                        break
                # calculate directional vector
                if found_valley:
                    selected_valley = found_valley
                    k_near = min(selected_valley)
                    k_far = k_near + self.s_max
                    if len(selected_valley) <= self.s_max:
                        # print("Valley is narrow")
                        k_far = max(selected_valley)
                    theta = (k_near + k_far) / 2
                    best_sector = int(theta)
                    theta_deg = best_sector * self.sector_angle
                    # print(f"Target angle: {target_angle_deg:.2f}° => Sector {target_sector}")
                    # print(f"Steering Valley: {selected_valley}")
                    # print(f"k_near: {k_near}, k_far: {k_far}, Steering direction sector: {theta}")
                    # print(f"Steering angle: {theta_deg:.2f}°")

        self.polar_histogram = polar_histogram
        self.theta_deg = theta_deg
        self.x_points = x_points
        self.y_points = y_points

        # calculation of directional vector
        if theta_deg is not None:
            theta_rad = np.radians(120.0 - theta_deg)  # -- maybe np. instead of .math?
            direction_vector = np.array([np.cos(theta_rad), np.sin(theta_rad), 0])
            print(theta_rad)
            print(f"Directional Vector: {direction_vector}")
            print("---------------------------------------")
            self.direction_vector = direction_vector
            return direction_vector
        else:
            direction_vector = np.array([0.0, 0.0, 0.0])
            print(f"Directional Vector: {direction_vector}")
            print("---------------------------------------")
            self.direction_vector = direction_vector
            return direction_vector
        # benchmark stuff
        # end_time = time.perf_counter()  # End timing
        # elapsed_time_ms = (end_time - start_time) * 1000
        # rospy.loginfo(f"update_histogram took {elapsed_time_ms:.3f} ms")
        # log_file_path = "/home/yannis/Documents/histogram_benchmark.txt"
        # with open(log_file_path, "a") as f:
        #     f.write(f"update_histogram took {elapsed_time_ms:.4f}ms\n")

    # def smooth_polar_histogram(self, polar_histogram, l=5):
    #
    #     smoothed = np.zeros_like(polar_histogram)
    #     length = len(polar_histogram)
    #     print(f"[DEBUG] Smoothing histogram with {length} sectors")
    #     for k in range(length):
    #         value = 0
    #         weight_sum = 0
    #         for offset in range(-l, l+1):
    #             idx = k + offset % length # wrap around
    #             weight = 2 if offset != -l and offset !=l else 1
    #
    #             value += weight * polar_histogram[idx]
    #             weight_sum += weight
    #             smoothed[k] = value / weight_sum
    #             print(f'V:{value}')
    #             print(f'WS:{weight_sum}')
    #             print(f'IDX:{idx}')
    #
    #     return smoothed
    # ------------------- ONLY USE THE PLOT FOR TESTING ---------------------
    def histogram_plot(self, target_point):
        # Start of plot gen
        fig = plt.figure(figsize=(12, 6))
        ax0 = fig.add_subplot(1, 2, 1)
        ax1 = fig.add_subplot(1, 2, 2, polar=True)

        # LIDAR PointCloud
        ax0.set_title('LIDAR Map')
        ax0.scatter(-self.y_points, self.x_points, c='blue', s=1, label='LIDAR points')
        ax0.plot(-target_point[1], target_point[0], 'mx', markersize=10)
        ax0.plot(-self.robot_position[1], self.robot_position[0], 'go', markersize=10, label='Robot Position')
        ax0.set_xlim(-self.max_range, self.max_range)
        ax0.set_ylim(-self.max_range, self.max_range)
        ax0.set_aspect('equal')
        ax0.grid(True)
        ax0.legend()

        # Polar Histogram
        ax1.set_title('Polar Histogram')
        ax1.set_theta_direction(-1)
        ax1.set_theta_zero_location('W', 30)
        colors = ['green' if self.polar_histogram[i] < self.obstacle_threshold else 'red'
                  for i in range(len(self.polar_histogram))]
        angles_rad = np.deg2rad(np.arange(0, 240, self.sector_angle))
        ax1.bar(angles_rad, self.polar_histogram, width=np.deg2rad(self.sector_angle), bottom=0.0, align='edge',
                color=colors)
        ax1.plot('m--', linewidth=0.5, label='sectors')  # needed to portray legend for graph accurately
        for angles in angles_rad:
            ax1.plot([angles, angles], [0, np.max(self.polar_histogram)], 'm--', linewidth=0.5)
        if self.theta_deg is not None:
            best_rad = np.deg2rad(self.theta_deg)
            ax1.plot([best_rad, best_rad], [0, np.max(self.polar_histogram)], 'b--', linewidth=2,
                     label='best direction')
        ax1.legend(loc='upper right')
        plt.tight_layout()
        plt.savefig('/home/suturo/Documents/vfh.png')
