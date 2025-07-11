#!/usr/bin/env python
"""
Balloon Tracking Software for MSGC Ground Station X

Author:	Larson Dean Brandstetter, CpE
Modified: Ronnel Walton and Mathew Clutter
Based on the Satellite Tracking Method from eutelsat and the University of Munich.

"""

import math
import numpy as np


class TrackingMath:

    EARTH_RADIUS_METERS = 6371000  # average radius of the earth in meters

    def __init__(self, ground_station_latitude, ground_station_longitude, ground_station_altitude, target_latitude, target_longitude, target_altitude):    
        # GPS information, gs is ground station, t is target
        # Inputs are in degrees, converted to radians for calc (Why do this conversion?)
        # Alt is in meters

        self.ground_station_longitude = math.radians(ground_station_longitude)
        self.ground_station_latitude = math.radians(ground_station_latitude)
        self.ground_station_altitude = ground_station_altitude
        self.target_longitude = math.radians(target_longitude)
        self.target_latitude = math.radians(target_latitude)
        self.target_altitude = target_altitude
        self.target_geocentric_radius = TrackingMath.EARTH_RADIUS_METERS + self.target_altitude # radius of the earth plus altitude of target
        self.ground_station_geocentric_radius = TrackingMath.EARTH_RADIUS_METERS + self.ground_station_altitude # radius of the earth plus gs altitude
        self.longitude_difference = self.ground_station_longitude - self.target_longitude

        self.distance = self.distance()

    def distance(self):
        # calculates distance from ground station to the balloon (in m)
        component_a = math.pow((self.target_geocentric_radius*math.cos(self.target_latitude)*math.cos(self.longitude_difference)-self.ground_station_geocentric_radius*math.cos(self.ground_station_latitude)),2)
        component_b = math.pow(self.target_geocentric_radius,2)*math.pow(math.cos(self.target_latitude),2)*math.pow(math.sin(self.longitude_difference),2)
        component_c = math.pow((self.target_geocentric_radius*math.sin(self.target_latitude)-self.ground_station_geocentric_radius*math.sin(self.ground_station_latitude)),2)
        self.distance = math.sqrt(component_a+component_b+component_c)
        return self.distance

    def elevation(self):
        # calculates the elevation from the ground station to the ballon (degrees)
        component_a = math.cos(self.ground_station_latitude)*self.target_geocentric_radius*math.cos(self.target_latitude)*math.cos(self.longitude_difference)
        component_b = self.target_geocentric_radius*math.sin(self.ground_station_latitude)*math.sin(self.target_latitude)
        self.elevation_angle = -math.asin(-((component_a+component_b-self.ground_station_geocentric_radius)/self.distance))

        self.elevation_angle = np.rad2deg(self.elevation_angle)

        if self.elevation_angle > 90:
            return 90
        elif self.elevation_angle < 0:
            return 0
        else:
            return self.elevation_angle

    def azimuth(self):
        # calculates the azimuth from the ground station to the balloon (compass bearing degrees)
        component_a = -(self.target_geocentric_radius*math.cos(self.target_latitude)*math.sin(self.longitude_difference))/(self.distance*math.cos(np.deg2rad(self.elevation_angle)))
        component_b = -(self.target_geocentric_radius*((math.sin(self.ground_station_latitude)*math.cos(self.target_latitude)*math.cos(self.longitude_difference))-
                         (math.cos(self.ground_station_latitude)*math.sin(self.target_latitude))))/(self.distance*math.cos(np.deg2rad(self.elevation_angle)))
        self.azimuth_angle = np.rad2deg(math.atan2(component_a, component_b))

        while self.azimuth_angle < 0:
            self.azimuth_angle += 360

        while self.azimuth_angle > 360:
            self.azimuth_angle -= 360

        return self.azimuth_angle
