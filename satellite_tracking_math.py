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
    """Satellite and balloon tracking mathematics calculator.

    Implements geodetic calculations for determining antenna pointing angles
    from a ground station to a target (satellite/balloon) using spherical
    geometry and Earth's coordinate system.

    Based on the Satellite Tracking Method from Eutelsat and the University of Munich.
    Provides distance, azimuth, and elevation calculations for precise antenna pointing.

    Attributes:
        EARTH_RADIUS_METERS: Average Earth radius in meters (6,371,000m)
    """

    EARTH_RADIUS_METERS = 6371000  # average radius of the earth in meters

    def __init__(
        self,
        ground_station_latitude: float,
        ground_station_longitude: float,
        ground_station_altitude: float,
        target_latitude: float,
        target_longitude: float,
        target_altitude: float,
    ) -> None:
        """Initialize tracking calculation parameters.

        Converts geographic coordinates to radians and calculates geocentric radii
        for both ground station and target positions. Establishes baseline parameters
        for subsequent tracking calculations.

        Args:
            ground_station_latitude: Ground station latitude in decimal degrees
            ground_station_longitude: Ground station longitude in decimal degrees
            ground_station_altitude: Ground station altitude above sea level in meters
            target_latitude: Target latitude in decimal degrees
            target_longitude: Target longitude in decimal degrees
            target_altitude: Target altitude above sea level in meters

        Note:
            SAFETY CRITICAL: Coordinate accuracy directly affects antenna pointing.
            Invalid coordinates can result in tracking failure or equipment damage
            from incorrect antenna positioning commands.
        """
        # GPS information, gs is ground station, t is target
        # Inputs are in degrees, converted to radians for calc (Why do this conversion?)
        # Alt is in meters

        self.ground_station_longitude = math.radians(ground_station_longitude)
        self.ground_station_latitude = math.radians(ground_station_latitude)
        self.ground_station_altitude = ground_station_altitude
        self.target_longitude = math.radians(target_longitude)
        self.target_latitude = math.radians(target_latitude)
        self.target_altitude = target_altitude
        self.target_geocentric_radius = (
            TrackingMath.EARTH_RADIUS_METERS + self.target_altitude
        )  # radius of the earth plus altitude of target
        self.ground_station_geocentric_radius = (
            TrackingMath.EARTH_RADIUS_METERS + self.ground_station_altitude
        )  # radius of the earth plus gs altitude
        self.longitude_difference = (
            self.ground_station_longitude - self.target_longitude
        )

        self.distance = self.distance()

    def distance(self) -> float:
        """Calculate great circle distance from ground station to target.

        Computes 3D distance using spherical geometry, accounting for Earth's
        curvature and altitude differences between ground station and target.

        Returns:
            float: Distance from ground station to target in meters
        """
        # calculates distance from ground station to the balloon (in m)
        component_a = math.pow(
            (
                self.target_geocentric_radius
                * math.cos(self.target_latitude)
                * math.cos(self.longitude_difference)
                - self.ground_station_geocentric_radius
                * math.cos(self.ground_station_latitude)
            ),
            2,
        )
        component_b = (
            math.pow(self.target_geocentric_radius, 2)
            * math.pow(math.cos(self.target_latitude), 2)
            * math.pow(math.sin(self.longitude_difference), 2)
        )
        component_c = math.pow(
            (
                self.target_geocentric_radius * math.sin(self.target_latitude)
                - self.ground_station_geocentric_radius
                * math.sin(self.ground_station_latitude)
            ),
            2,
        )
        distance = math.sqrt(component_a + component_b + component_c)
        return distance

    def elevation(self) -> float:
        """Calculate elevation angle from ground station to target.

        Determines the vertical pointing angle required for antenna to track target.
        Elevation is measured from horizontal plane (0°) to zenith (90°).

        Returns:
            float: Elevation angle in degrees (0-90)

        Note:
            SAFETY CRITICAL: Controls antenna elevation movement. Values outside
            0-90° range are clamped to prevent mechanical damage. Verify antenna
            physical limits before operation.
        """
        # calculates the elevation from the ground station to the ballon (degrees)
        component_a = (
            math.cos(self.ground_station_latitude)
            * self.target_geocentric_radius
            * math.cos(self.target_latitude)
            * math.cos(self.longitude_difference)
        )
        component_b = (
            self.target_geocentric_radius
            * math.sin(self.ground_station_latitude)
            * math.sin(self.target_latitude)
        )
        self.elevation_angle = -math.asin(
            -(
                (component_a + component_b - self.ground_station_geocentric_radius)
                / self.distance
            )
        )

        self.elevation_angle = np.rad2deg(self.elevation_angle)

        if self.elevation_angle > 90:
            return 90
        elif self.elevation_angle < 0:
            return 0
        else:
            return self.elevation_angle

    def azimuth(self) -> float:
        """Calculate azimuth angle from ground station to target.

        Determines the horizontal pointing angle required for antenna to track target.
        Azimuth is measured clockwise from true north (0°-360°).

        Returns:
            float: Azimuth angle in degrees (0-360)

        Note:
            SAFETY CRITICAL: Controls antenna azimuth rotation. Monitor for cable
            wrap and mechanical limits. Continuous rotation may damage RF connections
            or exceed physical rotation constraints.
        """
        # calculates the azimuth from the ground station to the balloon (compass bearing degrees)
        component_a = -(
            self.target_geocentric_radius
            * math.cos(self.target_latitude)
            * math.sin(self.longitude_difference)
        ) / (self.distance * math.cos(np.deg2rad(self.elevation_angle)))
        component_b = -(
            self.target_geocentric_radius
            * (
                (
                    math.sin(self.ground_station_latitude)
                    * math.cos(self.target_latitude)
                    * math.cos(self.longitude_difference)
                )
                - (
                    math.cos(self.ground_station_latitude)
                    * math.sin(self.target_latitude)
                )
            )
        ) / (self.distance * math.cos(np.deg2rad(self.elevation_angle)))
        self.azimuth_angle = np.rad2deg(math.atan2(component_a, component_b))

        while self.azimuth_angle < 0:
            self.azimuth_angle += 360

        while self.azimuth_angle > 360:
            self.azimuth_angle -= 360

        return self.azimuth_angle
