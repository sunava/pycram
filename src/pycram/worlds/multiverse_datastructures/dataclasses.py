from dataclasses import dataclass

from typing_extensions import List


@dataclass
class MultiverseMetaData:
    """Meta data for the Multiverse Client, the simulation_name should be non-empty and unique for each simulation"""
    world_name: str = "world"
    simulation_name: str = "cram"
    length_unit: str = "m"
    angle_unit: str = "rad"
    mass_unit: str = "kg"
    time_unit: str = "s"
    handedness: str = "rhs"


@dataclass
class RayResult:
    """
    A dataclass to store the ray result. The ray result contains the body name that the ray intersects with and the
    distance from the ray origin to the intersection point.
    """
    body_name: str
    distance: float

    def intersected(self) -> bool:
        """
        Check if the ray intersects with a body.
        return: Whether the ray intersects with a body.
        """
        return self.distance >= 0 and self.body_name != ""


@dataclass
class MultiverseContactPoint:
    """
    A dataclass to store the contact point returned from Multiverse.
    """
    body_name: str
    contact_force: List[float]
    contact_torque: List[float]
