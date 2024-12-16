import em
import math
import os
import yaml
from argparse import ArgumentParser


class ConfigGenerator:
    def __init__(self, nominal_config_path: str, output_config_path: str):
        self.nominal_config_path = nominal_config_path
        self.output_config_path = output_config_path

        file_dir = os.path.dirname(os.path.realpath(__file__))
        self.hpp_template_path = f"{file_dir}/templates/robot_config.hpp.em"

        with open(nominal_config_path, "r") as file:
            self.nominal_config = yaml.safe_load(file)

        self.is_hpp = output_config_path.endswith(".hpp")
        self.is_yaml = output_config_path.endswith(".yaml")

        if not self.is_hpp and not self.is_yaml:
            raise ValueError(
                f"Output file ({output_config_path}) must be either .hpp or .yaml"
            )

        self.full_config = {}

    def parse(self):
        """Generate a configuration based on the nominal configuration."""
        result = {}

        result.update(self._get_nominal_values())
        result.update(self._get_tofs_config())
        result.update(self._get_wheel_base())
        result.update(self._get_starting_pose())

        self.full_config = result

    def write(self):
        if self.is_hpp:
            with open(self.hpp_template_path) as template_file:
                template = template_file.read()
            config_hpp = em.expand(template, {"config": self.full_config})

            with open(self.output_config_path, "w") as file:
                file.write(config_hpp)

        elif self.is_yaml:
            with open(self.output_config_path, "w") as file:
                yaml.dump(self.full_config, file)

    def _get_starting_pose(self) -> dict:
        # distance between the rear of the chassis and the base_link
        rear_to_base_link_dist = (
            self.nominal_config["chassis_size"][0] / 2
            - self.nominal_config["base_link_to_chassis"][0]
        )
        x = rear_to_base_link_dist + self.nominal_config["wall_width"] / 2
        y = -self.nominal_config["cell_size"] / 2
        yaw = 0.0
        cell_inner_size = (
            self.nominal_config["cell_size"] - self.nominal_config["wall_width"]
        )

        return {
            "starting_pose": [x, y, yaw],
            "cell_inner_size": cell_inner_size,
        }

    def _get_wheel_base(self) -> dict:
        return {
            "wheel_base": (
                self.nominal_config["chassis_size"][1]
                + self.nominal_config["wheel_length"]
            )
        }

    def _get_tofs_config(self) -> dict:
        tof_range_cells = math.ceil(
            self.nominal_config["tof_range"] / (self.nominal_config["cell_size"])
        )
        tof_side = self._pose_deg_to_rad(self.nominal_config["tofs_poses"]["side"])
        tof_diag = self._pose_deg_to_rad(self.nominal_config["tofs_poses"]["diag"])
        tof_front = self._pose_deg_to_rad(self.nominal_config["tofs_poses"]["front"])
        return {
            "tofs_poses": {
                "right_side": tof_side,
                "right_diag": tof_diag,
                "right_front": tof_front,
                "left_side": self._mirror_pose(tof_side),
                "left_diag": self._mirror_pose(tof_diag),
                "left_front": self._mirror_pose(tof_front),
            },
            "tof_range_cells": tof_range_cells,
        }

    def _get_nominal_values(self) -> dict:
        COPY_KEYS = [
            "wall_width",
            "cell_size",
            "tof_range",
            "maze_size",
            "freq_imu_enc",
            "freq_tofs",
            "chassis_size",
            "chassis_mass",
            "base_link_to_chassis",
            "wheel_radius",
            "wheel_length",
            "wheel_mass",
            "motor_radius",
            "motor_length",
            "motor_mass",
            "num_particles",
            "num_eff_particles_threshold",
        ]

        return {key: self.nominal_config[key] for key in COPY_KEYS}

    def _pose_deg_to_rad(self, pose: list) -> list:
        """Convert the yaw of a pose from degrees to radians."""
        return [pose[0], pose[1], math.radians(pose[2])]

    def _mirror_pose(self, pose: list) -> list:
        """Mirror a pose (x, y, yaw) around the x-axis."""
        return [pose[0], -pose[1], -pose[2]]


def parse_args() -> tuple[str, str]:
    parser = ArgumentParser()
    parser.add_argument(
        "nominal_config", type=str, help="Path to the nominal configuration file."
    )
    parser.add_argument(
        "output_config", type=str, help="Path to the output configuration file."
    )
    args = parser.parse_args()
    return args.nominal_config, args.output_config


def main():
    nominal_config_path, output_config_path = parse_args()
    generator = ConfigGenerator(nominal_config_path, output_config_path)
    generator.parse()
    generator.write()


if __name__ == "__main__":
    main()
