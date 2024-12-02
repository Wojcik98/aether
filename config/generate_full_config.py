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

        result["chassis_size"] = self.nominal_config["chassis_size"]
        result["chassis_mass"] = self.nominal_config["chassis_mass"]
        result["wheel_radius"] = self.nominal_config["wheel_radius"]
        result["wheel_length"] = self.nominal_config["wheel_length"]
        result["wheel_mass"] = self.nominal_config["wheel_mass"]
        result["motor_radius"] = self.nominal_config["motor_radius"]
        result["motor_length"] = self.nominal_config["motor_length"]
        result["motor_mass"] = self.nominal_config["motor_mass"]

        tof_side = self._pose_deg_to_rad(self.nominal_config["tofs_poses"]["side"])
        tof_diag = self._pose_deg_to_rad(self.nominal_config["tofs_poses"]["diag"])
        tof_front = self._pose_deg_to_rad(self.nominal_config["tofs_poses"]["front"])
        result["tofs_poses"] = {
            "right_side": tof_side,
            "right_diag": tof_diag,
            "right_front": tof_front,
            "left_side": self._mirror_pose(tof_side),
            "left_diag": self._mirror_pose(tof_diag),
            "left_front": self._mirror_pose(tof_front),
        }

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
