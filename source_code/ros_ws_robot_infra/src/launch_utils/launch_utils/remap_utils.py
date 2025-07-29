#  The ROS robot project (launch file utils)
#  Load topic remappings from a YAML file.
#  Copyright 2024 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2024.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https: www.gnu.org/licenses/>.

from typing import Union
from ament_index_python import get_package_share_directory
from launch_ros.actions import SetRemap
import yaml
import os


def _load_yaml(package: str, filename: str):
    remapping_file = os.path.join(
        get_package_share_directory(package),
        "config", filename
    )

    with open(remapping_file, "r") as f:
        return yaml.safe_load(f)


def load_remappings(package: str, filename: str) -> list[SetRemap]:
    data = _load_yaml(package, filename)
    ret_list = []

    for entry in data.get("remappings", []):
        ret_list.append(SetRemap(entry["from"], entry["to"]))
    return ret_list


def load_remappings_tuple(package: str, filename: str, node_name: str="") -> list[tuple[str, str]]:
    data = _load_yaml(package, filename)
    ret_list = []

    for node in data.get("remappings", []):
        if node == "_" or (node == node_name if node_name != "" else True):
            for rmp in entry.get("remappings", {}).get(node):
                ret_list.append((rmp["from"].replace("<node_name>", node_name),
                                 rmp["to"].replace("<node_name>", node_name)))
    return ret_list
