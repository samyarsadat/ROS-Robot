from ament_index_python import get_package_share_directory
import yaml
import os

def load_remappings(package: str, filename: str, node_name: str) -> list:
    remapping_file = os.path.join(
        get_package_share_directory(package),
        "config", filename
    )

    with open(remapping_file, "r") as f:
        data = yaml.safe_load(f)

    ret_list = []
    for node in data.get("remappings", []):
        if node == node_name or node == "-*-":
            rmp_entry = data.get("remappings", {}).get(node)[0]
            ret_list.append((rmp_entry["from"], rmp_entry["to"]))

    return ret_list