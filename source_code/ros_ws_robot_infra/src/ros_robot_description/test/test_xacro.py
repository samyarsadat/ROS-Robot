from ament_index_python.packages import get_package_share_directory
import itertools
import os
import xacro


def test_robot_desc():
    test_vals = {
        "namespace": ["", "my_robot"],
        "use_sim": ["False", "True"]
    }

    value_combos = list(itertools.product(*test_vals.values()))
    all_combos_dict = [dict(zip(test_vals.keys(), values)) for values in value_combos]

    desc_pkg_share_dir = get_package_share_directory("ros_robot_description")
    xacro_file_path = os.path.join(desc_pkg_share_dir, "urdf", "ros_robot.urdf.xacro")

    for combo in all_combos_dict:
        try:
            xacro.process_file(xacro_file_path, mappings=combo)
        except xacro.XacroException as e:
            kvs = [f"{key}: {value}" for key, value in combo.items()]
            assert False, f"Xacro parsing failed: {str(e)}! [{', '.join(kvs)}]"
