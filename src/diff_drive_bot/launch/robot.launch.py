import os

def generate_launch_description():
    package = "diff_drive_bot"
    urdf_path = os.path.join(get_package_share_directory(package), "urdf", "robot.urdf.xacro")

