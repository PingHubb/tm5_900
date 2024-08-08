import os


def create_folders(start, end, increment, base_path):
    # Ensure the base path exists
    if not os.path.exists(base_path):
        os.makedirs(base_path)

    # Create folders from start to end with a given increment
    current = start
    while current <= end:
        folder_name = f"{current:.1f}cm"
        folder_path = os.path.join(base_path, folder_name)
        os.makedirs(folder_path, exist_ok=True)
        print(f"Created folder: {folder_path}")
        current += increment


# Define the start, end, increment, and base directory
start_cm = 1.6
end_cm = 2.0
increment_cm = 0.1
base_directory = "/home/ping2/ros2_ws/src/lan_control/lan_control/resources/experiment_2"  # Change this to your desired path

# Call the function to create the folders
create_folders(start_cm, end_cm, increment_cm, base_directory)
