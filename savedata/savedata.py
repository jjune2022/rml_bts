import os
import numpy as np

def save_to_txt(data, folder_path, filename):
    """
    Save a list of data to a text file.

    Parameters:
    - data: List of data to save
    - folder_path: Folder path where the file will be saved
    - filename: Name of the file (without extension)
    """
    os.makedirs(folder_path, exist_ok=True)
    file_path = os.path.join(folder_path, f"{filename}.txt")

    with open(file_path, 'w') as file:
        for item in data:
            file.write(f"{item}\n")
    print(f"Data saved to {file_path}")

def unify_length(data_list, max_length, fill_value=np.nan):
    """
    Ensure all data lists are the same length by padding with a fill value.

    Parameters:
    - data_list: List of data lists
    - max_length: Target length
    - fill_value: Value to fill for missing entries

    Returns:
    - List of unified length data arrays
    """
    unified_data = []
    for data in data_list:
        unified_data.append(
            list(data) + [fill_value] * (max_length - len(data))
        )
    return unified_data

def save_data(heading_error, cross_track_error, odom_x, odom_y,
                  linear_velocity, path_type, max_length, controller):
    print('end')
    """
    Save heading error, cross track error, and odometry data to text files.

    Parameters:
    - heading_error: List of heading errors
    - cross_track_error: List of cross track errors
    - odom_x: List of odometry x-coordinates
    - odom_y: List of odometry y-coordinates
    - linear_velocity: Linear velocity for naming files
    - path_type: Path type for folder naming
    - max_length: Target length to unify all data lists
    """
    base_dir = "/home/rml/grad_test_data"
    he_folder = os.path.join(base_dir, "HE", path_type, f"{linear_velocity}")
    cxe_folder = os.path.join(base_dir, "CXE", path_type, f"{linear_velocity}")
    odom_folder = os.path.join(base_dir, "Odom", path_type, f"{linear_velocity}")

    # Unify data lengths
    heading_error, cross_track_error, odom_x, odom_y = unify_length(
        [heading_error, cross_track_error, odom_x, odom_y], max_length
    )

    # Save data
    save_to_txt(heading_error, he_folder, f"HE_{linear_velocity}_{controller}")
    save_to_txt(cross_track_error, cxe_folder, f"CXE_{linear_velocity}_{controller}")
    save_to_txt(zip(odom_x, odom_y), odom_folder, f"Odom_{linear_velocity}_{controller}")


