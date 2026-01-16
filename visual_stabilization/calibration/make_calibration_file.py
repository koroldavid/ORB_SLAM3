import numpy as np
import yaml
import tarfile
import os
import shutil

# Path to the tarball
TARBALL_PATH = '/tmp/calibrationdata.tar.gz'
EXTRACT_DIR = '/tmp/calibration_data/'

# Path to the output YAML file
PATH_TO_OUTPUT_FILE = '/home/drones/ORB_SLAM3/visual_stabilization/calibration/'
OUTPUT_FILE = 'slam_rpi5_stereo_calibration.yaml'

# Function to extract the tarball
def extract_tarball(tarball_path, extract_dir):
    if not os.path.exists(extract_dir):
        os.makedirs(extract_dir)
    with tarfile.open(tarball_path, 'r:gz') as tar:
        tar.extractall(path=extract_dir)
    print(f"Extracted files to {extract_dir}")

# Load the YAML files for left and right cameras
def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

# Extract camera parameters from the loaded YAML data
def extract_camera_params(camera_data):
    fx = camera_data['camera_matrix']['data'][0]
    fy = camera_data['camera_matrix']['data'][4]
    cx = camera_data['camera_matrix']['data'][2]
    cy = camera_data['camera_matrix']['data'][5]
    
    k1 = camera_data['distortion_coefficients']['data'][0]
    k2 = camera_data['distortion_coefficients']['data'][1]
    p1 = camera_data['distortion_coefficients']['data'][2]
    p2 = camera_data['distortion_coefficients']['data'][3]
    
    return fx, fy, cx, cy, k1, k2, p1, p2

# Extract rectification matrix from the YAML data
def extract_rectification_matrix(camera_data):
    return np.array(camera_data['rectification_matrix']['data']).reshape(3, 3)

# Extract the projection matrix from the YAML data
def extract_projection_matrix(camera_data):
    return np.array(camera_data['projection_matrix']['data']).reshape(3, 4)

def calculate_rotation_difference(R1, R2):
    R1_inv = np.linalg.inv(R1)
    R_diff = np.dot(R1_inv, R2)
    return R_diff

def calculate_t_diff(R1, t1, t2):
    R1_inv = np.linalg.inv(R1)
    t_diff = np.dot(R1_inv, (t2 - t1))
    return t_diff

def extract_translation_from_projection(P):
    # P is 3x4, translation is last column divided by fx
    fx = P[0, 0]
    Tx = P[0, 3]
    t = np.array([Tx / fx, 0.0, 0.0])
    return t

# Function to save the final SLAM calibration to a YAML file
def save_slam_calibration_to_file(camera1_params, camera2_params, T_c1_c2, output_file, threshold=1e-6):
    # Преобразуем матрицу в список
    T_c1_c2_list = T_c1_c2.flatten().tolist()
    # Применяем порог для исключения очень маленьких значений
    T_c1_c2_list = [round(t, 8) if abs(t) > threshold else 0.0 for t in T_c1_c2_list]
    # Форматируем данные в строки по 4 элемента в каждой
    data_lines = []
    for i in range(0, len(T_c1_c2_list), 4):
        line = ', '.join([f'{T_c1_c2_list[j]:.8f}' for j in range(i, min(i + 4, len(T_c1_c2_list)))])
        if i + 4 < len(T_c1_c2_list):
            data_lines.append(line + ',')
        else:
            data_lines.append(line)
    data_str = '\n         '.join(data_lines)

    # Собираем финальную строку для записи в файл
    calibration_data = f"""%YAML:1.0

#--------------------------------------------------------------------------------------------
# System config
#--------------------------------------------------------------------------------------------

# When the variables are commented, the system doesn't load a previous session or not store the current one

# If the LoadFile doesn't exist, the system give a message and create a new Atlas from scratch
#System.LoadAtlasFromFile: "Session_MH01_MH02_MH03_Stereo60_Pseudo"

# The store file is created from the current session, if a file with the same name exists it is deleted
#System.SaveAtlasToFile: "Session_MH01_MH02_MH03_Stereo60_Pseudo"

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------
File.version: "1.0"

Camera.type: "PinHole"

# Camera calibration and distortion parameters (OpenCV) 

Camera1.fx: {camera1_params[0]}
Camera1.fy: {camera1_params[1]}
Camera1.cx: {camera1_params[2]}
Camera1.cy: {camera1_params[3]}

Camera1.k1: {camera1_params[4]}
Camera1.k2: {camera1_params[5]}
Camera1.p1: {camera1_params[6]}
Camera1.p2: {camera1_params[7]}

Camera2.fx: {camera2_params[0]}
Camera2.fy: {camera2_params[1]}
Camera2.cx: {camera2_params[2]}
Camera2.cy: {camera2_params[3]}

Camera2.k1: {camera2_params[4]}
Camera2.k2: {camera2_params[5]}
Camera2.p1: {camera2_params[6]}
Camera2.p2: {camera2_params[7]}

Camera.width: 320
Camera.height: 240

# Camera frames per second 
Camera.fps: 30

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

# Close/Far threshold. Baseline times.
Stereo.ThDepth: 60.0
Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [{data_str}]
  
#--------------------------------------------------------------------------------------------
# IMU Parameters
#--------------------------------------------------------------------------------------------

# Transformation from camera 0 to body-frame (imu)
IMU.T_b_c1: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [0.13553262757638626, -0.9822579869806232, 0.12961540755255102, 0.014788528664512458,
         -0.10514535436195005, -0.14434347637008307, -0.983925513077844, 0.10736558349125136,
         0.9851778323319804, 0.11972555216900271, -0.12284311474188078, -0.04558093832923264,
         0.0, 0.0, 0.0, 1.0]

# IMU noise
IMU.NoiseAcc: 0.0006414067563726991
IMU.AccWalk: 7.079826007482991e-05 
IMU.NoiseGyro: 6.105141437749343e-05 
IMU.GyroWalk: 4.128732963366765e-06 
IMU.Frequency: 250.0

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1000

# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast			
ORBextractor.iniThFAST: 30
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
Viewer.imageViewScale: 1.0
"""
    
    # Записываем данные в файл
    with open(output_file, 'w') as f:
        f.write(calibration_data)
    
    print(f"Calibration saved to {output_file}")

def main():
    # Extract the tarball containing the YAML files
    extract_tarball(TARBALL_PATH, EXTRACT_DIR)
    
    # Load the camera calibration data from the extracted files
    left_data = load_yaml(os.path.join(EXTRACT_DIR, 'left.yaml'))
    right_data = load_yaml(os.path.join(EXTRACT_DIR, 'right.yaml'))

    # Extract parameters for left and right cameras
    camera1_params = extract_camera_params(left_data)
    camera2_params = extract_camera_params(right_data)
    
    # Extract rectification matrices for left and right cameras
    R_c1 = extract_rectification_matrix(left_data)
    R_c2 = extract_rectification_matrix(right_data)
    
    # Extract projection matrices for left and right cameras
    projection_matrix_left = extract_projection_matrix(left_data)
    projection_matrix_right = extract_projection_matrix(right_data)

    # Получаем трансляции из projection matrix
    t1 = extract_translation_from_projection(projection_matrix_left)
    t2 = extract_translation_from_projection(projection_matrix_right)
    
    # Вычисляем относительную матрицу поворота и трансляции
    R_diff = calculate_rotation_difference(R_c1, R_c2)
    t_diff = calculate_t_diff(R_c1, t1, t2)

    # Собираем 4x4 матрицу
    T_c1_c2 = np.eye(4)
    T_c1_c2[:3, :3] = R_diff
    T_c1_c2[:3, 3] = t_diff
    
    # Save the final SLAM calibration to the output path
    output_path = os.path.join(PATH_TO_OUTPUT_FILE, OUTPUT_FILE)
    save_slam_calibration_to_file(camera1_params, camera2_params, T_c1_c2, output_path)

    # Copy left.yaml and right.yaml to the output directory
    shutil.copy(os.path.join(EXTRACT_DIR, 'left.yaml'), os.path.join(PATH_TO_OUTPUT_FILE, 'left.yaml'))
    shutil.copy(os.path.join(EXTRACT_DIR, 'right.yaml'), os.path.join(PATH_TO_OUTPUT_FILE, 'right.yaml'))
    
    print(f"left.yaml and right.yaml copied to {PATH_TO_OUTPUT_FILE}")

if __name__ == "__main__":
    main()

