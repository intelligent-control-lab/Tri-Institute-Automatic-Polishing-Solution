import shutil
import os


source_dir = "scan_data_temporary"
destination_dir = "destination"
if not os.path.exists(destination_dir):
    os.makedirs(destination_dir)
files = os.listdir(source_dir)

for file in files:
    source_file = os.path.join(source_dir, file)
    destination_file = os.path.join(destination_dir, file)
    
    if os.path.isfile(source_file):
        try:
            shutil.copy(source_file, destination_file)
            print(f"Copied file: {source_file} -> {destination_file}")
        except Exception as e:
            print(f"Error copying file {source_file} to {destination_file}: {e}")
            
