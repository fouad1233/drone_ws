import subprocess

# Define the path to your .sh script
script_path = '/home/erhangk/drone_ws/src/ros_guides/scripts/draw_circle.py'
script_path1 = '/home/erhangk/drone_ws/src/ros_guides/scripts/my_first_node.py'


# Run the .sh script using subprocess
try:
    subprocess.Popen(['gnome-terminal', '--', 'python3', script_path])
    subprocess.Popen(['gnome-terminal', '--', 'python3', script_path1])

except Exception as e:
    print(f"Error running the script: {e}")