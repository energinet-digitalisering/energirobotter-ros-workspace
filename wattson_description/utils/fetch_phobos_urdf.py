import shutil
import re

# --- Configuration ---
source_file = "/home/drone/energinet/Files/wattson_model/Wattson/urdf/Wattson.urdf"
destination_file = "/home/drone/energinet/elrik_ws/src/energirobotter-ros-workspace/wattson_description/urdf/phobos_generated.urdf"

# --- Step 1: Copy file ---
shutil.copy(source_file, destination_file)

# --- Step 2: Read, modify, and write ---
with open(destination_file, "r") as f:
    content = f.read()

# Replace all "../" with package URI
content = content.replace("../", "package://wattson_description/")

# Replace <collision name="..."> with <collision>
content = re.sub(r'<collision\s+name="[^"]*">', "<collision>", content)

# Write back the modified content
with open(destination_file, "w") as f:
    f.write(content)

print(f"Updated URDF saved to: {destination_file}")
