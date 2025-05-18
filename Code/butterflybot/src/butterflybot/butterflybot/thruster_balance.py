import os
import math
import xml.etree.ElementTree as ET

# Load the SDF file
sdf_file = os.path.expanduser("~/butterflybot/src/butterflybot/urdf/butterflybot.sdf")
tree = ET.parse(sdf_file)
root = tree.getroot()

# Find the model (should be one top-level <model>)
model = root.find(".//model")
if model is None:
    raise ValueError("No <model> tag found in SDF.")

# Compute center of mass (simplified average of all link poses)
link_positions = []
for link in model.findall("link"):
    pose = link.find("pose")
    if pose is not None:
        x, y, z, *_ = map(float, pose.text.strip().split())
        link_positions.append((x, y, z))

if not link_positions:
    raise ValueError("No link poses found to compute CoM.")

com_x = sum(x for x, y, z in link_positions) / len(link_positions)
com_y = sum(y for x, y, z in link_positions) / len(link_positions)
com_z = sum(z for x, y, z in link_positions) / len(link_positions)

# Process thruster joints
updated = 0
for joint in model.findall("joint"):
    name = joint.get("name", "")
    if "thruster" not in name.lower():
        continue

    pose_elem = joint.find("pose")
    if pose_elem is None:
        continue

    x, y, z, *_ = map(float, pose_elem.text.strip().split())
    dx = x - com_x
    dy = y - com_y
    torque = dx * dy  # Example 2D torque influence
    thrust = math.hypot(dx, dy)  # Euclidean distance in 2D

    # Apply simple scaling to determine an example effort coefficient
    coefficient = thrust + (0.1 * torque)

    # Update <velocity> or <effort> tag (depends on actuator type)
    effort_elem = joint.find(".//effort")
    if effort_elem is None:
        effort_elem = ET.SubElement(joint, "effort")
    effort_elem.text = f"{coefficient:.3f}"

    updated += 1
    print(f"Updated joint '{name}' with effort coefficient: {coefficient:.3f}")

# Save modified SDF
backup_file = sdf_file + ".bak"
os.rename(sdf_file, backup_file)
tree.write(sdf_file, encoding="utf-8", xml_declaration=True)

print(f"\n✅ Updated {updated} thruster joints.")
print(f"📁 Original file backed up to: {backup_file}")
print(f"💾 Updated SDF saved at: {sdf_file}")
