import xml.etree.ElementTree as ET

tree = ET.parse('your_urdf_file.urdf')
root = tree.getroot()

link = root.find(".//link[@name='your_link_name']")
inertial = link.find('inertial')
mass = inertial.find('mass')
mass.set('value', '2.0')

tree.write('your_new_urdf_file.urdf')
