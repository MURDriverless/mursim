import xml.etree.ElementTree as ET

tree = ET.parse('model.sdf')
children = tree.getroot().getchildren()

for child in children:
    tree = child
for child in tree.getchildren():
    try:
        pose = child.find('pose').text
        if (pose):
            print(pose)
    except:
        '''
        AHHHHHHHHHHHHHHHHHHHHHHH
        '''
        pass
