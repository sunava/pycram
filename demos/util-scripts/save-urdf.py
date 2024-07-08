from urdf_parser_py.urdf import URDF

kitchen = URDF.from_parameter_server('iai_kitchen')
print (kitchen)
f = open("../../resources/pre_robocup_5.urdf", "w")
f.write(kitchen.to_xml_string())

