#!/usr/bin/env python3

# import yaml

# with open("mixedexp_3d.yaml") as stream:
#     try:
#         print(yaml.safe_load(stream))
#         my_dict = yaml.load(stream,Loader=yaml.FullLoader)
#         print(my_dict)
#     except yaml.YAMLError as exc:
#         pass
#         # print(exc)

# print(my_dict)

import yaml
from pathlib import Path
import numpy as np



yaml_dict = yaml.safe_load(Path("./mixedexp_3d_10a.yaml").read_text())

print(yaml_dict['ros_ns_str'])
print(type(yaml_dict['ros_ns_str']))

formation   =   np.zeros((len(yaml_dict['ros_ns_str']),3),dtype=np.float64)

for index, x in np.ndenumerate(formation):
    if isinstance(yaml_dict['formation'][index[0]][index[1]], str):
        formation[index[0],index[1]]    =   eval(yaml_dict['formation'][index[0]][index[1]])

    elif isinstance(yaml_dict['formation'][index[0]][index[1]], float) or isinstance(yaml_dict['formation'][index[0]][index[1]], int):
        formation[index[0],index[1]]    =   np.float64(yaml_dict['formation'][index[0]][index[1]])

adjacency   =   np.zeros((len(yaml_dict['ros_ns_str']),len(yaml_dict['ros_ns_str'])),dtype=np.float64)

for index, x in np.ndenumerate(adjacency):
    if isinstance(yaml_dict['adjacency'][index[0]][index[1]], float) or isinstance(yaml_dict['adjacency'][index[0]][index[1]], int):
        adjacency[index[0],index[1]]    =   np.float64(yaml_dict['adjacency'][index[0]][index[1]])

# print(adjacency)
# print(type(adjacency))
# print(yaml_dict['formation'][index[0]][index[1]])
    # print(type(yaml_dict['formation'][index[0]][index[1]]))
    # print(type(x))

# print(formation)