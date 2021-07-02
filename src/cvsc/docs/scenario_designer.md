## scenario designer

- 综述：为了方便用户自定义和路线和场景，设计的一款工具，用户可以在地图上通过简单的鼠标操作实现路线自定义和设置障碍物。一切的操作建立在carla服务端开启的情况下。（支持CARLA0.9.5以上版本）

### free_scenario_design.py

- 界面操作的主要实现，通过client端获取carla的地图，再通过carla官方给出的API进行障碍物的添加和waypoint 的生成，以及保存为最后的config配置文件。  
- API参考:https://carla.readthedocs.io/en/latest/python_api/  
- 结构图以及用到的API：

### catmull_rom_spline.py

- catmull算法的实现，通过用户手动设置的点实现轨迹的生成和离散取点，在free_scenario_design.py 中调用。



### csv_to_xml_helper.py

- 实现csv格式到xml格式的转换
- 调用方式：命令行输入python csv_to_xml_helper.py route input_file_name output_file_name 
- 生成的文件自动放在output_xmls文件夹下

