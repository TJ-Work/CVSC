

## ros 

#### ros spin（）与ros spinOnce（）

- [博客](http://www.cnblogs.com/liu-fa/p/5925381.html  )
- [官方基础教程](http://wiki.ros.org/ROS/Tutorials )

#### ros 基础

- [博客](https://www.cnblogs.com/BlueMountain-HaggenDazs/p/6269815.html)
- [gitbook](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/chapter7/7.1.html)

## c++ 

- [arg,arv](https://blog.csdn.net/dgreh/article/details/80985928)

- [ros传递参数](https://www.jianshu.com/p/02ee8f513295)

## python 

- [python .format填充用法 ](https://blog.csdn.net/u014770372/article/details/76021988)  
- [python assert（）断言用法](https://blog.csdn.net/qq_39247153/article/details/81082313)
- [python super 函数](https://www.runoob.com/python/python-func-super.html) 
- [python raise 异常](https://blog.csdn.net/u014148798/article/details/52288326)
- [python抽象类](https://blog.csdn.net/ahilll/article/details/82380467/)
- [python stacticmethod ](https://blog.csdn.net/handsomekang/article/details/9615239)
- [python abstractmethod](https://blog.csdn.net/xiemanR/article/details/72629164/)
- [threading lock 用法](https://blog.csdn.net/qq_21439971/article/details/79356248)
- [py_trees](https://py-trees.readthedocs.io/en/devel/)
- [python dict iteritems](http://byliu.github.io/2016/04/04/python-dict%E5%87%A0%E7%A7%8D%E9%81%8D%E5%8E%86%E6%96%B9%E5%BC%8F%E6%80%A7%E8%83%BD%E7%AE%80%E5%8D%95%E6%AF%94%E8%BE%83/)
- [补充](https://blog.csdn.net/yedoubushishen/article/details/51984524)
- [python argparse 命令行](https://docs.python.org/3/howto/argparse.html#introducing-positional-arguments)

## carla 知识学习

- [官网](https://http://carla.org//)
- [博客](https://blog.csdn.net/chepwavege/article/details/90904813)
- [github](https://github.com/carla-simulator)
- [车辆物理模型](https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Vehicles.html)
- [carla_ros_bridge 官方教程](https://github.com/carla-simulator/ros-bridge)
- 仿真API: 现在customed_carla_simulation中，进一步用python\json的写法包装了提供了足够好的车辆结构
  设定API。
  进一步回顾几个重要的API, 进一步工作可能有用

```
client = carla.Client(host, port)
client.set_timeout(sec)
world = client.get_world()
actor_list = world.get_actors()
actor.attribute.get(​"role_name"​) ​#找名字 actor_list.filter(​"*traffic_light*"​) ​#找所有灯对应actor得到的一个 list/generator
waypoint=world.get_map().get_waypoint(carla.Location) ​#对应一个点找路线点 waypoint.next(distance) ​#根据一个点找一定距离外的下一个点
```

- world.wait_for_tick()： blocks until a new tick is received from the simulator. This function is not very reliable as you may skip ticks, and you can easily end up in a dead-lock if using synchronous mode.
- world.on_tick(callback)： registers a callback (function or function object) to be called every time a new tick is received, the callback is executed asynchronously in the background.
- world.tick()： tells the simulator to perform a tick (i.e. advance one step). This only has effect in synchronous mode.

## 