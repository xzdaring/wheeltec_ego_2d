# 离线 Gazebo 模型资源

来源：https://github.com/osrf/gazebo_models
固定版本：8163eb4b5e7e21985c6591d1c0bfb56468c0093f

仅复制 ego_test_world2.world 引用的六类外部资源：
cafe_table、cardboard_box、car_wheel、coke_can、first_2015_trash_can、table_marble。
保留模型配置中的作者与许可证字段，第三方网格、材质、纹理未作修改。

launch 将安装后的 models 目录加入 GAZEBO_MODEL_PATH，并保留已有搜索目录。
这些资源跟随 robot_sim 安装，无需 ~/.gazebo/models 缓存或在线模型库。
