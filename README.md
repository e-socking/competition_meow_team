
-----------包说明：
1.config：内含提取遮罩的参数
2.launch：内含一个运行节点的launch文件
3.results：
有一张测试时的截图（因为测试时没留心截图所以只有这一张）
另外，裁判系统生成的结果文件分别在此目录下的子目录里


----------运行命令：
1.运行节点：
ros2 run team_meow_challenge sphere_node
ros2 run team_meow_challenge armor_node
ros2 run team_meow_challenge rect_node

2.使用launch运行节点：
ros2 launch team_meow_challenge vision.launch.py node_to_launch:=rect_node        rect
ros2 launch team_meow_challenge vision.launch.py node_to_launch:=armor_node       armor
ros2 launch team_meow_challenge vision.launch.py node_to_launch:=sphere_node      sphere



-----------技术报告：
https://www.yuque.com/ensocking/kgxhb3/cxblucg7s93tc93f#
