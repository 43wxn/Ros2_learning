# Ros2_learning
学习Ros2
# 节点相关的CLI
## 运行节点
`ros2 run <package_name> <executable_name>`**注意是CMakeList.txt中给的生成的可执行文件的名称 而不是源码中的节点名字**
## 查看节点列表
`ros2 node list`
## 查看节点信息
`ros2 node info <node_name>`
## 重映射节点名称
`ros2 run <package_name> <node_name> --ros-args --remap __node:=<my_node_name>`

# 工作空间/功能包/节点
>一个工作空间可以有多个功能包，一个功能包可以有多个节点

## 功能包
**ROS2中功能包根据编译方式的不同分为三种类型。**
- ament_python，适用于python程序
- cmake，适用于C++
- ament_cmake，适用于C++程序,是cmake的增强版
  
### 与功能包相关的指令
```bash
create       Create a new ROS2 package

executables  Output a list of package specific executables

list         Output a list of available packages

prefix       Output the prefix path of a package

xml          Output the XML of the package manifest or a specific tag
```
### 创建功能包
`ros2 pkg create <package_name> --build-type {cmake,ament_cmake,ament_python} --dependencies <依赖名称>`
### 列出可执行文件
`ros2 list executable ` 后面可加功能包名字 表示列出这个功能包的所有可执行文件
### 列出所有功能包
`ros2 pkg list`
### 列出功能包的路径前缀
`ros2 pkg prefix <package_name>
### 列出功能包的xml文件
>每一个功能包都有一个标配的manifest.xml文件，用于记录这个包的名字，构建工具，编译信息，拥有者，干啥用的等信息。通过这个信息，就可以自动为该功能包安装依赖，构建时确定编译顺序等
`ros2 pkg xml <package_name>`

## 编译工具 ： colcon
### 编译一个包
`colcon build --packages-select <package_name>`
### 不编译测试单元
`colcon build --packages-select <package_name> --cmake-args -DBUILD_TESTING=0`
### 运行编译的包的测试
`colcon tset`
### 允许通过更改src下的部分文件来改变install
>每次更改src中的python脚本时就不需要重新build了
`colcon build --symlink-install`

## 工作空间标准布局
1.install/lib/<package_name>/    -->放置<package_name>中的节点的可执行文件
2.install/include/<package_name> -->放置<package_name>中的头文件
3.install/share/<package_name>   -->放置资源（例如launch文件，配置文件等）


# colcon 构建
## colcon build COMMAND
1.`--packages-select <package_name1> <package_name2>...   构建指定的包`

2.`--packages-up-to <package_name>构建指定的包和它依赖的包`

3.`--packages-above 构建整个工作区的包 如果对其中一个包进行了修改 那么会重构该包 以及递归构建依赖此包的所有包`

4.`--build-base参数和--install-base，指定构建目录和安装目录`

5.`--merge-install 合并构建目录`

6.`--log-level {debug,info warning,error,critical} 设置日志级别 例如 --log-level info`

 
# 话题与服务
## 常用的指令
`ros2 topic list   列出所有活动中的话题`

`ros2 topic list -t    增加消息类型 同样列出活动中的话题`

`ros2 topic echo <topic_name>  打印实时话题内容`

`ros2 topic info <topic_name>   打印话题信息`

`ros2 interface show <message_name> 查看消息类型信息`

`ros2 topic pub <topic_name> <message_name> 'data:(消息内容)'   向某个话题的某个订阅者发送消息`

## 导入消息接口（对于ament_cmake类型的功能包）
1.在CMakeList.txt中，先find_package(<消息类型所在的功能包>)，在ament_target(可执行文件  功能包1  功能包2 ...)

2.在package.xml中，<depend>消息接口所在功能包</depend>

3.在源码中导入消息接口，#include "消息功能包/xxx/xxx.cpp"

## 创建publisher
```cpp
template<typename MessageT , typename AllocatorT , typename PublisherT >
std::shared_ptr<PublisherT> rclcpp::Node::create_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const PublisherOptionsWithAllocator<AllocatorT> & options = PublisherOptionsWithAllocator<AllocatorT>()
)
```
publisher函数声明：至少要传入消息类型/接口（msgT） ，话题名称（topic_name） ，服务指令（qos）

## 创建定时器
```cpp
template<typename CallbackT>
WallTimer::SharedPtr create_wall_timer(
  std::chrono::nanoseconds period,   // 时间间隔
  CallbackT && callback              // 回调函数
);
```
这里的回调函数必须是一个可调用的对象，若直接传入一个普通函数 则编译会报错。
所以需要使用bind（）函数将回调函数的指针与this对象绑定形成一个新的可调用对象。
***也可以使用Lambda表达式------>建议使用***

```cpp
class TopicPublisher:public rclcpp::Node {
    public:
        TopicPublisher(std::string name):Node(name) {
            RCLCPP_INFO_STREAM(this->get_logger(), name << " 节点已创建");
            command_publisher = this->create_publisher<std_msgs::msg::String>("command", 10);
            timer = this->create_wall_timer(std::chrono::milliseconds(500), [this]()
                                            { this->timer_callback(); });
        }

    private:
        void timer_callback() {
            //创建消息
            std_msgs::msg::String message;
            message.data = "you are handsome!";
            command_publisher->publish(message);
        }
        //command_publisher是SharePtr类型 这个类型在rclcpp的Publisher模板中
        //必须传入消息类型
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher;
        rclcpp::TimerBase::SharedPtr timer;
};
```

## 创建订阅者
```cpp
template<typename MessageT , typename CallbackT , typename AllocatorT , typename CallbackMessageT , typename SubscriptionT , typename MessageMemoryStrategyT >
std::shared_ptr< SubscriptionT > rclcpp::Node::create_subscription 	( 	const std::string &  	topic_name,
		const rclcpp::QoS &  	qos,
		CallbackT &&  	callback,
		const SubscriptionOptionsWithAllocator< AllocatorT > &  	options = SubscriptionOptionsWithAllocator<AllocatorT>(),
		typename MessageMemoryStrategyT::SharedPtr  	msg_mem_strat = (      MessageMemoryStrategyT::create_default()    ) 
	) 	
```
创建订阅者时 只需要调用订阅者节点的 `create_subscription<message_type>("topic_name",qos,call_back())` 即可 
其中`message_type` 例如 `std_msgs::msg::String`

`qos` 消息队列深度 一般为10

`call_back()` 回调函数和发布者的回调函数一样 必须是可调用的对象 一定是一个对象！对象！对象！ 


 

