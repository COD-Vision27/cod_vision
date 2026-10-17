# 一. `linux/can.h`

#### 自带串口帧结构体

```cpp
struct can_frame {
    canid_t can_id;   // 32 位 CAN ID + 标志位
    __u8    can_dlc;  // 数据长度码，范围 0 ~ 8
    __u8    data[8] __attribute__((aligned(8))); // 载荷数据
};
```

`PF_CAN`，`SOCK_RAW`，`CAN_RAW`

套接字为通信接口的抽象，所以要传入选择什么协议，什么方式去通信。

创建套接字传参，分别代表:是用CAN协议簇，原始套接字类型,使用 RAW 协议.

`int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);`

# 二. `net/if.h`

#### 处理接口名和接口索引转换

#### 套接字创建完后需要bind绑定CAN接口，这个库则是把索引名字转到索引

#### 提供了两种方式

1. 

#### `unsigned int ifindex = if_nametoindex("can0");`

2. 

```cpp
struct ifreq ifr;
strncpy(ifr.ifr_name, "can0", IFNAMSIZ);
ioctl(s, SIOCGIFINDEX, &ifr);
unsigned int idx = ifr.ifindex;
```

#### 随后加入`sockaddr_can`结构体的成员`can_ifindex`中即可.

#### `ifreq`结构体

```cpp
struct ifreq {
    char ifr_name[IFNAMSIZ];   // 接口名，如 "can0"
    union {
        struct sockaddr ifr_addr;
        struct sockaddr ifr_hwaddr;
        short           ifr_flags;
        int             ifr_ifindex;  // 接口索引
        int             ifr_mtu;
        // ...
    } ifr_ifru;
};

#define ifr_ifindex ifr_ifru.ifr_ifindex
```

# 三. `sys/epoll.h`

#### 监听多个socket,不关心什么协议，是个套接字就可以监听,UDS,tcp等也可以用

#### 面对多个socket,读取数据会相互堵塞。`epoll.h`将套接字注册到内核，由内核告诉程序哪些套接字有内容可读。

```cpp
struct epoll_event {
    uint32_t     events;  // 事件类型掩码
    epoll_data_t data;    // 用户数据
};

typedef union epoll_data {
    void    *ptr;
    int      fd;
    uint32_t u32;
    uint64_t u64;
} epoll_data_t;
```

- **`events`**：关注的事件类型（见下文宏）。
- **`data`**：用户自定义数据，`epoll_wait()` 返回时会原样带回来，通常存放 fd 或自定义结构体指针，用来区分是哪个套接字触发了事件。

| 宏             | 含义                           |
| -------------- | ------------------------------ |
| `EPOLLIN`      | 有数据可读                     |
| `EPOLLOUT`     | 可写（发送缓冲区有空位）       |
| `EPOLLERR`     | 发生错误                       |
| `EPOLLHUP`     | 对端挂断                       |
| `EPOLLRDHUP`   | 对端关闭读方向                 |
| `EPOLLET`      | 边沿触发模式（Edge Triggered） |
| `EPOLLONESHOT` | 只监听一次，触发后自动移除     |

#### 创建一个epoll:`int epfd = epoll_create1(0);`

#### 操作监听到epoll: `int epoll_ctl(int epfd, int op, int fd, struct epoll_event *event);`

| `op` 值         | 含义                                    |
| --------------- | --------------------------------------- |
| `EPOLL_CTL_ADD` | 添加新的 fd 到 epoll                    |
| `EPOLL_CTL_MOD` | 修改已注册 fd 的事件                    |
| `EPOLL_CTL_DEL` | 从 epoll 中删除 fd（`event` 可为 NULL） |

#### 等待套接字:

```cpp
int epoll_wait(int epfd, struct epoll_event *events,int maxevents, int timeout);
```

- `events`：输出数组，内核把就绪的事件填入。
- `maxevents`：本次最多返回多少个事件（必须 ≥ 1）。
- `timeout`：毫秒；`-1` 表示永久阻塞，`0` 表示立即返回。
- 返回值：就绪的 fd 数量，0 表示超时，-1 表示出错。

# 四. `sys/ioctl.h`

#### 获取接口各种信息,与`net/if`搭配使用，结构体`ifreq`在`net/if`

#### 主要函数

```cpp
int ioctl(int fd, unsigned long request, ...);
```

| 请求码          | 作用                | 第三个参数       |
| --------------- | ------------------- | ---------------- |
| `SIOCGIFINDEX`  | 获取接口索引        | `struct ifreq *` |
| `SIOCGIFFLAGS`  | 获取接口标志        | `struct ifreq *` |
| `SIOCGIFMTU`    | 获取接口 MTU        | `struct ifreq *` |
| `SIOCGIFHWADDR` | 获取硬件地址（MAC） | `struct ifreq *` |
| `SIOCSIFMTU`    | 设置接口 MTU        | `struct ifreq *` |
| `SIOCSIFFLAGS`  | 设置接口标志        | `struct ifreq *` |

# 五. `unistd.h`

#### 提供基础读写帧信息

`ssize_t read(int fd, void *buf, size_t count);`

`ssize_t write(int fd, const void *buf, size_t count);`

`int close(int fd);`
