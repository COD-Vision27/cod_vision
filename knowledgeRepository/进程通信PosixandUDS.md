```
#include <sys/socket.h>   // socket, bind, listen, accept, connect, send, recv, struct sockaddr, AF_UNIX, SOCK_STREAM...
#include <sys/un.h>       // struct sockaddr_un, sun_path, AF_UNIX / AF_LOCAL
#include <unistd.h>       // read, write, close, unlink
```

# POSIX 进程共享内存



```cpp
int shm_open(const char *name, int oflag, mode_t mode);
```

返回fd,类似文件id/名字

作用：类似声明一块内存空间，返回fd

参数释义：

1. name:以 `/` 开头的文件名，实际位置在`/dev/shm/`
2. oflag: 
   1. `O_CREAT`：不存在就创建。
   2. `O_EXCL`：配合 `O_CREAT`，已存在就报错。**用于判断“我是不是第一个创建者”**。
   3. `O_RDWR` / `O_RDONLY`：读写权限。
   4. `O_TRUNC`：已存在则截断为 0。
3. mode:
   1. 带O_CREAT生效，权限位

```cpp
int ftruncate(int fd, off_t length);
```

作用：告诉此共享内存想要分配的大小

1. fd: 要设置共享内存大小的文件id
2. 大小，kb单位

```cpp
void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);
```

作用：把刚刚想要的分配到一个地址

1. addr: 希望映射到的地址，填写null时让内核自动选择

2. length: 映射长度，必须小于ftruncate

3. prot: 此内存访问方式

   1. `PROT_READ`：可读。

      `PROT_WRITE`：可写。

      `PROT_EXEC`：可执行（共享内存几乎不用）

4. flags: 映射的类型
   1. MAP_SHARED
   2. MAP_PRIVATE
   3. MAP_FIXED
   4. 一般shared,其他的参数不懂
5. fd: 要映射的共享内存id
6. offset:地址偏移，一般0即可

```cpp
int munmap(void *addr, size_t length);
```

作用：解除映射，如果不显式使用，直到进程结束才会自动取消映射，否则会一直占用空间

1. addr：映射起始地址
2. length：顾名思义，长度

```cpp
int shm_unlink(const char *name);
```

作用：删除共享内存fd名字/id，不使用的话，即使进程退出对象还是会在目录下

1. name: 共享内存文件名字

### 读写数据：

写：映射后直接将数据memcpy到那个地址即可

读： 映射后直接获取数据即可

# Unix Domain Socket

流程：

服务端：

socket() → bind() → listen() → accept() → read()/write() → close() → unlink()

客户端：

socket() → connect() → read()/write() → close()



```cpp
int socket(int domain, int type, int protocol);
```

作用：创建一个socket,通信节点

参数：

1. domain: 地址簇
   - `AF_UNIX`：Unix Domain Socket
   - `AF_INET`：TCP/IP (网络通信)
2. type: 通信类型
   - `SOCK_STREAM`：面向连接、可靠、字节流（类似 TCP）
   - `SOCK_DGRAM`：无连接、保留消息边界（类似 UDP）
   - `SOCK_SEQPACKET`：面向连接 + 保留消息边界
3. `protocol`：协议，填 `0` 即可

```cpp
int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen);
```

作用：给 socket 分配一个地址（文件路径），其他进程通过这个路径找到你。

参数：

1. `sockfd`：`socket()` 返回的 fd
2. `addr`：地址结构体，UDS 用 `struct sockaddr_un`
3. `addrlen`：地址结构体大小

sockaddr结构体结构:

```cpp
struct sockaddr_un {
    sa_family_t sun_family;   // 必须填 AF_UNIX
    char        sun_path[108]; // 文件路径
};
```



```c++
int listen(int sockfd, int backlog);
```

 作用: 监听客户端连接

参数：

1. `sockfd`：`socket()` 返回的 fd
2. `backlog`：等待队列最大长度，一般填 `5` 或 `128`

注意：不调用 `listen`，`accept` 无法工作。

```cpp
int accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen);
```

作用：从等待队列中取出一个客户端连接，返回一个新的 fd 用于和该客户端通信。

参数：

1. `sockfd`：监听 socket 的 fd
2. `addr`：用来接收客户端的地址，不需要就填 `NULL`
3. `addrlen`：地址长度，不需要就填 `NULL`

返回：新的 fd（称为 `cfd`），失败返回 `-1`。



```cpp
int connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen);
```

作用：客户端主动连接服务端。

参数：

1. `sockfd`：`socket()` 返回的 fd
2. `addr`：服务端地址（文件路径）
3. `addrlen`：地址结构体大小

读写数据：

```cpp
ssize_t write(int fd, const void *buf, size_t count);
ssize_t read(int fd, void *buf, size_t count);
```

```cpp
int close(int fd);//关闭 fd，断开连接。
```

```cpp
int unlink(const char *pathname); //删除文件系统里的 socket 文件
```





