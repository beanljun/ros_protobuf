#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/epoll.h>
#include <errno.h>
#include <ctype.h>

#define MAXLINE 8192
#define SERV_PORT 8000
#define OPEN_MAX 1000
int main(int argc, char *argv[]) {
    // 记录是第几个连接上来的客户端
    int num = 0;
    
    // 创建监听的套接字
    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    
    // 端口复用
    // 告诉操作系统允许在同一个端口上启动一个新的监听套接字，即使之前的套接字还未完全关闭
    int opt = 1;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    struct sockaddr_in servaddr;                            // 用于存储IP地址和端口号
    bzero(&servaddr, sizeof(servaddr));                // 将servaddr结构体的所有字节初始化为0
    servaddr.sin_family = AF_INET;                          // 使用IPv4网络协议
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY); // 设置IP地址，INADDR_ANY表示可以接受任意IP地址的客户端连接
    servaddr.sin_port = htons(SERV_PORT);        // 设置端口号，宏定义为8000
    // 绑定, 将创建的套接字（listenfd）绑定到指定的IP地址和端口号上
    bind(listenfd, (struct sockaddr *) &servaddr, sizeof(servaddr));
    
    // 监听, 设置内核监听队列的长度为20
    listen(listenfd, 20);
    
    //创建epoll模型, efd指向红黑树根节点,OPEN_MAX为红黑树能容纳的节点数，用于初始化红黑树容量，实际可以超过OPEN_MAX
    int efd = epoll_create(OPEN_MAX);
    if(efd == -1) {
        perror("epoll_create error");
        exit(1);
    }
    

    struct epoll_event tep;
    // 设置 tep 的 events 字段为 EPOLLIN “可读”事件，
    // listenfd 关联的文件描述符变得可读时（例如，一个网络套接字有新的连接请求），epoll 会通知应用程序。
    tep.events = EPOLLIN;
    // 设置事件关联的文件描述符（listenfd）
    tep.data.fd = listenfd;
    //将lfd及对应的结构体设置到树上,efd可找到该树
    int res = epoll_ctl(efd, EPOLL_CTL_ADD, listenfd, &tep);
    if(res == -1) {
        perror("epoll_ctl error");
        exit(1);
    }
    
    socklen_t clilen;                           // 用于存储地址的长度，在调用如accept等函数时使用
    struct sockaddr_in cliaddr;                 // 用于存储客户端的地址信息
    // ep[] : epoll_wait参数
    struct epoll_event ep[OPEN_MAX];            // 用于存储epoll_wait返回的就绪事件
    char buf[MAXLINE], str[INET_ADDRSTRLEN];    // buf用于存储接收和发送的数据，str用于存储网络地址的字符串形式
    
    while(1) {
    
        // 等待文件描述符上的事件发生, 返回就绪的文件描述符数目，-1表示阻塞到有事件发生为止
        int nready = epoll_wait(efd, ep, OPEN_MAX, -1);
        if(nready == -1) {
            perror("epoll_wait error");
            exit(1);
        }
        
        for (int i = 0; i < nready; i++) {
            //如果不是"读"事件, 跳过，下一个
            if (!(ep[i].events & EPOLLIN)) {
                continue;
            }
            
            //判断满足事件的fd是不是lfd
            if (ep[i].data.fd == listenfd) {
                clilen = sizeof(cliaddr);
                // 接受连接请求
                int connfd = accept(listenfd, (struct sockaddr *)&cliaddr, &clilen);
                
                printf("received from %s at PORT %d\n",
                       inet_ntop(AF_INET, &cliaddr.sin_addr, str, sizeof(str)),
                       ntohs(cliaddr.sin_port));
                printf("cfd %d---client %d\n", connfd, ++num);
                
                tep.events = EPOLLIN;
                tep.data.fd = connfd;
                res = epoll_ctl(efd, EPOLL_CTL_ADD, connfd, &tep);
                if(res == -1) {
                    perror("epoll_ctl error");
                    exit(1);
                }
            } else { // 不是监听的文件描述符, 通信的fd
                int sockfd = ep[i].data.fd;
                int n = read(sockfd, buf, MAXLINE);
                
                //读到0,说明客户端关闭链接
                if (n == 0) {
                    //将该文件描述符从红黑树摘除
                    res = epoll_ctl(efd, EPOLL_CTL_DEL, sockfd, NULL);
                    if(res == -1) {
                        perror("epoll_ctl error");
                        exit(1);
                    }
                    //关闭与该客户端的链接
                    close(sockfd);
                    printf("client[%d] closed connection\n", sockfd);
                } else if (n < 0) {            //出错
                    perror("read n < 0 error: ");
                    res = epoll_ctl(efd, EPOLL_CTL_DEL, sockfd, NULL);
                    close(sockfd);
                } else  {      // 实际读到了字节数
                    for (int j = 0; j < n; j++) {
                        buf[j] = toupper(buf[j]);   //转大写,写回给客户端
                    }
                    write(STDOUT_FILENO, buf, n); //发送到终端, 也可以打印出来
                    write(sockfd, buf, n); //发送给客户端
                }
            }
        }
    }
    close(listenfd);
    close(efd);
    
    return 0;
}