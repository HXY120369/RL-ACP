#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <thread>
#include <chrono>
#include <random>
#include <iostream>
#include <time.h> //for logging
#include <fstream> //for logging

#define MYPORT "49050"
#define MAXBUFLEN 2048 //Ok?

struct timespec currentTime,startTime;

// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa){
  if (sa->sa_family == AF_INET){
    return &(((struct sockaddr_in*)sa)->sin_addr);
  }
  return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

//Blindly imported for time calcs for logging
void timespec_diff(struct timespec *start, struct timespec *stop, struct timespec *result){
    if ((stop->tv_nsec - start->tv_nsec) < 0) {
        result->tv_sec = stop->tv_sec - start->tv_sec - 1;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result->tv_sec = stop->tv_sec - start->tv_sec;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec;
    }
    return;
}
double getDoubleTimeDiff(struct timespec *start, struct timespec *stop){
    struct timespec result_spec;
    double result;
    timespec_diff(start,stop,&result_spec);
    result = (double)result_spec.tv_sec + ((double)(result_spec.tv_nsec))*(1e-9);
    return result;
}
double getDoubleTime(struct timespec yTime){
    double result;
    result = (double)yTime.tv_sec + ((double)(yTime.tv_nsec))*(1e-9);
    return result;
}

unsigned char *package(unsigned int seq){
    unsigned char *data = (unsigned char *)malloc(1024*sizeof(unsigned char));
    unsigned int n1 = seq>>24;
    unsigned int n2 = (seq>>16) & 0xff;
    unsigned int n3 = (seq>>8) & 0xff;
    unsigned int n4 = seq & 0xff;
    data[0] = (unsigned char)n1;
    data[1] = (unsigned char)n2;
    data[2] = (unsigned char)n3;
    data[3] = (unsigned char)n4;
    return data;
}
unsigned int unpack(unsigned char buffer[]){
    unsigned int l1,l2,l3,l4;
    l1 = (unsigned int)buffer[0];
    l2 = (unsigned int)buffer[1];
    l3 = (unsigned int)buffer[2];
    l4 = (unsigned int)buffer[3];
    unsigned int sq = (l1<<24) + (l2<<16) + (l3<<8) + l4;
    return sq;
}

std::ofstream fileServerLog("Log/Server_log.txt", std::ios::out);

// 这个 main 函数实现了一个UDP服务器的完整生命周期：初始化、监听、处理和关闭 
int main(void){
  //Variable Declarations
  int sockfd;     //  socket file descriptor, 创建套接字之后，系统会返回一个整数作为这个套接字的唯一标识符，bind, sendto, recvfrom都会通过这个整数来进行
  struct addrinfo hints, *servinfo, *p;  // 用于处理网络地址信息的3个结构体变量
  int rv;         //  return value, 用来存储 getaddrinfo 函数的返回值，如果 rv 不为0，则表示发生了错误 
  int numbytes;   //  存储接收或发送的数据的字节数
  struct sockaddr_storage their_addr;   //  通用套接字地址存储结构，用来存储客户端的地址信息（包括IP地址和端口号）
  unsigned char buf[MAXBUFLEN];     //  临时存储从网络中接收到的原始数据 
  socklen_t addr_len;
  char s[INET6_ADDRSTRLEN];
  unsigned int seq = 0;   //  存储从客户端数据包中解析出来的序列号

  //Specifying IP type and Data Packet type
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC; // UNSPEC means suitable for IPv4 & IPv6, set to AF_INET to force IPv4
  hints.ai_socktype = SOCK_DGRAM; // 使用 UDP (数据报套接字)协议
  hints.ai_flags = AI_PASSIVE; // use my IP

  // 调用 getaddrinfo 函数来获取用于在本机上创建服务器的地址信息
  if ((rv = getaddrinfo(NULL, MYPORT, &hints, &servinfo)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    return 1;
  }

  // loop through all the results and bind to the first we can
  for(p = servinfo; p != NULL; p = p->ai_next) {
    // 创建套接字
    if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      perror("listener: socket");
      continue;
    }
    // 绑定套接字
    if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
      close(sockfd);
      perror("listener: bind");
      continue;
    }
    break;
  }

  // 循环后的错误检查，表示没有找到可用的套接字
  if (p == NULL) {
    fprintf(stderr, "listener: failed to bind socket\n");
    return 2;
  }

  freeaddrinfo(servinfo);  // 释放 servinfo 链表的内存
  printf("listener: waiting to recvfrom...\n");

  //Assign value to startTime, print error, if CANT
  if(clock_gettime(CLOCK_REALTIME,&startTime)==-1){
    printf("error\n");
  }  
  fileServerLog << "Seq;Time;Timestamp;PacketSize\n";

  // 这个无限循环是服务器程序的主题，用于持续地执行服务器的核心任务：等待客户端数据包->接收->处理->响应
  do{
    memset(buf,0,sizeof(buf));//might not be needed

    // 服务器进行监听并接收客户端的数据
    addr_len = sizeof their_addr;
    if ((numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1 , 0, (struct sockaddr *)&their_addr, &addr_len)) == -1) {
      perror("recvfrom");
      exit(1);
    }

    // 处理与记录数据
    seq = unpack(buf);//get the sequence number as int from packet
    if(clock_gettime(CLOCK_REALTIME,&currentTime)==-1){
      printf("error\n");
    }
    fileServerLog << seq << ";" << getDoubleTimeDiff(&startTime,&currentTime) << ";" << getDoubleTime(currentTime) << ";" << numbytes << "\n";
    printf("listener: Recieved packet containing %d bytes\n", numbytes);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 返回一个ACK数据包以响应客户端
    unsigned char *packet = package(seq);
    if ((numbytes = sendto(sockfd, packet, 20, 0, (struct sockaddr *)&their_addr, addr_len)) == -1) {
      perror("talker: sendto");
      exit(1);
    }
    free(packet);
    //if(getDoubleTimeDiff(&startTime,&currentTime)>=500.0)
      //break;
  } while(true);
  close(sockfd);
  return 0;
}