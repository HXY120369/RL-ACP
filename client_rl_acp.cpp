//`g++ -std=c++11 client.cpp -o client` use that to compile this file
//`./client localhost 526 500 2` to run it
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
#include <ctime>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>  
#include <cstdlib>
#include <mutex>
#include <chrono>
#include <thread>
#include <vector>
#include <map>
#include <random>  // 用于实现 Epsilon-greedy

#define MAXBUFLEN 2048 //Ok?
#define ITERATIONS 500000

using namespace std;
mutex mtx;

//Variable Declarations
const char* SERVERPORT= "49050";
const double ageAlpha = 0.2;
const int array_size = 500000;
const int age_size = 500000;
const int local_size = 10000;
struct timespec *tx,*rx,start_time;
time_t *systemTime;
time_t RTT = 0.0;

static uint32_t transmission_seq,receiver_seq;
int lastSeqNum = 0;
int outOfOrderPackets = 0;
double interRate = 50; //this is lambda

static double age_estimate[age_size] = {0};
static double lambda[age_size] = {0};

static time_t controlPacketDelay[age_size] = {0};
static time_t controlDepartureTime[age_size] = {0};
time_t currentControlTime = 0;
static int controlIndex = 0;

static time_t controlTime = 0;
static time_t depTime = 0;
static time_t depTime_control= 0;
static time_t prevReceiveTime = 0;
static double prevAverageBacklog = 0;
static double currentAverageBacklog = 0;
static double calcLambda = 0;
static double n=0;

static double globalBacklog[age_size] = {0};
static time_t backlogTime[age_size] = {0};
static int backlogIndex = 0;
static double prev_age_estimate = 0;
static double recent_age_estimate = 0;

static time_t received_delay[array_size];
int update_index = 1;
int lambda_index = 1;
bool controlTaken = false;
static double alpha = 0.2;

time_t controlPacketDelay_local[local_size]={0};
time_t controlDepartureTime_local[local_size]={0};
time_t backlogTime_local[local_size] = {0};
double globalBacklog_local[local_size] = {0};

// Q-Learning Hyperparameters
const double ALPHA = 0.1;     // learning rate，
const double GAMMA = 0.95;     // discount factor
double epsilon;               // 探索率
bool is_training_mode = true; // 默认为训练模式
const std::vector<double> ACTIONS = {-1.0, -0.5, -0.25, -0.1, 0.0, 0.1, 0.25, 0.5, 1.0};
const int actions_num = ACTIONS.size();
const int states_num = 81;
// epsilon衰减参数
double initial_epsilon = 0.9;
double min_epsilon = 0.1;
double epsilon_decay = 0.002;
static int training_steps = 0;
static int last_state_index = -1;   // 用于跟踪上一个状态
static int last_action_index = -1;  // 用于跟踪上一个动作
static time_t min_RTT_history = 99999999999;

// 创建 Q-Table
std::vector<std::vector<double>> q_table(states_num, std::vector<double>(actions_num, -100.0));
void save_q_table(const std::string& filename = "q_table.txt"){
    std::ofstream file(filename);
    if(file.is_open()){
      for(int i=0;i<states_num;i++){
        for(int j=0;j<actions_num;j++)
           file << q_table[i][j] << (j == actions_num-1 ? "" : " "); 
        file << std::endl;
      }
    }
}
void load_q_table(const std::string& filename = "q_table.txt"){
    std::ifstream file(filename);
    if(file.is_open()){
      for(int i=0;i<states_num;i++)
        for(int j=0;j<actions_num;j++)
          file >> q_table[i][j];
      std::cout << "Q-table loaded successfully from" << filename << '\n';
    }
}

// 将连续的状态转换为离散的状态索引 (0-80)
int get_state_index(double delta_aoi, double delta_backlog, double current_backlog, time_t rtt_ns) {
    int aoi_state, delta_backlog_state, current_backlog_state, rtt_state; // rate_state;

    if (rtt_ns > 0){
      if (rtt_ns < min_RTT_history){
        min_RTT_history = rtt_ns;
      }
    }
    double baseline_rtt = (min_RTT_history > 1000) ? (double)min_RTT_history : 100.0*1e6;

    // 1. ΔAoI 离散化
    double aoi_threshold = 5.0 * 1e6;
    if (delta_aoi < -aoi_threshold) aoi_state = 0; // 明显改善
    else if (delta_aoi > aoi_threshold) aoi_state = 2; // 明显恶化
    else aoi_state = 1; // 稳定

    // 2. ΔBacklog 离散化
    if (delta_backlog < -0.5) delta_backlog_state = 0; // 减少
    else if (delta_backlog > 0.5) delta_backlog_state = 2; // 增加
    else delta_backlog_state = 1; // 稳定

    // 3. backlog 离散化
    if (current_backlog < 0.5) current_backlog_state = 0; // 低
    else if (current_backlog < 1.5) current_backlog_state = 1; // 中
    else current_backlog_state = 2; // 高

    // 4. RTT 离散化 (单位: ns)ƒ
    double rtt_ratio = (double)rtt_ns / baseline_rtt;
    if (rtt_ratio < 1.2) rtt_state = 0; // 低 
    else if (rtt_ratio < 1.8) rtt_state = 1; // 
    else rtt_state = 2; // 高

    // 将4个离散状态组合成一个唯一的索引
    return aoi_state * 27 + delta_backlog_state * 9 + current_backlog_state * 3 + rtt_state;
}

//Log files opening these files in overwite(out) append state is app
ofstream fileOOOLog("Log_rlacp/Out_of_Order_log.txt", ios::out);
ofstream fileTxLog("Log_rlacp/Tx_log.txt", ios::out);
ofstream fileArrLog("Log_rlacp/Arrival_log.txt", ios::out);
ofstream fileYLog("Log_rlacp/Y_log.txt", ios::out);
ofstream fileAgeEst("Log_rlacp/Age_Est.txt", ios::out);
ofstream filecontrol("Log_rlacp/control_log.txt", ios::out);
ofstream filebacklogArr("Log_rlacp/backlog_arrival.txt", ios::out);
ofstream fileAvgbacklog("Log_rlacp/Avg_backlog.txt", ios::out);
ofstream fileDebugAge("Log_rlacp/debug_age.txt", ios::out);

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
time_t getDoubleTimeDiff(struct timespec *start, struct timespec *stop){
  struct timespec result_spec;
  time_t result;
  timespec_diff(start,stop,&result_spec);
  result = result_spec.tv_sec * 1e9 + ((result_spec.tv_nsec));
  return result;
}
time_t getDoubleTimeNow(){
  struct timespec temp_time, result_time;
  time_t result;
  if(clock_gettime(CLOCK_REALTIME,&temp_time)==-1){
    printf("getDoubleTimeNow ERROR\n");
    exit(1);
  }
  timespec_diff(&start_time, &temp_time, &result_time);
  result = result_time.tv_sec * 1e9 + ((result_time.tv_nsec));
  return result;
}

static void updateInterRate(time_t RTT_local) {
    // 速率变化限制器，确保新发送的速率不会比上一个周期的速率变化得过于剧烈
    if (calcLambda < 0.75 * lambda[lambda_index - 1])      lambda[lambda_index] = 0.75 * lambda[lambda_index - 1];   
    else if (calcLambda > 1.25 * lambda[lambda_index - 1]) lambda[lambda_index] = 1.25 * lambda[lambda_index - 1];
    else lambda[lambda_index] = calcLambda;

    // 将新速率应用到全局变量interRate，main线程会依据这个变量来决定下一次发送数据包的等待时间
    mtx.lock(); 
    interRate = lambda[lambda_index];
    mtx.unlock();
    fileArrLog << getDoubleTimeNow() << "\t" << interRate << "\t" << calcLambda << '\n';
    lambda_index++;
    if (lambda_index > age_size) {
        lambda[0] = lambda[lambda_index - 1];
        lambda_index = 1;
    }
}

static void getBacklogArrival(time_t sampleTime) {
    int backlog;
    backlog = transmission_seq - receiver_seq; // 计算瞬时的网络积压
    mtx.lock(); 	//we take mutex before handling global arrays
    globalBacklog[backlogIndex] = backlog;    // 记录瞬时积压数据
    backlogTime[backlogIndex] = sampleTime;   // 记录此时的时间戳
    backlogIndex++;
    mtx.unlock();	//done with global edits
    filebacklogArr << getDoubleTimeNow() << "\t" << globalBacklog[backlogIndex - 1] << "\t" << backlogTime[backlogIndex - 1] << "\t" << backlogIndex << '\n';
}

// 计算一个控制周期内的时间加权平均积压 B_k
static void getAverageBacklog(time_t backlogTime_local[], double globalBacklog_local[], int backlogIndex_local, time_t currentBacklogTime) {
    double sumBacklog = 0;
    time_t sumTime = 0;
    // if (backlogIndex_local == 1) printf("NO MORE PACKETS TO SEND:BACKLOG\n");
    if (backlogIndex_local >= 2) {  //calculate area code
        for (int i = 0; i < backlogIndex_local; i++) {
          sumBacklog = sumBacklog + globalBacklog_local[i]*(backlogTime_local[i + 1] - backlogTime_local[i]) * 1e-9;
        }
        sumTime = backlogTime_local[backlogIndex_local] - backlogTime_local[0];
        currentAverageBacklog = sumBacklog / (1e-9 * sumTime);
    } else currentAverageBacklog = globalBacklog_local[backlogIndex_local - 1];
    fileAvgbacklog << getDoubleTimeNow() << "\t" << sumBacklog << "\t" << currentAverageBacklog << "\t" << sumTime << "\t" << currentBacklogTime << "\t" << backlogIndex_local << "\t" << '\n';
}

// 计算一个控制周期内的时间加权平均信息年龄 delta_k
static void getAgeEstimate(time_t controlPacketDelay_local[], time_t controlDepartureTime_local[], int controlIndex_local, time_t currentControlTime_local, time_t prevReceiveTime_local, time_t lastDelay_local) {
    double ageEstimate = 0;
    time_t sum_denominator = 0;

    if (controlIndex_local > 1) { 
      // 将整个周期的 AoI 曲线分割成一个个小的梯形，然后分别计算每个梯形的面积并累加
      for (int i = 0; i < controlIndex_local; i++) {
        ageEstimate = ageEstimate + (controlPacketDelay_local[i] * controlDepartureTime_local[i + 1]) + (0.5 * controlDepartureTime_local[i + 1] * controlDepartureTime_local[i + 1]);
        fileDebugAge << getDoubleTimeNow() << "\t" << ageEstimate << "\t" <<  controlPacketDelay_local[i] << "\t"<< controlDepartureTime_local[i+1] << "\t" << '\n';
      }
      // 计算总时长
      for (int i = 1; i <= controlIndex_local; i++) sum_denominator = sum_denominator + controlDepartureTime_local[i];
      recent_age_estimate = ageEstimate / sum_denominator;  // 计算平均年龄
    } else recent_age_estimate = lastDelay_local;

    age_estimate[update_index] = recent_age_estimate; //age_estimate is a global array
    update_index++;
    fileAgeEst << getDoubleTimeNow() << "\t" << age_estimate[update_index-1] << "\t" << ageEstimate << "\t" << sum_denominator << "\t" <<  controlPacketDelay_local[controlIndex_local-1] << "\t"<< controlDepartureTime_local[controlIndex_local-1] << "\t" << "\n";
    if (update_index > age_size) { //handling array overflow
        age_estimate[0] = age_estimate[update_index - 1];
        update_index = 1;
    }
}

// 基于强化学习（q_learning）的控制方法
static void *rlcontrolAction(void* x1) {
  while(true){
    time_t prevControlTime= currentControlTime;
    currentControlTime = getDoubleTimeNow();   
    time_t scheduleTime;
    mtx.lock(); 		//taking mutex lock here
    time_t RTT_local = RTT;
    time_t depTime_local = depTime;
    time_t prevReceiveTime_local = prevReceiveTime;

    int controlIndex_local = controlIndex;
    int backlogIndex_local = backlogIndex;
  
    //*** Taking Snapshot for Age calculation ***//
    for(int i=0; i<controlIndex; i++) controlPacketDelay_local[i]=controlPacketDelay[i];
    for(int i=0; i<controlIndex; i++) controlDepartureTime_local[i]=controlDepartureTime[i];
    //*** Taking Snapshot for Average backlog function ***//
    for(int i=0; i<backlogIndex; i++) backlogTime_local[i] = backlogTime[i];
    for(int i=0; i<backlogIndex; i++) globalBacklog_local[i] = globalBacklog[i];

    //@tanyas: added last delay part in control action
    time_t lastDelay = controlPacketDelay_local[controlIndex - 1];
    time_t lastDepartureTime = currentControlTime - prevReceiveTime_local;
    time_t calculatedDelay = lastDelay + lastDepartureTime;
    controlPacketDelay_local[controlIndex] = calculatedDelay;
    controlDepartureTime_local[controlIndex] = lastDepartureTime;
  
    controlDepartureTime[0] = 0;
    controlPacketDelay[0] = calculatedDelay;
    controlIndex = 1; //set index value to size of controlTime array
    // flush of control array complete
    //@tanyas: Added Backlog handle and flush part
    time_t currentBacklogTime = getDoubleTimeNow();
    backlogTime_local[backlogIndex] = currentBacklogTime; //make a copy to both local and global server
    backlogTime[backlogIndex] = currentBacklogTime;
    globalBacklog[0] = globalBacklog[backlogIndex-1];
    backlogTime[0] = backlogTime[backlogIndex];
    backlogIndex = 1;
    mtx.unlock(); //unlocking mutex here

    if(controlIndex_local==1){
      depTime_control= currentControlTime-prevReceiveTime_local;
      time_t oldDepTime_control = depTime;
      depTime = (1 - alpha) * oldDepTime_control + alpha*depTime_control;   
    }
    getAgeEstimate(controlPacketDelay_local, controlDepartureTime_local, controlIndex_local, currentControlTime, prevReceiveTime_local, lastDelay);
    getAverageBacklog(backlogTime_local, globalBacklog_local, backlogIndex_local, currentBacklogTime);

    // Q-Learning 核心逻辑
    // 1. 获取当前状态的离散索引
    double delta_aoi = recent_age_estimate - prev_age_estimate;
    double delta_backlog = currentAverageBacklog - prevAverageBacklog;
    double current_rate = lambda[lambda_index > 0 ? lambda_index-1 : 0];
    int current_state_index = get_state_index(delta_aoi, delta_backlog, currentAverageBacklog, RTT_local);

    // 奖励计算
    double age_in_seconds = recent_age_estimate / 1e9;
    double w_age = 10;
    double w_backlog = 2;
    double penalty = w_age * age_in_seconds + w_backlog * currentAverageBacklog;
    if (currentAverageBacklog > 1.0)
      penalty += 2.0 * (currentAverageBacklog * currentAverageBacklog);
    if (current_rate < 10.0)
      penalty += 2.0 * (10.0 - current_rate);
    if (current_rate > 20.0)
      penalty += 1.0 * (current_rate - 60.0);
    if (current_rate < 1.0)
      penalty -= 0.2 * current_rate;
    
    double reward = - penalty;

    // 2. 计算上一个动作的奖励（reward），奖励必须在上一个动作之行后，观察到当前状态时才能计算
    if(last_state_index != -1 && is_training_mode){
      // 3. 更新Q-Table, 只有在训练模式下才执行Q-Table的更新
      // 找到当前状态下能获得的最大Q值
      double max_q_next_state = *std::max_element(q_table[current_state_index].begin(), q_table[current_state_index].end());
      // Q-Learning 更新公式
      double old_q_value = q_table[last_state_index][last_action_index];
      double new_q_value = old_q_value + ALPHA * (reward + GAMMA * max_q_next_state - old_q_value);
      q_table[last_state_index][last_action_index] = new_q_value;
    }

    // 4. 选择并执行新动作（Epsilon-Greedy）
    int action_index;
    if((double)rand() / RAND_MAX < epsilon){  // Epsilon-greedy 策略
      action_index = rand() % actions_num;    // 探索：随机选择一个动作
    } else {
        // --- 利用：寻找最优动作（改进版）---
        // 1. 首先找到当前状态下的最大Q值
        double max_q_value = -std::numeric_limits<double>::infinity();
        std::vector<int> best_actions;
        const double t = 1e-6;

        for (int i=0;i<actions_num; i++){
          double q_val = q_table[current_state_index][i];
          if (q_val > max_q_value + t){
            max_q_value = q_val;
            best_actions.clear();
            best_actions.push_back(i);
          }
          else if (std::abs(q_val - max_q_value) < t){
            best_actions.push_back(i);
          }
        }

        // 从所有最优动作中随机选择一个
        if (!best_actions.empty())
          action_index = best_actions[rand() % best_actions.size()];
        else
          action_index = 0;
    }

    double desiredChangeinBacklog = ACTIONS[action_index]; 
    calcLambda = (1e9 / depTime_local) + (1e9 * desiredChangeinBacklog / RTT_local);

    // 5. 记录当前状态和动作用于下一次更新
    last_state_index = current_state_index;
    last_action_index = action_index;

    filecontrol << getDoubleTimeNow() << "\t" << recent_age_estimate << "\t" << prev_age_estimate << "\t" << delta_backlog << "\t" << currentControlTime << "\t" << depTime_local << "\t" << controlIndex << "\t" << "\n";

    prev_age_estimate = recent_age_estimate;
    controlTime = currentControlTime;
    prevAverageBacklog = currentAverageBacklog;
    controlTaken = true;
    if (calcLambda < 5.0) calcLambda = 5.0;

    updateInterRate(RTT_local);
    // printf("ControlAction Block RTT_local  %lu\n", RTT_local);
    // printf("ControlAction Block depTime_local  %lu\n", depTime_local);

    // 定期保存 Q-Table（只有在训练模式下才保存）
    if(is_training_mode){
      static int counter = 0;
      if(++counter % 100 == 0) { //每100个控制周期保存一次
        save_q_table();
        printf(">> [Auto-Save] Steps: %d, Current Epsilon: %.4f\n", training_steps, epsilon);
      }

      // 探索率不断衰减
      training_steps++;

      double current_decay = exp(-epsilon_decay * training_steps);
      epsilon = min_epsilon + (initial_epsilon - min_epsilon) * current_decay;
    }
    scheduleTime = (10e9 / interRate);
    // printf("ControlAction Block scheduleTime %lu\n", scheduleTime);
    this_thread::sleep_for(chrono::nanoseconds(scheduleTime));
  }
  return NULL;
}

unsigned char *package(unsigned int seq) {
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
unsigned int unpack(unsigned char buffer[]) {
    unsigned int l1,l2,l3,l4;
    l1 = (unsigned int)buffer[0];
    l2 = (unsigned int)buffer[1];
    l3 = (unsigned int)buffer[2];
    l4 = (unsigned int)buffer[3];
    unsigned int sq = (l1<<24) + (l2<<16) + (l3<<8) + l4;
    return sq;
}

void *Receiver(void *sock) {
  int sockfd = *(int *)sock;
  struct addrinfo hints, *servinfo, *p;
  int rv, numbytes;
  struct sockaddr_storage their_addr;
  unsigned char buf[MAXBUFLEN];
  socklen_t addr_len;
  
  while(1){
    // 1.接收从服务端发送回来的数据包，即ACK
    memset(buf,0,sizeof(buf));
    addr_len = sizeof their_addr;
    if ((numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1 , 0, (struct sockaddr *)&their_addr, &addr_len)) == -1) {
      perror("recvfrom");
      exit(1);
    }
    receiver_seq = unpack(buf);
    getBacklogArrival(getDoubleTimeNow());
    time_t currentReceiveTime = getDoubleTimeNow();

    // 2.检测收到的ACK是否乱序
    if(receiver_seq<lastSeqNum) outOfOrderPackets += 1;
    else  lastSeqNum = receiver_seq;
    fileOOOLog << getDoubleTimeNow() << "\t" << outOfOrderPackets << "\n";

    // 3.精确计算RTT
    if(clock_gettime(CLOCK_REALTIME,&rx[receiver_seq])==-1) // 获取并记录当前ACK包到达的精确接收时间，存储在全局数据rx中
      printf("error\n");
    printf("The receiver_seq is %u\n", receiver_seq);
    // 计算瞬时RTT，用刚记录的接受时间rx减去住县城之前为该序列号记录的发送时间tx
    systemTime[receiver_seq] = getDoubleTimeDiff(&tx[receiver_seq], &rx[receiver_seq]);
    received_delay[receiver_seq] = systemTime[receiver_seq];
    
    // 4.更新平滑平均RTT（EWMA）
    if(receiver_seq == 1){  // 对于第一个ACK，直接将其瞬时RTT作为全局平均RTT的初始值
      mtx.lock();
      RTT = systemTime[receiver_seq];
      mtx.unlock();
      printf("FIRST RTT IS %lu\n",RTT);
      pthread_t thread_control;
      int cn;
      cn = pthread_create(&thread_control, NULL, rlcontrolAction, NULL);
      if(cn==-1){
        perror("rlcontrolAction Thread gone\n");
        exit(1);
      }
    }
    else{  // 后续RTT都是用EWMA来计算全局平均RTT
      mtx.lock();
      RTT = (1.0 - ageAlpha)*RTT + ageAlpha*systemTime[receiver_seq];
      mtx.unlock();
    }

    // 5.计算ACK到达间隔（Departure Time）
    time_t departureTime;
    mtx.lock();
    if(receiver_seq == 1){
      departureTime = currentReceiveTime;
      depTime = departureTime;
    } else {
      departureTime = currentReceiveTime - prevReceiveTime;
      time_t oldDepTime = depTime;
      depTime = (1 - alpha) * oldDepTime + alpha*departureTime;
    }
    prevReceiveTime = currentReceiveTime;
    // For updated control action code
    if (controlTaken == true) {
      controlDepartureTime[controlIndex] = currentReceiveTime - currentControlTime;
      controlTaken = false;
    } else controlDepartureTime[controlIndex] = departureTime;
    if(receiver_seq > 0)  controlPacketDelay[controlIndex] = received_delay[receiver_seq - 1]; 
    else                  controlPacketDelay[controlIndex] = 0;
    controlIndex++;     //@tanyas: Unlike simulator, where function call and calculation is instantaneous, things arent same in real. 
                        // New packets are arriving while control action code is being executed leading to increments in controlIndex.
    mtx.unlock();		//done with mutex for control
    fileYLog << getDoubleTimeNow() << "\t" << RTT << "\t" << depTime << "\t" << controlPacketDelay[controlIndex - 1] << "\t" << controlDepartureTime[controlIndex - 1] << "\t" << controlIndex << "\t" << currentReceiveTime << "\t" << currentControlTime << "\t" << "\n";
  }
}

time_t ping_to_get_rtt(int sockfd,struct addrinfo *p){
  time_t rtt_total = 0;
  int seqNum;
  for(seqNum = 0; seqNum < 10; seqNum++){
    unsigned char* packet = package((unsigned)seqNum); // 使用packet函数将当前的序列号seqNum打包成一个字节流
    time_t send_time = getDoubleTimeNow();
    // 将打包好的数据包通过 sendto 发送给服务器
    if (sendto(sockfd, packet, 64, 0, p->ai_addr, p->ai_addrlen) == -1) {
      perror("talker1: sendto");
      exit(1);
    }

    struct sockaddr_storage their_addr;
    socklen_t addr_len;
    unsigned char buf[MAXBUFLEN];
    memset(buf,0,sizeof(buf));
    addr_len = sizeof their_addr;
    // recvfrom方法是一个阻塞式调用方法，程序会在这里暂停，等待服务器的回应（ACK）
    if (recvfrom(sockfd, buf, MAXBUFLEN-1 , 0, (struct sockaddr *)&their_addr, &addr_len) == -1) {
      perror("recvfrom");
      exit(1);
    }
    time_t recv_time = getDoubleTimeNow();  // 一旦收到ACK,使用此方法获取接收时间戳 recv_time

    receiver_seq = unpack(buf);  // 从收到的ACK中解析出序列号
    if(receiver_seq!=seqNum){    // 检查收到的ACK序列号是否与刚刚发送的序列号匹配
      printf("I wanted packet %d BUT I got %d", seqNum, receiver_seq);
      exit(1);
    }
    rtt_total+=(recv_time-send_time);
  }
  return rtt_total/seqNum;//avg RTT
}

int main(int argc, char *argv[]){

  std::ifstream q_file("q_table.txt");
  if (q_file.good()) {
    q_file.close();
    load_q_table();
  } else {
    std::cout << "q_table.txt not found, starting with a new table." << std::endl;
  }

  // 获取一个高精度的程序启动时间戳 start_time, 它将作为后续所有相对时间测量的基准
  if(clock_gettime(CLOCK_REALTIME,&start_time)==-1) printf("error\n");
   
  int sockfd;
  struct addrinfo hints, *servinfo, *p;
  int rv;
  int numbytes;
  tx = (struct timespec *)malloc(ITERATIONS*sizeof(struct timespec)); //array of transmission+recieving times
  rx = (struct timespec *)malloc(ITERATIONS*sizeof(struct timespec));
  systemTime = (time_t *)malloc(ITERATIONS*sizeof(time_t));

  if (argc != 6) { // 在Usage信息中加上 mode 参数
    fprintf(stderr,"Usage: talker hostname message_bytes num_messages inter_arrival_time port\n");
    exit(1);
  }
  SERVERPORT = argv[4];
  std::string mode = argv[5];
  if(mode == "test"){
    is_training_mode = false;
    epsilon = 0.0;
    std::cout << "******** RUNNING IN TEST MODE ********" << "\n";
  } else if(mode == "train"){
    is_training_mode = true;
    epsilon = initial_epsilon;
    std::cout << "******** RUNNING IN TRAINING MODE ********" << "\n";
  } else {
    fprintf(stderr, "Error: Invalid mode '%s'. Choose 'train' or 'test'.\n", mode.c_str());
    exit(1);
  }

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;

  if ((rv = getaddrinfo(argv[1], SERVERPORT, &hints, &servinfo)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    return 1;
  }
  // loop through all the results and make a socket
  for(p = servinfo; p != NULL; p = p->ai_next) {
    if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      perror("talker: socket");
      continue;
    }
    break;
  }
  if (p == NULL) {
    fprintf(stderr, "talker: failed to create socket\n");
    return 2;
  }

  // 调用 ping_to_get_rtt 方法获取初试 RTT
  printf("Before ping\n");
  interRate = 1e9/ping_to_get_rtt(sockfd, p);
  printf("%f\n", 1/interRate);
  printf("After ping\n");
  lambda[0] = interRate;

  // 创建 Receive 线程，该线程会独立持续地监听和处理从服务器返回的ACK包
  pthread_t thread_rec;
  int rc;
  /************* Receiver Initialization begins *****************/
  rc = pthread_create(&thread_rec, NULL, Receiver, &sockfd);
  if(rc==-1){
    perror("Receiver Thread gone\n");
    exit(1);
  }
  /************* Receiver Initialization ends *****************/

  static int numbytes_new = numbytes;
  for(int seqNum=1; seqNum<=stoi(argv[3]); seqNum++){ //seqNum is m_packetsSent
    mtx.lock();
    time_t arrivalTime = 1e9/interRate;
    mtx.unlock();
    this_thread::sleep_for(chrono::nanoseconds(arrivalTime));

    // 打包数据并发送至服务端
    unsigned char* packet = package((unsigned)seqNum);
    cout << "sending a packet seq:"<<(unsigned)seqNum<<" of length "<<sizeof(packet)/2<<"\n";
    if(stoi(argv[2])<4){
      printf("Too less bytes. I need minimum 4"); 
      exit(1);
    }
    if(clock_gettime(CLOCK_REALTIME,&tx[seqNum])==-1) printf("error\n");
    cout << "send to file descriptor" << sockfd << ". The current sending rate is "<< interRate << endl;
    if ((numbytes_new = sendto(sockfd, packet, stoi(argv[2]), 0, p->ai_addr, p->ai_addrlen)) == -1) {
      perror("talker2: sendto"); 
      exit(1);
    }
    free(packet);

    // 更新状态
    transmission_seq = seqNum;
    fileTxLog << seqNum << '\t' << getDoubleTimeNow() << '\n'; //Logging
    getBacklogArrival(getDoubleTimeNow());
  }

  freeaddrinfo(servinfo);
  if(is_training_mode) save_q_table();
  return 0;
}