# RL-ACP+ Introduction
---
Age Control Protocol provides a practical transport layer solution for real-time status update applications by dynamically adapting update rates through end-to-end feedback. ACP operates over UDP to avoid the head-of-line blocking associated with TCP.

ACP was published in IEEE WoWMoM 2019. ACP+ was published at IEEE INFOCOM AOI Workshop 2021. An upgraded version of ACP+ was published at IEEE/ACM Transactions on Networking 2024.
* T. Shreedhar, S. K. Kaul and R. D. Yates, "An Age Control Transport Protocol for Delivering Fresh Updates in the Internet-of-Things," 2019 IEEE 20th International Symposium on "A World of Wireless, Mobile and Multimedia Networks" (WoWMoM), 2019, pp. 1-7.
* T. Shreedhar, S. K. Kaul and R. D. Yates, "An Empirical Study of Ageing in the Cloud," IEEE INFOCOM 2021 - IEEE Conference on Computer Communications Workshops (INFOCOM WKSHPS).
* T. Shreedhar, S. K. Kaul, and R. D. Yates, “Acp+: An age control protocol for the internet,” IEEE/ACM Transactions on Networking, vol. 32, pp. 3253–3268, August 2024

RL-ACP+ is a reinforcement learning approach for control optimization in Age Control Protocol. We use a Reinforcement Learning agent to replace the fixed control module in ACP+. The RL agent possesses autonomous learning capabilities, enabling it to learn the complex mapping from network states to optimal actions through continuous interaction with network environments.

# The setup of server_delay.cpp
---
This file implements a single-hop Client-Server architecture built upon UDP. The server here introduces an artificial processing delay based on a predefined model before echoing an ACK back to the client  

# How to run ACP+ and RL-ACP+
---
1. Create executables of C++ source code for client_acp.cpp, client_rl_acp.cpp and server_delay.cpp
* g++ -std=c++11 client_acp.cpp -o client_acp -lpthread
* g++ -std=c++11 client_rl_acp.cpp -o client_rl_acp -lpthread
* g++ -std=c++11 server_delay.cpp -o server_delay -lpthread
2. Run server_delay on a terminal and pass a port number along with it
* ./server_delay 49050
3. Run client_acp or client_rl_acp on another terminal
* ./client_acp localhost 526 2000 2 100 49050
* ./client_rl_acp localhost 526 20000 49050 train
* ./client_rl_acp localhost 526 2000 49050 test
For client_rl_acp, the last parameter in the command line indicates the current state the algorithm. The parameter 'train' indicates that the system is in training mode, in which the source will send a relatively large number of updates to train a complete Q-table. The parameter 'test' indicates that the system is in testing mode, in which the source will send a certain number of packets to test the performance of the trained Q-table.
