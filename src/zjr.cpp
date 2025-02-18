#include <vector>
#include<iostream>
using namespace std;
int n = 0; //设备数量
vector<int> result;

vector<int> CalcStartTime(double max_power, const vector<double>& power, const vector<double>& cost) {//同时进行的最大功率， 每个设备的功率，每个小时的钱
    n = power.size();
    result = vector<int>(n, -1);
    if(cost.size() != 24) cout << "cost num error" << endl;
    vector<double> running_cost(cost.size(), 0);
    for(int i = 0; i < 24; ++i) {
        for(int j = 0; j < 6; ++j) {
            running_cost[i] += cost[(i + j) % 6];
        }
    }

    //循环

}