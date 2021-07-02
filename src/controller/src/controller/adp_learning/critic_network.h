#ifndef _CRITIC_NETWORK_H_
#define _CRITIC_NETWORK_H_
#include <cmath>
#include<iostream>
#include <fstream>
#include <Eigen/Dense>

class critic_network {
 private:
    double learning_rate = 0.01 ; //learning rate
    double reward_count = 0.95; //the reward count 
    Eigen::MatrixXd parameter_layer1;
    Eigen::MatrixXd parameter_layer2;
    Eigen::MatrixXd input_variable;
    Eigen::MatrixXd hidden_layer;
    Eigen::MatrixXd p_hidden_layer;
    double output;
    double reference_signal;
    int hidden_number;
    static double active_function(double input) 
    {
        // 1 - exp( - qi(t) ) / 1 + exp( - qi(t) ) )
        double ret = std::exp(- input);
        double tmp = (2)/(1 + ret) - 1;
        return tmp;
    }
 public:
    static critic_network& Get_critic_network();
    critic_network() ;
    double return_J_cost();
    double return_reference();
    Eigen::MatrixXd return_parameter_layer1();
    Eigen::MatrixXd return_parameter_layer2();
    void init();

    double output_J_cost(double , double ,double );
    void update_weight(double , double , double , double , double );
};

#endif 