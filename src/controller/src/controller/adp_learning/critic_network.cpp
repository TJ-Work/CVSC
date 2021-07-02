/******************************************************
    Author : shipeng_liu 
    Email : 1196075299@qq.com
    Description: Adaptive dynamic programming 
******************************************************/
#include <cmath>
#include<iostream>
#include <fstream>
#include <Eigen/Dense>
#include <iomanip>

using namespace std;

/****************************************************************** 
    Critic Network
    Input: S, diff_s, steer_cmd
    number of hidden_layer: 1
    number of active_function: hidden_number
    output: J_cost 
    parameter_layer1: parameters from input to hidden_layer
    parameter_layer2: parameters from hidden_layer to output
******************************************************************/

class critic_network {

  private:

    double learning_rate_basic = 0.1 ; //learning rate
    double reward_count = 0.95; //the reward count 
    Eigen::MatrixXd parameter_layer1;
    Eigen::MatrixXd parameter_layer2;
    Eigen::MatrixXd input_variable;
    Eigen::MatrixXd hidden_layer;
    Eigen::MatrixXd p_hidden_layer;
    double output;
    double reference_signal;
    double last_reference_signal;
    int hidden_number;
    static double active_function(double input) {
        // 1 - exp( - qi(t) ) / 1 + exp( - qi(t) ) )
        double ret = std::exp(- input);
        double tmp = (2)/(1 + ret) - 1;
        return tmp;
    }


  public:
    static critic_network& Get_critic_network() 
    {
        static critic_network singleton;
        return singleton;
    }

    critic_network() 
    {
        hidden_number = 4;
        init();
    }
    double return_J_cost()
    {
        return output;
    }
    double return_reference()
    {
        return reference_signal;
    }
    Eigen::MatrixXd return_parameter_layer1()
    {
        return parameter_layer1;
    }
    Eigen::MatrixXd return_parameter_layer2()
    {
        return parameter_layer2;
    }
    Eigen::MatrixXd return_p_hidden_layer()
    {
        return p_hidden_layer;
    }
    void init()
    {
        parameter_layer1 = Eigen::MatrixXd::Random(3, hidden_number);
        parameter_layer2 = Eigen::MatrixXd::Random(hidden_number, 1);
        
        hidden_layer = Eigen::MatrixXd::Random(1, hidden_number);
        p_hidden_layer = Eigen::MatrixXd::Random(1, hidden_number);
        input_variable = Eigen::MatrixXd::Random(1,3);
        output = 0;

    }

    double output_J_cost(double s, double diff_s,double steer_cmd)
    {
        input_variable(0,0) = s;
        input_variable(0,1) = diff_s;
        input_variable(0,2) = steer_cmd;
        hidden_layer = input_variable * parameter_layer1;
        // limit hidden layer
        
        for (int i = 0; i < hidden_number; i++)
        {
            p_hidden_layer(0,i) = active_function(hidden_layer(0,i));
            // limit the p_Hidden_layer
            if (p_hidden_layer(0,i) < 0.000001 && p_hidden_layer(0,i) > 0 )
                p_hidden_layer(0,i) = 0.000001;
            else if (p_hidden_layer(0,i) > -0.000001 && p_hidden_layer(0,i) < 0)
                p_hidden_layer(0,i) = - 0.000001;
        }
        

        Eigen::MatrixXd test = p_hidden_layer * parameter_layer2;
        output = test(0,0);
        // limit the J_cost
        if (output < 0)
        {
            output = 0;
        }
        
        return output;
    }

    void update_weight(double J_cost, double Last_J_cost, double refer_s)
    {
        // update learning rate:
        last_reference_signal = reference_signal;
        reference_signal =  refer_s;
        //double learning_rate = learning_rate_basic * (1+2/(1+fabs(reference_signal - last_reference_signal)) );
        double learning_rate = learning_rate_basic * 4;
        double ect = reward_count*J_cost -  reference_signal;
        //double reference_signal = 0.5;
        cout << "\ncritic_network_information:\n";
        cout << left << setw(20) << "[ refer: " << right
             << setw(20) << reference_signal << "]" << endl;
        cout << left << setw(20) << "[ J_cost: " << right
             << setw(20) << J_cost << "]" << endl;
        cout << left << setw(20) << "[ ect: " << right
             << setw(20) << ect << "]" << endl;
        cout << "[ p_hidden_layer: " << endl;
        cout << p_hidden_layer << endl;
        // cout <<  "[ parameter_layer1: " << endl;
        // cout << parameter_layer1 << endl;
        
        // cout << "[ parameter_layer2: " << endl;
        // cout << parameter_layer2 << endl;

        for (int j = 0; j < hidden_number; j++)
        {
            
            for (int k = 0; k < 3; k++)
            {
                
                double derivative_1 = reward_count * ect * parameter_layer2(j,0) * 0.5 * (1 - pow(p_hidden_layer(0,j),2)) * input_variable(0,k);
               
                double delta_weight1 = learning_rate * (- derivative_1);
                
                parameter_layer1(k,j) += delta_weight1; 
            }
            double derivative =  reward_count * ect * (p_hidden_layer(0,j));
            double delta_weight = learning_rate * (- derivative);
            parameter_layer2(j,0) += delta_weight;

        }
    }
}; 