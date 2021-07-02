/******************************************************
    Author : shipeng_liu 
    Email : 1196075299@qq.com
    Description: Adaptive dynamic programming 
******************************************************/
#include <cmath>
#include<iostream>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>
#include "rambot_controller/src/controller/adp_learning/critic_network.cpp"
using namespace std;

/****************************************************************** 
    Action Network
    Input: S, diff_s
    number of hidden_layer: 1
    number of active_function: hidden_number
    output: steer_cmd
    parameter_layer1: parameters from input to hidden_layer
    parameter_layer2: parameters from hidden_layer to output
******************************************************************/

class action_network {

  private:

    double learning_rate = 0.00001 ; //learning rate
    double reward_count = 1;
    Eigen::MatrixXd parameter_layer1;
    Eigen::MatrixXd parameter_layer2;
    Eigen::MatrixXd input_variable;
    Eigen::MatrixXd hidden_layer;
    Eigen::MatrixXd p_hidden_layer;
    double output;
    int hidden_number;
    double deriva_J_u;
    static double active_function(double input) {
        // 1 - exp( - qi(t) ) / 1 + exp( - qi(t) ) )
        double ret = std::exp(- input);
        double tmp = (2)/(1 + ret) - 1;
        return tmp;
    }


  public:

    action_network() 
    {
        hidden_number = 4;
        init();
    }

    void init()
    {
        parameter_layer1 = Eigen::MatrixXd::Random(2, hidden_number);
        parameter_layer2 = Eigen::MatrixXd::Random(hidden_number, 1);
        
        hidden_layer = Eigen::MatrixXd::Random(1, hidden_number);
        p_hidden_layer = Eigen::MatrixXd::Random(1, hidden_number);
        input_variable = Eigen::MatrixXd::Random(1,2);
        output = 0;

    }

    double output_cmd(double s, double diff_s)
    {
        input_variable(0,0) = s;
        input_variable(0,1) = diff_s;
        //cout << parameter_layer1 << endl;
        //cout << parameter_layer2 << endl;
        hidden_layer = input_variable * parameter_layer1;
        
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
        output = active_function(output);
        return output;
    }

    void update_weight(double J_cost, critic_network *critic_network)
    {
       
        
        double reference_signal = 0;
        double eat = J_cost - reference_signal;
        cout << "\naction_network_information:\n";
        cout << left << setw(20) << "[ refer: " << right
             << setw(20) << reference_signal << "]" << endl;
        cout << left << setw(20) << "[ J_cost: " << right
             << setw(20) << J_cost << "]" << endl;
        cout << left << setw(20) << "[ eat: " << right
             << setw(20) << eat << "]" << endl;
        cout << left << setw(20) << "[ p_hidden_layer: " << right
             << setw(20) << p_hidden_layer << "]" << endl;
        cout << "[ p_hidden_layer: " << endl;
        cout << p_hidden_layer << endl;
        
        // cout <<  "[ parameter_layer1: " << endl;
        // cout << parameter_layer1 << endl;
        
        // cout << "[ parameter_layer2: " << endl;
        // cout << parameter_layer2 << endl;
    
             
        // compute the derivative of J to U
        for (int l = 0; l < 4; l++)
        {
            deriva_J_u += ( critic_network->return_parameter_layer2()(l,0) ) * 0.5 * (1 - pow(critic_network->return_p_hidden_layer()(0,l),2)) * critic_network->return_parameter_layer1()(2,l);
        }
        for (int j = 0; j < hidden_number; j++)
        {
            //compute the parameter_layer1
            for (int k = 0; k < 2; k++)
            {
                
                double derivative_1 = reward_count * eat *  0.5 * (1-pow(output,2)) * parameter_layer2(j,0) * 0.5 * (1 - pow(p_hidden_layer(0,j),2)) * input_variable(0,k) * deriva_J_u;
                double delta_weight1 = learning_rate * (- derivative_1);
                
                parameter_layer1(k,j) += delta_weight1; 
            }

            // compute the parameter_layer2 
           
            double derivative =  reward_count * eat * 0.5 * (1-pow(output,2)) *(p_hidden_layer(0,j)) * deriva_J_u;
            
            double delta_weight = learning_rate * (- derivative);
            parameter_layer2(j,0) += delta_weight;

        }
        
    }
};
    
    