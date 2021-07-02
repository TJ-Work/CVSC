#include "rambot_controller/src/controller/adp_learning/critic_network.h"
/******************************************************
    Author : shipeng_liu 
    Email : 1196075299@qq.com
    Description: Adaptive dynamic programming 
******************************************************/

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

static critic_network& critic_network::Get_critic_network() 
{
    static critic_network singleton;
    return singleton;
}
critic_network::critic_network() 
{
    hidden_number = 4;
    init();
}
double critic_network::return_J_cost()
{
    return output;
}
double critic_network::return_reference()
{
    return reference_signal;
}
Eigen::MatrixXd critic_network::return_parameter_layer1()
{
    return parameter_layer1;
}
Eigen::MatrixXd critic_network::return_parameter_layer2()
{
    return parameter_layer2;
}

void critic_network::init()
{
    parameter_layer1 = Eigen::MatrixXd::Random(3, hidden_number);
    parameter_layer2 = Eigen::MatrixXd::Random(hidden_number, 1);
    
    hidden_layer = Eigen::MatrixXd::Random(1, hidden_number);
    p_hidden_layer = Eigen::MatrixXd::Random(1, hidden_number);
    input_variable = Eigen::MatrixXd::Random(1,3);
    output = 0;

}

double critic_network::output_J_cost(double s, double diff_s,double steer_cmd)
{
    input_variable(0,0) = s;
    input_variable(0,1) = diff_s;
    input_variable(0,2) = steer_cmd;
    cout << "parameter_layer1"<<parameter_layer1 << endl;
    cout << "parameter_layer2"<< parameter_layer2 << endl;
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
    
    
    return output;
}

void critic_network::update_weight(double J_cost, double Last_J_cost, double s, double diff_s, double steer_cmd)
{
    reference_signal =  (pow(s,2) + pow(diff_s,2));
    
    //double reference_signal = 0.5;
    cout << "refer: "<<reference_signal<<endl;
    cout << "J_cost: "<< J_cost <<endl; 
    double ect = J_cost - reference_signal;
    cout << "ect"<< ect << endl; 
    cout << "p_hidden_layer()" << p_hidden_layer << endl;
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


    

    