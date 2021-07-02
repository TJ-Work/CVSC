#include <cmath>
#include<iostream>
#include <fstream>
#include <Eigen/Dense>
using namespace std;





class RBFN {

  private:
    double epseta = 0.5 ; //learning rate
    double seta = 0.0005; // seta
    int xn;  //2
    int k;   //5
    Eigen::MatrixXd b;
    Eigen::MatrixXd last_b ;
    Eigen::MatrixXd last_last_b ;
    
    Eigen::MatrixXd c;
    Eigen::MatrixXd last_c ;
    Eigen::MatrixXd last_last_c ;

    Eigen::MatrixXd w ;
    Eigen::MatrixXd last_w ;
    Eigen::MatrixXd last_last_w ;

   
    Eigen::MatrixXd dk;
    
    Eigen::MatrixXd  s_input;
    double Derivative_teacher_E;

    static double gaussian(double d, double b) {
        // exp( -(d^2)/2*s^2 )
        double tmp = - (d) / (2 * b*b);
        double ret = std::exp(tmp);
        return ret;
    }


  public:
    RBFN(int xn1, int k1) {
        xn = xn1;
        k = k1;
        init();
        

    }

    void init() {
        
        b = Eigen::MatrixXd::Random(k, 1);
        last_b = Eigen::MatrixXd::Random(k, 1);
        last_last_b = Eigen::MatrixXd::Random(k, 1);
    
        c = Eigen::MatrixXd::Random(k, xn);
        last_c = Eigen::MatrixXd::Random(k, xn);
        last_last_c = Eigen::MatrixXd::Random(k, xn);

        w = Eigen::MatrixXd::Random(k, 1);
        last_w = Eigen::MatrixXd::Random(k, 1);
        last_last_w = Eigen::MatrixXd::Random(k, 1);

        
        dk = Eigen::MatrixXd::Random(k, 1);
        cout << "w:    " << w <<endl;
        cout << "b:    " << b <<endl;
        cout << "c:    " << c <<endl;


    }
    void calc_distances() {
        int d;
        for (int i = 0; i < k; i++) {
            for (int j =0 ; j < xn; j++){
                d += pow( s_input(0,j)-c(i,j) , 2 );
            }
        dk(i,1) = d;
        }
    }


    void training(Eigen::MatrixXd s_input1,
                       double Derivative_teacher_E1) {
        s_input = s_input1;
        Derivative_teacher_E = Derivative_teacher_E1;

        //update the weight
        Eigen::MatrixXd w_next = w;
        Eigen::MatrixXd b_next = b;
        Eigen::MatrixXd c_next = c;
        for (int i = 0; i < k;i++){
            double delta_b =  Derivative_teacher_E * w(i,0) * gaussian(dk(i,0),b(i,0)) * (dk(i,0)) / pow(b(i,0),3);
            double delta_w =  Derivative_teacher_E * gaussian(dk(i,0),b(i,0));
            cout << "delta_w " << delta_w <<endl;
            w_next(i,0) = w(i,0) + epseta * delta_w + seta * (w(i,0) - last_w(i,0));
            b_next(i,0) = b(i,0) + epseta * delta_b + seta * (b(i,0) - last_b(i,0));
            for (int j =0; j < xn; j++){
                double delta_c =  Derivative_teacher_E * w(i,0) * (s_input(0,j) - c(i,j))/pow(b(i,0),2);
                c(i,j) = last_c(i,j) + epseta *delta_c + seta * (c(i,j) - last_c(i,j));
            }
        }

        last_w = w;
        w = w_next;
        last_b = b;
        b = b_next;
        last_c = c;
        c = c_next;

        //print the weitht
        cout << "w:    " << w <<endl;
        cout << "b:    " << b <<endl;
        cout << "c:    " << c <<endl;

    }
   

    double output() {
        double output;
        for (int i = 0; i < k;i++)
        {
           output = output + (w(i,0) * gaussian(dk(i,0),b(i,0)));
        }
        return output;
    }




    ~RBFN() {
        
    }


    
};
