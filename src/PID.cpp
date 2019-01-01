#include "PID.h"
#include <math.h>
#include <iostream>



using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
    this->total_err = 0.0;

    //this->dKp = Kp;
    //this->dKi = Ki;
    //this->dKd = Kd;
    this->dp[0] = 0.1*Kp;
    this->dp[1] = 0.1*Ki;
    this->dp[2] = 0.1*Kd;

    this->turn_index = 0;

    this->twiddle_switch = twiddle;
    this->total_err_count = 0;
    this->twiddle_step = 0;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    total_err += pow(cte,2.0);
    total_err_count++;
}

double PID::calSteerValue()
{
    return -Kp*p_error - Kd*d_error - Ki*i_error;
}
double PID::TotalError() {
    return total_err;
}

double PID::avgError() {
    return total_err/total_err_count;
}

void PID::resetError()
{
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    total_err = 0.0;
    total_err_count = 0;
}

void PID::twiddle_1()
{
    if(twiddle_step == 0)
    {
        switch(turn_index)
        {
            case 0:
                Kp += dp[turn_index];
                break;
            case 1:
                Ki += dp[turn_index];
                break;
            case 2:
                Kd += dp[turn_index];
                break;
            default:
                break;
        }         
    }

}

//tune coeff by twiddle
void PID::twiddle_2(double &best_err)
{
    double cur_err = avgError();

    if(twiddle_step == 0)
    {
        if(cur_err < best_err)
        {
            dp[turn_index] *= (1.1);
            best_err = cur_err;
            turn_index++;
        }
        else
        {
            switch(turn_index)
            {
                case 0:
                    Kp -= 2*dp[turn_index];
                    break;
                case 1:
                    Ki -= 2*dp[turn_index];
                    break;
                case 2:
                    Kd -= 2*dp[turn_index];
                    break;
                default:
                    break;
            }   
            twiddle_step = 1;
        }
    }
    else 
    {
        if(cur_err < best_err)
        {
            dp[turn_index] *= (1.1);
            best_err = cur_err;
        }
        else
        {
            switch(turn_index)
            {
                case 0:
                    Kp += dp[turn_index];
                    break;
                case 1:
                    Ki += dp[turn_index];
                    break;
                case 2:
                    Kd += dp[turn_index];
                    break;
                default:
                    break;
            }  
            dp[turn_index] *= (0.9);
        }
        twiddle_step = 0;
        turn_index++;
    }

    
    if(turn_index > 2)
    {
        turn_index = 0;
        printCoef();
    }
    resetError();
}

void PID::printCoef()
{

    cout<<"***************************************************"<<endl;
    cout<<"Kp is "<<Kp<<endl;
    cout<<"Ki is "<<Ki<<endl;
    cout<<"Kd is "<<Kd<<endl;
    cout<<"tol is "<<(dp[0]+dp[1]+dp[2])<<endl;
    cout<<"current err is "<<avgError()<<endl;
    cout<<"***************************************************"<<endl;
}



