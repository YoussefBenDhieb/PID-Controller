#include "PID.h"
#include <algorithm>
#include <math.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	//Initialize PID parameters
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	//initialize errors
	this->p_error = 0;
	this->d_error = 0;
	this->i_error = 0;
	//initialize total error
	this->t_cte = 0;
	//intialize twiddle parameters
	this->dKp = 0.02;
	this->dKi = 0.02;
	this->dKd = 0.02;
	//initialize counter
	this->c = 0;
	//Initialize the iterator
	this->it = 0;
	//Initialize best error
	this->best_error = 1000000;
	//Initialize throttle
	this->throttle = 0.25;

}

void PID::UpdateError(double cte) {
	this->i_error = +cte;
	this->d_error = cte-PID::p_error;
	this->p_error = cte;
	this->c += 1;
	this->t_cte += cte;
}

double PID::GetThrottle() {
	return this->throttle;
}

double PID::Run() {
	return max(min(1.0,-this->Kp * this->p_error -this->Kd * this->d_error - this->Ki * this->i_error),-1.0); 
}

void PID::Twiddle(){
	if(this->dKp +this->dKd + this->dKi >0.02){
		this->it = this->it%3;
		
		double p, d;
		
		if(this->it == 0){
			p = this->Kp;
			d = this->dKp;
		}else if(this->it == 1){
			p = this->Kd;
			d = this->dKd;
		}else if(this->it == 2){
			p = this->Ki;
			d = this->dKi;
		}


		if(this->c == 0){
			p += d;
			if(this->it == 0){
				this->Kp = p;
				this->dKp = d;
			}else if(this->it == 1){
				this->Kd = p;
				this->dKd = d;
			}else if(this->it == 2){
				this->Ki = p;
				this->dKi = d;
			}
		}else if(this->c == this->N_ITER ){
			if(this->t_cte < this->best_error){
				this->best_error = this->t_cte;
				this->t_cte = 0;
				d *=1.1;
				this->c = 0;
				if(this->it == 0){
					this->Kp = p;
					this->dKp = d;
				}else if(this->it == 1){
					this->Kd = p;
					this->dKd = d;
				}else if(this->it == 2){
					this->Ki = p;
					this->dKi = d;
				}
				this->it +=1; 
			}else{
				p -= 2*d;
				this->t_cte = 0;
				if(this->it == 0){
					this->Kp = p;
					this->dKp = d;
				}else if(this->it == 1){
					this->Kd = p;
					this->dKd = d;
				}else if(this->it == 2){
					this->Ki = p;
					this->dKi = d;
				}
			}
		}else if (this->c == 2* this->N_ITER ){
			if(this->t_cte < this->best_error){
				this->best_error = this->t_cte;
				this->t_cte = 0;
				d *=1.1;
				this->c = 0;
				if(this->it == 0){
					this->Kp = p;
					this->dKp = d;
				}else if(this->it == 1){
					this->Kd = p;
					this->dKd = d;
				}else if(this->it == 2){
					this->Ki = p;
					this->dKi = d;
				}
				this->it +=1; 
			}else{
				p += d;
				d *=0.9;
				this->c = 0;
				if(this->it == 0){
					this->Kp = p;
					this->dKp = d;
				}else if(this->it == 1){
					this->Kd = p;
					this->dKd = d;
				}else if(this->it == 2){
					this->Ki = p;
					this->dKi = d;
				} 
				this->it +=1;
			}
		}

		
	}
}
