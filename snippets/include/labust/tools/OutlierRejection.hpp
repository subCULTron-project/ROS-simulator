/*
 * OutlierRejection.hpp
 *
 *  Created on: Jun 11, 2015
 *      Author: filip
 */

#ifndef OUTLIERREJECTION_HPP_
#define OUTLIERREJECTION_HPP_

#include <Eigen/Dense>
#include <ros/ros.h>

#include <std_msgs/Float32.h>

namespace labust
{
	namespace tools
	{
		class OutlierRejection
		{
		public:
			OutlierRejection(int input_size, double lambda, double a_wk0 = 1.0, double b_wk0 = 1.0):
				input_size(input_size),
				lambda(lambda),
				a_wk0(a_wk0),
				b_wk0(b_wk0),
				variance_k(1.0),
				exp_wk(1.0),
				y_k(0.0),
				x_k(Eigen::VectorXd::Zero(input_size)),
				exp_beta_k(Eigen::VectorXd::Zero(input_size)),
				beta_0(Eigen::VectorXd::Zero(input_size)),
				N_k(0.0),
				sum_wyy(0.0),
				sum_wxx(Eigen::MatrixXd::Zero(input_size, input_size)),
				sigma_beta_k(1000*Eigen::MatrixXd::Identity(input_size, input_size)),
				sum_wyx(Eigen::VectorXd::Zero(input_size))
			{
				//this->init();
				sigma_beta_0_inv = sigma_beta_k.inverse();

				ros::NodeHandle nh;
				pubWK = nh.advertise<std_msgs::Float32>("outlier_wk",1);
				pubSIGMA = nh.advertise<std_msgs::Float32>("outlier_sigma",1);
				pubY = nh.advertise<std_msgs::Float32>("outlier_y",1);


			}

			~OutlierRejection()
			{

			}

			//void init()
			//{
			//}

			void step(Eigen::VectorXd input, double output, double* y_filt, double* sigma, double* w){

				/*** Automatic Outlier Detection: A Bayesian Approach
				 *   Robotics and Automation, 2007 IEEE International Conference on In Robotics and Automation,
				 *   2007 IEEE International Conference on (April 2007), pp. 2489-2494, doi:10.1109/robot.2007.363693
				 *   by Jo-Anne Ting, Aaron D'Souza, Stefan Schaal
				 ***/

				x_k = input;
				y_k = output;

				//ROS_ERROR("Input: %f, %f, %f",input(0), input(1), input(2));
				//ROS_ERROR("Output: %f",output);

				//exp_wk = (a_wk0+0.5)/(b_wk0+1/(2*variance_k)*(std::pow(y_k-exp_beta_k.transpose()*x_k,2)+x_k.transpose()*sigma_beta_k*x_k));

				N_k = 1 + lambda*N_k;
				sum_wxx = exp_wk*x_k*x_k.transpose()+lambda*sum_wxx;
				sum_wyx = exp_wk*y_k*x_k + lambda*sum_wyx;
				sum_wyy = exp_wk*y_k*y_k + lambda*sum_wyy;

			/*	ROS_ERROR("Nk: %f", N_k);
				ROS_ERROR("sum_wxx");
				ROS_ERROR_STREAM(sum_wxx);
				ROS_ERROR("sum_wyx");
				ROS_ERROR_STREAM(sum_wyx);
				ROS_ERROR("sum_wyy");
				ROS_ERROR_STREAM(sum_wyy);
*/
				sigma_beta_k = (sigma_beta_0_inv + 1/variance_k*sum_wxx);
				sigma_beta_k = sigma_beta_k.inverse(); // Ovaj korak se moze zamjeniti rekurzivnim oblikom (u clanku referenca).
				exp_beta_k = sigma_beta_k*(sigma_beta_0_inv*beta_0 + 1/variance_k*sum_wyx);

				//ROS_ERROR("sigma_beta_k");
				//ROS_ERROR_STREAM(sigma_beta_k);
				//ROS_ERROR("exp_beta_k");
				//ROS_ERROR_STREAM(exp_beta_k);

				exp_wk = (a_wk0+0.5)/(b_wk0+1/(2*variance_k)*(std::pow(y_k-exp_beta_k.transpose()*x_k,2)+x_k.transpose()*sigma_beta_k*x_k));

				//ROS_ERROR("exp(w_k): %f", exp_wk);

				Eigen::VectorXd Iones(Eigen::VectorXd::Constant(input_size,1.0));

				double p1 = (2*sum_wyx.transpose()*exp_beta_k)(0,0);
				double p2 = (exp_beta_k.transpose()*sum_wxx*exp_beta_k)(0,0);
				double p3 = (Iones.transpose()*(sum_wxx*sigma_beta_k).diagonal())(0,0);

				variance_k = 1/N_k*(sum_wyy - p1 + p2  + p3);

				//ROS_ERROR("variance_k: %f", variance_k);

				*y_filt = exp_beta_k.transpose()*input;
				*sigma = variance_k;
				*w = exp_wk;

				std_msgs::Float32 msg;
				msg.data = exp_wk;
				pubWK.publish(msg);
				msg.data = variance_k;
				pubSIGMA.publish(msg);
				msg.data = exp_beta_k.transpose()*input;
				pubY.publish(msg);

			}

			int input_size;
			double y_k,lambda, exp_wk, variance_k, a_wk0, b_wk0, sum_wyy;
			Eigen::VectorXd exp_beta_k, beta_0, x_k, sum_wyx;
			Eigen::MatrixXd sigma_beta_k, sigma_beta_0_inv, sum_wxx;
			double N_k;

			ros::Publisher pubWK, pubSIGMA, pubY;
		};
	}
}

#endif /* OUTLIERREJECTION_HPP_ */
