#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0,0,0,0;
  // estimation vector size should not be zero
  if(estimations.size()!=ground_truth.size() || estimations.size()==0)
  {
    cout<< "Invalid estimation size or ground truth data"<<endl;
    return rmse;
  }
  // accumulate squared residuals

  for(unsigned int i=0;i<estimations.size();++i){
    VectorXd residual=estimations[i]-ground_truth[i];
    residual=residual.array()*residual.array();
    rmse+=residual;
  }
  rmse=rmse/estimations.size();
  rmse=rmse.array().sqrt();


}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4);

  // map state parameters to local var

  float px=x_state(0);
  float py=x_state(1);
  float vx=x_state(2);
  float vy=x_state(3);

  float c1=px*px+py*py;
  float c2=sqrt(c1);
  float c3=c1*c2;

  if(fabs(c1) < 0.0001){
    std::cout << "Function CalculateJacobian() has Error: Division by Zero" << std::endl;
    return Hj;
  }
	
  Hj<< (px/c2),(py/c2),0,0,
      -(px/c1),(px/c1),0,0,
      (py*(vx*py-vy*px)/c3),(py*(vy*px-vx*py)/c3),(px/c2),(py/c2);


  return Hj;

}
