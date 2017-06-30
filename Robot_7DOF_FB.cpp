


#include "Robot_7DOF_FB.h"
#include <vector>
#include "Matrix.h"
#include "MatrixMath.h"

#define _USE_MATH_DEFINES // for C++
#include <math.h>

using namespace std;

//歐拉 Z1X2Y3 Intrinsic Rotaions相對於當前坐標系的的旋轉
Matrix R_z1x2y3(float alpha,float beta,float gamma)
{
	Matrix ans(3,3);
	ans << cos(alpha)*cos(gamma)-sin(alpha)*sin(beta)*sin(gamma)	<< -cos(beta)*sin(alpha)	<< cos(alpha)*sin(gamma)+cos(gamma)*sin(alpha)*sin(beta)
        << cos(gamma)*sin(alpha)+cos(alpha)*sin(beta)*sin(gamma)	<< cos(alpha)*cos(beta)		<< sin(alpha)*sin(gamma)-cos(alpha)*cos(gamma)*sin(beta)
        << -cos(beta)*sin(gamma)									<< sin(beta)				<< cos(beta)*cos(gamma);

	return ans;

}

float norm(const Matrix& v)
{
	int i=0;
	float ans=0;

	for(i=1;i<=v.getRows();i++)
	{
		ans+=pow(v(i,1),2);
	}
	
	ans=sqrt(ans);

	return ans;
}


Matrix Rogridues(float theta,const Matrix& V_A)
{
	Matrix R_a(4,4);

	float cs = cos( theta );
    float sn = sin( theta );

	R_a	<<	cs + pow(V_A.getNumber(1,1),2)*(1-cs)								<<	V_A.getNumber(1,1)*V_A.getNumber(2,1)*(1-cs)-V_A.getNumber(3,1)*sn	<<	V_A.getNumber(1,1)*V_A.getNumber(3,1)*(1-cs)+V_A.getNumber(2,1)*sn  <<	0
		<<	V_A.getNumber(1,1)*V_A.getNumber(2,1)*(1-cs)+V_A.getNumber(3,1)*sn	<<	cos(theta)+pow(V_A.getNumber(2,1),2)*(1-cs)							<<	V_A.getNumber(2,1)*V_A.getNumber(3,1)*(1-cs)-V_A.getNumber(1,1)*sn	<<	0
        <<	V_A.getNumber(1,1)*V_A.getNumber(3,1)*(1-cs)-V_A.getNumber(2,1)*sn	<<	V_A.getNumber(2,1)*V_A.getNumber(3,1)*(1-cs)+V_A.getNumber(1,1)*sn	<<	cs+pow(V_A.getNumber(3,1),2)*(1-cs)									<<	0
        <<	0																	<<	0																	<<	0																	<<	1;
    
	return R_a;
}
int IK_7DOF_nonFB(const float l1,const float l2,const float l3,const float x_base,const float y_base,const float z_base,const float x_end,const float y_end,const float z_end,const float alpha,const float beta,const float gamma,const float Rednt_alpha,float* theta)
{
	int i=0;
	
	//Out put initial to zero
	for(i=AXIS1;i<AXIS7;i++)
	{
		theta[i]=0;
	}

	Matrix R(3,3);
	R=R_z1x2y3(alpha,beta,gamma);

	Matrix V_H_hat_x(3,1);
	V_H_hat_x=Matrix::ExportCol(R,1);//取出歐拉角轉換的旋轉矩陣，取出第1行為X軸旋轉後向量
	V_H_hat_x*=1/norm(V_H_hat_x);
	
	Matrix V_H_hat_y(3,1);
	V_H_hat_y=Matrix::ExportCol(R,2);//取出歐拉角轉換的旋轉矩陣，取出第2行為Y軸旋轉後向量
	V_H_hat_y*=1/norm(V_H_hat_y);
	

	Matrix V_r_end(3,1);
	V_r_end	<<x_end-x_base
			<<y_end-y_base
			<<z_end-z_base;


	Matrix V_r_h(3,1);
	V_r_h=V_H_hat_x*L3;

	Matrix V_r_wst(3,1);
	V_r_wst=V_r_end-V_r_h;	

	//theat 4
	theta[AXIS4]=-(float)(M_PI-acos((pow(l1,2)+pow(l2,2)-pow(norm(V_r_wst),2))/(2*l1*l2)));


	Matrix V_r_m(3,1);
	V_r_m=(pow(l1,2)-pow(l2,2)+pow(norm(V_r_wst),2))/(2*pow(norm(V_r_wst),2))*V_r_wst;



	//Redundant circle 半徑R
	float Rednt_cir_R = pow(l1,2)- pow( (pow(l1,2)-pow(l2,2)+pow(norm(V_r_wst),2))/(2*norm(V_r_wst)) , 2);
	Rednt_cir_R=sqrt(Rednt_cir_R);


	//圓中心點到Elbow向量 V_r_u
	Matrix V_shz(3,1);
	V_shz	<<0
			<<0
			<<1;

	Matrix V_alpha_hat(3,1);//V_alpha_hat=cross(V_r_wst,V_shz)/norm(cross(V_r_wst,V_shz));
	Matrix temp_cross(3,1);
	temp_cross=MatrixMath::cross(V_r_wst,V_shz); //錯誤
	V_alpha_hat=temp_cross*(1/norm(temp_cross));

	Matrix V_beta_hat(3,1);//V_beta_hat=cross(V_r_wst,V_alpha_hat)/norm(cross(V_r_wst,V_alpha_hat));
	temp_cross=MatrixMath::cross(V_r_wst,V_alpha_hat);
	V_beta_hat=temp_cross*(1/norm(temp_cross));



	Matrix temp(4,4);//temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst)) *[Rednt_cir_R*V_beta_hat;1];  //Rednt_alpha的方向和論文上的方向性相反
	Matrix V_r_wst_unit =V_r_wst*(1/norm(V_r_wst));
	Matrix V_temp3x1(3,1);//需要寫一個可以補1的函試
	Matrix V_temp4x1(4,1);
	V_temp3x1=V_beta_hat*Rednt_cir_R;
	V_temp4x1.Vec_ext_1_row(V_temp3x1,1); //3x1 extend to 4x1

	temp=Rogridues(Rednt_alpha,V_r_wst_unit)*V_temp4x1;


	Matrix V_R_u(3,1);
	V_R_u.Vec_export_3_row(temp);
	

	Matrix V_r_u(3,1);
	V_r_u=V_r_m+V_R_u;

	theta[AXIS1]=atan2(-V_r_u(1,1),-V_r_u(3,1));//theta(1)=atan2(-V_r_u(1),-V_r_u(3));


	if (theta[AXIS1] !=0) 
		theta[AXIS2]=atan2(V_r_u(2,1),-V_r_u(1,1)/sin(theta[AXIS1]));
	else
		theta[AXIS2]=atan2(-V_r_u(2,1),V_r_u(3,1));
	


	//theat 3
	//theta(3)=atan2( sin(theta(2))*sin(theta(1))*V_r_wst(1)+cos(theta(2))*V_r_wst(2)+sin(theta(2))*cos(theta(1))*V_r_wst(3),cos(theta(1))*V_r_wst(1)-sin(theta(1))*V_r_wst(3));
	theta[AXIS3]=atan2( sin(theta[AXIS2])*sin(theta[AXIS1])*V_r_wst(1,1)+cos(theta[AXIS2])*V_r_wst(2,1)+sin(theta[AXIS2])*cos(theta[AXIS1])*V_r_wst(3,1),cos(theta[AXIS1])*V_r_wst(1,1)-sin(theta[AXIS1])*V_r_wst(3,1));



	//theat 5
	Matrix V_r_f(3,1);
	V_r_f=V_r_wst-V_r_u;

	Matrix V_Axis6(3,1);
	V_Axis6=MatrixMath::cross(V_H_hat_y,-V_r_f)*(1/norm(MatrixMath::cross(V_H_hat_y,-V_r_f)));//V_Axis6=cross(V_H_hat_y,-V_r_f)/norm(cross(V_H_hat_y,-V_r_f));

	Matrix V_r_wst_u(3,1);
	V_r_wst_u=V_r_wst+V_Axis6;

	Matrix A1_4(4,4);
	A1_4=MatrixMath::RotY(theta[AXIS1])*MatrixMath::RotX(theta[AXIS2])*MatrixMath::RotZ(theta[AXIS3])*MatrixMath::Tz(-l1)*MatrixMath::RotY(theta[AXIS4]);//A1_4=Ry(theta(1))*Rx(theta(2))*Rz(theta(3))*Tz(-L1)*Ry(theta(4));
	

	Matrix V_temp_f(4,1);
	Matrix V_r_wst_u_4x1(4,1);
	V_r_wst_u_4x1.Vec_ext_1_row(V_r_wst_u,1);
	

	V_temp_f=MatrixMath::Inv(A1_4)*V_r_wst_u_4x1;//V_temp_f=inv(A1_4)*[V_r_wst_u;1]; //(3.31) 這個是補一列1上去的意思,need fix
	theta[AXIS5]=atan2(V_temp_f(2,1),V_temp_f(1,1));//theta(5)=atan2(V_temp_f(2),V_temp_f(1));

	
	//theat 6
	Matrix V_r_wst_r(3,1);
	V_r_wst_r=V_r_wst+V_H_hat_y;

	Matrix A1_5(4,4);
	A1_5=A1_4*MatrixMath::RotZ(theta[AXIS5])*MatrixMath::Tz(-l2);//A1_5=A1_4*Rz(theta(5))*Tz(-L2);
	
	Matrix V_temp_g(4,4);
	Matrix V_r_wst_r_4x1(4,1);
	V_r_wst_r_4x1.Vec_ext_1_row(V_r_wst_r,1);
	
	V_temp_g=MatrixMath::Inv(A1_5)*V_r_wst_r_4x1; //V_temp_g=inv(A1_5)*[V_r_wst_r;1]; //(3.38)  這個是補一列1上去的意思,need fix
	
	theta[AXIS6]=atan2(V_temp_g(3,1),V_temp_g(2,1));


	//theat 7
	Matrix V_r_wst_f(3,1);
	V_r_wst_f=V_r_wst+V_H_hat_x;

	Matrix A1_6(4,4);
	A1_6=A1_5*MatrixMath::RotX(theta[AXIS6]);

	Matrix V_temp_h(3,1);
	Matrix V_r_wst_f_4x1(4,1);
	V_r_wst_f_4x1.Vec_ext_1_row(V_r_wst_f,1);
	
	V_temp_h=MatrixMath::Inv(A1_6)*V_r_wst_f_4x1; //V_temp_h=inv(A1_6)*[V_r_wst_f;1]; 
	
	theta[AXIS7]=atan2(-V_temp_h(1,1),-V_temp_h(3,1));//theta(7)=atan2(-V_temp_h(1),-V_temp_h(3));


	return 0;
}


//第七軸為roll軸

int IK_7DOF_FB7roll(const float linkL[6],const float base[3],const float Pend[3],const float PoseAngle[3],const float Rednt_alpha,float* out_theta)
{
    //輸出參數initial
	float theta[7]={0};
	float tempfloat=0.0;
    //輸入連桿長度
	//linkL[0];//L0 頭到肩膀
	//linkL[1];//L1 上臂L型長邊
	//linkL[2];//L2 上臂L型短邊
	//linkL[3];//L3 上臂L型短邊
	//linkL[4];//L4 上臂L型長邊
	//linkL[5];//L5 end effector

    // == 求出H_hat_x ==//
	Matrix  R(3,3);
    R=R_z1x2y3(PoseAngle[0],PoseAngle[1],PoseAngle[2]); //alpha,beta,gamma

    Matrix V_H_hat_x(3,1);
	V_H_hat_x=Matrix::ExportCol(R,1);//取出歐拉角轉換的旋轉矩陣，取出第1行為X軸旋轉後向量
	V_H_hat_x*=1/norm(V_H_hat_x);

    Matrix V_H_hat_z(3,1);
	V_H_hat_z=Matrix::ExportCol(R,3);//取出歐拉角轉換的旋轉矩陣，取出第3行為Z軸旋轉後向量
	V_H_hat_z*=1/norm(V_H_hat_z);
 
	Matrix V_r_end(3,1);
	V_r_end	<<Pend[0]-base[0]//x
			<<Pend[1]-base[1]//y
			<<Pend[2]-base[2];//z
    
	Matrix V_r_h(3,1);
    V_r_h=V_H_hat_x*linkL[5]; //L5

	Matrix V_r_wst(3,1);
	V_r_wst=V_r_end-V_r_h;	
  
	// ==Axis4== //
    float ru_norm=sqrt(pow(linkL[1],2)+pow(linkL[2],2)); //L型的斜邊長度
    float rf_norm=sqrt(pow(linkL[3],2)+pow(linkL[4],2));
	
    float theta_tmp=acos((pow(ru_norm,2) + pow(rf_norm,2)- pow(norm(V_r_wst),2)) / (2*ru_norm*rf_norm));
    theta[AXIS4]=(float)(2*M_PI)-atan2(linkL[1],linkL[2])-atan2(linkL[4],linkL[3])-theta_tmp;

    // ==Axis1 2== //
	Matrix V_r_m(3,1);
    V_r_m=(pow(ru_norm,2)-pow(rf_norm,2)+pow(norm(V_r_wst),2))/(2*pow(norm(V_r_wst),2))*V_r_wst;

	//Redundant circle 半徑R
	float Rednt_cir_R = pow(ru_norm,2)- pow((pow(ru_norm,2)-pow(rf_norm,2)+pow(norm(V_r_wst),2))/(2*norm(V_r_wst)) , 2);
	Rednt_cir_R=sqrt(Rednt_cir_R);

   
	//圓中心點到Elbow向量 V_r_u
	Matrix V_shx(3,1);
	V_shx	<<1
			<<0
			<<0;

	Matrix V_shy(3,1);
	V_shy	<<0
			<<1
			<<0;

	Matrix V_shz(3,1);
	V_shz	<<0
			<<0
			<<1;

	Matrix V_alpha_hat(3,1);//V_alpha_hat=cross(V_r_wst,V_shz)/norm(cross(V_r_wst,V_shz));
	Matrix temp_cross(3,1);
	temp_cross=MatrixMath::cross(V_r_wst,V_shz); //錯誤
	V_alpha_hat=temp_cross*(1/norm(temp_cross));

	Matrix V_beta_hat(3,1);//V_beta_hat=cross(V_r_wst,V_alpha_hat)/norm(cross(V_r_wst,V_alpha_hat));
	temp_cross=MatrixMath::cross(V_r_wst,V_alpha_hat);
	V_beta_hat=temp_cross*(1/norm(temp_cross));

	Matrix temp(4,4);//temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst)) *[Rednt_cir_R*V_beta_hat;1];  //Rednt_alpha的方向和論文上的方向性相反
	Matrix V_r_wst_unit =V_r_wst*(1/norm(V_r_wst));
	Matrix V_temp3x1(3,1);//需要寫一個可以補1的函試
	Matrix V_temp4x1(4,1);
	V_temp3x1=V_beta_hat*Rednt_cir_R;
	V_temp4x1.Vec_ext_1_row(V_temp3x1,1); //3x1 extend to 4x1
	temp=Rogridues(Rednt_alpha,V_r_wst_unit)*V_temp4x1;

	
	Matrix V_R_u(3,1);//V_R_u=temp(1:3,1);
	V_R_u.Vec_export_3_row(temp);
	

	Matrix V_r_u(3,1);//V_r_u=V_r_m+V_R_u;
	V_r_u=V_r_m+V_R_u;

	Matrix V_r_f(3,1);// V_r_f=V_r_wst-V_r_u;
	V_r_f=V_r_wst-V_r_u;

	Matrix Vn_u_f(3,1);//Vn_u_f=cross(V_r_u,V_r_f)/norm(cross(V_r_u,V_r_f)); //ru 及 rf的法向量
	temp_cross=MatrixMath::cross(V_r_u,V_r_f); 
	Vn_u_f=temp_cross*(1/norm(temp_cross));
	float theta_upoff=atan(linkL[2]/linkL[1]);
	V_temp4x1.Vec_ext_1_row(V_r_u,1);//temp=Rogridues(-theta_upoff,Vn_u_f)*[V_r_u;1];  
	temp=Rogridues(-theta_upoff,Vn_u_f)*V_temp4x1;
	Matrix V_ru_l1(3,1);//旋轉 V_r_u  到V_ru_l1
	V_ru_l1.Vec_export_3_row(temp);
	
	V_ru_l1=linkL[1]*V_ru_l1*(1/norm(V_ru_l1)); //調整成L1長度
	
	theta[AXIS1]=atan2(V_ru_l1(1,1),-V_ru_l1(3,1));//theta(1)=atan2(V_ru_l1(1),-V_ru_l1(3));


	if (theta[AXIS1] !=0) 
		theta[AXIS2]=atan2(V_ru_l1(2,1),V_ru_l1(1,1)/sin(theta[AXIS1]));
	else
		theta[AXIS2]=atan2(V_ru_l1(2,1),-V_ru_l1(3,1));


	// ==Axis3== //
	//看shy(V_r_u,V_r_f的法向量)經過1,2軸旋轉後  與V_r_u,V_r_f 需要第3軸轉多少
	Matrix nV_shy;
	nV_shy=V_shy*(-1);
	V_temp4x1.Vec_ext_1_row(nV_shy,1);// V_n_yrot12=Ry(-theta(1))*Rx(theta(2))*[-V_shy;1];  //第一軸和大地Y座標方向相反
	temp=MatrixMath::RotY(-theta[AXIS1])*MatrixMath::RotX(theta[AXIS2])*V_temp4x1;
	
	Matrix V_n_yrot12(3,1);
	V_n_yrot12.Vec_export_3_row(temp);//V_n_yrot12=V_n_yrot12(1:3,1);

	Matrix Vn_nuf_nyrot12;
	Vn_nuf_nyrot12=MatrixMath::cross(Vn_u_f,V_n_yrot12);
	Vn_nuf_nyrot12=Vn_nuf_nyrot12*(1/norm(Vn_nuf_nyrot12));
	tempfloat=MatrixMath::dot(V_n_yrot12,Vn_u_f)*(1/norm(V_n_yrot12))*(1/norm(Vn_u_f));// temp=V_n_yrot12'*Vn_u_f/norm(V_n_yrot12)/norm(Vn_u_f); 


	//Vn_u_f 和 V_n_yrot12的法向量   與 V_ru_l1同方向 theta(3)需要加負號
	if ( norm(Vn_nuf_nyrot12 - V_ru_l1*(1/norm(V_ru_l1))) < DEF_VERY_SMALL )
		theta[AXIS3]=-acos(tempfloat);
	else
		theta[AXIS3]=acos(tempfloat);
	
	
    // ==Axis5== //
    float theat_lowoff=atan(linkL[3]/linkL[4]); //旋轉V_r_f 到 V_rf_l4
	V_temp4x1.Vec_ext_1_row(V_r_f,1);//temp=Rogridues(theat_lowoff,Vn_u_f)*[V_r_f;1];  //旋轉 V_r_f  V_rf_l4
	temp=Rogridues(theat_lowoff,Vn_u_f)*V_temp4x1; 
	Matrix V_rf_l4(3,1);
	V_rf_l4.Vec_export_3_row(temp);// V_rf_l4=temp(1:3,1);
	V_rf_l4=V_rf_l4*linkL[4]*(1/norm(V_rf_l4)); //調整成L4長度
	
	//V_n_rfl4 及V_n_rf形成的平面 的法向量
	Matrix Vn_rfl4_nuf;
	Vn_rfl4_nuf=MatrixMath::cross(V_rf_l4,Vn_u_f)*(1/norm(MatrixMath::cross(V_rf_l4,Vn_u_f)));//Vn_rfl4_nuf=cross(V_rf_l4,Vn_u_f)/norm(cross(V_rf_l4,Vn_u_f));
	float t_rfl4_nuf=(MatrixMath::dot(Vn_rfl4_nuf,V_r_wst)- MatrixMath::dot(Vn_rfl4_nuf,V_r_end))/pow(norm(Vn_rfl4_nuf),2);//  t_rfl4_nuf=(Vn_rfl4_nuf'*V_r_wst-Vn_rfl4_nuf'*V_r_end)/(norm(Vn_rfl4_nuf)^2); //V_n_rf,V_n_rfl4平面上，且經過V_r_end點的直線參數式的t 為rfl4_nuf
	Matrix Vproj_end_rfl4_nuf;
	Vproj_end_rfl4_nuf=V_r_end+t_rfl4_nuf*Vn_rfl4_nuf;//V_r_end 沿著V_n_rfl4,V_n_rf平面法向量投影在平面上的點
	Matrix V_wst_to_projend_rfl4_nuf;
	V_wst_to_projend_rfl4_nuf=Vproj_end_rfl4_nuf-V_r_wst;
	
	//V_n_rfl4 及V_n_rf形成的平面 的法向量
	//防止在acos(1.000000.....)的時候會出現虛部的情況
	tempfloat=MatrixMath::dot(V_rf_l4,V_wst_to_projend_rfl4_nuf)*(1/norm(V_rf_l4))*(1/norm(V_wst_to_projend_rfl4_nuf));//temp=V_rf_l4'*V_wst_to_projend_rfl4_nuf/norm(V_rf_l4)/norm(V_wst_to_projend_rfl4_nuf);
	if (abs(tempfloat-1) < DEF_VERY_SMALL) 
	{
		if (tempfloat >0)
			tempfloat=1;
		else
			tempfloat=-1;
	}	

	Matrix Vn_rfl4_WstToProjEndRfl4Nuf;
	Vn_rfl4_WstToProjEndRfl4Nuf=MatrixMath::cross(V_rf_l4*(1/norm(V_rf_l4)) , V_wst_to_projend_rfl4_nuf*(1/norm(V_wst_to_projend_rfl4_nuf)));
	Vn_rfl4_WstToProjEndRfl4Nuf=Vn_rfl4_WstToProjEndRfl4Nuf*(1/norm(Vn_rfl4_WstToProjEndRfl4Nuf));
 
	//平面法向量 和 Vn_rfl4_nuf  同邊要加負號  判斷theta5要往上或往下轉
	if (norm(Vn_rfl4_WstToProjEndRfl4Nuf - Vn_rfl4_nuf) < DEF_VERY_SMALL)
        theta[AXIS5]=-acos(tempfloat); 
    else
        theta[AXIS5]=acos(tempfloat); 

	// ==Axis6== //
	V_temp4x1.Vec_ext_1_row(Vn_u_f,1);
	temp=Rogridues(-theta[AXIS5],Vn_rfl4_nuf)*V_temp4x1; 
	Matrix Vn_nuf_rotx5_along_NRfl4Nuf(3,1);
	Vn_nuf_rotx5_along_NRfl4Nuf.Vec_export_3_row(temp);  //Vn_nuf_rotx5_along_NRfl4Nuf=temp(1:3,1);//nuf 沿著 Vn_rfl4_nuf 旋轉第5軸角度得到投影點與目標點平面的法向量
	Vn_nuf_rotx5_along_NRfl4Nuf=Vn_nuf_rotx5_along_NRfl4Nuf*(1/norm(Vn_nuf_rotx5_along_NRfl4Nuf));

	Matrix V_wst_to_end;
	V_wst_to_end=V_r_end-V_r_wst;

	Matrix Vn_WstToEnd_WstToProjEndRfl4Nuf;
	Vn_WstToEnd_WstToProjEndRfl4Nuf=MatrixMath::cross(V_wst_to_end,V_wst_to_projend_rfl4_nuf);//V_wst_to_projend 和 V_wst_to_end的法向量
	Vn_WstToEnd_WstToProjEndRfl4Nuf=Vn_WstToEnd_WstToProjEndRfl4Nuf*(1/norm(Vn_WstToEnd_WstToProjEndRfl4Nuf));
    
	//利用法向量方向 判斷theta6旋轉方向
	tempfloat=MatrixMath::dot(V_wst_to_projend_rfl4_nuf,V_wst_to_end)*(1/norm(V_wst_to_projend_rfl4_nuf)*(1/norm(V_wst_to_end))); //temp=V_wst_to_projend_rfl4_nuf'*V_wst_to_end/norm(V_wst_to_projend_rfl4_nuf)/norm(V_wst_to_end);

	if (norm(Vn_WstToEnd_WstToProjEndRfl4Nuf - Vn_nuf_rotx5_along_NRfl4Nuf) < DEF_VERY_SMALL)
        theta[AXIS6]=-acos(tempfloat); 
    else
        theta[AXIS6]=acos(tempfloat); 
	
	// ==Axis7== //
	Matrix V_x_rot1to6;
	V_temp4x1.Vec_ext_1_row(V_shx,1);
	V_x_rot1to6=MatrixMath::RotY(-theta[AXIS1])*MatrixMath::RotX(theta[AXIS2])*V_temp4x1;//V_x_rot1to6=Ry(-theta(1))*Rx(theta(2))*[V_shx;1];  //第一軸和大地Z座標方向相反
	
	//V_shx經過1to軸旋轉後變應該要與末點座標系的Z軸貼齊
	temp=Rogridues(theta[AXIS3],V_ru_l1*(1/norm(V_ru_l1)))*V_x_rot1to6;		//temp=Rogridues(theta(3),V_ru_l1/norm(V_ru_l1))*V_x_rot1to6;  
	temp=Rogridues(theta[AXIS4],Vn_u_f*(1/norm(Vn_u_f)))*temp;				//temp=Rogridues(theta(4),Vn_u_f/norm(Vn_u_f))*temp;  
	temp=Rogridues(theta[AXIS5],Vn_rfl4_nuf*(1/norm(Vn_rfl4_nuf)))*temp;	//temp=Rogridues(theta(5),Vn_rfl4_nuf/norm(Vn_rfl4_nuf))*temp; 
	temp=Rogridues(theta[AXIS6],Vn_nuf_rotx5_along_NRfl4Nuf)*temp;			//temp=Rogridues(theta(6),Vn_nuf_rotx5_along_NRfl4Nuf)*temp; 
	V_x_rot1to6.Vec_export_3_row(temp);										//V_x_rot1to6=temp(1:3,1); 
	V_x_rot1to6=V_x_rot1to6*(1/norm(V_x_rot1to6));

	//xrot1to6 和 V_H_hat_z 的法向量來判斷第7軸旋轉方向
	Matrix Vn_xrot1to6_VHhatz;
	Vn_xrot1to6_VHhatz=MatrixMath::cross(V_x_rot1to6,V_H_hat_z);
	Vn_xrot1to6_VHhatz=Vn_xrot1to6_VHhatz*(1/norm(Vn_xrot1to6_VHhatz));

	//V_shx經過123456軸旋轉後和末點座標系的Z軸還差幾度
	theta[AXIS7]=acos(MatrixMath::dot(V_x_rot1to6,V_H_hat_z)*(1/norm(V_x_rot1to6))*(1/norm(V_H_hat_z)));// theta(7)=acos(V_x_rot1to6'*V_H_hat_z/norm(V_x_rot1to6)/norm(V_H_hat_z));

	if (norm(Vn_xrot1to6_VHhatz - V_H_hat_x) <  DEF_VERY_SMALL)
        theta[AXIS7]=theta[AXIS7];
    else
        theta[AXIS7]=-theta[AXIS7];  
	
	memcpy(out_theta,theta,sizeof(theta));
	
	return 0;
}

