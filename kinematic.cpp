// kinematic.cpp : 定義主控台應用程式的進入點。
//
//#include "stdafx.h"



//可傳二為陣列指標
//void cRobot_Joint::MatrixMultiply_4x4(float (*aMatrix)[DEF_MATRIX_COL_LENGTH],float(*bMatrix)[DEF_MATRIX_COL_LENGTH],float(*product)[DEF_MATRIX_COL_LENGTH])
//{
//
//	//float aMatrix[DEF_MATRIX_ROW_LENGTH][DEF_MATRIX_COL_LENGTH] = {{1,2,3,4}, {5,6,7,8},{1,2,3,4}, {5,6,7,8}};
//    //float bMatrix[DEF_MATRIX_ROW_LENGTH][DEF_MATRIX_COL_LENGTH] = {{1,2,3,4}, {5,6,7,8},{1,2,3,4}, {5,6,7,8}};
//    //float product[DEF_MATRIX_ROW_LENGTH][DEF_MATRIX_COL_LENGTH] = {0};
//
//    for (int row = 0; row < DEF_MATRIX_ROW_LENGTH; row++) 
//	{
//        for (int col = 0; col < DEF_MATRIX_COL_LENGTH; col++) 
//		{
//            // Multiply the row of A by the column of B to get the row, column of product.
//            for (int inner = 0; inner < DEF_MATRIX_COL_LENGTH; inner++) 
//			{
//                product[row][col] += aMatrix[row][inner] * bMatrix[inner][col];
//            }  
//        }
//    }
//}

//using namespace std;
//15
//16void printTwoDimDynamicArray(vector<vector<int> > ivec) {
//17  for(int y = 0; y != ivec.size(); ++y) {
//18    for(int x = 0; x != ivec[0].size(); ++x) {
//19      cout << ivec[y][x] << " ";
//20    }
//21    cout << endl;
//22  }
//23}
//24
//25int main() {
//26  const int sizex = 3;
//27  const int sizey = 2;
//28  vector<vector<int> > ivec(sizey, vector<int>(sizex));
//29  
//30  for(int y = 0; y != sizey; ++y) {
//31    for(int x = 0; x != sizex; ++x) {
//32      ivec[y][x] = y + x;
//33    }
//34  }
//35  
//36  printTwoDimDynamicArray(ivec);
//37  
//38  getch();
//39}


#include <vector>
using namespace std;
#include "Matrix.h"
#include "MatrixMath.h"


#define _USE_MATH_DEFINES // for C++
#include <math.h>
//int main()
//{
//	//vectro 範例測試
//	//vector<int> v, v1, v2;
//	//int array[] = {0,1,2,3,4};
//
//	//vector<int> v1;
//	//vector<double> v2;
//	//vector<int> v3_ini5(3,5);//將v設成3個元素，每個元素都設5，
//	//// 清除內容, 重新設大小
//	//v1.clear(), v2.clear();
//	//v1.resize(5);
//	//v2.resize(10);
//
//	//// 新增元素
//	//int i;
//	////for(i=0; i<v1.size(); i++) v1.push_back(i);
//	//for(i=0; i<v2.size(); i++) v2[i] = double(i);
//	//vector<int> row;
//
//	Matrix matrix_a(2,2);
//
//	vector<vector<float>> abc;
//
//	matrix_a(1,1)=0.5;
//	matrix_a(1,2)=1;
//	matrix_a(2,1)=1.5;
//	matrix_a(2,2)=2;
//
//	float a=matrix_a(1,1);
//	float b=matrix_a(1,2);
//	float c=matrix_a(2,1);
//	float d=matrix_a(2,2);
//
//	Matrix matrix_b(3,3);
//
//	// Fill Matrix with data.
//    matrix_b << 1  << 2  << 3
//             << 4  << 5  << 6
//             << 7  << 8  << 9;
//
//	matrix_b.print();
//
//	getchar();
//	return 0;
//}




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

//function R_a=Rogridues(theta,V_A)
//
//R_a=[   cos(theta)+V_A(1)^2*(1-cos(theta))              V_A(1)*V_A(2)*(1-cos(theta))-V_A(3)*sin(theta)   V_A(1)*V_A(3)*(1-cos(theta))+V_A(2)*sin(theta)     0;
//        V_A(1)*V_A(2)*(1-cos(theta))+V_A(3)*sin(theta)  cos(theta)+V_A(2)^2*(1-cos(theta))               V_A(2)*V_A(3)*(1-cos(theta))-V_A(1)*sin(theta)     0;
//        V_A(1)*V_A(3)*(1-cos(theta))-V_A(2)*sin(theta)  V_A(2)*V_A(3)*(1-cos(theta))+V_A(1)*sin(theta)   cos(theta)+V_A(3)^2*(1-cos(theta))                 0;
//        0                                               0                                                0                                                  1
//    ];
//end

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
enum{
	AXIS1=0,
	AXIS2,
	AXIS3,
	AXIS4,
	AXIS5,
	AXIS6,
	AXIS7
};

int IK_7DOF_stanley(const float l1,const float l2,const float l3,const float x_base,const float y_base,const float z_base,const float x_end,const float y_end,const float z_end,const float alpha,const float beta,const float gamma,const float Rednt_alpha,float* theta)
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



	Matrix temp(4,4);//temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst)) *[Rednt_cir_R*V_beta_hat;1];  %Rednt_alpha的方向和論文上的方向性相反
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
	

	V_temp_f=MatrixMath::Inv(A1_4)*V_r_wst_u_4x1;//V_temp_f=inv(A1_4)*[V_r_wst_u;1]; %(3.31) 這個是補一列1上去的意思,need fix
	theta[AXIS5]=atan2(V_temp_f(2,1),V_temp_f(1,1));//theta(5)=atan2(V_temp_f(2),V_temp_f(1));

	
	//theat 6
	Matrix V_r_wst_r(3,1);
	V_r_wst_r=V_r_wst+V_H_hat_y;

	Matrix A1_5(4,4);
	A1_5=A1_4*MatrixMath::RotZ(theta[AXIS5])*MatrixMath::Tz(-l2);//A1_5=A1_4*Rz(theta(5))*Tz(-L2);
	
	Matrix V_temp_g(4,4);
	Matrix V_r_wst_r_4x1(4,1);
	V_r_wst_r_4x1.Vec_ext_1_row(V_r_wst_r,1);
	
	V_temp_g=MatrixMath::Inv(A1_5)*V_r_wst_r_4x1; //V_temp_g=inv(A1_5)*[V_r_wst_r;1]; %(3.38)  這個是補一列1上去的意思,need fix
	
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



void fn1( void );
void fn1()  
{  
   printf( "next.\n" );  
   getchar();
}  


//測試結果
//1.目前det 和inverse都有問題
//2.DeleteRow DeleteColumn也有問題
//3.anotherMatrix *=  myMatrix; 目前修改後可以，但是不確定運算子多載這樣修改對不隊
//
//int main() 
//{
//	
//    //DigitalOut myled(LED1);
//    //Timer t,t2;
//    
//    //t.start();
//	float theta[7]={0};
//	int rt = IK_7DOF_stanley(L1,L2,L3,0,0,0,60,0,0,0,0,0,(float)-M_PI*0.5,theta);
////---
//    Matrix myMatrix(3,3);
//    Matrix anotherMatrix;
//
//    // Fill Matrix with data.
//    myMatrix << 1  << 2  << 3
//             << 4  << 5  << 6
//             << 7  << 8  << 9;
//
//    printf( "\nmyMatrix:\n\n");
//    myMatrix.print();
//    printf( "\n" );
//
//	/*Matrix myMatrix2(3,3);
//	myMatrix2 << 1  << 0  << 0
//             << 0  << 1  << 0
//             << 0  << 0  << 1;*/
//	float mydet = MatrixMath::det( myMatrix );
//	
//	printf( "det mymatrix=%f:\n\n",mydet);
//    
//    
//    
//    // Matrix operations //
//
//    // Add 5 to negative Matrix 
//    anotherMatrix = - myMatrix + 5;
//
//    printf( "Result Matrix: anotherMatrix = - myMatrix + 5\n\n" );
//    anotherMatrix.print();
//    printf( "\n" );
//    
//    // Matrix Multiplication *
//    anotherMatrix *=  myMatrix;
//
//    printf( "\nanotherMatrix = anotherMatrix * myMatrix\n\n" );
//    anotherMatrix.print();
//    printf( "\n" );
//    
//    // Scalar Matrix Multiplication anotherMatrix *= 0.5
//    anotherMatrix *= 0.5;
//
//    printf( "\nResult Matrix *= 0.5:\n\n" );
//    anotherMatrix.print();
//    printf( "    _______________________________ \n" );
//
//
//    printf("\n\n *** MEMBER OPERATIONS *** \n\n");
//
//    //Copy myMatrix
//    Matrix temp( myMatrix );
//
//    // Resize Matrix
//    temp.Resize(4,4);
//    printf("\nAdded one Column, one Row to the limits of myMatrix saved in temp Matrix\n");
//    temp.print();
//
//    //Delete those new elements, we don't need them anyway.
//    Matrix::DeleteRow( temp, 4 );
//    Matrix::DeleteCol( temp, 4 );
//
//    printf("\nBack to normal\n");
//    temp.print();
//
//    
//    // Make room at the begining of Matrix
//    Matrix::AddRow( temp, 1 );
//    Matrix::AddCol( temp, 1 );
//    
//    printf("\nAdded Just one Row and column to the beginning\n");
//    temp.print();
//
//    // Take the second Row as a new Matrix
//    anotherMatrix = Matrix::ExportRow( temp, 2 );
//    printf("\nExport Second Row \n");
//    anotherMatrix.print();
//
//    // The second Column as a new Matrix, then transpose it to make it a Row
//    anotherMatrix = Matrix::ExportCol( temp, 2 );
//    anotherMatrix = MatrixMath::Transpose( anotherMatrix );
//    printf("\nAnd Export Second Column and Transpose it \n");
//    anotherMatrix.print();
//
//    // This will Check to see if your are reduce to a single Row or Column
//    temp = Matrix::ToPackedVector( myMatrix );
//    printf("\nInitial Matrix turned into a single Row\n");
//    temp.print();
//           
//    //  Matrix Math  //
//    printf("\n\n *** Matrix Inverse and Determinant ***\n");
//    
//    //Matrix BigMat( 8, 8 );
//    //
//    //BigMat   << 1 << 0.3 << 1.0 << 1 << 3 << 0.5 << 7.12 << 899
//    //         << 2 << 3.2 << 4.1 << 0 << 4 << 0.8 << 9.26 << 321
//    //         << 5 << 6.0 << 1   << 1 << 2 << 7.4 << 3.87 << 562
//    //         << 1 << 0.0 << 2.7 << 1 << 1 << 4.6 << 1.21 << 478
//    //         << 2 << 3.7 << 48  << 2 << 0 << 77  << 0.19 << 147
//    //         << 1 << 1.0 << 3.8 << 7 << 1 << 9.9 << 7.25 << 365
//    //         << 9 << 0.9 << 2.7 << 8 << 0 << 13  << 4.16 << 145
//    //         << 7 << 23  << 28  << 9 << 9 << 1.7 << 9.16 << 156;
//
//    //printf( "\nBigMat:\n");
//    //BigMat.print();
//    //printf( "\n" );
//
//    //t2.start();
//    //float determ = MatrixMath::det( BigMat );//有問題
//
//    //Matrix myInv = MatrixMath::Inv( BigMat );
//    ////t2.stop();
//
//    //printf( "\nBigMat's Determinant is: %f \n", determ);
//    //printf( "\n" );
//    //
//    //printf( "\nBigMat's Inverse is:\n");
//    //myInv.print();
//    //printf( "\n" );
//
//    //***  Homogenous Transformations **//
//    
//    printf( "\n\n *** TRANSFORMATIONS *** \n\n");
//
//    Matrix rot;
//
//    printf( " RotX  0.5 rad \n" );
//    rot = MatrixMath::RotX(0.5);
//    rot.print();
//    printf( "    _______________________________ \n\n" );
//
//    printf( " RotY  0.5 rad \n" );
//    rot = MatrixMath::RotY(0.5);
//    rot.print();
//    printf( "    _______________________________ \n\n" );
//
//    printf( " RotZ  0.5 rad \n" );
//    rot = MatrixMath::RotZ(0.5);
//    rot.print();
//    printf( "    _______________________________ \n\n" );

    //printf( " Transl  5x 3y 4z\n" ); //有問題
    //rot = MatrixMath::Transl( 5, 3, 4 );
    //rot.print();
    //printf( "    _______________________________ \n\n" );
	
 //---
 
    //t.stop();
    
    //float bigtime = t2.read();
    //float average = 12.149647 - bigtime;
        
    //printf( "\n\nThe time for all those operations in mbed was : %f seconds\n", t.read() );
    //printf( "\nOnly operations witout any print takes: 12.149647 seconds\n" );
    //printf( "\nDue to the 8x8 matrix alone takes: %f \n",bigtime );
    //printf( "\nSo normal 4x4 matrix ops: %f\n", average );
           
    //while(1) {
    //    myled = 1;
    //    wait(0.2);
    //    myled = 0;
    //    wait(0.2);
    //}
//}
