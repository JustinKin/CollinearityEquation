#include<iostream>
#include<memory>
#include<vector>
#include<string>
#include <chrono>
#include<D:/QinJunyou/C/Eigen3/Eigen/Eigen>

#include "HEAD.H"

using namespace Eigen;
using namespace std;

//no point_Pixo
CollinearityEquation::CollinearityEquation( Eigen::Vector3f &point_World_,
                                            Eigen::Vector2f &point_PicPrin_,
                                            Eigen::Vector2f &foclen_Equ_,
                                            Eigen::Vector3f &tranT_Vec_,
                                            Eigen::Matrix3f &rot_Mat_ ) :
    point_World(point_World_),point_PicPrin(point_PicPrin_),
    foclen_Equ(foclen_Equ_),tranT_Vec(tranT_Vec_),rot_Mat(rot_Mat_){}
//all
CollinearityEquation::CollinearityEquation( Eigen::Vector3f &point_World_,
                                            Eigen::Vector2f &point_Pixo_,
                                            Eigen::Vector2f &point_PicPrin_,
                                            Eigen::Vector2f &foclen_Equ_,
                                            Eigen::Vector3f &tranT_Vec_,
                                            Eigen::Matrix3f &rot_Mat_ ) :
    point_World(point_World_),point_Pixo(point_Pixo_),point_PicPrin(point_PicPrin_),
    foclen_Equ(foclen_Equ_),tranT_Vec(tranT_Vec_),rot_Mat(rot_Mat_){}

shared_ptr<CollinearityEquation> Initialize()
{
    cout<<"please input World Coordinate: (X,Y,Z) \n";
    Vector3f point_World;
    cin>>point_World[0]>>point_World[1]>>point_World[2];

/*     cout<<"please input Pic Coordinate of opt: (~x,~y) \n";
    Eigen::Vector2f point_Pixo;
    cin>>point_Pixo[0]>>point_Pixo[1];
 */

    cout<<"please input Pic Principle Coordinate: (Cx,Cy) \n";
    Eigen::Vector2f point_PicPrin;
    cin>>point_PicPrin[0]>>point_PicPrin[1];

    cout<<"please input Equivalent Focal Length: (Fx,Fy) \n";
    Eigen::Vector2f foclen_Equ;
    cin>>foclen_Equ[0]>>foclen_Equ[1];

    cout<<"please input Translation Vector: (Tx,Ty,Tz) \n";
    Eigen::Vector3f tranT_Vec;
    cin>>tranT_Vec[0]>>tranT_Vec[1]>>tranT_Vec[2];

    cout<<"please input Rotation Matrix: (r0 ~ r8) \n";
    Eigen::Matrix3f rot_Mat;
    cin>>rot_Mat(0,0)>>rot_Mat(0,1)>>rot_Mat(0,2)
       >>rot_Mat(1,0)>>rot_Mat(1,1)>>rot_Mat(1,2)
       >>rot_Mat(2,0)>>rot_Mat(2,1)>>rot_Mat(2,2);


    return make_shared<CollinearityEquation>
            (CollinearityEquation(point_World,point_PicPrin,foclen_Equ,tranT_Vec,rot_Mat));
}