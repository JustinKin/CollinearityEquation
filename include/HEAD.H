#ifndef __HEAD_H
#define __HEAD_H

#include<iostream>
#include<memory>
#include<D:/QinJunyou/C/Eigen3/Eigen/Eigen>


class CollinearityEquation
{
public:
    //no point_Pixo
    CollinearityEquation(Eigen::Vector3f&,
                         Eigen::Vector2f&,
                         Eigen::Vector2f&,
                         Eigen::Vector3f&,
                         Eigen::Matrix3f&);
    //all
    CollinearityEquation(Eigen::Vector3f&,
                         Eigen::Vector2f&,
                         Eigen::Vector2f&,
                         Eigen::Vector2f&,
                         Eigen::Vector3f&,
                         Eigen::Matrix3f&);
    Eigen::Vector3f GetWorldCoordinate() const { return point_World; }
    Eigen::Vector2f GetPicoCoordinate() const { return point_Pixo; }
    Eigen::Vector2f GetPicPrinCoordinate() const { return point_PicPrin; }
    Eigen::Vector2f GetEquFoclen() const { return foclen_Equ; }
    Eigen::Vector3f GetTranVec() const { return tranT_Vec; }
    Eigen::Matrix3f GetRotMatrix() const { return rot_Mat; }
    Eigen::Matrix<float,3,4> GetParameters_in() const ;
    Eigen::Matrix4f GetParameters_out() const ;
    void ComputePoint_Pixo(const std::shared_ptr<CollinearityEquation>&);
    Eigen::Vector2f GetPoint_Pixo() const { return point_Pixo; }
    ~CollinearityEquation() = default;

private:
    Eigen::Vector3f point_World;
    Eigen::Vector2f point_Pixo;
    Eigen::Vector2f point_PicPrin;
    Eigen::Vector2f foclen_Equ;
    Eigen::Vector3f tranT_Vec;
    Eigen::Matrix3f rot_Mat;

};

std::shared_ptr<CollinearityEquation> Initialize();










#endif