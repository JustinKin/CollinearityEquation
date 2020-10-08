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
    Eigen::Vector3f GetWorldCoordinate() { return point_World; }
    Eigen::Vector2f GetPicoCoordinate() { return point_Pixo; }
    Eigen::Vector2f GetPicPrinCoordinate() { return point_PicPrin; }
    Eigen::Vector2f GetEquFoclen() { return foclen_Equ; }
    Eigen::Vector3f GetTranVec() { return tranT_Vec; }
    Eigen::Matrix3f GetRotMatrix() { return rot_Mat; }
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