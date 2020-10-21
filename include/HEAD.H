#ifndef __HEAD_H
#define __HEAD_H

#include<iostream>
#include<memory>
#include<D:/QinJunyou/C/Eigen3/Eigen/Eigen>

class WorldPara;

class CamPara
{

public:
    CamPara() = default;
    CamPara(Eigen::Vector2f& point_PicPrin_,
            Eigen::Vector2f& foclen_Equ_,
            Eigen::Vector3f& tranT_Vec_,
            Eigen::Matrix3f& rot_Mat_);
    Eigen::Vector2f GetPicPrinCoordinate() const { return point_PicPrin; }
    Eigen::Vector2f GetEquFoclen() const { return foclen_Equ; }
    Eigen::Vector3f GetTranVec() const { return tranT_Vec; }
    Eigen::Matrix3f GetRotMatrix() const { return rot_Mat; }
    Eigen::Matrix<float,3,4> GetParameters_in() const ;
    Eigen::Matrix4f GetParameters_out() const ;

    virtual ~CamPara();

private:
    Eigen::Vector2f point_PicPrin;
    // Equivalent Focal Length: (Fx,Fy)
    Eigen::Vector2f foclen_Equ;
    // Translation Vector: (Tx,Ty,Tz)
    Eigen::Vector3f tranT_Vec;
    // Rotation Matrix: (r0 ~ r8)
    Eigen::Matrix3f rot_Mat;
};

class Calculate : public CamPara
{
public:
    Calculate() = default;
    ~Calculate();


private:
    //multiple cameras
    void ComputePoint_World(const std::vector<std::shared_ptr<WorldPara>>&);
};


class WorldPara : public Calculate
{

public:
    WorldPara() = default;
    WorldPara(  Eigen::Vector3f& point_World_,
                Eigen::Vector2f& point_PicPrin_,
                Eigen::Vector2f& foclen_Equ_,
                Eigen::Vector3f& tranT_Vec_,
                Eigen::Matrix3f& rot_Mat_);
    Eigen::Vector3f GetWorldCoordinate() const { return point_World; }

    virtual ~WorldPara();

private:
    // World Coordinate: (X,Y,Z)
    Eigen::Vector3f point_World;

};

class PicPara_opt : public Calculate
{

public:
    PicPara_opt() = default;
    PicPara_opt(Eigen::Vector2f& point_Pixo_,
                Eigen::Vector2f& point_PicPrin_,
                Eigen::Vector2f& foclen_Equ_,
                Eigen::Vector3f& tranT_Vec_,
                Eigen::Matrix3f& rot_Mat_);
    Eigen::Vector2f GetPicoCoordinate() const { return point_Pixo; }

    virtual ~PicPara_opt();

private:
    // Pic Coordinate of opt: (~x,~y)
    Eigen::Vector2f point_Pixo;

};


class CollinearityEquation
{
friend void ComputePoint_World(const std::shared_ptr<CollinearityEquation>&,const std::shared_ptr<CollinearityEquation>&);

public:
    CollinearityEquation() = default;
    //no point_Pixo: (~x,~y)
    CollinearityEquation(Eigen::Vector3f& point_World_,
                         Eigen::Vector2f& point_PicPrin_,
                         Eigen::Vector2f& foclen_Equ_,
                         Eigen::Vector3f& tranT_Vec_,
                         Eigen::Matrix3f& rot_Mat_);
    //no point_World: (X,Y,Z)
    CollinearityEquation(Eigen::Vector2f& point_Pixo_,
                         Eigen::Vector2f& point_PicPrin_,
                         Eigen::Vector2f& foclen_Equ_,
                         Eigen::Vector3f& tranT_Vec_,
                         Eigen::Matrix3f& rot_Mat_);
    //all
    CollinearityEquation(Eigen::Vector3f& point_World_,
                         Eigen::Vector2f& point_Pixo_,
                         Eigen::Vector2f& point_PicPrin_,
                         Eigen::Vector2f& foclen_Equ_,
                         Eigen::Vector3f& tranT_Vec_,
                         Eigen::Matrix3f& rot_Mat_);
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
    // World Coordinate: (X,Y,Z)
    Eigen::Vector3f point_World;
    // Pic Coordinate of opt: (~x,~y)
    Eigen::Vector2f point_Pixo;
    // Pic Principle Coordinate: (Cx,Cy)
    Eigen::Vector2f point_PicPrin;
    // Equivalent Focal Length: (Fx,Fy)
    Eigen::Vector2f foclen_Equ;
    // Translation Vector: (Tx,Ty,Tz)
    Eigen::Vector3f tranT_Vec;
    // Rotation Matrix: (r0 ~ r8)
    Eigen::Matrix3f rot_Mat;

};

std::shared_ptr<CollinearityEquation> Initialize_CP(std::ifstream &ComputePoint_Pixo_);
std::shared_ptr<CollinearityEquation> Initialize_CW(std::ifstream &ComputePoint_World_);

void ShowResult_CP(std::shared_ptr<CollinearityEquation> cp_, std::ofstream &out_);
void ShowResult_CW(std::shared_ptr<CollinearityEquation> cw1_, std::shared_ptr<CollinearityEquation> cw2_, std::ofstream &out_);
void ComputePoint_World(const std::shared_ptr<CollinearityEquation>&,const std::shared_ptr<CollinearityEquation>&);









#endif