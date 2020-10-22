#ifndef __HEAD_H
#define __HEAD_H
#include<iostream>
#include<memory>
#include<D:/QinJunyou/C/Eigen3/Eigen/Eigen>

class WorldPara;
class PicPara_opt;
class CamPara
{
public:
    CamPara() = default;
    CamPara(Eigen::Vector2f& point_PicPrin_,
            Eigen::Vector2f& foclen_Equ_,
            Eigen::Vector3f& tranT_Vec_,
            Eigen::Matrix3f& rot_Mat_);
    ~CamPara() = default;

    // Pic Principle Coordinate: (Cx,Cy)
    Eigen::Vector2f point_PicPrin;
    // Equivalent Focal Length: (Fx,Fy)
    Eigen::Vector2f foclen_Equ;
    // Translation Vector: (Tx,Ty,Tz)
    Eigen::Vector3f tranT_Vec;
    // Rotation Matrix: (r0 ~ r8)
    Eigen::Matrix3f rot_Mat;
};

class Calculate
{
public:
    Calculate() = default;
    ~Calculate() = default;
    std::vector<std::shared_ptr<CamPara>> GetCamPara() const { return Cams; }

private:
    // parameters of cameras
    std::vector<std::shared_ptr<CamPara>> Cams;
    //multiple cameras
    virtual void ComputePoint(const std::shared_ptr<Calculate>&) = 0;
    //save and show results
    virtual void ShowResult(const std::shared_ptr<Calculate>&, std::ofstream &) = 0;
};


class WorldPara : public Calculate
{
friend class Calculate;
public:
    WorldPara() = default;
    WorldPara(  std::vector<std::shared_ptr<CamPara>> Cams_,
                Eigen::Vector3f& point_World_);
    std::vector<Eigen::Vector3f> GetWorldCoordinate() const { return point_World; }
    void ComputePoint(const std::shared_ptr<Calculate>& WorldPara_) override;
    void ShowResult(const std::shared_ptr<Calculate>& WorldPara_, std::ofstream & out_) override;
    ~WorldPara() = default;

private:
    // World Coordinate: (X,Y,Z)
    std::vector<Eigen::Vector3f> point_World;
    // the result is Pic Coordinate of opt: (~x,~y)
    std::vector<Eigen::Vector2f> point_Pixo;

};

class PicPara_opt : public Calculate
{
friend class Calculate;
public:
    PicPara_opt() = default;
    PicPara_opt(std::vector<std::shared_ptr<CamPara>> Cams_,
                Eigen::Vector2f& point_Pixo_);
    std::vector<Eigen::Vector2f> GetPicoCoordinate() const { return point_Pixo; }
    void ComputePoint(const std::shared_ptr<Calculate>& PicPara_opt_) override;
    void ShowResult(const std::shared_ptr<Calculate>& PicPara_opt_, std::ofstream & out_) override;
    ~PicPara_opt() = default;

private:
    // Pic Coordinate of opt: (~x,~y)
    std::vector<Eigen::Vector2f> point_Pixo;
    // the result is World Coordinate: (X,Y,Z)
    std::vector<Eigen::Vector3f> point_World;

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