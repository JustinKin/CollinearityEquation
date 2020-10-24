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
    Calculate(std::vector<std::shared_ptr<CamPara>> Cams_);
    // Calculate(std::vector<std::shared_ptr<CamPara>> Cams_,
    //           std::vector<std::shared_ptr<std::vector<Eigen::Vector3f>>> point_World_);
    // Calculate(std::vector<std::shared_ptr<CamPara>> Cams_,
    //           std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>> point_Pixo_);
    virtual ~Calculate() = default;

    // parameters of cameras
    std::vector<std::shared_ptr<CamPara>> Cams;
    std::shared_ptr<std::vector<Eigen::Matrix<float,3,4>>> CamPara_in;
    std::shared_ptr<std::vector<Eigen::Matrix4f>> CamPara_out;
    std::vector<std::shared_ptr<CamPara>> GetCamPara() const { return Cams; }
    virtual void ComputeCamPara_in();
    virtual void ComputeCamPara_out();

private:
    //read a list of txt for Initialize
    virtual void Initialize(std::shared_ptr<Calculate>,const std::string &file_) =0;
    //multiple cameras
    virtual void ComputePoint(const std::shared_ptr<Calculate>&) = 0;
    //save and show results
    virtual void ShowResult(const std::shared_ptr<Calculate>&, const std::string &outfile_) = 0;
};

// class


class WorldPara : public Calculate
{
friend class Calculate;
public:
    WorldPara() = default;
    // WorldPara(  std::vector<std::shared_ptr<CamPara>> Cams_,
    //             std::vector<std::shared_ptr<std::vector<Eigen::Vector3f>>>& point_World_);
    std::vector<std::shared_ptr<std::vector<Eigen::Vector3f>>> GetWorldCoordinate() const { return point_World; }
    void Initialize(std::shared_ptr<Calculate> WorldPara_,const std::string &file_) override;
    void ComputePoint(const std::shared_ptr<Calculate>& WorldPara_) override;
    void ShowResult(const std::shared_ptr<Calculate>& WorldPara_, const std::string &outfile_) override;
    ~WorldPara() = default;

private:
    // World Coordinate: (X,Y,Z)
    std::vector<std::shared_ptr<std::vector<Eigen::Vector3f>>> point_World;
    // the result is Pic Coordinate of opt: (~x,~y)
    std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>> point_Pixo;
};

class PicPara_opt : public Calculate
{
friend class Calculate;
public:
    PicPara_opt() = default;
    // PicPara_opt(std::vector<std::shared_ptr<CamPara>> Cams_,
    //             std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>>& point_Pixo_);
    std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>> GetPicoCoordinate() const { return point_Pixo; }
    void Initialize(std::shared_ptr<Calculate> PicPara_opt_,const std::string &file_) override;
    void ComputePoint(const std::shared_ptr<Calculate>& PicPara_opt_) override;
    void ShowResult(const std::shared_ptr<Calculate>& PicPara_opt_, const std::string &outfile_) override;
    ~PicPara_opt() = default;

private:
    // Pic Coordinate of opt: (~x,~y)
    std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>> point_Pixo;
    // the result is World Coordinate: (X,Y,Z)
    std::vector<Eigen::Vector3f> point_World;
};

std::vector<std::shared_ptr<std::vector<std::string>>> ReadFiles(const std::string &file_);
void CheckData(std::vector<std::shared_ptr<std::vector<std::string>>> file_ , const std::string &str_) ;









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