/*
TODO:
1. void FixAberration for cali class
2. add noise for forward 2 class
 */


#ifndef __HEAD_H
#define __HEAD_H
#include <iostream>
#include <memory>
#include <D:/QinJunyou/C/Eigen3/Eigen/Eigen>

class WorldPara;
class PicPara_opt;
class CamPara
{
public:
    CamPara() = default;
    CamPara(Eigen::Vector2f &point_PicPrin_,
            Eigen::Vector2f &foclen_Equ_,
            Eigen::Vector3f &tranT_Vec_,
            Eigen::Matrix3f &rot_Mat_);
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
    virtual ~Calculate() = default;

    std::vector<std::shared_ptr<CamPara>> GetCamPara() const { return Cams; }
    virtual void ComputeCamPara_in();
    virtual void ComputeCamPara_out();

// protected:
    // parameters of cameras
    std::vector<std::shared_ptr<CamPara>> Cams;
    std::shared_ptr<std::vector<Eigen::Matrix<float, 3, 4>>> CamPara_in;
    std::shared_ptr<std::vector<Eigen::Matrix4f>> CamPara_out;
    // Aberration coefficients:（k0~k4）
    std::shared_ptr<std::vector<Eigen::Matrix<float,5,1>>> coe_Aberr;
private:
    //read a list of txt for Initialize
    virtual void Initialize(std::shared_ptr<Calculate>, const std::string &file_) = 0;
    //multiple cameras
    virtual void ComputePoint(const std::shared_ptr<Calculate> &) = 0;
    // fix Aberration
    virtual void FixAberration(const std::shared_ptr<Calculate> &) = 0;
    //save and show results
    virtual void ShowResult(const std::shared_ptr<Calculate> &, const std::string &outfile_) = 0;
};

class WorldPara : public Calculate
{
    friend class Calculate;

public:
    WorldPara() = default;
    std::vector<std::shared_ptr<std::vector<Eigen::Vector3f>>> GetWorldCoordinate() const { return point_World; }
    void Initialize(std::shared_ptr<Calculate> WorldPara_, const std::string &file_) override;
    void ComputePoint(const std::shared_ptr<Calculate> &WorldPara_) override;
    void ShowResult(const std::shared_ptr<Calculate> &WorldPara_, const std::string &outfile_) override;
    void FixAberration(const std::shared_ptr<Calculate> &WorldPara_) override;
    ~WorldPara() = default;

private:
    // World Coordinate: (X,Y,Z)
    std::vector<std::shared_ptr<std::vector<Eigen::Vector3f>>> point_World;
    // the result is Pic Coordinate of opt: (~x,~y)
    std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>> point_Pixo;
    // Pic Coordinate of real: (x,y)
    std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>> point_Pix;
};

class PicPara_opt : public Calculate
{
    friend class Calculate;

public:
    PicPara_opt() = default;
    std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>> GetPicoCoordinate() const { return point_Pixo; }
    void Initialize(std::shared_ptr<Calculate> PicPara_opt_, const std::string &file_) override;
    void ComputePoint(const std::shared_ptr<Calculate> &PicPara_opt_) override;
    void ShowResult(const std::shared_ptr<Calculate> &PicPara_opt_, const std::string &outfile_) override;
    void FixAberration(const std::shared_ptr<Calculate> &PicPara_opt_) override;
    ~PicPara_opt() = default;

private:
    // Pic Coordinate of opt: (~x,~y)
    std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>> point_Pixo;
    // Pic Coordinate of real: (x,y)
    std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>> point_Pix;
    // the result is World Coordinate: (X,Y,Z)
    std::vector<Eigen::Vector3f> point_World;
};

class Calibration : public Calculate
{
    friend class Calculate;
public:
    Calibration() = default;
    void Initialize(std::shared_ptr<Calculate> Calibration_, const std::string &file_) override;
    void ComputePoint(const std::shared_ptr<Calculate> &Calibration_) override;
    void ShowResult(const std::shared_ptr<Calculate> &Calibration_, const std::string &outfile_) override;
    void FixAberration(const std::shared_ptr<Calculate> &Calibration_) override;
    ~Calibration() = default;


private:
    // World Coordinate: (X,Y,Z)
    std::vector<std::shared_ptr<std::vector<Eigen::Vector3f>>> point_World;
    // Pic Coordinate of real: (x,y)
    std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>> point_Pix;
    // Pic Coordinate of opt: (~x,~y)
    std::vector<std::shared_ptr<std::vector<Eigen::Vector2f>>> point_Pixo;

};

std::vector<std::shared_ptr<std::vector<std::string>>> ReadFiles(const std::string &file_);
void CheckData(std::vector<std::shared_ptr<std::vector<std::string>>> file_, const std::string &str_);
#endif