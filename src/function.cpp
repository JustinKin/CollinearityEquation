#include<iostream>
#include<memory>
#include<vector>
#include<string>
#include <chrono>
#include <sstream>
#include <fstream>
#include<D:/QinJunyou/C/Eigen3/Eigen/Eigen>

#include "HEAD.H"

// using namespace Eigen;
using namespace std;

CamPara::CamPara(Eigen::Vector2f& point_PicPrin_,
                 Eigen::Vector2f& foclen_Equ_,
                 Eigen::Vector3f& tranT_Vec_,
                 Eigen::Matrix3f& rot_Mat_) :
    point_PicPrin(point_PicPrin_), foclen_Equ(foclen_Equ_),tranT_Vec(tranT_Vec_), rot_Mat(rot_Mat_){}

Calculate::Calculate(vector<shared_ptr<CamPara>> Cams_) : Cams(Cams_) {}

WorldPara::WorldPara(vector<shared_ptr<CamPara>> Cams_,
                     vector<shared_ptr<vector<Eigen::Vector3f>>>& point_World_) :
    Calculate::Calculate(Cams_),point_World(point_World_){}

shared_ptr<Calculate> WorldPara::Initialize(const string &file_)
{
    auto files = ReadFiles(file_);
    string str("World Coordinate: (X,Y,Z)");
    CheckData(files,str);
    vector<shared_ptr<CamPara>> Cams;
    for(const auto &eachfile : files)
    {
        Eigen::Vector2f point_PicPrin;
        Eigen::Vector2f foclen_Equ;
        Eigen::Vector3f tranT_Vec;
        Eigen::Matrix3f rot_Mat;
        vector<shared_ptr<vector<Eigen::Vector3f>>> point_World;
        string line;
        for(int i = 1 ;i <12; i += 2)
        {
            istringstream iss((*eachfile)[i]);
            string s;
            // TODO: sepatrate case 1,3,5,7 to a individual function
            switch(i)
                {
                    case 1:
                        {
                            int j = 0;
                            while(iss >>s)
                                point_PicPrin[j++] = stof(s);
                        };
                        break;
                    case 3:
                        {
                            int j = 0;
                            while(iss >>s)
                                foclen_Equ[j++] = stof(s);
                        };
                        break;
                    case 5:
                        {
                            int j = 0;
                            while(iss >>s)
                                tranT_Vec[j++] = stof(s);
                        };
                        break;
                    case 7:
                        {
                            int c = 0;
                            while(iss >>s)
                                rot_Mat(0,c++) = stof(s);
                            for(int j = 1; j <3; ++j)
                            {
                                istringstream is((*eachfile)[++i]);
                                string s;
                                int k = 0;
                                while(is >>s)
                                    rot_Mat(j,k++) = stof(s);
                            }
                        };
                        break;
                    case 11:
                        {
                            Eigen::Vector3f point;
                            vector<Eigen::Vector3f> vec_point;
                            auto bg = (*eachfile).begin() + 11;
                            auto end = (*eachfile).end();
                            for( ; bg != end; ++bg)
                            {
                                istringstream is(*bg);
                                string s;
                                int j = 0;
                                while(is >> s)
                                    point[j++] = stof(s);
                                vec_point.push_back(point);
                            }
                            point_World.push_back(make_shared<vector<Eigen::Vector3f>>(vec_point));
                        };
                        break;
                    default:
                        {
                            cerr<<"read error!\n";
                            return nullptr;
                        }
                }
        }
        Cams.push_back(make_shared<CamPara>(CamPara(point_PicPrin,foclen_Equ,tranT_Vec,rot_Mat)));
    }
    return make_shared<WorldPara>(WorldPara(Cams,point_World));
}


PicPara_opt::PicPara_opt(vector<shared_ptr<CamPara>> Cams_,
                         vector<shared_ptr<vector<Eigen::Vector2f>>>& point_Pixo_) :
    Calculate::Calculate(Cams_),point_Pixo(point_Pixo_){}

vector<shared_ptr<vector<string>>> ReadFiles(const string &file_)
{
    int i =0;
    string line;
    vector<string> file;
    vector<shared_ptr<vector<string>>> vec_ptr;
    while(1)
    {
        string filename(file_ + to_string(i++) + ".txt");
        ifstream in(filename);
        if(in)
        {
            while(getline(in,line))
            {
                file.push_back(line);
            }
            in.close();
            vec_ptr.push_back(make_shared<vector<string>>(file));
        }
        else
        {
            return vec_ptr;
        }
    }
}

const void CheckData(vector<shared_ptr<vector<string>>> file, string &str_)
{
    int i = 1;
    for(const auto &c : file)
    {
        string s1("Pic Principle Coordinate: (Cx,Cy)");
        string s2("Equivalent Focal Length: (Fx,Fy)");
        string s3("Translation Vector: (Tx,Ty,Tz)");
        string s4("Rotation Matrix: (r0 ~ r8)");

        auto bg = (*c).cbegin();
        if(*bg != s1 || *(bg + 2) != s2 || *(bg + 4) != s3 || *(bg + 6) != s4 || *(bg + 10) != str_)
            throw("Data error: " + i);
        ++i;
    }
}

shared_ptr<vector<Eigen::Matrix<float,3,4>>>
    Calculate::GetCamPara_in(const shared_ptr<Calculate>& Cal_) const
{
    Eigen::Matrix<float,3,4>CamPara_in;
    vector<Eigen::Matrix<float,3,4>> vec_camIn;
    for(const auto &cam : Cal_->Cams)
    {
        CamPara_in<<cam->foclen_Equ[0], 0, cam->point_PicPrin[0], 0,
                    0, cam->foclen_Equ[1], cam->point_PicPrin[1], 0,
                    0, 0, 1, 0;
        vec_camIn.push_back(CamPara_in);
    }
    return make_shared<vector<Eigen::Matrix<float,3,4>>>(vec_camIn);
}

shared_ptr<vector<Eigen::Matrix4f>>
    Calculate::GetCamPara_out(const shared_ptr<Calculate>& Cal_) const
{
    Eigen::Matrix4f CamPara_out;
    vector<Eigen::Matrix4f> vec_camOut;
    for(const auto &cam : Cal_->Cams)
    {
    CamPara_out<<cam->rot_Mat(0,0), cam->rot_Mat(0,1), cam->rot_Mat(0,2), cam->tranT_Vec[0],
                 cam->rot_Mat(1,0), cam->rot_Mat(1,1), cam->rot_Mat(1,2), cam->tranT_Vec[1],
                 cam->rot_Mat(2,0), cam->rot_Mat(2,1), cam->rot_Mat(2,2), cam->tranT_Vec[2],
                                 0,                 0,                 0,                 1;
    vec_camOut.push_back(CamPara_out);
    }
    return make_shared<vector<Eigen::Matrix4f>>(vec_camOut);
}

void WorldPara::ComputePoint(const shared_ptr<Calculate>& WorldPara_)
{
    const auto camIn = WorldPara_->GetCamPara_in(WorldPara_);
    const auto camOut = WorldPara_->GetCamPara_out(WorldPara_);
    const auto point_World = WorldPara_->point_World;
    auto bgIn = (*camIn).begin();
    auto endIn = (*camIn).end();
    auto bgOut = (*camOut).begin();

}
















//no point_Pixo: (~x,~y)
CollinearityEquation::CollinearityEquation( Eigen::Vector3f &point_World_,
                                            Eigen::Vector2f &point_PicPrin_,
                                            Eigen::Vector2f &foclen_Equ_,
                                            Eigen::Vector3f &tranT_Vec_,
                                            Eigen::Matrix3f &rot_Mat_ ) :
    point_World(point_World_),point_PicPrin(point_PicPrin_),
    foclen_Equ(foclen_Equ_),tranT_Vec(tranT_Vec_),rot_Mat(rot_Mat_){}
//no point_World: (X,Y,Z)
CollinearityEquation::CollinearityEquation( Eigen::Vector2f &point_Pixo_,
                                            Eigen::Vector2f &point_PicPrin_,
                                            Eigen::Vector2f &foclen_Equ_,
                                            Eigen::Vector3f &tranT_Vec_,
                                            Eigen::Matrix3f &rot_Mat_ ) :
    point_Pixo(point_Pixo_),point_PicPrin(point_PicPrin_),
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

shared_ptr<CollinearityEquation> Initialize_CP(std::ifstream &cpin_)
{
    Eigen::Vector3f point_World;
    Eigen::Vector2f point_PicPrin;
    Eigen::Vector2f foclen_Equ;
    Eigen::Vector3f tranT_Vec;
    Eigen::Matrix3f rot_Mat;

    string line;
    vector<string> vline;

    while(getline(cpin_,line))
    {
        vline.push_back(line);
    }
    for(int i = 1 ;i <12; i += 2)
    {
        istringstream iss(vline[i]);
        string s;
        switch(i)
            {
                case 1:
                    {
                        int j = 0;
                        while(iss >>s)
                            point_World[j++] = stof(s);
                    };
                    break;
                case 3:
                    {
                        int j = 0;
                        while(iss >>s)
                            point_PicPrin[j++] = stof(s);
                    };
                    break;
                case 5:
                    {
                        int j = 0;
                        while(iss >>s)
                            foclen_Equ[j++] = stof(s);
                    };
                    break;
                case 7:
                    {
                        int j = 0;
                        while(iss >>s)
                            tranT_Vec[j++] = stof(s);
                    };
                    break;
                case 9:
                    {
                        int c = 0;
                        while(iss >>s)
                            rot_Mat(0,c++) = stof(s);
                        for(int j = 1; j <3; ++j)
                        {
                            istringstream is(vline[++i]);
                            string s;
                            int k = 0;
                            while(is >>s)
                                rot_Mat(j,k++) = stof(s);
                        }
                    };
                    break;
                default:
                    {
                        cerr<<"read error!\n";
                        return make_shared<CollinearityEquation> (CollinearityEquation());
                    }
            }
    }
    return make_shared<CollinearityEquation>
            (CollinearityEquation(point_World,point_PicPrin,foclen_Equ,tranT_Vec,rot_Mat));
}

shared_ptr<CollinearityEquation> Initialize_CW(std::ifstream &cwin_)
{
    Eigen::Vector2f point_Pixo;
    Eigen::Vector2f point_PicPrin;
    Eigen::Vector2f foclen_Equ;
    Eigen::Vector3f tranT_Vec;
    Eigen::Matrix3f rot_Mat;

    string line;
    vector<string> vline;

    while(getline(cwin_,line))
    {
        vline.push_back(line);
    }
    for(int i = 1 ;i <12; i += 2)
    {
        istringstream iss(vline[i]);
        string s;
        switch(i)
            {
                case 1:
                    {
                        int j = 0;
                        while(iss >>s)
                            point_Pixo[j++] = stof(s);
                    };
                    break;
                case 3:
                    {
                        int j = 0;
                        while(iss >>s)
                            point_PicPrin[j++] = stof(s);
                    };
                    break;
                case 5:
                    {
                        int j = 0;
                        while(iss >>s)
                            foclen_Equ[j++] = stof(s);
                    };
                    break;
                case 7:
                    {
                        int j = 0;
                        while(iss >>s)
                            tranT_Vec[j++] = stof(s);
                    };
                    break;
                case 9:
                    {
                        int c = 0;
                        while(iss >>s)
                            rot_Mat(0,c++) = stof(s);
                        for(int j = 1; j <3; ++j)
                        {
                            istringstream is(vline[++i]);
                            string s;
                            int k = 0;
                            while(is >>s)
                                rot_Mat(j,k++) = stof(s);
                        }
                    };
                    break;
                default:
                    {
                        cerr<<"read error!\n";
                        return make_shared<CollinearityEquation> (CollinearityEquation());
                    }
            }
    }
    return make_shared<CollinearityEquation>
            (CollinearityEquation(point_Pixo,point_PicPrin,foclen_Equ,tranT_Vec,rot_Mat));
}
Eigen::Matrix<float,3,4>
CollinearityEquation::GetParameters_in() const
{
    Eigen::Matrix<float,3,4>parameters_in;
    parameters_in<<foclen_Equ[0], 0, point_PicPrin[0], 0,
                    0, foclen_Equ[1], point_PicPrin[1], 0,
                    0, 0, 1, 0;
    return parameters_in;
}

Eigen::Matrix4f
CollinearityEquation::GetParameters_out() const
{
    Eigen::Matrix4f parameters_out;
    parameters_out<<rot_Mat(0,0), rot_Mat(0,1), rot_Mat(0,2), tranT_Vec[0],
                    rot_Mat(1,0), rot_Mat(1,1), rot_Mat(1,2), tranT_Vec[1],
                    rot_Mat(2,0), rot_Mat(2,1), rot_Mat(2,2), tranT_Vec[2],
                               0,            0,            0,            1;
    return parameters_out;
}

void CollinearityEquation::ComputePoint_Pixo(const shared_ptr<CollinearityEquation>& p)
{
    Eigen::Vector4f point_World1(point_World[0],point_World[1],point_World[2],1);
    Eigen::Vector3f tmp;
    tmp = ((p->GetParameters_in()) * (p->GetParameters_out()) * point_World1);
    p->point_Pixo[0] = tmp[0] / tmp[2];
    p->point_Pixo[1] = tmp[1] / tmp[2];
}

void ComputePoint_World(const shared_ptr<CollinearityEquation>& p1, const shared_ptr<CollinearityEquation>& p2)
{
    Eigen::Matrix<float,4,3> A;
    Eigen::Vector4f B;
    float   x1 = p1->GetPoint_Pixo()[0] - p1->GetPicPrinCoordinate()[0],
            y1 = p1->GetPoint_Pixo()[1] - p1->GetPicPrinCoordinate()[1];
    float   x2 = p2->GetPoint_Pixo()[0] - p2->GetPicPrinCoordinate()[0],
            y2 = p2->GetPoint_Pixo()[1] - p2->GetPicPrinCoordinate()[1];
    A(0,0) = (x1) * p1->GetRotMatrix()(2,0) - p1->GetEquFoclen()[0] * p1->GetRotMatrix()(0,0);
    A(1,0) = (y1) * p1->GetRotMatrix()(2,0) - p1->GetEquFoclen()[1] * p1->GetRotMatrix()(1,0);
    A(2,0) = (x2) * p2->GetRotMatrix()(2,0) - p2->GetEquFoclen()[0] * p2->GetRotMatrix()(0,0);
    A(3,0) = (y2) * p2->GetRotMatrix()(2,0) - p2->GetEquFoclen()[1] * p2->GetRotMatrix()(1,0);

    A(0,1) = (x1) * p1->GetRotMatrix()(2,1) - p1->GetEquFoclen()[0] * p1->GetRotMatrix()(0,1);
    A(1,1) = (y1) * p1->GetRotMatrix()(2,1) - p1->GetEquFoclen()[1] * p1->GetRotMatrix()(1,1);
    A(2,1) = (x2) * p2->GetRotMatrix()(2,1) - p2->GetEquFoclen()[0] * p2->GetRotMatrix()(0,1);
    A(3,1) = (y2) * p2->GetRotMatrix()(2,1) - p2->GetEquFoclen()[1] * p2->GetRotMatrix()(1,1);

    A(0,2) = (x1) * p1->GetRotMatrix()(2,2) - p1->GetEquFoclen()[0] * p1->GetRotMatrix()(0,2);
    A(1,2) = (y1) * p1->GetRotMatrix()(2,2) - p1->GetEquFoclen()[1] * p1->GetRotMatrix()(1,2);
    A(2,2) = (x2) * p2->GetRotMatrix()(2,2) - p2->GetEquFoclen()[0] * p2->GetRotMatrix()(0,2);
    A(3,2) = (y2) * p2->GetRotMatrix()(2,2) - p2->GetEquFoclen()[1] * p2->GetRotMatrix()(1,2);

    B[0] =  p1->GetEquFoclen()[0] * p1->GetTranVec()[0] - x1 * p1->GetTranVec()[2];
    B[1] =  p1->GetEquFoclen()[1] * p1->GetTranVec()[1] - y1 * p1->GetTranVec()[2];
    B[2] =  p2->GetEquFoclen()[0] * p2->GetTranVec()[0] - x2 * p2->GetTranVec()[2];
    B[3] =  p2->GetEquFoclen()[1] * p2->GetTranVec()[1] - y2 * p2->GetTranVec()[2];

    Eigen::Matrix<float,3,4> AT= A.transpose();
    Eigen::Matrix3f a = AT * A;
    Eigen::Vector3f b = AT * B;
    float a_D = a.determinant();
    for(int i = 0; i <3; ++i)
    {
        Eigen::Matrix3f tmp = a;
        a(0,i) = b[0];
        a(1,i) = b[1];
        a(2,i) = b[2];
        p1->point_World[i] = (tmp.determinant()) / a_D;
    }
}

void ShowResult_CP(shared_ptr<CollinearityEquation> p, ofstream &out)
{
    cout<<"\n================result================\n";
    cout<<"World Coordinate: (X,Y,Z):\n"<<p->GetWorldCoordinate()<<"\n\n"<<
    "Pic Principle Coordinate: (Cx,Cy):\n"<<p->GetPicPrinCoordinate()<<"\n\n"<<
    "Equivalent Focal Length: (Fx,Fy):\n"<<p->GetEquFoclen()<<"\n\n"<<
    "Translation Vector: (Tx,Ty,Tz):\n"<<p->GetTranVec()<<"\n\n"<<
    "Rotation Matrix: (r0 ~ r8):\n"<<p->GetRotMatrix()<<"\n\n";
    cout<<"Parameters_in:\n" << p->GetParameters_in()<<"\n\n";
    cout<<"Parameters_out:\n"<< p->GetParameters_out()<<"\n\n";
    p->ComputePoint_Pixo(p);
    cout<<"Pic Coordinate of opt: (~x,~y):\n"<<p->GetPoint_Pixo()<<"\n\n";

    out<<"\n================result================\n";
    out<<"World Coordinate: (X,Y,Z):\n"<<p->GetWorldCoordinate()<<"\n\n"<<
    "Pic Principle Coordinate: (Cx,Cy):\n"<<p->GetPicPrinCoordinate()<<"\n\n"<<
    "Equivalent Focal Length: (Fx,Fy):\n"<<p->GetEquFoclen()<<"\n\n"<<
    "Translation Vector: (Tx,Ty,Tz):\n"<<p->GetTranVec()<<"\n\n"<<
    "Rotation Matrix: (r0 ~ r8):\n"<<p->GetRotMatrix()<<"\n\n";
    out<<"Parameters_in:\n" << p->GetParameters_in()<<"\n\n";
    out<<"Parameters_out:\n"<< p->GetParameters_out()<<"\n\n";
    out<<"Pic Coordinate of opt: (~x,~y):\n"<<p->GetPoint_Pixo()<<"\n\n";
}

void ShowResult_CW(shared_ptr<CollinearityEquation> p1, shared_ptr<CollinearityEquation> p2, std::ofstream &out)
{
    cout<<"\n================result================\n";
    cout<<"Cam_1:\n";
    cout<<"Pic Coordinate of opt: (~x,~y):\n"<<p1->GetPoint_Pixo()<<"\n\n";
    cout<<"Pic Principle Coordinate: (Cx,Cy):\n"<<p1->GetPicPrinCoordinate()<<"\n\n"<<
    "Equivalent Focal Length: (Fx,Fy):\n"<<p1->GetEquFoclen()<<"\n\n"<<
    "Translation Vector: (Tx,Ty,Tz):\n"<<p1->GetTranVec()<<"\n\n"<<
    "Rotation Matrix: (r0 ~ r8):\n"<<p1->GetRotMatrix()<<"\n\n";
    cout<<"Parameters_in:\n" << p1->GetParameters_in()<<"\n\n";
    cout<<"Parameters_out:\n"<< p1->GetParameters_out()<<"\n\n";
    cout<<"\nCam_2:\n";
    cout<<"Pic Coordinate of opt: (~x,~y):\n"<<p2->GetPoint_Pixo()<<"\n\n";
    cout<<"Pic Principle Coordinate: (Cx,Cy):\n"<<p2->GetPicPrinCoordinate()<<"\n\n"<<
    "Equivalent Focal Length: (Fx,Fy):\n"<<p2->GetEquFoclen()<<"\n\n"<<
    "Translation Vector: (Tx,Ty,Tz):\n"<<p2->GetTranVec()<<"\n\n"<<
    "Rotation Matrix: (r0 ~ r8):\n"<<p2->GetRotMatrix()<<"\n\n";
    cout<<"Parameters_in:\n" << p2->GetParameters_in()<<"\n\n";
    cout<<"Parameters_out:\n"<< p2->GetParameters_out()<<"\n\n";
    cout<<"World Coordinate: (X,Y,Z):\n"<<p1->GetWorldCoordinate()<<"\n\n";

    out<<"\n================result================\n";
    out<<"Cam_1:\n";
    out<<"Pic Coordinate of opt: (~x,~y):\n"<<p1->GetPoint_Pixo()<<"\n\n";
    out<<"Pic Principle Coordinate: (Cx,Cy):\n"<<p1->GetPicPrinCoordinate()<<"\n\n"<<
    "Equivalent Focal Length: (Fx,Fy):\n"<<p1->GetEquFoclen()<<"\n\n"<<
    "Translation Vector: (Tx,Ty,Tz):\n"<<p1->GetTranVec()<<"\n\n"<<
    "Rotation Matrix: (r0 ~ r8):\n"<<p1->GetRotMatrix()<<"\n\n";
    out<<"Parameters_in:\n" << p1->GetParameters_in()<<"\n\n";
    out<<"Parameters_out:\n"<< p1->GetParameters_out()<<"\n\n";

    out<<"\nCam_2:\n";
    out<<"Pic Coordinate of opt: (~x,~y):\n"<<p2->GetPoint_Pixo()<<"\n\n";
    out<<"Pic Principle Coordinate: (Cx,Cy):\n"<<p2->GetPicPrinCoordinate()<<"\n\n"<<
    "Equivalent Focal Length: (Fx,Fy):\n"<<p2->GetEquFoclen()<<"\n\n"<<
    "Translation Vector: (Tx,Ty,Tz):\n"<<p2->GetTranVec()<<"\n\n"<<
    "Rotation Matrix: (r0 ~ r8):\n"<<p2->GetRotMatrix()<<"\n\n";
    out<<"Parameters_in:\n" << p2->GetParameters_in()<<"\n\n";
    out<<"Parameters_out:\n"<< p2->GetParameters_out()<<"\n\n";

    out<<"World Coordinate: (X,Y,Z):\n"<<p1->GetWorldCoordinate()<<"\n\n";

}
