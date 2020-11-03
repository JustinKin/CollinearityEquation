#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <random>
#include <ctime>
#include <cmath>
#include <D:/QinJunyou/C/Eigen3/Eigen/Eigen>
#include<D:/QinJunyou/C/Eigen3/Eigen/LU>

#include "HEAD.H"

using namespace std;

CamPara::CamPara(Eigen::Vector2f &point_PicPrin_,
                 Eigen::Vector2f &foclen_Equ_,
                 Eigen::Vector3f &tranT_Vec_,
                 Eigen::Matrix3f &rot_Mat_) : point_PicPrin(point_PicPrin_), foclen_Equ(foclen_Equ_), tranT_Vec(tranT_Vec_), rot_Mat(rot_Mat_) {}

Calculate::Calculate(vector<shared_ptr<CamPara>> Cams_) : Cams(Cams_) {}

void WorldPara::Initialize(std::shared_ptr<Calculate> WorldPara_, const string &file_)
{
    // read in coe_Aberration
    string file_coeAberr("coe_Aberration_");
    auto files_Ab = ReadFiles(file_coeAberr);
    Eigen::Matrix<float,5,1> coeAberr;
    vector<Eigen::Matrix<float,5,1>> vec_coeAberr;
    // only a txt of coe_Aberration
    for(const auto &coe : *(files_Ab[0]))
    {
        // cout << "= = "<<endl;
        istringstream iss(coe);
        string s;
        int i = 0;
        while(iss >> s)
            coeAberr(i++,0) = stof(s);
        vec_coeAberr.push_back(coeAberr);
    }
    WorldPara_->coe_Aberr = make_shared<vector<Eigen::Matrix<float,5,1>>>(vec_coeAberr);
    // read in world parameters
    auto files = ReadFiles(file_);
    string str("World Coordinate: (X,Y,Z)");
    CheckData(files, str);
    for (const auto &eachfile : files)
    {
        Eigen::Vector2f point_PicPrin;
        Eigen::Vector2f foclen_Equ;
        Eigen::Vector3f tranT_Vec;
        Eigen::Matrix3f rot_Mat;
        // string line;
        for (int i = 1; i < 12; i += 2)
        {
            istringstream iss((*eachfile)[i]);
            string s;
            // TODO: sepatrate case 1,3,5,7 to a individual function
            switch (i)
            {
            case 1:
            {
                int j = 0;
                while (iss >> s)
                    point_PicPrin[j++] = stof(s);
            };
            break;
            case 3:
            {
                int j = 0;
                while (iss >> s)
                    foclen_Equ[j++] = stof(s);
            };
            break;
            case 5:
            {
                int j = 0;
                while (iss >> s)
                    tranT_Vec[j++] = stof(s);
            };
            break;
            case 7:
            {
                int c = 0;
                while (iss >> s)
                    rot_Mat(0, c++) = stof(s);
                for (int j = 1; j < 3; ++j)
                {
                    istringstream is((*eachfile)[++i]);
                    string s;
                    int k = 0;
                    while (is >> s)
                        rot_Mat(j, k++) = stof(s);
                }
            };
            break;
            case 11:
            {
                Eigen::Vector3f point;
                vector<Eigen::Vector3f> vec_point;
                auto bg = (*eachfile).begin() + 11;
                auto end = (*eachfile).end();
                for (; bg != end; ++bg)
                {
                    istringstream is(*bg);
                    string s;
                    int j = 0;
                    while (is >> s)
                        point[j++] = stof(s);
                    vec_point.push_back(point);
                }
                this->point_World.push_back(make_shared<vector<Eigen::Vector3f>>(vec_point));
            };
            break;
            default:
            {
                cerr << "read error!\n";
            }
            break;
            }
        }
        WorldPara_->Cams.push_back(make_shared<CamPara>(CamPara(point_PicPrin, foclen_Equ, tranT_Vec, rot_Mat)));
    }
}

vector<shared_ptr<vector<string>>> ReadFiles(const string &file_)
{
    int i = 1;
    string line;
    vector<shared_ptr<vector<string>>> vec_ptr;
    while (1)
    {
        vector<string> file;
        string filename(file_ + to_string(i++) + ".txt");
        ifstream in(filename);
        if (in)
        {
            while (getline(in, line))
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

void CheckData(vector<shared_ptr<vector<string>>> file, const string &str_)
{
    int i = 1;
    for (const auto &c : file)
    {
        string s1("Pic Principle Coordinate: (Cx,Cy)");
        string s2("Equivalent Focal Length: (Fx,Fy)");
        string s3("Translation Vector: (Tx,Ty,Tz)");
        string s4("Rotation Matrix: (r0 ~ r8)");

        auto bg = (*c).cbegin();
        if (*bg != s1 || *(bg + 2) != s2 || *(bg + 4) != s3 || *(bg + 6) != s4 || *(bg + 10) != str_)
            throw("Data error: " + i);
        ++i;
    }
}

void Calculate::ComputeCamPara_in()
{
    Eigen::Matrix<float, 3, 4> CamPara_in;
    vector<Eigen::Matrix<float, 3, 4>> vec_camIn;
    for (const auto &cam : this->Cams)
    {
        CamPara_in << cam->foclen_Equ[0], 0, cam->point_PicPrin[0], 0,
            0, cam->foclen_Equ[1], cam->point_PicPrin[1], 0,
            0, 0, 1, 0;
        vec_camIn.push_back(CamPara_in);
    }
    this->CamPara_in = make_shared<vector<Eigen::Matrix<float, 3, 4>>>(vec_camIn);
}

void Calculate::ComputeCamPara_out()
{
    Eigen::Matrix4f CamPara_out;
    vector<Eigen::Matrix4f> vec_camOut;
    for (const auto &cam : this->Cams)
    {
        CamPara_out << cam->rot_Mat(0, 0), cam->rot_Mat(0, 1), cam->rot_Mat(0, 2), cam->tranT_Vec[0],
            cam->rot_Mat(1, 0), cam->rot_Mat(1, 1), cam->rot_Mat(1, 2), cam->tranT_Vec[1],
            cam->rot_Mat(2, 0), cam->rot_Mat(2, 1), cam->rot_Mat(2, 2), cam->tranT_Vec[2],
            0, 0, 0, 1;
        vec_camOut.push_back(CamPara_out);
    }
    this->CamPara_out = make_shared<vector<Eigen::Matrix4f>>(vec_camOut);
}

void WorldPara::FixAberration(const std::shared_ptr<Calculate> &WorldPara_)
{
    // compute real (x,y) using k0 ~ k4
    // k0 ~ k4 for each camera
    auto bg_coeAb = (*(WorldPara_->coe_Aberr)).begin();
    auto bg_Cams = (WorldPara_->Cams).begin();
    for(const auto &eachfile : this->point_Pixo)
    {
        const auto &k0 = ((*bg_coeAb)(0,0));
        const auto &k1 = ((*bg_coeAb)(1,0));
        const auto &k2 = ((*bg_coeAb)(2,0));
        const auto &k3 = ((*bg_coeAb)(3,0));
        const auto &k4 = ((*bg_coeAb)(4,0));
        const auto &Cx = ((*bg_Cams)->point_PicPrin[0]);
        const auto &Cy = ((*bg_Cams)->point_PicPrin[1]);
        const auto &Fx = ((*bg_Cams)->foclen_Equ[0]);
        const auto &Fy = ((*bg_Cams)->foclen_Equ[1]);
        vector<Eigen::Vector2f> vec_Pix;
        for(const auto & point : (*eachfile))
        {
            float xd = (point[0] - Cx) / Fx;
            float yd = (point[1] - Cy) / Fy;
            float dt_x = xd * (xd*xd + yd*yd)*k0 + (xd*xd + yd*yd)*k1 + xd*xd*k3 + xd*yd*k4;
            float dt_y = yd * (xd*xd + yd*yd)*k0 + (xd*xd + yd*yd)*k2 + xd*yd*k3 + yd*yd*k4;
            Eigen::Vector2f point_pix(point[0] + dt_x, point[1] + dt_y);
            vec_Pix.push_back(point_pix);
        }
        this->point_Pix.push_back(make_shared<vector<Eigen::Vector2f>>(vec_Pix));
        ++bg_coeAb;
        ++bg_Cams;
    }
}

void WorldPara::ComputePoint(const shared_ptr<Calculate> &WorldPara_)
{
    FixAberration(WorldPara_);
    const auto camIn = WorldPara_->CamPara_in;
    const auto camOut = WorldPara_->CamPara_out;
    const auto point_World = this->point_World;
    auto bgIn = (*camIn).begin();
    auto bgOut = (*camOut).begin();
    for (const auto &pw : point_World)
    {
        vector<Eigen::Vector2f> vec_pixo;
        for (const auto &point : (*pw))
        {
            Eigen::Vector4f point_World1(point[0], point[1], point[2], 1.0);
            Eigen::Vector3f tmp;
            Eigen::Vector2f pixo;
            tmp = ((*bgIn) * (*bgOut) * point_World1);
            pixo[0] = tmp[0] / tmp[2];
            pixo[1] = tmp[1] / tmp[2];
            vec_pixo.push_back(pixo);
        }
        this->point_Pixo.push_back(make_shared<vector<Eigen::Vector2f>>(vec_pixo));
        ++bgIn;
        ++bgOut;
    }
}

void WorldPara::ShowResult(const shared_ptr<Calculate> &WorldPara_, const string &outfile_)
{
    string outfile(outfile_ + "*.txt");
    cout << "\n================Misson Completed================\n";
    cout << "Resoults saved in : " << outfile << "\n\n";

    const auto CamPara = WorldPara_->GetCamPara();
    auto CamPara_in = (*WorldPara_->CamPara_in).begin();
    auto CamPara_out = (*WorldPara_->CamPara_out).begin();
    auto bgPixo = (this->point_Pixo).begin();
    auto bgPix = (this->point_Pix).begin();
    int i = 1;
    for (const auto &cam : CamPara)
    {
        ofstream out(outfile_ + to_string(i++) + ".txt");
        out << "Pic Principle Coordinate: (Cx,Cy):\n"
            << cam->point_PicPrin[0] << " " << cam->point_PicPrin[1] << "\n\n"
            << "Equivalent Focal Length: (Fx,Fy):\n"
            << cam->foclen_Equ[0] << " " << cam->foclen_Equ[1] << "\n\n"
            << "Translation Vector: (Tx,Ty,Tz):\n"
            << cam->tranT_Vec[0] << " " << cam->tranT_Vec[1] << " " << cam->tranT_Vec[2] << "\n\n"
            << "Rotation Matrix: (r0 ~ r8):\n"
            << cam->rot_Mat << "\n\n"
            << "Parameters_in:\n"
            << (*CamPara_in) << "\n\n"
            << "Parameters_out:\n"
            << (*CamPara_out) << "\n\n";
        auto results_o = (*bgPixo);
        auto bgRes_o = (*results_o).begin();
        auto edRes_o = (*results_o).end();
        out << "Pic Coordinate of opt: (~x,~y):\n";
        for (; bgRes_o != edRes_o; ++bgRes_o)
            out << (*bgRes_o)[0] << " " << (*bgRes_o)[1] << "\n";
        auto results = (*bgPix);
        auto bgRes = (*results).begin();
        auto edRes = (*results).end();
        out << "\nPic Coordinate of real: (x,y):\n";
        for (; bgRes != edRes; ++bgRes)
            out << (*bgRes)[0] << " " << (*bgRes)[1] << "\n";
        ++bgPixo;
        ++bgPix;
        ++CamPara_in;
        ++CamPara_out;
        out.close();
    }
}

void PicPara_opt::Initialize(std::shared_ptr<Calculate> PicPara_opt_, const string &file_)
{
    // read in coe_Aberration
    string file_coeAberr("coe_Aberration_");
    auto files_Ab = ReadFiles(file_coeAberr);
    Eigen::Matrix<float,5,1> coeAberr;
    vector<Eigen::Matrix<float,5,1>> vec_coeAberr;
    // only a txt of coe_Aberration
    for(const auto &coe : *(files_Ab[0]))
    {
        istringstream iss(coe);
        string s;
        int i = 0;
        while(iss >> s)
            coeAberr(i++,0) = stof(s);
        vec_coeAberr.push_back(coeAberr);
    }
    PicPara_opt_->coe_Aberr = make_shared<vector<Eigen::Matrix<float,5,1>>>(vec_coeAberr);
    // read in PicPara_pix
    auto files = ReadFiles(file_);
    string str("Pic Coordinate of pix: (x,y)");
    CheckData(files, str);
    for (const auto &eachfile : files)
    {
        Eigen::Vector2f point_PicPrin;
        Eigen::Vector2f foclen_Equ;
        Eigen::Vector3f tranT_Vec;
        Eigen::Matrix3f rot_Mat;
        // string line;
        for (int i = 1; i < 12; i += 2)
        {
            istringstream iss((*eachfile)[i]);
            string s;
            // TODO: sepatrate case 1,3,5,7 to a individual function
            switch (i)
            {
            case 1:
            {
                int j = 0;
                while (iss >> s)
                    point_PicPrin[j++] = stof(s);
            };
            break;
            case 3:
            {
                int j = 0;
                while (iss >> s)
                    foclen_Equ[j++] = stof(s);
            };
            break;
            case 5:
            {
                int j = 0;
                while (iss >> s)
                    tranT_Vec[j++] = stof(s);
            };
            break;
            case 7:
            {
                int c = 0;
                while (iss >> s)
                    rot_Mat(0, c++) = stof(s);
                for (int j = 1; j < 3; ++j)
                {
                    istringstream is((*eachfile)[++i]);
                    string s;
                    int k = 0;
                    while (is >> s)
                        rot_Mat(j, k++) = stof(s);
                }
            };
            break;
            case 11:
            {
                Eigen::Vector2f point;
                vector<Eigen::Vector2f> vec_point;
                auto bg = (*eachfile).begin() + 11;
                auto end = (*eachfile).end();
                for (; bg != end; ++bg)
                {
                    istringstream is(*bg);
                    string s;
                    int j = 0;
                    while (is >> s)
                        point[j++] = stof(s);
                    vec_point.push_back(point);
                }
                this->point_Pix.push_back(make_shared<vector<Eigen::Vector2f>>(vec_point));
            };
            break;
            default:
            {
                cerr << "read error!\n";
            }
            break;
            }
        }
        PicPara_opt_->Cams.push_back(make_shared<CamPara>(CamPara(point_PicPrin, foclen_Equ, tranT_Vec, rot_Mat)));
    }
}

void PicPara_opt::FixAberration(const std::shared_ptr<Calculate> &PicPara_opt_)
{
    // compute real (x,y) using k0 ~ k4
    // k0 ~ k4 for each camera
    auto bg_coeAb = (*(PicPara_opt_->coe_Aberr)).begin();
    auto bg_Cams = (PicPara_opt_->Cams).begin();
    const unsigned iter_max = 100;
    const double residual = 1e-10;
    for(const auto &eachfile : this->point_Pix)
    {
        const auto &k0 = ((*bg_coeAb)(0,0));
        const auto &k1 = ((*bg_coeAb)(1,0));
        const auto &k2 = ((*bg_coeAb)(2,0));
        const auto &k3 = ((*bg_coeAb)(3,0));
        const auto &k4 = ((*bg_coeAb)(4,0));
        const auto &Cx = ((*bg_Cams)->point_PicPrin[0]);
        const auto &Cy = ((*bg_Cams)->point_PicPrin[1]);
        const auto &Fx = ((*bg_Cams)->foclen_Equ[0]);
        const auto &Fy = ((*bg_Cams)->foclen_Equ[1]);
        vector<Eigen::Vector2f> vec_Pixo;
        for(const auto & point : (*eachfile))
        {
            float x_tmp = point[0];
            float y_tmp = point[1];
            float xd = (x_tmp - Cx) / Fx;
            float yd = (y_tmp - Cy) / Fy;
            float dt_x_bef = xd * (xd*xd + yd*yd)*k0 + (xd*xd + yd*yd)*k1 + xd*xd*k3 + xd*yd*k4;
            float dt_y_bef = yd * (xd*xd + yd*yd)*k0 + (xd*xd + yd*yd)*k2 + xd*yd*k3 + yd*yd*k4;
            unsigned iter = 0;
            for(;iter < iter_max; ++iter)
            {
                x_tmp = point[0] - dt_x_bef;
                y_tmp = point[1] - dt_y_bef;
                xd = (x_tmp - Cx) / Fx;
                yd = (y_tmp - Cy) / Fy;
                float dt_x = xd * (xd*xd + yd*yd)*k0 + (xd*xd + yd*yd)*k1 + xd*xd*k3 + xd*yd*k4;
                float dt_y = yd * (xd*xd + yd*yd)*k0 + (xd*xd + yd*yd)*k2 + xd*yd*k3 + yd*yd*k4;
                double res_x = abs(dt_x - dt_x_bef);
                double res_y = abs(dt_y - dt_y_bef);
                if(res_x < residual && res_y <residual)
                    break;
                else
                {
                    dt_x_bef = dt_x;
                    dt_y_bef = dt_y;
                }

            }
            cout<<"unsigned iter ="<<iter<<endl;
            Eigen::Vector2f pixo(x_tmp, y_tmp);
            vec_Pixo.push_back(pixo);
        }
        this->point_Pixo.push_back(make_shared<vector<Eigen::Vector2f>>(vec_Pixo));
        ++bg_coeAb;
        ++bg_Cams;
    }
}

void PicPara_opt::ComputePoint(const shared_ptr<Calculate> &PicPara_opt_)
{
    const int piontsNo = ((this->point_Pixo)[0])->size();
    auto CamPara = PicPara_opt_->GetCamPara();
    const auto point_Pixo = this->point_Pixo;
    const int rows = CamPara.size() * 2;
    for (int count = 0; count < piontsNo; ++count)
    {
        auto bgCam = CamPara.begin();
        Eigen::Matrix<float, Eigen::Dynamic, 3> A_initial;
        Eigen::VectorXf B_initial;
        A_initial.resize(rows, 3);
        B_initial.resize(rows);
        int row = 0;
        for (const auto &pixs : point_Pixo)
        {
            const auto &Cxy = (*bgCam)->point_PicPrin;
            const auto &Fxy = (*bgCam)->foclen_Equ;
            const auto &Txyz = (*bgCam)->tranT_Vec;
            const auto &r = (*bgCam)->rot_Mat;
            const auto point = (*pixs)[count];
            float a = (point[0] - Cxy[0]);
            float b = (point[1] - Cxy[1]);
            for (int i = 0; i < 3; ++i)
            {
                A_initial(row, i) = a * r(2, i) - Fxy[0] * r(0, i);
                A_initial(row + 1, i) = b * r(2, i) - Fxy[1] * r(1, i);
            }
            B_initial[row++] = Fxy[0] * Txyz[0] - a * Txyz[2];
            B_initial[row++] = Fxy[1] * Txyz[1] - b * Txyz[2];
            ++bgCam;
        }
        Eigen::Vector3f world;
        Eigen::Matrix<float, 3, Eigen::Dynamic> AT = A_initial.transpose();
        const Eigen::Matrix3f A = AT * A_initial;
        const Eigen::Vector3f B = AT * B_initial;
        const double D_A = A.determinant();
        for (int j = 0; j < 3; ++j)
        {
            Eigen::Matrix3f tmp = A;
            tmp(0, j) = B[0];
            tmp(1, j) = B[1];
            tmp(2, j) = B[2];
            world[j] = (tmp.determinant()) / D_A;
        }
        this->point_World.push_back(world);
    }
}

void PicPara_opt::ShowResult(const shared_ptr<Calculate> &PicPara_opt_, const string &outfile_)
{
    string outfile(outfile_ + "*.txt");
    cout << "\n================Misson Completed================\n";
    cout << "Resoults saved in : " << outfile << "\n\n";

    const auto CamPara = PicPara_opt_->GetCamPara();
    auto CamPara_in = (*PicPara_opt_->CamPara_in).begin();
    auto CamPara_out = (*PicPara_opt_->CamPara_out).begin();
    int i = 1;
    ofstream out(outfile_ + ".txt");
    for (const auto &cam : CamPara)
    {
        out << "Camera : " << i++ << "\n";
        out << "Pic Principle Coordinate: (Cx,Cy):\n"
            << cam->point_PicPrin[0] << " " << cam->point_PicPrin[1] << "\n\n"
            << "Equivalent Focal Length: (Fx,Fy):\n"
            << cam->foclen_Equ[0] << " " << cam->foclen_Equ[1] << "\n\n"
            << "Translation Vector: (Tx,Ty,Tz):\n"
            << cam->tranT_Vec[0] << " " << cam->tranT_Vec[1] << " " << cam->tranT_Vec[2] << "\n\n"
            << "Rotation Matrix: (r0 ~ r8):\n"
            << cam->rot_Mat << "\n\n"
            << "Parameters_in:\n"
            << (*CamPara_in) << "\n\n"
            << "Parameters_out:\n"
            << (*CamPara_out) << "\n\n";
        out << "==================================\n\n";
        ++CamPara_in;
        ++CamPara_out;
    }
    out << "World Coordinate: (X,Y,Z):\n";
    for (const auto &world : (this->point_World))
        out << world[0] << " " << world[1] << " " << world[2] << "\n";
    out.close();
}

void Calibration::Initialize(std::shared_ptr<Calculate> Calibration_, const string &file_)
{
    cout<<"Please input noise of World coordinate : (0 ~ 0.005m)\n";
    float noise_world;
    cin>> noise_world;
    cout<<"Please input noise of Pix coordinate : (0 ~ 0.02pix)\n";
    float noise_pix;
    cin>> noise_pix;

    default_random_engine e(time(0));
    normal_distribution<float> w(0,noise_world), p(0,noise_pix);

    string cali_world(file_ + "world_");
    string cali_pix(file_ + "pix_");
    auto file_world = ReadFiles(cali_world);
    for (const auto &eachfile : file_world)
    {
        vector<Eigen::Vector3f> vec_World;
        Eigen::Vector3f point_World;
        for(const auto &piont : (*eachfile))
        {
            istringstream iss(piont);
            string s;
            int i = 0;
            while(iss >> s)
                point_World[i++] = stof(s) + w(e);
            vec_World.push_back(point_World);
        }
        this->point_World.push_back(make_shared<vector<Eigen::Vector3f>>(vec_World));
    }
    auto file_pix = ReadFiles(cali_pix);
    for (const auto &eachfile : file_pix)
    {
        vector<Eigen::Vector2f> vec_Pix;
        // vector<Eigen::Vector2f> vec_Pixo;
        Eigen::Vector2f point;
        for(const auto &piont : (*eachfile))
        {
            istringstream iss(piont);
            string s;
            int i = 0;
            while(iss >> s)
                point[i++] = stof(s) + p(e);
            vec_Pix.push_back(point);
            // vec_Pixo.push_back(point_Pix);
        }
        this->point_Pix.push_back(make_shared<vector<Eigen::Vector2f>>(vec_Pix));
        // using pix of real as piox for the first time of iteration
        this->point_Pixo.push_back(make_shared<vector<Eigen::Vector2f>>(vec_Pix));
    }


}

void Calibration::ComputePoint(const shared_ptr<Calculate> &Calibration_)
{

    // for each camera, each "cali_pix_*.txt" corresponding to a camera
    // each camera corresponding to a group coe_Aberration
    for(const auto& eachfile : (this->point_Pix))
    {
        unsigned rows = ((*((this->point_Pix)[0])).size()) * 2;//this->point_Pix)[i]
        Eigen::MatrixXf K;
        K.resize(rows,11);
        Eigen::MatrixXf U;
        U.resize(rows,1);
        auto bg_worldpoint = ((this->point_World)[0])->begin();
        auto bg_pixo = (this->point_Pixo).begin();//2020.11.02
        // auto ed_worldpoint = ((this->point_World)[0])->end();
        auto bg_pixpiont = eachfile->begin();
        // iteration of solving k0 ~ k4
        for(;;)
        {
            // cout << "Calibration::FixAberration :  1" << "\n\n";
            // Initialize K ,U
            // r = row
            unsigned r = 0;
            const auto& worldpoint = *bg_worldpoint;
            const auto& pixpoint = *bg_pixpiont;
            for(; r < rows; )
            {
                // cout << "Calibration::FixAberration :  2" << "\n\n";
                Eigen::Matrix<float,1,4> XYZ1(worldpoint[0],worldpoint[1],worldpoint[2],1);
                Eigen::Matrix<float,1,4> ZERO0(0,0,0,0);

                K.row(r) << XYZ1, ZERO0, (-pixpoint[0] * worldpoint).transpose();
                U(r++,0) = pixpoint[0];
                K.row(r) << ZERO0, XYZ1, (-pixpoint[1] * worldpoint).transpose();
                U(r++,0) = pixpoint[1];
                ++bg_worldpoint;
                ++bg_pixpiont;
            }
            // solve s0 ~ s10
            Eigen::MatrixXf KT;
            KT.resize(11,rows);
            KT = K.transpose();
            Eigen::Matrix<float,11,11> a = KT * K;
            Eigen::Matrix<float,11,1> b = KT * U;
            Eigen::Matrix<float,11,1> s = a.lu().solve(b);
            //  solve m0 ~ m11
            float m[12];
            m[11] = 1 / sqrt(s(8,1) * s(8,1) + s(9,1) * s(9,1) + s(10,1) * s(10,1));
            for(int i = 0; i < 11; ++i)
                m[i] = s(i,1) * m[11];
            Eigen::Matrix<float,3,4> M;
            M << m[0], m[1], m[2],
                 m[3], m[4], m[5],
                 m[6], m[7], m[8],
                 m[9], m[10], m[11];
            // compute (~x,~y) of temporary
            vector<Eigen::Vector2f> vec_pixo;
            for(const auto& wp : *((this->point_World)[0]))
            {
                Eigen::Vector4f point_World1(wp[0], wp[1], wp[2], 1.0);
                Eigen::Vector3f tmp;
                Eigen::Vector2f pixo;
                tmp = M * point_World1;
                pixo[0] = tmp[0] / tmp[2];
                pixo[1] = tmp[1] / tmp[2];
                vec_pixo.push_back(pixo);
            }
            this->point_Pixo.clear();
            this->point_Pixo.push_back(make_shared<vector<Eigen::Vector2f>>(vec_pixo));
            // compute Aberration dt_x,dt_y
            this->ComputeAberration();
            // compute k0 ~ k4
            this->ComputeCoe_Aberr();
            // fix Aberration

        }
    }

}

void Calibration::ComputeAberration()
{
    auto bg_pixo = (this->point_Pixo).begin();
    for(const auto& pixfile :this->point_Pix)
    {
        vector<Eigen::Vector2f> vec_Aberr;
        auto iter_pixo = (*bg_pixo)->begin();
        for(const auto& pix : *pixfile)
        {
            vec_Aberr.push_back(pix - (*iter_pixo));
            ++iter_pixo;
        }
        this->Aberration.push_back(make_shared<vector<Eigen::Vector2f>>(vec_Aberr));
        ++bg_pixo;
    }
}

void Calibration::ComputeCoe_Aberr()
{

}

void Calibration::FixAberration(const shared_ptr<Calculate> &Calibration_)
{
    cout << "Calibration::FixAberration : " << "\n\n";

}

void Calibration::ShowResult(const shared_ptr<Calculate> &Calibration_, const string &outfile_)
{
    string outfile(outfile_ + ".txt");
    cout << "\n================Misson Completed================\n";
    cout << "Resoults saved in : " << outfile << "\n\n";
    const auto CamPara = Calibration_->GetCamPara();
    auto CamPara_in = (*Calibration_->CamPara_in).begin();
    auto CamPara_out = (*Calibration_->CamPara_out).begin();
    auto iter_coe_Aberr = (*(this->coe_Aberr)).begin();
    int i = 1;
    ofstream out(outfile_ + ".txt");
    for (const auto &cam : CamPara)
    {
        out << "Camera : " << i++ << "\n";
        out << "Pic Principle Coordinate: (Cx,Cy)\n"
            << cam->point_PicPrin[0] << " " << cam->point_PicPrin[1] << "\n\n"
            << "Equivalent Focal Length: (Fx,Fy)\n"
            << cam->foclen_Equ[0] << " " << cam->foclen_Equ[1] << "\n\n"
            << "Translation Vector: (Tx,Ty,Tz)\n"
            << cam->tranT_Vec[0] << " " << cam->tranT_Vec[1] << " " << cam->tranT_Vec[2] << "\n\n"
            << "Rotation Matrix: (r0 ~ r8)\n"
            << cam->rot_Mat << "\n\n"
            << "Parameters_in\n"
            << (*CamPara_in) << "\n\n"
            << "Parameters_out\n"
            << (*CamPara_out) << "\n\n"
            << "Aberration coefficients:（k0~k4）\n"
            << (*iter_coe_Aberr)[0]<<" "<< (*iter_coe_Aberr)[1]<<" "<<(*iter_coe_Aberr)[2]<<" "
            << (*iter_coe_Aberr)[3]<<" "<< (*iter_coe_Aberr)[4]<< "\n\n";

        out << "==================================\n\n";
        ++CamPara_in;
        ++CamPara_out;
        ++iter_coe_Aberr;
    }
    out.close();

}