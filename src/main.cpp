//#define EIGEN_USE_MKL_ALL
//#define EIGEN_VECTORIZE_SSE4_2

#include<iostream>
#include <fstream>
#include<vector>
#include<string>
#include<D:/QinJunyou/C/Eigen3/Eigen/Eigen>
#include "HEAD.H"

using namespace std;

int main(int argc,char *argv[])
{
    string  ComputePoint_Pixo("cam_world"),
            ComputePoint_World("cam_pix"),
            result_pixo("result_pixo_"),
            result_world("result_world");
    int choice;
 BG:
    cout << "==========================================\n";
    cout << "1.Compute Pic Coordinate of opt: (~x,~y)\n";
    cout << "2.Compute World Coordinate: (X,Y,Z):\n";
    cout << "3.EXIT\n\n";
    while(cin >> choice)
    {
        switch (choice)
        {
            case 1:
                {   WorldPara worldpara;
                    auto p = make_shared<WorldPara>(worldpara);
                    worldpara.Initialize(p,ComputePoint_Pixo);
                    p->ComputeCamPara_in();
                    p->ComputeCamPara_out();
                    worldpara.ComputePoint(p);
                    worldpara.ShowResult(p,result_pixo);
                };
                break;
            case 2:
                {
                    PicPara_opt picpara_opt;
                    auto p = make_shared<PicPara_opt>(picpara_opt);
                    picpara_opt.Initialize(p,ComputePoint_World);
                    p->ComputeCamPara_in();
                    p->ComputeCamPara_out();
                    picpara_opt.ComputePoint(p);
                    picpara_opt.ShowResult(p,result_world);
                };
                break;
            default:
                {
                    return 0;
                }
                continue;
        }
        goto BG;
    }



// =============================================================================
    system("pause");
    return 0;
}