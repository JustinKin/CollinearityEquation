//#define EIGEN_USE_MKL_ALL
//#define EIGEN_VECTORIZE_SSE4_2

#include<iostream>
#include <fstream>
#include<vector>
#include<string>
#include <chrono>
#include<D:/QinJunyou/C/Eigen3/Eigen/Eigen>
#include "HEAD.H"


// using namespace Eigen;
using namespace std;

   int main()
{
    double t1, c=0;
    cout << "  Mission Start:  \n\n";
    auto start = chrono::high_resolution_clock::now();
// =============================================================================
    string  ComputePoint_Pixo("cam_world"),
            ComputePoint_World("cam_pix"),
            result_pixo("result_pixo_"),
            result_world("result_world_");
    WorldPara worldpara;
    int choice;
 BG:
    cout << "=======================================\n";
    cout << "1.Compute Pic Coordinate of opt: (~x,~y)\n";
    cout << "2.Compute World Coordinate: (X,Y,Z):\n";
    cout << "3.EXIT\n\n";
    while(cin >> choice)
    {
        switch (choice)
        {
            case 1:
                {
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
/*                     ifstream in_CW1(ComputePoint_World1);
                    if(!in_CW1)
                    {
                        cerr<<"couldn't open : " + ComputePoint_World1;
                        return -1;
                    }
                    auto p1 = Initialize_CW(in_CW1);
                    in_CW1.close();

                    ifstream in_CW2(ComputePoint_World2);
                    if(!in_CW2)
                    {
                        cerr<<"couldn't open : " + ComputePoint_World2;
                        return -1;
                    }
                    auto p2 = Initialize_CW(in_CW2);
                    in_CW2.close();

                    ComputePoint_World(p1,p2);
                    ofstream out_CW(result_world);
                    ShowResult_CW(p1,p2,out_CW);
                    out_CW.close();
 */
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
    auto end = chrono::high_resolution_clock::now();
    t1 = std::chrono::duration<double>(end - start).count();
    cout << "Elapsed Time:  \nt1 = " << t1 << " s \n"
         << "**Mission Completed !**\n\n\n"
         << "\a";
    system("pause");
    return 0;
}