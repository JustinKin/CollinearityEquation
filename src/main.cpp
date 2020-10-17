//#define EIGEN_USE_MKL_ALL
//#define EIGEN_VECTORIZE_SSE4_2

#include<iostream>
#include <fstream>
#include<vector>
#include<string>
#include <chrono>
#include<D:/QinJunyou/C/Eigen3/Eigen/Eigen>
#include "HEAD.H"


using namespace Eigen;
using namespace std;


   int main()
{
    double t1, c=0;
    cout << "  Mission Start:  \n\n";
    auto start = chrono::high_resolution_clock::now();
// =============================================================================
    string  ComputePoint_Pixo("ComputePoint_Pixo.txt"),
            ComputePoint_World("ComputePoint_World"),
            result_pixo("result_pixo"),
            result_world("result_world");
    ifstream in_CP(ComputePoint_Pixo);
    if(!in_CP)
    {
        cerr<<"couldn't open : " + ComputePoint_Pixo;
        return -1;
    }
    ofstream out_CP(result_pixo);
    // ifstream in_CW(ComputePoint_World);
    // ofstream out_CW(result_world);

    auto p = Initialize_CP(in_CP);
    in_CP.close();

    cout<<"\n========result========\n";
    cout<<"World Coordinate: (X,Y,Z):\n"<<p->GetWorldCoordinate()<<"\n\n"<<
    "Pic Principle Coordinate: (Cx,Cy):\n"<<p->GetPicPrinCoordinate()<<"\n\n"<<
    "Equivalent Focal Length: (Fx,Fy):\n"<<p->GetEquFoclen()<<"\n\n"<<
    "Translation Vector: (Tx,Ty,Tz):\n"<<p->GetTranVec()<<"\n\n"<<
    "Rotation Matrix: (r0 ~ r8):\n"<<p->GetRotMatrix()<<"\n\n";

    cout<<"Parameters_in:\n" << p->GetParameters_in()<<"\n\n";
    cout<<"Parameters_out:\n"<< p->GetParameters_out()<<"\n\n";
    p->ComputePoint_Pixo(p);
    cout<<"Pic Coordinate of opt: (~x,~y):\n"<<p->GetPoint_Pixo()<<"\n\n";




// =============================================================================
    auto end = chrono::high_resolution_clock::now();
    t1 = std::chrono::duration<double>(end - start).count();
    cout << "Elapsed Time:  \nt1 = " << t1 << " s \n"
         << "**Mission Completed !**\n\n\n"
         << "\a";
    system("pause");
    return 0;
}