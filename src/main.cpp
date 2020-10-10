//#define EIGEN_USE_MKL_ALL
//#define EIGEN_VECTORIZE_SSE4_2

#include<iostream>
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



auto p = Initialize();


cout<<p->GetWorldCoordinate()<<"\n"<<
p->GetPicPrinCoordinate()<<"\n"<<
p->GetEquFoclen()<<"\n"<<
p->GetTranVec()<<"\n"<<
p->GetRotMatrix()<<"\n\n";

cout << p->GetParameters_in()<<"\n";
cout << p->GetParameters_out()<<"\n\n";
p->ComputePoint_Pixo(p);
cout<<p->GetPoint_Pixo()<<"\n";




// =============================================================================
    auto end = chrono::high_resolution_clock::now();
    t1 = std::chrono::duration<double>(end - start).count();
    cout << "Elapsed Time:  \nt1 = " << t1 << " s \n"
         << "**Mission Completed !**\n\n\n"
         << "\a";
    system("pause");
    return 0;
}