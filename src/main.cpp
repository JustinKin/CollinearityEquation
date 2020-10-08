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

/*     MatrixXd a;
    for(int i =0; i <1000000; ++i)
    {
        a = MatrixXd::Random(4, 4);
        if(abs(a.determinant())<0.0001)
        ++c;
    }
    cout<<a.determinant()<<endl;
 */

/* Vector3f a( -129.571, 137.61, 981.975), b( -126.9, 152.4, 980.1) , m(-17.484, 21.1585, 135.622);
a *= 0.001;
b *= 0.001;
m *= 0.001;
auto cc1 = a.cross(m);
cout << "cc1: \n"<<cc1<<endl<<'\n';
auto test1 = cc1.dot(cc1);
cout<<test1<<endl;

auto cc2 = a.cross(b);
cout << "cc2: \n"<<cc2<<endl<<'\n';
auto test2 = cc2.dot(cc2);
cout<<test2<<endl; */

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