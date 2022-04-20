#ifndef FILTERCOMMON
#define FILTERCOMMON
#include <Eigen/Dense>
#include <iostream>
#include <vector>
using namespace Eigen;

//using following arguments to boost function arguments transfer
typedef const Ref<const MatrixXd> matXdArgConst;
typedef const Ref<const MatrixXf> matXfArgConst;
typedef const Ref<const MatrixXi> matXiArgConst;

typedef Ref<MatrixXd> matXdArg;
typedef Ref<MatrixXf> matXfArg;
typedef Ref<MatrixXi> matXfAri;



#endif // FILTERCOMMON

