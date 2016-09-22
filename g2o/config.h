#ifndef CONFIG_H

#define CONFIG_H
#define G2O_OPENMP 1
//#define G2O_SHARED_LIBS 0
#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
#error "g2o requires column major Eigen matrices (see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=422)"
#endif

#endif