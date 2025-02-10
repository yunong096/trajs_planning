/*
 * @FilePath: /CubicSpline/cubic_spline.h
 * @Reference: https://github.com/AtsushiSakai/PythonRobotics
 */
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

template <typename T>
std::vector<T> cs_diff(const std::vector<T>& input) {
    std::vector<T> result;
    for (size_t i = 1; i < input.size(); ++i) {
        result.push_back(input[i] - input[i - 1]);
    }
    return result;
}

template <typename T>
bool cs_hasNegative(const std::vector<T>& vec) {
    return std::find_if(vec.begin(), vec.end(), [](T num) { return num < 0; }) != vec.end();
}

class CubicSpline1D {
    private:
        std::vector<double> x_, y_;
        
        int nx_;

        std::vector<double> h_;

        std::vector<double> a_, b_, c_, d_;

        Eigen::MatrixXd A_, B_;


        void CalA() {
            A_ = Eigen::MatrixXd::Zero(nx_, nx_);
            A_(0, 0) = 1.0;

            for (int i = 0; i < nx_ - 1; ++i) {
                if (i != nx_ - 2) {
                    A_(i+1, i+1) = 2.0 * (h_[i] + h_[i + 1]);
                }
                A_(i + 1, i) = h_[i];
                A_(i, i + 1) = h_[i];
            }

            A_(0, 1) = 0.1;
            A_(nx_ - 1, nx_ - 2) = 0.0;
            A_(nx_ - 1, nx_ - 1) = 1.0;
        }

        void CalB() {
            B_ = Eigen::MatrixXd::Zero(nx_, 1);
            // h_[i] = 0 ?
            for (int i = 0; i < nx_ -2; ++i) {
                B_(i+1) = 3.0 * (a_[i+2] - a_[i+1]) / h_[i+1] - 
                        3.0 * (a_[i+1] - a_[i]) / h_[i];
            }
        }

        void SearchIndex(const int x, int *index_x) {
            int index;
            BisectRight(x_, x, &index);
            *index_x = index - 1;
        }

    public:
        CubicSpline1D() {};
        ~CubicSpline1D() {};

        void Init(const std::vector<double> & x, const std::vector<double> & y) {
            h_ = cs_diff(x);
            if (cs_hasNegative(h_)) {
                std::cout << "ERROR: x coordinates must be sorted in ascending order." << std::endl;
            }

            x_ = x;
            y_ = y;
            nx_ = x.size();

            a_ = y;

            CalA();
            CalB();

            Eigen::MatrixXd c = A_.inverse() * B_;
            // std::vector<double> c_vec(c.col(0).data(), c.col(0).data() + c.col(0).size());
            std::vector<double> c_vec;

            for (int i = 0; i < c.rows(); ++i) {
                for (int j = 0; j < c.cols(); ++j) {
                    c_vec.push_back(c(i, j));
                }
            }

            c_ = c_vec;

            double b_i, d_i;
            for (int i = 0; i < nx_ - 1; ++i) {
                d_i = (c_[i+1] - c_[i]) / h_[i] / 3.0;
                b_i = 1.0 / h_[i] * (a_[i+1] - a_[i]) - h_[i] / 3.0 * (2.0 * c_[i] + c_[i + 1]);
                // d_i = i;
                // b_i = i;
                d_.push_back(d_i);
                b_.push_back(b_i);
            }
        }

        Vector2d CalPosition(const double & x) {
            Vector2d pos;
            if (x < x_[0] || x > x_[x_.size() - 1]) {
                std::cout << "ERROR: x is outside the data point's x range." << std::endl;
                return;
            }

            int index = 0;
            SearchIndex(x, &index);

            double dx = x - x_[index];

            double position = a_[index] + b_[index] * dx + c_[index] * dx * dx + 
                            d_[index] * std::pow(dx, 3);
            // double position = 1.0;
            double dposition =b_[index] + 2 * c_[index] * dx + 3 * d_[index] * std::pow(dx, 2);

            pos  << position, dposition;
        }

        void Reset() {};

        void BisectRight(const std::vector<double> & a, const double & x, int * index) {
            int hi = a.size();
            double lo = 0.0;
            int mid;
            
            while (lo < hi) {
            mid = std::floor((lo + hi) / 2);
            if (x < a[mid]) {
                    hi = mid;
            } else {
                    lo = mid + 1;
            }
            }

            *index = lo;
        }
};
