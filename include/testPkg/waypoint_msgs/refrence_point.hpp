#include <string>
#include <vector>
class ReferencePoint  {
    public:
    ReferencePoint() = default;
    // const double kDuplicatedPointsEpsilon = 1e-7;
    ReferencePoint(double k, double dk, double h, double x_, double y_) : x(x_), y(y_), kappa_(k), dkappa_(dk), heading(h){}
    ReferencePoint(double x_, double y_) : x(x_), y(y_){}

    double kappa() const;
    double dkappa() const;

    double ReferencePoint::kappa() const { return kappa_; }

    double ReferencePoint::dkappa() const { return dkappa_; }

    private:
        double kappa_ = 0.0;
        double dkappa_ = 0.0;
        double heading = 0.0;
        double x = 0.0, y = 0.0;
};