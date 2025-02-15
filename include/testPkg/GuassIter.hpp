#include <iostream>
#include <vector>
#include <functional>
#include <cmath>

class GaussLegendreIntegration {
public:
    enum class NodeCount {
        Three = 3,
        Four = 4,
        Five = 5
    };

    std::function<double(double)> func;

    GaussLegendreIntegration(){};
    GaussLegendreIntegration(NodeCount count) {
        switch (count) {
            case NodeCount::Three:
                initializeNodesAndWeights(nodes3, weights3);
                break;
            case NodeCount::Four:
                initializeNodesAndWeights(nodes4, weights4);
                break;
            case NodeCount::Five:
                initializeNodesAndWeights(nodes5, weights5);
                break;
        }
    }

    double integrate(double a, double b) const {
        double integral = 0.0;
        for (size_t i = 0; i < nodes.size(); ++i) {
            double xi = 0.5 * (nodes[i] + 1) * (b - a) + a; // 线性变换到区间 [a, b]
            integral += weights[i] * func(xi);
        }
        return integral * 0.5 * (b - a);
    }

private:
    std::vector<double> nodes;
    std::vector<double> weights;

    // 系数表
    static const std::vector<double> nodes3;
    static const std::vector<double> weights3;
    static const std::vector<double> nodes4;
    static const std::vector<double> weights4;
    static const std::vector<double> nodes5;
    static const std::vector<double> weights5;

    void initializeNodesAndWeights(const std::vector<double>& n, const std::vector<double>& w) {
        nodes = n;
        weights = w;
    }
};

// int main() {
//     auto func = [](double x) { return std::exp(-x * x); }; // 被积函数
//     double a = 0.0;
//     double b = 1.0;

//     // 使用3个节点
//     GaussLegendreIntegration integrator3(GaussLegendreIntegration::NodeCount::Three);
//     double result3 = integrator3.integrate(func, a, b);
//     std::cout << "Integral result with 3 nodes: " << result3 << std::endl;

//     // 使用4个节点
//     GaussLegendreIntegration integrator4(GaussLegendreIntegration::NodeCount::Four);
//     double result4 = integrator4.integrate(func, a, b);
//     std::cout << "Integral result with 4 nodes: " << result4 << std::endl;

//     // 使用5个节点
//     GaussLegendreIntegration integrator5(GaussLegendreIntegration::NodeCount::Five);
//     double result5 = integrator5.integrate(func, a, b);
//     std::cout << "Integral result with 5 nodes: " << result5 << std::endl;

//     return 0;
// }