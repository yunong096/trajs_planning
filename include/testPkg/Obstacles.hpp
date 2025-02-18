#include <vector>
#include </usr/local/include/eigen3/Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
using namespace Eigen;
using namespace std;

struct obstacle {
	Vector2d speed;
	vector<double> vertex_x;
	vector<double> vertex_y;
	obstacle(const Vector2d& x, const vector<double>& y, const vector<double>& z)
				: speed(x), vertex_x(y), vertex_y(z) {}
};

class Obstacles {
public:
	Obstacles() {
		num = 11;
		obstacle obs1(Vector2d{0, 0}, vector<double>{138.5, 28.5, 28.5, 138.5}, vector<double>{68.3, 68.3, 73.5, 73.5});
		obstacle obs2(Vector2d{0, 0}, vector<double>{76.7, 7.7, 7.7, 76.7}, vector<double>{0.8, 0.8, 6.3, 6.3});
		obstacle obs3(Vector2d{0, 0}, vector<double>{76.7, 7.7, 7.7, 76.7}, vector<double>{13.4, 13.4, 24.5, 24.5});
		obstacle obs4(Vector2d{0, 0}, vector<double>{76.7, 7.7, 7.7, 76.7}, vector<double>{31.7, 31.7, 43.1, 43.1});
		obstacle obs5(Vector2d{0, 0}, vector<double>{76.7, 7.7, 7.7, 76.7}, vector<double>{50.2, 50.2, 61.3, 61.3});
		obstacle obs6(Vector2d{0, 0}, vector<double>{138.5, 83.8, 83.8, 138.5}, vector<double>{0.8, 0.8, 6.3, 6.3});
		obstacle obs7(Vector2d{0, 0}, vector<double>{138.5, 83.8, 83.8, 138.5}, vector<double>{13.4, 13.4, 24.5, 24.5});
		obstacle obs8(Vector2d{0, 0}, vector<double>{138.5, 83.8, 83.8, 138.5}, vector<double>{31.7, 31.7, 43.1, 43.1});
		obstacle obs9(Vector2d{0, 0}, vector<double>{138.5, 83.8, 83.8, 138.5}, vector<double>{50.2, 50.2, 61.3, 61.3});
		// obstacle obs10(Vector2d{-1, 0}, vector<double>{50, 54.7, 54.7, 50}, vector<double>{66.95, 66.95, 64.95, 64.95});
		// obstacle obs11(Vector2d{1, 0}, vector<double>{54, 58.7, 58.7, 54}, vector<double>{64.95, 64.95, 62.95, 62.95});
		obstacle obs10(Vector2d{0, -1}, vector<double>{1.17, 3.17, 3.17, 1.17}, vector<double>{66.95-25, 66.95-25, 66.95 + 4.7-25, 66.95+ 4.7-25});
		obstacle obs11(Vector2d{0, 1}, vector<double>{3.17, 5.17, 5.17, 3.17}, vector<double>{64.95-20, 64.95-20, 64.95+ 4.7-20, 64.95+ 4.7-20});
		obs = { obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9, obs10, obs11 };
		moving_ind = vector<int>{9, 10};
	};
	~Obstacles() {};
	void ReCover() {}
	vector<obstacle> getObs() { return obs; }
	int getNum() { return num; }
	 vector<int> getMoveInds() { return moving_ind; }
private:
	int num;
	vector<obstacle> obs;
	vector<int> moving_ind;
};