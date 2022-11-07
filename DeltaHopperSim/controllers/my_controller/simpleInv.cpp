#include "simpleInv.h"
void simpleInv(double a[3][3], double matinv[3][3]) {
	double determinant = 0.0;
	for (int i = 0; i < 3; i++)
		determinant = determinant + (a[0][i] * (a[1][(i + 1) % 3] * a[2][(i + 2) % 3] - a[1][(i + 2) % 3] * a[2][(i + 1) % 3]));

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			matinv[j][i] = ((a[(i + 1) % 3][(j + 1) % 3] * a[(i + 2) % 3][(j + 2) % 3]) - (a[(i + 1) % 3][(j + 2) % 3] * a[(i + 2) % 3][(j + 1) % 3])) / determinant;
	}

}

void printmat(double mat[3][3]) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << mat[i][j] << " ";
		}
		std::cout << std::endl;
	}
}