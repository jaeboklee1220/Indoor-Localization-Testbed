/***********************************************************************************************/
/************************************/ / CHAN Algorithm/ /****************************************/
/***********************************************************************************************/
/*3. The Chan method is described in Y.T.Chan and K.C.Ho, "A Simple and Efficient Estimator for Hyperbolic
*        Location", IEEE Trans. SIGNAL PROCES., Vol.42, pp.1905-1915, Aug. 1994*/
#include "multilateration.h"

rowvec MultiLateration::multilaterate(rowvec tdoa, mat anchor_coords, int cleSolver3D)
{
	colvec ddoa, delta;
	vec a, b, c, delta2rsd1, delta2rsd2;
	double rs1;
	mat result1;

	ddoa = (tdoa.st() - tdoa(0)) * CMperS; //convert time to distance (s to m)
	ddoa = ddoa.rows(1, numAnchors - 1);

	mat di;
	di << ddoa(0,0) << endr << ddoa(1,0) << endr << ddoa(2,0) << endr;

	x_hat = chan_al(di, S);
	result1 << refAnchorCoords(0) + x_hat(0, 0) << refAnchorCoords(1) + x_hat(1, 0);
	result1.set_size(1, 3);
	result1.at(0, 2) = 0;

	return result1.row(0);
}
mat MultiLateration::chan_al(mat di, mat H)
{
	mat x_hat = ones<mat>(1, 2);;
	mat K = sum(H % H, 1);

	mat h = 0.5 * (di % di - K );
	mat Ga;
	
	Ga = (-1) * join_rows(H, di);

	mat Q;
	Q = 0.5 * (ones<mat>(3, 3) + eye<mat>(3, 3));

	mat za0;
	za0 = inv(Ga.t() * inv(Q) * Ga) * Ga.t() * inv(Q) * h;
	x_hat << za0(0, 0) << za0(1, 0);

	mat x_tmp;
	x_tmp << x_hat(0, 0) << x_hat(0, 1) << endr << x_hat(0, 0) << x_hat(0, 1) << endr
		<< x_hat(0, 0) << x_hat(0, 1) << endr;
	mat B_tmp;
	B_tmp = sqrt(sum((x_tmp - H) % (x_tmp - H), 1));

	mat B;
	B = zeros<mat>(3, 3);
	B(0, 0) = B_tmp(0, 0);
	B(1, 1) = B_tmp(1, 0);
	B(2, 2) = B_tmp(2, 0);
	//cout << B;

	mat Fi;
	mat c_mat;
	c_mat = CMperS * CMperS * eye<mat>(3, 3);
	//cout << c_mat;
	Fi = c_mat * B * Q * B;
	//cout << Fi;
	mat cov_za;
	cov_za = inv(Ga.t() * inv(Fi) * Ga);
	mat za1;
	za1 = cov_za * Ga.t() * inv(Fi) * h;
	mat Ba2;
	Ba2 = zeros<mat>(3, 3);
	Ba2(0, 0) = za1(0, 0);
	Ba2(1, 1) = za1(1, 0);
	Ba2(2, 2) = za1(2, 0);
	mat sh;
	sh = za1 % za1;
	mat sFi;
	sFi = 4 * Ba2 * cov_za * Ba2;
	mat sGa;
	sGa << 1 << 0 << endr << 0 << 1 << endr << 1 << 1 << endr;
	mat za2;
	za2 = inv(sGa.t() * inv(sFi) * sGa) * sGa.t() * inv(sFi) * sh;
	za2 = sqrt(za2);
	x_hat << za1(0, 0) << endr << za1(1, 0) << endr;

	return x_hat;
}
