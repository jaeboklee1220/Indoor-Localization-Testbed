/***********************************************************************************************/
/************************************/ / SI Algorithm/ /****************************************/
/***********************************************************************************************/
/*2. The Spherical Interpolation(SI) method is described in Julius O. Smith and Jonathan S. Abel,
		 "The Spherical Interpolation Method of Source Localization", IEEE J.OCEANIC.ENG., Vol.12, 
          pp.246-252,Jan. 1987.*/
#include "multilateration.h"

rowvec MultiLateration::multilaterate(rowvec tdoa, mat anchor_coords, int cleSolver3D)
{
	colvec ddoa;
	double rs1;
	mat result1;

	ddoa = (tdoa.st() - tdoa(0)) * CMperS; //convert time to distance (s to m)
	ddoa = ddoa.rows(1, numAnchors - 1);

	mat di;
	di << ddoa(0,0) << endr << ddoa(1,0) << endr << ddoa(2,0) << endr;

	x_hat = SI_al(di,S);
	result1 << refAnchorCoords(0) + x_hat(0, 0) << refAnchorCoords(1) + x_hat(1, 0);
	result1.set_size(1, 3);
	result1.at(0, 2) = 0;

	return result1.row(0);
}
mat MultiLateration::SI_al(mat di, mat H)
{
	Ps = S * Sw;
	Ps_ver = eye(3, 3) - Ps;
	mat Rs;
	Rs = (ddoa.st() * Ps_ver * Ps_ver * delta) / (2 * ddoa.st() * Ps_ver * Ps_ver * ddoa);
	double rs;
	rs = Rs(0);
	mat x_hat;
	x_hat = 0.5 * Sw * (delta - 2 * rs * ddoa);
	return x_hat;
}