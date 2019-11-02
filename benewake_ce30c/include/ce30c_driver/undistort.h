#include "base.h"

float bilinear_interpolation(float _targetX, float _targetY, float _P11, float _P12, float _P21, float _P22)
{
	float alpha_x = _targetX - (int)_targetX;
	float alpha_y = _targetY - (int)_targetY;

	float interpl = _P11 * _P12 * _P21 * _P22;
	if(interpl != 0)
    {
        return ((1 - alpha_x) * (1 - alpha_y) * _P11 + alpha_x * (1 - alpha_y) * _P12 + (1 - alpha_x) * alpha_y * _P21 + alpha_x * alpha_y * _P22);
    }
    else
    {
        if(alpha_x < 0.5 && alpha_y < 0.5)
            return _P11;
        else if(alpha_x >= 0.5 && alpha_y < 0.5)
            return _P12;
        else if(alpha_x < 0.5 && alpha_y >= 0.5)
            return _P21;
        else
            return _P22;
    }
}

void remap(unsigned short *_src, unsigned short *_dst, float *_mapX, float *_mapY, int _h, int _w)
{
	for (int i = 0; i < _h; i++)
	{
		for (int j = 0; j < _w; j++)
		{
			float targetX = _mapX[i * _w + j], targetY = _mapY[i * _w + j];
			int x = (int)targetX, y = (int)targetY;
			if (y >= 0 && y <= HEIGHT - 2)
            {
                unsigned short p11 = _src[y * WIDTH + x];
                unsigned short p12 = _src[y * WIDTH + (x + 1)];
                unsigned short p21 = _src[(y + 1) * WIDTH + x];
                unsigned short p22 = _src[(y + 1) * WIDTH + (x + 1)];
                _dst[i * _w + j] = (unsigned short)bilinear_interpolation(targetX, targetY, p11, p12, p21, p22);
            }
            else
            {
                _dst[i * _w + j] = 0;
            }
		}
	}
}

// inverse matrix (simplified for camera matrix)
void inverse_matrix(double *_A, double *_B, int _n)
{
	int i, j, k;
	float mx, temp, iT;
	double *C = new double[_n * _n];
	for (i = 0; i < _n; i++){
        for (j = 0; j < _n; j++){
            C[i * _n + j] = _A[i * _n + j];
            _B[i * _n + j] = i == j ? 1.0 : 0.0;
        }
	}

	for (i = 0; i < _n; i++)
	{
		/*mx = C[i][i];
		k = i;
		for (j = i + 1; j < _n; j++)
		{
            if (fabs(C[j][i]) > fabs(mx))
            {
                mx = C[j][i];
                k = j;
            }
		}

		if (k != i)
		{
            for (j = 0; j < _n; j++)
            {
                temp = C[i][j];
                C[i][j] = C[k][j];
                C[k][j] = temp;

                temp = _B[i][j];
                _B[i][j] = _b[k][j];
                _B[k][j] = temp;
		}
		}*/

		temp = C[i * _n + i];
		for (j = 0; j < _n; j++)
		{
			C[i * _n + j] = C[i * _n + j] / temp;
			_B[i * _n + j] = _B[i * _n + j] / temp;
		}

		for (j = 0; j < _n; j++)
		{
			if (j != i)
			{
				iT = C[j * _n + i];
				for (k = 0; k < _n; k++)
				{
					C[j * _n + k] = C[j * _n + k] - C[i * _n + k] * iT;
					_B[j * _n + k] = _B[j * _n + k] - _B[i * _n + k] * iT;
				}
			}
		}
	}
}

void init_fisheye_map(double *_CameraMatrix, double *_Coeffs, float *_mapx, float *_mapy, int _height, int _width)
{
	double *NewCameraMatrix = (double*)malloc(sizeof(double) * 3 * 3);
    memcpy(NewCameraMatrix, _CameraMatrix, 9 * sizeof(double));
    NewCameraMatrix[2] = (_width - 1)* 0.5;
    NewCameraMatrix[5] = (_height - 1) * 0.5;

	double *ir = (double*)malloc(sizeof(double) * 3 * 3);
	inverse_matrix(NewCameraMatrix, ir, 3);
	double u0 = _CameraMatrix[2], v0 = _CameraMatrix[5];
	double fx = _CameraMatrix[0], fy = _CameraMatrix[4];

	for (int i = 0; i < _height; i++)
	{
		double _x = i * ir[1] + ir[2], _y = i * ir[4] + ir[5], _w = i * ir[7] + ir[8];

		for (int j = 0; j < _width; j++, _x += ir[0], _y += ir[3], _w += ir[6])
		{
			double x = _x/_w, y = _y/_w;

			double r = sqrt(x*x + y*y);
			double theta = atan(r);

			double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta2*theta4, theta8 = theta4*theta4;
			double theta_d = theta * (1 + _Coeffs[0] * theta2 + _Coeffs[1] * theta4 + _Coeffs[2] * theta6 + _Coeffs[3] * theta8);

			double scale = (r == 0) ? 1.0 : theta_d / r;
			double u = fx * x * scale + u0;
			double v = fy * y * scale + v0;

			_mapx[i * _width + j] = (float)u;
			_mapy[i * _width + j] = (float)v;
		}
	}
}
