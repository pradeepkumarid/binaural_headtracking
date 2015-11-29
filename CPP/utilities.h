#include "allocore/math/al_Mat.hpp"
#include "allocore/math/al_Vec.hpp"

#ifndef UTILITIES_H
#define UTILITIES_H

#define PORT 8888
#define SERVER "192.168.43.79"
#define BUFLEN 1024

#define PI 3.14159

#define CHUNK 1024
#define WAV_FILE_NAME "/home/pradeep/Q4/SpatialAudio/Project/CPP/media/dani_california.wav"
//#define WAV_FILE_NAME "/home/pradeep/Q4/SpatialAudio/Project/CPP/media/imperial_march_full.wav"

/************************************************************************************/
/*!
 *  @brief          Helper function to  access element [i][j][k] of a "3D array" of dimensions [dim1][dim2][dim3]
 *                  stored in a 1D data array
 *
 */
/************************************************************************************/
inline const std::size_t array3DIndex(const unsigned long i,
		const unsigned long j,
		const unsigned long k,
		const unsigned long dim1,
		const unsigned long dim2,
		const unsigned long dim3)
{
	return dim2 * dim3 * i + dim3 * j + k;
}

/************************************************************************************/
/*!
 *  @brief          Helper function to  access element [i][j] of a "2D array" of dimensions [dim1][dim2]
 *                  stored in a 1D data array
 *
 */
/************************************************************************************/
inline const std::size_t array2DIndex(const unsigned long i,
		const unsigned long j,
		const unsigned long dim1,
		const unsigned long dim2)
{
	return dim2 * i + j;
}



inline void convolve(float inputBuff[], std::vector< double > source_pos, int index, float outputBuff[], float overlapBuff[])
{
    int Alen = CHUNK, Blen = 200;
	for (int i = 0; i < (Alen + Blen - 1); i++ )
	{
		int i1 = i;
		float tmp =0;

		for (int j = 0; j < Blen; j++ )
		{
			if(i1>=0 && i1<Alen)
				tmp += inputBuff[i1] * source_pos[index+j];
			i1--;
		}
		if(i<Alen)
		{
			if(i<Blen-1)
				outputBuff[i] = tmp + overlapBuff[i];
			else
				outputBuff[i] = tmp;
		}
		else
			overlapBuff[i-Alen] = tmp;
	}

}

inline void loadRotMatrix(al::Mat3f &R, float phi, float theta, float psi)
{
	phi = phi * PI / 180.0f;
	theta = theta * PI / 180.0f;
	psi = psi * PI / 180.0f;

	R(0,0) = cos(phi) * cos (theta);
	R(0,1) = (cos(phi) * sin (theta) * sin (psi)) - (sin (phi) * cos (psi));
	R(0,2) = (cos(phi) * sin (theta) * cos (psi)) + (sin(phi) * sin(psi));

	R(1,0) = sin(phi) * cos (theta);
	R(1,1) = (sin(phi) * sin(theta) * sin (psi) ) + (cos(phi) * cos(psi));
	R(1,2) = (sin(phi) * sin(theta) * cos(psi) ) - (cos(phi) * sin(psi));

	R(2,0) = -sin(theta);
	R(2,1) = cos(theta) * sin(psi);
	R(2,2) = cos(theta) * cos(psi);
}

al::Mat3f inverse(al::Mat3f A)
{
	al::Mat3f result;
	double determinant =    +A(0,0)*(A(1,1)*A(2,2)-A(2,1)*A(1,2))
	                        -A(0,1)*(A(1,0)*A(2,2)-A(1,2)*A(2,0))
	                        +A(0,2)*(A(1,0)*A(2,1)-A(1,1)*A(2,0));
	double invdet = 1/determinant;
	result(0,0) =  (A(1,1)*A(2,2)-A(2,1)*A(1,2))*invdet;
	result(1,0) = -(A(0,1)*A(2,2)-A(0,2)*A(2,1))*invdet;
	result(2,0) =  (A(0,1)*A(1,2)-A(0,2)*A(1,1))*invdet;
	result(0,1) = -(A(1,0)*A(2,2)-A(1,2)*A(2,0))*invdet;
	result(1,1) =  (A(0,0)*A(2,2)-A(0,2)*A(2,0))*invdet;
	result(2,1) = -(A(0,0)*A(1,2)-A(1,0)*A(0,2))*invdet;
	result(0,2) =  (A(1,0)*A(2,1)-A(2,0)*A(1,1))*invdet;
	result(1,2) = -(A(0,0)*A(2,1)-A(2,0)*A(0,1))*invdet;
	result(2,2) =  (A(0,0)*A(1,1)-A(1,0)*A(0,1))*invdet;

	return result;
}



#endif
