#ifndef UTILITIES_H
#define UTILITIES_H



#define CHUNK 1024
#define WAV_FILE_NAME "/home/pradeep/Q4/SpatialAudio/Project/CPP/media/imperial_march_full.wav"

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

//COnv test code
//float A[] = {1,2,3,4,5};
//float B[] = {1,2,1};
//float C[5];
//float buf[] = {1,2};
//
//int Alen = 5, Blen = 3;
//
//for (int i = 0; i < (Alen + Blen-1); i++ )
//{
//	int i1 = i;
//	float tmp =0;
//
//	for (int j = 0; j < Blen; j++ )
//	{
//		if(i1>=0 && i1<Alen)
//			tmp += A[i1] * B[j];
//		i1--;
//	}
//	if(i<Alen)
//	{
//		if(i<Blen-1)
//			C[i] = tmp + buf[i];
//		else
//			C[i] = tmp;
//	}
//	else
//		buf[i-Alen] = tmp;
//}
//
//std::cout<<"\nC:";
//for(int i=0;i<5;i++) std::cout<<C[i]<<" ";
//
//std::cout<<"\nBuf:";
//for(int i=0;i<2;i++) std::cout<<buf[i]<<" ";


inline void convolve(float inputBuff[], std::vector< double > source_pos, int index, float outputBuff[], float overlapBuff[])
{
    int Alen = 1024, Blen = 200;
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



#endif
