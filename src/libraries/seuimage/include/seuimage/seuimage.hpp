#ifndef __SEU_IMAGE_HPP
#define __SEU_IMAGE_HPP

#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>
#include "cudamat.hpp"

namespace seuimage
{
    bool YUV422ToRGB(CudaMatC &yuv422, CudaMatC &rgb);
    bool BayerToRGB(CudaMatC &bayer, CudaMatC &rgb);
    bool Resize(CudaMatC &mSrc, CudaMatC &mDst);

    bool RGB8uTo32fNorm(CudaMatC &mSrc, CudaMatF &mDst);
    bool PackedToPlanar(CudaMatF &packed, CudaMatF &planar);
    bool RGBToBGR(CudaMatC &rgb, CudaMatC &bgr);

    bool WhiteBalance(CudaMatC &rgb, float rgain, float ggain, float bgain);
    bool Undistored(CudaMatC &in, CudaMatC &out, float *pCamK, float *pDistort, 
                                float *pInvNewCamK, float *pMapx, float *pMapy);
}

#endif
