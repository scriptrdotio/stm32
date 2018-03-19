/**
 ******************************************************************************
 * @file    flat_top_win.h
 * @author  MP
 * @version V1.0.0
 * @date    21-July-2016
 * @brief   This file contains table of flat top window values for FFT 
 *          project
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLAT_TOP_WIN_H
#define __FLAT_TOP_WIN_H

const float scaleFactor = 4.6448;

const float flatTopWin[1024] = 
{
 9.0513e-04,     9.0410e-04,     9.0102e-04,     8.9587e-04,     8.8865e-04,     8.7937e-04,     8.6801e-04,     8.5456e-04,     
 8.3903e-04,     8.2139e-04,     8.0164e-04,     7.7976e-04,     7.5574e-04,     7.2957e-04,     7.0122e-04,     6.7068e-04,     
 6.3793e-04,     6.0294e-04,     5.6570e-04,     5.2619e-04,     4.8437e-04,     4.4023e-04,     3.9373e-04,     3.4485e-04,     
 2.9355e-04,     2.3981e-04,     1.8360e-04,     1.2489e-04,     6.3628e-05,    -2.0576e-07,    -6.6651e-05,    -1.3575e-04,     
-2.0753e-04,    -2.8203e-04,    -3.5931e-04,    -4.3938e-04,    -5.2231e-04,    -6.0813e-04,    -6.9688e-04,    -7.8861e-04,     
-8.8337e-04,    -9.8119e-04,    -1.0821e-03,    -1.1862e-03,    -1.2936e-03,    -1.4041e-03,    -1.5180e-03,    -1.6353e-03,     
-1.7559e-03,    -1.8800e-03,    -2.0077e-03,    -2.1389e-03,    -2.2737e-03,    -2.4121e-03,    -2.5543e-03,    -2.7003e-03,     
-2.8501e-03,    -3.0038e-03,    -3.1614e-03,    -3.3230e-03,    -3.4886e-03,    -3.6583e-03,    -3.8322e-03,    -4.0103e-03,     
-4.1926e-03,    -4.3792e-03,    -4.5702e-03,    -4.7656e-03,    -4.9654e-03,    -5.1698e-03,    -5.3788e-03,    -5.5923e-03,     
-5.8106e-03,    -6.0335e-03,    -6.2612e-03,    -6.4937e-03,    -6.7311e-03,    -6.9734e-03,    -7.2207e-03,    -7.4729e-03,     
-7.7302e-03,    -7.9926e-03,    -8.2601e-03,    -8.5327e-03,    -8.8106e-03,    -9.0937e-03,    -9.3821e-03,    -9.6759e-03,     
-9.9750e-03,    -1.0279e-02,    -1.0589e-02,    -1.0905e-02,    -1.1225e-02,    -1.1552e-02,    -1.1884e-02,    -1.2221e-02,     
-1.2564e-02,    -1.2912e-02,    -1.3267e-02,    -1.3626e-02,    -1.3992e-02,    -1.4363e-02,    -1.4739e-02,    -1.5122e-02,     
-1.5510e-02,    -1.5903e-02,    -1.6302e-02,    -1.6707e-02,    -1.7118e-02,    -1.7534e-02,    -1.7956e-02,    -1.8383e-02,     
-1.8816e-02,    -1.9255e-02,    -1.9699e-02,    -2.0149e-02,    -2.0604e-02,    -2.1064e-02,    -2.1530e-02,    -2.2002e-02,     
-2.2478e-02,    -2.2960e-02,    -2.3447e-02,    -2.3940e-02,    -2.4437e-02,    -2.4940e-02,    -2.5447e-02,    -2.5960e-02,     
-2.6477e-02,    -2.6999e-02,    -2.7526e-02,    -2.8058e-02,    -2.8594e-02,    -2.9134e-02,    -2.9679e-02,    -3.0228e-02,     
-3.0781e-02,    -3.1338e-02,    -3.1899e-02,    -3.2464e-02,    -3.3033e-02,    -3.3605e-02,    -3.4180e-02,    -3.4759e-02,     
-3.5341e-02,    -3.5926e-02,    -3.6514e-02,    -3.7104e-02,    -3.7698e-02,    -3.8293e-02,    -3.8891e-02,    -3.9491e-02,     
-4.0092e-02,    -4.0696e-02,    -4.1300e-02,    -4.1907e-02,    -4.2514e-02,    -4.3122e-02,    -4.3731e-02,    -4.4341e-02,     
-4.4950e-02,    -4.5560e-02,    -4.6170e-02,    -4.6779e-02,    -4.7388e-02,    -4.7996e-02,    -4.8602e-02,    -4.9208e-02,     
-4.9812e-02,    -5.0414e-02,    -5.1013e-02,    -5.1611e-02,    -5.2206e-02,    -5.2797e-02,    -5.3386e-02,    -5.3971e-02,     
-5.4552e-02,    -5.5130e-02,    -5.5702e-02,    -5.6270e-02,    -5.6834e-02,    -5.7391e-02,    -5.7943e-02,    -5.8489e-02,     
-5.9029e-02,    -5.9562e-02,    -6.0088e-02,    -6.0607e-02,    -6.1118e-02,    -6.1621e-02,    -6.2115e-02,    -6.2601e-02,     
-6.3078e-02,    -6.3545e-02,    -6.4002e-02,    -6.4449e-02,    -6.4885e-02,    -6.5311e-02,    -6.5724e-02,    -6.6127e-02,     
-6.6516e-02,    -6.6894e-02,    -6.7258e-02,    -6.7608e-02,    -6.7945e-02,    -6.8268e-02,    -6.8576e-02,    -6.8869e-02,     
-6.9146e-02,    -6.9407e-02,    -6.9652e-02,    -6.9880e-02,    -7.0091e-02,    -7.0284e-02,    -7.0458e-02,    -7.0615e-02,     
-7.0752e-02,    -7.0870e-02,    -7.0967e-02,    -7.1044e-02,    -7.1101e-02,    -7.1136e-02,    -7.1149e-02,    -7.1140e-02,     
-7.1108e-02,    -7.1054e-02,    -7.0975e-02,    -7.0872e-02,    -7.0745e-02,    -7.0593e-02,    -7.0415e-02,    -7.0211e-02,     
-6.9981e-02,    -6.9724e-02,    -6.9439e-02,    -6.9127e-02,    -6.8786e-02,    -6.8416e-02,    -6.8017e-02,    -6.7589e-02,     
-6.7130e-02,    -6.6640e-02,    -6.6119e-02,    -6.5567e-02,    -6.4983e-02,    -6.4366e-02,    -6.3716e-02,    -6.3033e-02,     
-6.2315e-02,    -6.1564e-02,    -6.0777e-02,    -5.9956e-02,    -5.9098e-02,    -5.8205e-02,    -5.7275e-02,    -5.6308e-02,     
-5.5303e-02,    -5.4261e-02,    -5.3180e-02,    -5.2061e-02,    -5.0902e-02,    -4.9704e-02,    -4.8466e-02,    -4.7188e-02,     
-4.5869e-02,    -4.4509e-02,    -4.3107e-02,    -4.1664e-02,    -4.0178e-02,    -3.8649e-02,    -3.7078e-02,    -3.5463e-02,     
-3.3804e-02,    -3.2101e-02,    -3.0354e-02,    -2.8562e-02,    -2.6725e-02,    -2.4843e-02,    -2.2915e-02,    -2.0941e-02,     
-1.8921e-02,    -1.6854e-02,    -1.4740e-02,    -1.2580e-02,    -1.0371e-02,    -8.1155e-03,    -5.8118e-03,    -3.4600e-03,     
-1.0598e-03,     1.3890e-03,     3.8865e-03,     6.4330e-03,     9.0286e-03,     1.1674e-02,     1.4368e-02,     1.7112e-02,     
 1.9906e-02,     2.2750e-02,     2.5644e-02,     2.8589e-02,     3.1583e-02,     3.4628e-02,     3.7724e-02,     4.0870e-02,     
 4.4066e-02,     4.7314e-02,     5.0612e-02,     5.3960e-02,     5.7360e-02,     6.0810e-02,     6.4311e-02,     6.7863e-02,     
 7.1466e-02,     7.5119e-02,     7.8823e-02,     8.2577e-02,     8.6382e-02,     9.0237e-02,     9.4142e-02,     9.8098e-02,     
 1.0210e-01,     1.0616e-01,     1.1026e-01,     1.1442e-01,     1.1862e-01,     1.2287e-01,     1.2718e-01,     1.3153e-01,     
 1.3593e-01,     1.4037e-01,     1.4487e-01,     1.4941e-01,     1.5400e-01,     1.5864e-01,     1.6332e-01,     1.6805e-01,     
 1.7283e-01,     1.7765e-01,     1.8252e-01,     1.8743e-01,     1.9238e-01,     1.9738e-01,     2.0243e-01,     2.0751e-01,     
 2.1264e-01,     2.1782e-01,     2.2303e-01,     2.2829e-01,     2.3358e-01,     2.3892e-01,     2.4429e-01,     2.4971e-01,     
 2.5516e-01,     2.6065e-01,     2.6618e-01,     2.7174e-01,     2.7735e-01,     2.8298e-01,     2.8865e-01,     2.9436e-01,     
 3.0010e-01,     3.0587e-01,     3.1168e-01,     3.1751e-01,     3.2338e-01,     3.2927e-01,     3.3520e-01,     3.4115e-01,     
 3.4713e-01,     3.5314e-01,     3.5918e-01,     3.6523e-01,     3.7132e-01,     3.7743e-01,     3.8356e-01,     3.8971e-01,     
 3.9588e-01,     4.0207e-01,     4.0829e-01,     4.1452e-01,     4.2077e-01,     4.2703e-01,     4.3331e-01,     4.3961e-01,     
 4.4592e-01,     4.5224e-01,     4.5857e-01,     4.6492e-01,     4.7127e-01,     4.7764e-01,     4.8401e-01,     4.9039e-01,     
 4.9677e-01,     5.0316e-01,     5.0955e-01,     5.1595e-01,     5.2234e-01,     5.2874e-01,     5.3514e-01,     5.4154e-01,     
 5.4793e-01,     5.5432e-01,     5.6071e-01,     5.6709e-01,     5.7346e-01,     5.7983e-01,     5.8619e-01,     5.9253e-01,     
 5.9887e-01,     6.0519e-01,     6.1151e-01,     6.1780e-01,     6.2408e-01,     6.3035e-01,     6.3659e-01,     6.4282e-01,     
 6.4903e-01,     6.5522e-01,     6.6138e-01,     6.6752e-01,     6.7364e-01,     6.7973e-01,     6.8579e-01,     6.9183e-01,     
 6.9784e-01,     7.0382e-01,     7.0977e-01,     7.1568e-01,     7.2156e-01,     7.2741e-01,     7.3322e-01,     7.3900e-01,     
 7.4473e-01,     7.5043e-01,     7.5609e-01,     7.6171e-01,     7.6728e-01,     7.7282e-01,     7.7831e-01,     7.8375e-01,     
 7.8915e-01,     7.9450e-01,     7.9980e-01,     8.0505e-01,     8.1025e-01,     8.1540e-01,     8.2050e-01,     8.2554e-01,     
 8.3053e-01,     8.3547e-01,     8.4034e-01,     8.4516e-01,     8.4993e-01,     8.5463e-01,     8.5927e-01,     8.6385e-01,     
 8.6837e-01,     8.7283e-01,     8.7722e-01,     8.8155e-01,     8.8581e-01,     8.9000e-01,     8.9413e-01,     8.9819e-01,     
 9.0218e-01,     9.0610e-01,     9.0995e-01,     9.1373e-01,     9.1744e-01,     9.2107e-01,     9.2463e-01,     9.2812e-01,     
 9.3153e-01,     9.3486e-01,     9.3812e-01,     9.4130e-01,     9.4440e-01,     9.4742e-01,     9.5037e-01,     9.5323e-01,     
 9.5602e-01,     9.5872e-01,     9.6134e-01,     9.6388e-01,     9.6634e-01,     9.6871e-01,     9.7100e-01,     9.7321e-01,     
 9.7533e-01,     9.7737e-01,     9.7932e-01,     9.8118e-01,     9.8296e-01,     9.8465e-01,     9.8626e-01,     9.8778e-01,     
 9.8921e-01,     9.9055e-01,     9.9181e-01,     9.9297e-01,     9.9405e-01,     9.9504e-01,     9.9594e-01,     9.9675e-01,     
 9.9747e-01,     9.9810e-01,     9.9864e-01,     9.9909e-01,     9.9945e-01,     9.9972e-01,     9.9990e-01,     9.9999e-01,     
 9.9999e-01,     9.9990e-01,     9.9972e-01,     9.9945e-01,     9.9909e-01,     9.9864e-01,     9.9810e-01,     9.9747e-01,     
 9.9675e-01,     9.9594e-01,     9.9504e-01,     9.9405e-01,     9.9297e-01,     9.9181e-01,     9.9055e-01,     9.8921e-01,     
 9.8778e-01,     9.8626e-01,     9.8465e-01,     9.8296e-01,     9.8118e-01,     9.7932e-01,     9.7737e-01,     9.7533e-01,     
 9.7321e-01,     9.7100e-01,     9.6871e-01,     9.6634e-01,     9.6388e-01,     9.6134e-01,     9.5872e-01,     9.5602e-01,     
 9.5323e-01,     9.5037e-01,     9.4742e-01,     9.4440e-01,     9.4130e-01,     9.3812e-01,     9.3486e-01,     9.3153e-01,     
 9.2812e-01,     9.2463e-01,     9.2107e-01,     9.1744e-01,     9.1373e-01,     9.0995e-01,     9.0610e-01,     9.0218e-01,     
 8.9819e-01,     8.9413e-01,     8.9000e-01,     8.8581e-01,     8.8155e-01,     8.7722e-01,     8.7283e-01,     8.6837e-01,     
 8.6385e-01,     8.5927e-01,     8.5463e-01,     8.4993e-01,     8.4516e-01,     8.4034e-01,     8.3547e-01,     8.3053e-01,     
 8.2554e-01,     8.2050e-01,     8.1540e-01,     8.1025e-01,     8.0505e-01,     7.9980e-01,     7.9450e-01,     7.8915e-01,     
 7.8375e-01,     7.7831e-01,     7.7282e-01,     7.6728e-01,     7.6171e-01,     7.5609e-01,     7.5043e-01,     7.4473e-01,     
 7.3900e-01,     7.3322e-01,     7.2741e-01,     7.2156e-01,     7.1568e-01,     7.0977e-01,     7.0382e-01,     6.9784e-01,     
 6.9183e-01,     6.8579e-01,     6.7973e-01,     6.7364e-01,     6.6752e-01,     6.6138e-01,     6.5522e-01,     6.4903e-01,     
 6.4282e-01,     6.3659e-01,     6.3035e-01,     6.2408e-01,     6.1780e-01,     6.1151e-01,     6.0519e-01,     5.9887e-01,     
 5.9253e-01,     5.8619e-01,     5.7983e-01,     5.7346e-01,     5.6709e-01,     5.6071e-01,     5.5432e-01,     5.4793e-01,     
 5.4154e-01,     5.3514e-01,     5.2874e-01,     5.2234e-01,     5.1595e-01,     5.0955e-01,     5.0316e-01,     4.9677e-01,     
 4.9039e-01,     4.8401e-01,     4.7764e-01,     4.7127e-01,     4.6492e-01,     4.5857e-01,     4.5224e-01,     4.4592e-01,     
 4.3961e-01,     4.3331e-01,     4.2703e-01,     4.2077e-01,     4.1452e-01,     4.0829e-01,     4.0207e-01,     3.9588e-01,     
 3.8971e-01,     3.8356e-01,     3.7743e-01,     3.7132e-01,     3.6523e-01,     3.5918e-01,     3.5314e-01,     3.4713e-01,     
 3.4115e-01,     3.3520e-01,     3.2927e-01,     3.2338e-01,     3.1751e-01,     3.1168e-01,     3.0587e-01,     3.0010e-01,     
 2.9436e-01,     2.8865e-01,     2.8298e-01,     2.7735e-01,     2.7174e-01,     2.6618e-01,     2.6065e-01,     2.5516e-01,     
 2.4971e-01,     2.4429e-01,     2.3892e-01,     2.3358e-01,     2.2829e-01,     2.2303e-01,     2.1782e-01,     2.1264e-01,     
 2.0751e-01,     2.0243e-01,     1.9738e-01,     1.9238e-01,     1.8743e-01,     1.8252e-01,     1.7765e-01,     1.7283e-01,     
 1.6805e-01,     1.6332e-01,     1.5864e-01,     1.5400e-01,     1.4941e-01,     1.4487e-01,     1.4037e-01,     1.3593e-01,     
 1.3153e-01,     1.2718e-01,     1.2287e-01,     1.1862e-01,     1.1442e-01,     1.1026e-01,     1.0616e-01,     1.0210e-01,     
 9.8098e-02,     9.4142e-02,     9.0237e-02,     8.6382e-02,     8.2577e-02,     7.8823e-02,     7.5119e-02,     7.1466e-02,     
 6.7863e-02,     6.4311e-02,     6.0810e-02,     5.7360e-02,     5.3960e-02,     5.0612e-02,     4.7314e-02,     4.4066e-02,     
 4.0870e-02,     3.7724e-02,     3.4628e-02,     3.1583e-02,     2.8589e-02,     2.5644e-02,     2.2750e-02,     1.9906e-02,     
 1.7112e-02,     1.4368e-02,     1.1674e-02,     9.0286e-03,     6.4330e-03,     3.8865e-03,     1.3890e-03,    -1.0598e-03,     
-3.4600e-03,    -5.8118e-03,    -8.1155e-03,    -1.0371e-02,    -1.2580e-02,    -1.4740e-02,    -1.6854e-02,    -1.8921e-02,     
-2.0941e-02,    -2.2915e-02,    -2.4843e-02,    -2.6725e-02,    -2.8562e-02,    -3.0354e-02,    -3.2101e-02,    -3.3804e-02,     
-3.5463e-02,    -3.7078e-02,    -3.8649e-02,    -4.0178e-02,    -4.1664e-02,    -4.3107e-02,    -4.4509e-02,    -4.5869e-02,     
-4.7188e-02,    -4.8466e-02,    -4.9704e-02,    -5.0902e-02,    -5.2061e-02,    -5.3180e-02,    -5.4261e-02,    -5.5303e-02,     
-5.6308e-02,    -5.7275e-02,    -5.8205e-02,    -5.9098e-02,    -5.9956e-02,    -6.0777e-02,    -6.1564e-02,    -6.2315e-02,     
-6.3033e-02,    -6.3716e-02,    -6.4366e-02,    -6.4983e-02,    -6.5567e-02,    -6.6119e-02,    -6.6640e-02,    -6.7130e-02,     
-6.7589e-02,    -6.8017e-02,    -6.8416e-02,    -6.8786e-02,    -6.9127e-02,    -6.9439e-02,    -6.9724e-02,    -6.9981e-02,     
-7.0211e-02,    -7.0415e-02,    -7.0593e-02,    -7.0745e-02,    -7.0872e-02,    -7.0975e-02,    -7.1054e-02,    -7.1108e-02,     
-7.1140e-02,    -7.1149e-02,    -7.1136e-02,    -7.1101e-02,    -7.1044e-02,    -7.0967e-02,    -7.0870e-02,    -7.0752e-02,     
-7.0615e-02,    -7.0458e-02,    -7.0284e-02,    -7.0091e-02,    -6.9880e-02,    -6.9652e-02,    -6.9407e-02,    -6.9146e-02,     
-6.8869e-02,    -6.8576e-02,    -6.8268e-02,    -6.7945e-02,    -6.7608e-02,    -6.7258e-02,    -6.6894e-02,    -6.6516e-02,     
-6.6127e-02,    -6.5724e-02,    -6.5311e-02,    -6.4885e-02,    -6.4449e-02,    -6.4002e-02,    -6.3545e-02,    -6.3078e-02,     
-6.2601e-02,    -6.2115e-02,    -6.1621e-02,    -6.1118e-02,    -6.0607e-02,    -6.0088e-02,    -5.9562e-02,    -5.9029e-02,     
-5.8489e-02,    -5.7943e-02,    -5.7391e-02,    -5.6834e-02,    -5.6270e-02,    -5.5702e-02,    -5.5130e-02,    -5.4552e-02,     
-5.3971e-02,    -5.3386e-02,    -5.2797e-02,    -5.2206e-02,    -5.1611e-02,    -5.1013e-02,    -5.0414e-02,    -4.9812e-02,     
-4.9208e-02,    -4.8602e-02,    -4.7996e-02,    -4.7388e-02,    -4.6779e-02,    -4.6170e-02,    -4.5560e-02,    -4.4950e-02,     
-4.4341e-02,    -4.3731e-02,    -4.3122e-02,    -4.2514e-02,    -4.1907e-02,    -4.1300e-02,    -4.0696e-02,    -4.0092e-02,     
-3.9491e-02,    -3.8891e-02,    -3.8293e-02,    -3.7698e-02,    -3.7104e-02,    -3.6514e-02,    -3.5926e-02,    -3.5341e-02,     
-3.4759e-02,    -3.4180e-02,    -3.3605e-02,    -3.3033e-02,    -3.2464e-02,    -3.1899e-02,    -3.1338e-02,    -3.0781e-02,     
-3.0228e-02,    -2.9679e-02,    -2.9134e-02,    -2.8594e-02,    -2.8058e-02,    -2.7526e-02,    -2.6999e-02,    -2.6477e-02,     
-2.5960e-02,    -2.5447e-02,    -2.4940e-02,    -2.4437e-02,    -2.3940e-02,    -2.3447e-02,    -2.2960e-02,    -2.2478e-02,     
-2.2002e-02,    -2.1530e-02,    -2.1064e-02,    -2.0604e-02,    -2.0149e-02,    -1.9699e-02,    -1.9255e-02,    -1.8816e-02,     
-1.8383e-02,    -1.7956e-02,    -1.7534e-02,    -1.7118e-02,    -1.6707e-02,    -1.6302e-02,    -1.5903e-02,    -1.5510e-02,     
-1.5122e-02,    -1.4739e-02,    -1.4363e-02,    -1.3992e-02,    -1.3626e-02,    -1.3267e-02,    -1.2912e-02,    -1.2564e-02,     
-1.2221e-02,    -1.1884e-02,    -1.1552e-02,    -1.1225e-02,    -1.0905e-02,    -1.0589e-02,    -1.0279e-02,    -9.9750e-03,     
-9.6759e-03,    -9.3821e-03,    -9.0937e-03,    -8.8106e-03,    -8.5327e-03,    -8.2601e-03,    -7.9926e-03,    -7.7302e-03,     
-7.4729e-03,    -7.2207e-03,    -6.9734e-03,    -6.7311e-03,    -6.4937e-03,    -6.2612e-03,    -6.0335e-03,    -5.8106e-03,     
-5.5923e-03,    -5.3788e-03,    -5.1698e-03,    -4.9654e-03,    -4.7656e-03,    -4.5702e-03,    -4.3792e-03,    -4.1926e-03,     
-4.0103e-03,    -3.8322e-03,    -3.6583e-03,    -3.4886e-03,    -3.3230e-03,    -3.1614e-03,    -3.0038e-03,    -2.8501e-03,     
-2.7003e-03,    -2.5543e-03,    -2.4121e-03,    -2.2737e-03,    -2.1389e-03,    -2.0077e-03,    -1.8800e-03,    -1.7559e-03,     
-1.6353e-03,    -1.5180e-03,    -1.4041e-03,    -1.2936e-03,    -1.1862e-03,    -1.0821e-03,    -9.8119e-04,    -8.8337e-04,     
-7.8861e-04,    -6.9688e-04,    -6.0813e-04,    -5.2231e-04,    -4.3938e-04,    -3.5931e-04,    -2.8203e-04,    -2.0753e-04,     
-1.3575e-04,    -6.6651e-05,    -2.0576e-07,     6.3628e-05,     1.2489e-04,     1.8360e-04,     2.3981e-04,     2.9355e-04,     
 3.4485e-04,     3.9373e-04,     4.4023e-04,     4.8437e-04,     5.2619e-04,     5.6570e-04,     6.0294e-04,     6.3793e-04,     
 6.7068e-04,     7.0122e-04,     7.2957e-04,     7.5574e-04,     7.7976e-04,     8.0164e-04,     8.2139e-04,     8.3903e-04,     
 8.5456e-04,     8.6801e-04,     8.7937e-04,     8.8865e-04,     8.9587e-04,     9.0102e-04,     9.0410e-04,     9.0513e-04

}; 


#endif /* __FLAT_TOP_WIN_H */