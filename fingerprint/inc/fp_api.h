#ifndef __FP_API_H
#define __FP_API_H

#include "ufvp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct {

		ufvp_module_t	common;

		/**
		 * @brief	对指纹图像数据进行特征提取,生成指纹特征数据
		 *
		 * @Param	pFingerImgBuf	指纹图像数据指针,输入参数
		 * @Param	nWidth			图像的宽度信息,输入参数
		 * @Param	nHeight			图像的高度信息,输入参数
		 * @Param	pFeatureData	指纹特征数据指针,存储生成的指纹特征数据,由调用者分配内存空间,输出参数
		 * @Param	pExternPar		备用,输入/出参数
		 *
		 * @retval	成功:返回0;失败:返回错误代码
		 */
		int (*FeatureExtract)(
				unsigned char	*pFingerImgBuf,
				unsigned int	nWidth,
				unsigned int	nHeight,
				unsigned char	*pFeatureData,
				void			*pExternPar);

		/**
		 * @brief	将指纹特征数据与数据库中的指纹特征数据比对,得到相似度
		 *
		 * @Param	pFeatureData		指纹特征数据指针,输入参数
		 * @Param	pFeatureDataBase	数据库中指纹特征数据指针,输入参数
		 * @Param	pfSimilarity		相似度,取值范围为0.00~1.00,值0.00表示不匹配,值1.00表示完全匹配,输出参数
		 * @Param	pExternPar			备用,输入/出参数
		 *
		 * @retval	成功:返回0;失败:返回错误代码
		 */
		int (*FeatureMatch)(
				unsigned char	*pFeatureData,
				unsigned char	*pFeatureDataBase,
				float			*pfSimilarity,
				void			*pExternPar);

		/**
		 * @brief	合并指纹特征数据到数据库中的指纹特征数据
		 *
		 * @Param	pFeatureData		指纹特征数据指针,输入参数
		 * @Param	pFeatureDataBase	数据库中指纹特征数据指针,输入参数
		 * @Param	pExternPar			备用,输入/出参数
		 *
		 * @retval	成功:返回0;失败:返回错误代码
		 */
		int (*FeatureEnroll)(
				unsigned char	*pFeatureData,
				unsigned char	*pFeatureDataBase,
				void			*pExternPar);


	}fp_core_t;

#ifdef __cplusplus
}
#endif

#endif
