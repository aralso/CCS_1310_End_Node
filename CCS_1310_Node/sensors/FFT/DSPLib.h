#ifndef __DSPLIB_H__
#define __DSPLIB_H__

#ifdef __USE_IQMATHLIB__ // Include and use QmathLib and IQmathLib libraries

#include "QmathLib.h"
#include "IQmathLib.h"

#endif  //__USE_IQMATHLIB__

/*#include "DSPLib_types.h"               // Include DSPLib type definitions
#include "DSPLib_support.h"             // Include DSPLib support functions
#include "DSPLib_vector.h"              // Include DSPLib vector functions
#include "DSPLib_matrix.h"              // Include DSPLib matrix functions
#include "DSPLib_filter.h"              // Include DSPLib filter functions
#include "DSPLib_transform.h"           // Include DSPLib transform functions
#include "DSPLib_utility.h"             // Include DSPLib utility functions*/


// **************** DSPLib type definitions ***************************


#ifndef __QMATHLIB_H__ // Define the _q15 type if it is not defined by the QmathLib header file.

/*!
 *  @brief Signed fixed point data type with 1 integer bit and 15 fractional bits.
 */
typedef int16_t _q15;

#endif  //__QMATHLIB_H__

#ifndef __IQMATHLIB_H__ // Define the _iq15 type if it is not defined by the IQmathLib header file.

/*!
 *  @brief Signed fixed point data type with 17 integer bits and 15 fractional bits.
 */
typedef int32_t _iq15;

#endif  //__QMATHLIB_H__

/*!
 *  @brief Signed fixed point data type with 1 integer bit and 31 fractional bits.
 */
typedef int32_t _q31;

/*!
 *  @brief Signed fixed point data type with 33 integer bits and 31 fractional bits.
 */
typedef int64_t _iiq31;

/*!
 *  @brief Unsigned fixed point data type with 1 integer bit and 15 fractional bits.
 */
typedef uint16_t _uq15;

/*!
 *  @brief Unsigned fixed point data type with 1 integer bit and 31 fractional bits.
 */
typedef uint32_t _uq31;

/*!
 *  @brief Enumerated type to return the status of an operation.
 */
typedef enum {
    MSP_SUCCESS,            /*!< Successful operation. */
    MSP_SIZE_ERROR,         /*!< Invalid size, see API for restrictions. */
    MSP_SHIFT_SIZE_ERROR,   /*!< Invalid shift size, see API for restrictions. */
    MSP_TABLE_SIZE_ERROR    /*!< Invalid table size, see API for restrictions. */
} msp_status;


// **************** DSPLib support functions  ***************************


/*!
 *  @brief Offset used to store table size.
 */
#define DSPLIB_TABLE_OFFSET     2

/*!
 *  @brief Increment size for complex data.
 */
#define CMPLX_INCREMENT (2)

/*!
 *  @brief Access the real portion of complex data.
 */
#define CMPLX_REAL(ptr) ((ptr)[0])

/*!
 *  @brief Access the imaginary portion of complex data.
 */
#define CMPLX_IMAG(ptr) ((ptr)[1])

#ifdef __USE_IQMATHLIB__ // Provide definition for the hidden _IQ31mpy function.

extern int32_t _IQ31mpy(int32_t A, int32_t B);

/*!
 *  @brief Fractional Q15 multiply function with 16-bit arguments and results.
 */
#define __mpyf_w(A, B)  _Q15mpy((A), (B))

/*!
 *  @brief Fractional Q31 multiply function with 32-bit arguments and results.
 */
#define __mpyf_l(A, B)  _IQ31mpy((A), (B))

#else
/*!
 *  @brief Fractional Q15 multiply function with 16-bit arguments and results.
 */
#define __mpyf_w(A, B)  ((int16_t)(((int32_t)(A) * (int32_t)(B)) >> 15))

/*!
 *  @brief Fractional Q31 multiply function with 32-bit arguments and results.
 */
#define __mpyf_l(A, B)  ((int32_t)(((int64_t)(A) * (int64_t)(B)) >> 31))

#endif //__USE_IQMATHLIB__


// **************** DSPLib vector functions ***************************


/*!
 *  @brief Parameter structure for the vector add functions.
 */
typedef struct msp_add_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_add_q15_params;

/*!
 *  @brief Parameter structure for the vector subtract functions.
 */
typedef struct msp_sub_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_sub_q15_params;

/*!
 *  @brief Parameter structure for the vector multiply functions.
 */
typedef struct msp_mpy_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_mpy_q15_params;

/*!
 *  @brief Parameter structure for the vector multiply and accumulate function.
 */
typedef struct msp_mac_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_mac_q15_params;

/*!
 *  @brief Parameter structure for the vector negate function.
 */
typedef struct msp_neg_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_neg_q15_params;

/*!
 *  @brief Parameter structure for the vector absolute value function.
 */
typedef struct msp_abs_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_abs_q15_params;

/*!
 *  @brief Parameter structure for the vector offset function.
 */
typedef struct msp_offset_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
    _q15        offset;             /*!< Offset to add to each vector element. */
} msp_offset_q15_params;

/*!
 *  @brief Parameter structure for the vector scale function.
 */
typedef struct msp_scale_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
    _q15        scale;              /*!< Q15 fractional scale to multiply each vector element by. */
    uint8_t     shift;              /*!< Unsigned integer value to shift each vector result left by. */
} msp_scale_q15_params;

/*!
 *  @brief Parameter structure for the vector shift function.
 */
typedef struct msp_shift_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
    int8_t      shift;              /*!< Integer value to shift each vector element by. Positive values will shift left and negative values will shift right. */
} msp_shift_q15_params;

/*!
 *  @brief Parameter structure for the signed vector maximum function.
 */
typedef struct msp_max_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_max_q15_params;

/*!
 *  @brief Parameter structure for the unsigned vector maximum function.
 */
typedef struct msp_max_uq15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_max_uq15_params;

/*!
 *  @brief Parameter structure for the signed vector minimum function.
 */
typedef struct msp_min_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_min_q15_params;

/*!
 *  @brief Parameter structure for the unsigned vector minimum function.
 */
typedef struct msp_min_uq15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_min_uq15_params;

/*!
 *  @brief Parameter structure for the complex vector add functions.
 */
typedef struct msp_cmplx_add_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_cmplx_add_q15_params;

/*!
 *  @brief Parameter structure for the complex vector subtract functions.
 */
typedef struct msp_cmplx_sub_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_cmplx_sub_q15_params;

/*!
 *  @brief Parameter structure for the complex vector multiply functions.
 */
typedef struct msp_cmplx_mpy_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_cmplx_mpy_q15_params;

/*!
 *  @brief Parameter structure for the complex vector multiply and accumulate function.
 */
typedef struct msp_cmplx_mac_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_cmplx_mac_q15_params;

/*!
 *  @brief Parameter structure for the complex vector conjugate function.
 */
typedef struct msp_cmplx_conj_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_cmplx_conj_q15_params;

/*!
 *  @brief Parameter structure for the complex vector scale by real function.
 */
typedef struct msp_cmplx_scale_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
    _q15        scale;              /*!< Q15 real fractional scale to multiply each complex vector element by. */
    uint8_t     shift;              /*!< Unsigned integer value to shift each complex vector result left by. */
} msp_cmplx_scale_q15_params;

/*!
 *  @brief Parameter structure for the complex vector shift function.
 */
typedef struct msp_cmplx_shift_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
    int8_t      shift;              /*!< Integer value to shift each complex vector element by. Positive values will shift left and negative values will shift right. */
} msp_cmplx_shift_q15_params;

/*!
 *  @brief Element wise addition of two source vectors.
 *  @param params Pointer to the vector add parameter structure.
 *  @param srcA Pointer to the first source data vector.
 *  @param srcB Pointer to the second source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_add_q15(const msp_add_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise subtraction of two source vectors.
 *  @param params Pointer to the vector add parameter structure.
 *  @param srcA Pointer to the first source data vector.
 *  @param srcB Pointer to the second source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_sub_q15(const msp_sub_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication of two source vectors.
 *  @param params Pointer to the vector multiply parameter structure.
 *  @param srcA Pointer to the first source data vector.
 *  @param srcB Pointer to the second source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_mpy_q15(const msp_mpy_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication and accumulate (dot product) of two source vectors.
 *  @param params Pointer to the vector multiply and accumulate parameter structure.
 *  @param srcA Pointer to the first source data vector.
 *  @param srcB Pointer to the second source data vector.
 *  @param dst Pointer to the 32-bit destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_mac_q15(const msp_mac_q15_params *params, const _q15 *srcA, const _q15 *srcB, _iq15 *result);

/*!
 *  @brief Element wise multiplication of a single source vector with negative one.
 *  @param params Pointer to the vector negate parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_neg_q15(const msp_neg_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise absolute value of a single source vector.
 *  @param params Pointer to the vector absolute value parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_abs_q15(const msp_abs_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise addition of a single source vector with an offset.
 *  @param params Pointer to the vector offset parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_offset_q15(const msp_offset_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication of a single source vector with a Q15 scale value and shift left by scale.
 *  @param params Pointer to the vector scale structure.
 *  @param src Pointer to the source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function can return MSP_SUCCESS or MSP_SHIFT_SIZE_ERROR if an invalid shift parameter is given.
 */
extern msp_status msp_scale_q15(const msp_scale_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise left or right shift of a single source vector.
 *  @param params Pointer to the vector shift parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function can return MSP_SUCCESS or MSP_SHIFT_SIZE_ERROR if an invalid shift parameter is given.
 */
extern msp_status msp_shift_q15(const msp_shift_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Find the signed maximum value of a single source vector.
 *  @param params Pointer to the signed vector maximum parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param max Pointer to the maximum result vector of size one.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_max_q15(const msp_max_q15_params *params, const _q15 *src, _q15 *max);

/*!
 *  @brief Find the unsigned maximum value of a single source vector.
 *  @param params Pointer to the unsigned vector maximum parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param max Pointer to the maximum result vector of size one.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_max_uq15(const msp_max_uq15_params *params, const _uq15 *src, _uq15 *max);

/*!
 *  @brief Find the signed minimum value of a single source vector.
 *  @param params Pointer to the signed vector minimum parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param min Pointer to the minimum result vector of size one.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_min_q15(const msp_min_q15_params *params, const _q15 *src, _q15 *min);

/*!
 *  @brief Find the unsigned minimum value of a single source vector.
 *  @param params Pointer to the unsigned vector minimum parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param min Pointer to the minimum result vector of size one.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_min_uq15(const msp_min_uq15_params *params, const _uq15 *src, _uq15 *min);

/*!
 *  @brief Element wise addition of two complex source vectors without saturation.
 *  @param params Pointer to the complex vector add parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_add_q15(const msp_cmplx_add_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise addition of two complex source vectors with saturation.
 *  @param params Pointer to the complex vector add parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_add_sat_q15(const msp_cmplx_add_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise subtraction of two complex source vectors without saturation.
 *  @param params Pointer to the complex vector add parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_sub_q15(const msp_cmplx_sub_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise subtraction of two complex source vectors with saturation.
 *  @param params Pointer to the complex vector add parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_sub_sat_q15(const msp_cmplx_sub_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);


/*!
 *  @brief Element wise Q15 multiplication of two complex source vectors.
 *  @param params Pointer to the complex vector multiply parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_mpy_q15(const msp_cmplx_mpy_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication of a complex source vector with a real source vector.
 *  @param params Pointer to the complex vector multiply parameter structure.
 *  @param srcCmplx Pointer to the complex source data vector.
 *  @param srcReal Pointer to the real source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_mpy_real_q15(const msp_cmplx_mpy_q15_params *params, const _q15 *srcCmplx, const _q15 *srcReal, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication and accumulate (dot product) of two complex source vectors.
 *  @param params Pointer to the complex vector multiply and accumulate parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the 32-bit complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_mac_q15(const msp_cmplx_mac_q15_params *params, const _q15 *srcA, const _q15 *srcB, _iq15 *result);

/*!
 *  @brief Element wise complex conjugate of a single complex source vector.
 *  @param params Pointer to the complex vector conjugate parameter structure.
 *  @param src Pointer to the complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_conj_q15(const msp_cmplx_conj_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication of a single complex source vector with a Q15 real scale value and shift left by scale.
 *  @param params Pointer to the complex vector scale structure.
 *  @param src Pointer to the complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function can return MSP_SUCCESS or MSP_SHIFT_SIZE_ERROR if an invalid shift parameter is given.
 */
extern msp_status msp_cmplx_scale_q15(const msp_cmplx_scale_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise left or right shift of a single complex source vector.
 *  @param params Pointer to the complex vector shift parameter structure.
 *  @param src Pointer to the complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function can return MSP_SUCCESS or MSP_SHIFT_SIZE_ERROR if an invalid shift parameter is given.
 */
extern msp_status msp_cmplx_shift_q15(const msp_cmplx_shift_q15_params *params, const _q15 *src, _q15 *dst);


// **************** DSPLib matrix functions ***************************


// **************** DSPLib filter functions ***************************

/*!
 *  @brief Parameter structure for the Goertzel algorithm filter.
 */
typedef struct msp_goertzel_q15_params {
    uint16_t    length;             /*!< Length of the source data. */
    _q15        cosCoeff;           /*!< Cos coefficient used for calculating the result. */
    _q15        sinCoeff;           /*!< Sin coefficient used for calculating the result. */
    int16_t     *output;            /*!< Complex vector of size one to store the result. */
} msp_goertzel_q15_params;

/*!
 *  @brief Goertzel algorithm filter to calculate a single frequency of the DFT.
 *  @param params Pointer to the Goertzel parameter structure.
 *  @param src Pointer to the source data to filter.
 *  @return none.
 */
extern msp_status msp_goertzel_q15(const msp_goertzel_q15_params *params, const _q15 *src, _q15 *dst);


// **************** DSPLib transform functions ***************************


/*!
 *  @brief Parameter structure for _q15 complex bit-reversal.
 */
typedef struct msp_cmplx_bitrev_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    const uint16_t          *bitReverseTable;   /*!< Pointer to the bit-reversal table descriptor to use with size greater than or equal to source length. */
} msp_cmplx_bitrev_q15_params;

/*!
 *  @brief Parameter structure for _q15 real FFT functions.
 */
typedef struct msp_fft_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    bool                    bitReverse;         /*!< Perform bit-reversal of input first. */
    const uint16_t          *bitReverseTable;   /*!< Pointer to the bit-reversal table descriptor to use with size greater than or equal to half the source length. This is not required if bitReverse is false. */
    const _q15              *twiddleTable;      /*!< Pointer to the twiddle coefficient table descriptor to use with size greater than or equal to half the source length. */
    const _q15              *splitTable;        /*!< Pointer to the split coefficient table with size greater than or equal to the source length. */
} msp_fft_q15_params;

/*!
 *  @brief Parameter structure for _q15 real inverse FFT functions.
 */
typedef struct msp_ifft_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    bool                    bitReverse;         /*!< Perform bit-reversal of input first. */
    const uint16_t          *bitReverseTable;   /*!< Pointer to the bit-reversal table descriptor to use with size greater than or equal to half the source length. This is not required if bitReverse is false. */
    const _q15              *twiddleTable;      /*!< Pointer to the twiddle coefficient table descriptor to use with size greater than or equal to half the source length. */
    const _q15              *splitTable;        /*!< Pointer to the split coefficient table with size greater than or equal to the source length. */
} msp_ifft_q15_params;

/*!
 *  @brief Parameter structure for _q15 complex FFT functions.
 */
typedef struct msp_cmplx_fft_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    bool                    bitReverse;         /*!< Perform bit-reversal of input first. */
    const uint16_t          *bitReverseTable;   /*!< Pointer to the bit-reversal table descriptor to use with size greater than or equal to source length. This is not required if bitReverse is false. */
    const _q15              *twiddleTable;      /*!< Pointer to the complex twiddle coefficient table descriptor to use with size greater than or equal to source length. */
} msp_cmplx_fft_q15_params;

/*!
 *  @brief Parameter structure for _q15 complex inverse FFT functions.
 */
typedef struct msp_cmplx_ifft_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    bool                    bitReverse;         /*!< Perform bit-reversal of input first. */
    const uint16_t          *bitReverseTable;   /*!< Pointer to the bit-reversal table descriptor to use with size greater than or equal to source length. This is not required if bitReverse is false. */
    const _q15              *twiddleTable;      /*!< Pointer to the complex twiddle coefficient table descriptor to use with size greater than or equal to source length. */
} msp_cmplx_ifft_q15_params;

/*!
 *  @brief Parameter structure for performing the split operation.
 */
typedef struct msp_split_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    const _q15              *splitTable;        /*!< Pointer to the split coefficient table with size greater than or equal to the source length. */
} msp_split_q15_params;

/*!
 *  @brief Parameter structure for performing the inverse split operation.
 */
typedef struct msp_isplit_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    const _q15              *splitTable;        /*!< Pointer to the split coefficient table with size greater than or equal to the source length. */
} msp_isplit_q15_params;

/*!
 *  @brief Complex bit-reversal function.
 *  @param params Pointer to the complex bit-reversal parameter structure.
 *  @param src Pointer to the complex data array to perform the bit-reversal on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_cmplx_bitrev_q15(const msp_cmplx_bitrev_q15_params *params, _q15 *src);

/*!
 *  @brief Real FFT function without fixed scaling at each stage.
 *  @param params Pointer to the real FFT parameter structure.
 *  @param src Pointer to the real data array to perform the FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_fft_q15(const msp_fft_q15_params *params, _q15 *src);

/*!
 *  @brief Real FFT function with fixed scaling by two at each stage.
 *  @param params Pointer to the real FFT parameter structure.
 *  @param src Pointer to the real data array to perform the FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_fft_scale_q15(const msp_fft_q15_params *params, _q15 *src);

/*!
 *  @brief Real inverse FFT function without fixed scaling at each stage.
 *  @param params Pointer to the real inverse FFT parameter structure.
 *  @param src Pointer to the real data array to perform the inverse FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_ifft_q15(const msp_ifft_q15_params *params, _q15 *src);

/*!
 *  @brief Real FFT function with fixed scaling by two at each stage.
 *  @param params Pointer to the real inverse FFT parameter structure.
 *  @param src Pointer to the real data array to perform the inverse FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_ifft_scale_q15(const msp_ifft_q15_params *params, _q15 *src);

/*!
 *  @brief Complex FFT function without fixed scaling at each stage.
 *  @param params Pointer to the complex FFT parameter structure.
 *  @param src Pointer to the complex data array to perform the FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_cmplx_fft_q15(const msp_cmplx_fft_q15_params *params, _q15 *src);

/*!
 *  @brief Complex FFT function with fixed scaling by two at each stage.
 *  @param params Pointer to the complex FFT parameter structure.
 *  @param src Pointer to the complex data array to perform the FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_cmplx_fft_scale_q15(const msp_cmplx_fft_q15_params *params, _q15 *src);

/*!
 *  @brief Complex inverse FFT function without fixed scaling at each stage.
 *  @param params Pointer to the complex inverse FFT parameter structure.
 *  @param src Pointer to the complex data array to perform the inverse FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_cmplx_ifft_q15(const msp_cmplx_ifft_q15_params *params, _q15 *src);

/*!
 *  @brief Complex inverse FFT function with fixed scaling by two at each stage.
 *  @param params Pointer to the complex inverse FFT parameter structure.
 *  @param src Pointer to the complex data array to perform the inverse FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_cmplx_ifft_scale_q15(const msp_cmplx_ifft_q15_params *params, _q15 *src);

/*!
 *  @brief Split operation for performing the final step of a real FFT.
 *  @param src Pointer to the data array to perform the split operation on.
 *  @param length Length of the data array.
 *  @param splitTable Pointer to the split operation table descriptor.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_split_q15(const msp_split_q15_params *params, _q15 *src);

/*!
 *  @brief Inverse split operation for performing the final step of a real inverse FFT.
 *  @param src Pointer to the data array to perform the inverse split operation on.
 *  @param length Length of the data array.
 *  @param splitTable Pointer to the split operation table descriptor.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_isplit_q15(const msp_isplit_q15_params *params, _q15 *src);

/*!
 *  @brief Bit reversal lookup table for size length 16.
 */
extern const uint16_t msp_cmplx_bitrev_table_16_ui16[DSPLIB_TABLE_OFFSET+16];

/*!
 *  @brief Bit reversal lookup table for size length 32.
 */
extern const uint16_t msp_cmplx_bitrev_table_32_ui16[DSPLIB_TABLE_OFFSET+32];

/*!
 *  @brief Bit reversal lookup table for size length 64.
 */
extern const uint16_t msp_cmplx_bitrev_table_64_ui16[DSPLIB_TABLE_OFFSET+64];

/*!
 *  @brief Bit reversal lookup table for size length 128.
 */
extern const uint16_t msp_cmplx_bitrev_table_128_ui16[DSPLIB_TABLE_OFFSET+128];

/*!
 *  @brief Bit reversal lookup table for size length 256.
 */
extern const uint16_t msp_cmplx_bitrev_table_256_ui16[DSPLIB_TABLE_OFFSET+256];

/*!
 *  @brief Bit reversal lookup table for size length 512.
 */
extern const uint16_t msp_cmplx_bitrev_table_512_ui16[DSPLIB_TABLE_OFFSET+512];

/*!
 *  @brief Bit reversal lookup table for size length 1024.
 */
extern const uint16_t msp_cmplx_bitrev_table_1024_ui16[DSPLIB_TABLE_OFFSET+1024];

/*!
 *  @brief Bit reversal lookup table for size length 2048.
 */
extern const uint16_t msp_cmplx_bitrev_table_2048_ui16[DSPLIB_TABLE_OFFSET+2048];

/*!
 *  @brief Bit reversal lookup table for size length 4096.
 */
extern const uint16_t msp_cmplx_bitrev_table_4096_ui16[DSPLIB_TABLE_OFFSET+4096];

/*!
*  @brief Twiddle factor table for FFT of size 16.
*/
extern const _q15 msp_cmplx_twiddle_table_16_q15[DSPLIB_TABLE_OFFSET+16];

/*!
*  @brief Twiddle factor table for FFT of size 32.
*/
extern const _q15 msp_cmplx_twiddle_table_32_q15[DSPLIB_TABLE_OFFSET+32];

/*!
*  @brief Twiddle factor table for FFT of size 64.
*/
extern const _q15 msp_cmplx_twiddle_table_64_q15[DSPLIB_TABLE_OFFSET+64];

/*!
*  @brief Twiddle factor table for FFT of size 128.
*/
extern const _q15 msp_cmplx_twiddle_table_128_q15[DSPLIB_TABLE_OFFSET+128];

/*!
*  @brief Twiddle factor table for FFT of size 256.
*/
extern const _q15 msp_cmplx_twiddle_table_256_q15[DSPLIB_TABLE_OFFSET+256];

/*!
*  @brief Twiddle factor table for FFT of size 512.
*/
extern const _q15 msp_cmplx_twiddle_table_512_q15[DSPLIB_TABLE_OFFSET+512];

/*!
*  @brief Twiddle factor table for FFT of size 1024.
*/
extern const _q15 msp_cmplx_twiddle_table_1024_q15[DSPLIB_TABLE_OFFSET+1024];

/*!
*  @brief Twiddle factor table for FFT of size 2048.
*/
extern const _q15 msp_cmplx_twiddle_table_2048_q15[DSPLIB_TABLE_OFFSET+2048];

/*!
*  @brief Twiddle factor table for FFT of size 4096.
*/
extern const _q15 msp_cmplx_twiddle_table_4096_q15[DSPLIB_TABLE_OFFSET+4096];

/*!
*  @brief Split coefficient table for real FFT of size 16.
*/
extern const _q15 msp_split_table_16_q15[DSPLIB_TABLE_OFFSET+8];

/*!
*  @brief Split coefficient table for real FFT of size 32.
*/
extern const _q15 msp_split_table_32_q15[DSPLIB_TABLE_OFFSET+16];

/*!
*  @brief Split coefficient table for real FFT of size 64.
*/
extern const _q15 msp_split_table_64_q15[DSPLIB_TABLE_OFFSET+32];

/*!
*  @brief Split coefficient table for real FFT of size 128.
*/
extern const _q15 msp_split_table_128_q15[DSPLIB_TABLE_OFFSET+64];

/*!
*  @brief Split coefficient table for real FFT of size 256.
*/
extern const _q15 msp_split_table_256_q15[DSPLIB_TABLE_OFFSET+128];

/*!
*  @brief Split coefficient table for real FFT of size 512.
*/
extern const _q15 msp_split_table_512_q15[DSPLIB_TABLE_OFFSET+256];

/*!
*  @brief Split coefficient table for real FFT of size 1024.
*/
extern const _q15 msp_split_table_1024_q15[DSPLIB_TABLE_OFFSET+512];

/*!
*  @brief Split coefficient table for real FFT of size 2048.
*/
extern const _q15 msp_split_table_2048_q15[DSPLIB_TABLE_OFFSET+1024];

/*!
*  @brief Split coefficient table for real FFT of size 4096.
*/
extern const _q15 msp_split_table_4096_q15[DSPLIB_TABLE_OFFSET+2048];



// **************** DSPLib utility functions ***************************


#endif //__DSPLIB_H__
