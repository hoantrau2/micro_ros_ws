/*--------------------------------------------------------------------------*/
/* Copyright 2020-2021 NXP                                                  */
/*                                                                          */
/* NXP Confidential. This software is owned or controlled by NXP and may    */
/* only be used strictly in accordance with the applicable license terms.   */
/* By expressly accepting such terms or by downloading, installing,         */
/* activating and/or otherwise using the software, you are agreeing that    */
/* you have read, and that you agree to comply with and are bound by, such  */
/* license terms. If you do not agree to be bound by the applicable license */
/* terms, then you may not retain, install, activate or otherwise use the   */
/* software.                                                                */
/*--------------------------------------------------------------------------*/

/** @file  mcuxClRsa_NoVerify.c
 *  @brief mcuxClRsa: function, which is called in case of no verification
 */

#include <stdint.h>

#include <mcuxCsslFlowProtection.h>
#include <mcuxClPkc.h>
#include <internal/mcuxClPkc_Macros.h>
#include <internal/mcuxClPkc_ImportExport.h>

#include <mcuxClRsa.h>
#include <internal/mcuxClRsa_Internal_Functions.h>
#include <internal/mcuxClRsa_Internal_Types.h>
#include <internal/mcuxClRsa_Internal_PkcDefs.h>

const mcuxClRsa_SignVerifyMode_t mcuxClRsa_Mode_Verify_NoVerify =
{
  .EncodeVerify_FunId = MCUX_CSSL_FP_FUNCTION_CALLED(mcuxClRsa_noVerify),
  .pHashAlgo1 = NULL,
  .pHashAlgo2 = NULL,
  .pPaddingFunction = mcuxClRsa_noVerify
};

MCUX_CSSL_FP_FUNCTION_DEF(mcuxClRsa_noVerify)
mcuxClRsa_Status_Protected_t mcuxClRsa_noVerify(
  mcuxClSession_Handle_t       pSession,
  const uint8_t * const       pInput,
  const uint32_t              inputLength,
  uint8_t * const             pVerificationInput,
  const mcuxClHash_Algo_t *    pHashAlgo,
  const uint8_t *             pLabel,
  const uint32_t              saltlabelLength,
  const uint32_t              keyBitLength,
  const uint32_t              options,
  uint8_t * const             pOutput)
{
  MCUX_CSSL_FP_FUNCTION_ENTRY(mcuxClRsa_noVerify);

  uint16_t * pOperands = (uint16_t *)&pSession->cpuWa.buffer[pSession->cpuWa.used];
  pOperands[MCUXCLRSA_INTERNAL_UPTRTINDEX_NOVERIFY_IN] = MCUXCLPKC_PTR2OFFSET(pVerificationInput);
  pOperands[MCUXCLRSA_INTERNAL_UPTRTINDEX_NOVERIFY_TMP] = MCUXCLPKC_PTR2OFFSET((uint8_t *) (& pSession->pkcWa.buffer[pSession->pkcWa.used]));

  MCUXCLPKC_SETUPTRT(pOperands);

  uint32_t keyByteLength = keyBitLength / 8U;
  MCUXCLPKC_PS1_SETLENGTH(0u, MCUXCLPKC_ROUNDUP_SIZE(keyByteLength));

  MCUXCLPKC_FP_SECUREEXPORTBIGENDIANFROMPKC((uint8_t * )pOutput, MCUXCLRSA_INTERNAL_UPTRTINDEX_NOVERIFY_IN,
      MCUXCLRSA_INTERNAL_UPTRTINDEX_NOVERIFY_TMP, keyByteLength);

  MCUX_CSSL_FP_FUNCTION_EXIT(mcuxClRsa_noVerify, MCUXCLRSA_STATUS_VERIFYPRIMITIVE_OK,
  	      MCUX_CSSL_FP_FUNCTION_CALLED(mcuxClPkc_SecureExportBigEndianFromPkc));
}
