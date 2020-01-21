// Dutchtronix AVR Oscilloscope Clock
//
//  Copyright © 2010 Johannes P.M. de Rie
//
//  All Rights Reserved
//
//  This file is part of the Dutchtronix Oscilloscope Clock Distribution.
//  Use, modification, and re-distribution is permitted subject to the
//  terms in the file named "LICENSE.TXT", which contains the full text
//  of the legal notices and should always accompany this Distribution.
//
//  This software is provided "AS IS" with NO WARRANTY OF ANY KIND.
//
//  This notice (including the copyright and warranty disclaimer)
//  must be included in all copies or derivations of this software.
//
// Flash Support declarations
//
FlashTblEntry_t* AddFlashItem(ScanTblEntry_t* pScanArg, byte flashCnt, VectorTblEntry_t* pVectAltArg);
FlashTblEntry_t* CheckFlashItem(ScanTblEntry_t* pScanArg, VectorTblEntry_t* pVectArg);
void  RemoveFlashItem(FlashTblEntry_t **pFlashPtr);
void InitFlashTbl(void);
void ProcessFlashTbl(void);
void UpdateFlashEntry(FlashTblEntry_t* pFlashPtr, ScanTblEntry_t* pScanArg, VectorTblEntry_t* pVectNew);

