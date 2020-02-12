// Dutchtronix AVR Oscilloscope Clock
//
//  Copyright @ 2010 Johannes P.M. de Rie
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
#include <Arduino.h>
#include <pgmspace.h>

#include "./ClkConfig.h"
#include "./ClkData.h"
#include "./ClkFlash.h"
#include "./ClkDebug.h"

FlashTblEntry_t* AddFlashItem(ScanTblEntry_t* pScanArg, byte flashCnt, VectorTblEntry_t* pVectAltArg)
{
	byte i;
	FlashTblEntry_t* pF;
	for (i = 0; i < MaxFlashTblEntries; ++i) {
		pF = &FlashTbl[i];
		if (pF->pScan == NULL) {
			break;
		}
	}
	ASSERT(i < MaxFlashTblEntries);		//Should always be space
	if (i < MaxFlashTblEntries) {
		ASSERT(pF == &FlashTbl[i]);
		pF->pScan = pScanArg;
		pF->pVect = pF->pVectOrig = pScanArg->pVect;
		pF->stat = TRUE;
		pF->cnt = flashCnt;
		pF->pVectAlt = pVectAltArg;
	} else {
		--i;							//return dummy entry OR NULL??
	}
	return &(FlashTbl[i]);
}

FlashTblEntry_t* CheckFlashItem(ScanTblEntry_t* pScanArg, VectorTblEntry_t* pVectAltArg)
{
	byte i;
	for (i = 0; i < MaxFlashTblEntries; ++i) {
		FlashTblEntry_t* pF = &FlashTbl[i];
		if (pF->pScan != NULL) {
			if ((pF->pScan == pScanArg) && (pF->pVectAlt == pVectAltArg)) {
				return pF;
			}
		}
	}
	return NULL;		// not found
}

void RemoveFlashItemDirect(FlashTblEntry_t *pFlashPtr)
{
	if (pFlashPtr != NULL) {
		ScanTblEntry_t* p = pFlashPtr->pScan;
		p->pVect = pFlashPtr->pVectOrig;
		memset(pFlashPtr, 0, sizeof(FlashTblEntry_t));
	}
}

void RemoveFlashItem(FlashTblEntry_t **pFlashPtr)
{
	RemoveFlashItemDirect(*pFlashPtr);
	*pFlashPtr = NULL;
}

void InitFlashTbl(void)
{
//
// Unneeded in RELEASE mode since all memory is set to 0 at startup.
//
#if DEBUG
	memset(FlashTbl, 0, MaxFlashTblEntries* sizeof(FlashTblEntry_t));
#endif
}

void ProcessFlashTbl(void)
{
	for (byte i = 0; i < MaxFlashTblEntries; ++i) {
		FlashTblEntry_t* pFlashPtr = &FlashTbl[i];
		if (pFlashPtr->pScan != NULL) {
			VectorTblEntry_t* p1;
			ScanTblEntry_t*  p2;
			ASSERT(pFlashPtr->pScan != NULL);
			if (pFlashPtr->cnt != 255) {
				if (--pFlashPtr->cnt == 0) {
					RemoveFlashItemDirect(pFlashPtr);
					return;
				}
			}
			if (pFlashPtr->stat) {		//ON, turn OFF
				p1 = pFlashPtr->pVectAlt;
			} else {					//OFF, turn ON
				p1 = pFlashPtr->pVect;
			}
			p2 = pFlashPtr->pScan;
			p2->pVect = p1;				//Update ScanTable
			pFlashPtr->stat ^= 1;		//toggle
		}
	}
}
//
// Update the Vector Table Pointer for a currently flashing Entry
//
void UpdateFlashEntry(FlashTblEntry_t* pFlashPtr, ScanTblEntry_t* pScanArg, VectorTblEntry_t* pVectNew)
{
	if (pFlashPtr != NULL) {
		if (pFlashPtr->pScan == pScanArg) {
			pFlashPtr->pVect = pFlashPtr->pVectOrig = pVectNew;
		}
	}
}
