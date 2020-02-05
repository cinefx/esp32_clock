// ClkDebug.h
//
extern void DebugSerialOut(char ch);
extern void DebugSerialu08(byte b);
extern void DebugSerialu16(uint w);
extern void DebugSerialOutStrFlash(PGM_P pSrc);
extern void DebugSerialOutStr(char *pSrc);
extern void ShowCnt(uint w);
extern void ShowSecSkippedMsg(void);
extern void ShowCModState(byte state, byte val);

void DebugAssert(uint line);
BOOL UsrDebugReport(char c);
BOOL UsrDebugSetting(byte mode);
BOOL UsrDebugSetting0(char c);


#if DEBUG
#define ASSERT(x) if (!(x)) {DebugAssert(__LINE__);}
#else
#define ASSERT(x)
#endif
