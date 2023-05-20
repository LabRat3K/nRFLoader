#ifndef _rf_debug_h
#define _rf_debug_h

// -----  Menu type -----
typedef struct {
  char          name[17];      // 16 Characters plus 0x0 EOS
  uint8_t 	size;
  uint8_t 	offset;
} tRFRegInfo;

#endif


