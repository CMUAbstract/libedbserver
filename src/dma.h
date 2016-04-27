#ifndef DMA_H
#define DMA_H

#if defined(__TI_COMPILER_VERSION__)
#define __DMA_ACCESS_REG__      __SFR_FARPTR
#elif defined(__GNUC__)
#define __DMA_ACCESS_REG__      uintptr_t
#else
#error Compiler not supported!
#endif

#endif // DMA_H
