#include "DEV9.h"
#include "IopDma.h"
#include "ACCORE.h"
#include "ACJV.h"
#include "ACMACROS.h"
#include "common/Console.h"

#define ANS(addr, what) case addr: Console.Error("%-16s %08X: %04X", __FUNCTION__, addr, what); return what

enum ACCORE::DMA::TT ACCORE::DMA::PendTrasnfType;

u16 INTR_REG = 0;

u16 ACCORE::Read16(u32 mem) {
    switch (mem) {
	case 0x1241C000:
    	return INTR_REG;
    
    break;
    default: Console.Error("%-16s %08X:  %04X", "ACUNK::Read16", mem, 0); return 0;
    }
    return 0;
}

void ACCORE::Write16(u32 mem, u16 value) {
		switch (mem) {
		case ACJV_CTR_START: Console.Warning("ACJV::START"); ACJV::enabled = true; break;
		case ACJV_CTR_STOP:  Console.Warning("ACJV::STOP");  ACJV::enabled = false;  break;
		case 0x1241510C:  Console.Warning("ACCORE::INTR  DISABLE_ACATA_INTR"); break;
		case 0x1241511C:  Console.Warning("ACCORE::INTR  DISABLE_ACUART_INTR"); break;
		// ACFPGA UPLOAD MMIO
		case 0x1241600A:
		case 0x12416014:
		case 0x12416016:
		case 0x12416018:
		case 0x1241601A:
			break;
		case ACCORE_FPGA_BEGIN_PROGRAM:
			INTR_REG |= (0x1000|0x2000);
			break;
		case ACCPRE_FPGA_FINISH_PROGRAM:
			CLRB(INTR_REG, (0x1000|0x2000));
			break;

		default: Console.Error("%-16s %08X = %04X", "ACUNK::write16", mem, value); break;
		}
}

bool ACCORE::hasPendingInterrupt() {
	return INTR_REG != 0;
}

void ACCORE::intr(int INTRN) {
	switch (INTRN)
	{
	case INTRN_ATA:
		INTR_REG |= CAUS_ATA;
		break;

	default:

		return;
	}
	dev9Irq(1);
}

void ACCORE::Interrupt(u32 mem, u16 v) {
	switch (mem)
	{
	case ACCORE_INTR_ATA:
		Console.Warning("ACCORE:  ACATA_INTR_CLEAR: %04X", v);
		CLRB(INTR_REG, CAUS_ATA);
		break;
	case ACCORE_INTR_UART:
		Console.Warning("ACCORE: ACUART_INTR_CLEAR: %04X", v);
		break;
	default:
		Console.Warning("ACCORE: unknown INTR write to: %08X", mem);
		break;
	}
}