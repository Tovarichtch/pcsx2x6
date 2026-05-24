#include "ACATA.h"
#include "ACATAPI.h"
#include "common/Console.h"

#include "ACMACROS.h"

#include "ACATA_CMD_RESPONSES"

std::string ACATA::imgpath = "";
ACMEDIATYPE ACATA::MediaType;

u32 ACATA::last_read = 0;
u32 ACATA::last_write = 0;
u32 atacmd_responding = -1;
u32 atacmd_response_traverse = 0;

u16 ACATA::read16(u32 addr) {
    last_read = addr;
    switch (addr)
    {
    case ACATA_R_DATA:
        return ACATA::handle_dataR(addr);
    case ACATA_R_STATUS_ALT:
        return R_STATUS;
    case ACATA_R_NSECTOR:
        return R_NSECTOR;
    case ACATA_R_SECTOR:
        return R_SECTOR;
    case ACATA_R_ERROR:
        return R_ERROR;
    case ACATA_R_SELECT:
        return R_SELECT;
    case ACATA_R_LCYL:
        return R_LCYL;
    case ACATA_R_HCYL:
        return R_HCYL;
    case ACATA_R_STATUS:
        if (ACATA::last_write == ACATA_R_STATUS && ACATA::cmd_handled == 0 && ((R_SELECT & ACATA_UNIT1) == 0)) {
            // reading R_STATUS after writing zero to it? this is the ACATA probe. we have to respond BUSY at least once for the driver to keep going
            // FIXME
            if (R_STATUS & ATA_STAT_BUSY)
                CLRB(R_STATUS, ATA_STAT_BUSY);
            else
                R_STATUS |= ATA_STAT_BUSY;
        }
        return R_STATUS;
    
    default:
    	Console.Error("%-16s %08X: ", "ACATA::read16", addr);
        break;
    }
    return 0;
}

#define MMIO_RESPOND(V, O, N) if (V == O) V = N


void ACATA::write16(u32 addr, u16 val) {
    last_write = addr;
    u16 V = val;
    switch (addr) {
    case ACATA_R_NSECTOR: R_NSECTOR = val & 0xFF; break;
    case ACATA_R_SECTOR:  R_SECTOR  = val & 0xFF; break;
    case ACATA_R_FEATURE: R_FEATURE = val & 0xFF; break;
    case ACATA_R_CONTROL: R_CONTROL = val & 0xFF; break;
    case ACATA_R_LCYL:    R_LCYL    = val & 0xFF; break;
    case ACATA_R_HCYL:    R_HCYL    = val & 0xFF; break;
    case ACATA_R_SELECT:  R_SELECT  = val & 0xFF; break;
    case ACATA_R_COMMAND:
        ACATA::handle_cmd(val); return;
    
    case ACATA_R_DATA:
        ACATA::handle_dataW(addr, val);
    break;

    default:
    	Console.Error("%-16s %08X = %04X", "ACATA::write16", addr, val);
        break;
    }
}

u32 ACATA::cmd_handled;
u32 ACATA::cmd_handledc;
atapi_packet_t ACATA::ata_c_packet;

void ACATA::handle_dataW(u32 addr, u16 val) { // writes at R_DATA
    switch (ACATA::cmd_handled) {
    case ATA_C_PACKET:
        if (ACATA::cmd_handledc < 6) { // packet is followed by a 6 word PIO
            ACATA::ata_c_packet.raw[ACATA::cmd_handledc++] = val;
            if (ACATA::cmd_handledc == 6) {
                CLRB(R_STATUS, ATA_STAT_DRQ); // packet reception done
                ACATAPI::handle_cmd(ACATA::ata_c_packet); // may re-set DRQ for data-in commands
            }
        }
        break;
    
    default:
        Console.Error("ACATA: writing from %X while no pending CMD", addr);
        break;
    }
}

u16 ACATA::handle_dataR(u32 addr) { // PIO read at R_DATA
    switch (ACATA::cmd_handled) {
    case ATA_C_IDENTIFY_PACKET_DEVICE:
        if (ACATA::cmd_handledc < 256) {
            return ATA_R_IDENTIFY_PACKET_DEVICE[ACATA::cmd_handledc++];
        } else {ACATA::cmd_handled = -1; CLRB(R_STATUS, ATA_STAT_DRQ); R_STATUS |= ATA_STAT_READY;}
        break;
    case ATA_C_SET_FEATURES:
    break;
    case ATA_C_PACKET:
        if (ACATAPI::has_pio_data())
            return ACATAPI::pio_read_word();
        break;
    
    default:
        Console.Error("ACATA: reading from %X while no pending CMD", ACATA_R_DATA);
    }
    return R_DATA;
}

void ACATA::handle_cmd(u16 val) {
    switch (val) {
    case ATA_C_NOP:
        ACATA::cmd_handled = val;
        // the only situation when we will recieve a 0 (twice) to this reg, is during ACATA init.
        // here, the ata busy flag must be present at least once, on the following checks to this addr
        break;
    case ATA_C_IDENTIFY_PACKET_DEVICE:
        Console.Warning("ATA_C_IDENTIFY_PACKET_DEVICE");
        ACATA::cmd_handled = val;
        ACATA::cmd_handledc = 0;
        R_STATUS |= ATA_STAT_DRQ;
    break;
    case ATA_C_PACKET:
        ACATA::cmd_handled = val;
        ACATA::cmd_handledc = 0;
        if (!ACATA_ISDMA) Console.Error("ATA_C_PACKET: over PIO recieved");
        R_NSECTOR = 0x01; // ATAPI: CoD=1, IO=0 → ready for command packet
        R_STATUS |= ATA_STAT_DRQ;
        CLRB(R_STATUS, ATA_STAT_BUSY);
        CLRB(R_STATUS, ATA_STAT_ERR);
        CLRB(R_STATUS_ALT, ATA_STAT_ERR);
    break;
    case ATA_C_SET_FEATURES:
        Console.Warning("ATA_C_SET_FEATURES: FEATURE:%04X, R_NSECTOR:%04X, R_SECTOR:%04X, R_LCYL:%04X, R_HCYL:%04X",
            R_FEATURE, R_NSECTOR, R_SECTOR, R_LCYL, R_HCYL);
    break;
    
    
    default:
        Console.Error("unhandled ATACMD %X", val);
        break;
    }
}

void ACATA_SETUP() {
    ACATA::TH::readBuffer = new u8[ACATA::TH::readBufferLen];
}

#include "common/Path.h"

void ACATA::SetImgPath(const char* S) {
	ACATA::imgpath = Path::ToNativePath(Path::GetDirectory(S));
	Console.WriteLn("arcade image lookup path set to '%s'\n", ACATA::imgpath.c_str());
}

void ACATA::SetEnv(std::string ata_img_path, std::string ata_img_filename, std::string Media) {
	//ACATA::imgpath = Path::ToNativePath(Path::GetDirectory(ata_img_path));
	Console.WarningFmt("ata_img_path '{}' | ata_img_filename '{}'", ata_img_path, ata_img_filename);
    ACATA::imgpath = Path::Combine(ata_img_path, ata_img_filename);
    ACATA::MediaType = ACMEDIATYPE_FROM_STRING(Media);
	Console.WarningFmt("ACENV: ata_img: '{}'", ACATA::imgpath);
	Console.WarningFmt("mediat: '{}'", (int)ACATA::MediaType);
}


u16 ACATA::R_DATA;
u16 ACATA::R_FEATURE;
u16 ACATA::R_ERROR;
u16 ACATA::R_NSECTOR;
u16 ACATA::R_SECTOR;
u16 ACATA::R_LCYL;
u16 ACATA::R_HCYL;
u16 ACATA::R_SELECT;
u16 ACATA::R_STATUS;
u16 ACATA::R_COMMAND;
u16 ACATA::R_STATUS_ALT;
u16 ACATA::R_CONTROL;
