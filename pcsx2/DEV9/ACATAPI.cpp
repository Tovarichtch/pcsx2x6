#include "ACCORE.h"
#include "ACATAPI.h"
#include "ACATA.h"
#include "common/Console.h"

#include "ACMACROS.h"

#include <sys/stat.h>
#include <cstring>

static u8 atapi_pio_buf[65536];
static u32 atapi_pio_len = 0;
static u32 atapi_pio_pos = 0;

static void atapi_pio_setup(u32 len) {
    atapi_pio_len = len;
    atapi_pio_pos = 0;
    ACATA::R_LCYL = len & 0xFF;
    ACATA::R_HCYL = (len >> 8) & 0xFF;
    ACATA::R_NSECTOR = 0x02;
    ACATA::R_STATUS |= ATA_STAT_DRQ;
    CLRB(ACATA::R_STATUS, ATA_STAT_BUSY);
    CLRB(ACATA::R_STATUS, ATA_STAT_ERR);
    ACCORE::intr(ACCORE::INTRN_ATA);
}

static void atapi_complete_nodata() {
    atapi_pio_len = 0;
    atapi_pio_pos = 0;
    ACATA::R_NSECTOR = 0x03;
    CLRB(ACATA::R_STATUS, ATA_STAT_DRQ);
    CLRB(ACATA::R_STATUS, ATA_STAT_ERR);
    CLRB(ACATA::R_STATUS, ATA_STAT_BUSY);
    ACATA::R_STATUS |= ATA_STAT_READY;
    ACATA::R_LCYL = 0;
    ACATA::R_HCYL = 0;
    ACCORE::intr(ACCORE::INTRN_ATA);
}

u16 ACATAPI::pio_read_word() {
    if (atapi_pio_pos * 2 < atapi_pio_len) {
        u32 offset = atapi_pio_pos * 2;
        u16 val = atapi_pio_buf[offset];
        if (offset + 1 < atapi_pio_len)
            val |= (u16)atapi_pio_buf[offset + 1] << 8;
        atapi_pio_pos++;
        if (atapi_pio_pos * 2 >= atapi_pio_len) {
            ACATA::R_NSECTOR = 0x03;
            CLRB(ACATA::R_STATUS, ATA_STAT_DRQ);
            ACATA::R_STATUS |= ATA_STAT_READY;
            ACCORE::intr(ACCORE::INTRN_ATA);
        }
        return val;
    }
    return 0;
}

bool ACATAPI::has_pio_data() {
    return atapi_pio_len > 0 && atapi_pio_pos * 2 < atapi_pio_len;
}

void ACATAPI::handle_cmd(atapi_packet_t P) {
    u32 transf_lba = ATAPI_PKT_GETLBA(P);
    u32 nsec = ATAPI_PKT_GETLEN(P);
    switch (P.pkt.opcode) {
    case ATAPICMD::TEST_UNIT_READY:
        Console.Warning("ACATAPI:TEST_UNIT_READY");
        atapi_complete_nodata();
        break;

    case ATAPICMD::READ_CAPACITY: {
        Console.Warning("ACATAPI:READ_CAPACITY");
        struct stat st;
        if (ACATA::TH::IMAGE && fstat(fileno(ACATA::TH::IMAGE), &st) == 0 && st.st_size > 0) {
            u32 last_lba = (u32)(st.st_size / ACATAPI::CONSTANTS::DVD_SECTORSIZE) - 1;
            u32 block_len = ACATAPI::CONSTANTS::DVD_SECTORSIZE;
            Console.Warning("  last_lba=0x%08X, block_len=0x%08X", last_lba, block_len);
            atapi_pio_buf[0] = (last_lba >> 24) & 0xFF;
            atapi_pio_buf[1] = (last_lba >> 16) & 0xFF;
            atapi_pio_buf[2] = (last_lba >>  8) & 0xFF;
            atapi_pio_buf[3] = last_lba & 0xFF;
            atapi_pio_buf[4] = (block_len >> 24) & 0xFF;
            atapi_pio_buf[5] = (block_len >> 16) & 0xFF;
            atapi_pio_buf[6] = (block_len >>  8) & 0xFF;
            atapi_pio_buf[7] = block_len & 0xFF;
            atapi_pio_setup(8);
        } else {
            Console.Error("ACATAPI:READ_CAPACITY: image not open or fstat failed");
            ACATA::R_STATUS |= ATA_STAT_ERR;
            ACATA::R_ERROR = ATA_ERR_ABORT;
        }
        break;
    }

    case ATAPICMD::MODE_SENSE: {
        u8 page_code = P.raw8[2] & 0x3F;
        Console.Warning("ACATAPI:MODE_SENSE page=0x%02X", page_code);
        memset(atapi_pio_buf, 0, 8);
        // MODE_SENSE(10) header: 8 bytes
        atapi_pio_buf[0] = 0x00; // mode data length MSB
        atapi_pio_buf[1] = 0x06; // mode data length LSB (6 = 8 total - 2)
        // bytes 2-7: medium type, device-specific, reserved, block desc length = all 0
        atapi_pio_setup(8);
        break;
    }

    case ATAPICMD::READ_10: {
        Console.Warning("ACATAPI:READ_10: lba=%u, sectors=%u", transf_lba, nsec);
        if (!ACATA::TH::IMAGE || nsec == 0) {
            Console.Error("ACATAPI:READ_10: no image or zero sectors");
            ACATA::R_STATUS |= ATA_STAT_ERR;
            ACATA::R_ERROR = ATA_ERR_ABORT;
            atapi_complete_nodata();
            break;
        }
        u32 total = nsec * ACATAPI::CONSTANTS::DVD_SECTORSIZE;
        if (total > sizeof(atapi_pio_buf)) {
            Console.Error("ACATAPI:READ_10: transfer too large (%u bytes)", total);
            ACATA::R_STATUS |= ATA_STAT_ERR;
            ACATA::R_ERROR = ATA_ERR_ABORT;
            atapi_complete_nodata();
            break;
        }
        off_t offset = (off_t)transf_lba * ACATAPI::CONSTANTS::DVD_SECTORSIZE;
        fseeko(ACATA::TH::IMAGE, offset, SEEK_SET);
        size_t rd = fread(atapi_pio_buf, 1, total, ACATA::TH::IMAGE);
        if (rd == total) {
            atapi_pio_setup(total);
        } else {
            Console.Error("ACATAPI:READ_10: short read (%zu/%u)", rd, total);
            ACATA::R_STATUS |= ATA_STAT_ERR;
            ACATA::R_ERROR = ATA_ERR_ABORT;
            atapi_complete_nodata();
        }
        break;
    }

    default:
        Console.Error("ACATAPI: UNK_CMD %02X, lba:%08X, nsec:%04X", P.raw8[0], transf_lba, nsec);
        break;
    }
}

u16 ACATAPI::Read10(u32 lba, u16 tlen) {
    return 0;
}

void ACATAPI::Setup() {
    u32 SectorSizes[3] = {/*TODO: CHECK CD SECTOR SIZE*/0, ACATAPI::CONSTANTS::DVD_SECTORSIZE, 512};
    ACATA::TH::sectorsize = SectorSizes[ACATA::MediaType];
}
