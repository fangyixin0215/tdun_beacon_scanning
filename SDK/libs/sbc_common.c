/**
 *************************************************************************************
 * @file    sbc_common.c
 * @brief   An implementation of SBC decoder for bluetooth.
 *
 * @author  Aixing.Li
 * @version V1.0.0
 *
 * &copy; 2017 BEKEN Corporation Ltd. All rights reserved.
 *
 *************************************************************************************
 */
#include "sbc_common.h"

/**
 * SBC constant table OFFSET4 for calculate bit allocation
 */
static const int8_t SBC_COMMON_OFFSET4[4][4] =
{
    { -1, 0, 0, 0 },
    { -2, 0, 0, 1 },
    { -2, 0, 0, 1 },
    { -2, 0, 0, 1 }
};

/**
 * SBC constant table OFFSET8 for calculate bit allocation
 */
static const int8_t SBC_COMMON_OFFSET8[4][8] =
{
    { -2, 0, 0, 0, 0, 0, 0, 1 },
    { -3, 0, 0, 0, 0, 0, 1, 2 },
    { -4, 0, 0, 0, 0, 0, 1, 2 },
    { -4, 0, 0, 0, 0, 0, 1, 2 }
};

/**
 * Calculates the CRC-8 of the first len bits in data
 */
static const uint8_t SBC_CRC_TABLE[256] =
{
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53,
    0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E,
    0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4,
    0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19,
    0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40,
    0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D,
    0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7,
    0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A,
    0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75,
    0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8,
    0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2,
    0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F,
    0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66,
    0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB,
    0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1,
    0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C,
    0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

static const uint16_t SBC_SAMPLE_RATES[] =
{
    16000, 32000, 44100, 48000
};

uint16_t sbc_common_sample_rate_get(uint32_t idx)
{
    return SBC_SAMPLE_RATES[idx];
}

uint8_t sbc_common_crc8(const uint8_t* data, uint32_t len)
{
    uint8_t  crc = 0x0f;
    uint8_t  octet;
    uint32_t i;

    crc = SBC_CRC_TABLE[crc ^ data[0]];
    crc = SBC_CRC_TABLE[crc ^ data[1]];

    data++; //Skip CRC byte

    for(i = 2; i < len / 8; i++)
    {
        crc = SBC_CRC_TABLE[crc ^ data[i]];
    }

    octet = data[i];

    for(i = 0; i < (len & 7); i++)
    {
        char bit = ((octet ^ crc) & 0x80) >> 7;

        crc   = ((crc & 0x7f) << 1) ^ (bit ? 0x1d : 0);
        octet = octet << 1;
    }

    return crc;
}

void sbc_common_bit_allocation(SbcCommonContext* sbc)
{
    int32_t  ch;
    int32_t  sb;
    int32_t  slicecount;
    int32_t  bitcount;
    int32_t  bitslice;
    int32_t  max_bitneed;
    int32_t  loudness;

    int8_t*  sf;
    int8_t*  bits;
    int32_t* bitneed;

    if((sbc->channel_mode == SBC_CHANNEL_MODE_MONO) || (sbc->channel_mode == SBC_CHANNEL_MODE_DUAL_CHANNEL))
    {
        for(ch = 0; ch < sbc->channel_mode + 1; ch++)
        {
            sf      = sbc->scale_factor[ch];
            bits    = sbc->bits[ch];
            bitneed = sbc->mem[ch];

            max_bitneed = 0;
    
            if(sbc->allocation_method == SBC_ALLOCATION_METHOD_SNR)
            {
                for(sb = 0; sb < sbc->subbands; sb++)
                {
                    bitneed[sb] = sf[sb];
                    if(bitneed[sb] > max_bitneed)
                    {
                        max_bitneed = bitneed[sb];
                    }
                }
            }
            else
            {
                uint8_t sri = sbc->sample_rate_index;

                for(sb = 0; sb<sbc->subbands; sb++)
                {
                    if(sf[sb] == 0)
                    {
                        bitneed[sb] = -5;
                    }
                    else
                    {
                        if(sbc->subbands == 4)
                        {
                            loudness = sf[sb] - SBC_COMMON_OFFSET4[sri][sb];
                        }
                        else
                        {
                            loudness = sf[sb] - SBC_COMMON_OFFSET8[sri][sb];
                        }

                        if(loudness > 0)
                        {
                            bitneed[sb] = loudness / 2;
                        }
                        else
                        {
                            bitneed[sb] = loudness;
                        }
                    }

                    if(bitneed[sb] > max_bitneed)
                    {
                        max_bitneed = bitneed[sb];
                    }
                }
            }

            bitcount   = 0;
            slicecount = 0;
            bitslice   = max_bitneed + 1;

            do
            {
                bitslice--;
                bitcount += slicecount;
                slicecount = 0;

                for(sb = 0; sb < sbc->subbands; sb++)
                {
                    if((bitneed[sb] > bitslice + 1) && (bitneed[sb] < bitslice + 16))
                    {
                        slicecount++;
                    }
                    else if(bitneed[sb] == bitslice + 1)
                    {
                        slicecount += 2;
                    }
                }
            }while(bitcount + slicecount < sbc->bitpool);

            if(bitcount + slicecount == sbc->bitpool)
            {
                bitcount += slicecount;
                bitslice--;
            }

            for(sb = 0; sb < sbc->subbands; sb++)
            {
                if(bitneed[sb] < bitslice + 2)
                {
                    bits[sb] = 0;
                }
                else
                {
                    bits[sb] = bitneed[sb] - bitslice;
                    if(bits[sb] > 16)
                    {
                        bits[sb] = 16;
                    }
                }
            }

            for(sb = 0; bitcount < sbc->bitpool && sb < sbc->subbands; sb++)
            {
                if((bits[sb] >= 2) && (bits[sb] < 16))
                {
                    bits[sb]++;
                    bitcount++;
                }
                else if((bitneed[sb] == bitslice+1) && (sbc->bitpool > bitcount + 1))
                {
                    bits[sb]  = 2;
                    bitcount += 2;
                }
            }

            for(sb = 0; bitcount < sbc->bitpool && sb < sbc->subbands; sb++)
            {
                if(bits[sb] < 16)
                {
                    bits[sb]++;
                    bitcount++;
                }
            }
        }
    }
    else if((sbc->channel_mode == SBC_CHANNEL_MODE_STEREO) || (sbc->channel_mode == SBC_CHANNEL_MODE_JOINT_STEREO))
    {
        max_bitneed = 0;
        if(sbc->allocation_method == SBC_ALLOCATION_METHOD_SNR)
        {
            for(ch = 0; ch < 2; ch++)
            {
                sf      = sbc->scale_factor[ch];
                bitneed = sbc->mem[ch];

                for(sb = 0; sb < sbc->subbands; sb++)
                {
                    bitneed[sb] = sf[sb];

                    if(bitneed[sb] > max_bitneed)
                    {
                        max_bitneed = bitneed[sb];
                    }
                }
            }
        }
        else
        {
            uint8_t sri = sbc->sample_rate_index;

            for(ch = 0; ch < 2; ch++)
            {
                sf      = sbc->scale_factor[ch];
                bitneed = sbc->mem[ch];

                for(sb = 0; sb < sbc->subbands; sb++)
                {
                    if(sf[sb] == 0)
                    {
                        bitneed[sb] = -5;
                    }
                    else
                    {
                        if(sbc->subbands == 4)
                        {
                            loudness = sf[sb] - SBC_COMMON_OFFSET4[sri][sb];
                        }
                        else
                        {
                            loudness = sf[sb] - SBC_COMMON_OFFSET8[sri][sb];
                        }

                        if(loudness > 0)
                        {
                            bitneed[sb] = loudness / 2;
                        }
                        else
                        {
                            bitneed[sb] = loudness;
                        }
                    }

                    if(bitneed[sb] > max_bitneed)
                    {
                        max_bitneed = bitneed[sb];
                    }
                }
            }
        }

        bitcount   = 0;
        slicecount = 0;
        bitslice   = max_bitneed + 1;

        do
        {
            bitslice--;
            bitcount += slicecount;
            slicecount = 0;

            for(ch = 0; ch < 2; ch++)
            {
                bitneed = sbc->mem[ch];

                for(sb = 0; sb < sbc->subbands; sb++)
                {
                    if((bitneed[sb] > bitslice + 1) && (bitneed[sb] < bitslice + 16))
                    {
                        slicecount++;
                    }
                    else if(bitneed[sb] == bitslice + 1)
                    {
                        slicecount += 2;
                    }
                }
            }
        }while(bitcount + slicecount < sbc->bitpool);

        if(bitcount + slicecount == sbc->bitpool)
        {
            bitcount += slicecount;
            bitslice--;
        }

        for(ch = 0; ch < 2; ch++)
        {
            bits    = sbc->bits[ch];
            bitneed = sbc->mem[ch];

            for(sb = 0; sb < sbc->subbands; sb++)
            {
                if(bitneed[sb] < bitslice + 2)
                {
                    bits[sb] = 0;
                }
                else
                {
                    bits[sb] = bitneed[sb] - bitslice;
                    if(bits[sb] > 16)
                    {
                        bits[sb] = 16;
                    }
                }
            }
        }

        sb = 0;

        while(bitcount < sbc->bitpool)
        {
            bits    = sbc->bits[0];
            bitneed = sbc->mem[0];

            if((bits[sb] >= 2) && (bits[sb] < 16))
            {
                bits[sb]++;
                bitcount++;
            }
            else if((bitneed[sb] == bitslice + 1) && (sbc->bitpool > bitcount + 1))
            {
                bits[sb]  = 2;
                bitcount += 2;
            }

            if(bitcount >= sbc->bitpool)
            {
                break;
            }

            bits    = sbc->bits[1];
            bitneed = sbc->mem[1];

            if((bits[sb] >= 2) && (bits[sb] < 16))
            {
                bits[sb]++;
                bitcount++;
            }
            else if((bitneed[sb] == bitslice + 1) && (sbc->bitpool > bitcount + 1))
            {
                bits[sb]  = 2;
                bitcount += 2;
            }

            if(++sb >= sbc->subbands)
            {
                break;
            }
        }

        sb = 0;

        while(bitcount < sbc->bitpool)
        {
            bits = sbc->bits[0];

            if(bits[sb] < 16)
            {
                bits[sb]++;

                if(++bitcount >= sbc->bitpool)
                {
                    break;
                }                
            }            

            bits = sbc->bits[1];

            if(bits[sb] < 16)
            {
                bits[sb]++;
                bitcount++;
            }

            if(++sb >= sbc->subbands)
            {
                break;
            }
        }
    }
}
