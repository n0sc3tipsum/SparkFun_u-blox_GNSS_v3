/*
  Use the NEO-D9S L-Band receiver to provide corrections as SPARTN data over Serial
  By: SparkFun Electronics / Paul Clark
  Based on original code by: u-blox AG / Michael Ammann
  v3 updates: Decembe 22nd, 2022
  License: MIT. See license file for more information.

  This example shows how to obtain SPARTN correction data from a NEO-D9S L-Band receiver and push it over Serial (UART).
  This example parses (extracts) SPARTN packets from the L-Band data stream and validates them before pushing.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/16481
  NEO-D9S:      https://www.sparkfun.com/products/19390
  Combo Board:  https://www.sparkfun.com/products/20167

  Hardware Connections:
  Use a Qwiic cable to connect the NEO-D9S to your board
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <driver/uart.h>      //Required for uart_set_rx_full_threshold() on cores <v2.0.5
HardwareSerial serialGNSS(2); // TX on 17, RX on 16

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myLBand; // NEO-D9S

//const uint32_t myLBandFreq = 1556290000; // Uncomment this line to use the US SPARTN 1.8 service
const uint32_t myLBandFreq = 1545260000; // Uncomment this line to use the EU SPARTN 1.8 service

#define OK(ok) (ok ? F("  ->  OK") : F("  ->  ERROR!")) // Convert uint8_t into OK/ERROR

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Stolen from https://github.com/u-blox/ubxlib/blob/master/common/spartn/src/u_spartn_crc.c
static const uint8_t u8Crc4Table[] = {
    0x00U, 0x0BU, 0x05U, 0x0EU, 0x0AU, 0x01U, 0x0FU, 0x04U,
    0x07U, 0x0CU, 0x02U, 0x09U, 0x0DU, 0x06U, 0x08U, 0x03U,
    0x0EU, 0x05U, 0x0BU, 0x00U, 0x04U, 0x0FU, 0x01U, 0x0AU,
    0x09U, 0x02U, 0x0CU, 0x07U, 0x03U, 0x08U, 0x06U, 0x0DU,
    0x0FU, 0x04U, 0x0AU, 0x01U, 0x05U, 0x0EU, 0x00U, 0x0BU,
    0x08U, 0x03U, 0x0DU, 0x06U, 0x02U, 0x09U, 0x07U, 0x0CU,
    0x01U, 0x0AU, 0x04U, 0x0FU, 0x0BU, 0x00U, 0x0EU, 0x05U,
    0x06U, 0x0DU, 0x03U, 0x08U, 0x0CU, 0x07U, 0x09U, 0x02U,
    0x0DU, 0x06U, 0x08U, 0x03U, 0x07U, 0x0CU, 0x02U, 0x09U,
    0x0AU, 0x01U, 0x0FU, 0x04U, 0x00U, 0x0BU, 0x05U, 0x0EU,
    0x03U, 0x08U, 0x06U, 0x0DU, 0x09U, 0x02U, 0x0CU, 0x07U,
    0x04U, 0x0FU, 0x01U, 0x0AU, 0x0EU, 0x05U, 0x0BU, 0x00U,
    0x02U, 0x09U, 0x07U, 0x0CU, 0x08U, 0x03U, 0x0DU, 0x06U,
    0x05U, 0x0EU, 0x00U, 0x0BU, 0x0FU, 0x04U, 0x0AU, 0x01U,
    0x0CU, 0x07U, 0x09U, 0x02U, 0x06U, 0x0DU, 0x03U, 0x08U,
    0x0BU, 0x00U, 0x0EU, 0x05U, 0x01U, 0x0AU, 0x04U, 0x0FU,
    0x09U, 0x02U, 0x0CU, 0x07U, 0x03U, 0x08U, 0x06U, 0x0DU,
    0x0EU, 0x05U, 0x0BU, 0x00U, 0x04U, 0x0FU, 0x01U, 0x0AU,
    0x07U, 0x0CU, 0x02U, 0x09U, 0x0DU, 0x06U, 0x08U, 0x03U,
    0x00U, 0x0BU, 0x05U, 0x0EU, 0x0AU, 0x01U, 0x0FU, 0x04U,
    0x06U, 0x0DU, 0x03U, 0x08U, 0x0CU, 0x07U, 0x09U, 0x02U,
    0x01U, 0x0AU, 0x04U, 0x0FU, 0x0BU, 0x00U, 0x0EU, 0x05U,
    0x08U, 0x03U, 0x0DU, 0x06U, 0x02U, 0x09U, 0x07U, 0x0CU,
    0x0FU, 0x04U, 0x0AU, 0x01U, 0x05U, 0x0EU, 0x00U, 0x0BU,
    0x04U, 0x0FU, 0x01U, 0x0AU, 0x0EU, 0x05U, 0x0BU, 0x00U,
    0x03U, 0x08U, 0x06U, 0x0DU, 0x09U, 0x02U, 0x0CU, 0x07U,
    0x0AU, 0x01U, 0x0FU, 0x04U, 0x00U, 0x0BU, 0x05U, 0x0EU,
    0x0DU, 0x06U, 0x08U, 0x03U, 0x07U, 0x0CU, 0x02U, 0x09U,
    0x0BU, 0x00U, 0x0EU, 0x05U, 0x01U, 0x0AU, 0x04U, 0x0FU,
    0x0CU, 0x07U, 0x09U, 0x02U, 0x06U, 0x0DU, 0x03U, 0x08U,
    0x05U, 0x0EU, 0x00U, 0x0BU, 0x0FU, 0x04U, 0x0AU, 0x01U,
    0x02U, 0x09U, 0x07U, 0x0CU, 0x08U, 0x03U, 0x0DU, 0x06U
};
static const uint8_t u8Crc8Table[] = {
    0x00U, 0x07U, 0x0EU, 0x09U, 0x1CU, 0x1BU, 0x12U, 0x15U,
    0x38U, 0x3FU, 0x36U, 0x31U, 0x24U, 0x23U, 0x2AU, 0x2DU,
    0x70U, 0x77U, 0x7EU, 0x79U, 0x6CU, 0x6BU, 0x62U, 0x65U,
    0x48U, 0x4FU, 0x46U, 0x41U, 0x54U, 0x53U, 0x5AU, 0x5DU,
    0xE0U, 0xE7U, 0xEEU, 0xE9U, 0xFCU, 0xFBU, 0xF2U, 0xF5U,
    0xD8U, 0xDFU, 0xD6U, 0xD1U, 0xC4U, 0xC3U, 0xCAU, 0xCDU,
    0x90U, 0x97U, 0x9EU, 0x99U, 0x8CU, 0x8BU, 0x82U, 0x85U,
    0xA8U, 0xAFU, 0xA6U, 0xA1U, 0xB4U, 0xB3U, 0xBAU, 0xBDU,
    0xC7U, 0xC0U, 0xC9U, 0xCEU, 0xDBU, 0xDCU, 0xD5U, 0xD2U,
    0xFFU, 0xF8U, 0xF1U, 0xF6U, 0xE3U, 0xE4U, 0xEDU, 0xEAU,
    0xB7U, 0xB0U, 0xB9U, 0xBEU, 0xABU, 0xACU, 0xA5U, 0xA2U,
    0x8FU, 0x88U, 0x81U, 0x86U, 0x93U, 0x94U, 0x9DU, 0x9AU,
    0x27U, 0x20U, 0x29U, 0x2EU, 0x3BU, 0x3CU, 0x35U, 0x32U,
    0x1FU, 0x18U, 0x11U, 0x16U, 0x03U, 0x04U, 0x0DU, 0x0AU,
    0x57U, 0x50U, 0x59U, 0x5EU, 0x4BU, 0x4CU, 0x45U, 0x42U,
    0x6FU, 0x68U, 0x61U, 0x66U, 0x73U, 0x74U, 0x7DU, 0x7AU,
    0x89U, 0x8EU, 0x87U, 0x80U, 0x95U, 0x92U, 0x9BU, 0x9CU,
    0xB1U, 0xB6U, 0xBFU, 0xB8U, 0xADU, 0xAAU, 0xA3U, 0xA4U,
    0xF9U, 0xFEU, 0xF7U, 0xF0U, 0xE5U, 0xE2U, 0xEBU, 0xECU,
    0xC1U, 0xC6U, 0xCFU, 0xC8U, 0xDDU, 0xDAU, 0xD3U, 0xD4U,
    0x69U, 0x6EU, 0x67U, 0x60U, 0x75U, 0x72U, 0x7BU, 0x7CU,
    0x51U, 0x56U, 0x5FU, 0x58U, 0x4DU, 0x4AU, 0x43U, 0x44U,
    0x19U, 0x1EU, 0x17U, 0x10U, 0x05U, 0x02U, 0x0BU, 0x0CU,
    0x21U, 0x26U, 0x2FU, 0x28U, 0x3DU, 0x3AU, 0x33U, 0x34U,
    0x4EU, 0x49U, 0x40U, 0x47U, 0x52U, 0x55U, 0x5CU, 0x5BU,
    0x76U, 0x71U, 0x78U, 0x7FU, 0x6AU, 0x6DU, 0x64U, 0x63U,
    0x3EU, 0x39U, 0x30U, 0x37U, 0x22U, 0x25U, 0x2CU, 0x2BU,
    0x06U, 0x01U, 0x08U, 0x0FU, 0x1AU, 0x1DU, 0x14U, 0x13U,
    0xAEU, 0xA9U, 0xA0U, 0xA7U, 0xB2U, 0xB5U, 0xBCU, 0xBBU,
    0x96U, 0x91U, 0x98U, 0x9FU, 0x8AU, 0x8DU, 0x84U, 0x83U,
    0xDEU, 0xD9U, 0xD0U, 0xD7U, 0xC2U, 0xC5U, 0xCCU, 0xCBU,
    0xE6U, 0xE1U, 0xE8U, 0xEFU, 0xFAU, 0xFDU, 0xF4U, 0xF3U
};

static const uint16_t u16Crc16Table[] = {
    0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
    0x8108U, 0x9129U, 0xA14AU, 0xB16BU, 0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
    0x1231U, 0x0210U, 0x3273U, 0x2252U, 0x52B5U, 0x4294U, 0x72F7U, 0x62D6U,
    0x9339U, 0x8318U, 0xB37BU, 0xA35AU, 0xD3BDU, 0xC39CU, 0xF3FFU, 0xE3DEU,
    0x2462U, 0x3443U, 0x0420U, 0x1401U, 0x64E6U, 0x74C7U, 0x44A4U, 0x5485U,
    0xA56AU, 0xB54BU, 0x8528U, 0x9509U, 0xE5EEU, 0xF5CFU, 0xC5ACU, 0xD58DU,
    0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76D7U, 0x66F6U, 0x5695U, 0x46B4U,
    0xB75BU, 0xA77AU, 0x9719U, 0x8738U, 0xF7DFU, 0xE7FEU, 0xD79DU, 0xC7BCU,
    0x48C4U, 0x58E5U, 0x6886U, 0x78A7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
    0xC9CCU, 0xD9EDU, 0xE98EU, 0xF9AFU, 0x8948U, 0x9969U, 0xA90AU, 0xB92BU,
    0x5AF5U, 0x4AD4U, 0x7AB7U, 0x6A96U, 0x1A71U, 0x0A50U, 0x3A33U, 0x2A12U,
    0xDBFDU, 0xCBDCU, 0xFBBFU, 0xEB9EU, 0x9B79U, 0x8B58U, 0xBB3BU, 0xAB1AU,
    0x6CA6U, 0x7C87U, 0x4CE4U, 0x5CC5U, 0x2C22U, 0x3C03U, 0x0C60U, 0x1C41U,
    0xEDAEU, 0xFD8FU, 0xCDECU, 0xDDCDU, 0xAD2AU, 0xBD0BU, 0x8D68U, 0x9D49U,
    0x7E97U, 0x6EB6U, 0x5ED5U, 0x4EF4U, 0x3E13U, 0x2E32U, 0x1E51U, 0x0E70U,
    0xFF9FU, 0xEFBEU, 0xDFDDU, 0xCFFCU, 0xBF1BU, 0xAF3AU, 0x9F59U, 0x8F78U,
    0x9188U, 0x81A9U, 0xB1CAU, 0xA1EBU, 0xD10CU, 0xC12DU, 0xF14EU, 0xE16FU,
    0x1080U, 0x00A1U, 0x30C2U, 0x20E3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
    0x83B9U, 0x9398U, 0xA3FBU, 0xB3DAU, 0xC33DU, 0xD31CU, 0xE37FU, 0xF35EU,
    0x02B1U, 0x1290U, 0x22F3U, 0x32D2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
    0xB5EAU, 0xA5CBU, 0x95A8U, 0x8589U, 0xF56EU, 0xE54FU, 0xD52CU, 0xC50DU,
    0x34E2U, 0x24C3U, 0x14A0U, 0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U,
    0xA7DBU, 0xB7FAU, 0x8799U, 0x97B8U, 0xE75FU, 0xF77EU, 0xC71DU, 0xD73CU,
    0x26D3U, 0x36F2U, 0x0691U, 0x16B0U, 0x6657U, 0x7676U, 0x4615U, 0x5634U,
    0xD94CU, 0xC96DU, 0xF90EU, 0xE92FU, 0x99C8U, 0x89E9U, 0xB98AU, 0xA9ABU,
    0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18C0U, 0x08E1U, 0x3882U, 0x28A3U,
    0xCB7DU, 0xDB5CU, 0xEB3FU, 0xFB1EU, 0x8BF9U, 0x9BD8U, 0xABBBU, 0xBB9AU,
    0x4A75U, 0x5A54U, 0x6A37U, 0x7A16U, 0x0AF1U, 0x1AD0U, 0x2AB3U, 0x3A92U,
    0xFD2EU, 0xED0FU, 0xDD6CU, 0xCD4DU, 0xBDAAU, 0xAD8BU, 0x9DE8U, 0x8DC9U,
    0x7C26U, 0x6C07U, 0x5C64U, 0x4C45U, 0x3CA2U, 0x2C83U, 0x1CE0U, 0x0CC1U,
    0xEF1FU, 0xFF3EU, 0xCF5DU, 0xDF7CU, 0xAF9BU, 0xBFBAU, 0x8FD9U, 0x9FF8U,
    0x6E17U, 0x7E36U, 0x4E55U, 0x5E74U, 0x2E93U, 0x3EB2U, 0x0ED1U, 0x1EF0U
};

static const uint32_t u32Crc24Table[] = {
    0x00000000U, 0x00864CFBU, 0x008AD50DU, 0x000C99F6U, 0x0093E6E1U, 0x0015AA1AU, 0x001933ECU, 0x009F7F17U,
    0x00A18139U, 0x0027CDC2U, 0x002B5434U, 0x00AD18CFU, 0x003267D8U, 0x00B42B23U, 0x00B8B2D5U, 0x003EFE2EU,
    0x00C54E89U, 0x00430272U, 0x004F9B84U, 0x00C9D77FU, 0x0056A868U, 0x00D0E493U, 0x00DC7D65U, 0x005A319EU,
    0x0064CFB0U, 0x00E2834BU, 0x00EE1ABDU, 0x00685646U, 0x00F72951U, 0x007165AAU, 0x007DFC5CU, 0x00FBB0A7U,
    0x000CD1E9U, 0x008A9D12U, 0x008604E4U, 0x0000481FU, 0x009F3708U, 0x00197BF3U, 0x0015E205U, 0x0093AEFEU,
    0x00AD50D0U, 0x002B1C2BU, 0x002785DDU, 0x00A1C926U, 0x003EB631U, 0x00B8FACAU, 0x00B4633CU, 0x00322FC7U,
    0x00C99F60U, 0x004FD39BU, 0x00434A6DU, 0x00C50696U, 0x005A7981U, 0x00DC357AU, 0x00D0AC8CU, 0x0056E077U,
    0x00681E59U, 0x00EE52A2U, 0x00E2CB54U, 0x006487AFU, 0x00FBF8B8U, 0x007DB443U, 0x00712DB5U, 0x00F7614EU,
    0x0019A3D2U, 0x009FEF29U, 0x009376DFU, 0x00153A24U, 0x008A4533U, 0x000C09C8U, 0x0000903EU, 0x0086DCC5U,
    0x00B822EBU, 0x003E6E10U, 0x0032F7E6U, 0x00B4BB1DU, 0x002BC40AU, 0x00AD88F1U, 0x00A11107U, 0x00275DFCU,
    0x00DCED5BU, 0x005AA1A0U, 0x00563856U, 0x00D074ADU, 0x004F0BBAU, 0x00C94741U, 0x00C5DEB7U, 0x0043924CU,
    0x007D6C62U, 0x00FB2099U, 0x00F7B96FU, 0x0071F594U, 0x00EE8A83U, 0x0068C678U, 0x00645F8EU, 0x00E21375U,
    0x0015723BU, 0x00933EC0U, 0x009FA736U, 0x0019EBCDU, 0x008694DAU, 0x0000D821U, 0x000C41D7U, 0x008A0D2CU,
    0x00B4F302U, 0x0032BFF9U, 0x003E260FU, 0x00B86AF4U, 0x002715E3U, 0x00A15918U, 0x00ADC0EEU, 0x002B8C15U,
    0x00D03CB2U, 0x00567049U, 0x005AE9BFU, 0x00DCA544U, 0x0043DA53U, 0x00C596A8U, 0x00C90F5EU, 0x004F43A5U,
    0x0071BD8BU, 0x00F7F170U, 0x00FB6886U, 0x007D247DU, 0x00E25B6AU, 0x00641791U, 0x00688E67U, 0x00EEC29CU,
    0x003347A4U, 0x00B50B5FU, 0x00B992A9U, 0x003FDE52U, 0x00A0A145U, 0x0026EDBEU, 0x002A7448U, 0x00AC38B3U,
    0x0092C69DU, 0x00148A66U, 0x00181390U, 0x009E5F6BU, 0x0001207CU, 0x00876C87U, 0x008BF571U, 0x000DB98AU,
    0x00F6092DU, 0x007045D6U, 0x007CDC20U, 0x00FA90DBU, 0x0065EFCCU, 0x00E3A337U, 0x00EF3AC1U, 0x0069763AU,
    0x00578814U, 0x00D1C4EFU, 0x00DD5D19U, 0x005B11E2U, 0x00C46EF5U, 0x0042220EU, 0x004EBBF8U, 0x00C8F703U,
    0x003F964DU, 0x00B9DAB6U, 0x00B54340U, 0x00330FBBU, 0x00AC70ACU, 0x002A3C57U, 0x0026A5A1U, 0x00A0E95AU,
    0x009E1774U, 0x00185B8FU, 0x0014C279U, 0x00928E82U, 0x000DF195U, 0x008BBD6EU, 0x00872498U, 0x00016863U,
    0x00FAD8C4U, 0x007C943FU, 0x00700DC9U, 0x00F64132U, 0x00693E25U, 0x00EF72DEU, 0x00E3EB28U, 0x0065A7D3U,
    0x005B59FDU, 0x00DD1506U, 0x00D18CF0U, 0x0057C00BU, 0x00C8BF1CU, 0x004EF3E7U, 0x00426A11U, 0x00C426EAU,
    0x002AE476U, 0x00ACA88DU, 0x00A0317BU, 0x00267D80U, 0x00B90297U, 0x003F4E6CU, 0x0033D79AU, 0x00B59B61U,
    0x008B654FU, 0x000D29B4U, 0x0001B042U, 0x0087FCB9U, 0x001883AEU, 0x009ECF55U, 0x009256A3U, 0x00141A58U,
    0x00EFAAFFU, 0x0069E604U, 0x00657FF2U, 0x00E33309U, 0x007C4C1EU, 0x00FA00E5U, 0x00F69913U, 0x0070D5E8U,
    0x004E2BC6U, 0x00C8673DU, 0x00C4FECBU, 0x0042B230U, 0x00DDCD27U, 0x005B81DCU, 0x0057182AU, 0x00D154D1U,
    0x0026359FU, 0x00A07964U, 0x00ACE092U, 0x002AAC69U, 0x00B5D37EU, 0x00339F85U, 0x003F0673U, 0x00B94A88U,
    0x0087B4A6U, 0x0001F85DU, 0x000D61ABU, 0x008B2D50U, 0x00145247U, 0x00921EBCU, 0x009E874AU, 0x0018CBB1U,
    0x00E37B16U, 0x006537EDU, 0x0069AE1BU, 0x00EFE2E0U, 0x00709DF7U, 0x00F6D10CU, 0x00FA48FAU, 0x007C0401U,
    0x0042FA2FU, 0x00C4B6D4U, 0x00C82F22U, 0x004E63D9U, 0x00D11CCEU, 0x00575035U, 0x005BC9C3U, 0x00DD8538U
};

static const uint32_t u32Crc32Table[] = {
    0x00000000U, 0x04C11DB7U, 0x09823B6EU, 0x0D4326D9U, 0x130476DCU, 0x17C56B6BU, 0x1A864DB2U, 0x1E475005U,
    0x2608EDB8U, 0x22C9F00FU, 0x2F8AD6D6U, 0x2B4BCB61U, 0x350C9B64U, 0x31CD86D3U, 0x3C8EA00AU, 0x384FBDBDU,
    0x4C11DB70U, 0x48D0C6C7U, 0x4593E01EU, 0x4152FDA9U, 0x5F15ADACU, 0x5BD4B01BU, 0x569796C2U, 0x52568B75U,
    0x6A1936C8U, 0x6ED82B7FU, 0x639B0DA6U, 0x675A1011U, 0x791D4014U, 0x7DDC5DA3U, 0x709F7B7AU, 0x745E66CDU,
    0x9823B6E0U, 0x9CE2AB57U, 0x91A18D8EU, 0x95609039U, 0x8B27C03CU, 0x8FE6DD8BU, 0x82A5FB52U, 0x8664E6E5U,
    0xBE2B5B58U, 0xBAEA46EFU, 0xB7A96036U, 0xB3687D81U, 0xAD2F2D84U, 0xA9EE3033U, 0xA4AD16EAU, 0xA06C0B5DU,
    0xD4326D90U, 0xD0F37027U, 0xDDB056FEU, 0xD9714B49U, 0xC7361B4CU, 0xC3F706FBU, 0xCEB42022U, 0xCA753D95U,
    0xF23A8028U, 0xF6FB9D9FU, 0xFBB8BB46U, 0xFF79A6F1U, 0xE13EF6F4U, 0xE5FFEB43U, 0xE8BCCD9AU, 0xEC7DD02DU,
    0x34867077U, 0x30476DC0U, 0x3D044B19U, 0x39C556AEU, 0x278206ABU, 0x23431B1CU, 0x2E003DC5U, 0x2AC12072U,
    0x128E9DCFU, 0x164F8078U, 0x1B0CA6A1U, 0x1FCDBB16U, 0x018AEB13U, 0x054BF6A4U, 0x0808D07DU, 0x0CC9CDCAU,
    0x7897AB07U, 0x7C56B6B0U, 0x71159069U, 0x75D48DDEU, 0x6B93DDDBU, 0x6F52C06CU, 0x6211E6B5U, 0x66D0FB02U,
    0x5E9F46BFU, 0x5A5E5B08U, 0x571D7DD1U, 0x53DC6066U, 0x4D9B3063U, 0x495A2DD4U, 0x44190B0DU, 0x40D816BAU,
    0xACA5C697U, 0xA864DB20U, 0xA527FDF9U, 0xA1E6E04EU, 0xBFA1B04BU, 0xBB60ADFCU, 0xB6238B25U, 0xB2E29692U,
    0x8AAD2B2FU, 0x8E6C3698U, 0x832F1041U, 0x87EE0DF6U, 0x99A95DF3U, 0x9D684044U, 0x902B669DU, 0x94EA7B2AU,
    0xE0B41DE7U, 0xE4750050U, 0xE9362689U, 0xEDF73B3EU, 0xF3B06B3BU, 0xF771768CU, 0xFA325055U, 0xFEF34DE2U,
    0xC6BCF05FU, 0xC27DEDE8U, 0xCF3ECB31U, 0xCBFFD686U, 0xD5B88683U, 0xD1799B34U, 0xDC3ABDEDU, 0xD8FBA05AU,
    0x690CE0EEU, 0x6DCDFD59U, 0x608EDB80U, 0x644FC637U, 0x7A089632U, 0x7EC98B85U, 0x738AAD5CU, 0x774BB0EBU,
    0x4F040D56U, 0x4BC510E1U, 0x46863638U, 0x42472B8FU, 0x5C007B8AU, 0x58C1663DU, 0x558240E4U, 0x51435D53U,
    0x251D3B9EU, 0x21DC2629U, 0x2C9F00F0U, 0x285E1D47U, 0x36194D42U, 0x32D850F5U, 0x3F9B762CU, 0x3B5A6B9BU,
    0x0315D626U, 0x07D4CB91U, 0x0A97ED48U, 0x0E56F0FFU, 0x1011A0FAU, 0x14D0BD4DU, 0x19939B94U, 0x1D528623U,
    0xF12F560EU, 0xF5EE4BB9U, 0xF8AD6D60U, 0xFC6C70D7U, 0xE22B20D2U, 0xE6EA3D65U, 0xEBA91BBCU, 0xEF68060BU,
    0xD727BBB6U, 0xD3E6A601U, 0xDEA580D8U, 0xDA649D6FU, 0xC423CD6AU, 0xC0E2D0DDU, 0xCDA1F604U, 0xC960EBB3U,
    0xBD3E8D7EU, 0xB9FF90C9U, 0xB4BCB610U, 0xB07DABA7U, 0xAE3AFBA2U, 0xAAFBE615U, 0xA7B8C0CCU, 0xA379DD7BU,
    0x9B3660C6U, 0x9FF77D71U, 0x92B45BA8U, 0x9675461FU, 0x8832161AU, 0x8CF30BADU, 0x81B02D74U, 0x857130C3U,
    0x5D8A9099U, 0x594B8D2EU, 0x5408ABF7U, 0x50C9B640U, 0x4E8EE645U, 0x4A4FFBF2U, 0x470CDD2BU, 0x43CDC09CU,
    0x7B827D21U, 0x7F436096U, 0x7200464FU, 0x76C15BF8U, 0x68860BFDU, 0x6C47164AU, 0x61043093U, 0x65C52D24U,
    0x119B4BE9U, 0x155A565EU, 0x18197087U, 0x1CD86D30U, 0x029F3D35U, 0x065E2082U, 0x0B1D065BU, 0x0FDC1BECU,
    0x3793A651U, 0x3352BBE6U, 0x3E119D3FU, 0x3AD08088U, 0x2497D08DU, 0x2056CD3AU, 0x2D15EBE3U, 0x29D4F654U,
    0xC5A92679U, 0xC1683BCEU, 0xCC2B1D17U, 0xC8EA00A0U, 0xD6AD50A5U, 0xD26C4D12U, 0xDF2F6BCBU, 0xDBEE767CU,
    0xE3A1CBC1U, 0xE760D676U, 0xEA23F0AFU, 0xEEE2ED18U, 0xF0A5BD1DU, 0xF464A0AAU, 0xF9278673U, 0xFDE69BC4U,
    0x89B8FD09U, 0x8D79E0BEU, 0x803AC667U, 0x84FBDBD0U, 0x9ABC8BD5U, 0x9E7D9662U, 0x933EB0BBU, 0x97FFAD0CU,
    0xAFB010B1U, 0xAB710D06U, 0xA6322BDFU, 0xA2F33668U, 0xBCB4666DU, 0xB8757BDAU, 0xB5365D03U, 0xB1F740B4U
};
uint8_t uSpartnCrc4(const uint8_t *pU8Msg, size_t size)
{
    // Initialize local variables
    uint8_t u8TableRemainder;
    uint8_t u8Remainder = 0; // Initial remainder

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (size_t x = 0; x < size; x++) {
        u8TableRemainder = pU8Msg[x] ^ u8Remainder;
        u8Remainder = u8Crc4Table[u8TableRemainder];
    }

    return u8Remainder;
}
uint8_t uSpartnCrc8(const uint8_t *pU8Msg, size_t size)
{
    // Initialize local variables
    uint8_t u8TableRemainder;
    uint8_t u8Remainder = 0; // Initial remainder

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (size_t x = 0; x < size; x++) {
        u8TableRemainder = pU8Msg[x] ^ u8Remainder;
        u8Remainder = u8Crc8Table[u8TableRemainder];
    }

    return u8Remainder;
}
uint16_t uSpartnCrc16(const uint8_t *pU8Msg, size_t size)
{
    // Initialize local variables
    uint16_t u16TableRemainder;
    uint16_t u16Remainder = 0; // Initial remainder
    uint8_t  u8NumBitsInCrc = (8 * sizeof(uint16_t));

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (size_t x = 0; x < size; x++) {
        u16TableRemainder = pU8Msg[x] ^ (u16Remainder >> (u8NumBitsInCrc - 8));
        u16Remainder = u16Crc16Table[u16TableRemainder] ^ (u16Remainder << 8);
    }

    return u16Remainder;
}
uint32_t uSpartnCrc24(const uint8_t *pU8Msg, size_t size)
{
    // Initialize local variables
    uint32_t u32TableRemainder;
    uint32_t u32Remainder = 0; // Initial remainder
    uint8_t u8NumBitsInCrc = (8 * sizeof(uint8_t) * 3);

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (size_t x = 0; x < size; x++) {
        u32TableRemainder = pU8Msg[x] ^ (u32Remainder >> (u8NumBitsInCrc - 8));
        u32Remainder = u32Crc24Table[u32TableRemainder] ^ (u32Remainder << 8);
        u32Remainder = u32Remainder & 0x00FFFFFF; // Only interested in 24 bits
    }

    return u32Remainder;
}
uint32_t uSpartnCrc32(const uint8_t *pU8Msg, size_t size)
{
    // Initialize local variables
    uint32_t u32TableRemainder;
    uint32_t u32Remainder = 0xFFFFFFFFU; // Initial remainder
    uint8_t u8NumBitsInCrc = (8 * sizeof(uint32_t));
    uint32_t u32FinalXORValue = 0xFFFFFFFFU;

    // Compute the CRC value
    // Divide each byte of the message by the corresponding polynomial
    for (size_t x = 0; x < size; x++) {
        u32TableRemainder = pU8Msg[x] ^ (u32Remainder >> (u8NumBitsInCrc - 8));
        u32Remainder = u32Crc32Table[u32TableRemainder] ^ (u32Remainder << 8);
    }

    u32Remainder = u32Remainder ^ u32FinalXORValue;

    return u32Remainder;
}

// Parse SPARTN data
uint8_t * parseSPARTN(uint8_t incoming, bool &valid, uint16_t &len)
{
  typedef enum {
    waitingFor73,
    TF002_TF006,
    TF007,
    TF009,
    TF016,
    TF017,
    TF018
  } parseStates;
  static parseStates parseState = waitingFor73;

  static uint8_t spartn[1100];

  static uint16_t frameCount;
  static uint8_t messageType;
  static uint16_t payloadLength;
  static uint16_t EAF;
  static uint8_t crcType;
  static uint16_t crcBytes;
  static uint8_t frameCRC;
  static uint8_t messageSubtype;
  static uint16_t timeTagType;
  static uint16_t authenticationIndicator;
  static uint16_t embeddedApplicationLengthBytes;
  static uint16_t TF007toTF016;

  valid = false;

  switch(parseState)
  {
    case waitingFor73:
      if (incoming == 0x73)
      {
        parseState = TF002_TF006;
        frameCount = 0;
        spartn[0] = incoming;
      }
      break;
    case TF002_TF006:
      spartn[1 + frameCount] = incoming;
      if (frameCount == 0)
      {
        messageType = incoming >> 1;
        payloadLength = incoming & 0x01;
      }
      if (frameCount == 1)
      {
        payloadLength <<= 8;
        payloadLength |= incoming;
      }
      if (frameCount == 2)
      {
        payloadLength <<= 1;
        payloadLength |= incoming >> 7;
        EAF = (incoming >> 6) & 0x01;
        crcType = (incoming >> 4) & 0x03;
        switch (crcType)
        {
          case 0:
            crcBytes = 1;
            break;
          case 1:
            crcBytes = 2;
            break;
          case 2:
            crcBytes = 3;
            break;
          default:
            crcBytes = 4;
            break;
        }
        frameCRC = incoming & 0x0F;
        spartn[3] = spartn[3] & 0xF0; // Zero the 4 LSBs before calculating the CRC
        if (uSpartnCrc4(&spartn[1], 3) == frameCRC)
        {
          spartn[3] = incoming; // Restore TF005 and TF006 now we know the data is valid
          parseState = TF007;
          //Serial.println("Header CRC is valid");
          //Serial.printf("payloadLength %d EAF %d crcType %d\n", payloadLength, EAF, crcType);
        }
        else
        {
          parseState = waitingFor73;
          //Serial.println("Header CRC is INVALID");
        }
      }
      frameCount++;
      break;
    case TF007:
      spartn[4] = incoming;
      messageSubtype = incoming >> 4;
      timeTagType = (incoming >> 3) & 0x01;
      //Serial.printf("timeTagType %d\n", timeTagType);
      if (timeTagType == 0)
        TF007toTF016 = 4;
      else
        TF007toTF016 = 6;
      if (EAF > 0)
        TF007toTF016 += 2;
      parseState = TF009;
      frameCount = 1;          
      break;
    case TF009:
      spartn[4 + frameCount] = incoming;
      frameCount++;
      if (frameCount == TF007toTF016)
      {
        if (EAF == 0)
        {
          authenticationIndicator = 0;
          embeddedApplicationLengthBytes = 0;
        }
        else
        {
          authenticationIndicator = (incoming >> 3) & 0x07;
          //Serial.printf("authenticationIndicator %d\n", authenticationIndicator);
          if (authenticationIndicator <= 1)
            embeddedApplicationLengthBytes = 0;
          else
          {
            switch(incoming & 0x07)
            {
              case 0:
                embeddedApplicationLengthBytes = 8; // 64 bits
                break;
              case 1:
                embeddedApplicationLengthBytes = 12; // 96 bits
                break;
              case 2:
                embeddedApplicationLengthBytes = 16; // 128 bits
                break;
              case 3:
                embeddedApplicationLengthBytes = 32; // 256 bits
                break;
              default:
                embeddedApplicationLengthBytes = 64; // 512 / TBD bits
                break;
            }
          }
          //Serial.printf("embeddedApplicationLengthBytes %d\n", embeddedApplicationLengthBytes);
        }
        parseState = TF016;
        frameCount = 0;                  
      }
      break;
    case TF016:
      spartn[4 + TF007toTF016 + frameCount] = incoming;
      frameCount++;
      if (frameCount == payloadLength)
      {
        if (embeddedApplicationLengthBytes > 0)
        {
          parseState = TF017;
          frameCount = 0;
        }
        else               
        {
          parseState = TF018;
          frameCount = 0;
        }
      }
      break;
    case TF017:
      spartn[4 + TF007toTF016 + payloadLength + frameCount] = incoming;
      frameCount++;
      if (frameCount == embeddedApplicationLengthBytes)
      {
        parseState = TF018;
        frameCount = 0;        
      }
      break;
    case TF018:
      spartn[4 + TF007toTF016 + payloadLength + embeddedApplicationLengthBytes + frameCount] = incoming;
      frameCount++;
      if (frameCount == crcBytes)
      {
          parseState = waitingFor73;
          uint16_t numBytes = 4 + TF007toTF016 + payloadLength + embeddedApplicationLengthBytes;
          //Serial.printf("numBytes %d\n", numBytes);
          uint8_t *ptr = &spartn[numBytes];
          switch (crcType)
          {
            case 0:
            {
              uint8_t expected = *ptr;
              if (uSpartnCrc8(&spartn[1], numBytes - 1) == expected) // Don't include the preamble in the CRC
              {
                valid = true;
                len = numBytes + 1;
                //Serial.println("SPARTN CRC-8 is valid");
              }
              else
              {
                //Serial.println("SPARTN CRC-8 is INVALID");
              }
            }
            break;
            case 1:
            {
              uint16_t expected = *ptr++;
              expected <<= 8;
              expected |= *ptr;
              if (uSpartnCrc16(&spartn[1], numBytes - 1) == expected) // Don't include the preamble in the CRC
              {
                valid = true;
                len = numBytes + 2;
                //Serial.println("SPARTN CRC-16 is valid");
              }
              else
              {
                //Serial.println("SPARTN CRC-16 is INVALID");
              }
            }
            break;
            case 2:
            {
              uint32_t expected = *ptr++;
              expected <<= 8;
              expected |= *ptr++;
              expected <<= 8;
              expected |= *ptr;
              uint32_t crc = uSpartnCrc24(&spartn[1], numBytes - 1); // Don't include the preamble in the CRC
              if (crc == expected)
              {
                valid = true;
                len = numBytes + 3;
                //Serial.println("SPARTN CRC-24 is valid");
              }
              else
              {
                //Serial.printf("SPARTN CRC-24 is INVALID: 0x%06X vs 0x%06X\n", expected, crc);
              }
            }
            break;
            default:
            {
              uint32_t expected = *ptr++;
              expected <<= 8;
              expected |= *ptr++;
              expected <<= 8;
              expected |= *ptr++;
              expected <<= 8;
              expected |= *ptr;
              if (uSpartnCrc32(&spartn[1], numBytes - 1) == expected)
              {
                valid = true;
                len = numBytes + 4;
                //Serial.println("SPARTN CRC-32 is valid");
              }
              else
              {
                //Serial.println("SPARTN CRC-32 is INVALID");
              }
            }
            break;
          }
      }
      break;
  }

  (void)messageType; // Fix perky compiler warnings as errors
  (void)messageSubtype;

  return &spartn[0];
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: pushRXMPMP will be called when new PMP data arrives
// See u-blox_structs.h for the full definition of UBX_RXM_PMP_message_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setRXMPMPmessageCallbackPtr
//        /               _____  This _must_ be UBX_RXM_PMP_message_data_t
//        |              /              _____ You can use any name you like for the struct
//        |              |             /
//        |              |             |
void pushRXMPMP(UBX_RXM_PMP_message_data_t *pmpData)
{
  //Extract the raw message payload length
  uint16_t payloadLen = ((uint16_t)pmpData->lengthMSB << 8) | (uint16_t)pmpData->lengthLSB;

  uint16_t numBytesUserData = pmpData->payload[2] | ((uint16_t)pmpData->payload[3] << 8);
  uint16_t fecBits = pmpData->payload[20] | ((uint16_t)pmpData->payload[21] << 8);
  float ebno = (float)pmpData->payload[22] / 8;
  
  Serial.print(F("New RXM-PMP data received. userData: "));
  Serial.print(numBytesUserData);
  Serial.print(F(" Bytes. fecBits: "));
  Serial.print(fecBits);
  Serial.print(F(". ebno (dB): "));
  Serial.print(ebno);
  Serial.println(F("."));

  //Parse the SPARTN data stream contained in the userData
  for (uint16_t i = 0; i < numBytesUserData; i++)
  {
    bool valid = false;
    uint16_t len;
    uint8_t *spartn = parseSPARTN(pmpData->payload[24 + i], valid, len);

    if (valid)
    {
      Serial.print(F("Valid SPARTN data parsed. Pushing "));
      Serial.print(len);
      Serial.println(F(" bytes to Serial..."));
      
      serialGNSS.write(spartn, len); // Push the SPARTN data to the module using Serial
    }
  }

  (void)payloadLen;
  //serialGNSS.write(&pmpData->sync1, (size_t)payloadLen + 6); // Push the sync chars, class, ID, length and payload
  //serialGNSS.write(&pmpData->checksumA, (size_t)2); // Push the checksum bytes

}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println(F("NEO-D9S SPARTN Corrections"));

  serialGNSS.begin(38400); // UART2 on pins 16/17.
                                           
  Wire.begin(); //Start I2C

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin and configure the NEO-D9S L-Band receiver

  //myLBand.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myLBand.begin(Wire, 0x43) == false) //Connect to the u-blox NEO-D9S using Wire port. The D9S default I2C address is 0x43 (not 0x42)
  {
    Serial.println(F("u-blox NEO-D9S not detected at default I2C address. Please check wiring."));
    delay(2000);
  }
  Serial.println(F("u-blox NEO-D9S connected"));

  myLBand.newCfgValset(); // Create a new Configuration Interface message - this defaults to VAL_LAYER_RAM_BBR (change in RAM and BBR)
  myLBand.addCfgValset(UBLOX_CFG_PMP_CENTER_FREQUENCY,     myLBandFreq); // Default 1539812500 Hz
  myLBand.addCfgValset(UBLOX_CFG_PMP_SEARCH_WINDOW,        2200);        // Default 2200 Hz
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_SERVICE_ID,       0);           // Default 1 
  myLBand.addCfgValset(UBLOX_CFG_PMP_SERVICE_ID,           21845);       // Default 50821
  myLBand.addCfgValset(UBLOX_CFG_PMP_DATA_RATE,            2400);        // Default 2400 bps
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_DESCRAMBLER,      1);           // Default 1
  myLBand.addCfgValset(UBLOX_CFG_PMP_DESCRAMBLER_INIT,     26969);       // Default 23560
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_PRESCRAMBLING,    0);           // Default 0
  myLBand.addCfgValset(UBLOX_CFG_PMP_UNIQUE_WORD,          16238547128276412563ull); 
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C,   1);           // Ensure UBX-RXM-PMP is enabled on the I2C port 
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART1, 1);           // Output UBX-RXM-PMP on UART1
  myLBand.addCfgValset(UBLOX_CFG_UART2OUTPROT_UBX,         1);           // Enable UBX output on UART2
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART2, 1);           // Output UBX-RXM-PMP on UART2
  myLBand.addCfgValset(UBLOX_CFG_UART1_BAUDRATE,           38400);       // match baudrate with ZED default
  myLBand.addCfgValset(UBLOX_CFG_UART2_BAUDRATE,           38400);       // match baudrate with ZED default
  bool ok = myLBand.sendCfgValset(); // Apply the settings
  
  Serial.print(F("L-Band: configuration "));
  Serial.println(OK(ok));

  myLBand.softwareResetGNSSOnly(); // Do a restart

  myLBand.setRXMPMPmessageCallbackPtr(&pushRXMPMP); // Call pushRXMPMP when new PMP data arrives. Push it to the GNSS

}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  myLBand.checkUblox(); // Check for the arrival of new PMP data and process it.
  myLBand.checkCallbacks(); // Check if any LBand callbacks are waiting to be processed.
}
