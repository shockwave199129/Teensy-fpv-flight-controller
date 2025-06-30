#include "calibration_storage.h"
#include <EEPROM.h>

namespace {
    const uint16_t MAGIC = 0xC0DE;
    struct StoredCal {
        uint16_t magic;
        CalibrationData data;
        uint32_t crc;
    };

    uint32_t crc32(const uint8_t* buf, size_t len) {
        uint32_t crc = 0xFFFFFFFF;
        for(size_t i=0;i<len;i++){
            uint8_t b=buf[i];
            crc ^= b;
            for(int j=0;j<8;j++) crc = (crc & 1)? (crc>>1)^0xEDB88320 : (crc>>1);
        }
        return ~crc;
    }
}

namespace CalibrationStorage {

bool save(const CalibrationData& data) {
    StoredCal st;
    st.magic = MAGIC;
    st.data = data;
    st.crc = crc32((uint8_t*)&data, sizeof(CalibrationData));

    const uint8_t* p = (const uint8_t*)&st;
    for(size_t i=0;i<sizeof(StoredCal);i++) EEPROM.write(i, p[i]);
    // Note: Some platforms (e.g. ESP) require EEPROM.commit(), Teensy does not.
    return true;
}

bool load(CalibrationData& data) {
    StoredCal st;
    uint8_t* p = (uint8_t*)&st;
    for(size_t i=0;i<sizeof(StoredCal);i++) p[i]=EEPROM.read(i);
    if(st.magic!=MAGIC) return false;
    uint32_t calc = crc32((uint8_t*)&st.data, sizeof(CalibrationData));
    if(calc!=st.crc) return false;
    data = st.data;
    return true;
}

} 