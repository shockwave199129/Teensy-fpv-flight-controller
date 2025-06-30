#include "esc_dma.h"
#include <DMAChannel.h>
#include "hal_timer_pwm.h"

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#ifndef DMAMUX_SOURCE_FLEXPWM3_WRITE0
#define DMAMUX_SOURCE_FLEXPWM3_WRITE0 20  // placeholder source; update per MCU datasheet
#endif

namespace {
// Timing lookup ns per bit
constexpr uint16_t bitTimeNS(EscProtocol p) {
  switch(p) {
    case ESC_PROTOCOL_DSHOT150: return 6667; // 150kHz
    case ESC_PROTOCOL_DSHOT300: return 3333;
    case ESC_PROTOCOL_DSHOT600: return 1667;
    case ESC_PROTOCOL_DSHOT1200: return 833;
    default: return 1667;
  }
}

constexpr uint8_t FLEXPWM_MODULE = 3; // use FlexPWM3
DMAChannel dmaTx;
EscProtocol currentProto = ESC_PROTOCOL_DSHOT600;
uint8_t motorCnt = 4;
static uint16_t pulseBuffer[4][16]; // one 16-bit pattern per motor
static uint8_t pins[4];

// ------------------- Telemetry capture structures -------------------
static volatile uint32_t lastEdgeMicros[4] = {0};
static volatile uint16_t pulseWidths[4][20]; // capture up to 20 pulses per packet
static volatile uint8_t pulseCount[4] = {0};
static volatile bool packetReady[4] = {false};

static bool telemetry_enabled = false;  // set by requestTelemetry()

static void IRAM_ATTR handleEdge(uint8_t motorIdx) {
  uint32_t now = micros();
  uint32_t diff = now - lastEdgeMicros[motorIdx];
  lastEdgeMicros[motorIdx] = now;
  if (pulseCount[motorIdx] < 20) {
    pulseWidths[motorIdx][pulseCount[motorIdx]] = (uint16_t)diff;
    pulseCount[motorIdx]++;
    if (pulseCount[motorIdx] >= 20) {
      packetReady[motorIdx] = true;
    }
  }
}

// Wrapper ISR functions for attachInterrupt (Teensy requires function pointers)
void IRAM_ATTR handleEdge0() { handleEdge(0); }
void IRAM_ATTR handleEdge1() { handleEdge(1); }
void IRAM_ATTR handleEdge2() { handleEdge(2); }
void IRAM_ATTR handleEdge3() { handleEdge(3); }
}

namespace ESC_DMA {

void init(EscProtocol proto, const uint8_t* motorPins, uint8_t motorCount) {
  currentProto = proto;
  motorCnt = motorCount;
  for(uint8_t i=0;i<motorCnt;i++){pins[i]=motorPins[i];}
  // Configure FlexPWM submodules – minimal config (details omitted for brevity)
  // TODO: set clock, prescaler, MOD value according to bitTimeNS(proto)

  // Configure DMA channel to write waveform per frame – here we only set up shell.
  dmaTx.disable();
  dmaTx.sourceBuffer(reinterpret_cast<volatile const uint16_t*>(&pulseBuffer[0][0]), sizeof(pulseBuffer));
  dmaTx.triggerAtHardwareEvent(DMAMUX_SOURCE_FLEXPWM3_WRITE0); // example trigger
  dmaTx.disableOnCompletion();

  // Setup edge interrupts for telemetry capture (bidirectional DShot)
  attachInterrupt(digitalPinToInterrupt(pins[0]), handleEdge0, CHANGE);
  if(motorCount>1) attachInterrupt(digitalPinToInterrupt(pins[1]), handleEdge1, CHANGE);
  if(motorCount>2) attachInterrupt(digitalPinToInterrupt(pins[2]), handleEdge2, CHANGE);
  if(motorCount>3) attachInterrupt(digitalPinToInterrupt(pins[3]), handleEdge3, CHANGE);
}

// Helper to prepare 16-bit DShot packet -> 16 PWM duty words (hi/lo)
static void encodePacket(uint16_t packet, uint16_t* dest, uint16_t hiTicks, uint16_t loTicks){
  for(int i=0;i<16;i++){
    bool bit = packet & (1<<(15-i));
    dest[i] = bit?hiTicks:loTicks;
  }
}

void sendThrottle(const uint16_t* values, bool requestTelemetry){
  // compute hi/lo ticks
  uint16_t modTicks = HalTimerPWM::dshot_mod_ticks(currentProto);
  uint16_t hiTicks0 = HalTimerPWM::dshot_hi_ticks0(currentProto); // high portion for bit0
  uint16_t hiTicks1 = HalTimerPWM::dshot_hi_ticks1(currentProto); // high portion for bit1
  uint16_t loTicks0 = modTicks - hiTicks0;
  uint16_t loTicks1 = modTicks - hiTicks1;  // low portion for bit1 (unused but kept)
  (void)loTicks1;
  for(uint8_t m=0;m<motorCnt;m++){
    uint16_t p = values[m]<<1;
    bool telemBit = telemetry_enabled && requestTelemetry;
    if(telemBit) p|=1;
    // checksum
    uint16_t csum=0,csum_data=p;
    for(int j=0;j<3;j++){csum^=csum_data; csum_data>>=4;} csum&=0x0F;
    p=(p<<4)|csum;
    // hiTicks for bit=1, loTicks for bit=0
    encodePacket(p,pulseBuffer[m],hiTicks1,loTicks0);
  }
  dmaTx.enable();
}

void sendCommand(uint8_t motor, uint16_t command){
  uint16_t val = (command|0x700)<<1; // 11-bit cmd
  uint16_t csum=0,csum_data=val; for(int j=0;j<3;j++){csum^=csum_data; csum_data>>=4;} csum&=0x0F;
  uint16_t packet = (val<<4)|csum;
  uint16_t hiTicks1 = HalTimerPWM::dshot_hi_ticks1(currentProto);
  uint16_t hiTicks0 = HalTimerPWM::dshot_hi_ticks0(currentProto);
  encodePacket(packet, pulseBuffer[motor], hiTicks1, hiTicks0);
  dmaTx.enable();
}

void requestTelemetry(bool enable){
  telemetry_enabled = enable;
}

bool getTelemetry(uint8_t motorIndex, uint16_t& eRPM, uint16_t& voltage, uint8_t& temperature, uint16_t &current){
  if(motorIndex>=motorCnt) return false;
  if(!packetReady[motorIndex]) return false;

  // default values
  eRPM = 0;
  voltage = 0;
  temperature = 0;
  current = 0;
  noInterrupts();
  uint16_t widths[20];
  for(int i=0;i<20;i++) widths[i]=pulseWidths[motorIndex][i];
  pulseCount[motorIndex]=0;
  packetReady[motorIndex]=false;
  interrupts();

  // Determine threshold
  uint16_t minW=0xFFFF,maxW=0;
  for(int i=0;i<20;i++){ if(widths[i]<minW) minW=widths[i]; if(widths[i]>maxW) maxW=widths[i]; }
  uint16_t thr = (minW+maxW)/2;

  uint16_t packet=0;
  int bitIdx=0;
  for(int i=0;i<20 && bitIdx<16;i++){
    bool bit = widths[i] > thr;
    packet = (packet<<1) | (bit?1:0);
    bitIdx++;
  }
  // Simple validation: compute CRC4 (same as checksum) and compare
  uint8_t crc_calc=0, tmp=(packet>>0);
  for(int i=0;i<3;i++){crc_calc ^= tmp; tmp >>=4;}
  crc_calc &= 0x0F;
  uint8_t crc_recv = packet & 0x0F;
  if(crc_calc!=crc_recv){ return false; }

  uint16_t data = packet >> 4; // 12 bits payload

  // Detect Extended Telemetry pattern: exponent LSB ==1 and MSB of mantissa ==0
  uint8_t exponent = (data >> 9) & 0x07;
  uint8_t mantissaMSB = (data >> 8) & 0x01;
  bool isExtended = ((exponent & 0x01) && (mantissaMSB == 0));

  if(isExtended){
    uint8_t type = (data >> 8) & 0x0F; // upper 4 bits
    uint8_t value8 = data & 0xFF;      // lower 8 bits
    switch(type){
      case 0x02: // Temperature °C
        temperature = value8;
        break;
      case 0x04: // Voltage 0.25V per LSB -> convert to mV
        voltage = (uint16_t)(value8 * 250);
        break;
      case 0x06: // Current Ampere
        current = (uint16_t)(value8 * 1000); // mA
        break;
      default:
        break;
    }
    // For extended frames eRPM not updated.
    return true;
  }

  // For standard eRPM telemetry: eRPM = data*100 per spec
  eRPM = data*100;
  return true;
}

bool getRawPacket(uint8_t motorIndex, uint16_t &payload) {
  if (motorIndex >= motorCnt) return false;
  if (!packetReady[motorIndex]) return false;

  noInterrupts();
  uint16_t widths[20];
  for (int i = 0; i < 20; i++) widths[i] = pulseWidths[motorIndex][i];
  pulseCount[motorIndex] = 0;
  packetReady[motorIndex] = false;
  interrupts();

  // Determine threshold between high and low pulses
  uint16_t minW = 0xFFFF, maxW = 0;
  for (int i = 0; i < 20; i++) {
    if (widths[i] < minW) minW = widths[i];
    if (widths[i] > maxW) maxW = widths[i];
  }
  uint16_t thr = (minW + maxW) / 2;

  uint16_t packet = 0;
  int bitIdx = 0;
  for (int i = 0; i < 20 && bitIdx < 16; i++) {
    bool bit = widths[i] > thr;
    packet = (packet << 1) | (bit ? 1 : 0);
    bitIdx++;
  }

  // Verify CRC4 (last 4 bits)
  uint8_t crc_calc = 0, tmp = (packet >> 0);
  for (int i = 0; i < 3; i++) {
    crc_calc ^= tmp;
    tmp >>= 4;
  }
  crc_calc &= 0x0F;
  uint8_t crc_recv = packet & 0x0F;
  if (crc_calc != crc_recv) {
    return false;
  }

  payload = packet >> 4; // 12-bit payload
  return true;
}

} // namespace ESC_DMA 