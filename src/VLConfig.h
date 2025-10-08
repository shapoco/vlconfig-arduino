#ifndef VLCONFIG_H
#define VLCONFIG_H

#ifndef VLCFG_VLCONFIG_HPP
#define VLCFG_VLCONFIG_HPP

// #include "vlcfg/receiver.hpp"

#ifndef VLCFG_RECEIVER_HPP
#define VLCFG_RECEIVER_HPP

// #include "vlcfg/common.hpp"

#ifndef VLCFG_COMMON_HPP
#define VLCFG_COMMON_HPP

#include <stdint.h>

#ifndef VLBS_RX_BAUDRATE
#define VLBS_RX_BAUDRATE (10)
#endif

#ifndef VLBS_RX_SAMPLES_PER_BIT
#define VLBS_RX_SAMPLES_PER_BIT (10)
#endif

#ifdef VLCFG_DEBUG

// Arduino の場合
#ifdef ARDUINO

#define VLCFG_PRINTF_BUFF_SIZE (256)

#include <Arduino.h>
int vlcfg_printf(const char* fmt, ...);

#ifdef VLCFG_IMPLEMENTATION
int vlcfg_printf(const char* fmt, ...) {
  int len = 0;
  char buf[VLCFG_PRINTF_BUFF_SIZE];

  va_list arg_ptr;
  va_start(arg_ptr, fmt);
  len = vsnprintf(buf, VLCFG_PRINTF_BUFF_SIZE, fmt, arg_ptr);
  va_end(arg_ptr);

  // output to the serial console through the 'Serial'
  len = Serial.write((uint8_t*)buf, (size_t)len);

  return len;
}
#endif

#define VLCFG_PRINTF(fmt, ...)                                \
  do {                                                        \
    vlcfg_printf("[VLCFG:%d] " fmt, __LINE__, ##__VA_ARGS__); \
  } while (0)

#else

#include <stdio.h>
#ifdef __FILE_NAME__
#define VLCFG_PRINTF(fmt, ...)                                      \
  do {                                                              \
    printf("[%s:%d] " fmt, __FILE_NAME__, __LINE__, ##__VA_ARGS__); \
  } while (0)
#else
#define VLCFG_PRINTF(fmt, ...)             \
  do {                                     \
    printf("[VLCFG] " fmt, ##__VA_ARGS__); \
  } while (0)
#endif

#endif

#else

#define VLCFG_PRINTF(fmt, ...) \
  do {                         \
  } while (0)
#endif

#define VLCFG_THROW(x)                                                   \
  do {                                                                   \
    Result __throw_ret = (x);                                            \
    VLCFG_PRINTF("Error: %s (Code=%d)\n", result_to_string(__throw_ret), \
                 (int)__throw_ret);                                      \
    return __throw_ret;                                                  \
  } while (0)

#define VLCFG_TRY(x)                    \
  do {                                  \
    Result __try_ret = (x);             \
    if (__try_ret != Result::SUCCESS) { \
      VLCFG_THROW(__try_ret);           \
    }                                   \
  } while (0)

namespace vlcfg {

static constexpr uint8_t SYMBOL_BITS = 5;

static constexpr uint32_t RX_BIT_PERIOD_US = 1000000 / VLBS_RX_BAUDRATE;
static constexpr uint32_t RX_SAMPLE_PERIOD_US =
    RX_BIT_PERIOD_US / VLBS_RX_SAMPLES_PER_BIT;

static constexpr uint8_t MAX_ENTRY_COUNT = 32;
static constexpr uint8_t MAX_KEY_LEN = 16;

static constexpr int16_t SYMBOL_CONTROL = -1;
static constexpr int16_t SYMBOL_SYNC = -2;
static constexpr int16_t SYMBOL_SOF = -3;
static constexpr int16_t SYMBOL_EOF = -4;
static constexpr int16_t SYMBOL_INVALID = -16;

enum class PcsState : uint8_t {
  LOS,
  RXED_SYNC1,
  RXED_SYNC2,
  RXED_SOF,
  RXED_BYTE,
  RXED_EOF,
};

struct CdrOutput {
  bool signal_detected;
  bool rxed;
  bool rx_bit;
};

struct PcsOutput {
  PcsState state;
  bool rxed;
  int16_t rx_byte;
};

enum class Result : uint8_t {
  SUCCESS,
  ERR_NULL_POINTER,
  ERR_OVERFLOW,
  ERR_TOO_MANY_ENTRIES,
  ERR_KEY_TYPE_MISMATCH,
  ERR_KEY_TOO_LONG,
  ERR_KEY_NOT_FOUND,
  ERR_VALUE_TYPE_MISMATCH,
  ERR_VALUE_TOO_LONG,
  ERR_LOS,
  ERR_EOF_EXPECTED,
  ERR_UNEXPECTED_EOF,
  ERR_EXTRA_BYTES,
  ERR_BAD_SHORT_COUNT,
  ERR_UNSUPPORTED_TYPE,
  ERR_BAD_CRC,
};

enum class ValueType : int8_t {
  // UINT = 0,
  // INT = 1,
  BYTE_STR = 2,
  TEXT_STR = 3,
  // ARRAY = 4,
  MAP = 5,
  // TAG = 6,
  // SIMPLE_FLOAT = 7,
  BOOLEAN = 8,
  NONE = -1,
};

enum ConfigEntryFlags : uint8_t {
  ENTRY_RECEIVED = 0x01,
};

struct ConfigEntry {
  const char* key;
  void* buffer;
  ValueType type;
  uint8_t capacity;
  uint8_t received;
  uint8_t flags;

  inline bool was_received() const { return (flags & ENTRY_RECEIVED) != 0; }
};

const char* result_to_string(Result res);
int16_t find_key(const ConfigEntry* entries, const char* key);
ConfigEntry* entry_from_key(ConfigEntry* entries, const char* key);
uint32_t crc32(const uint8_t* data, uint16_t length);
uint16_t median3(uint16_t a, uint16_t b, uint16_t c);

#ifdef VLCFG_IMPLEMENTATION

const char* result_to_string(Result res) {
  switch (res) {
    case Result::SUCCESS: return "SUCCESS";
    case Result::ERR_NULL_POINTER: return "ERR_NULL_POINTER";
    case Result::ERR_OVERFLOW: return "ERR_OVERFLOW";
    case Result::ERR_TOO_MANY_ENTRIES: return "ERR_TOO_MANY_ENTRIES";
    case Result::ERR_KEY_TYPE_MISMATCH: return "ERR_KEY_TYPE_MISMATCH";
    case Result::ERR_KEY_TOO_LONG: return "ERR_KEY_TOO_LONG";
    case Result::ERR_KEY_NOT_FOUND: return "ERR_KEY_NOT_FOUND";
    case Result::ERR_VALUE_TYPE_MISMATCH: return "ERR_VALUE_TYPE_MISMATCH";
    case Result::ERR_VALUE_TOO_LONG: return "ERR_VALUE_TOO_LONG";
    case Result::ERR_LOS: return "ERR_LOS";
    case Result::ERR_EOF_EXPECTED: return "ERR_EOF_EXPECTED";
    case Result::ERR_UNEXPECTED_EOF: return "ERR_UNEXPECTED_EOF";
    case Result::ERR_EXTRA_BYTES: return "ERR_EXTRA_BYTES";
    case Result::ERR_BAD_SHORT_COUNT: return "ERR_BAD_SHORT_COUNT";
    case Result::ERR_UNSUPPORTED_TYPE: return "ERR_UNSUPPORTED_TYPE";
    case Result::ERR_BAD_CRC: return "ERR_BAD_CRC";
    default: return "(Unknown Error)";
  }
}

int16_t find_key(const ConfigEntry* entries, const char* key) {
  if (entries == nullptr || key == nullptr) return -1;

  int16_t entry_index = -1;
  for (uint8_t i = 0; i < MAX_ENTRY_COUNT; i++) {
    const ConfigEntry& entry = entries[i];
    if (entry.key == nullptr) {
      return -1;
    }
    for (uint8_t j = 0; j < MAX_KEY_LEN + 1; j++) {
      if (entry.key[j] != key[j]) break;
      if (key[j] == '\0') return i;
    }
  }
  return -1;
}

ConfigEntry* entry_from_key(ConfigEntry* entries, const char* key) {
  int16_t index = find_key(entries, key);
  if (index < 0) return nullptr;
  return &entries[index];
}

uint32_t crc32(const uint8_t* data, uint16_t length) {
  uint32_t crc = 0xffffffff;
  for (uint32_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    crc ^= byte;
    for (uint8_t j = 0; j < 8; j++) {
      uint32_t mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xedb88320 & mask);
    }
  }
  return ~crc;
}

uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) {
    if (b > c) {
      return b;
    } else if (a > c) {
      return c;
    } else {
      return a;
    }
  } else {
    if (a > c) {
      return a;
    } else if (b > c) {
      return c;
    } else {
      return b;
    }
  }
}

#endif

}  // namespace vlcfg

#endif
// #include "vlcfg/rx_cdr.hpp"

#ifndef VLCFG_RX_CDR_HPP
#define VLCFG_RX_CDR_HPP

// #include "vlcfg/common.hpp"


namespace vlcfg {

static const uint16_t PHASE_PERIOD = VLBS_RX_SAMPLES_PER_BIT;
static const uint16_t ADC_AVE_PERIOD = PHASE_PERIOD * SYMBOL_BITS * 2;
static const uint8_t ADC_BITS = 12;

class RxCdr {
 private:
  uint8_t amp_det_count;
  bool amp_det;
  uint16_t sig_det_count;
  bool sig_det;
  uint16_t peak_max;
  uint16_t peak_min;
  uint16_t threshold;
  uint8_t last_digital_level;
  uint8_t phase;
  uint8_t sample_phase;
  uint8_t edge_level[PHASE_PERIOD];

 public:
  inline RxCdr() { init(); }
  void init();
  Result update(uint16_t adc_val, CdrOutput* out);
  inline bool signal_detected() const { return sig_det; }
};

#ifdef VLCFG_IMPLEMENTATION
static uint16_t u16log2(uint16_t x);

static const uint16_t U16LOG2_TABLE[17] = {
    0,   22,  44,  63,  82,  100, 118, 134, 150,
    165, 179, 193, 207, 220, 232, 244, 256,
};

void RxCdr::init() {
  sig_det_count = 0;
  amp_det = false;
  sig_det = false;
  threshold = 2048;
  last_digital_level = false;
  phase = 0;
  sample_phase = PHASE_PERIOD * 3 / 4;
  peak_min = 9999;
  peak_max = 0;
  VLCFG_PRINTF("RX CDR initialized.\n");
}

// clock data recovery
Result RxCdr::update(uint16_t adc_val, CdrOutput* out) {
  out->rxed = false;

  if (out == nullptr) {
    VLCFG_THROW(Result::ERR_NULL_POINTER);
  }

  // amplitude detection
  if (amp_det_count < ADC_AVE_PERIOD) {
    amp_det_count++;
    if (adc_val > peak_max) peak_max = adc_val;
    if (adc_val < peak_min) peak_min = adc_val;
  } else {
    amp_det_count = 0;
    amp_det = (peak_max - peak_min) >= (1 << (ADC_BITS - 7));
    threshold = u16log2((peak_max + peak_min) / 2);
    peak_max = adc_val;
    peak_min = adc_val;
  }

  bool los = !amp_det;

  adc_val = u16log2(adc_val);

  // level/edge detection
  int16_t hysteresis = (last_digital_level != 0) ? 0x100 : -0x100;
  bool digital_level = adc_val + hysteresis >= threshold;
  bool edge = (digital_level != last_digital_level);
  last_digital_level = digital_level;

  // edge logging
  if (edge) {
    if (edge_level[phase] < PHASE_PERIOD * 2) {
      edge_level[phase] += PHASE_PERIOD;
    }
  } else {
    if (edge_level[phase] > 0) {
      edge_level[phase]--;
    }
  }

  // data phase detection
  if (edge) {
    uint8_t edge_max_level = 0;
    int edge_max_phase = 0;
    for (int i = 0; i < PHASE_PERIOD; i++) {
      if (edge_level[i] > edge_max_level) {
        edge_max_level = edge_level[i];
        edge_max_phase = i;
      }
    }
    auto last_phase = sample_phase;
    sample_phase = edge_max_phase + PHASE_PERIOD / 2;
    if (sample_phase >= PHASE_PERIOD) {
      sample_phase -= PHASE_PERIOD;
    }

#if 0
    // ディスプレイの点滅のジッタがかなり大きいので
    // 位相変化のチェックはしない
    int8_t phase_diff = sample_phase - last_phase;
    if (phase_diff < -PHASE_PERIOD / 2) {
      phase_diff += PHASE_PERIOD;
    } else if (phase_diff > PHASE_PERIOD / 2) {
      phase_diff -= PHASE_PERIOD;
    }
    constexpr int8_t TOL = PHASE_PERIOD / 4;
    los |= (phase_diff < -TOL || TOL < phase_diff);
#endif
  }

  // signal detection
  if (los) {
    sig_det_count = 0;
    sig_det = false;
  } else if (sig_det_count < PHASE_PERIOD * 4) {
    sig_det_count++;
    sig_det = false;
  } else {
    sig_det = true;
  }
  out->signal_detected = sig_det;

  // data recovery
  if (sig_det && phase == sample_phase) {
    const uint8_t tol = (PHASE_PERIOD + 4) / 5;
    const uint8_t mn = PHASE_PERIOD - tol;
    const uint8_t mx = PHASE_PERIOD + tol;
    out->rxed = true;
    out->rx_bit = digital_level;
  }

  // step CDR phase
  if (phase < PHASE_PERIOD - 1) {
    phase++;
  } else {
    phase = 0;
  }

  return Result::SUCCESS;
}

static uint16_t u16log2(uint16_t x) {
  if (x == 0) return 0;

  uint16_t ret = 0xc000;
  if (x & 0xf000) {
    if (x & 0xc000) {
      x >>= 2;
      ret += 0x2000;
    }
    if (x & 0x2000) {
      x >>= 1;
      ret += 0x1000;
    }
  } else {
    if (!(x & 0xffc0)) {
      x <<= 6;
      ret -= 0x6000;
    }
    if (!(x & 0xfe00)) {
      x <<= 3;
      ret -= 0x3000;
    }
    if (!(x & 0xf800)) {
      x <<= 2;
      ret -= 0x2000;
    }
    if (!(x & 0xf000)) {
      x <<= 1;
      ret -= 0x1000;
    }
  }

  int index = (x >> 8) & 0xf;
  uint16_t a = U16LOG2_TABLE[index];
  uint16_t b = U16LOG2_TABLE[index + 1];
  uint16_t q = x & 0xff;
  uint16_t p = 256 - q;
  ret += (a * p + b * q) >> 4;

  return ret;
}

#endif

}  // namespace vlcfg

#endif// #include "vlcfg/rx_decoder.hpp"

#ifndef VLCFG_RX_DECODER_HPP
#define VLCFG_RX_DECODER_HPP

// #include "vlcfg/common.hpp"

// #include "vlcfg/rx_buff.hpp"

#ifndef VLCFG_RX_BUFF_HPP
#define VLCFG_RX_BUFF_HPP

// #include "vlcfg/common.hpp"


namespace vlcfg {

class RxBuff {
 public:
  const uint16_t capacity;
  uint8_t *buff;
  uint16_t write_pos = 0;
  uint16_t read_pos = 0;

  inline RxBuff(int capacity)
      : capacity(capacity), buff(new uint8_t[capacity]) {}
  inline ~RxBuff() { delete[] buff; }

  inline void init() {
    write_pos = 0;
    read_pos = 0;
  }

  inline uint16_t size() const { return write_pos - read_pos; }

  inline const uint8_t &operator[](uint16_t index) const { return buff[index]; }

  inline int peek(int offset) {
    if (read_pos + offset >= write_pos) {
      return -1;
    }
    return buff[read_pos + offset];
  }

  inline Result push(uint8_t b) {
    if (write_pos >= capacity) {
      VLCFG_THROW(Result::ERR_OVERFLOW);
    }
    buff[write_pos++] = b;
    return Result::SUCCESS;
  }

  inline Result popBytes(uint8_t *out, uint16_t len) {
    if (size() < len) {
      VLCFG_THROW(Result::ERR_UNEXPECTED_EOF);
    }
    if (out == nullptr) {
      VLCFG_THROW(Result::ERR_NULL_POINTER);
    }
    for (uint16_t i = 0; i < len; i++) {
      out[i] = buff[read_pos++];
    }
    return Result::SUCCESS;
  }

  inline Result skip(uint16_t len) {
    if (size() < len) {
      VLCFG_THROW(Result::ERR_UNEXPECTED_EOF);
    }
    read_pos += len;
    return Result::SUCCESS;
  }

  inline Result popU8(uint8_t *out) { return popBytes(out, 1); }
  inline Result popU16(uint16_t *out) {
    uint8_t buf[2];
    VLCFG_TRY(popBytes(buf, sizeof(buf)));
    *out = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    return Result::SUCCESS;
  }
  inline Result popU32(uint32_t *out) {
    uint8_t buf[4];
    VLCFG_TRY(popBytes(buf, sizeof(buf)));
    *out = (static_cast<uint32_t>(buf[0]) << 24) |
           (static_cast<uint32_t>(buf[1]) << 16) |
           (static_cast<uint32_t>(buf[2]) << 8) | buf[3];
    return Result::SUCCESS;
  }

  Result read_item_header_u32(ValueType *value_type, uint32_t *param);
  Result check_and_remove_crc();
};

#ifdef VLCFG_IMPLEMENTATION

Result RxBuff::read_item_header_u32(ValueType *value_type, uint32_t *param) {
  uint8_t ib;
  VLCFG_TRY(popU8(&ib));

  uint8_t mt = ib >> 5;
  uint8_t short_count = (ib & 0x1f);

  if (mt < 7) {
    *value_type = static_cast<ValueType>(mt);

    if (short_count <= 23) {
      *param = short_count;
    } else if (short_count == 24) {
      uint8_t tmp;
      VLCFG_TRY(popU8(&tmp));
      *param = tmp;
    } else if (short_count == 25) {
      uint16_t tmp;
      VLCFG_TRY(popU16(&tmp));
      *param = tmp;
    } else if (short_count == 26) {
      uint32_t tmp;
      VLCFG_TRY(popU32(&tmp));
      *param = tmp;
    } else {
      VLCFG_THROW(Result::ERR_BAD_SHORT_COUNT);
    }
  } else if (short_count == 20 || short_count == 21) {
    *value_type = ValueType::BOOLEAN;
    *param = short_count - 20;
    return Result::SUCCESS;
  } else {
    VLCFG_THROW(Result::ERR_UNSUPPORTED_TYPE);
  }

  return Result::SUCCESS;
}

Result RxBuff::check_and_remove_crc() {
  if (write_pos < 4) {
    VLCFG_THROW(Result::ERR_UNEXPECTED_EOF);
  }
  write_pos -= 4;

  uint32_t calcedCrc = crc32(buff, write_pos);
  uint32_t recvCrc = static_cast<uint32_t>(buff[write_pos]) << 24 |
                     static_cast<uint32_t>(buff[write_pos + 1]) << 16 |
                     static_cast<uint32_t>(buff[write_pos + 2]) << 8 |
                     static_cast<uint32_t>(buff[write_pos + 3]);
  if (calcedCrc != recvCrc) VLCFG_THROW(Result::ERR_BAD_CRC);
  VLCFG_PRINTF("CRC OK: 0x%08X\n", calcedCrc);
  return Result::SUCCESS;
}

#endif

}  // namespace vlcfg

#endif

namespace vlcfg {

enum class RxState : uint8_t {
  IDLE,
  RECEIVING,
  COMPLETED,
  ERROR,
};

class RxDecoder {
 private:
  RxBuff buff;

  ConfigEntry* entries = nullptr;
  RxState state = RxState::IDLE;

 public:
  inline RxDecoder(int capacity) : buff(capacity) { buff.init(); }

  void init(ConfigEntry* dst);
  Result update(PcsOutput* in, RxState* rx_state);
  inline RxState get_state() const { return state; }
  inline ConfigEntry* entry_from_key(const char* key) const {
    return vlcfg::entry_from_key(entries, key);
  }

 private:
  Result update_state(PcsOutput* in);
  Result rx_complete();
  Result read_key(int16_t* entry_index);
  Result read_value(ConfigEntry* entry);
};

#ifdef VLCFG_IMPLEMENTATION

void RxDecoder::init(ConfigEntry* entries) {
  this->buff.init();
  this->entries = entries;
  if (entries) {
    for (uint8_t i = 0; i < MAX_ENTRY_COUNT; i++) {
      ConfigEntry& entry = entries[i];
      if (entry.key == nullptr) break;
      entry.flags &= ~ConfigEntryFlags::ENTRY_RECEIVED;
      entry.received = 0;
    }
  }
  this->state = RxState::IDLE;
  VLCFG_PRINTF("RX Decoder initialized.\n");
}

Result RxDecoder::update(PcsOutput* in, RxState* rx_state) {
  Result ret = update_state(in);
  if (ret != Result::SUCCESS) {
    state = RxState::ERROR;
  }
  if (rx_state) {
    *rx_state = state;
  }
  return ret;
}

Result RxDecoder::update_state(PcsOutput* in) {
  switch (state) {
    case RxState::IDLE:
      if (in->rxed && in->rx_byte == SYMBOL_SOF) {
        state = RxState::RECEIVING;
      }
      break;

    case RxState::RECEIVING:
      if (in->state == PcsState::LOS) {
        VLCFG_THROW(Result::ERR_LOS);
      } else if (in->rxed) {
        if (in->rx_byte == SYMBOL_EOF) {
          VLCFG_TRY(rx_complete());
          state = RxState::COMPLETED;
        } else if (0 <= in->rx_byte && in->rx_byte <= 255) {
          VLCFG_PRINTF("rxed: 0x%02X\n", (int)in->rx_byte);
          VLCFG_TRY(buff.push(in->rx_byte));
        } else {
          VLCFG_THROW(Result::ERR_EOF_EXPECTED);
        }
      }
      break;

    default: break;
  }
  return Result::SUCCESS;
}

Result RxDecoder::rx_complete() {
  VLCFG_PRINTF("%d bytes received.\n", (int)buff.size());
  // #ifdef VLCFG_DEBUG
  //   VLCFG_PRINTF("buffer content:\n");
  //   for (uint16_t i = 0; i < buff.size(); i++) {
  //     VLCFG_PRINTF("  [%4d] %02X", (int)i, (int)buff.peek(i));
  //   }
  // #endif

  VLCFG_TRY(buff.check_and_remove_crc());

  uint32_t param;
  ValueType mtype;

  uint16_t pos = 0;
  VLCFG_TRY(buff.read_item_header_u32(&mtype, &param));
  if (mtype != ValueType::MAP) {
    VLCFG_THROW(Result::ERR_UNSUPPORTED_TYPE);
  }
  if (param > MAX_ENTRY_COUNT) {
    VLCFG_THROW(Result::ERR_TOO_MANY_ENTRIES);
  }

  uint8_t num_entries = param;

  VLCFG_PRINTF("CBOR object, num_entries=%d\n", num_entries);

  for (uint8_t i = 0; i < num_entries; i++) {
    // match key
    int16_t entry_index = -1;
    VLCFG_TRY(read_key(&entry_index));
    if (entry_index < 0) {
      VLCFG_THROW(Result::ERR_KEY_NOT_FOUND);
    }
    ConfigEntry& entry = entries[entry_index];

    // value
    VLCFG_TRY(read_value(&entry));
  }

  if (buff.size() != 0) {
    VLCFG_THROW(Result::ERR_EXTRA_BYTES);
  }

  VLCFG_PRINTF("CBOR parsing completed successfully.\n");

  return Result::SUCCESS;
}

Result RxDecoder::read_key(int16_t* entry_index) {
  // read key
  ValueType mtype;
  uint32_t param;
  VLCFG_TRY(buff.read_item_header_u32(&mtype, &param));
  if (mtype != ValueType::TEXT_STR) {
    VLCFG_THROW(Result::ERR_KEY_TYPE_MISMATCH);
  }
  if (param > MAX_KEY_LEN) {
    VLCFG_THROW(Result::ERR_KEY_TOO_LONG);
  }
  uint8_t rx_key_len = param;
  char rx_key[MAX_KEY_LEN + 1];
  VLCFG_TRY(buff.popBytes((uint8_t*)rx_key, rx_key_len));
  rx_key[rx_key_len] = '\0';
  VLCFG_PRINTF("key: '%s'\n", rx_key);

  *entry_index = find_key(entries, rx_key);

  VLCFG_PRINTF("--> field_index=%d\n", *entry_index);
  return Result::SUCCESS;
}

Result RxDecoder::read_value(ConfigEntry* entry) {
  ValueType mtype;
  uint32_t param;
  VLCFG_TRY(buff.read_item_header_u32(&mtype, &param));

  if (entry != nullptr) {
    if (mtype != entry->type) {
      VLCFG_THROW(Result::ERR_VALUE_TYPE_MISMATCH);
    }
    if (entry->buffer == nullptr) {
      VLCFG_THROW(Result::ERR_NULL_POINTER);
    }
  }

  uint8_t len = 0;
  switch (mtype) {
    case ValueType::BYTE_STR:
    case ValueType::TEXT_STR: {
      bool is_text = (mtype == ValueType::TEXT_STR);
      len = param;
      if (entry != nullptr) {
        uint8_t buff_req = is_text ? len + 1 : len;
        if (buff_req > entry->capacity) {
          VLCFG_THROW(Result::ERR_VALUE_TOO_LONG);
        }
        uint8_t* dst = (uint8_t*)entry->buffer;
        VLCFG_TRY(buff.popBytes(dst, len));
        if (is_text) {
          dst[len] = '\0';
          VLCFG_PRINTF("string value: '%s'\n", (char*)entry->buffer);
        }
      } else {
        VLCFG_TRY(buff.skip(len));
      }
    } break;

    case ValueType::BOOLEAN: {
      len = 1;
      if (entry != nullptr) {
        if (entry->capacity < 1) {
          VLCFG_THROW(Result::ERR_VALUE_TOO_LONG);
        }
        *((uint8_t*)entry->buffer) = param;
        VLCFG_PRINTF("boolean value: %s\n", param ? "true" : "false");
      }
    } break;

    default: VLCFG_THROW(Result::ERR_VALUE_TYPE_MISMATCH);
  }

  entry->flags |= ConfigEntryFlags::ENTRY_RECEIVED;
  entry->received = len;
  return Result::SUCCESS;
}

#endif

}  // namespace vlcfg

#endif
// #include "vlcfg/rx_pcs.hpp"

#ifndef VLCFG_RX_PCS_HPP
#define VLCFG_RX_PCS_HPP

// #include "vlcfg/common.hpp"


namespace vlcfg {

class RxPcs {
 private:
  PcsState state;
  uint16_t shift_reg;
  uint8_t phase;

 public:
  inline RxPcs() { init(); }
  void init();
  Result update(const CdrOutput *in, PcsOutput *out);
  inline PcsState get_state() const { return state; }

 private:
  void reset_internal();
};

#ifdef VLCFG_IMPLEMENTATION

static const int8_t DECODE_TABLE[1 << SYMBOL_BITS] = {
    SYMBOL_INVALID,  // 0b00000
    SYMBOL_INVALID,  // 0b00001
    SYMBOL_INVALID,  // 0b00010
    SYMBOL_SOF,      // 0b00011
    SYMBOL_INVALID,  // 0b00100
    0x0,             // 0b00101
    0x1,             // 0b00110
    SYMBOL_EOF,      // 0b00111
    SYMBOL_INVALID,  // 0b01000
    0x2,             // 0b01001
    SYMBOL_CONTROL,  // 0b01010
    0x3,             // 0b01011
    0x4,             // 0b01100
    0x5,             // 0b01101
    0x6,             // 0b01110
    SYMBOL_INVALID,  // 0b01111
    SYMBOL_INVALID,  // 0b10000
    SYMBOL_SYNC,     // 0b10001
    0x7,             // 0b10010
    0x8,             // 0b10011
    0x9,             // 0b10100
    0xA,             // 0b10101
    0xB,             // 0b10110
    SYMBOL_INVALID,  // 0b10111
    0xC,             // 0b11000
    0xD,             // 0b11001
    0xE,             // 0b11010
    SYMBOL_INVALID,  // 0b11011
    0xF,             // 0b11100
    SYMBOL_INVALID,  // 0b11101
    SYMBOL_INVALID,  // 0b11110
    SYMBOL_INVALID,  // 0b11111
};

void RxPcs::init() {
  reset_internal();
  VLCFG_PRINTF("RX PCS initialized.\n");
}

Result RxPcs::update(const CdrOutput *in, PcsOutput *out) {
  if (out == nullptr) {
    VLCFG_THROW(Result::ERR_NULL_POINTER);
  }

  if (!in->signal_detected) {
    reset_internal();
    out->state = state;
    out->rxed = false;
    return Result::SUCCESS;
  } else if (!in->rxed) {
    out->state = state;
    out->rxed = false;
    return Result::SUCCESS;
  }

  // shift register
  constexpr uint16_t SHIFT_REG_MASK = (1 << (SYMBOL_BITS * 2)) - 1;
  shift_reg = (shift_reg << 1) & SHIFT_REG_MASK;
  if (in->rx_bit) shift_reg |= 1;

  constexpr uint8_t SYMBOL_MASK = (1 << SYMBOL_BITS) - 1;
  int8_t nibble_h = DECODE_TABLE[(shift_reg >> SYMBOL_BITS) & SYMBOL_MASK];
  int8_t nibble_l = DECODE_TABLE[shift_reg & SYMBOL_MASK];
  bool rxed_sync = false, rxed_sof = false, rxed_eof = false;
  if (nibble_h == SYMBOL_CONTROL) {
    rxed_sync = (nibble_l == SYMBOL_SYNC);
    rxed_sof = (nibble_l == SYMBOL_SOF);
    rxed_eof = (nibble_l == SYMBOL_EOF);
  }

  bool rxed = false;
  PcsState last_state = state;
  if (state == PcsState::LOS) {
    if (rxed_sync) {
      // symbol lock
      phase = 0;
      state = PcsState::RXED_SYNC1;
    } else {
      state = PcsState::LOS;
    }
  } else if (phase < (SYMBOL_BITS * 2 - 1)) {
    phase++;
  } else {
    phase = 0;

    // symbol decode
    switch (state) {
      case PcsState::RXED_SYNC1:
        if (rxed_sync) {
          state = PcsState::RXED_SYNC2;
        } else {
          state = PcsState::LOS;
        }
        break;

      case PcsState::RXED_SYNC2:
        if (rxed_sof) {
          rxed = true;
          out->rx_byte = SYMBOL_SOF;
          state = PcsState::RXED_SOF;
        } else if (rxed_sync) {
          state = PcsState::RXED_SYNC2;
        } else {
          state = PcsState::LOS;
        }
        break;

      case PcsState::RXED_SOF:
      case PcsState::RXED_BYTE:
        if (rxed_eof) {
          rxed = true;
          out->rx_byte = SYMBOL_EOF;
          state = PcsState::RXED_EOF;
        } else if (nibble_h >= 0 && nibble_l >= 0) {
          rxed = true;
          out->rx_byte = (nibble_h << 4) | nibble_l;
          state = PcsState::RXED_BYTE;
        } else {
          state = PcsState::LOS;
        }
        break;

      case PcsState::RXED_EOF:
        if (rxed_sof) {
          rxed = true;
          out->rx_byte = SYMBOL_SOF;
          state = PcsState::RXED_SOF;
        } else if (rxed_sync) {
          state = PcsState::RXED_SYNC2;
        } else {
          state = PcsState::LOS;
        }
        break;

      default: break;
    }
  }

#ifdef VLCFG_DEBUG
  if (last_state != state) {
    VLCFG_PRINTF("PCS State: %d --> %d\n", (int)last_state, (int)state);
  }
#endif

  out->state = state;
  out->rxed = rxed;
  return Result::SUCCESS;
}

void RxPcs::reset_internal() {
  state = PcsState::LOS;
  phase = 0;
  shift_reg = 0;
}

#endif

}  // namespace vlcfg

#endif

namespace vlcfg {

class Receiver {
 public:
  RxCdr cdr;
  RxPcs pcs;
  RxDecoder decoder;

 private:
  bool last_bit;
  uint8_t last_byte;

 public:
  inline Receiver(int rx_buff_size = 256, ConfigEntry *entries = nullptr)
      : decoder(rx_buff_size) {
    init(entries);
  }

  void init(ConfigEntry *entries);
  Result update(uint16_t adc_val, RxState *rx_state);

  inline bool signal_detected() const { return cdr.signal_detected(); }
  inline PcsState get_pcs_state() const { return pcs.get_state(); }
  inline RxState get_decoder_state() const { return decoder.get_state(); }

  inline bool get_last_bit() const { return last_bit; }
  inline uint8_t get_last_byte() const { return last_byte; }

  inline ConfigEntry *entry_from_key(const char *key) const {
    return decoder.entry_from_key(key);
  }
};  // class

#ifdef VLCFG_IMPLEMENTATION

void Receiver::init(ConfigEntry *entries) {
  cdr.init();
  pcs.init();
  decoder.init(entries);
  VLCFG_PRINTF("Receiver initialized.\n");
}

Result Receiver::update(uint16_t adc_val, RxState *rx_state) {
  CdrOutput cdrOut;
  VLCFG_TRY(cdr.update(adc_val, &cdrOut));
  if (cdrOut.rxed) last_bit = cdrOut.rx_bit;

  PcsOutput pcsOut;
  VLCFG_TRY(pcs.update(&cdrOut, &pcsOut));
  if (pcsOut.rxed) last_byte = pcsOut.rx_byte;

  VLCFG_TRY(decoder.update(&pcsOut, rx_state));

  return Result::SUCCESS;
}

#endif

}  // namespace vlcfg

#endif

#endif

using VlcfgResult = vlcfg::Result;
using VlcfgType = vlcfg::ValueType;
using VlcfgEntry = vlcfg::ConfigEntry;
using VlcfgReceiver = vlcfg::Receiver;
using VlcfgRxState = vlcfg::RxState;

#endif
