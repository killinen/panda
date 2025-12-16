const int I30_MAX_STEER = 384;             // like stock
const int I30_MAX_RT_DELTA = 112;          // max delta torque allowed for real time checks
const uint32_t I30_RT_INTERVAL = 250000;   // 250ms between real time checks
const int I30_MAX_RATE_UP = 3;
const int I30_MAX_RATE_DOWN = 7;
const int I30_DRIVER_TORQUE_ALLOWANCE = 50;
const int I30_DRIVER_TORQUE_FACTOR = 2;
const int I30_STANDSTILL_THRSLD = 30;  // ~1kph

const int I30_MAX_ACCEL = 200;  // 1/100 m/s2
const int I30_MIN_ACCEL = -350; // 1/100 m/s2

#define I30_GET_INTERCEPTOR(msg) (((GET_BYTE((msg), 0) << 8) + GET_BYTE((msg), 1) + (GET_BYTE((msg), 2) << 8) + GET_BYTE((msg), 3)) / 2U) // avg between 2 tracks
#define I30_GAS_INTERCEPTOR_SAFETY

// These are messages that will/can be sent to the busses
const CanMsg I30_TX_MSGS[] = {
  {558, 1, 5},  // SSC Bus 1
  // {832, 0, 8},  // LKAS11 Bus 0 (non i30 msg)
  // {1265, 0, 4}, // CLU11 Bus 0 (is present in i30 but IMHO would not need in here)
  // {1157, 0, 4}, // LFAHDA_MFC Bus 0 (non i30 msg)
};

const CanMsg I30_LONG_TX_MSGS[] = {
  {512, 1, 6},  // GAS_COMMAND Bus 1
  {558, 1, 5},  // SSC Bus 1
};

 // Each AddrCheckStruct.msg array supports up to 3 CAN messages for validation.
// 1. Why 3 Entries?
//    - Allows monitoring of multiple related messages (e.g., torque and checksum checks).
//    - Fixed size ensures predictable memory usage and simplifies parsing logic in embedded systems.
//    - Balances memory efficiency and future expandability for features needing multiple messages.
// 2. Why Are They Not All Populated?
//    - Not all features require multiple messages; some only need 1 or 2 messages.
//    - Unused entries are initialized to { 0 }, which marks them as placeholders.
//    - This avoids wasting memory and ensures clarity in code maintenance.
// 3. Use in Functions:
//    - Functions processing the msg array loop through all entries and stop at { 0 }, 
//      which acts as a termination marker.
//    - This eliminates the need for explicit array length tracking, simplifying runtime logic.
AddrCheckStruct i30_addr_checks[] = {
  {.msg = {{0x081, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 12000U}, { 0 }, { 0 }}},   // EMS_DCT2 (129)
  {.msg = {{0x165, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 12000U}, { 0 }, { 0 }}},   // VSM2 (357)
  {.msg = {{0x1F1, 0, 8, .check_checksum = false, .expected_timestep = 20000U}, { 0 }, { 0 }}},                      // TCS5 (497)
  {.msg = {{0x260, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 12000U}, { 0 }, { 0 }}},    // EMS6 (260)
  {.msg = {{0x2B0, 0, 5, .check_checksum = true, .max_counter = 15U, .expected_timestep = 12000U}, { 0 }, { 0 }}},   // SAS1 (688)
  {.msg = {{0x22F, 1, 8, .check_checksum = false, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},  // SSC (559)
  // {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
  //          {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  // {.msg = {{902, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  // {.msg = {{916, 0, 8, .check_checksum = true, .max_counter = 7U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  // {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define I30_ADDR_CHECK_LEN (sizeof(i30_addr_checks) / sizeof(i30_addr_checks[0]))

// Longitudinal variant adds pedal interceptor monitoring on top of the standard checks.
AddrCheckStruct i30_long_addr_checks[] = {
  {.msg = {{0x081, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 12000U}, { 0 }, { 0 }}},   // EMS_DCT2 (129)
  {.msg = {{0x165, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 12000U}, { 0 }, { 0 }}},   // VSM2 (357)
  {.msg = {{0x1F1, 0, 8, .check_checksum = false, .expected_timestep = 20000U}, { 0 }, { 0 }}},                      // TCS5 (497)
  {.msg = {{0x260, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 12000U}, { 0 }, { 0 }}},    // EMS6 (260)
  {.msg = {{0x2B0, 0, 5, .check_checksum = true, .max_counter = 15U, .expected_timestep = 12000U}, { 0 }, { 0 }}},   // SAS1 (688)
  {.msg = {{0x22F, 1, 8, .check_checksum = false, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},  // SSC (559)
  {.msg = {{513, 1, 6, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},   // Gas interceptor (bus 1)
};
#define I30_LONG_ADDR_CHECK_LEN (sizeof(i30_long_addr_checks) / sizeof(i30_long_addr_checks[0]))

// older i30 models have less checks due to missing counters and checksums
// const int I30_PARAM_EV_GAS = 1;
// const int I30_PARAM_HYBRID_GAS = 2;
const int I30_PARAM_LONGITUDINAL = 4;
// const int I30_PARAM_I30_LONGITUDINAL = 17;

enum {
  I30_BTN_NONE = 0,
  I30_BTN_RESUME = 1,
  I30_BTN_SET = 2,
  I30_BTN_CANCEL = 4,
};

// some newer HKG models can re-enable after spamming cancel button,
// so keep track of user button presses to deny engagement if no interaction
const uint8_t I30_PREV_BUTTON_SAMPLES = 8;  // roughly 160 ms
uint8_t i30_last_button_interaction;  // button messages since the user pressed an enable button
// bool i30_ev_gas_signal = false;
// bool i30_hybrid_gas_signal = false;
bool i30_longitudinal = false;
static bool i30_stock_cruise_main = false;

#ifdef I30_GAS_INTERCEPTOR_SAFETY
static uint8_t counter_pedal_last = 0;
#endif

addr_checks i30_rx_checks = {i30_addr_checks, I30_ADDR_CHECK_LEN};

#ifdef I30_GAS_INTERCEPTOR_SAFETY
static uint8_t crc8_pedal(const uint8_t *data, int len) {
  uint8_t crc = 0xFFU;
  const uint8_t poly = 0xD5U;
  for (int i = len - 1; i >= 0; i--) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if ((((uint32_t)crc) & ((uint32_t)0x80U)) != ((uint32_t)0U)) {
        crc = (uint8_t)(((uint8_t)(crc << 1)) ^ poly);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

// Wrapper for CANPacket_t
static uint8_t i30_compute_pedal_crc(CANPacket_t *to_push) {
  uint8_t dat[5];
  for (int i = 0; i < 5; i++) {
    dat[i] = GET_BYTE(to_push, i);
  }
  return crc8_pedal(dat, 5);
}
#endif

static uint8_t i30_get_counter(CANPacket_t *to_push) {
    uint32_t addr = GET_ADDR(to_push);
  
    uint8_t cnt;
    if (addr == 129U) {
      cnt = GET_BYTE(to_push, 7) & 0xFU;
    } else if (addr == 357U) {
      cnt = GET_BYTE(to_push, 6) & 0xFU;
    } else if (addr == 559U) {
      cnt = GET_BYTE(to_push, 1) & 0xFU;
    } else if (addr == 513U) {
      cnt = GET_BYTE(to_push, 4) & 0xFU;
    } else if (addr == 608U) {
      cnt = (GET_BYTE(to_push, 7) >> 4) & 0x3U;
    } else if (addr == 688U) {
      cnt = GET_BYTE(to_push, 4) & 0xFU;
    // } else if (addr == 902) {
    //   cnt = ((GET_BYTE(to_push, 3) >> 6) << 2) | (GET_BYTE(to_push, 1) >> 6);
    // } else if (addr == 916) {
    //   cnt = (GET_BYTE(to_push, 1) >> 5) & 0x7;
    // } else if (addr == 1057) {
    //   cnt = GET_BYTE(to_push, 7) & 0xF;
    // } else if (addr == 1265) {
    //   cnt = (GET_BYTE(to_push, 3) >> 4) & 0xFU;
    } else {
      cnt = 0U;
    }
    return cnt;
  }

static uint32_t i30_get_checksum(CANPacket_t *to_push) {
  uint32_t addr = GET_ADDR(to_push);

  uint8_t chksum;
  if (addr == 129U) {
    chksum = (GET_BYTE(to_push, 7) >> 4) & 0xFU;
  } else if (addr == 357U) {
    chksum = GET_BYTE(to_push, 7);
  } else if (addr == 513U) {
    chksum = GET_BYTE(to_push, 5);
  } else if (addr == 559U) {
    chksum = GET_BYTE(to_push, 0);
  } else if (addr == 608U) {
    chksum = GET_BYTE(to_push, 7) & 0xFU;
  } else if (addr == 688U) {
    chksum = (GET_BYTE(to_push, 4) >> 4) & 0xFU;
  // } else if (addr == 902) {
  //   chksum = ((GET_BYTE(to_push, 7) >> 6) << 2) | (GET_BYTE(to_push, 5) >> 6);
  // } else if (addr == 916) {
  //   chksum = GET_BYTE(to_push, 6) & 0xFU;
  // } else if (addr == 1057) {
  //   chksum = GET_BYTE(to_push, 7) >> 4;
  } else {
    chksum = 0U;
  }
  return chksum;
}

static uint32_t i30_compute_checksum(CANPacket_t *to_push) {
    uint32_t addr = GET_ADDR(to_push);

    uint8_t chksum = 0;
    int data_length = 8;
    if ((addr == 357U) || (addr == 688U)) {
      data_length = (addr == 688U) ? 5 : 7;
      // Use XOR checksum algorithm on the first 7 bytes for 357 and 5 bytes for 688
      for (int i = 0; i < data_length; i++) {
        uint8_t b = GET_BYTE(to_push, i);
        // Remove checksum nibble based on address and byte position
        if ((addr == 688U) && (i == 4)) {
          b &= 0x0FU;  // Mask checksum byte
        }
        chksum ^= b;
      }
      if (addr == 688U) {
        //chksum &= 0x0FU;
        uint32_t tmp = (uint32_t)chksum;
        // cppcheck-suppress misra-c2012-10.4 ; tooling false-positive on explicit unsigned operations
        chksum = (uint8_t)((tmp & (uint32_t)0x0FU) ^ (tmp >> (uint32_t)4U));
      }
    } else if (addr == 513U) {
      chksum = i30_compute_pedal_crc(to_push);
    } else if (addr == 559U) {
      uint16_t ssc_chksum = (uint16_t)559U;
      data_length = 7;
      // 0 byte is the checksum
      for (int i = 1; i < data_length; i++) {
        uint8_t b = GET_BYTE(to_push, i);
        ssc_chksum += b;
      }
      // Add upper and lower bytes of the checksum
      ssc_chksum = (uint16_t)((ssc_chksum & 0xFFU) + (ssc_chksum >> 8));

      // Mask to keep only the lower 8 bits
      chksum = (uint8_t)(ssc_chksum & 0xFFU);
    } else {
      // Standard checksum algorithm for addresses 608, 916, and 129
      for (int i = 0; i < data_length; i++) {
        uint8_t b = GET_BYTE(to_push, i);

        // Remove checksum nibble based on address and byte position
        if (((addr == 608U) && (i == 7)) || ((addr == 129U) && (i == 7))) {
          b &= (addr == 129U) ? 0x0FU : 0xF0U;  // Mask checksum byte
        }

        // Sum the nibbles (4-bit parts of the byte)
        chksum += (b % 16U) + (b / 16U);
      }

      // Final checksum calculation with modulo 16 for other addresses
      chksum = (16U - (chksum % 16U)) % 16U;
    }

    return chksum;
  }

static int i30_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &i30_rx_checks,
                                 i30_get_checksum, i30_compute_checksum,
                                 i30_get_counter);

  bool bus0 = GET_BUS(to_push) == 0U;
  // cppcheck-suppress misra-c2012-10.4 ; tooling false-positive on mixed-width unsigned compares
  bool bus1_interceptor = (GET_BUS(to_push) == 1U) && (GET_ADDR(to_push) == (uint32_t)513U);

  if (valid && (bus0 || bus1_interceptor)) {
    int addr = GET_ADDR(to_push);

    // Skip the steer torque as now, add in SSC stuff later
    // if (addr == 593) {
    //   int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ffU) * 0.79) - 808; // scale down new driver torque signal to match previous one
    //   // update array of samples
    //   update_sample(&torque_driver, torque_driver_new);
    // }

    // ACC steering wheel buttons
    // I think this is OP cruise state machine, I just mod this to my own use
    if (addr == 1264) {   // Changed the cruise button adress for I30 1265 -> 1264, should this be on the address checks?
      int cruise_button = GET_BYTE(to_push, 0) & 0x7U;
      int main_button = GET_BIT(to_push, 24U);

      if ((cruise_button == I30_BTN_RESUME) || (cruise_button == I30_BTN_SET) || (cruise_button == I30_BTN_CANCEL) || (main_button != 0)) {
        i30_last_button_interaction = 0U;
      } else {
        i30_last_button_interaction = MIN(i30_last_button_interaction + 1U, I30_PREV_BUTTON_SAMPLES);
      }

      if (i30_longitudinal) {
        // exit controls on cancel press
        if (cruise_button == I30_BTN_CANCEL) {
          controls_allowed = 0;
        }

        // enter controls on falling edge of resume or set
        bool set = (cruise_button == I30_BTN_NONE) && (cruise_button_prev == I30_BTN_SET);
        bool res = (cruise_button == I30_BTN_NONE) && (cruise_button_prev == I30_BTN_RESUME);
        if (set || res) {
          if (!i30_stock_cruise_main) {
            controls_allowed = 1;
          }
        }

        cruise_button_prev = cruise_button;
      }
    }

    // Lateral-only: enter controls when stock cruise engages, exit when disengaged
    if ((!i30_longitudinal) && (addr == 608)) {
      int cruise_engaged = (GET_BYTE(to_push, 3) >> 2) & 0x1U;
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // enter controls on rising edge of ACC and user button press, exit controls when ACC off
    // check for gas interceptor
    if (addr == 513) {
      gas_interceptor_detected = true;
      int gas_interceptor = I30_GET_INTERCEPTOR(to_push);
      const int I30_GAS_INTERCEPTOR_THRESHOLD = 2340; // avg of raw vals at 0 gas is 2250
      gas_pressed = gas_interceptor > I30_GAS_INTERCEPTOR_THRESHOLD;
    }

    // read gas pressed signal
    if (!gas_interceptor_detected) {
      if (addr == 608) {  // ICE
        gas_pressed = (GET_BYTE(to_push, 7) >> 6) != 0U;
      } else {
      }
    }

    // sample wheel speed, averaging opposite corners
    if (addr == 497) {    // 0x1F1
      uint32_t speed_fl = (GET_BYTES_04(to_push) >> 16U) & 0xFFFU;
      uint32_t speed_rr = (GET_BYTES_48(to_push) >> 20U) & 0xFFFU;
      // i30 uses 12-bit speed; scale to match standstill threshold
      uint32_t speed_scaled = (speed_fl + speed_rr) * 2U;
      vehicle_moving = speed_scaled > (uint32_t)I30_STANDSTILL_THRSLD;
    }

    if (addr == 129) {    // 0x081
        brake_pressed = (GET_BYTE(to_push, 0) >> 7) != 0U;
    }

    bool stock_ecu_detected = false;

    // For i30 longitudinal (pedal interceptor), don't allow openpilot controls while
    // stock cruise is in the MAIN (standby) state. We check CRUISE_LAMP_M on message
    // 608 (bit 25).
    if (i30_longitudinal && (addr == 608)) {
      i30_stock_cruise_main = (((GET_BYTE(to_push, 3) >> 1) & 0x1U) != 0U);
      if (i30_stock_cruise_main) {
        controls_allowed = 0;
      }
    }

    generic_rx_checks(stock_ecu_detected);
  }
  return valid;
}

static int i30_tx_hook(CANPacket_t *to_send, bool longitudinal_allowed) {

  int tx = 1;
  int addr = GET_ADDR(to_send);

  if (i30_longitudinal) {
    tx = msg_allowed(to_send, I30_LONG_TX_MSGS, sizeof(I30_LONG_TX_MSGS)/sizeof(I30_LONG_TX_MSGS[0]));
  } else {
    tx = msg_allowed(to_send, I30_TX_MSGS, sizeof(I30_TX_MSGS)/sizeof(I30_TX_MSGS[0]));
  }

#ifdef I30_GAS_INTERCEPTOR_SAFETY
  // GAS Pedal Interceptor command
  if (addr == 512) {
    bool enable = GET_BIT(to_send, 39U);
    int gas_command = GET_BYTE(to_send, 0) | (GET_BYTE(to_send, 1) << 8);
    int gas_command2 = GET_BYTE(to_send, 2) | (GET_BYTE(to_send, 3) << 8);
    uint8_t counter = (GET_BYTE(to_send, 4) >> 0) & 0xFU;
    uint8_t checksum = GET_BYTE(to_send, 5);

    bool violation = false;
    const int GAS_COMMAND_MIN = 3600;
    const int GAS_COMMAND_MAX = 3608;
    const int GAS_COMMAND2_MIN = 900;
    const int GAS_COMMAND2_MAX = 904;

    if (!i30_longitudinal) {
      violation = true;
    } else if (enable) {
      if ((gas_command < GAS_COMMAND_MIN) || (gas_command > GAS_COMMAND_MAX)) {
        violation = true;
      }
      if ((gas_command2 < GAS_COMMAND2_MIN) || (gas_command2 > GAS_COMMAND2_MAX)) {
        violation = true;
      }
      // What does longitudinal_allowed mean?
      if (!longitudinal_allowed) {
        violation = true;
      }
    } else {
      if ((gas_command != GAS_COMMAND_MIN) || (gas_command2 != GAS_COMMAND2_MIN)) {
        violation = true;
      }
    }

    const bool neutral = !enable && (gas_command == GAS_COMMAND_MIN) && (gas_command2 == GAS_COMMAND2_MIN);
    const bool counter_valid = (((counter_pedal_last + 1U) & 0xFU) == counter);
    const bool counter_initted = (counter_pedal_last != 0xFFU);
    if ((!counter_initted || !counter_valid) && !neutral) {
      violation = true;
    }

    if (i30_compute_pedal_crc(to_send) != checksum) {
      violation = true;
    }

    if (violation) {
      tx = 0;
    } else {
      if (tx != 0) {
        counter_pedal_last = counter;
      }
    }
  }
#endif
/*
  // LKA STEER: safety check
  if (addr == 832) {
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x7ffU) - 1024U;
    bool steer_req = GET_BIT(to_send, 27U) != 0U;
    uint32_t ts = microsecond_timer_get();
    bool violation = false;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, I30_MAX_STEER, -I30_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        I30_MAX_STEER, I30_MAX_RATE_UP, I30_MAX_RATE_DOWN,
        I30_DRIVER_TORQUE_ALLOWANCE, I30_DRIVER_TORQUE_FACTOR);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, I30_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > I30_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed or mismatch with CF_Lkas_ActToi bit
      if ((!controls_allowed || !steer_req) && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) {
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }
*/
  // UDS: Only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (addr == 2000) {
    if ((GET_BYTES_04(to_send) != 0x00803E02U) || (GET_BYTES_48(to_send) != 0x0U)) {
      tx = 0;
    }
  }
/*
  // BUTTONS: used for resume spamming and cruise cancellation
  // TODO!!!! When have time look at this, in I30 there is same msg ID, but different functionality (CLU3)
  if ((addr == 1265) && !i30_longitudinal) {
    int button = GET_BYTE(to_send, 0) & 0x7U;

    bool allowed_resume = (button == 1) && controls_allowed;
    bool allowed_cancel = (button == 4) && cruise_engaged_prev;
    if (!(allowed_resume || allowed_cancel)) {
      tx = 0;
    }
  }
*/
  return tx;
}

static int i30_fwd_hook(int bus_num, CANPacket_t *to_fwd) {

  int bus_fwd = -1;
  (void)bus_num;
  (void)to_fwd;

  // No forwarding for i30 thank you
  
  // forward cam to ccan and viceversa, except lkas cmd
  //if (bus_num == 0) {
  //  bus_fwd = 2;
  //}
  //if ((bus_num == 2) && (addr != 832) && (addr != 1157)) {
  //  bus_fwd = 0;
  //}

  // No forwarding for i30 thank you
  //if (i30_longitudinal) {
  //  bus_fwd = -1;
  //}

  return bus_fwd;
}

static const addr_checks* i30_init(uint16_t param) {
  i30_last_button_interaction = I30_PREV_BUTTON_SAMPLES;
  // Default to lateral-only. Enable pedal interceptor longitudinal with the flag.
  i30_longitudinal = GET_FLAG(param, I30_PARAM_LONGITUDINAL);
  i30_stock_cruise_main = false;
#ifdef I30_GAS_INTERCEPTOR_SAFETY
  counter_pedal_last = 0xFFU;
#endif

  if (i30_longitudinal) {
    i30_rx_checks = (addr_checks){i30_long_addr_checks, I30_LONG_ADDR_CHECK_LEN};
  } else {
    i30_rx_checks = (addr_checks){i30_addr_checks, I30_ADDR_CHECK_LEN};
  }

  return &i30_rx_checks;
}

const safety_hooks i30_hooks = {
  .init = i30_init,
  .rx = i30_rx_hook,
  .tx = i30_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = i30_fwd_hook,
};
