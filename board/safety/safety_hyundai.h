const int HYUNDAI_MAX_STEER = 384;             // like stock
const int HYUNDAI_MAX_RT_DELTA = 112;          // max delta torque allowed for real time checks
const uint32_t HYUNDAI_RT_INTERVAL = 250000;   // 250ms between real time checks
const int HYUNDAI_MAX_RATE_UP = 3;
const int HYUNDAI_MAX_RATE_DOWN = 7;
const int HYUNDAI_DRIVER_TORQUE_ALLOWANCE = 50;
const int HYUNDAI_DRIVER_TORQUE_FACTOR = 2;
const int HYUNDAI_STANDSTILL_THRSLD = 30;  // ~1kph

const int HYUNDAI_MAX_ACCEL = 200;  // 1/100 m/s2
const int HYUNDAI_MIN_ACCEL = -350; // 1/100 m/s2

const CanMsg HYUNDAI_TX_MSGS[] = {
  {558, 1, 5}, // SSC Bus 1 with BP
  {832, 0, 8},  // LKAS11 Bus 0
  {1265, 0, 4}, // CLU11 Bus 0
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
 };

const CanMsg HYUNDAI_LONG_TX_MSGS[] = {
  {832, 0, 8},  // LKAS11 Bus 0
  {1265, 0, 4}, // CLU11 Bus 0
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
  {1056, 0, 8}, // SCC11 Bus 0
  {1057, 0, 8}, // SCC12 Bus 0
  {1290, 0, 8}, // SCC13 Bus 0
  {905, 0, 8},  // SCC14 Bus 0
  {1186, 0, 2}, // FRT_RADAR11 Bus 0
  {909, 0, 8},  // FCA11 Bus 0
  {1155, 0, 8}, // FCA12 Bus 0
  {2000, 0, 8}, // radar UDS TX addr Bus 0 (for radar disable)
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
AddrCheckStruct hyundai_addr_checks[] = {
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
#define HYUNDAI_ADDR_CHECK_LEN (sizeof(hyundai_addr_checks) / sizeof(hyundai_addr_checks[0]))

AddrCheckStruct hyundai_long_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .check_checksum = true, .max_counter = 7U, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1265, 0, 4, .check_checksum = false, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_LONG_ADDR_CHECK_LEN (sizeof(hyundai_long_addr_checks) / sizeof(hyundai_long_addr_checks[0]))

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_legacy_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{916, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},
};
#define HYUNDAI_LEGACY_ADDR_CHECK_LEN (sizeof(hyundai_legacy_addr_checks) / sizeof(hyundai_legacy_addr_checks[0]))

const int HYUNDAI_PARAM_EV_GAS = 1;
const int HYUNDAI_PARAM_HYBRID_GAS = 2;
const int HYUNDAI_PARAM_LONGITUDINAL = 4;

bool hyundai_legacy = false;
bool hyundai_ev_gas_signal = false;
bool hyundai_hybrid_gas_signal = false;
bool hyundai_longitudinal = false;

addr_checks hyundai_rx_checks = {hyundai_addr_checks, HYUNDAI_ADDR_CHECK_LEN};

static uint8_t hyundai_get_counter(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t cnt;
  if (addr == 129) {
    cnt = GET_BYTE(to_push, 7) & 0xF;
  } else if (addr == 357) {
    cnt = GET_BYTE(to_push, 6) & 0xF;
  } else if (addr == 559) {
    cnt = GET_BYTE(to_push, 1) & 0xF;
  } else if (addr == 608) {
    cnt = (GET_BYTE(to_push, 7) >> 4) & 0x3;
  } else if (addr == 688) {
    cnt = GET_BYTE(to_push, 4) & 0xF;
  } else if (addr == 902) {
    cnt = ((GET_BYTE(to_push, 3) >> 6) << 2) | (GET_BYTE(to_push, 1) >> 6);
  } else if (addr == 916) {
    cnt = (GET_BYTE(to_push, 1) >> 5) & 0x7;
  } else if (addr == 1057) {
    cnt = GET_BYTE(to_push, 7) & 0xF;
  } else if (addr == 1265) {
    cnt = (GET_BYTE(to_push, 3) >> 4) & 0xFU;
  } else {
    cnt = 0;
  }
  return cnt;
}

static uint8_t hyundai_get_checksum(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t chksum;
  if (addr == 129) {
    chksum = (GET_BYTE(to_push, 7) >> 4) & 0xF;
  } else if (addr == 357) {
    chksum = GET_BYTE(to_push, 7);
  } else if (addr == 559) {
    chksum = GET_BYTE(to_push, 0);
  } else if (addr == 608) {
    chksum = GET_BYTE(to_push, 7) & 0xF;
  } else if (addr == 688) {
    chksum = (GET_BYTE(to_push, 4) >> 4) & 0xF;
  } else if (addr == 916) {
    chksum = GET_BYTE(to_push, 6) & 0xF;
  } else if (addr == 1057) {
    chksum = GET_BYTE(to_push, 7) >> 4;
  } else {
    chksum = 0;
  }
  return chksum;
}

static uint8_t hyundai_compute_checksum(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t chksum = 0;
  uint8_t data_length = 8;
  if (addr == 357 || addr == 688) {
    data_length = (addr == 688) ? 5 : 7;
    // Use XOR checksum algorithm on the first 7 bytes for 357 and 5 bytes for 688
    for (int i = 0; i < data_length; i++) {
      uint8_t b = GET_BYTE(to_push, i);
      // Remove checksum nibble based on address and byte position
      if (addr == 688 && i == 4) {
        b &= 0x0FU;  // Mask checksum byte
      }
      chksum ^= b;
    }
    if (addr == 688) {
      //chksum &= 0x0FU;
      chksum = (chksum & 0x0F) ^ (chksum >> 4);
    }
  } else if (addr == 559) {
    uint16_t ssc_chksum = addr;
    data_length = 7;
    // 0 byte is the checksum
    for (int i = 1; i < data_length; i++) {
      uint8_t b = GET_BYTE(to_push, i);
      ssc_chksum += b;
    }
    // Add upper and lower bytes of the checksum
    ssc_chksum = (ssc_chksum & 0xFF) + (ssc_chksum >> 8);

    // Mask to keep only the lower 8 bits
    chksum = ssc_chksum & 0xFF;
  } else {
    // Standard checksum algorithm for addresses 608, 916, and 129
    for (int i = 0; i < data_length; i++) {
      uint8_t b = GET_BYTE(to_push, i);

      // Remove checksum nibble based on address and byte position
      if ((addr == 608 && i == 7) || (addr == 129 && i == 7)) {
        b &= (addr == 129) ? 0x0FU : 0xF0U;  // Mask checksum byte
      }

      // Sum the nibbles (4-bit parts of the byte)
      chksum += (b % 16U) + (b / 16U);
    }

    // Final checksum calculation with modulo 16 for other addresses
    chksum = (16U - (chksum % 16U)) % 16U;
  }

  return chksum;
}

static int hyundai_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &hyundai_rx_checks,
                                 hyundai_get_checksum, hyundai_compute_checksum,
                                 hyundai_get_counter);

  if (valid && (GET_BUS(to_push) == 0U)) {
    int addr = GET_ADDR(to_push);

    // Skip the steer torque as now, add in SSC stuff later
    // if (addr == 593) {
    //   int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ffU) * 0.79) - 808; // scale down new driver torque signal to match previous one
    //   // update array of samples
    //   update_sample(&torque_driver, torque_driver_new);
    // }

    if (hyundai_longitudinal) {
      // ACC steering wheel buttons
      if (addr == 1265) {
        int button = GET_BYTE(to_push, 0) & 0x7U;
        switch (button) {
          case 1:  // resume
          case 2:  // set
            controls_allowed = 1;
            break;
          case 4:  // cancel
            controls_allowed = 0;
            break;
          default:
            break;  // any other button is irrelevant
        }
      }
    } else {
      // enter controls on rising edge of ACC, exit controls on ACC off
      if (addr == 608) {
        // 3 bit from 3 byte
        int cruise_engaged = (GET_BYTE(to_push, 3) >> 2) & 0x01;
        if (cruise_engaged && !cruise_engaged_prev) {
          controls_allowed = 1;
        }
        if (!cruise_engaged) {
          controls_allowed = 0;
        }
        cruise_engaged_prev = cruise_engaged;
      }
    }

    // read gas pressed signal
    if ((addr == 881) && hyundai_ev_gas_signal) {
      gas_pressed = (((GET_BYTE(to_push, 4) & 0x7FU) << 1) | GET_BYTE(to_push, 3) >> 7) != 0U;
    } else if ((addr == 881) && hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BYTE(to_push, 7) != 0U;
    } else if (addr == 608) {  // ICE
      gas_pressed = (GET_BYTE(to_push, 7) >> 6) != 0U;
    } else {
    }

    // sample wheel speed, averaging opposite corners
    if (addr == 497) {    // 0x1F1
      int hyundai_speed = (GET_BYTES_04(to_push) >> 16) & 0xFFF;  // FL
      hyundai_speed += (GET_BYTES_48(to_push) >> 20) & 0xFFF;  // RR
      hyundai_speed *= 2;		// This was originally divided by 2 but this is hack for i30 12bit (vs 14bit) speed value comply with HYUNDAI_STANDSTILL_THRSLD
      vehicle_moving = hyundai_speed > HYUNDAI_STANDSTILL_THRSLD;
    }

    if (addr == 129) {    // 0x081
      brake_pressed = (GET_BYTE(to_push, 0) >> 7) != 0;
    }

    bool stock_ecu_detected = false;

    // If openpilot is controlling longitudinal we need to ensure the radar is turned off
    // Enforce by checking we don't see SCC12
    if (hyundai_longitudinal && (addr == 1057)) {
      stock_ecu_detected = true;
    }
    generic_rx_checks(stock_ecu_detected);
  }
  return valid;
}

static int hyundai_tx_hook(CANPacket_t *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);

  if (hyundai_longitudinal) {
    tx = msg_allowed(to_send, HYUNDAI_LONG_TX_MSGS, sizeof(HYUNDAI_LONG_TX_MSGS)/sizeof(HYUNDAI_LONG_TX_MSGS[0]));
  } else {
    tx = msg_allowed(to_send, HYUNDAI_TX_MSGS, sizeof(HYUNDAI_TX_MSGS)/sizeof(HYUNDAI_TX_MSGS[0]));
  }

  // FCA11: Block any potential actuation
  if (addr == 909) {
    int CR_VSM_DecCmd = GET_BYTE(to_send, 1);
    int FCA_CmdAct = (GET_BYTE(to_send, 2) >> 5) & 1U;
    int CF_VSM_DecCmdAct = (GET_BYTE(to_send, 3) >> 7) & 1U;

    if ((CR_VSM_DecCmd != 0) || (FCA_CmdAct != 0) || (CF_VSM_DecCmdAct != 0)) {
      tx = 0;
    }
  }

  // ACCEL: safety check
  if (addr == 1057) {
    int desired_accel_raw = (((GET_BYTE(to_send, 4) & 0x7U) << 8) | GET_BYTE(to_send, 3)) - 1023U;
    int desired_accel_val = ((GET_BYTE(to_send, 5) << 3) | (GET_BYTE(to_send, 4) >> 5)) - 1023U;

    int aeb_decel_cmd = GET_BYTE(to_send, 2);
    int aeb_req = (GET_BYTE(to_send, 6) >> 6) & 1U;

    bool violation = 0;

    if (!controls_allowed) {
      if ((desired_accel_raw != 0) || (desired_accel_val != 0)) {
        violation = 1;
      }
    }
    violation |= max_limit_check(desired_accel_raw, HYUNDAI_MAX_ACCEL, HYUNDAI_MIN_ACCEL);
    violation |= max_limit_check(desired_accel_val, HYUNDAI_MAX_ACCEL, HYUNDAI_MIN_ACCEL);

    violation |= (aeb_decel_cmd != 0);
    violation |= (aeb_req != 0);

    if (violation) {
      tx = 0;
    }
  }

  // LKA STEER: safety check
  if (addr == 832) {
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x7ffU) - 1024U;
    uint32_t ts = microsecond_timer_get();
    bool violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, HYUNDAI_MAX_STEER, -HYUNDAI_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        HYUNDAI_MAX_STEER, HYUNDAI_MAX_RATE_UP, HYUNDAI_MAX_RATE_DOWN,
        HYUNDAI_DRIVER_TORQUE_ALLOWANCE, HYUNDAI_DRIVER_TORQUE_FACTOR);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, HYUNDAI_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > HYUNDAI_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
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

  // UDS: Only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (addr == 2000) {
    if ((GET_BYTES_04(to_send) != 0x00803E02U) || (GET_BYTES_48(to_send) != 0x0U)) {
      tx = 0;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent (VAL 4) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((addr == 1265) && !controls_allowed) {
    if ((GET_BYTES_04(to_send) & 0x7U) != 4U) {
      tx = 0;
    }
  }

  // 1 allows the message through
  return tx;
}

static int hyundai_fwd_hook(int bus_num, CANPacket_t *to_fwd) {

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  // forward cam to ccan and viceversa, except lkas cmd
  if (bus_num == 0) {
    bus_fwd = 2;
  }
  if ((bus_num == 2) && (addr != 832) && (addr != 1157)) {
    bus_fwd = 0;
  }

  return bus_fwd;
}

static const addr_checks* hyundai_init(int16_t param) {
  controls_allowed = false;
  relay_malfunction_reset();

  hyundai_legacy = false;
  hyundai_ev_gas_signal = GET_FLAG(param, HYUNDAI_PARAM_EV_GAS);
  hyundai_hybrid_gas_signal = !hyundai_ev_gas_signal && GET_FLAG(param, HYUNDAI_PARAM_HYBRID_GAS);

#ifdef ALLOW_DEBUG
  hyundai_longitudinal = GET_FLAG(param, HYUNDAI_PARAM_LONGITUDINAL);
#endif

  if (hyundai_longitudinal) {
    hyundai_rx_checks = (addr_checks){hyundai_long_addr_checks, HYUNDAI_LONG_ADDR_CHECK_LEN};
  } else {
    hyundai_rx_checks = (addr_checks){hyundai_addr_checks, HYUNDAI_ADDR_CHECK_LEN};
  }
  return &hyundai_rx_checks;
}

static const addr_checks* hyundai_legacy_init(int16_t param) {
  controls_allowed = false;
  relay_malfunction_reset();

  hyundai_legacy = true;
  hyundai_longitudinal = false;
  hyundai_ev_gas_signal = GET_FLAG(param, HYUNDAI_PARAM_EV_GAS);
  hyundai_hybrid_gas_signal = !hyundai_ev_gas_signal && GET_FLAG(param, HYUNDAI_PARAM_HYBRID_GAS);
  hyundai_rx_checks = (addr_checks){hyundai_legacy_addr_checks, HYUNDAI_LEGACY_ADDR_CHECK_LEN};
  return &hyundai_rx_checks;
}

const safety_hooks hyundai_hooks = {
  .init = hyundai_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
};

const safety_hooks hyundai_legacy_hooks = {
  .init = hyundai_legacy_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_fwd_hook,
};
