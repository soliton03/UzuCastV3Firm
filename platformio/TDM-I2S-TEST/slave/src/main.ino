#include <Arduino.h>
#include <driver/i2s.h>

static constexpr uint32_t kSampleRate = 44100;

static const int RX_BCK = 32;
static const int RX_WS = 33;
static const int RX_DIN = 35;

static inline uint16_t tagU16(int16_t v) {
  return (uint16_t)v;
}

// --- Phase analyzer (sync = R=FFFF from Master) ---
static constexpr size_t kHistBins = 16;

static uint32_t g_phase_id = 0;
static bool g_in_sync = false;
static uint32_t g_sync_run = 0;
static bool g_phase_open = false;
static uint32_t g_ph_frames = 0;
static uint32_t g_ph_r_ffff = 0;
static uint32_t g_ph_r_aaaa = 0;
static uint32_t g_ph_r_aa55 = 0;
static uint32_t g_ph_r_5500 = 0;
static uint32_t g_ph_r_aa00 = 0;
static uint32_t g_ph_r_zero = 0;
static uint32_t g_ph_r_other = 0;
static uint32_t g_ph_l_hist[kHistBins];
static uint16_t g_ph_l_hist_base = 0;
static uint32_t g_ph_l_plus1 = 0;
static uint32_t g_ph_l_minus1 = 0;
static uint32_t g_ph_l_match_mode = 0;
static uint16_t g_ph_l_mode = 0;
static uint32_t g_ph_l_mode_count = 0;
static uint32_t g_ph_counter_ok = 0;
static uint32_t g_ph_counter_off1 = 0;
static uint32_t g_ph_counter_other = 0;
static uint16_t g_ph_counter_exp = 0x1000;
static int16_t g_last_l = 0;
static int16_t g_last_r = 0;
static bool g_counter_phase = false;

// --- DATA_CHECK: fixed pattern (5555/AAAA) ---
static uint32_t g_fix_frames = 0;
static uint32_t g_fix_r_aaaa = 0;
static uint32_t g_fix_l_5555 = 0;
static uint32_t g_fix_l_5554 = 0;
static uint32_t g_fix_l_other = 0;
static uint32_t g_fix_report_seq = 0;

// --- DATA_CHECK: UZC period (AA00 + 159x AA55 + 5500) ---
static uint32_t g_uzc_period_id = 0;
static bool g_uzc_open = false;
static bool g_uzc_saw_aa00 = false;
static uint32_t g_uzc_aa55 = 0;
static uint32_t g_uzc_l_ok = 0;
static uint32_t g_uzc_l_off1 = 0;
static uint32_t g_uzc_l_bad = 0;
static uint16_t g_uzc_exp_l = 1;
static uint16_t g_uzc_end_r = 0;
static uint16_t g_uzc_end_l = 0;
static uint32_t g_uzc_complete = 0;
static uint32_t g_uzc_bad_end = 0;

static void resetPhaseStats() {
  g_ph_frames = 0;
  g_ph_r_ffff = g_ph_r_aaaa = g_ph_r_aa55 = 0;
  g_ph_r_5500 = g_ph_r_aa00 = g_ph_r_zero = g_ph_r_other = 0;
  for (size_t i = 0; i < kHistBins; i++) {
    g_ph_l_hist[i] = 0;
  }
  g_ph_l_hist_base = 0;
  g_ph_l_plus1 = g_ph_l_minus1 = g_ph_l_match_mode = 0;
  g_ph_l_mode = 0;
  g_ph_l_mode_count = 0;
  g_ph_counter_ok = g_ph_counter_off1 = g_ph_counter_other = 0;
  g_ph_counter_exp = 0x1000;
  g_counter_phase = false;
}

static void updateLHist(uint16_t l) {
  if (g_ph_frames == 1) {
    g_ph_l_hist_base = l;
    g_ph_l_mode = l;
    g_ph_l_mode_count = 1;
  }
  if (l == g_ph_l_mode) {
    g_ph_l_mode_count++;
  }
  const int16_t idx = (int16_t)l - (int16_t)g_ph_l_hist_base;
  if (idx >= 0 && idx < (int16_t)kHistBins) {
    g_ph_l_hist[(size_t)idx]++;
    if (g_ph_l_hist[(size_t)idx] > g_ph_l_mode_count) {
      g_ph_l_mode = (uint16_t)(g_ph_l_hist_base + idx);
      g_ph_l_mode_count = g_ph_l_hist[(size_t)idx];
    }
  }
}

static void printPhaseReport() {
  if (g_ph_frames == 0) {
    return;
  }
  uint32_t dom = g_ph_r_aaaa;
  const char* dom_name = "AAAA";
  if (g_ph_r_aa55 > dom) {
    dom = g_ph_r_aa55;
    dom_name = "AA55";
  }
  if (g_ph_r_5500 > dom) {
    dom = g_ph_r_5500;
    dom_name = "5500";
  }
  if (g_ph_r_aa00 > dom) {
    dom = g_ph_r_aa00;
    dom_name = "AA00";
  }
  if (g_ph_r_zero > dom) {
    dom = g_ph_r_zero;
    dom_name = "0000";
  }
  Serial.printf(
      "ANALYSIS phase=%lu frames=%lu domR=%s R:AAAA=%lu AA55=%lu 5500=%lu AA00=%lu other=%lu | "
      "L_mode=%04X cnt=%lu L[+1]=%lu L[-1]=%lu | CNT ok=%lu off1=%lu bad=%lu\n",
      (unsigned long)g_phase_id, (unsigned long)g_ph_frames, dom_name, (unsigned long)g_ph_r_aaaa,
      (unsigned long)g_ph_r_aa55, (unsigned long)g_ph_r_5500, (unsigned long)g_ph_r_aa00,
      (unsigned long)g_ph_r_other,
      (unsigned)g_ph_l_mode, (unsigned long)g_ph_l_mode_count, (unsigned long)g_ph_l_plus1,
      (unsigned long)g_ph_l_minus1, (unsigned long)g_ph_counter_ok, (unsigned long)g_ph_counter_off1,
      (unsigned long)g_ph_counter_other);
}

static void closePhase() {
  if (!g_phase_open) {
    return;
  }
  printPhaseReport();
  g_phase_open = false;
  g_phase_id++;
}

static void openPhase() {
  closePhase();
  resetPhaseStats();
  g_phase_open = true;
}

static void resetUzcPeriod() {
  g_uzc_open = true;
  g_uzc_saw_aa00 = false;
  g_uzc_aa55 = 0;
  g_uzc_l_ok = g_uzc_l_off1 = g_uzc_l_bad = 0;
  g_uzc_exp_l = 1;
  g_uzc_end_r = 0;
  g_uzc_end_l = 0;
}

static void printUzcPeriodReport() {
  g_uzc_period_id++;
  const bool end_ok = (g_uzc_end_r == 0x5500);
  if (end_ok) {
    g_uzc_complete++;
  } else {
    g_uzc_bad_end++;
  }
  Serial.printf(
      "DATA_CHECK UZC period=%lu aa00=%u aa55=%lu (exp159) L_ok=%lu L_off1=%lu L_bad=%lu "
      "endR=%04X endL=%04X end_ok=%u\n",
      (unsigned long)g_uzc_period_id, (unsigned)g_uzc_saw_aa00, (unsigned long)g_uzc_aa55,
      (unsigned long)g_uzc_l_ok, (unsigned long)g_uzc_l_off1, (unsigned long)g_uzc_l_bad,
      (unsigned)g_uzc_end_r, (unsigned)g_uzc_end_l, (unsigned)end_ok);
  g_uzc_open = false;
}

static void dataCheckFrame(int16_t l, int16_t r) {
  const uint16_t lu = tagU16(l);
  const uint16_t ru = tagU16(r);

  if (ru == 0xAAAA) {
    g_fix_frames++;
    g_fix_r_aaaa++;
    if (lu == 0x5555) {
      g_fix_l_5555++;
    } else if (lu == 0x5554) {
      g_fix_l_5554++;
    } else {
      g_fix_l_other++;
    }
    if (g_fix_frames >= 4410) {
      g_fix_report_seq++;
      Serial.printf(
          "DATA_CHECK FIXED seq=%lu frames=%lu R_AAAA=%lu L_5555=%lu L_5554=%lu L_other=%lu\n",
          (unsigned long)g_fix_report_seq, (unsigned long)g_fix_frames, (unsigned long)g_fix_r_aaaa,
          (unsigned long)g_fix_l_5555, (unsigned long)g_fix_l_5554, (unsigned long)g_fix_l_other);
      g_fix_frames = g_fix_r_aaaa = g_fix_l_5555 = g_fix_l_5554 = g_fix_l_other = 0;
    }
  }

  if (ru == 0xAA00) {
    if (g_uzc_open && g_uzc_saw_aa00) {
      printUzcPeriodReport();
    }
    resetUzcPeriod();
    g_uzc_saw_aa00 = true;
    return;
  }

  if (ru == 0x5500) {
    if (g_uzc_open) {
      g_uzc_end_r = ru;
      g_uzc_end_l = lu;
      printUzcPeriodReport();
    }
    return;
  }

  if (ru == 0xAA55 && g_uzc_open) {
    g_uzc_aa55++;
    if (lu == g_uzc_exp_l) {
      g_uzc_l_ok++;
    } else if (lu == (uint16_t)(g_uzc_exp_l - 1)) {
      g_uzc_l_off1++;
    } else {
      g_uzc_l_bad++;
    }
    if (g_uzc_exp_l < 159) {
      g_uzc_exp_l++;
    }
  }
}

static void analyzeFrame(int16_t l, int16_t r) {
  const uint16_t lu = tagU16(l);
  const uint16_t ru = tagU16(r);
  g_last_l = l;
  g_last_r = r;
  dataCheckFrame(l, r);

  if (ru == 0xFFFF) {
    g_sync_run++;
    if (g_sync_run >= 24 && g_phase_open && g_ph_frames > 256) {
      closePhase();
    }
    g_in_sync = true;
    return;
  }

  if (g_in_sync && g_sync_run >= 8) {
    g_in_sync = false;
    g_sync_run = 0;
    openPhase();
  } else if (!g_phase_open) {
    g_sync_run = 0;
    openPhase();
  } else {
    g_sync_run = 0;
  }

  g_ph_frames++;
  switch (ru) {
    case 0xAAAA:
      g_ph_r_aaaa++;
      break;
    case 0xAA55:
      g_ph_r_aa55++;
      break;
    case 0x5500:
      g_ph_r_5500++;
      break;
    case 0xAA00:
      g_ph_r_aa00++;
      break;
    case 0x0000:
      g_ph_r_zero++;
      break;
    default:
      g_ph_r_other++;
      break;
  }

  updateLHist(lu);

  if (g_ph_l_mode_count > 32 && lu == (uint16_t)(g_ph_l_mode + 1)) {
    g_ph_l_plus1++;
  }
  if (g_ph_l_mode_count > 32 && lu == (uint16_t)(g_ph_l_mode - 1)) {
    g_ph_l_minus1++;
  }
  if (lu == g_ph_l_mode) {
    g_ph_l_match_mode++;
  }

  if (ru == 0xAAAA && lu >= 0x1000 && lu <= 0x10FF) {
    g_counter_phase = true;
    if (lu == g_ph_counter_exp) {
      g_ph_counter_ok++;
    } else if (lu == (uint16_t)(g_ph_counter_exp + 1)) {
      g_ph_counter_off1++;
    } else {
      g_ph_counter_other++;
    }
    g_ph_counter_exp = (uint16_t)(0x1000 + ((g_ph_counter_exp - 0x1000 + 1) & 0xFF));
  }
}

static bool i2s_rx_init_slave_stereo_16() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX);
  cfg.sample_rate = kSampleRate;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = RX_BCK;
  pins.ws_io_num = RX_WS;
  pins.data_in_num = RX_DIN;
  pins.data_out_num = I2S_PIN_NO_CHANGE;

  if (i2s_driver_install(I2S_NUM_1, &cfg, 0, nullptr) != ESP_OK) {
    return false;
  }
  if (i2s_set_pin(I2S_NUM_1, &pins) != ESP_OK) {
    return false;
  }
  return i2s_set_clk(I2S_NUM_1, kSampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO) ==
         ESP_OK;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\nTDM-I2S-TEST Slave analyzer"));
  Serial.println(F("Sync=R=FFFF between phases; DATA_CHECK for fixed/UZC"));
  if (!i2s_rx_init_slave_stereo_16()) {
    Serial.println(F("I2S RX init FAILED"));
    while (true) {
      delay(1000);
    }
  }
  resetPhaseStats();
  Serial.println(F("Listening..."));
}

void loop() {
  for (int burst = 0; burst < 64; burst++) {
    static int16_t tmp[128 * 2];
    size_t rbytes = 0;
    esp_err_t e = i2s_read(I2S_NUM_1, tmp, sizeof(tmp), &rbytes, 0);
    if (e != ESP_OK || rbytes == 0) {
      break;
    }
    const uint32_t frames = (uint32_t)(rbytes / (sizeof(int16_t) * 2));
    for (uint32_t i = 0; i < frames; i++) {
      analyzeFrame(tmp[i * 2 + 0], tmp[i * 2 + 1]);
    }
  }
}
