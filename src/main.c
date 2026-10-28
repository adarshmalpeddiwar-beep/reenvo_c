#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define MIN_STEP_RATE 50
#define MAX_STEP_RATE 20000
#define START_STEP_RATE 200
#define DEFAULT_STEP_RATE 100
#define MIN_ACCEL 100
#define MAX_ACCEL 200000
#define DEFAULT_ACCEL 8000
#define MAX_JOG_DURATION_MS 5000
#define CONTROL_PERIOD_MS 10

typedef struct {
  const char *name;
  gpio_num_t step_pin;
  gpio_num_t dir_pin;
  ledc_timer_t timer;
  ledc_channel_t channel;
  bool running;
  int direction;          // level currently on the DIR pin
  int target_direction;   // requested direction; reached after slowing down
  uint32_t step_rate;     // pulse rate currently being output
  uint32_t target_rate;   // requested pulse rate; reached by ramping
  uint32_t accel;         // steps/s^2
  int64_t stop_at_us;
} motor_t;

static motor_t motors[] = {
    {.name = "T86S", .step_pin = GPIO_NUM_6, .dir_pin = GPIO_NUM_7,
     .timer = LEDC_TIMER_0, .channel = LEDC_CHANNEL_0,
     .direction = 1, .accel = DEFAULT_ACCEL},
    {.name = "NEMA17", .step_pin = GPIO_NUM_14, .dir_pin = GPIO_NUM_19,
     .timer = LEDC_TIMER_1, .channel = LEDC_CHANNEL_1,
     .direction = 1, .accel = DEFAULT_ACCEL},
};

static const gpio_num_t sensor_pins[] = {GPIO_NUM_0, GPIO_NUM_1};
static const gpio_num_t ssr_pin = GPIO_NUM_18;
static bool ssr_on;
static SemaphoreHandle_t control_mutex;

static void stop_motor(motor_t *motor) {
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel));
  motor->running = false;
  motor->step_rate = 0;
  motor->stop_at_us = 0;
}

static bool set_step_rate(motor_t *motor, uint32_t rate) {
  if (ledc_set_freq(LEDC_LOW_SPEED_MODE, motor->timer, rate) != ESP_OK) {
    stop_motor(motor);
    printf("%s stopped: cannot generate %lu steps/s.\n", motor->name,
           (unsigned long)rate);
    return false;
  }
  motor->step_rate = rate;
  return true;
}

// Starts the motor, or retargets it if it is already running. A running motor
// keeps its current speed and ramps toward the new target; a direction change
// is applied by the control task once the motor has slowed to START_STEP_RATE.
// Returns true if the motor was started or its direction/speed changed.
static bool start_motor(motor_t *motor, int direction, uint32_t step_rate,
                        uint32_t duration_ms) {
  const bool changed = !motor->running ||
                       motor->target_direction != direction ||
                       motor->target_rate != step_rate;
  motor->target_direction = direction;
  motor->target_rate = step_rate;
  motor->stop_at_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;
  if (motor->running) {
    return changed;
  }

  motor->direction = direction;
  ESP_ERROR_CHECK(gpio_set_level(motor->dir_pin, motor->direction));
  esp_rom_delay_us(20);  // DIR setup time before the first STEP edge
  const uint32_t first_rate =
      step_rate < START_STEP_RATE ? step_rate : START_STEP_RATE;
  if (!set_step_rate(motor, first_rate)) {
    return false;
  }
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, 512));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel));
  motor->running = true;
  return true;
}

static void update_motor(motor_t *motor, int64_t elapsed_us) {
  uint32_t goal = motor->target_rate;
  if (motor->target_direction != motor->direction) {
    if (motor->step_rate <= START_STEP_RATE) {
      motor->direction = motor->target_direction;
      ESP_ERROR_CHECK(gpio_set_level(motor->dir_pin, motor->direction));
    } else if (goal > START_STEP_RATE) {
      goal = START_STEP_RATE;
    }
  }

  int64_t step = (int64_t)motor->accel * elapsed_us / 1000000;
  if (step < 1) {
    step = 1;
  }
  int64_t rate = motor->step_rate;
  if (rate < goal) {
    rate = rate + step > goal ? goal : rate + step;
  } else if (rate > goal) {
    rate = rate - step < goal ? goal : rate - step;
  }
  if (rate < MIN_STEP_RATE) {
    rate = MIN_STEP_RATE;
  }
  if ((uint32_t)rate != motor->step_rate) {
    set_step_rate(motor, (uint32_t)rate);
  }
}

static void print_help(void) {
  printf("\nCommands:\n"
         "  status                                  Show inputs and outputs\n"
         "  jog <axis 1|2> <f|r> <ms> [steps/s]    Bounded motor jog\n"
         "  accel <axis 1|2> <steps/s^2>            Set acceleration\n"
         "  stop [1|2|all]                          Stop motor(s)\n"
         "  ssr on|off                              Set SSR output\n"
         "  help                                    Show this help\n"
         "\nJog duration: 1-%d ms; speed: %d-%d steps/s (default %d).\n"
         "Acceleration: %d-%d steps/s^2 (default %d).\n"
         "Sending jog again before it ends extends it, so a client can run a\n"
         "motor continuously by repeating the command.\n"
         "Axis 1 = T86S, axis 2 = NEMA17 driver.\n\n",
         MAX_JOG_DURATION_MS, MIN_STEP_RATE, MAX_STEP_RATE, DEFAULT_STEP_RATE,
         MIN_ACCEL, MAX_ACCEL, DEFAULT_ACCEL);
}

static void print_status(void) {
  if (xSemaphoreTake(control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    printf("Status unavailable: controller busy.\n");
    return;
  }

  printf("Sensors: A0/GPIO0=%s  A1/GPIO1=%s | "
         "Limits: A2/GPIO2=%s  A3/GPIO3=%s\n",
         gpio_get_level(sensor_pins[0]) ? "HIGH" : "LOW",
         gpio_get_level(sensor_pins[1]) ? "HIGH" : "LOW",
         gpio_get_level(GPIO_NUM_2) ? "TRIPPED/HIGH" : "normal/LOW",
         gpio_get_level(GPIO_NUM_3) ? "TRIPPED/HIGH" : "normal/LOW");
  printf("Motors: 1=%s  2=%s | SSR=%s | Rate: 1=%lu 2=%lu | Dir: 1=%c 2=%c\n",
         motors[0].running ? "running" : "stopped",
         motors[1].running ? "running" : "stopped", ssr_on ? "ON" : "OFF",
         (unsigned long)motors[0].step_rate, (unsigned long)motors[1].step_rate,
         motors[0].direction ? 'f' : 'r', motors[1].direction ? 'f' : 'r');
  xSemaphoreGive(control_mutex);
}

static bool parse_integer(const char *text, long min, long max, long *value) {
  char *end = NULL;
  const long parsed = strtol(text, &end, 10);
  if (text[0] == '\0' || end == text || *end != '\0' || parsed < min ||
      parsed > max) {
    return false;
  }
  *value = parsed;
  return true;
}

static void handle_command(char *line) {
  char *save = NULL;
  char *command = strtok_r(line, " \t\r\n", &save);
  if (command == NULL) {
    return;
  }

  if (strcmp(command, "help") == 0) {
    print_help();
    return;
  }
  if (strcmp(command, "status") == 0) {
    print_status();
    return;
  }

  if (strcmp(command, "jog") == 0) {
    char *axis_text = strtok_r(NULL, " \t\r\n", &save);
    char *direction_text = strtok_r(NULL, " \t\r\n", &save);
    char *duration_text = strtok_r(NULL, " \t\r\n", &save);
    char *speed_text = strtok_r(NULL, " \t\r\n", &save);
    char *extra = strtok_r(NULL, " \t\r\n", &save);
    long axis, duration, speed = DEFAULT_STEP_RATE;
    if (axis_text == NULL || direction_text == NULL || duration_text == NULL ||
        extra != NULL || !parse_integer(axis_text, 1, 2, &axis) ||
        !parse_integer(duration_text, 1, MAX_JOG_DURATION_MS, &duration) ||
        (speed_text != NULL &&
         !parse_integer(speed_text, MIN_STEP_RATE, MAX_STEP_RATE, &speed)) ||
        (strcmp(direction_text, "f") != 0 && strcmp(direction_text, "r") != 0)) {
      printf("Invalid jog command. Example: jog 1 f 500 100\n");
      return;
    }

    motor_t *motor = &motors[axis - 1];
    if (xSemaphoreTake(control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
      printf("Jog rejected: controller busy.\n");
      return;
    }
    const bool was_running = motor->running;
    const bool changed = start_motor(motor, strcmp(direction_text, "f") == 0,
                                     (uint32_t)speed, (uint32_t)duration);
    xSemaphoreGive(control_mutex);
    if (changed) {
      printf("Axis %ld jog %s: %s at %ld steps/s for %ld ms.\n", axis,
             was_running ? "updated" : "started", direction_text, speed,
             duration);
    }
    return;
  }

  if (strcmp(command, "accel") == 0) {
    char *axis_text = strtok_r(NULL, " \t\r\n", &save);
    char *accel_text = strtok_r(NULL, " \t\r\n", &save);
    char *extra = strtok_r(NULL, " \t\r\n", &save);
    long axis, accel;
    if (axis_text == NULL || accel_text == NULL || extra != NULL ||
        !parse_integer(axis_text, 1, 2, &axis) ||
        !parse_integer(accel_text, MIN_ACCEL, MAX_ACCEL, &accel)) {
      printf("Usage: accel <1|2> <%d-%d>\n", MIN_ACCEL, MAX_ACCEL);
      return;
    }
    if (xSemaphoreTake(control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
      printf("Accel rejected: controller busy.\n");
      return;
    }
    motors[axis - 1].accel = (uint32_t)accel;
    xSemaphoreGive(control_mutex);
    printf("Axis %ld acceleration set to %ld steps/s^2.\n", axis, accel);
    return;
  }

  if (strcmp(command, "stop") == 0) {
    char *axis_text = strtok_r(NULL, " \t\r\n", &save);
    char *extra = strtok_r(NULL, " \t\r\n", &save);
    if (extra != NULL ||
        (axis_text != NULL && strcmp(axis_text, "all") != 0 &&
         strcmp(axis_text, "1") != 0 && strcmp(axis_text, "2") != 0)) {
      printf("Usage: stop [1|2|all]\n");
      return;
    }
    if (xSemaphoreTake(control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
      printf("Stop rejected: controller busy.\n");
      return;
    }
    if (axis_text == NULL || strcmp(axis_text, "all") == 0) {
      stop_motor(&motors[0]);
      stop_motor(&motors[1]);
      printf("Both motors stopped.\n");
    } else {
      const int axis = atoi(axis_text);
      stop_motor(&motors[axis - 1]);
      printf("Axis %d stopped.\n", axis);
    }
    xSemaphoreGive(control_mutex);
    return;
  }

  if (strcmp(command, "ssr") == 0) {
    char *state = strtok_r(NULL, " \t\r\n", &save);
    char *extra = strtok_r(NULL, " \t\r\n", &save);
    if (state == NULL || extra != NULL ||
        (strcmp(state, "on") != 0 && strcmp(state, "off") != 0)) {
      printf("Usage: ssr on|off\n");
      return;
    }
    if (xSemaphoreTake(control_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
      printf("SSR command rejected: controller busy.\n");
      return;
    }
    ssr_on = strcmp(state, "on") == 0;
    ESP_ERROR_CHECK(gpio_set_level(ssr_pin, ssr_on));
    xSemaphoreGive(control_mutex);
    printf("SSR %s.\n", ssr_on ? "ON" : "OFF");
    return;
  }

  printf("Unknown command: %s. Type 'help'.\n", command);
}

static void configure_gpio(void) {
  const uint64_t input_mask = (1ULL << GPIO_NUM_0) | (1ULL << GPIO_NUM_1) |
                              (1ULL << GPIO_NUM_2) | (1ULL << GPIO_NUM_3);
  gpio_config_t input_config = {
      .pin_bit_mask = input_mask,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&input_config));

  gpio_config_t output_config = {
      .pin_bit_mask = (1ULL << GPIO_NUM_6) | (1ULL << GPIO_NUM_7) |
                      (1ULL << GPIO_NUM_14) | (1ULL << GPIO_NUM_19) |
                      (1ULL << GPIO_NUM_18),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&output_config));
  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_6, 0));
  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_7, 0));
  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_14, 0));
  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_19, 0));
  ESP_ERROR_CHECK(gpio_set_level(ssr_pin, 0));

  for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = motors[i].timer,
        .freq_hz = DEFAULT_STEP_RATE,
        // The 40 MHz crystal covers the whole MIN..MAX_STEP_RATE range at
        // 10-bit resolution; the 80 MHz PLL cannot divide down to 50 Hz.
        .clk_cfg = LEDC_USE_XTAL_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
    ledc_channel_config_t channel_config = {
        .gpio_num = motors[i].step_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = motors[i].channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = motors[i].timer,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
  }
}

// Without the driver, stdin on the USB Serial/JTAG console is non-blocking and
// fgets() returns NULL as soon as no input is pending.
static void configure_console(void) {
  setvbuf(stdin, NULL, _IONBF, 0);
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
  usb_serial_jtag_driver_config_t usb_config =
      USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));
  usb_serial_jtag_vfs_use_driver();
}

static void safety_task(void *arg) {
  (void)arg;
  int64_t last_us = esp_timer_get_time();
  while (true) {
    if (xSemaphoreTake(control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      const int64_t now_us = esp_timer_get_time();
      for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        if (!motors[i].running) {
          continue;
        }
        if (now_us >= motors[i].stop_at_us) {
          stop_motor(&motors[i]);
          printf("%s jog complete.\n", motors[i].name);
        } else {
          update_motor(&motors[i], now_us - last_us);
        }
      }
      last_us = now_us;
      xSemaphoreGive(control_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

static void input_monitor_task(void *arg) {
  (void)arg;
  int previous[4] = {-1, -1, -1, -1};
  const gpio_num_t pins[] = {GPIO_NUM_0, GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3};
  const char *labels[] = {"Sensor A0/GPIO0", "Sensor A1/GPIO1",
                          "Limit A2/GPIO2", "Limit A3/GPIO3"};

  while (true) {
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
      const int level = gpio_get_level(pins[i]);
      if (level != previous[i]) {
        printf("%s: %s%s\n", labels[i],
               i >= 2 && level ? "TRIPPED/" : "",
               level ? "HIGH" : "LOW");
        previous[i] = level;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void app_main(void) {
  control_mutex = xSemaphoreCreateMutex();
  if (control_mutex == NULL) {
    ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
  }

  configure_gpio();
  configure_console();
  if (xTaskCreate(safety_task, "motor_safety", 3072, NULL, 5, NULL) != pdPASS ||
      xTaskCreate(input_monitor_task, "input_monitor", 3072, NULL, 4, NULL) !=
          pdPASS) {
    ESP_ERROR_CHECK(ESP_FAIL);
  }

  printf("\nGlyph C6 wired USB control ready.\n");
  printf("Connect the USB cable and open a serial terminal at 115200 baud.\n");
  printf("Type 'help' for commands. Sensor/limit changes print automatically.\n");
  print_help();

  char line[128];
  while (fgets(line, sizeof(line), stdin) != NULL) {
    handle_command(line);
  }
  printf("USB serial input closed; stopping motors and turning SSR off.\n");
  if (xSemaphoreTake(control_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    stop_motor(&motors[0]);
    stop_motor(&motors[1]);
    ssr_on = false;
    ESP_ERROR_CHECK(gpio_set_level(ssr_pin, 0));
    xSemaphoreGive(control_mutex);
  }
  vTaskDelete(NULL);
}
