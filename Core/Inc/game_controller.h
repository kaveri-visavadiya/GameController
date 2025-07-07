// game_controller.h

#include <stdint.h>

#ifndef GAME_CONTROLLER_H
#define GAME_CONTROLLER_H

typedef struct {
  uint16_t buttons;
  uint8_t left_x;
  uint8_t left_y;
  uint8_t right_x;
  uint8_t right_y;
  // uint8_t haptic_feedback;
} game_controller_report;

// Declare the variable as extern so other files know it exists
extern game_controller_report game_controller;

#endif // GAME_CONTROLLER_H
