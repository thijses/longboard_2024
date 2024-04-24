/*
this file just holds some fun tunes, blinking-patterns and that sort of stuff for user feedback.

*/


// #pragma once  // i'll just use the ifndef method
#ifndef USER_FEEDBACK_H
#define USER_FEEDBACK_H


#include "thijsUIpulses.h"
#ifdef IS_TRANSMITTER
  const float buzzerDutyVolumeDefault = 0.05;
  
  const float buzzerDutyVolume_ready = buzzerDutyVolumeDefault;
  UIpulse buzzer_readyPulse[] = {
    // pacman
//    {37, 0.9, 622, buzzerDutyVolume_ready},
//    {37, 0.9, 659, buzzerDutyVolume_ready},
//    {37, 0.9, 698, buzzerDutyVolume_ready},
//    {37, 0.9, 698, buzzerDutyVolume_ready},
//    {37, 0.9, 740, buzzerDutyVolume_ready},
//    {37, 0.9, 784, buzzerDutyVolume_ready},
//    {37, 0.9, 784, buzzerDutyVolume_ready},
//    {37, 0.9, 831, buzzerDutyVolume_ready},
//    {75, 0.9, 880, buzzerDutyVolume_ready},
//    {150, 0.9, 988, buzzerDutyVolume_ready},
    // sonic (3x speed)
    {50, 0.9, 587, buzzerDutyVolume_ready},
    {100, 0.9, 494, buzzerDutyVolume_ready},
    {50, 0.9, 587, buzzerDutyVolume_ready},
    {100, 0.9, 554, buzzerDutyVolume_ready},
    {50, 0.9, 587, buzzerDutyVolume_ready},
    {100, 0.9, 554, buzzerDutyVolume_ready},
    {250, 0.72, 440, buzzerDutyVolume_ready},
  };
#else // is board
  const float buzzerDutyVolumeDefault = 0.05;
  
  const float buzzerDutyVolume_ready = buzzerDutyVolumeDefault;
  UIpulse buzzer_readyPulse[] = {
    // pacman
//    {37, 0.9, 622, buzzerDutyVolume_ready},
//    {37, 0.9, 659, buzzerDutyVolume_ready},
//    {37, 0.9, 698, buzzerDutyVolume_ready},
//    {37, 0.9, 698, buzzerDutyVolume_ready},
//    {37, 0.9, 740, buzzerDutyVolume_ready},
//    {37, 0.9, 784, buzzerDutyVolume_ready},
//    {37, 0.9, 784, buzzerDutyVolume_ready},
//    {37, 0.9, 831, buzzerDutyVolume_ready},
//    {75, 0.9, 880, buzzerDutyVolume_ready},
//    {150, 0.9, 988, buzzerDutyVolume_ready},
    // sonic (3x speed)
    {50, 0.9, 587, buzzerDutyVolume_ready},
    {100, 0.9, 494, buzzerDutyVolume_ready},
    {50, 0.9, 587, buzzerDutyVolume_ready},
    {100, 0.9, 554, buzzerDutyVolume_ready},
    {50, 0.9, 587, buzzerDutyVolume_ready},
    {100, 0.9, 554, buzzerDutyVolume_ready},
    {250, 0.72, 440, buzzerDutyVolume_ready},
  };
#endif // is board/transmitter

#endif // USER_FEEDBACK_H
