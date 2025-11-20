// config.hpp
#ifndef _CONFIG_HPP
#define _CONFIG_HPP

#include <Arduino.h>
#include <array>

// Thruster x 6
const uint8_t ESC_SIG_1 = 0;
const uint8_t ESC_SIG_2 = 1;
const uint8_t ESC_SIG_3 = 2;
const uint8_t ESC_SIG_4 = 3;
const uint8_t ESC_SIG_5 = 6;
const uint8_t ESC_SIG_6 = 7;

// Fron LED
const uint8_t  LED1 = 8;
const uint8_t  LED2 = 9;

// Relay Signal
const uint8_t RELAY_SIG = 28;

// GPIO Extentions
const uint8_t SIG11 = 10;
const uint8_t SIG12 = 11;
const uint8_t SIG13 = 12;
const uint8_t SIG14 = 13;
const uint8_t SIG21 = 14;
const uint8_t SIG22 = 15;
const uint8_t SIG23 = 26;
const uint8_t SIG24 = 27;

// GPIO (analog) Extention
const uint8_t SIG_SPARE = 29;

// I2C Extention
const uint8_t I2C_SDA = 4;
const uint8_t I2C_SCL = 5;

// On Board LED
const uint8_t ON_BOARD_LED = 16;

// Array
const std::array<uint8_t, 6> ESC_SIGS = {
    ESC_SIG_1, ESC_SIG_2, ESC_SIG_3, ESC_SIG_4, ESC_SIG_5, ESC_SIG_6};
const std::array<uint8_t, 2> LEDS = {LED1, LED2};
const std::array<uint8_t, 8> SIGS = {
    SIG11, SIG12, SIG13, SIG14, SIG21, SIG22, SIG23, SIG24};

#endif