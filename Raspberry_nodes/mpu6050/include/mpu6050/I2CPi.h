/**
 * @author  Dwindra Sulistyoutomo
 */

#ifndef _I2CPI_H
#define _I2CPI_H

#include <cstdint>
#include <wiringPiI2C.h>

// DEBUG libraries
// You need to uncomment some of the includes below to print for debugging.
// Commented in order 
// #include <iostream>     // main print function std::cout
// #include <iomanip>      // to print hex in two-digit. e.g 0x0A
// #include <bitset>       // to print values in binary

class I2CPi {
    public:
        // ---------- SETUP ----------
        /**
         * Setup I2C device file handler
         * 
         * @param dev_address {uint8_t} I2C device adress
         * 
         * @return {int} file handler using wiringPiI2CSetup
         */
        static int Setup(uint8_t dev_address);

        // ---------- READ ----------
        /**
         * Read 1 bit from the given register address
         * 
         * @param fd            {int}       I2C device file handler
         * @param reg_address   {uint8_t}   register address
         * @param bit_number    {uint8_t}   bit to read (0-7)
         * 
         * @return {uint8_t} value of the given bit of the address
         */
        static uint8_t ReadBit(int fd, uint8_t reg_address, uint8_t bit_number);

        /**
         * Read bits from the given register address 
         * 
         * @param fd            {int}       I2C device file handler
         * @param reg_address   {uint8_t}   register address
         * @param bit_start     {uint8_t}   start bit to read (0-7)
         * @param length        {uint8_t}   length of bits to read (<=8)
         * 
         * @return {uint8_t} value of the given bits of the address
         */
        static uint8_t ReadBits(int fd, uint8_t reg_address, uint8_t bit_start, uint8_t length);

        /**
         * Read a byte from the given register address 
         * 
         * @param fd            {int}       I2C device file handler
         * @param reg_address   {uint8_t}   register address
         * 
         * @return {uint8_t} value of the given byte of the address
         */
        static uint8_t ReadByte(int fd, uint8_t reg_address);

        /**
         * Read several bytes from the given register address 
         * 
         * @param fd            {int}       I2C device file handler
         * @param reg_address   {uint8_t}   register address
         * @param length        {uint8_t}   length of bytes to read
         * 
         * @return {uint8_t*} value of the given byte of the address
         */
        static uint8_t* ReadBytes(int fd, uint8_t reg_address, uint8_t length);

        /**
         * Read a word (2 bytes) from the given register address and its following address (reg_address+1) 
         * 
         * @param fd            {int}       I2C device file handler
         * @param reg_address   {uint8_t}   register address
         * 
         * @return {uint16_t} value of the given word of the address
         */
        static uint16_t ReadWord(int fd, uint8_t reg_address);

        // ---------- WRITE ----------
        /**
         * Write bit to the given register address 
         * 
         * @param fd            {int}       I2C device file handler
         * @param reg_address   {uint8_t}   register address
         * @param bit_number    {uint8_t}   bit to write (0-7)
         * @param data          {uint8_t}   data to write
         */
        static void WriteBit(int fd, uint8_t reg_address, uint8_t bit_number, uint8_t data);

        /**
         * Write bits to the given register address 
         * 
         * @param fd            {int}       I2C device file handler
         * @param reg_address   {uint8_t}   register address
         * @param bit_start     {uint8_t}   start bit to write (0-7)
         * @param length        {uint8_t}   length of bits to write (<=8)
         * @param data          {uint8_t}   data to write
         */
        static void WriteBits(int fd, uint8_t reg_address, uint8_t bit_start, uint8_t length, uint8_t data);
        /**
         * Write a byte to the given register address
         * 
         * @param fd            {int}       I2C device file handler
         * @param reg_address   {uint8_t}   register address
         * @param data          {uint8_t}   data to write
         */
        static void WriteByte(int fd, uint8_t reg_address, uint8_t data);

        /**
         * Write several bytes to the given register address 
         * 
         * @param fd            {int}       I2C device file handler
         * @param reg_address   {uint8_t}   register address
         * @param data          {uint8_t*}  array of byte data to write
         * @param length        {uint8_t}   length of bytes to write
         */
        static void WriteBytes(int fd, uint8_t reg_address, uint8_t *data, uint8_t length);

        /**
         * Write a word (2 bytes) to the given register address and its following address (reg_address+1)
         * 
         * @param fd            {int}       I2C device file handler
         * @param reg_address   {uint8_t}   register address
         * @param data          {uint8_t}   data to write
         */
        static void WriteWord(int fd, uint8_t reg_address, uint16_t data);
};

#endif