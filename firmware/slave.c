/**
 * @file    slave.c
 * @author  Daniel Lockhead
 * @date    2024
 *
 * @brief   Main loop to control the Pico Slave Device
 *
 * @details The Slave Pico are the secondary controller for the InterconnectIO Box.
 * The slave accept a byte command followed by a byte data.
 *
 *
 * @copyright Copyright (c) 2024, D.Lockhead. All rights reserved.
 *
 * This software is licensed under the BSD 3-Clause License.
 * See the LICENSE file for more details.
 */

#include <i2c_fifo.h>
#include <i2c_slave.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "hardware/watchdog.h"
#include "userconfig.h"

static const uint I2C_OFFSET_ADDRESS = 0x20;  // ofsset to add to the physical address read
static const uint PICO_PORT_ADDRESS = 0x21;   // Pico address where port is used
static const uint REG_STATUS = 100;           // Register used to report Status

static const uint I2C_BAUDRATE = 100000;       // 100 kHz
static const uint I2C_SLAVE_ADDRESS_IO0 = 26;  // Bit 0 of I2C Address
static const uint I2C_SLAVE_ADDRESS_IO1 = 27;  // Bit 1 of I2C Address

// For this example, we run both the master and slave from the same board.
// You'll need to wire pin GP4 to GP6 (SDA), and pin GP5 to GP7 (SCL).
static const uint I2C_SLAVE_SDA_PIN = 20;  // PICO_DEFAULT_I2C_SDA_PIN; // 4
static const uint I2C_SLAVE_SCL_PIN = 21;  // PICO_DEFAULT_I2C_SCL_PIN; // 5

static const uint32_t GPIO_BOOT_MASK = 0b00011100010011111111111111111111;
static const uint32_t PORT0_MASK = 0xfful;
static const uint32_t PORT1_MASK = 0b111111110000000000;
static const uint PORT1_OFFSET = 10;  // shift to do for match mask

// static const uint32_t GPIO_SET_DIR_MASK =  0b0001000001111111111111111111111;  // GPIO MASK
static const uint32_t GPIO_SET_DIR_MASK = 0b0011110001111111111111111111111;    // GPIO MASK
static const uint32_t GPIO_SLV1_DIR_MASK = 0b00010000000000000000000000000000;  // IO Slave
static const uint32_t GPIO_SLV1_OUT_MASK = 0x00ul;                              // All output to 0
static const uint32_t GPIO_SLV2_DIR_MASK = 0b0011110001111111111111111111111;   // Relay Slave, all
static const uint32_t GPIO_SLV2_OUT_MASK = 0x00ul;                              // All output to 0

static const uint32_t GPIO_BANK0_MASK = 0xfful;  // Bank 0
static const uint32_t GPIO_BANK1_MASK = 0b00000000000000111111110000000000;
;  // Bank 1

//#define PICO_DEFAULT_UART_TX_PIN 8  // if Serial enabled
//#define PICO_DEFAULT_UART_RX_PIN 9

/*
Portable array-based cyclic FIFO queue.
Copy from Internet
*/

/**
 * @brief Queue implementation for MESSAGE structures.
 */
#define MESSAGE_SIZE 64 /**< Size of each message. */
#define QUEUE_SIZE 64   /**< Size of the queue (set high for development). */

/**
 * @brief Structure representing a message.
 */
typedef struct
{
  char data[MESSAGE_SIZE]; /**< Data contained in the message. */
} MESSAGE;

/**
* @brief Global queue structure.
*/
  struct
  {
    MESSAGE messages[QUEUE_SIZE];  /// Array of messages.
    int begin;                     /// Index of the first message.
    int end;                       /// Index of the last message.
    int current_load;              /// Current number of messages in the queue.
  } queue;

  /**
   * @brief Enqueue a message into the queue.
   *
   * @param message Pointer to the message to be enqueued.
   * @return true if the message was successfully enqueued, false if the queue is full.
   */
  bool enque(MESSAGE* message)
  {
    if (queue.current_load < QUEUE_SIZE)
    {
      if (queue.end == QUEUE_SIZE)
      {
        queue.end = 0;
      }
      queue.messages[queue.end] = *message;  /// Add the message to the queue.
      queue.end++;
      queue.current_load++;
      return true;
    }
    else
    {
      return false;
    }
  }

  /**
   * @brief Initialize the queue.
   */
  void init_queue()
  {
    queue.begin = 0;                                                   /// Initialize the beginning index.
    queue.end = 0;                                                     /// Initialize the end index.
    queue.current_load = 0;                                            /// Initialize the current load.
    memset(&queue.messages[0], 0, QUEUE_SIZE * sizeof(MESSAGE_SIZE));  /// Clear the queue.
  }

  /**
   * @brief Dequeue a message from the queue.
   *
   * @param message Pointer to the message to be dequeued.
   * @return true if the message was successfully dequeued, false if the queue is empty.
   */
  bool deque(MESSAGE* message)
  {
    if (queue.current_load > 0)
    {
      *message = queue.messages[queue.begin];                    /// Retrieve the message from the queue.
      memset(&queue.messages[queue.begin], 0, sizeof(MESSAGE));  /// Clear the dequeued message.
      queue.begin = (queue.begin + 1) % QUEUE_SIZE;              /// Update the beginning index.
      queue.current_load--;
      return true;
    }
    else
    {
      return false;
    }
  }

  /**
   * @brief Error flags collected during execution
   *
   */
  static union
  {
    uint8_t all_flags;  /// All flags combined into a single byte.
    struct
    {
      uint8_t cfg : 1;      /// Configuration error flag.
      uint8_t cmd : 1;      /// Command error flag.
      uint8_t error : 1;    /// General error flag.
      uint8_t watch : 1;    /// Watchdog error flag.
      uint8_t sparesA : 1;  /// Spare flag A.
      uint8_t sparesB : 1;  /// Spare flag B.
      uint8_t sparesC : 1;  /// Spare flag C.
      uint8_t sparesD : 1;  /// Spare flag D.
    };
  } status;

  /**
   * @brief The slave implements a 128 byte memory. The memory address use the command byte value as memory pointer,
   *        The 8 bit data is written starting at command value
   *
   */
  static struct
  {
    uint8_t reg[128];          // contains data following command byte
    uint8_t reg_address;       // contains command number
    uint8_t reg_status;        // contains status of command
    bool reg_address_written;  // Flag for command byte received
    uint8_t i2c_add;
  } context;

  /**
   * @brief Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls
   * printing to stdio may interfere with interrupt handling.
   *
   * @param i2c i2c instance used
   * @param event interrupt from receive or transmit
   */
  static void i2c_slave_handler(i2c_inst_t* i2c, i2c_slave_event_t event)
  {
    MESSAGE rec;
    uint8_t cmd;  /// keep command value
    bool tvalue;
    uint8_t svalue;
    volatile uint32_t maskvalue, lvalue;

    switch (event)
    {
      case I2C_SLAVE_RECEIVE:              /// master has written some data
        if (!context.reg_address_written)  /// if command data already received
        {
          // writes always start with the memory address
          context.reg_address = i2c_read_byte(i2c);  // read Command byte
          context.reg_address_written = true;
          // sprintf(&rec.data[0],"On i2c  cmd");
          // enque(&rec);
        }
        else                                                      /// read data byte
        {                                                         // WRITE COMMAND
          context.reg[context.reg_address] = i2c_read_byte(i2c);  // read Byte
                                                                  // sprintf(&rec.data[0],"On i2c  data");
                                                                  // enque(&rec);

          cmd = context.reg_address;

          /// Based on Command number, an action is executed

          switch (cmd)
          {  /// Command byte

            case 10:  // Clear Gpio
              gpio_put(context.reg[context.reg_address], 0);
              sprintf(&rec.data[0], "Cmd %02d, Clear Gpio: %02d ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 11:  // Set Gpio
              gpio_put(context.reg[context.reg_address], 1);
              sprintf(&rec.data[0], "Cmd %02d, Set Gpio: %02d ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 12:  // Clear Bank
              if (context.reg[context.reg_address] < 10)
              {                                            // bank 0
                gpio_put_masked(GPIO_BANK0_MASK, 0x00ul);  // Set Output
              }
              else
              {
                gpio_put_masked(GPIO_BANK1_MASK, 0x00ul);  // Set Output
              }
              sprintf(&rec.data[0], "Cmd %02d, Clear Bank Gpio: %02d ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 20:                                              // Set Gpio Direction to Output
              gpio_set_dir(context.reg[context.reg_address], 1);  // turn OFF Led
              sprintf(&rec.data[0], "Cmd %02d, Set Dir Out Gpio: %02d ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 21:                                              // Set Gpio Direction to Input
              gpio_set_dir(context.reg[context.reg_address], 0);  // turn OFF Led
              sprintf(&rec.data[0], "Cmd %02d, Set dir In Gpio: %02d ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 30:                                                                               // Set GPIO strength = 2mA
              gpio_set_drive_strength(context.reg[context.reg_address], GPIO_DRIVE_STRENGTH_2MA);  // set Value
              sprintf(&rec.data[0], "Cmd %02d, 2mA Gpio: %02d ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 31:                                                                               // Set GPIO strength = 4mA
              gpio_set_drive_strength(context.reg[context.reg_address], GPIO_DRIVE_STRENGTH_4MA);  // set Value
              sprintf(&rec.data[0], "Cmd %02d, 4mA Gpio: %02d ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 32:                                                                               // Set GPIO strength = 8mA
              gpio_set_drive_strength(context.reg[context.reg_address], GPIO_DRIVE_STRENGTH_8MA);  // set Value
              sprintf(&rec.data[0], "Cmd %02d, 8mA Gpio: %02d ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 33:                                                                                // Set GPIO strength = 12mA
              gpio_set_drive_strength(context.reg[context.reg_address], GPIO_DRIVE_STRENGTH_12MA);  // set Value
              sprintf(&rec.data[0], "Cmd %02d, 12mA Gpio: %02d ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 41:                                           // Set pull-up
              gpio_pull_up(context.reg[context.reg_address]);  // turn ON pull-up
              sprintf(&rec.data[0], "Cmd %02d, Pull-up Gpio: %02d,  ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 50:                                                 // Clear pull-up and pull-down
              gpio_disable_pulls(context.reg[context.reg_address]);  // turn ON pull-up
              sprintf(&rec.data[0], "Cmd %02d, Clear pull-up, pull-down Gpio: %02d,  ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 51:                                             // Set pull-down
              gpio_pull_down(context.reg[context.reg_address]);  // turn ON pull-down
              sprintf(&rec.data[0], "Cmd %02d, Pull-down Gpio: %02d,  ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 60:  // Set PAD state, Nothing to do other than save on register
              sprintf(&rec.data[0], "Cmd %02d, Pad State: %01d ", cmd, context.reg[context.reg_address]);
              enque(&rec);
              break;

            case 61:  // Set GPx to PAD state
              maskvalue = 0xfful;
              hw_write_masked(&pads_bank0_hw->io[context.reg[context.reg_address]], context.reg[cmd - 1], maskvalue);  // Set Pad state
              sprintf(&rec.data[0], "Cmd %02d, Set Pad State to Gpio: %02d ,State: 0x%01x ", cmd, context.reg[context.reg_address],
                      context.reg[cmd - 1]);
              enque(&rec);
              break;

            case 80:  // Set Direction  Port 0 using 8 bit mask
              if (context.i2c_add == PICO_PORT_ADDRESS)
              {  // if command valid following i2c_address
                maskvalue = context.reg[context.reg_address];
                gpio_set_dir_masked(PORT0_MASK, maskvalue);  // Set Direction
                sprintf(&rec.data[0], "Cmd %02d, Port0, dir: 0x%02lx,  ", cmd, maskvalue);
              }
              else
              {
                sprintf(&rec.data[0], "Cmd %02d, Not Valid for I2C Pico: 0x%02x,  ", cmd, context.i2c_add);
                status.cmd = 1;  // raise error flag
              }
              enque(&rec);
              break;

            case 81:  // Set Output on 8 bit port 0
              if (context.i2c_add == PICO_PORT_ADDRESS)
              {  // if command valid following i2c_address
                maskvalue = context.reg[context.reg_address];
                gpio_put_masked(PORT0_MASK, maskvalue);  // Set Direction
                sprintf(&rec.data[0], "Cmd %02d, Port0, 8 bit Out: 0x%02x,  ", cmd, context.reg[context.reg_address]);
              }
              else
              {
                sprintf(&rec.data[0], "Cmd %02d, Not Valid for I2C Pico: 0x%02x,  ", cmd, context.i2c_add);
                status.cmd = 1;  // raise error flag
              }
              enque(&rec);
              break;

            case 90:  // Set Direction of port 1 using 8 bit mask
              if (context.i2c_add == PICO_PORT_ADDRESS)
              {  // if command valid following i2c_address
                maskvalue = context.reg[context.reg_address] << PORT1_OFFSET;
                gpio_set_dir_masked(PORT1_MASK, maskvalue);  // Set Direction
                sprintf(&rec.data[0], "Cmd %02d, Port1, dir: 0x%02lx,  ", cmd, maskvalue);
              }
              else
              {
                sprintf(&rec.data[0], "Cmd %02d, Not Valid for I2C Pico: 0x%02x,  ", cmd, context.i2c_add);
                status.cmd = 1;  // raise error flag
              }
              enque(&rec);
              break;

            case 91:  // Set Output on 8 bit port 1
              if (context.i2c_add == PICO_PORT_ADDRESS)
              {  // if command valid following i2c_address
                maskvalue = context.reg[context.reg_address] << PORT1_OFFSET;
                gpio_put_masked(PORT1_MASK, maskvalue);  // Set Output
                sprintf(&rec.data[0], "Cmd %02d, Port1, 8 bit Out: 0x%02x,  ", cmd, context.reg[context.reg_address]);
              }
              else
              {
                sprintf(&rec.data[0], "Cmd %02d, Not Valid for I2C Pico: 0x%02x,  ", cmd, context.i2c_add);
              }
              enque(&rec);
              break;
          }
        }
        break;

      case I2C_SLAVE_REQUEST:  // master is requesting data
                               // load from register
        cmd = context.reg_address;

        // For command requesting a Get Value, The register is updated before return the content
        // For readback of Set value, we just return the contents of register

        switch (cmd)
        {  // Command byte yo et register

          case 01:  // get Major Version
            context.reg[context.reg_address] = IO_SLAVE_VERSION_MAJOR;
            sprintf(&rec.data[0], "Cmd %02d, MAJ Version: %02d ", cmd, context.reg[context.reg_address]);
            enque(&rec);
            break;

          case 02:  // get Minor Version
            context.reg[context.reg_address] = IO_SLAVE_VERSION_MINOR;
            sprintf(&rec.data[0], "Cmd %02d, MIN Version: %02d ", cmd, context.reg[context.reg_address]);
            enque(&rec);
            break;

          case 13:                    // read Bank status
            lvalue = gpio_get_all();  // Read All GPIO
            if (context.reg[context.reg_address] < 10)
            {                              // bank 0
              svalue = (uint8_t)(lvalue);  // read lower bank
            }
            else
            {
              svalue = (uint8_t)(lvalue >> 10);  // read high bank
            }
            sprintf(&rec.data[0], "Cmd %02d, Bank: %02d, read: 0x%01x ", cmd, context.reg[context.reg_address], svalue);
            enque(&rec);
            context.reg[context.reg_address] = svalue;
            break;

          case 15:                                                // read True value of Gpio
            tvalue = gpio_get(context.reg[context.reg_address]);  // Read true Value
            sprintf(&rec.data[0], "Cmd %02d, read True Gpio: %02d ,State: %01d ", cmd, context.reg[context.reg_address], tvalue);
            enque(&rec);
            context.reg[context.reg_address] = tvalue;
            break;

          case 25:                                                    // get GPIO Direction
            tvalue = gpio_get_dir(context.reg[context.reg_address]);  // Read Direction Value
            sprintf(&rec.data[0], "Cmd %02d, Red Dir Gpio: %02d ,State: %01d ", cmd, context.reg[context.reg_address], tvalue);
            enque(&rec);
            context.reg[context.reg_address] = tvalue;
            break;

          case 35:                                                               // get GPIO strength
            svalue = gpio_get_drive_strength(context.reg[context.reg_address]);  // Read strength Value
            sprintf(&rec.data[0], "Cmd %02d, Read strenght Gpio: %02d ,State: %01d ", cmd, context.reg[context.reg_address], svalue);
            enque(&rec);
            context.reg[context.reg_address] = svalue;
            break;

          case 45:                                                         // get pull-up
            tvalue = gpio_is_pulled_up(context.reg[context.reg_address]);  // Read true Value
            sprintf(&rec.data[0], "Cmd %02d, read pull-up Gpio: %02d ,State: %01d ", cmd, context.reg[context.reg_address], tvalue);
            enque(&rec);
            context.reg[context.reg_address] = tvalue;
            break;

          case 55:                                                           // get pull-down
            tvalue = gpio_is_pulled_down(context.reg[context.reg_address]);  // Read true Value
            sprintf(&rec.data[0], "Cmd %02d, Read pull-down Gpio: %02d ,State: %01d ", cmd, context.reg[context.reg_address], tvalue);
            enque(&rec);
            context.reg[context.reg_address] = tvalue;
            break;

          case 65:                                                                // get PAD state
            svalue = pads_bank0_hw->io[context.reg[context.reg_address]] & 0xff;  // Read gpio PAD Value
            sprintf(&rec.data[0], "Cmd %02d, Gpio: %02d ,Read PAD State: 0x%01x ", cmd, context.reg[context.reg_address], svalue);
            enque(&rec);
            context.reg[context.reg_address] = svalue;
            break;

          case 85:  // get Port 0 value
            if (context.i2c_add == PICO_PORT_ADDRESS)
            {                            // if command valid following i2c_address
              lvalue = gpio_get_all();   // Read All GPIO
              svalue = (uint8_t)lvalue;  // keep only 8 bit
              sprintf(&rec.data[0], "Cmd %02d,Read Port0 8 bit In: 0x%01x ", cmd, svalue);
              context.reg[context.reg_address] = svalue;
            }
            else
            {
              sprintf(&rec.data[0], "Cmd %02d, Not Valid for I2C Pico: 0x%02x,  ", cmd, context.i2c_add);
              status.cmd = 1;  // raise error flag
            }
            enque(&rec);
            break;

          case 95:  // get Port 1 value
            if (context.i2c_add == PICO_PORT_ADDRESS)
            {                           // if command valid following i2c_address
              lvalue = gpio_get_all();  // Read All GPIO
              svalue = (uint8_t)(lvalue >> PORT1_OFFSET);
              sprintf(&rec.data[0], "Cmd %02d, Read Port1 8 bit In: 0x%01x ", cmd, svalue);
              context.reg[context.reg_address] = svalue;
            }
            else
            {
              sprintf(&rec.data[0], "Cmd %02d, Not Valid for I2C Pico: 0x%02x,  ", cmd, context.i2c_add);
              status.cmd = 1;  // raise error flag
            }
            enque(&rec);

            break;

          case 100:  // get statsus register, nothing to do
            context.reg[REG_STATUS] = status.all_flags;
            sprintf(&rec.data[0], "Cmd %02d,Status register: 0x%01x ", cmd, context.reg[REG_STATUS]);
            enque(&rec);
            break;
        }

        i2c_write_byte(i2c, context.reg[context.reg_address]);
        sprintf(&rec.data[0], "Read Cmd : %02d , Value: %02d ", cmd, context.reg[context.reg_address]);
        enque(&rec);

        break;
      case I2C_SLAVE_FINISH:  // master has signalled Stop / Restart
        context.reg_address_written = false;
        // sprintf(&rec.data[0],"On i2c_finish");
        // enque(&rec);

        break;
      default:
        break;
    }
  }

  /**
   * @brief function who read the 2 externals pins to define the I2C address to use.
   *        The address = 0x20 + value of 2 externals pins
   *
   * @return uint8_t Address to use for the Pico slave
   */
  uint8_t read_i2c_address()
  {
    uint8_t io0, io1, I2C_address;

    gpio_set_function(I2C_SLAVE_ADDRESS_IO0, GPIO_FUNC_SIO);  // Set mode to software IO Control
    gpio_set_dir(I2C_SLAVE_ADDRESS_IO0, false);               // Set IO to input
    gpio_pull_up(I2C_SLAVE_ADDRESS_IO0);                      // Set to pull-up

    gpio_set_function(I2C_SLAVE_ADDRESS_IO1, GPIO_FUNC_SIO);  // Set mode to software IO Control
    gpio_set_dir(I2C_SLAVE_ADDRESS_IO1, false);               // Set IO to input
    gpio_pull_up(I2C_SLAVE_ADDRESS_IO1);                      // Set to pull-up

    io0 = gpio_get(I2C_SLAVE_ADDRESS_IO0);
    io1 = gpio_get(I2C_SLAVE_ADDRESS_IO1);
    I2C_address = I2C_OFFSET_ADDRESS + (io1 << 1) + io0;
    return I2C_address;
  }

  /**
   * @brief Set the up slave object
   *
   * @param i2c_add Address to use to configure the I2C slave
   */
  static void setup_slave(uint8_t i2c_add)
  {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);

    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, i2c_add, &i2c_slave_handler);
    // i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
  }

  /// Used on in development to loopback I2C to simulate master talking to slave

  #ifdef USE_MASTER_LOOPBACK

  static const uint I2C_MASTER_SDA_PIN = 6;  /// Master SDA pin
  static const uint I2C_MASTER_SCL_PIN = 7;  /// Master SCL pins

  /**
   * @brief Set the up master object
   *
   */
  static void setup_master()
  {
    gpio_init(I2C_MASTER_SDA_PIN);
    gpio_set_function(I2C_MASTER_SDA_PIN, GPIO_FUNC_I2C);
    // pull-ups are already active on slave side, this is just a fail-safe in case the wiring is faulty
    gpio_pull_up(I2C_MASTER_SDA_PIN);

    gpio_init(I2C_MASTER_SCL_PIN);
    gpio_set_function(I2C_MASTER_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_MASTER_SCL_PIN);

    i2c_init(i2c1, I2C_BAUDRATE);
  }

  /**
   * @brief send i2c data on the master pins
   *
   * @param cmd   commandto send
   * @param wdata data to send
   */
  static void send_master(uint8_t cmd, uint16_t wdata)
  {
    // Writing to A register
    int count;
    uint8_t buf[3];
    int buflgth;  // contains size of the buffer

    buflgth = 2;
    buf[0] = cmd;    // command
    buf[1] = wdata;  // gpio

    count = i2c_write_blocking(i2c1, context.i2c_add, buf, buflgth, false);
    if (count < 0)
    {
      puts("Couldn't write Register to slave");
      return;
    }
    printf("MAS: Write at register 0x%02d: %02d\n", buf[0], buf[1]);

    uint8_t ird[2];
    i2c_write_blocking(i2c1, context.i2c_add, buf, 1, false);
    i2c_read_blocking(i2c1, context.i2c_add, ird, buflgth - 1, false);

    printf("MAS:Read Register 0x%02d = %d \r\n", cmd, ird[0]);
  }

  #endif

  /**
   * @brief main loop to execute i2c command from master. Pico les is flashing to indicate heartbeat
   *
   * @return int   do nothing
   */
  int main()
  {
    MESSAGE rec;
    uint16_t ctr;    // counter used for flashing led
    uint16_t pulse;  // limit for flashing led frequency

    status.all_flags = 0;
    pulse = 200;  // slow led flashing frequency

    if (watchdog_caused_reboot())
    {
      status.watch = 1;
      pulse = 50;  // fast flashing led to indicate watchdog trig
    }

    gpio_init_mask(GPIO_BOOT_MASK);  // set which lines will be GPIO
    init_queue();                    // initialise queue for serial message
    stdio_init_all();

    fprintf(stdout, "Slave Version: %d.%d\n", IO_SLAVE_VERSION_MAJOR, IO_SLAVE_VERSION_MINOR);

    context.i2c_add = read_i2c_address();  // Setup I2C Address

    sprintf(&rec.data[0], "Pico Slave boot for I2C address 0x%02x", context.i2c_add);
    enque(&rec);  // Add message to the queue

    switch (context.i2c_add)
    {  // Config following i2C Address

      case 0x21:  // IO slave
        gpio_set_dir_masked(GPIO_SET_DIR_MASK, GPIO_SLV1_DIR_MASK);
        gpio_put_masked(GPIO_SET_DIR_MASK, GPIO_SLV1_OUT_MASK);
        sprintf(&rec.data[0], "Config for I2C address 0x%02x completed", context.i2c_add);
        break;

      case 0x22:
      case 0x23:  // Relay slave
        gpio_set_dir_masked(GPIO_SET_DIR_MASK, GPIO_SLV2_DIR_MASK);
        gpio_put_masked(GPIO_SET_DIR_MASK, GPIO_SLV2_OUT_MASK);
        sprintf(&rec.data[0], "Config for I2C address 0x%02x completed", context.i2c_add);
        break;

      default:  // not defined yet
        sprintf(&rec.data[0], "I2C address not supported for device at address  0x%02x", context.i2c_add);
        status.cfg = 1;  // raise error flag
        break;
    }
    enque(&rec);  // Add message to the queue

    setup_slave(context.i2c_add);
    // setup_master();  // for development only, using loopback

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);  // Configure Pico led board
    gpio_put(PICO_DEFAULT_LED_PIN, 1);             // turn ON green led on Pico

    // watchdog_enable(500, 1); // enable the watchdog
    ctr = 0;
    int mess = 0;

    while (1)
    {  // infinite loop, waiting for I2C command from Master

      watchdog_update();
      sleep_ms(10);
      ctr++;
      mess++;

      if (ctr > pulse)
      {
        gpio_put(PICO_DEFAULT_LED_PIN, 0);  // Turn OFF board led
        sleep_ms(200);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);  // Turn ON board led
        ctr = 0;
      }
      if (mess > 1500)
      {
        // printf("i2c add: 0x%02x\n", context.i2c_add); // for debug only
        fprintf(stdout, "Heartbeat I2C Slave add: 0x%02x  version: %d.%d\n", context.i2c_add, IO_SLAVE_VERSION_MAJOR, IO_SLAVE_VERSION_MINOR);
        mess = 0;
      }

      #ifdef USE_MASTER_LOOPBACK
          // Need loopback on I2C
          send_master(11, 28);     // test command when i2c loopback is used
          send_master(15, 0x02);   // test command
          send_master(85, 0xC0);   // test command
          send_master(100, 0x00);  // test command
      #endif

      while (deque(&rec))
      {
        gpio_put(PICO_DEFAULT_LED_PIN, 0);                         // Turn OFF board led
        printf("Pico %02x: %s\n", context.i2c_add, &rec.data[0]);  // send message to serial port
        sleep_ms(50);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);  // Turn ON board led
      }
    }
  }
