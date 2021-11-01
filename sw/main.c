/**********************************************************************//**
 * @file main.c
 * @author Cruz Garcia
 * @brief Basic read/write test to the external memory
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

// A location after the IMEM size (32 kB) -> 524,288 kB
#define EXT_MEM_REG(x) (*((volatile unsigned int*) (0x00080000UL + x * 4)))

/**********************************************************************//**
 **************************************************************************/
int main() {

  // UART setup
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // Exception capture
  neorv32_rte_setup();

  // Write/Read test
  neorv32_uart0_print("Hello there, I am just checking out...\n");
  neorv32_uart0_print("Writting to SDRAM\n");
  unsigned rw_cycles = 4;
  for (unsigned i = 0; i < rw_cycles; ++i){
    neorv32_uart0_printf("Writting %u\n", i);
    EXT_MEM_REG(i) = i;
  }

  neorv32_uart0_print("Reading from SDRAM\n");
  for (unsigned i = 0; i < rw_cycles; ++i){
    neorv32_uart0_printf("Read value: %x\n", EXT_MEM_REG(i));
  }

  neorv32_uart0_print("Done!\n");
  return 0;
}

