#include "bootloader.h"
#include "bootloader_rcc.h"
#include "bootloader_gpio.h"

/* GPIO config options can be just computed instead of stored in a table:
 *   - port base address = 0x40020000 + (port * 0x400)
 *   - RCC port: AHB1ENR
 *   - RCC bit: same as port
 */
#define GPIO_BASE            0x40020000
#define GPIO_STEP            0x00000400

#define GPIO_REGOFF_MODER    0x000
#define GPIO_REGOFF_OTYPER   0x004
#define GPIO_REGOFF_OSPEEDER 0x008
#define GPIO_REGOFF_PUPDR    0x00C
#define GPIO_REGOFF_IDR      0x010
#define GPIO_REGOFF_ODR      0x014
#define GPIO_REGOFF_BSRR     0x018
#define GPIO_REGOFF_LCKR     0x01C
#define GPIO_REGOFF_AFRL     0x020
#define GPIO_REGOFF_AFRH     0x024

BOOTCODE void bootloader_gpio_init(uint32_t gpiodesc)
{
  uint32_t line = (gpiodesc & GPIO_FLAGS_MASK_LINE) >> GPIO_FLAGS_SHIFT_LINE;
  uint32_t port = (gpiodesc & GPIO_FLAGS_MASK_PORT) >> GPIO_FLAGS_SHIFT_PORT;
  uint32_t base, val;

  //get port base address
  if(port >= 10)
    {
      return;
    }
  base = GPIO_BASE + (port * GPIO_STEP);

  //Enable clock to GPIO peripheral
  modreg32(RCC_AHB1ENR, 1<<port, 0);


  //configure 1-bit ports
  //Define initial state Initial state and output type (output only)
  if((gpiodesc & GPIO_FLAGS_MASK_MODE) == GPIO_MODE_OUT)
    {
      val = (gpiodesc & GPIO_FLAGS_MASK_INIT) >> GPIO_FLAGS_SHIFT_INIT;
      bootloader_gpio_write(gpiodesc, val);

      val = (gpiodesc & GPIO_FLAGS_MASK_TYPE) >> GPIO_FLAGS_SHIFT_TYPE;
      modreg32(base + GPIO_REGOFF_OTYPER, val << line, 1 << line);
    }
	
  //configure 2-bit ports
  line += line;
	
  //Define pin mode
  val = (gpiodesc & GPIO_FLAGS_MASK_MODE) >> GPIO_FLAGS_SHIFT_MODE;
  modreg32(base + GPIO_REGOFF_MODER, val << line, 3 << line);
	
  //Define output speed (output only)
  if((gpiodesc & GPIO_FLAGS_MASK_MODE) == GPIO_MODE_OUT)
    {
      val = (gpiodesc & GPIO_FLAGS_MASK_SPD) >> GPIO_FLAGS_SHIFT_SPD;
      modreg32(base + GPIO_REGOFF_OSPEEDER, val << line, 3 << line);
    }

  //Define pull mode
  val = (gpiodesc & GPIO_FLAGS_MASK_PULL) >> GPIO_FLAGS_SHIFT_PULL;
  modreg32(base + GPIO_REGOFF_PUPDR, val << line, 3 << line);

  //configure 4-bit ports
  line += line;
	
  //Offset 20 AFRL (for lines 0-7)
  //Offset 24 AFRH (for lines 8-15)
  val = (gpiodesc & GPIO_FLAGS_MASK_ALT) >> GPIO_FLAGS_SHIFT_ALT;
  if(line < (8 << 2))
    {
      modreg32(base + GPIO_REGOFF_AFRL, val << line, 15 << line);
    }
  else
    {
      line -= 32;
      modreg32(base + GPIO_REGOFF_AFRL, val << line, 15 << line);
    }
}

BOOTCODE void bootloader_gpio_write(uint32_t gpio, int state)
{
  uint32_t line = (gpio & GPIO_FLAGS_MASK_LINE) >> GPIO_FLAGS_SHIFT_LINE;
  uint32_t port = (gpio & GPIO_FLAGS_MASK_PORT) >> GPIO_FLAGS_SHIFT_PORT;
  uint32_t base;

  //get port base address
  if(port >= 10)
    {
      return;
    }
  base = GPIO_BASE + (port * GPIO_STEP);

  if(!state)
    {
      line += 16; //access BR instead of BS
    }
  base += GPIO_REGOFF_BSRR;
  putreg32(base, 1<<line);
}

BOOTCODE int bootloader_gpio_read(uint32_t gpio)
{
  uint32_t line = (gpio & GPIO_FLAGS_MASK_LINE) >> GPIO_FLAGS_SHIFT_LINE;
  uint32_t port = (gpio & GPIO_FLAGS_MASK_PORT) >> GPIO_FLAGS_SHIFT_PORT;
  uint32_t base;

  //get port base address
  if(port >= 10)
    {
      return 0;
    }
  base = GPIO_BASE + (port * GPIO_STEP);

  return (getreg32(base + GPIO_REGOFF_IDR) >> line) & 1;
}

