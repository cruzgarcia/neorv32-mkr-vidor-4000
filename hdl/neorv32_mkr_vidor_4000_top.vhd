library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_mkr_vidor_4000_top is
  generic (
    CLOCK_FREQUENCY     : natural := 48_000_000;  -- clock frequency of clk_i in Hz
    -- Internal Instruction memory (IMEM) --
    MEM_INT_IMEM_EN     : boolean := TRUE;     -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE   : natural := 32*1024;  -- size of processor-internal instruction memory in bytes
    MEM_INT_DMEM_SIZE   : natural := 8*1024;    -- size of processor-internal data memory in bytes
    -- External memory interface (WISHBONE)
    MEM_EXT_EN          : boolean := FALSE;   -- implement external memory bus interface?
    MEM_EXT_TIMEOUT     : natural := 255;    -- cycles after a pending bus access auto-terminates (0 = disabled)
    MEM_EXT_PIPE_MODE   : boolean := false;  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
    MEM_EXT_BIG_ENDIAN  : boolean := false;  -- byte order: true=big-endian, false=little-endian
    MEM_EXT_ASYNC_RX    : boolean := false;  -- use register buffer for RX data when false    
    --
    IO_SPI_EN           : boolean := true       -- implement serial peripheral interface (SPI)?
  );
  port (
    -- Global signals
    clk_i               : in  std_ulogic;
    rstn_i              : in  std_ulogic;
    -- GPIO
    gpio_o              : out std_ulogic_vector(7 downto 0);
    -- UART 0
    uart0_txd_o         : out std_ulogic;
    uart0_rxd_i         : in  std_ulogic;
    -- Serial Flash
    flash_cs_o          : out std_logic;
    flash_clk_o         : out std_logic;
    flash_so            : out std_logic;
    flash_si            : in  std_logic;
    flash_i02           : out std_logic;
    flash_i03           : out std_logic
  );
end entity;

architecture rtl of neorv32_mkr_vidor_4000_top is

  -- SPI
  constant c_spi_csn_flash_index      : integer := 0;
  -- Wishbone 
  signal wb_tag_o                     : std_ulogic_vector(02 downto 0);
  signal wb_adr_o                     : std_ulogic_vector(31 downto 0); 
  signal wb_dat_i                     : std_ulogic_vector(31 downto 0) := (others => 'U');
  signal wb_dat_o                     : std_ulogic_vector(31 downto 0);
  signal wb_we_o                      : std_ulogic;
  signal wb_sel_o                     : std_ulogic_vector(03 downto 0);
  signal wb_stb_o                     : std_ulogic;
  signal wb_cyc_o                     : std_ulogic;
  signal wb_lock_o                    : std_ulogic;
  signal wb_ack_i                     : std_ulogic := 'L';
  signal wb_err_i                     : std_ulogic := 'L';
  -- GPIO
  signal con_gpio_o                   : std_ulogic_vector(63 downto 0);
  signal con_gpio_i                   : std_ulogic_vector(63 downto 0);
  -- SPI 
  signal spi_sck                      : std_ulogic;
  signal spi_sdo                      : std_ulogic;
  signal spi_sdi                      : std_ulogic;
  signal spi_csn                      : std_ulogic_vector(7 downto 0);
  -- SDRAM
  SIGNAL s_sdrc_selfrefresh           : STD_LOGIC := '0';
  SIGNAL s_sdrc_power_down            : STD_LOGIC := '0';
  SIGNAL s_sdrc_init_done             : STD_LOGIC := '0';
  SIGNAL s_sdrc_wr_n                  : STD_LOGIC := '0';
  SIGNAL s_sdrc_rd_n                  : STD_LOGIC := '0';
  SIGNAL s_sdrc_addr                  : STD_LOGIC_VECTOR(20 DOWNTO 0) := (OTHERS => '0');
  SIGNAL s_sdrc_data_len              : STD_LOGIC_VECTOR( 6 DOWNTO 0) := (OTHERS => '0');
  SIGNAL s_sdrc_dqm                   : STD_LOGIC_VECTOR( 3 DOWNTO 0) := (OTHERS => '0');
  SIGNAL s_sdrc_data_i                : STD_LOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => '0');
  SIGNAL s_sdrc_data_o                : STD_LOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => '0');
  SIGNAL s_sdrc_busy_n                : STD_LOGIC := '0';
  SIGNAL s_sdrc_rd_valid              : STD_LOGIC := '0';
  SIGNAL s_sdrc_wrd_ack               : STD_LOGIC := '0';

begin

  -- TODO Sync logic for reset to PLL

  -- NEORV32
  neorv32_top_inst: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY              => CLOCK_FREQUENCY,   -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            => true,              -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        => true,              -- implement compressed extension?
    CPU_EXTENSION_RISCV_M        => true,              -- implement mul/div extension?
    CPU_EXTENSION_RISCV_Zicsr    => true,              -- implement CSR system?
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              => MEM_INT_IMEM_EN,   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE, -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    MEM_INT_DMEM_EN              => true,              -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE, -- size of processor-internal data memory in bytes
    --
    MEM_EXT_EN                   => MEM_EXT_EN,
    MEM_EXT_TIMEOUT              => MEM_EXT_TIMEOUT,
    MEM_EXT_PIPE_MODE            => MEM_EXT_PIPE_MODE,
    MEM_EXT_BIG_ENDIAN           => MEM_EXT_BIG_ENDIAN,
    MEM_EXT_ASYNC_RX             => MEM_EXT_ASYNC_RX,
    -- Processor peripherals --
    IO_GPIO_EN                   => true,              -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN                  => true,              -- implement machine system timer (MTIME)?
    IO_UART0_EN                  => true,              -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_TWI_EN                    => FALSE,              -- implement two-wire interface (TWI)?
    -- Processor peripherals --
    IO_SPI_EN                    => IO_SPI_EN
  )
  port map (
    -- Global control --
    clk_i                       => clk_i,
    rstn_i                      => rstn_i,
    -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    --wb_tag_o                    => wb_tag_o,
    --wb_adr_o                    => wb_adr_o,
    --wb_dat_i                    => wb_dat_i,
    --wb_dat_o                    => wb_dat_o,
    --wb_we_o                     => wb_we_o,
    --wb_sel_o                    => wb_sel_o,
    --wb_stb_o                    => wb_stb_o,
    --wb_cyc_o                    => wb_cyc_o,
    --wb_lock_o                   => wb_lock_o,
    --wb_ack_i                    => wb_ack_i,
    --wb_err_i                    => wb_err_i,
    -- GPIO
    gpio_o                      => con_gpio_o,
    gpio_i                      => con_gpio_i,
    -- Primary UART0
    uart0_txd_o                 => uart0_txd_o,
    uart0_rxd_i                 => uart0_rxd_i,
    -- SPI
    spi_sck_o                   => spi_sck,
    spi_sdo_o                   => spi_sdo,
    spi_sdi_i                   => spi_sdi,
    spi_csn_o                   => spi_csn
  );

  -- GPIO output
  gpio_o            <= con_gpio_o(7 downto 0);
  -- Flash
  flash_cs_o        <= spi_csn(c_spi_csn_flash_index);
  flash_clk_o       <= spi_sck;
  flash_so          <= spi_sdo;
  flash_i02         <= 'Z';
  flash_i03         <= 'Z';
  -- SPI CSn multiplexer
  proc_flash_din_mux : process(spi_csn, flash_si, spi_sdi)
  begin
    case spi_csn is
      when b"11111110" => spi_sdi <= flash_si;
      when others => spi_sdi <= '1';
    end case;
  end process;

  --=======================================================
end architecture;
