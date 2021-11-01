library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

library neorv32;
use neorv32.neorv32_package.all;

library sdram_controller;

entity neorv32_mkr_vidor_4000_top is
  generic (
    CLOCK_FREQUENCY     : natural := 48_000_000;  -- clock frequency of clk_i in Hz
    -- Internal Instruction memory (IMEM) --
    MEM_INT_IMEM_EN     : boolean := TRUE;     -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE   : natural := 32*1024;  -- size of processor-internal instruction memory in bytes
    MEM_INT_DMEM_SIZE   : natural := 8*1024;    -- size of processor-internal data memory in bytes
    -- External memory interface (WISHBONE)
    MEM_EXT_EN          : boolean := TRUE;   -- implement external memory bus interface?
    MEM_EXT_TIMEOUT     : natural := 255;    -- cycles after a pending bus access auto-terminates (0 = disabled)
    MEM_EXT_PIPE_MODE   : boolean := FALSE;  -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
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
    flash_i03           : out std_logic;
    -- SDRAM
    o_sdram_addr        : out   std_logic_vector(11 downto 0);                    -- addr
    o_sdram_ba          : out   std_logic_vector(1 downto 0);                     -- ba
    o_sdram_cas_n       : out   std_logic;                                        -- cas_n
    o_sdram_clk         : out   std_logic;
    o_sdram_cke         : out   std_logic;                                        -- cke
    o_sdram_cs_n        : out   std_logic;                                        -- cs_n
    io_sdram_dq         : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
    o_sdram_dqm         : out   std_logic_vector(1 downto 0);                     -- dqm
    o_sdram_ras_n       : out   std_logic;                                        -- ras_n
    o_sdram_we_n        : out   std_logic                                         -- we_n
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
  -- Avalon to SDRAM
  SIGNAL s_avl_sdram_read             : std_logic;
  SIGNAL s_avl_sdram_write            : std_logic;
  SIGNAL s_avl_sdram_waitrequest      : std_logic := '0';
  SIGNAL s_avl_sdram_byteenable       : std_logic_vector(3 downto 0);
  SIGNAL s_avl_sdram_address          : std_logic_vector(31 downto 0);
  SIGNAL s_avl_sdram_writedata        : std_logic_vector(31 downto 0);
  SIGNAL s_avl_sdram_readdata         : std_logic_vector(31 downto 0) := (others => '0');
  SIGNAL s_avl_sdram_readdatavalid    : STD_LOGIC;
  SIGNAL s_avl_sdram_burstcount       : STD_LOGIC_VECTOR(0 downto 0);
  -- GPIO
  signal con_gpio_o                   : std_ulogic_vector(63 downto 0);
  signal con_gpio_i                   : std_ulogic_vector(63 downto 0);
  -- SPI 
  signal spi_sck                      : std_ulogic;
  signal spi_sdo                      : std_ulogic;
  signal spi_sdi                      : std_ulogic;
  signal spi_csn                      : std_ulogic_vector(7 downto 0);

  component sdram_controller is
		port (
			clk_clk                    : in    std_logic                     := 'X';             -- clk
			reset_reset_n              : in    std_logic                     := 'X';             -- reset_n
			io_sdram_addr              : out   std_logic_vector(11 downto 0);                    -- addr
			io_sdram_ba                : out   std_logic_vector(1 downto 0);                     -- ba
			io_sdram_cas_n             : out   std_logic;                                        -- cas_n
			io_sdram_cke               : out   std_logic;                                        -- cke
			io_sdram_cs_n              : out   std_logic;                                        -- cs_n
			io_sdram_dq                : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
			io_sdram_dqm               : out   std_logic_vector(1 downto 0);                     -- dqm
			io_sdram_ras_n             : out   std_logic;                                        -- ras_n
			io_sdram_we_n              : out   std_logic;                                        -- we_n
			io_avl_sdram_waitrequest   : out   std_logic;                                        -- waitrequest
			io_avl_sdram_readdata      : out   std_logic_vector(31 downto 0);                    -- readdata
			io_avl_sdram_readdatavalid : out   std_logic;                                        -- readdatavalid
			io_avl_sdram_burstcount    : in    std_logic_vector(0 downto 0)  := (others => 'X'); -- burstcount
			io_avl_sdram_writedata     : in    std_logic_vector(31 downto 0) := (others => 'X'); -- writedata
			io_avl_sdram_address       : in    std_logic_vector(31 downto 0) := (others => 'X'); -- address
			io_avl_sdram_write         : in    std_logic                     := 'X';             -- write
			io_avl_sdram_read          : in    std_logic                     := 'X';             -- read
			io_avl_sdram_byteenable    : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- byteenable
			io_avl_sdram_debugaccess   : in    std_logic                     := 'X'              -- debugaccess
		);
	end component sdram_controller;

begin

  -- TODO Use a PLL?
  o_sdram_clk   <= clk_i;

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
    -- Wishbone bus interface (available if MEM_EXT_EN = true)
    wb_tag_o                    => wb_tag_o,
    wb_adr_o                    => wb_adr_o,
    wb_dat_i                    => wb_dat_i,
    wb_dat_o                    => wb_dat_o,
    wb_we_o                     => wb_we_o,
    wb_sel_o                    => wb_sel_o,
    wb_stb_o                    => wb_stb_o,
    wb_cyc_o                    => wb_cyc_o,
    wb_lock_o                   => wb_lock_o,
    wb_ack_i                    => wb_ack_i,
    wb_err_i                    => wb_err_i,
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

  -- Wishbone to Avalon MM
  s_avl_sdram_read        <= '1' when (wb_stb_o = '1' and wb_we_o = '0') else '0';
  s_avl_sdram_write       <= '1' when (wb_stb_o = '1' and wb_we_o = '1') else '0';
  s_avl_sdram_address     <= STD_LOGIC_VECTOR(wb_adr_o);
  s_avl_sdram_writedata   <= STD_LOGIC_VECTOR(wb_dat_o);
  s_avl_sdram_byteenable  <= STD_LOGIC_VECTOR(wb_sel_o);
  -- Avalon MM to Wishbone
  wb_dat_i                <= std_ulogic_vector(s_avl_sdram_readdata);
  wb_err_i                <= '0';
  
  proc_ack : PROCESS(wb_stb_o, wb_we_o, s_avl_sdram_waitrequest, s_avl_sdram_readdatavalid)
  BEGIN
    IF (wb_stb_o = '1' AND wb_we_o = '1') THEN
      -- Write request
      wb_ack_i            <= s_avl_sdram_waitrequest;
    ELSIF(wb_stb_o = '1' AND wb_we_o = '0') THEN
      wb_ack_i            <= s_avl_sdram_readdatavalid;
    ELSE
      wb_ack_i            <= '0';
    END IF;
  END PROCESS;

  inst_sdram : sdram_controller
  port map (
    clk_clk                    => clk_i,
    reset_reset_n              => rstn_i,
    --
    io_avl_sdram_waitrequest   => s_avl_sdram_waitrequest,
    io_avl_sdram_readdata      => s_avl_sdram_readdata,
    io_avl_sdram_readdatavalid => s_avl_sdram_readdatavalid,
    io_avl_sdram_burstcount    => s_avl_sdram_burstcount,
    io_avl_sdram_writedata     => s_avl_sdram_writedata,
    io_avl_sdram_address       => s_avl_sdram_address,
    io_avl_sdram_write         => s_avl_sdram_write,
    io_avl_sdram_read          => s_avl_sdram_read,
    io_avl_sdram_byteenable    => s_avl_sdram_byteenable,
    --
    io_sdram_addr              => o_sdram_addr,
    io_sdram_ba                => o_sdram_ba,
    io_sdram_cas_n             => o_sdram_cas_n,
    io_sdram_cke               => o_sdram_cke,
    io_sdram_cs_n              => o_sdram_cs_n,
    io_sdram_dq                => io_sdram_dq,
    io_sdram_dqm               => o_sdram_dqm,
    io_sdram_ras_n             => o_sdram_ras_n,
    io_sdram_we_n              => o_sdram_we_n
  );

  s_avl_sdram_burstcount <= "1";

  --=======================================================
end architecture;
