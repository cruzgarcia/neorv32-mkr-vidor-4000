
create_clock -name i_osc_clock -period 20.833 [get_ports {clk_i}]
derive_clock_uncertainty