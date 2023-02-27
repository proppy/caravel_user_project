(*blackbox*)
module TopLevel_oscillator_macro (
`ifdef USE_POWER_PINS
    inout VP,
    inout GND,
`endif
    input CLK_EN,
    output Y,
    input AND_OUT
);
endmodule // TopLevel_oscillator
