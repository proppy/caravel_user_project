`default_nettype none

module osc (
`ifdef USE_POWER_PINS
            inout vdd,
            inout vss,
`endif
            input  osc_en,
            (* clkbuf_inhibit *) output osc_out
);


   wire           osc_sig;

   TopLevel_oscillator_macro osc0(
`ifdef USE_POWER_PINS
        .VP(vdd),	// User area 1 1.8V power
        .GND(vss),	// User area 1 digital ground
`endif
       .CLK_EN(osc_en),
       .Y(osc_sig),
       .AND_OUT(osc_out)
   );
   assign osc_out = osc_sig & osc_en;
endmodule //top
`default_nettype wire
