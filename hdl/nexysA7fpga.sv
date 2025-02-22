////////////
// Top-level module for ECE 544 Project #2
// May have to be modified for your specific embedded system implementation
///////////
`timescale 1 ps / 1 ps

module nexysa7fpga
   (
	input logic			clk,
	input logic [15:0]	sw,
	input logic		    btnU,
	input logic		    btnR,
	input logic			btnL,
	input logic			btnD,
	input logic			btnC,			
	input logic			btnCpuReset,
	output logic [15:0]	led,
    output logic [7:0]	an,
	output logic [6:0]	seg,
    output logic		dp,
    output logic        pid_LED,     //pmod jb1 for led
    inout logic 		sclk_io,	 //i2c io 
    inout logic 		sda_io,
    input logic 		usb_uart_rxd,
    output logic 		usb_uart_txd
);
    
    //connect btn/sw to gpio
    logic [15:0]	    gpio_btn;
    logic [15:0]	    gpio_sw;   
    assign gpio_btn = {11'h000, btnC, btnU, btnD, btnL, btnR};
    assign gpio_sw = sw;


	//instantiate the embedded system    
	embsys embsys_i
	(.an_0(an),
	.btnC_0(btnC),
	.btnD_0(btnD),
	.btnL_0(btnL),
	.btnR_0(btnR),
	.btnU_0(btnU),
	.clk_100MHz(clk),
	.dp_0(dp),
	.gpio_btn_i_tri_i(gpio_btn),
	.gpio_sw_i_tri_i(gpio_sw),
	.led_0(led),
	.pwm_led(pid_LED),
	.resetn(btnCpuReset),
	.sclk_io(sclk_io),
	.sda_io(sda_io),
	.seg_0(seg),
	.sw_0(sw),
	.uart_rtl_0_rxd(usb_uart_rxd),
	.uart_rtl_0_txd(usb_uart_txd));
        
      
endmodule
