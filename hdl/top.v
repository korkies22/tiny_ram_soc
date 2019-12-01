/*
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

module top (
    input clk_16mhz,

    // onboard USB interface
    output pin_pu,
    output pin_usbp,
    output pin_usbn,

    // hardware UART
    output uart_out,
    input uart_in,

    // PWM
    output pinPwmIzqF,
    output pinPwmIzqB,
    output pinPwmDerF,
    output pinPwmDerB,

    output pin_7,
    output pin_8,

    // encoders
    input pinEncoderIF,
    input pinEncoderIB,
    input pinEncoderDF,
    
    input pinEncoderDB,

);
    assign pin_pu = 1'b1;
    assign pin_usbp = 1'b0;
    assign pin_usbn = 1'b0;

    wire clk = clk_16mhz;
  

    ///////////////////////////////////
    // Power-on Reset
    ///////////////////////////////////
    reg [5:0] reset_cnt = 0;
    wire resetn = &reset_cnt;

    always @(posedge clk) begin
        reset_cnt <= reset_cnt + !resetn;
    end
    
    ///////////////////////////////////
    // Peripheral Bus
    ///////////////////////////////////
    wire        iomem_valid;
    reg         iomem_ready;
    wire [3:0]  iomem_wstrb;
    wire [31:0] iomem_addr;
    wire [31:0] iomem_wdata;
    reg [31:0] clock_out;
    reg  [31:0] iomem_rdata;

    reg [31:0] clockO=0;
    reg [31:0] gpio;
    reg [31:0] leds;

    reg pwm_out;
    reg [31:0] pwm_connectorIF=0;
    reg [31:0] pwm_connectorIB=0;
    reg [31:0] pwm_connectorDF=0;
    reg [31:0] pwm_connectorDB=0;

    reg[31:0] pinTest=0;
    assign pin_8=pinTest[0];
    assign pin_7=clk;

    reg writeEncoderI=0;
    reg writeEncoderD=0;
    wire [31:0] encoderValueI;
    wire [31:0] encoderValueD;
    reg [31:0] encoderDataI=0;
    reg [31:0] encoderDataD=0;
    

    //assign user_led = gpio[0];
    //assign user_led = !pwm_out;
    //assign pin_20 = pwm_out;

    always @(posedge clk) begin
        if (!resetn) begin
            gpio <= 0;
        end else begin
            iomem_ready <= 0;
            writeEncoderI <=0; 
            writeEncoderD <=0; 

            ///////////////////////////
            // GPIO Peripheral
            ///////////////////////////
            if (iomem_valid && !iomem_ready && iomem_addr[31:8] == 24'h030002) begin
                iomem_ready <= 1;
                iomem_rdata <= clock_out;
                if (iomem_wstrb[0]) clockO[ 7: 0] <= iomem_wdata[ 7: 0];
                if (iomem_wstrb[1]) clockO[15: 8] <= iomem_wdata[15: 8];
                if (iomem_wstrb[2]) clockO[23:16] <= iomem_wdata[23:16];
                if (iomem_wstrb[3]) clockO[31:24] <= iomem_wdata[31:24];
            end

            if (iomem_valid && !iomem_ready && iomem_addr[31:8] == 24'h030003) begin
                iomem_ready <= 1;
                iomem_rdata <= encoderValueI;
                writeEncoderI<= iomem_wstrb!=0;
                if (iomem_wstrb[0]) encoderDataI[ 7: 0] <= iomem_wdata[ 7: 0];
                if (iomem_wstrb[1]) encoderDataI[15: 8] <= iomem_wdata[15: 8];
                if (iomem_wstrb[2]) encoderDataI[23:16] <= iomem_wdata[23:16];
                if (iomem_wstrb[3]) encoderDataI[31:24] <= iomem_wdata[31:24];
            end

            if (iomem_valid && !iomem_ready && iomem_addr[31:8] == 24'h030004) begin
                iomem_ready <= 1;
                iomem_rdata <= encoderValueD;
                writeEncoderD<= iomem_wstrb!=0;
                if (iomem_wstrb[0]) encoderDataD[ 7: 0] <= iomem_wdata[ 7: 0];
                if (iomem_wstrb[1]) encoderDataD[15: 8] <= iomem_wdata[15: 8];
                if (iomem_wstrb[2]) encoderDataD[23:16] <= iomem_wdata[23:16];
                if (iomem_wstrb[3]) encoderDataD[31:24] <= iomem_wdata[31:24];
            end

            if (iomem_valid && !iomem_ready && iomem_addr[31:8] == 24'h030005) begin
                iomem_ready <= 1;
                if (iomem_wstrb[0]) pwm_connectorIF[ 7: 0] <= iomem_wdata[ 7: 0];
                if (iomem_wstrb[1]) pwm_connectorIF[15: 8] <= iomem_wdata[15: 8];
                if (iomem_wstrb[2]) pwm_connectorIF[23:16] <= iomem_wdata[23:16];
                if (iomem_wstrb[3]) pwm_connectorIF[31:24] <= iomem_wdata[31:24];
            end

            if (iomem_valid && !iomem_ready && iomem_addr[31:8] == 24'h030006) begin
                iomem_ready <= 1;
                if (iomem_wstrb[0]) pwm_connectorIB[ 7: 0] <= iomem_wdata[ 7: 0];
                if (iomem_wstrb[1]) pwm_connectorIB[15: 8] <= iomem_wdata[15: 8];
                if (iomem_wstrb[2]) pwm_connectorIB[23:16] <= iomem_wdata[23:16];
                if (iomem_wstrb[3]) pwm_connectorIB[31:24] <= iomem_wdata[31:24];
            end

            if (iomem_valid && !iomem_ready && iomem_addr[31:8] == 24'h030007) begin
                iomem_ready <= 1;
                if (iomem_wstrb[0]) pwm_connectorDF[ 7: 0] <= iomem_wdata[ 7: 0];
                if (iomem_wstrb[1]) pwm_connectorDF[15: 8] <= iomem_wdata[15: 8];
                if (iomem_wstrb[2]) pwm_connectorDF[23:16] <= iomem_wdata[23:16];
                if (iomem_wstrb[3]) pwm_connectorDF[31:24] <= iomem_wdata[31:24];
            end

            if (iomem_valid && !iomem_ready && iomem_addr[31:8] == 24'h030008) begin
                iomem_ready <= 1;
                if (iomem_wstrb[0]) pwm_connectorDB[ 7: 0] <= iomem_wdata[ 7: 0];
                if (iomem_wstrb[1]) pwm_connectorDB[15: 8] <= iomem_wdata[15: 8];
                if (iomem_wstrb[2]) pwm_connectorDB[23:16] <= iomem_wdata[23:16];
                if (iomem_wstrb[3]) pwm_connectorDB[31:24] <= iomem_wdata[31:24];
            end

            if (iomem_valid && !iomem_ready && iomem_addr[31:8] == 24'h030009) begin
                iomem_ready <= 1;
                pinTest[0]=1;
            end

            
            ///////////////////////////
            // Template Peripheral
            ///////////////////////////
            if (iomem_valid && !iomem_ready && iomem_addr[31:24] == 8'h04) begin
                iomem_ready <= 1;
                iomem_rdata <= 32'h0;
            end
        end
    end

    ///////////////////////////////////
    // Custom Modules
    ///////////////////////////////////

    clock clock (
		.clk         (clk         ),
		.resetn      (resetn      ),
		.clock_out   (clock_out)
	);

    pwm pwmIF (
		.clk         (clk         ),
		.resetn      (resetn      ),
        .pwm_in      (pwm_connectorIF      ),
		.pwm_out     (pinPwmIzqF     )
	);

    pwm pwmIB (
		.clk         (clk         ),
		.resetn      (resetn      ),
        .pwm_in      (pwm_connectorIB      ),
		.pwm_out     (pinPwmIzqB    )
	);

    pwm pwmDF (
		.clk         (clk         ),
		.resetn      (resetn      ),
        .pwm_in      (pwm_connectorDF      ),
		.pwm_out     (pinPwmDerF     )
	);

    pwm pwmDB (
		.clk         (clk         ),
		.resetn      (resetn      ),
        .pwm_in      (pwm_connectorDB      ),
		.pwm_out     (pinPwmDerB    )
	);

    encoder encoderL (
		.clk         (clk         ),
		.resetn      (resetn      ),
        .writeEncoder      (writeEncoderI      ),
        .encoderData      (encoderDataI      ),
		.encoderValue     (encoderValueI     ),
        .pinEncoderF     (pinEncoderIF     ),
        .pinEncoderB     (pinEncoderIB     )
	);

    encoder encoderR (
		.clk         (clk         ),
		.resetn      (resetn      ),
        .writeEncoder      (writeEncoderD      ),
        .encoderData      (encoderDataD     ),
		.encoderValue     (encoderValueD     ),
        .pinEncoderF     (pinEncoderDF     ),
        .pinEncoderB     (pinEncoderDB     )
	);


    ///////////////////////////////////
    // PicoSOC
    ///////////////////////////////////

  picosoc #(
	.BARREL_SHIFTER(0),
	.ENABLE_MULDIV(0),
	.ENABLE_COMPRESSED(0),
	.ENABLE_COUNTERS(0),
	.ENABLE_IRQ_QREGS(1),
	.ENABLE_TWO_STAGE_SHIFT(0),
	.PROGADDR_RESET(32'h0005_0000), // beginning of user space in SPI flash
	.PROGADDR_IRQ(32'h0005_0010),
	.MEM_WORDS(3584),                // use 4KBytes of block RAM by default (8 RAMS)
	.STACKADDR(3584),   /* stack addr = byte offset; stack starts at 0x400, grows downward. Data starts at 0x400+. */
	.ENABLE_IRQ(1)
) soc (
  .clk          (clk         ),
  .resetn       (resetn      ),

  .ser_tx       (uart_out       ),
  .ser_rx       (uart_in       ),

	.irq_5        (1'b0),
	.irq_6        (1'b0        ),
	.irq_7        (1'b0        ),

	.iomem_valid  (iomem_valid ),
	.iomem_ready  (iomem_ready ),
	.iomem_wstrb  (iomem_wstrb ),
	.iomem_addr   (iomem_addr  ),
	.iomem_wdata  (iomem_wdata ),
	.iomem_rdata  (iomem_rdata )
);
endmodule