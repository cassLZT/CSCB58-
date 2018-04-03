module redgreentoe(CLOCK_50,
	// Keys and Switches
    KEY,                    // KEY[3:0]
    SW,                     // SW[3:0]
	 HEX0, HEX2, 
		// VGA output ports
		VGA_CLK,   					    // VGA Clock
		VGA_HS,							    // VGA H_SYNC
		VGA_VS,							    // VGA V_SYNC
		VGA_BLANK_N,				    // VGA BLANK
		VGA_SYNC_N,					    // VGA SYNC
		VGA_R,   						    // VGA Red[9:0]
		VGA_G,	 						    // VGA Green[9:0]
		VGA_B   						    // VGA Blue[9:0]
	);

	// INPUTS for module
	input	CLOCK_50;				    // 50 MHz Clock
	input [3:0] SW;           // SW[9:0] Set X,Y, Colour
	input [3:0] KEY;          // KEY[3:0] switchState X,Y, Colour
	output [6:0] HEX0, HEX2;

	// OUTPUTS for VGA
	output VGA_CLK;   		    // VGA Clock
	output VGA_HS;				    // VGA H_SYNC
	output VGA_VS;				    // VGA V_SYNC
	output VGA_BLANK_N;	      // VGA BLANKOut
	output VGA_SYNC_N;		    // VGA SYNC
	output [9:0] VGA_R;   		// VGA Red[9:0]
	output [9:0] VGA_G;	 		  // VGA Green[9:0]
	output [9:0] VGA_B;   		// VGA Blue[9:0]

	wire resetn;              // Reset Wire to all blocks
	assign resetn = KEY[0] | (win != 0);   // Set Reset to KEY[0]

	                      // Inputs to the controller
	wire [2:0] colour;        // Colour wire
	wire [7:0] x, xOut;       // X Wire
	wire [6:0] y, yOut;       // Y Wire

	wire loadX;               // Load X wire
	wire loadY;               // Load Y wire

	wire SwitchToX, SwitchToY;
	wire [8:0] r1, r2;
	wire [3:0] win;
	
	wire writeEn;             // Write Enable wire
	assign writeEn = ~KEY[1];  // Drive the Enable Signal with KEY[1]

	wire outPLayer;
	wire draw;
	// Create an Instance of a VGA controller - there can be only one!n(resetn),        // Reset on the common signal
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.

	vga_adapter VGA(
		.resetn(resetn),        // Reset on the common signal
		.clock(CLOCK_50),       // Clocked on common 50MHz signal
		.colour(colour),        // Colour
		.x(xOut),               // X Out
		.y(yOut),               // Y Out
		.plot(writeEn),         // Write Enable
		/* Signals for the DAC to drive the monitor. */
		.VGA_R(VGA_R),          // Red
		.VGA_G(VGA_G),          // Green
		.VGA_B(VGA_B),          // Blue
		.VGA_HS(VGA_HS),
		.VGA_VS(VGA_VS),
		.VGA_BLANK(VGA_BLANK_N),
		.VGA_SYNC(VGA_SYNC_N),
		.VGA_CLK(VGA_CLK)
		);
		
	defparam VGA.RESOLUTION = "160x120";           // Resolution
	defparam VGA.MONOCHROME = "FALSE";              // Display Type
	defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;       // Bits/Colour
	defparam VGA.BACKGROUND_IMAGE = "rowmaybe.mif";    // Background

	// Produces signals x,y,colour and writeEn/plotresetn(resetn),        // Reset on the common signal
	// Control Module
	command commandModule(
		.clk(CLOCK_50),           // Synchronize to 50MHz Clock
		.resetn(resetn),          // S1ynchronize to Reset Signal
		.switchState(writeEn),    // Set Switch State to the enable
		.loadX(loadX),            // Set Load X
		.loadY(loadY),            // Set Load Y
		.SX(SwitchToX),
		.SY(SwitchToY),
		.plot(plot),				// Set Plot signal	r1
		);
	
	hex_display h0(.IN(outPlayer), .OUT(HEX0));
	hex_display h2(.IN(win), .OUT(HEX2));
		
	datapath datapathModule(     // Datapath Module
		.clk(CLOCK_50),           // Synchronize to 50MHz Clock
		.resetn(resetn),           // Synchronize to Reset Signal
		.select(SW[3:0]),         // Select X,Y on SW[6:0]
		.loadX(loadX),            // Load X in
		.loadY(loadY),            // Load Y in
		.SwitchToX(SwitchToX),
		.SwitchToY(SwitchToY),
		.x(x),                 // X Out
		.y(y),                 // Y Out
		.colourOut(colour),
		.record1(r1),
		.record2(r2),
		.drawWhich(draw)
		);
	
	checkWin check(
		.r1(r1),
		.r2(r2),
		.clk(CLOCK_50),
		.win(win));

	drawCross drawC(
		.x(x),
		.y(y),
		.draw(draw),
		.plot(plot),
		.resetn(resetn),
		.xOut(xOut),
		.yOut(yOut),
		.clock(CLOCK_50)
		);
	displayPlayer dp(.loadX(loadX), .loadY(loadY), .clock(CLOCK_50), .player(outPlayer));
endmodule


//______________________  Generic VGA Command Module  __________________________
module command(clk, resetn, switchState, loadX, loadY, SY, SX, plot);
	// Inputs
	input clk;                        // Clock Signal
	input resetn;                     // Reset Signal
	input switchState;                // Switch State Signal

	// Outputs
	output reg loadX;                 // Load X
	output reg loadY;                 // Load Y
	output reg plot;                  // Plot
	output reg SX;
	output reg SY;
	
	localparam X = 3'b000;            // X  -> 0
	localparam holdX = 3'b001;        // X' -> 1
	localparam SwitchToY = 3'b010;     //2
	localparam SwitchToYWait = 3'b011; //3
	localparam Y = 3'b100;            //5 
	localparam holdY = 3'b101;         //6
	localparam SwitchToX = 3'b110;       //7
	localparam SwitchToXWait = 3'b111; //8

	reg [2:0] currentState;           // Current State register
	reg [2:0] nextState;              // Next State register

	always @(*)
	// State Table for set X,Y, Colour
	begin: StateTableAssignment
		case (currentState)

			X: nextState = switchState ? holdX : X;
		
			holdX: nextState = switchState ? holdX : SwitchToY;
			
			SwitchToY : nextState = switchState? SwitchToYWait : SwitchToY;
			SwitchToYWait : nextState = switchState ? SwitchToYWait : Y;
	
			Y: nextState = switchState ? holdY : Y;

			holdY: nextState = switchState ? holdY : SwitchToX;
			
			SwitchToX : nextState = switchState? SwitchToXWait : SwitchToX;
			SwitchToXWait : nextState = switchState ? SwitchToXWait : X;
			//default:	nextState = X;
		endcase
	end
	
	always @(*)
	// Enable Signals
	begin: setEnableSignals
		loadX = 1'b0;                  // Load X default to 0
		loadY = 1'b0;                  // Load Y default to 0
		SX = 1'b0;
		SY = 1'b0;
		plot = 1'b0;                   // Plot default to 0
		
		case (currentState)            // Set depending on the current state
			X: begin                     // X STATE
				loadX = 1'b1;              // X -> Load X
				plot = 1'b1; 
				end

			SwitchToY: begin                     
				SY = 1'b1;             
				end
			
			SwitchToX: begin                     
				SX = 1'b1;             
				end
			
			Y: begin                     // Y STATE
				loadY = 1'b1;              // Y -> Load Y
				plot = 1'b1;
				end
		endcase
	end
       
	always @(posedge clk)
	// State FlipFlop Assignment
	begin: stateFlipFlops
		if (! (resetn))                // If Reset is Clicked
			currentState <= X;           // Set the current to X

		else                           // If Reset is not clicked
			currentState <= nextState;   // Go to the next state
	end
endmodule


//______________________ 	output [2:0] colourOut;      // Output for colour Generic VGA Datapath Module  _________________________
module datapath(clk, resetn, select, loadX, loadY, SwitchToX, SwitchToY, x, y,colourOut, record1, record2, drawWhich);
	// Synchronization Inputs
	input clk;                       // Clock Signal
	input resetn;                     // Reset Signal
	
	// X,Y,Colour Inputs
	input [3:0] select;              // Selection for X,Y
	input loadX;                     // Load X
	input loadY;                     // Load Y
	input SwitchToX, SwitchToY;
	// Output Registers
	//output [7:0] xOut;           // Output for X
	//output [7:0] yOut;           // Output1 for Y
	//output [2:0] colourOut;
	
	// Registers
	output reg [2:0] colourOut;
	
	output reg [8:0] record1;
	output reg [8:0] record2;
	output reg drawWhich;
	
	wire [1:0] x,y;
	assign x = select[1:0];
	assign y = select[3:2];
	output x, y;
	
	// Controlling registers
	always @(posedge clk) 
	begin
		if (!resetn) begin              // If reset key is struck:
			//x <= 8'b0;                 // Reset X
			//y <= 8'b0;                 // Reset Y
			colourOut <= 3'b0;
			record1 <= 9'b0;
			record2 <= 9'b0;
		end

		else begin                  	
			
			if (loadX) begin
				colourOut <= 3'b100;
				drawWhich <= 1'b1;
				end
			else if (SwitchToY) begin               	 
				if (x == 2'b00 && y == 2'b00) begin
					record1[8] <= 1'b1;
				end
				else if (x == 2'b00 && y == 2'b01) begin
					record1[7] <= 1'b1;
				end
				else if (x == 2'b01 && y == 2'b00) begin
					record1[5] <= 1'b1;
				end
				else if (x == 2'b00 && y == 2'b10) begin
					record1[6] <= 1'b1;
				end
				else if (x == 2'b01 && y == 2'b01) begin
					record1[4] <= 1'b1;
				end
				else if (x == 2'b01 && y == 2'b10) begin
					record1[3] <= 1'b1;
				end
				else if (x == 2'b10 && y == 2'b00) begin
					record1[2] <= 1'b1;
				end
				else if (x == 2'b10 && y == 2'b01) begin
					record1[1] <= 1'b1;
				end
				else if (x == 2'b10 && y == 2'b10) begin
					record1[0] <= 1'b1;
				end
			end
		
			else if (loadY) begin
				colourOut <= 3'b010;
				drawWhich <= 0;
				end
			else if (SwitchToX) begin           	 
				if (x == 2'b00 && y == 2'b00) begin
					record2[8] <= 1'b1;
				end
				else if (x == 2'b00 && y == 2'b01) begin
					record2[7] <= 1'b1;
				end
				else if (x == 2'b01 && y == 2'b00) begin
					record2[5] <= 1'b1;
				end
				else if (x == 2'b00 && y == 2'b10) begin
					record2[6] <= 1'b1;
				end
				else if (x == 2'b01 && y == 2'b01) begin
					record2[4] <= 1'b1;
				end
				else if (x == 2'b01 && y == 2'b10) begin
					record2[3] <= 1'b1;
				end
				else if (x == 2'b10 && y == 2'b00) begin
					record2[2] <= 1'b1;
				end
				else if (x == 2'b10 && y == 2'b01) begin
					record2[1] <= 1'b1;
				end
				else if (x == 2'b10 && y == 2'b10) begin
					record2[0] <= 1'b1;
				end
			end
		end
	end	
	
endmodule

module checkWin(r1, r2, clk, win);
	input [8:0] r1, r2;
	input clk;
	output reg [3:0] win;
	
	reg p1Win, p2Win;
	
	always @(posedge clk)
	begin
		p1Win <= (& r1[8:6])| (& r1[5:3] )| (& r1[2:0])| (& { r1[8], r1[5], r1[2]} )| (& {r1[7], r1[4], r1[1]} )| (& {r1[6], r1[3], r1[0]} )| (& {r1[8], r1[4], r1[0]}) | (& {r1[6], r1[4], r1[2]});
		p2Win <= (& r2[8:6])| (& r2[5:3] )| (& r2[2:0])| (& { r2[8], r2[5], r2[2]} )| (& {r2[7], r2[4], r2[1]} )| (& {r2[6], r2[3], r2[0]} )| (& {r2[8], r2[4], r2[0]}) | (& {r2[6], r2[4], r2[2]});
		win <= 1'b0;
		if (p1Win && p2Win) begin
			win <= 4'b1101;
		end
		else if (p1Win) begin
			win <= 4'b1011;
		end
		else if (p2Win) begin
			win <= 4'b1010;
		end
		else if ((r1[0] | r2[0])&(r1[1] | r2[1])&(r1[2] | r2[2])&(r1[3] | r2[3])&(r1[4] | r2[4])&(r1[5] | r2[5])&(r1[6] | r2[6])&(r1[7] | r2[7])&(r1[8] | r2[8])) begin
			win <= 4'b1101;
		end
	end
endmodule 
	
	
module drawCross(x, y, draw, plot, clock, resetn, xOut, yOut);
	// Synchronization Inputs
	input clock;                     // Clock Signal
	input resetn;                    // Reset Signal

	// Maping Inputs
	input [7:0] x;                   // X Coordinate
	input [7:0] y;                   // Y Coordinate
	input plot, draw;                      // Plot

	// Outputs
	output reg [7:0] xOut;           // X Out
	output reg [7:0] yOut;           // Y Out

	
	reg [4:0] trace, t1, t2, t3, t4;                 // trace for counter
	reg [1:0] right, left;
	reg [7:0] xCount, yCount;
	
	always @(posedge clock)	
	begin
		if (draw == 1'b1) begin
			right <= 1'b1;
			xCount <= x*40 + 5;
			yCount <= y*40 + 5;
			if (!(resetn))                  
				begin             
					trace <= 5'b00000;         
				end
			if( trace < 5'b11110 && right == 1'b1)
				begin
					xCount <= xCount + 1'b1;    
					yCount <= yCount + 1'b1;   
					trace <= trace + 1;    
				end
			if ( right == 1'b1 && trace >= 5'b11110 )
				begin
					right <= 0;
					left  <= 1'b1;
					trace <= 0;
					xCount <= x*40 +35;
					yCount <= yCount - 5'b11110; 
				end
			if (trace < 5'b11110 && left == 1'b1)
				begin
					xCount <= xCount - 1'b1;
					yCount <= yCount + 1'b1; 
					trace <= 1'b1 + trace;
				end
			if (trace >= 5'b11110 && left == 1'b1)
				begin
					// reset necessary value
					left <= 0;
					trace <= 0;
				end	
		end
		
		else begin
			xCount <= x*40+5;
			yCount <= y*40+5;
			if (t1 < 5'b11110 && t2 == 0 && t3 == 0 && t4 == 0) begin
				xCount <= xCount + 1'b1;
				yCount <= y*40+5;
				t1 <= t1 + 1'b1;
			end

			if (t1 >= 5'b11110 && t2 < 5'b11110 && t3 == 0 && t4 == 0) begin
				xCount <= x*40 + 35;
				yCount <= yCount + 1'b1;
				t2 <= t2 + 1'b1;
			end
			if (t1 >= 5'b11110 && t2 >= 5'b11110 && t3 < 5'b11110 && t4 == 0) begin
				xCount <= xCount - 1'b1;
				yCount <= y*40+35;
				t3 <= t3 + 1'b1;
			end

			if (t1 >= 5'b11110 && t2 >= 5'b11110 && t3 >= 5'b11110 && t4 < 5'b11110) begin
				xCount <= x*40 + 5;
				yCount <= yCount - 1'b1;
				t4 <= t4 + 1'b1;
			end
			
			if (t1 >= 5'b11110 && t2 >= 5'b11110 && t3 >= 5'b11110 && t4 >= 5'b11110) begin
				t1 <= 0 ;
				t2 <= 0 ;
				t3 <= 0 ;
				t4 <= 0 ;
				
			end
			
		end
		xOut <= xCount;
		yOut <= yCount;
	end
	
	

endmodule


module displayPlayer(loadX, loadY, clock, player);
	
	input loadX, loadY, clock;
	
	output reg [3:0] player;
	
	always @(posedge clock)
	begin
		if(loadX == 1'b1)
		begin
			player <= 4'b0001;
		end
		if(loadY == 1'b1)
		begin
			player <= 1'b0010;
		end
	end
endmodule

module hex_display(IN, OUT);
    input [3:0] IN;
	 output reg [7:0] OUT;
	 
	 always @(*)
	 begin
		case(IN[3:0])
			4'b0000: OUT = 7'b1000000;
			4'b0001: OUT = 7'b1111001;
			4'b0010: OUT = 7'b0100100;
			4'b0011: OUT = 7'b0110000;
			4'b0100: OUT = 7'b0011001;
			4'b0101: OUT = 7'b0010010;
			4'b0110: OUT = 7'b0000010;
			4'b0111: OUT = 7'b1111000;
			4'b1000: OUT = 7'b0000000;
			4'b1001: OUT = 7'b0011000;
			4'b1010: OUT = 7'b0001000;
			4'b1011: OUT = 7'b0000011;
			4'b1100: OUT = 7'b1000110;
			4'b1101: OUT = 7'b0100001;
			4'b1110: OUT = 7'b0000110;
			4'b1111: OUT = 7'b0001110;
			
			default: OUT = 7'b0111111;
		endcase

	end
endmodule
