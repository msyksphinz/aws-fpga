//---------------------------------------------------------------------------
//##module FIFO
//---------------------------------------------------------------------------
module fifo
  #(
	parameter width   = 32,              // Data bus width
	parameter widthad = 4                // Address bus width
	)
   (
	input logic 			 CLK,
	input logic 			 nRST,
	input logic [width-1:0]  D,
	output logic [width-1:0] Q,
	input logic 			 WR,
	input logic 			 RD,
	output logic 			 FULL,
	output logic 			 EMPTY
	);
//---------------------------------------------------------------------------
//#parameter
//---------------------------------------------------------------------------
localparam numwords = 2**widthad;      // Number of words  2^lpm_widthad

//---------------------------------------------------------------------------
//#wire
//---------------------------------------------------------------------------
logic [width-1:0]           Q;
logic [widthad:0]           CNT;         //Num Of Used Buffer
logic                       FULL;
logic                       EMPTY;
logic [widthad-1:0]         WP;          //Write Pointer
logic [widthad-1:0]         RP;          //Read Pointer

//---------------------------------------------------------------------------
//#reg
//---------------------------------------------------------------------------
logic [widthad:0]           WCNT;
logic [widthad:0]           RCNT;
logic [width-1:0]           DATA    [numwords-1:0];

//---------------------------------------------------------------------------
//#assign
//---------------------------------------------------------------------------
assign  Q = DATA[RP];
assign  CNT = WCNT - RCNT;
assign  FULL  = CNT[widthad];
assign  EMPTY = (CNT==0);
assign  WP[widthad-1:0] = WCNT[widthad-1:0];
assign  RP[widthad-1:0] = RCNT[widthad-1:0];

//---------------------------------------------------------------------------
//#always
//---------------------------------------------------------------------------
always  @( posedge CLK or negedge nRST ) begin
  if ( !nRST ) begin
    WCNT      <= 0;
    RCNT      <= 0;
  end
  else begin
    if(WR & ~FULL)begin
      DATA[WP] <= D;
      WCNT <= WCNT + 1;
    end
    if(RD & ~EMPTY)begin
      RCNT <= RCNT + 1;
    end
  end
end
endmodule //End of FIFO
