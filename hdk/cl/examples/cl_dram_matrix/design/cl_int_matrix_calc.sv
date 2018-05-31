`default_nettype none

module cl_int_matrix_calc
   (
    input               clk,
    input               rst_n,

    input               matrix_calc_done,

    // input [31:0]        cfg_addr,
    // input [31:0]        cfg_wdata,
    // input               cfg_wr,
    // input               cfg_rd,
    // output logic        tst_cfg_ack,
    // output logic [31:0] tst_cfg_rdata,

    output [15:0]       cl_sh_irq_req,
    input [15:0]        sh_cl_irq_ack
    );

// logic [7:0]             cfg_vec_num;
// logic                   cfg_wr_stretch;
// logic                   cfg_rd_stretch;

// logic [7:0]             cfg_addr_q;        //Only care about lower 8-bits of address, upper bits are decoded somewhere else
// logic [31:0]            cfg_wdata_q;
//
// logic [31:0]            int_cfg_rdata;
//
//
// always_ff @(negedge rst_n or posedge clk) begin
//   if (!rst_n) begin
//     cfg_wr_stretch <= 0;
//     cfg_rd_stretch <= 0;
//     cfg_addr_q <= 0;
//     cfg_wdata_q <= 0;
//   end else begin
//     cfg_wr_stretch <= cfg_wr || (cfg_wr_stretch && !tst_cfg_ack);
//     cfg_rd_stretch <= cfg_rd || (cfg_rd_stretch && !tst_cfg_ack);
//     if (cfg_wr||cfg_rd) begin
//       cfg_addr_q <= cfg_addr[7:0];
//       cfg_wdata_q <= cfg_wdata;
//     end
//   end // else: !if(!rst_n)
// end // always_ff @


// //Readback mux
// always_ff @(negedge rst_n or posedge clk)
//   if (!rst_n)
//     tst_cfg_rdata <= 0;
//   else
//     tst_cfg_rdata <= int_cfg_rdata;
//
// //Ack for cycle
// always_ff @(negedge rst_n or posedge clk)
//   if (!rst_n)
//     tst_cfg_ack <= 0;
//   else
//     tst_cfg_ack <= ((cfg_wr_stretch||cfg_rd_stretch) && !tst_cfg_ack);


logic [15: 0]           w_int_input;
assign w_int_input = {15'h000, matrix_calc_done};

logic [15: 0]           int_ack;
logic [15: 0]           int_trig;
logic [15: 0]           int_wait;
logic [15: 0]           int_done;

// Addr 0x0 - Control Register

// Bit 15:0  - Trigger (Write) / Interrupt Waiting (Read)
// Bit 31:16 - Interrupt Done (W1C)
// assign int_cfg_rdata = {int_done, int_wait};

lib_pipe #(.WIDTH(32), .STAGES(4)) PIPE_IN (.clk (clk),
                                            .rst_n (rst_n),
                                            .in_bus({int_trig, sh_cl_irq_ack}),
                                            .out_bus({cl_sh_irq_req, int_ack})
                                            );

generate
  for (genvar idx = 0; idx < 16; idx++) begin

    // Create the Edge
    always_ff @(negedge rst_n or posedge clk) begin
      if (!rst_n) begin
        int_trig[idx] <= 1'b0;
      end else begin
        int_trig[idx] <= int_trig[idx] ? 1'b0 :
                         w_int_input[idx] ? 1'b1 :
                         int_trig[idx];
      end
    end

    // Interrupt Waiting
    always_ff @(negedge rst_n or posedge clk) begin
      if (!rst_n) begin
        int_wait[idx] <= 1'b0;
      end else begin
        int_wait[idx] <= int_trig[idx] ? 1'b1 :
                         int_wait[idx] && int_ack[idx] ? 1'b0 :
                         int_wait[idx];
      end
    end


    always_ff @(negedge rst_n or posedge clk) begin
      if (!rst_n) begin
        int_done[idx] <= 1'b0;
      end else begin
        int_done[idx] <= int_trig[idx] ? 1'b0 :
                         int_wait[idx] && int_ack[idx] ? 1'b1 :
                         int_done[idx];
      end
    end
  end // for (int idx = 0; idx < 16; idx++)
endgenerate

endmodule // cl_int_matrix_calc

`default_nettype wire
