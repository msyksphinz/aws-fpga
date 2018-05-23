`default_nettype none

module cl_dram_matrix_calc
  (
   input logic      clk,
   input logic      rst_n,
   axi_bus_t.slave  axi_if,
   cfg_bus_t.master cfg_if
   );

logic [ 2: 0]       ar_state;
localparam ar_init        = 3'b000;
localparam ar_req_mat1    = 3'b100;
localparam ar_wait_mat1   = 3'b101;
localparam ar_req_mat2    = 3'b110;
localparam ar_wait_mat2   = 3'b111;

logic [ 5: 0] ar_counter;
logic         job_finished;

logic [ 4: 0] ar_col_idx, ar_row_idx;
logic [63: 0] mat1_addr, mat1_addr_init;
logic [63: 0] mat2_addr, mat2_addr_init;
logic [63: 0] dst_addr_init;

logic         ar_update;

row_col_counter
  #(.col_width ( 5),
    .row_width ( 5),
    .col_size  (16),
    .row_size  (16))
u_ar_counter
  (
   .clk     (clk),
   .rst_n   (rst_n),

   .update  (ar_update),
   .col_idx (ar_col_idx),
   .row_idx (ar_row_idx)
   );

logic         axi_if_ar_fire; assign axi_if_ar_fire = axi_if.arvalid & axi_if.arready;
logic         axi_if_r_fire;  assign axi_if_r_fire  = axi_if.rvalid  & axi_if.rready;
logic         axi_if_aw_fire; assign axi_if_aw_fire = axi_if.awvalid & axi_if.awready;
logic         axi_if_w_fire;  assign axi_if_w_fire  = axi_if.wvalid  & axi_if.wready;

assign ar_update = (ar_state == ar_wait_mat2) && axi_if_ar_fire;

always_ff @ (posedge clk) begin
  if (!rst_n) begin
    ar_counter <= 'h0;
  end else begin
    if (ar_update) begin
      if (ar_counter != 15) ar_counter <= ar_counter + 'h1;
      else                  ar_counter <= 'h0;
    end
  end
end

// ====================================
// cfg_if bus check
// ====================================
always_ff @ (posedge clk) begin
  if (!rst_n) begin
    cfg_if.ack <= 1'b0;
    cfg_if.rdata <= 32'h0000_0000;
  end else begin
	if (cfg_if.rd || cfg_if.wr) begin
      cfg_if.rdata <= job_finished;
      cfg_if.ack   <= 1'b1;
    end else begin
      cfg_if.ack   <= 1'b0;
    end
  end // else: !if(!rst_n)
end // always_ff @

// ====================================
// matrix address setting
// ====================================
always_ff @ (posedge clk) begin
  if (!rst_n) begin
    mat1_addr_init <= 64'h0000_0000_0000_0000;
    mat2_addr_init <= 64'h0000_0000_0000_0000;
    dst_addr_init  <= 64'h0000_0000_0000_0000;
  end else begin
	if (cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h10) begin
      mat1_addr_init[31: 0] <= cfg_if.wdata[31: 0];
    end
	if (cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h14) begin
      mat1_addr_init[63:32] <= cfg_if.wdata[31: 0];
    end

	if (cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h18) begin
      mat2_addr_init[31: 0] <= cfg_if.wdata[31: 0];
    end
	if (cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h1c) begin
      mat2_addr_init[63:32] <= cfg_if.wdata[31: 0];
    end

	if (cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h20) begin
      dst_addr_init[31: 0] <= cfg_if.wdata[31: 0];
    end
	if (cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h24) begin
      dst_addr_init[63:32] <= cfg_if.wdata[31: 0];
    end

  end // else: !if(!rst_n)
end // always_ff @


// ====================================
// axi_bus address request
// ====================================
always_ff @ (posedge clk) begin
  if (!rst_n) begin
	axi_if.arvalid <= 1'b0;
	axi_if.araddr  <= 64'h0;

    ar_state <= ar_init;

    mat1_addr <= 32'h0000_0000;;
    mat2_addr <= 32'h0000_0000;;
  end else begin
    case (ar_state)
      ar_init: begin
	    if (cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h00 &&
		    !axi_if.arvalid) begin
          ar_state <= ar_req_mat1;
          mat1_addr <= mat1_addr_init;
          mat2_addr <= mat2_addr_init;
        end // else: !if(cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h00 &&...
      end // case: ar_init

      ar_req_mat1 : begin
	  	axi_if.arvalid <= 1'b1;
	  	axi_if.araddr  <= mat1_addr;
		axi_if.arid    <= 16'b0;     // Only 1 outstanding command
		axi_if.arlen   <= 8'h00;     // Always 1 burst
		axi_if.arsize  <= 3'b111;    // Always 128 bytes

        ar_state <= ar_wait_mat1;
      end

      ar_wait_mat1 : begin
	    if (axi_if_ar_fire) begin
		  axi_if.arvalid <= 1'b0;
          ar_state <= ar_req_mat2;
	    end
      end

      ar_req_mat2: begin

	  	axi_if.arvalid <= 1'b1;
	  	axi_if.araddr  <= mat2_addr;
		axi_if.arid    <= 16'b0;     // Only 1 outstanding command
		axi_if.arlen   <= 8'h00;     // Always 1 burst
		axi_if.arsize  <= 3'b010;    // Always 4 bytes

        ar_state <= ar_wait_mat2;
      end // case: ar_req_mat2

      ar_wait_mat2: begin

	    if (axi_if_ar_fire) begin

          axi_if.arvalid <= 1'b0;
          if (ar_counter != 15) begin
	  	    mat2_addr <= mat2_addr + 64'h40;  // Proceed 64-byte
            ar_state     <= ar_req_mat2;
          end else begin
            ar_state <= ar_init;
          end // else: !if(ar_counter != 15)

		  axi_if.arid    <= 16'b0;   // Only 1 outstanding command
		  axi_if.arlen   <= 8'h00;   // Always 1 burst
		  axi_if.arsize  <= 3'b010;  // Always 4 bytes

        end // if (axi_if_ar_fire)
      end // case: ar_wait_col
    endcase // case (state)
  end // else: !if(!rst_n)
end // always_ff @


logic [31: 0] matrix_row_data[15: 0];
logic         matrix_col_val;
logic [ 1: 0] rcv_state;
logic [ 4: 0] rcv_count;
logic [63: 0] mul_result;

logic [ 4: 0] calc_count;
logic [63: 0] dotp_result;
logic [31: 0] matrix_row_data_in;
logic         mult_vld;

assign job_finished = (calc_count == 16);

localparam rcv_state_row    = 2'b00;
localparam rcv_state_col    = 2'b01;
localparam rcv_state_store  = 2'b10;
integer       idx;
always_ff @ (posedge clk) begin
  if (!rst_n) begin
    for (idx = 0; idx < 16; idx=idx+1) begin
      matrix_row_data[idx] <= 32'h0000_0000;
    end
    matrix_col_val <= 1'b0;
    rcv_state <= rcv_state_row;
    rcv_count <= 5'h00;
  end else begin
	if (cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h00) begin
      rcv_state <= rcv_state_row;
    end else begin
      case (rcv_state)
        rcv_state_row  : begin
          if (axi_if_r_fire) begin
            matrix_row_data[ 0] <= axi_if.rdata[32* 0+31:32* 0];
            matrix_row_data[ 1] <= axi_if.rdata[32* 1+31:32* 1];
            matrix_row_data[ 2] <= axi_if.rdata[32* 2+31:32* 2];
            matrix_row_data[ 3] <= axi_if.rdata[32* 3+31:32* 3];
            matrix_row_data[ 4] <= axi_if.rdata[32* 4+31:32* 4];
            matrix_row_data[ 5] <= axi_if.rdata[32* 5+31:32* 5];
            matrix_row_data[ 6] <= axi_if.rdata[32* 6+31:32* 6];
            matrix_row_data[ 7] <= axi_if.rdata[32* 7+31:32* 7];
            matrix_row_data[ 8] <= axi_if.rdata[32* 8+31:32* 8];
            matrix_row_data[ 9] <= axi_if.rdata[32* 9+31:32* 9];
            matrix_row_data[10] <= axi_if.rdata[32*10+31:32*10];
            matrix_row_data[11] <= axi_if.rdata[32*11+31:32*11];
            matrix_row_data[12] <= axi_if.rdata[32*12+31:32*12];
            matrix_row_data[13] <= axi_if.rdata[32*13+31:32*13];
            matrix_row_data[14] <= axi_if.rdata[32*14+31:32*14];
            matrix_row_data[15] <= axi_if.rdata[32*15+31:32*15];

            rcv_state <= rcv_state_col;
            rcv_count <= 5'h00;
          end // if (axi_if_r_fire)
        end // case: rcv_state_row

        rcv_state_col : begin
          if (axi_if_r_fire) begin
            matrix_col_val <= 1'b1;
            if (rcv_count < 15) begin
              rcv_count <= rcv_count + 1;
            end else begin
              rcv_count <= 0;
            end
          end else begin
            matrix_col_val <= 1'b0;
          end // else: !if(axi_if_r_fire)
        end // case: rcv_state_col
      endcase // case (rcv_state)
    end // else: !if(cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h00)
  end // else: !if(!rst_n)
end // always_ff @

logic fifo_wr, fifo_empty, fifo_full;
assign fifo_wr = (rcv_state == rcv_state_col) & axi_if_r_fire;

logic [63: 0] fifo_rd_data;

assign axi_if.rready = !fifo_full;

logic [31: 0] rdata_selected_32;
assign rdata_selected_32 = (mat2_addr_init[5:2] == 4'h0) ? axi_if.rdata[32* 0+31:32* 0] :
                           (mat2_addr_init[5:2] == 4'h1) ? axi_if.rdata[32* 1+31:32* 1] :
                           (mat2_addr_init[5:2] == 4'h2) ? axi_if.rdata[32* 2+31:32* 2] :
                           (mat2_addr_init[5:2] == 4'h3) ? axi_if.rdata[32* 3+31:32* 3] :
                           (mat2_addr_init[5:2] == 4'h4) ? axi_if.rdata[32* 4+31:32* 4] :
                           (mat2_addr_init[5:2] == 4'h5) ? axi_if.rdata[32* 5+31:32* 5] :
                           (mat2_addr_init[5:2] == 4'h6) ? axi_if.rdata[32* 6+31:32* 6] :
                           (mat2_addr_init[5:2] == 4'h7) ? axi_if.rdata[32* 7+31:32* 7] :
                           (mat2_addr_init[5:2] == 4'h8) ? axi_if.rdata[32* 8+31:32* 8] :
                           (mat2_addr_init[5:2] == 4'h9) ? axi_if.rdata[32* 9+31:32* 9] :
                           (mat2_addr_init[5:2] == 4'hA) ? axi_if.rdata[32*10+31:32*10] :
                           (mat2_addr_init[5:2] == 4'hB) ? axi_if.rdata[32*11+31:32*11] :
                           (mat2_addr_init[5:2] == 4'hC) ? axi_if.rdata[32*12+31:32*12] :
                           (mat2_addr_init[5:2] == 4'hD) ? axi_if.rdata[32*13+31:32*13] :
                           (mat2_addr_init[5:2] == 4'hE) ? axi_if.rdata[32*14+31:32*14] :
                           (mat2_addr_init[5:2] == 4'hF) ? axi_if.rdata[32*15+31:32*15] :
                           32'h0000_0000;

fifo u_input_fifo
(
 .CLK   (clk),
 .nRST  (rst_n),
 .D     (rdata_selected_32),
 .Q     (fifo_rd_data),
 .WR    (fifo_wr),
 .RD    (!fifo_empty),
 .FULL  (fifo_full),
 .EMPTY (fifo_empty)
 );

always_ff @ (posedge clk) begin
  if (!rst_n) begin
    dotp_result <= 64'h0;
    calc_count <= 5'h00;
    mult_vld  <= 1'b0;
  end else begin
    if (cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h00) begin
      dotp_result <= 64'h0;
      calc_count  <= 5'h00;
      mult_vld    <= 1'b0;
    end else begin
      if (!fifo_empty) begin
        dotp_result <= dotp_result + mul_result;
        calc_count  <= calc_count  + 5'h01;
        if (calc_count == 15) begin
          mult_vld    <= 1'b1;
        end else begin
          mult_vld    <= 1'b0;
        end
      end else begin
        mult_vld  <= 1'b0;
      end // else: !if(!fifo_empty)
    end // else: !if(cfg_if.wr && cfg_if.addr[ 7: 0] == 8'h00)
  end // else: !if(!rst_n)
end // always_ff @


always_comb begin
  case (calc_count)
    4'd00   : matrix_row_data_in = matrix_row_data[ 0];
    4'd01   : matrix_row_data_in = matrix_row_data[ 1];
    4'd02   : matrix_row_data_in = matrix_row_data[ 2];
    4'd03   : matrix_row_data_in = matrix_row_data[ 3];
    4'd04   : matrix_row_data_in = matrix_row_data[ 4];
    4'd05   : matrix_row_data_in = matrix_row_data[ 5];
    4'd06   : matrix_row_data_in = matrix_row_data[ 6];
    4'd07   : matrix_row_data_in = matrix_row_data[ 7];
    4'd08   : matrix_row_data_in = matrix_row_data[ 8];
    4'd09   : matrix_row_data_in = matrix_row_data[ 9];
    4'd10   : matrix_row_data_in = matrix_row_data[10];
    4'd11   : matrix_row_data_in = matrix_row_data[11];
    4'd12   : matrix_row_data_in = matrix_row_data[12];
    4'd13   : matrix_row_data_in = matrix_row_data[13];
    4'd14   : matrix_row_data_in = matrix_row_data[14];
    4'd15   : matrix_row_data_in = matrix_row_data[15];
    default : matrix_row_data_in = 32'h0000_0000;
  endcase // case (calc_count)
end // always_comb


assign mul_result = {{32{matrix_row_data_in[31]}}, matrix_row_data_in} *
                    {{32{fifo_rd_data[31]}},       fifo_rd_data      };

assign axi_if.wdata = dotp_result;

logic [63: 0] fifo_dotp_result;
logic         fifo_mult_empty, fifo_mult_full;

fifo
  #(.width(64))
u_mult_result
(
 .CLK   (clk),
 .nRST  (rst_n),
 .D     (dotp_result),
 .Q     (fifo_dotp_result),
 .WR    (mult_vld),
 .RD    (!fifo_mult_empty),
 .FULL  (fifo_mult_full),
 .EMPTY (fifo_mult_empty)
 );

logic         store_state;
localparam store_state_init = 1'b0;
localparam store_state_wait = 1'b1;

always_ff @ (posedge clk) begin
  if (!rst_n) begin
	axi_if.awvalid <= 1'b0;
	axi_if.wvalid  <= 1'b0;

    store_state <= store_state_init;
  end else begin
    case (store_state)
      store_state_init : begin
        if (!fifo_mult_empty) begin
	      axi_if.awvalid <= 1'b1;
		  axi_if.wvalid  <= 1'b1;
		  axi_if.awlen   <= 8'h00;   // Always 1 burst
		  axi_if.awsize  <= 3'b011;  // Always 8 bytes
          axi_if.wstrb   <= 4'b0000; // Always lower 32-bit
          axi_if.awaddr  <= dst_addr_init;

          store_state <= store_state_wait;
        end else begin
	      axi_if.awvalid <= 1'b0;
		  axi_if.wvalid  <= 1'b0;
        end
      end // case: store_state_init
      store_state_wait : begin
        if (axi_if.awready) axi_if.awvalid <= 1'b0;
        if (axi_if.wready ) axi_if.wvalid  <= 1'b0;
        if (axi_if.wvalid & axi_if.wready) begin
          // Store Complete
          axi_if.awaddr <= axi_if.awaddr + 64'h8;
          store_state <= store_state_init;
        end
      end
    endcase // case (store_state)
  end // else: !if(!rst_n)
end // always_ff @


// always_ff @ (negedge clk) begin
//   if (u_input_fifo.WR) $write ("%t : [matrix] input FIFO write %08x\n", $time, u_input_fifo.D);
//   if (u_input_fifo.RD) $write ("%t : [matrix] input FIFO read  %08x\n", $time, u_input_fifo.Q);
// end


always_ff @ (negedge clk) begin
  // if (mult_vld)         $write ("%t : [matrix] result FIFO inserted %016x\n", $time, dotp_result     );
  if (!fifo_mult_empty) $write ("%t : [matrix] result FIFO read     %016x\n", $time, fifo_dotp_result);
end

always @ (negedge clk) begin
  if (cfg_if.wr) $display ("%t : [axi_mstr_cfg_bus W] ADDR=%x", $time, cfg_if.addr);
  if (cfg_if.rd) $display ("%t : [axi_mstr_cfg_bus R] ADDR=%x", $time, cfg_if.addr);
end // always @ (negedge clk)


always_ff @ (negedge clk) begin
  if (axi_if_ar_fire) $display ("%t : [cl_axi_mstr_bus AR] LEN=%d SIZE=%d ADDR=%x", $time, axi_if.arlen, axi_if.arsize, axi_if.araddr);
  if (axi_if_r_fire)  $display ("%t : [cl_axi_mstr_bus  R] DATA=%x",                $time, axi_if.rdata);
  if (axi_if_aw_fire) $display ("%t : [cl_axi_mstr_bus AW] LEN=%d SIZE=%d ADDR=%x", $time, axi_if.awlen, axi_if.awsize, axi_if.awaddr);
  if (axi_if_w_fire)  $display ("%t : [cl_axi_mstr_bus  W] STB=%x DATA=%x",         $time, axi_if.wstrb, axi_if.wdata);
end // always @ (negedge clk)

endmodule // cl_dram_matrix_calc

`default_nettype wire
