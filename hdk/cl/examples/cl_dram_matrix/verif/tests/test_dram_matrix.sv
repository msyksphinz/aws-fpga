// Amazon FPGA Hardware Development Kit
//
// Copyright 2016 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Amazon Software License (the "License"). You may not use
// this file except in compliance with the License. A copy of the License is
// located at
//
//    http://aws.amazon.com/asl/
//
// or in the "license" file accompanying this file. This file is distributed on
// an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, express or
// implied. See the License for the specific language governing permissions and
// limitations under the License.


module test_dram_matrix ();

import tb_type_defines_pkg::*;
`include "cl_common_defines.vh" // CL Defines with register addresses

int error_count;
int timeout_count;
int fail;
logic [3:0] status;

//transfer1 - length less than 64 byte.
int         matrix_size = 256 * 4;

logic       ddr_ready;
logic       rdata;

logic [31: 0] datasets1[255: 0];
logic [31: 0] datasets2[255: 0];

initial begin
  $readmemh ("datasets1.txt", datasets1);
  $readmemh ("datasets2.txt", datasets2);
end

initial begin

logic [63:0] host_memory_buffer_address;


  tb.power_up(.clk_recipe_a(ClockRecipe::A1),
              .clk_recipe_b(ClockRecipe::B0),
              .clk_recipe_c(ClockRecipe::C0));

  tb.nsec_delay(1000);
  tb.poke_stat(.addr(8'h0c), .ddr_idx(0), .data(32'h0000_0000));
  tb.poke_stat(.addr(8'h0c), .ddr_idx(1), .data(32'h0000_0000));
  tb.poke_stat(.addr(8'h0c), .ddr_idx(2), .data(32'h0000_0000));

  // de-select the ATG hardware

  tb.poke_ocl(.addr(64'h130), .data(0));
  tb.poke_ocl(.addr(64'h230), .data(0));
  tb.poke_ocl(.addr(64'h330), .data(0));
  tb.poke_ocl(.addr(64'h430), .data(0));

  // allow memory to initialize
  tb.nsec_delay(27000);

  $display("[%t] : Initializing buffers", $realtime);

  host_memory_buffer_address = 64'h0;

  //Queue data to be transfered to CL DDR
  tb.que_buffer_to_cl(.chan(0), .src_addr(host_memory_buffer_address), .cl_addr(64'h0000_0004_0000_0000), .len(matrix_size) ); // move buffer to DDR 0

  // Put test pattern in host memory
  for (int i = 0 ; i < matrix_size / 4 ; i++) begin
    tb.hm_put_byte(.addr(host_memory_buffer_address+0), .d(datasets1[i][ 7: 0]));
    tb.hm_put_byte(.addr(host_memory_buffer_address+1), .d(datasets1[i][15: 8]));
    tb.hm_put_byte(.addr(host_memory_buffer_address+2), .d(datasets1[i][23:16]));
    tb.hm_put_byte(.addr(host_memory_buffer_address+3), .d(datasets1[i][31:24]));
    host_memory_buffer_address+=4;
  end

  host_memory_buffer_address = 64'h0_0001_0000;

  tb.que_buffer_to_cl(.chan(1), .src_addr(host_memory_buffer_address), .cl_addr(64'h0000_0004_0001_0000), .len(matrix_size) );  // move buffer to DDR 1

  for (int i = 0 ; i < matrix_size / 4 ; i++) begin
    tb.hm_put_byte(.addr(host_memory_buffer_address+0), .d(datasets2[i][ 7: 0]));
    tb.hm_put_byte(.addr(host_memory_buffer_address+1), .d(datasets2[i][15: 8]));
    tb.hm_put_byte(.addr(host_memory_buffer_address+2), .d(datasets2[i][23:16]));
    tb.hm_put_byte(.addr(host_memory_buffer_address+3), .d(datasets2[i][31:24]));
    host_memory_buffer_address+=4;
  end

  $display("[%t] : starting H2C DMA channels ", $realtime);

  //Start transfers of data to CL DDR
  tb.start_que_to_cl(.chan(0));
  tb.start_que_to_cl(.chan(1));
  // tb.start_que_to_cl(.chan(2));
  // tb.start_que_to_cl(.chan(3));

  // wait for dma transfers to complete
  timeout_count = 0;
  do begin
    status[0] = tb.is_dma_to_cl_done(.chan(0));
    status[1] = tb.is_dma_to_cl_done(.chan(1));
    // status[2] = tb.is_dma_to_cl_done(.chan(2));
    // status[3] = tb.is_dma_to_cl_done(.chan(3));
    #10ns;
    timeout_count++;
  end while ((status != 4'hf) && (timeout_count < 4000));

  if (timeout_count >= 4000) begin
    $display("[%t] : *** ERROR *** Timeout waiting for dma transfers from cl", $realtime);
    error_count++;
  end

  if (timeout_count >= 1000) begin
    $display("[%t] : *** ERROR *** Timeout waiting for dma transfers from cl", $realtime);
    error_count++;
  end

  #1us;

  calc_dot_product (64'h00000004_00000000, 64'h00000004_00010000, 64'h00000004_00020000);
  calc_dot_product (64'h00000004_00000000, 64'h00000004_00010004, 64'h00000004_00020008);
  calc_dot_product (64'h00000004_00000000, 64'h00000004_00010008, 64'h00000004_00020010);
  calc_dot_product (64'h00000004_00000000, 64'h00000004_0001000c, 64'h00000004_00020018);

  calc_dot_product (64'h00000004_00000040, 64'h00000004_00010004, 64'h00000004_00020028);
  calc_dot_product (64'h00000004_00000080, 64'h00000004_00010008, 64'h00000004_00020030);
  calc_dot_product (64'h00000004_000000c0, 64'h00000004_0001000c, 64'h00000004_00020038);
  calc_dot_product (64'h00000004_00000100, 64'h00000004_00010010, 64'h00000004_00020038);

  // Power down
  #500ns;
  tb.power_down();

  $finish;
end // initial begin

endmodule // test_dram_hello


task calc_dot_product (input logic [63: 0] mat1_addr,
                       input logic [63: 0] mat2_addr,
                       input logic [63: 0] dst_addr);

logic [31: 0]                              matrix_finished;

  import tb_type_defines_pkg::*;

  $display ("Calculation Dot Product Start...");

  // Set Matrix1 Address
  tb.poke(.addr(32'h0510), .data(mat1_addr[31: 0]), .size(DataSize::UINT32), .intf(AxiPort::PORT_OCL)); // write register
  tb.poke(.addr(32'h0514), .data(mat1_addr[63:32]), .size(DataSize::UINT32), .intf(AxiPort::PORT_OCL)); // write register

  // Set Matrix2 Address
  tb.poke(.addr(32'h0518), .data(mat2_addr[31: 0]), .size(DataSize::UINT32), .intf(AxiPort::PORT_OCL)); // write register
  tb.poke(.addr(32'h051c), .data(mat2_addr[63:32]), .size(DataSize::UINT32), .intf(AxiPort::PORT_OCL)); // write register

  // Set Destination Address
  tb.poke(.addr(32'h0520), .data(dst_addr [31: 0]), .size(DataSize::UINT32), .intf(AxiPort::PORT_OCL)); // write register
  tb.poke(.addr(32'h0524), .data(dst_addr [63:32]), .size(DataSize::UINT32), .intf(AxiPort::PORT_OCL)); // write register


  tb.poke(.addr(`HELLO_WORLD_REG_ADDR), .data(32'hDEAD_BEEF), .size(DataSize::UINT16), .intf(AxiPort::PORT_OCL)); // write register

  $display ("Write Finished. Reading ...");

  matrix_finished = 0;

  while (matrix_finished == 0) begin
    tb.peek(.addr(`HELLO_WORLD_REG_ADDR), .data(matrix_finished), .size(DataSize::UINT16), .intf(AxiPort::PORT_OCL));         // start read & write
    $display ("Reading 0x%x from address 0x%x", matrix_finished, `HELLO_WORLD_REG_ADDR);
    #1us;
  end
endtask // calc_dot_product
