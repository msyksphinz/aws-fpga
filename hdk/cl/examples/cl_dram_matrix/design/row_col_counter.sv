`ifdef VCS
`default_nettype none
`endif // VCS

module row_col_counter
  #(
    parameter col_width = 5,
    parameter row_width = 5,
    parameter col_size  = 16,
    parameter row_size  = 16
    )
(
 input logic                   clk,
 input logic                   rst_n,
 input logic                   update,
 output logic [col_width-1: 0] col_idx,
 output logic [row_width-1: 0] row_idx
 );

always_ff @ (posedge clk) begin
  if (!rst_n) begin
    col_idx <= 'h0;
    row_idx <= 'h0;
  end else begin
    if (update) begin
      if (row_idx == row_size-1) begin
        row_idx <= 'h0;
        if (col_idx == col_size-1) begin
          col_idx <= 'h0;
        end else begin
          col_idx <= col_idx + 'h1;
        end
      end else begin
        row_idx <= row_idx + 'h1;
      end
    end // if (update)
  end // else: !if(!rst_n)
end // always_ff @


endmodule // row_col_counter

`ifdef VCS
`default_nettype wire
`endif // VCS
