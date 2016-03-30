`timescale 1 ns / 1 ps

module prdoor #(
  parameter WIDTH   = 1,
  parameter vDOOR   = {WIDTH{1'b0}}
)
(
  input   wire  [WIDTH  - 1 : 0]  I,
  input   wire                    S,
  output  wire  [WIDTH  - 1 : 0]  O
);

wire [WIDTH - 1 : 0] DOOR;
assign DOOR = vDOOR;
assign O = S ? I : DOOR;

endmodule
