module prctrl #(
  parameter DWIDTH = 32
)
(
  input   wire  [         3 : 0]    ID                     ,
  input   wire                      PR_VALID               ,
  input   wire  [DWIDTH - 1 : 0]    PR_DATA                ,
  //////////////////////////////////////////////////////////
  output  wire                      PR_DONE                ,
  //////////////////////////////////////////////////////////
  input   wire                      clk                    ,
  input   wire                      rstn
);

localparam INIT   = 3'b001;
localparam START  = 3'b010;
localparam END    = 3'b100;

reg  [2:0]  state   ;
reg         rPR_DONE;

wire       Start_Condition;
wire       End_Condition;
// wire [ 3:0] wAccn;
// wire [ 3:0] wType;
// wire [15:0] wCmd ;

// assign     wType = PR_DATA[DWIDTH - 1 : DWIDTH - 4]; // [31:28]
// assign     wAccn = PR_DATA[DWIDTH - 5 : DWIDTH - 8]; // [27:24]
// assign     wCmd  = PR_DATA[        15 : 0];

// assign     Start_Condition = (wType == 4'hD && wAccn == ID && wCmd == 16'hBEEF && PR_VALID == 1'b1);
// assign       End_Condition = (wType == 4'hD && wAccn == ID && wCmd == 16'hDEAD && PR_VALID == 1'b1);

assign  Start_Condition = ((PR_DATA == {4'hD, ID[3:0], 8'h0, 16'hBEEF} && PR_VALID == 1'b1) ? 1'b1 : 1'b0);
assign    End_Condition = ((PR_DATA == {4'hD, ID[3:0], 8'h0, 16'hDEAD} && PR_VALID == 1'b1) ? 1'b1 : 1'b0);

// assign Start_Condition = (PR_DATA  == {124'hBABEFACEDEADBEEFDEADBEEF0000000, ID[3:0]} ? 1'b1: 1'b0);
// assign End_Condition   = (PR_DATA  == {128'hBABEFACEDEADBEEFDEADBEEF0000001, ID[3:0]} ? 1'b1: 1'b0);

assign PR_DONE = rPR_DONE;
always @(posedge clk)
begin
  if (!rstn) begin
    state    <= INIT;
    rPR_DONE <= 1'b0;
  end
  else begin
    state    <= state;
    rPR_DONE <= rPR_DONE;

    case (state)
      INIT: begin
        if (Start_Condition) begin
          state    <= START;
          rPR_DONE <= 1'b0;
        end
      end

      START: begin
        if (End_Condition) begin
          state    <= END;
          rPR_DONE <= 1'b1;
        end
      end

      END: begin
        if (Start_Condition) begin
          state    <= START;
          rPR_DONE <= 1'b0;
        end
      end

      default: begin
        state   <= INIT;
        rPR_DONE <= 1'b0;
      end
    endcase
  end
end

endmodule
