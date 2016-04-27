`timescale 1ns / 1ps

module jit_dispatch #(
  parameter integer NUM_ACCs = 2
)
(
  output  wire            sR_tready   ,
  input   wire            sR_tvalid   ,
  input   wire  [31 : 0]  sR_tdata    ,
//// ---------------------------------------------------------------------------
//input   wire            mR_tready   ,
//output  wire            mR_tvalid   ,
//output  wire  [31 : 0]  mR_tdata    ,
//// ---------------------------------------------------------------------------
  output  wire  [ 3 : 0]  acc1_A1     ,
  output  wire  [ 3 : 0]  acc1_B1     ,
  output  wire  [ 5 : 0]  acc1_C1     ,
  output  wire  [15 : 0]  acc1_R1     ,
  output  wire  [15 : 0]  acc1_R2     ,
  output  wire  [15 : 0]  acc1_R3     ,
//input   wire            acc1_done   ,
//input   wire            acc1_idle   ,
//input   wire            acc1_ready  ,
//output  wire            acc1_start  ,
//// ---------------------------------------------------------------------------
  output  wire  [ 3 : 0]  acc2_A1     ,
  output  wire  [ 3 : 0]  acc2_B1     ,
  output  wire  [ 5 : 0]  acc2_C1     ,
  output  wire  [15 : 0]  acc2_R1     ,
  output  wire  [15 : 0]  acc2_R2     ,
  output  wire  [15 : 0]  acc2_R3     ,
//input   wire            acc2_done   ,
//input   wire            acc2_idle   ,
//input   wire            acc2_ready  ,
//output  wire            acc2_start  ,
//// ---------------------------------------------------------------------------
  output  wire  [ 3 : 0]  acc3_A1     ,
  output  wire  [ 3 : 0]  acc3_B1     ,
  output  wire  [ 5 : 0]  acc3_C1     ,
  output  wire  [15 : 0]  acc3_R1     ,
  output  wire  [15 : 0]  acc3_R2     ,
  output  wire  [15 : 0]  acc3_R3     ,
//input   wire            acc3_done   ,
//input   wire            acc3_idle   ,
//input   wire            acc3_ready  ,
//output  wire            acc3_start  ,
//// ---------------------------------------------------------------------------
  output  wire  [ 3 : 0]  acc4_A1     ,
  output  wire  [ 3 : 0]  acc4_B1     ,
  output  wire  [ 5 : 0]  acc4_C1     ,
  output  wire  [15 : 0]  acc4_R1     ,
  output  wire  [15 : 0]  acc4_R2     ,
  output  wire  [15 : 0]  acc4_R3     ,
//input   wire            acc4_done   ,
//input   wire            acc4_idle   ,
//input   wire            acc4_ready  ,
//output  wire            acc4_start  ,
//// ---------------------------------------------------------------------------
  output  wire  [ 3 : 0]  acc5_A1     ,
  output  wire  [ 3 : 0]  acc5_B1     ,
  output  wire  [ 5 : 0]  acc5_C1     ,
  output  wire  [15 : 0]  acc5_R1     ,
  output  wire  [15 : 0]  acc5_R2     ,
  output  wire  [15 : 0]  acc5_R3     ,
//input   wire            acc5_done   ,
//input   wire            acc5_idle   ,
//input   wire            acc5_ready  ,
//output  wire            acc5_start  ,
//// ---------------------------------------------------------------------------
  output  wire  [ 3 : 0]  acc6_A1     ,
  output  wire  [ 3 : 0]  acc6_B1     ,
  output  wire  [ 5 : 0]  acc6_C1     ,
  output  wire  [15 : 0]  acc6_R1     ,
  output  wire  [15 : 0]  acc6_R2     ,
  output  wire  [15 : 0]  acc6_R3     ,
//input   wire            acc6_done   ,
//input   wire            acc6_idle   ,
//input   wire            acc6_ready  ,
//output  wire            acc6_start  ,
//// ---------------------------------------------------------------------------
  output  wire  [ 3 : 0]  acc7_A1     ,
  output  wire  [ 3 : 0]  acc7_B1     ,
  output  wire  [ 5 : 0]  acc7_C1     ,
  output  wire  [15 : 0]  acc7_R1     ,
  output  wire  [15 : 0]  acc7_R2     ,
  output  wire  [15 : 0]  acc7_R3     ,
//input   wire            acc7_done   ,
//input   wire            acc7_idle   ,
//input   wire            acc7_ready  ,
//output  wire            acc7_start  ,
//// ---------------------------------------------------------------------------
  output  wire  [ 3 : 0]  acc8_A1     ,
  output  wire  [ 3 : 0]  acc8_B1     ,
  output  wire  [ 5 : 0]  acc8_C1     ,
  output  wire  [15 : 0]  acc8_R1     ,
  output  wire  [15 : 0]  acc8_R2     ,
  output  wire  [15 : 0]  acc8_R3     ,
//input   wire            acc8_done   ,
//input   wire            acc8_idle   ,
//input   wire            acc8_ready  ,
//output  wire            acc8_start  ,
//// ---------------------------------------------------------------------------
  input   wire            ACLK        ,
  input   wire            ARESETN
);
//       _    ____ ____           ____
//      / \  / ___/ ___|___   _  |___ \
//     / _ \| |  | |   / __| (_)   __) |
//    / ___ \ |__| |___\__ \  _   / __/
//   /_/   \_\____\____|___/ (_) |_____|
//
generate if (NUM_ACCs == 2) begin
  reg    [ 3 : 0]    racc1_A1   ;
  reg    [ 3 : 0]    racc1_B1   ;
  reg    [ 5 : 0]    racc1_C1   ;
  reg    [15 : 0]    racc1_R1   ;
  reg    [15 : 0]    racc1_R2   ;
  reg    [15 : 0]    racc1_R3   ;
  reg                racc1_done ;

  reg    [ 3 : 0]    racc2_A1   ;
  reg    [ 3 : 0]    racc2_B1   ;
  reg    [ 5 : 0]    racc2_C1   ;
  reg    [15 : 0]    racc2_R1   ;
  reg    [15 : 0]    racc2_R2   ;
  reg    [15 : 0]    racc2_R3   ;
  reg                racc2_done ;


  // ---------------------------------------------------------------------------
  parameter [5:0]    FETCH  = 0, // 1
                     DECODE = 1, // 2
                     TYPEA  = 2, // 4
                     TYPEB  = 3, // 8
                     TYPEC  = 4, // 16
                     START  = 5; // 32
  reg       [5:0]    state, next;

  parameter [39:0]   ACC1  = 0, // 1
                     ACC2  = 1, // 2
                     ACC3  = 2, // 4
                     ACC4  = 3, // 8
                     ACC5  = 4, // 16
                     ACC6  = 5, // 32
                     ACC7  = 6, // 64
                     ACC8  = 7, // 128
                     DONE1 = 8,
                     DONE2 = 9,
                     DONE3 = 10,
                     DONE4 = 11,
                     DONE5 = 12,
                     DONE6 = 13,
                     DONE7 = 14,
                     DONE8 = 15,
                     DONE11= 16,
                     DONE12= 17,
                     DONE13= 18,
                     DONE21= 19,
                     DONE22= 20,
                     DONE23= 21,
                     DONE31= 22,
                     DONE32= 23,
                     DONE33= 24,
                     DONE41= 25,
                     DONE42= 26,
                     DONE43= 27,
                     DONE51= 28,
                     DONE52= 29,
                     DONE53= 30,
                     DONE61= 31,
                     DONE62= 32,
                     DONE63= 33,
                     DONE71= 34,
                     DONE72= 35,
                     DONE73= 36,
                     DONE81= 37,
                     DONE82= 38,
                     DONE83= 39;
  reg    [39:0]      state_acc, next_acc;

  reg    [31 : 0]    rcmd       ;
  reg                rmR_tvalid ;
  reg    [31 : 0]    rmR_tdata  ;

  wire   [ 3 : 0]    wOP        ;
  wire   [ 3 : 0]    wACCn      ;
  wire   [ 3 : 0]    wREGn      ;
  wire   [15 : 0]    wVALUE     ;
  wire   [ 3 : 0]    wVA        ;
  wire   [ 3 : 0]    wVB        ;
  wire   [ 5 : 0]    wVC        ;

  wire               w_tvalid   ;
  wire               w_tready   ;
  wire   [31 : 0]    w_tdata    ;

  assign sR_tready = (state[FETCH] == 1);

  assign wOP       = rcmd[31:28] ;
  assign wACCn     = rcmd[27:24] ;
  assign wREGn     = rcmd[23:20] ;
  assign wVALUE    = rcmd[15: 0] ;

  assign wVA       = rcmd[ 3: 0] ;
  assign wVB       = rcmd[ 7: 4] ;
  assign wVC       = {(rcmd[11: 8] == 4'hF) ? 2'b11 : 2'b00, (wVB == 4'b0) ? 2'b01 : 2'b10, (wVA == 4'b0) ? 2'b01 : 2'b10};

  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc1_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc1_done)        racc1_done <= 1'b1;
  //     if (next_acc[DONE1])  racc1_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc2_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc2_done)        racc2_done <= 1'b1;
  //     if (next_acc[DONE2])  racc2_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // assign acc1_start = (state[TYPEA] && wACCn == 4'd1);
  // assign acc2_start = (state[TYPEA] && wACCn == 4'd2);
  // ---------------------------------------------------------------------------
  assign acc1_A1 = racc1_A1 ;
  assign acc1_B1 = racc1_B1 ;
  assign acc1_C1 = racc1_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_A1 <= 4'd0;
      racc1_B1 <= 4'd0;
      racc1_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd1) begin
        racc1_A1 <= wVA;
        racc1_B1 <= wVB;
        racc1_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_A1 = racc2_A1 ;
  assign acc2_B1 = racc2_B1 ;
  assign acc2_C1 = racc2_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_A1 <= 4'd0;
      racc2_B1 <= 4'd0;
      racc2_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd2) begin
        racc2_A1 <= wVA;
        racc2_B1 <= wVB;
        racc2_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc1_R1   = racc1_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd1 && wREGn == 4'd1)
        racc1_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc1_R2   = racc1_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd1 && wREGn == 4'd2)
        racc1_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc1_R3   = racc1_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd1 && wREGn == 4'd3)
        racc1_R3 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_R1   = racc2_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd2 && wREGn == 4'd1)
        racc2_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_R2   = racc2_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd2 && wREGn == 4'd2)
        racc2_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_R3   = racc2_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd2 && wREGn == 4'd3)
        racc2_R3 <= wVALUE;
    end
  end
  // ACC FSM ACK----------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     state_acc        <=  40'b0;
  //     state_acc[ACC1]  <=   1'b1;
  //   end
  //   else state_acc     <= next_acc;
  // end
  // ACC FSM LOGICAL------------------------------------------------------------
  // always @(*) begin
  //   next_acc  = 40'b0;

  //   case (1'b1)
  //     state_acc[ACC1] : begin
  //       if (racc1_done) next_acc[DONE1] = 1'b1;
  //       else            next_acc[ACC2]  = 1'b1;
  //     end

  //     state_acc[ACC2] : begin
  //       if (racc2_done) next_acc[DONE2] = 1'b1;
  //       else            next_acc[ACC1]  = 1'b1;
  //     end

  //     state_acc[DONE1] : begin
  //       if (w_tready)  next_acc[DONE11] = 1'b1;
  //       else           next_acc[DONE1]  = 1'b1;
  //     end

  //     state_acc[DONE11] : begin
  //       if (w_tready)  next_acc[DONE12] = 1'b1;
  //       else           next_acc[DONE11] = 1'b1;
  //     end

  //     state_acc[DONE12] : begin
  //       if (w_tready)  next_acc[DONE13] = 1'b1;
  //       else           next_acc[DONE12] = 1'b1;
  //     end

  //     state_acc[DONE13] : begin
  //       if (w_tready)  next_acc[ACC2]   = 1'b1;
  //       else           next_acc[DONE13] = 1'b1;
  //     end

  //     state_acc[DONE2] : begin
  //       if (w_tready)  next_acc[DONE21] = 1'b1;
  //       else           next_acc[DONE2]  = 1'b1;
  //     end

  //     state_acc[DONE21] : begin
  //       if (w_tready)  next_acc[DONE22] = 1'b1;
  //       else           next_acc[DONE21] = 1'b1;
  //     end

  //     state_acc[DONE22] : begin
  //       if (w_tready)  next_acc[DONE23] = 1'b1;
  //       else           next_acc[DONE22] = 1'b1;
  //     end

  //     state_acc[DONE23] : begin
  //       if (w_tready)  next_acc[ACC1]   = 1'b1;
  //       else           next_acc[DONE23] = 1'b1;
  //     end

  //   endcase
  // end
  // ---------------------------------------------------------------------------
  // assign w_tdata  = rmR_tdata  ;
  // always @(*) begin
  //   rmR_tdata = 32'd0;

  //   case (1'b1)
  //     next_acc[DONE1] : rmR_tdata = 32'hBABE0001;
  //     next_acc[DONE2] : rmR_tdata = 32'hBABE0002;
  //     default         : rmR_tdata = 32'hBABEFACE;
  //   endcase
  // end
  // // ---------------------------------------------------------------------------
  // assign w_tvalid = ( next_acc[DONE1] || next_acc[DONE11] || next_acc[DONE12] || next_acc[DONE13] ||
  //                     next_acc[DONE2] || next_acc[DONE21] || next_acc[DONE22] || next_acc[DONE23]);
  //============================================================================
  // Do Not Change below
  // rcmd-----------------------------------------------------------------------
  always @(posedge ACLK) begin
    if (!ARESETN) begin
        rcmd <= 15'd0;
    end
    else begin
      if (state[FETCH] == 1 && sR_tvalid == 1)
        rcmd <= sR_tdata;
    end
  end
  // FSM ACK--------------------------------------------------------------------
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      state        <=  7'b0;
      state[FETCH] <=  1'b1;
    end
    else state     <= next;
  end
  // FSM LOGICAL----------------------------------------------------------------
  always @(*) begin
    next = 6'b0;

    case (1'b1)
      state[FETCH] : begin
        if (sR_tvalid == 1) next[DECODE] = 1'b1;
        else                next[FETCH]  = 1'b1;
      end

      state[DECODE] : begin
        case(wOP)
          4'hA : begin
            next[TYPEA] = 1'b1;
          end
          4'hB : begin
            next[TYPEB] = 1'b1;
          end
          4'hC : begin
            next[TYPEC] = 1'b1;
          end
          default : begin
            next[FETCH] = 1'b1;
          end
        endcase
      end

      state[TYPEA] : begin
        next[FETCH] = 1'b1;
      end

      state[TYPEB] : begin
        next[FETCH] = 1'b1;
      end

      state[TYPEC] : begin
        next[FETCH] = 1'b1;
      end
    endcase
  end // End Always
//// ---------------------------------------------------------------------------
//jit_fifo u_axis_fifo(
//  .s_aclk         (ACLK                     ),
//  .m_aclk         (ACLK                     ),
//  .s_aresetn      (ARESETN                  ),
//  .s_axis_tvalid  (w_tvalid                 ),
//  .s_axis_tready  (w_tready                 ),
//  .s_axis_tdata   (w_tdata                  ),
//  .m_axis_tready  (mR_tready                ),
//  .m_axis_tvalid  (mR_tvalid                ),
//  .m_axis_tdata   (mR_tdata                 )
//);
end // NUM_ACCs == 2
endgenerate
//       _    ____ ____           _  _
//      / \  / ___/ ___|___   _  | || |
//     / _ \| |  | |   / __| (_) | || |_
//    / ___ \ |__| |___\__ \  _  |__   _|
//   /_/   \_\____\____|___/ (_)    |_|
//
generate if (NUM_ACCs == 4) begin
  reg    [ 3 : 0]    racc1_A1   ;
  reg    [ 3 : 0]    racc1_B1   ;
  reg    [ 5 : 0]    racc1_C1   ;
  reg    [15 : 0]    racc1_R1   ;
  reg    [15 : 0]    racc1_R2   ;
  reg    [15 : 0]    racc1_R3   ;
  reg                racc1_done ;

  reg    [ 3 : 0]    racc2_A1   ;
  reg    [ 3 : 0]    racc2_B1   ;
  reg    [ 5 : 0]    racc2_C1   ;
  reg    [15 : 0]    racc2_R1   ;
  reg    [15 : 0]    racc2_R2   ;
  reg    [15 : 0]    racc2_R3   ;
  reg                racc2_done ;

  reg    [ 3 : 0]    racc3_A1   ;
  reg    [ 3 : 0]    racc3_B1   ;
  reg    [ 5 : 0]    racc3_C1   ;
  reg    [15 : 0]    racc3_R1   ;
  reg    [15 : 0]    racc3_R2   ;
  reg    [15 : 0]    racc3_R3   ;
  reg                racc3_done ;

  reg    [ 3 : 0]    racc4_A1   ;
  reg    [ 3 : 0]    racc4_B1   ;
  reg    [ 5 : 0]    racc4_C1   ;
  reg    [15 : 0]    racc4_R1   ;
  reg    [15 : 0]    racc4_R2   ;
  reg    [15 : 0]    racc4_R3   ;
  reg                racc4_done ;

  // ---------------------------------------------------------------------------
  parameter [5:0]    FETCH  = 0, // 1
                     DECODE = 1, // 2
                     TYPEA  = 2, // 4
                     TYPEB  = 3, // 8
                     TYPEC  = 4, // 16
                     START  = 5; // 32
  reg       [5:0]    state, next;

  parameter [39:0]   ACC1  = 0, // 1
                     ACC2  = 1, // 2
                     ACC3  = 2, // 4
                     ACC4  = 3, // 8
                     ACC5  = 4, // 16
                     ACC6  = 5, // 32
                     ACC7  = 6, // 64
                     ACC8  = 7, // 128
                     DONE1 = 8,
                     DONE2 = 9,
                     DONE3 = 10,
                     DONE4 = 11,
                     DONE5 = 12,
                     DONE6 = 13,
                     DONE7 = 14,
                     DONE8 = 15,
                     DONE11= 16,
                     DONE12= 17,
                     DONE13= 18,
                     DONE21= 19,
                     DONE22= 20,
                     DONE23= 21,
                     DONE31= 22,
                     DONE32= 23,
                     DONE33= 24,
                     DONE41= 25,
                     DONE42= 26,
                     DONE43= 27,
                     DONE51= 28,
                     DONE52= 29,
                     DONE53= 30,
                     DONE61= 31,
                     DONE62= 32,
                     DONE63= 33,
                     DONE71= 34,
                     DONE72= 35,
                     DONE73= 36,
                     DONE81= 37,
                     DONE82= 38,
                     DONE83= 39;
  reg    [39:0]      state_acc, next_acc;

  reg    [31 : 0]    rcmd       ;
  reg                rmR_tvalid ;
  reg    [31 : 0]    rmR_tdata  ;

  wire   [ 3 : 0]    wOP        ;
  wire   [ 3 : 0]    wACCn      ;
  wire   [ 3 : 0]    wREGn      ;
  wire   [15 : 0]    wVALUE     ;
  wire   [ 3 : 0]    wVA        ;
  wire   [ 3 : 0]    wVB        ;
  wire   [ 5 : 0]    wVC        ;

  wire               w_tvalid   ;
  wire               w_tready   ;
  wire   [31 : 0]    w_tdata    ;

  assign sR_tready = (state[FETCH] == 1);

  assign wOP       = rcmd[31:28] ;
  assign wACCn     = rcmd[27:24] ;
  assign wREGn     = rcmd[23:20] ;
  assign wVALUE    = rcmd[15: 0] ;

  assign wVA       = rcmd[ 3: 0] ;
  assign wVB       = rcmd[ 7: 4] ;
  assign wVC       = {(rcmd[11: 8] == 4'hF) ? 2'b11 : 2'b00, (wVB == 4'b0) ? 2'b01 : 2'b10, (wVA == 4'b0) ? 2'b01 : 2'b10};
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc1_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc1_done)        racc1_done <= 1'b1;
  //     if (next_acc[DONE1])  racc1_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc2_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc2_done)        racc2_done <= 1'b1;
  //     if (next_acc[DONE2])  racc2_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc3_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc3_done)        racc3_done <= 1'b1;
  //     if (next_acc[DONE3])  racc3_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc4_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc4_done)        racc4_done <= 1'b1;
  //     if (next_acc[DONE4])  racc4_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // assign acc1_start = (state[TYPEA] && wACCn == 4'd1);
  // assign acc2_start = (state[TYPEA] && wACCn == 4'd2);
  // assign acc3_start = (state[TYPEA] && wACCn == 4'd3);
  // assign acc4_start = (state[TYPEA] && wACCn == 4'd4);
  // ---------------------------------------------------------------------------
  assign acc1_A1 = racc1_A1 ;
  assign acc1_B1 = racc1_B1 ;
  assign acc1_C1 = racc1_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_A1 <= 4'd0;
      racc1_B1 <= 4'd0;
      racc1_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd1) begin
        racc1_A1 <= wVA;
        racc1_B1 <= wVB;
        racc1_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_A1 = racc2_A1 ;
  assign acc2_B1 = racc2_B1 ;
  assign acc2_C1 = racc2_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_A1 <= 4'd0;
      racc2_B1 <= 4'd0;
      racc2_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd2) begin
        racc2_A1 <= wVA;
        racc2_B1 <= wVB;
        racc2_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc3_A1 = racc3_A1 ;
  assign acc3_B1 = racc3_B1 ;
  assign acc3_C1 = racc3_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc3_A1 <= 4'd0;
      racc3_B1 <= 4'd0;
      racc3_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd3) begin
        racc3_A1 <= wVA;
        racc3_B1 <= wVB;
        racc3_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc4_A1 = racc4_A1 ;
  assign acc4_B1 = racc4_B1 ;
  assign acc4_C1 = racc4_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc4_A1 <= 4'd0;
      racc4_B1 <= 4'd0;
      racc4_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd4) begin
        racc4_A1 <= wVA;
        racc4_B1 <= wVB;
        racc4_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc1_R1   = racc1_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd1 && wREGn == 4'd1)
        racc1_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc1_R2   = racc1_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd1 && wREGn == 4'd2)
        racc1_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc1_R3   = racc1_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd1 && wREGn == 4'd3)
        racc1_R3 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_R1   = racc2_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd2 && wREGn == 4'd1)
        racc2_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_R2   = racc2_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd2 && wREGn == 4'd2)
        racc2_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_R3   = racc2_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd2 && wREGn == 4'd3)
        racc2_R3 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc3_R1   = racc3_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc3_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd3 && wREGn == 4'd1)
        racc3_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc3_R2   = racc3_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc3_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd3 && wREGn == 4'd2)
        racc3_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc3_R3   = racc3_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc3_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd3 && wREGn == 4'd3)
        racc3_R3 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc4_R1   = racc4_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc4_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd4 && wREGn == 4'd1)
        racc4_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc4_R2   = racc4_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc4_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd4 && wREGn == 4'd2)
        racc4_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc4_R3   = racc4_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc4_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd4 && wREGn == 4'd3)
        racc4_R3 <= wVALUE;
    end
  end
  // ACC FSM ACK----------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     state_acc        <=  16'b0;
  //     state_acc[ACC1]  <=   1'b1;
  //   end
  //   else state_acc     <= next_acc;
  // end
  // ACC FSM LOGICAL------------------------------------------------------------
  // always @(*) begin
  //   next_acc  = 16'b0;

  //   case (1'b1)
  //     state_acc[ACC1] : begin
  //       if (racc1_done) next_acc[DONE1] = 1'b1;
  //       else            next_acc[ACC2]  = 1'b1;
  //     end

  //     state_acc[ACC2] : begin
  //       if (racc2_done) next_acc[DONE2] = 1'b1;
  //       else            next_acc[ACC3]  = 1'b1;
  //     end

  //     state_acc[ACC3] : begin
  //       if (racc3_done) next_acc[DONE3] = 1'b1;
  //       else            next_acc[ACC4] = 1'b1;
  //     end

  //     state_acc[ACC4] : begin
  //       if (racc4_done) next_acc[DONE4] = 1'b1;
  //       else            next_acc[ACC1] = 1'b1;
  //     end

  //     state_acc[DONE1] : begin
  //       if (w_tready)  next_acc[DONE11] = 1'b1;
  //       else           next_acc[DONE1]  = 1'b1;
  //     end

  //     state_acc[DONE11] : begin
  //       if (w_tready)  next_acc[DONE12] = 1'b1;
  //       else           next_acc[DONE11] = 1'b1;
  //     end

  //     state_acc[DONE12] : begin
  //       if (w_tready)  next_acc[DONE13] = 1'b1;
  //       else           next_acc[DONE12] = 1'b1;
  //     end

  //     state_acc[DONE13] : begin
  //       if (w_tready)  next_acc[ACC2]   = 1'b1;
  //       else           next_acc[DONE13] = 1'b1;
  //     end

  //     state_acc[DONE2] : begin
  //       if (w_tready)  next_acc[DONE21] = 1'b1;
  //       else           next_acc[DONE2]  = 1'b1;
  //     end

  //     state_acc[DONE21] : begin
  //       if (w_tready)  next_acc[DONE22] = 1'b1;
  //       else           next_acc[DONE21] = 1'b1;
  //     end

  //     state_acc[DONE22] : begin
  //       if (w_tready)  next_acc[DONE23] = 1'b1;
  //       else           next_acc[DONE22] = 1'b1;
  //     end

  //     state_acc[DONE23] : begin
  //       if (w_tready)  next_acc[ACC3]   = 1'b1;
  //       else           next_acc[DONE23] = 1'b1;
  //     end

  //     state_acc[DONE3] : begin
  //       if (w_tready)  next_acc[DONE31] = 1'b1;
  //       else           next_acc[DONE3]  = 1'b1;
  //     end

  //     state_acc[DONE31] : begin
  //       if (w_tready)  next_acc[DONE32] = 1'b1;
  //       else           next_acc[DONE31] = 1'b1;
  //     end

  //     state_acc[DONE32] : begin
  //       if (w_tready)  next_acc[DONE33] = 1'b1;
  //       else           next_acc[DONE32] = 1'b1;
  //     end

  //     state_acc[DONE33] : begin
  //       if (w_tready)  next_acc[ACC4]   = 1'b1;
  //       else           next_acc[DONE33] = 1'b1;
  //     end

  //     state_acc[DONE4] : begin
  //       if (w_tready)  next_acc[DONE41] = 1'b1;
  //       else           next_acc[DONE4]  = 1'b1;
  //     end

  //     state_acc[DONE41] : begin
  //       if (w_tready)  next_acc[DONE42] = 1'b1;
  //       else           next_acc[DONE41] = 1'b1;
  //     end

  //     state_acc[DONE42] : begin
  //       if (w_tready)  next_acc[DONE43] = 1'b1;
  //       else           next_acc[DONE42] = 1'b1;
  //     end

  //     state_acc[DONE43] : begin
  //       if (w_tready)  next_acc[ACC1]   = 1'b1;
  //       else           next_acc[DONE43] = 1'b1;
  //     end
  //   endcase
  // end
  // ---------------------------------------------------------------------------
  // assign w_tdata  = rmR_tdata  ;
  // always @(*) begin
  //   rmR_tdata = 32'd0;

  //   case (1'b1)
  //     next_acc[DONE1] : rmR_tdata = 32'hBABE0001;
  //     next_acc[DONE2] : rmR_tdata = 32'hBABE0002;
  //     next_acc[DONE3] : rmR_tdata = 32'hBABE0003;
  //     next_acc[DONE4] : rmR_tdata = 32'hBABE0004;
  //     default         : rmR_tdata = 32'hBABEFACE;
  //   endcase
  // end
  // // ---------------------------------------------------------------------------
  // assign w_tvalid = ( next_acc[DONE1] || next_acc[DONE11] || next_acc[DONE12] || next_acc[DONE13] ||
  //                     next_acc[DONE2] || next_acc[DONE21] || next_acc[DONE22] || next_acc[DONE23] ||
  //                     next_acc[DONE3] || next_acc[DONE31] || next_acc[DONE32] || next_acc[DONE33] ||
  //                     next_acc[DONE4] || next_acc[DONE41] || next_acc[DONE42] || next_acc[DONE43] );
  //============================================================================
  // Do Not Change below
  // rcmd-----------------------------------------------------------------------
  always @(posedge ACLK) begin
    if (!ARESETN) begin
        rcmd <= 15'd0;
    end
    else begin
      if (state[FETCH] == 1 && sR_tvalid == 1)
        rcmd <= sR_tdata;
    end
  end
  // FSM ACK--------------------------------------------------------------------
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      state        <=  7'b0;
      state[FETCH] <=  1'b1;
    end
    else state     <= next;
  end
  // FSM LOGICAL----------------------------------------------------------------
  always @(*) begin
    next = 6'b0;

    case (1'b1)
      state[FETCH] : begin
        if (sR_tvalid == 1) next[DECODE] = 1'b1;
        else                next[FETCH]  = 1'b1;
      end

      state[DECODE] : begin
        case(wOP)
          4'hA : begin
            next[TYPEA] = 1'b1;
          end
          4'hB : begin
            next[TYPEB] = 1'b1;
          end
          4'hC : begin
            next[TYPEC] = 1'b1;
          end
          default : begin
            next[FETCH] = 1'b1;
          end
        endcase
      end

      state[TYPEA] : begin
        next[FETCH] = 1'b1;
      end

      state[TYPEB] : begin
        next[FETCH] = 1'b1;
      end

      state[TYPEC] : begin
        next[FETCH] = 1'b1;
      end
    endcase
  end // End Always
//// ---------------------------------------------------------------------------
//jit_fifo u_axis_fifo(
//  .s_aclk         (ACLK                     ),
//  .m_aclk         (ACLK                     ),
//  .s_aresetn      (ARESETN                  ),
//  .s_axis_tvalid  (w_tvalid                 ),
//  .s_axis_tready  (w_tready                 ),
//  .s_axis_tdata   (w_tdata                  ),
//  .m_axis_tready  (mR_tready                ),
//  .m_axis_tvalid  (mR_tvalid                ),
//  .m_axis_tdata   (mR_tdata                 )
//);
end // NUM_ACCs == 4
endgenerate
//       _    ____ ____            ___
//      / \  / ___/ ___|___   _   ( _ )
//     / _ \| |  | |   / __| (_)  / _ \
//    / ___ \ |__| |___\__ \  _  | (_) |
//   /_/   \_\____\____|___/ (_)  \___/
//
generate if (NUM_ACCs == 8) begin
  reg    [ 3 : 0]    racc1_A1   ;
  reg    [ 3 : 0]    racc1_B1   ;
  reg    [ 5 : 0]    racc1_C1   ;
  reg    [15 : 0]    racc1_R1   ;
  reg    [15 : 0]    racc1_R2   ;
  reg    [15 : 0]    racc1_R3   ;
  reg                racc1_done ;

  reg    [ 3 : 0]    racc2_A1   ;
  reg    [ 3 : 0]    racc2_B1   ;
  reg    [ 5 : 0]    racc2_C1   ;
  reg    [15 : 0]    racc2_R1   ;
  reg    [15 : 0]    racc2_R2   ;
  reg    [15 : 0]    racc2_R3   ;
  reg                racc2_done ;

  reg    [ 3 : 0]    racc3_A1   ;
  reg    [ 3 : 0]    racc3_B1   ;
  reg    [ 5 : 0]    racc3_C1   ;
  reg    [15 : 0]    racc3_R1   ;
  reg    [15 : 0]    racc3_R2   ;
  reg    [15 : 0]    racc3_R3   ;
  reg                racc3_done ;

  reg    [ 3 : 0]    racc4_A1   ;
  reg    [ 3 : 0]    racc4_B1   ;
  reg    [ 5 : 0]    racc4_C1   ;
  reg    [15 : 0]    racc4_R1   ;
  reg    [15 : 0]    racc4_R2   ;
  reg    [15 : 0]    racc4_R3   ;
  reg                racc4_done ;

  reg    [ 3 : 0]    racc5_A1   ;
  reg    [ 3 : 0]    racc5_B1   ;
  reg    [ 5 : 0]    racc5_C1   ;
  reg    [15 : 0]    racc5_R1   ;
  reg    [15 : 0]    racc5_R2   ;
  reg    [15 : 0]    racc5_R3   ;
  reg                racc5_done ;

  reg    [ 3 : 0]    racc6_A1   ;
  reg    [ 3 : 0]    racc6_B1   ;
  reg    [ 5 : 0]    racc6_C1   ;
  reg    [15 : 0]    racc6_R1   ;
  reg    [15 : 0]    racc6_R2   ;
  reg    [15 : 0]    racc6_R3   ;
  reg                racc6_done ;

  reg    [ 3 : 0]    racc7_A1   ;
  reg    [ 3 : 0]    racc7_B1   ;
  reg    [ 5 : 0]    racc7_C1   ;
  reg    [15 : 0]    racc7_R1   ;
  reg    [15 : 0]    racc7_R2   ;
  reg    [15 : 0]    racc7_R3   ;
  reg                racc7_done ;

  reg    [ 3 : 0]    racc8_A1   ;
  reg    [ 3 : 0]    racc8_B1   ;
  reg    [ 5 : 0]    racc8_C1   ;
  reg    [15 : 0]    racc8_R1   ;
  reg    [15 : 0]    racc8_R2   ;
  reg    [15 : 0]    racc8_R3   ;
  reg                racc8_done ;
  // ---------------------------------------------------------------------------
  parameter [5:0]    FETCH  = 0, // 1
                     DECODE = 1, // 2
                     TYPEA  = 2, // 4
                     TYPEB  = 3, // 8
                     TYPEC  = 4, // 16
                     START  = 5; // 32
  reg       [5:0]    state, next;

  parameter [39:0]   ACC1  = 0, // 1
                     ACC2  = 1, // 2
                     ACC3  = 2, // 4
                     ACC4  = 3, // 8
                     ACC5  = 4, // 16
                     ACC6  = 5, // 32
                     ACC7  = 6, // 64
                     ACC8  = 7, // 128
                     DONE1 = 8,
                     DONE2 = 9,
                     DONE3 = 10,
                     DONE4 = 11,
                     DONE5 = 12,
                     DONE6 = 13,
                     DONE7 = 14,
                     DONE8 = 15,
                     DONE11= 16,
                     DONE12= 17,
                     DONE13= 18,
                     DONE21= 19,
                     DONE22= 20,
                     DONE23= 21,
                     DONE31= 22,
                     DONE32= 23,
                     DONE33= 24,
                     DONE41= 25,
                     DONE42= 26,
                     DONE43= 27,
                     DONE51= 28,
                     DONE52= 29,
                     DONE53= 30,
                     DONE61= 31,
                     DONE62= 32,
                     DONE63= 33,
                     DONE71= 34,
                     DONE72= 35,
                     DONE73= 36,
                     DONE81= 37,
                     DONE82= 38,
                     DONE83= 39;
  reg    [39:0]      state_acc, next_acc;

  reg    [31 : 0]    rcmd       ;
  reg                rmR_tvalid ;
  reg    [31 : 0]    rmR_tdata  ;

  wire   [ 3 : 0]    wOP        ;
  wire   [ 3 : 0]    wACCn      ;
  wire   [ 3 : 0]    wREGn      ;
  wire   [15 : 0]    wVALUE     ;
  wire   [ 3 : 0]    wVA        ;
  wire   [ 3 : 0]    wVB        ;
  wire   [ 5 : 0]    wVC        ;

  wire               w_tvalid   ;
  wire               w_tready   ;
  wire   [31 : 0]    w_tdata    ;

  assign sR_tready = (state[FETCH] == 1);

  assign wOP       = rcmd[31:28] ;
  assign wACCn     = rcmd[27:24] ;
  assign wREGn     = rcmd[23:20] ;
  assign wVALUE    = rcmd[15: 0] ;

  assign wVA       = rcmd[ 3: 0] ;
  assign wVB       = rcmd[ 7: 4] ;
  assign wVC       = {(rcmd[11: 8] == 4'hF) ? 2'b11 : 2'b00, (wVB == 4'b0) ? 2'b01 : 2'b10, (wVA == 4'b0) ? 2'b01 : 2'b10};
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc1_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc1_done)        racc1_done <= 1'b1;
  //     if (next_acc[DONE1])  racc1_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc2_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc2_done)        racc2_done <= 1'b1;
  //     if (next_acc[DONE2])  racc2_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc3_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc3_done)        racc3_done <= 1'b1;
  //     if (next_acc[DONE3])  racc3_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc4_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc4_done)        racc4_done <= 1'b1;
  //     if (next_acc[DONE4])  racc4_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc5_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc5_done)        racc5_done <= 1'b1;
  //     if (next_acc[DONE5])  racc5_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc6_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc6_done)        racc6_done <= 1'b1;
  //     if (next_acc[DONE6])  racc6_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc7_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc7_done)        racc7_done <= 1'b1;
  //     if (next_acc[DONE7])  racc7_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     racc8_done <= 1'b0;
  //   end
  //   else begin
  //     if (acc8_done)        racc8_done <= 1'b1;
  //     if (next_acc[DONE8])  racc8_done <= 1'b0;
  //   end
  // end
  // ---------------------------------------------------------------------------
  // assign acc1_start = (state[TYPEA] && wACCn == 4'd1);
  // assign acc2_start = (state[TYPEA] && wACCn == 4'd2);
  // assign acc3_start = (state[TYPEA] && wACCn == 4'd3);
  // assign acc4_start = (state[TYPEA] && wACCn == 4'd4);
  // assign acc5_start = (state[TYPEA] && wACCn == 4'd5);
  // assign acc6_start = (state[TYPEA] && wACCn == 4'd6);
  // assign acc7_start = (state[TYPEA] && wACCn == 4'd7);
  // assign acc8_start = (state[TYPEA] && wACCn == 4'd8);
  // ---------------------------------------------------------------------------
  assign acc1_A1 = racc1_A1 ;
  assign acc1_B1 = racc1_B1 ;
  assign acc1_C1 = racc1_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_A1 <= 4'd0;
      racc1_B1 <= 4'd0;
      racc1_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd1) begin
        racc1_A1 <= wVA;
        racc1_B1 <= wVB;
        racc1_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_A1 = racc2_A1 ;
  assign acc2_B1 = racc2_B1 ;
  assign acc2_C1 = racc2_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_A1 <= 4'd0;
      racc2_B1 <= 4'd0;
      racc2_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd2) begin
        racc2_A1 <= wVA;
        racc2_B1 <= wVB;
        racc2_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc3_A1 = racc3_A1 ;
  assign acc3_B1 = racc3_B1 ;
  assign acc3_C1 = racc3_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc3_A1 <= 4'd0;
      racc3_B1 <= 4'd0;
      racc3_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd3) begin
        racc3_A1 <= wVA;
        racc3_B1 <= wVB;
        racc3_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc4_A1 = racc4_A1 ;
  assign acc4_B1 = racc4_B1 ;
  assign acc4_C1 = racc4_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc4_A1 <= 4'd0;
      racc4_B1 <= 4'd0;
      racc4_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd4) begin
        racc4_A1 <= wVA;
        racc4_B1 <= wVB;
        racc4_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc5_A1 = racc5_A1 ;
  assign acc5_B1 = racc5_B1 ;
  assign acc5_C1 = racc5_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc5_A1 <= 4'd0;
      racc5_B1 <= 4'd0;
      racc5_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd5) begin
        racc5_A1 <= wVA;
        racc5_B1 <= wVB;
        racc5_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc6_A1 = racc6_A1 ;
  assign acc6_B1 = racc6_B1 ;
  assign acc6_C1 = racc6_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc6_A1 <= 4'd0;
      racc6_B1 <= 4'd0;
      racc6_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd6) begin
        racc6_A1 <= wVA;
        racc6_B1 <= wVB;
        racc6_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc7_A1 = racc7_A1 ;
  assign acc7_B1 = racc7_B1 ;
  assign acc7_C1 = racc7_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc7_A1 <= 4'd0;
      racc7_B1 <= 4'd0;
      racc7_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd7) begin
        racc7_A1 <= wVA;
        racc7_B1 <= wVB;
        racc7_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc8_A1 = racc8_A1 ;
  assign acc8_B1 = racc8_B1 ;
  assign acc8_C1 = racc8_C1 ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc8_A1 <= 4'd0;
      racc8_B1 <= 4'd0;
      racc8_C1 <= 5'd0;
    end
    else begin
      if (state[TYPEB] == 1'b1 && wACCn == 4'd8) begin
        racc8_A1 <= wVA;
        racc8_B1 <= wVB;
        racc8_C1 <= wVC;
      end
    end
  end
  // ---------------------------------------------------------------------------
  assign acc1_R1   = racc1_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd1 && wREGn == 4'd1)
        racc1_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc1_R2   = racc1_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd1 && wREGn == 4'd2)
        racc1_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc1_R3   = racc1_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc1_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd1 && wREGn == 4'd3)
        racc1_R3 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_R1   = racc2_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd2 && wREGn == 4'd1)
        racc2_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_R2   = racc2_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd2 && wREGn == 4'd2)
        racc2_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc2_R3   = racc2_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc2_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd2 && wREGn == 4'd3)
        racc2_R3 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc3_R1   = racc3_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc3_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd3 && wREGn == 4'd1)
        racc3_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc3_R2   = racc3_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc3_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd3 && wREGn == 4'd2)
        racc3_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc3_R3   = racc3_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc3_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd3 && wREGn == 4'd3)
        racc3_R3 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc4_R1   = racc4_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc4_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd4 && wREGn == 4'd1)
        racc4_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc4_R2   = racc4_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc4_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd4 && wREGn == 4'd2)
        racc4_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc4_R3   = racc4_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc4_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd4 && wREGn == 4'd3)
        racc4_R3 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc5_R1   = racc5_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc5_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd5 && wREGn == 4'd1)
        racc5_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc5_R2   = racc5_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc5_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd5 && wREGn == 4'd2)
        racc5_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc5_R3   = racc5_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc5_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd5 && wREGn == 4'd3)
        racc5_R3 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc6_R1   = racc6_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc6_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd6 && wREGn == 4'd1)
        racc6_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc6_R2   = racc6_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc6_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd6 && wREGn == 4'd2)
        racc6_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc6_R3   = racc6_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc6_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd6 && wREGn == 4'd3)
        racc6_R3 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc7_R1   = racc7_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc7_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd7 && wREGn == 4'd1)
        racc7_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc7_R2   = racc7_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc7_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd7 && wREGn == 4'd2)
        racc7_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc7_R3   = racc7_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc7_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd7 && wREGn == 4'd3)
        racc7_R3 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc8_R1   = racc8_R1    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc8_R1 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd8 && wREGn == 4'd1)
        racc8_R1 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc8_R2   = racc8_R2    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc8_R2 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd8 && wREGn == 4'd2)
        racc8_R2 <= wVALUE;
    end
  end
  // ---------------------------------------------------------------------------
  assign acc8_R3   = racc8_R3    ;
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      racc8_R3 <= 16'd0;
    end
    else begin
      if (state[TYPEC] == 1'b1 && wACCn == 4'd8 && wREGn == 4'd3)
        racc8_R3 <= wVALUE;
    end
  end
  // ACC FSM ACK----------------------------------------------------------------
  // always @(posedge ACLK) begin
  //   if (!ARESETN) begin
  //     state_acc        <=  40'b0;
  //     state_acc[ACC1]  <=   1'b1;
  //   end
  //   else state_acc     <= next_acc;
  // end
  // ACC FSM LOGICAL------------------------------------------------------------
  // always @(*) begin
  //   next_acc  = 40'b0;

  //   case (1'b1)
  //     state_acc[ACC1] : begin
  //       if (racc1_done) next_acc[DONE1] = 1'b1;
  //       else            next_acc[ACC2]  = 1'b1;
  //     end

  //     state_acc[ACC2] : begin
  //       if (racc2_done) next_acc[DONE2] = 1'b1;
  //       else            next_acc[ACC3]  = 1'b1;
  //     end

  //     state_acc[ACC3] : begin
  //       if (racc3_done) next_acc[DONE3] = 1'b1;
  //       else            next_acc[ACC4] = 1'b1;
  //     end

  //     state_acc[ACC4] : begin
  //       if (racc4_done) next_acc[DONE4] = 1'b1;
  //       else            next_acc[ACC5] = 1'b1;
  //     end

  //     state_acc[ACC5] : begin
  //       if (racc5_done) next_acc[DONE5] = 1'b1;
  //       else            next_acc[ACC6] = 1'b1;
  //     end

  //     state_acc[ACC6] : begin
  //       if (racc6_done) next_acc[DONE6] = 1'b1;
  //       else            next_acc[ACC7] = 1'b1;
  //     end

  //     state_acc[ACC7] : begin
  //       if (racc7_done) next_acc[DONE7] = 1'b1;
  //       else            next_acc[ACC8] = 1'b1;
  //     end

  //     state_acc[ACC8] : begin
  //       if (racc8_done) next_acc[DONE8] = 1'b1;
  //       else            next_acc[ACC1] = 1'b1;
  //     end

  //     state_acc[DONE1] : begin
  //       if (w_tready)  next_acc[DONE11] = 1'b1;
  //       else           next_acc[DONE1]  = 1'b1;
  //     end

  //     state_acc[DONE11] : begin
  //       if (w_tready)  next_acc[DONE12] = 1'b1;
  //       else           next_acc[DONE11] = 1'b1;
  //     end

  //     state_acc[DONE12] : begin
  //       if (w_tready)  next_acc[DONE13] = 1'b1;
  //       else           next_acc[DONE12] = 1'b1;
  //     end

  //     state_acc[DONE13] : begin
  //       if (w_tready)  next_acc[ACC2]   = 1'b1;
  //       else           next_acc[DONE13] = 1'b1;
  //     end

  //     state_acc[DONE2] : begin
  //       if (w_tready)  next_acc[DONE21] = 1'b1;
  //       else           next_acc[DONE2]  = 1'b1;
  //     end

  //     state_acc[DONE21] : begin
  //       if (w_tready)  next_acc[DONE22] = 1'b1;
  //       else           next_acc[DONE21] = 1'b1;
  //     end

  //     state_acc[DONE22] : begin
  //       if (w_tready)  next_acc[DONE23] = 1'b1;
  //       else           next_acc[DONE22] = 1'b1;
  //     end

  //     state_acc[DONE23] : begin
  //       if (w_tready)  next_acc[ACC3]   = 1'b1;
  //       else           next_acc[DONE23] = 1'b1;
  //     end

  //     state_acc[DONE3] : begin
  //       if (w_tready)  next_acc[DONE31] = 1'b1;
  //       else           next_acc[DONE3]  = 1'b1;
  //     end

  //     state_acc[DONE31] : begin
  //       if (w_tready)  next_acc[DONE32] = 1'b1;
  //       else           next_acc[DONE31] = 1'b1;
  //     end

  //     state_acc[DONE32] : begin
  //       if (w_tready)  next_acc[DONE33] = 1'b1;
  //       else           next_acc[DONE32] = 1'b1;
  //     end

  //     state_acc[DONE33] : begin
  //       if (w_tready)  next_acc[ACC4]   = 1'b1;
  //       else           next_acc[DONE33] = 1'b1;
  //     end

  //     state_acc[DONE4] : begin
  //       if (w_tready)  next_acc[DONE41] = 1'b1;
  //       else           next_acc[DONE4]  = 1'b1;
  //     end

  //     state_acc[DONE41] : begin
  //       if (w_tready)  next_acc[DONE42] = 1'b1;
  //       else           next_acc[DONE41] = 1'b1;
  //     end

  //     state_acc[DONE42] : begin
  //       if (w_tready)  next_acc[DONE43] = 1'b1;
  //       else           next_acc[DONE42] = 1'b1;
  //     end

  //     state_acc[DONE43] : begin
  //       if (w_tready)  next_acc[ACC5]   = 1'b1;
  //       else           next_acc[DONE43] = 1'b1;
  //     end

  //     state_acc[DONE5] : begin
  //       if (w_tready)  next_acc[DONE51] = 1'b1;
  //       else           next_acc[DONE5]  = 1'b1;
  //     end

  //     state_acc[DONE51] : begin
  //       if (w_tready)  next_acc[DONE52] = 1'b1;
  //       else           next_acc[DONE51] = 1'b1;
  //     end

  //     state_acc[DONE52] : begin
  //       if (w_tready)  next_acc[DONE53] = 1'b1;
  //       else           next_acc[DONE52] = 1'b1;
  //     end

  //     state_acc[DONE53] : begin
  //       if (w_tready)  next_acc[ACC6]   = 1'b1;
  //       else           next_acc[DONE53] = 1'b1;
  //     end

  //     state_acc[DONE6] : begin
  //       if (w_tready)  next_acc[DONE61] = 1'b1;
  //       else           next_acc[DONE6]  = 1'b1;
  //     end

  //     state_acc[DONE61] : begin
  //       if (w_tready)  next_acc[DONE62] = 1'b1;
  //       else           next_acc[DONE61] = 1'b1;
  //     end

  //     state_acc[DONE62] : begin
  //       if (w_tready)  next_acc[DONE63] = 1'b1;
  //       else           next_acc[DONE62] = 1'b1;
  //     end

  //     state_acc[DONE63] : begin
  //       if (w_tready)  next_acc[ACC7]   = 1'b1;
  //       else           next_acc[DONE63] = 1'b1;
  //     end

  //     state_acc[DONE7] : begin
  //       if (w_tready)  next_acc[DONE71] = 1'b1;
  //       else           next_acc[DONE7]  = 1'b1;
  //     end

  //     state_acc[DONE71] : begin
  //       if (w_tready)  next_acc[DONE72] = 1'b1;
  //       else           next_acc[DONE71] = 1'b1;
  //     end

  //     state_acc[DONE72] : begin
  //       if (w_tready)  next_acc[DONE73] = 1'b1;
  //       else           next_acc[DONE72] = 1'b1;
  //     end

  //     state_acc[DONE73] : begin
  //       if (w_tready)  next_acc[ACC8]   = 1'b1;
  //       else           next_acc[DONE73] = 1'b1;
  //     end

  //     state_acc[DONE8] : begin
  //       if (w_tready)  next_acc[DONE81] = 1'b1;
  //       else           next_acc[DONE8]  = 1'b1;
  //     end

  //     state_acc[DONE81] : begin
  //       if (w_tready)  next_acc[DONE82] = 1'b1;
  //       else           next_acc[DONE81] = 1'b1;
  //     end

  //     state_acc[DONE82] : begin
  //       if (w_tready)  next_acc[DONE83] = 1'b1;
  //       else           next_acc[DONE82] = 1'b1;
  //     end

  //     state_acc[DONE83] : begin
  //       if (w_tready)  next_acc[ACC1]   = 1'b1;
  //       else           next_acc[DONE83] = 1'b1;
  //     end
  //   endcase
  // end
  // ---------------------------------------------------------------------------
  // assign w_tdata  = rmR_tdata  ;
  // always @(*) begin
  //   rmR_tdata = 32'd0;

  //   case (1'b1)
  //     next_acc[DONE1] : rmR_tdata = 32'hBABE0001;
  //     next_acc[DONE2] : rmR_tdata = 32'hBABE0002;
  //     next_acc[DONE3] : rmR_tdata = 32'hBABE0003;
  //     next_acc[DONE4] : rmR_tdata = 32'hBABE0004;
  //     next_acc[DONE5] : rmR_tdata = 32'hBABE0005;
  //     next_acc[DONE6] : rmR_tdata = 32'hBABE0006;
  //     next_acc[DONE7] : rmR_tdata = 32'hBABE0007;
  //     next_acc[DONE8] : rmR_tdata = 32'hBABE0008;
  //     default         : rmR_tdata = 32'hBABEFACE;

  //   endcase
  // end
  // // ---------------------------------------------------------------------------
  // assign w_tvalid = ( next_acc[DONE1] || next_acc[DONE11] || next_acc[DONE12] || next_acc[DONE13] ||
  //                     next_acc[DONE2] || next_acc[DONE21] || next_acc[DONE22] || next_acc[DONE23] ||
  //                     next_acc[DONE3] || next_acc[DONE31] || next_acc[DONE32] || next_acc[DONE33] ||
  //                     next_acc[DONE4] || next_acc[DONE41] || next_acc[DONE42] || next_acc[DONE43] ||
  //                     next_acc[DONE5] || next_acc[DONE51] || next_acc[DONE52] || next_acc[DONE53] ||
  //                     next_acc[DONE6] || next_acc[DONE61] || next_acc[DONE62] || next_acc[DONE63] ||
  //                     next_acc[DONE7] || next_acc[DONE71] || next_acc[DONE72] || next_acc[DONE73] ||
  //                     next_acc[DONE8] || next_acc[DONE81] || next_acc[DONE82] || next_acc[DONE83] );
  //============================================================================
  // Do Not Change below
  // rcmd-----------------------------------------------------------------------
  always @(posedge ACLK) begin
    if (!ARESETN) begin
        rcmd <= 15'd0;
    end
    else begin
      if (state[FETCH] == 1 && sR_tvalid == 1)
        rcmd <= sR_tdata;
    end
  end
  // FSM ACK--------------------------------------------------------------------
  always @(posedge ACLK) begin
    if (!ARESETN) begin
      state        <=  7'b0;
      state[FETCH] <=  1'b1;
    end
    else state     <= next;
  end
  // FSM LOGICAL----------------------------------------------------------------
  always @(*) begin
    next = 6'b0;

    case (1'b1)
      state[FETCH] : begin
        if (sR_tvalid == 1) next[DECODE] = 1'b1;
        else                next[FETCH]  = 1'b1;
      end

      state[DECODE] : begin
        case(wOP)
          4'hA : begin
            next[TYPEA] = 1'b1;
          end
          4'hB : begin
            next[TYPEB] = 1'b1;
          end
          4'hC : begin
            next[TYPEC] = 1'b1;
          end
          default : begin
            next[FETCH] = 1'b1;
          end
        endcase
      end

      state[TYPEA] : begin
        next[FETCH] = 1'b1;
      end

      state[TYPEB] : begin
        next[FETCH] = 1'b1;
      end

      state[TYPEC] : begin
        next[FETCH] = 1'b1;
      end
    endcase
  end // End Always
//// ---------------------------------------------------------------------------
//jit_fifo u_axis_fifo(
//  .s_aclk         (ACLK                     ),
//  .m_aclk         (ACLK                     ),
//  .s_aresetn      (ARESETN                  ),
//  .s_axis_tvalid  (w_tvalid                 ),
//  .s_axis_tready  (w_tready                 ),
//  .s_axis_tdata   (w_tdata                  ),
//  .m_axis_tready  (mR_tready                ),
//  .m_axis_tvalid  (mR_tvalid                ),
//  .m_axis_tdata   (mR_tdata                 )
//);
end // NUM_ACCs == 8
endgenerate
endmodule
