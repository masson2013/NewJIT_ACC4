`timescale 1ns / 1ps

module jit_couple
(
  output  wire             sInA_tready     ,
  input   wire             sInA_tvalid     ,
  input   wire  [31 : 0]   sInA_tdata      ,
  output  wire             sInB_tready     ,
  input   wire             sInB_tvalid     ,
  input   wire  [31 : 0]   sInB_tdata      ,
  input   wire             mOutC_tready    ,
  output  wire             mOutC_tvalid    ,
  output  wire  [31 : 0]   mOutC_tdata     ,

  output  wire             scInA_tready    ,
  input   wire             scInA_tvalid    ,
  input   wire  [31 : 0]   scInA_tdata     ,
  output  wire             scInB_tready    ,
  input   wire             scInB_tvalid    ,
  input   wire  [31 : 0]   scInB_tdata     ,
  input   wire             mcOutC_tready   ,
  output  wire             mcOutC_tvalid   ,
  output  wire  [31 : 0]   mcOutC_tdata    ,

  output  wire             sAccInC_tready  ,
  input   wire             sAccInC_tvalid  ,
  input   wire  [31 : 0]   sAccInC_tdata   ,
  input   wire             mAccOutA_tready ,
  output  wire             mAccOutA_tvalid ,
  output  wire  [31 : 0]   mAccOutA_tdata  ,
  input   wire             mAccOutB_tready ,
  output  wire             mAccOutB_tvalid ,
  output  wire  [31 : 0]   mAccOutB_tdata  ,

  input   wire  [ 3 : 0]   CONF            ,

  input   wire             ACLK            ,
  input   wire             ARESETN
);

  assign mOutC_tvalid    =  sAccInC_tvalid                ;
  assign mOutC_tdata     =  sAccInC_tdata                 ;
  assign mcOutC_tvalid   =  sAccInC_tvalid                ;
  assign mcOutC_tdata    =  sAccInC_tdata                 ;
  assign sAccInC_tready  =  mOutC_tready | mcOutC_tready  ;


  jit_mux #(2) u_AccOutA_mux(
    .s1_tready  (sInA_tready     ),
    .s1_tvalid  (sInA_tvalid     ),
    .s1_tdata   (sInA_tdata      ),
    .s2_tready  (scInA_tready    ),
    .s2_tvalid  (scInA_tvalid    ),
    .s2_tdata   (scInA_tdata     ),
    .s3_tready  (sC3_tready      ),
    .s3_tvalid  ( 1'd0           ),
    .s3_tdata   (32'd0           ),
    .s4_tready  (sC4_tready      ),
    .s4_tvalid  ( 1'd0           ),
    .s4_tdata   (32'd0           ),
    .s5_tready  (sC5_tready      ),
    .s5_tvalid  ( 1'd0           ),
    .s5_tdata   (32'd0           ),
    .s6_tready  (sC6_tready      ),
    .s6_tvalid  ( 1'd0           ),
    .s6_tdata   (32'd0           ),
    .s7_tready  (sC7_tready      ),
    .s7_tvalid  ( 1'd0           ),
    .s7_tdata   (32'd0           ),
    .s8_tready  (sC8_tready      ),
    .s8_tvalid  ( 1'd0           ),
    .s8_tdata   (32'd0           ),
    .mO_tready  (mAccOutA_tready ),
    .mO_tvalid  (mAccOutA_tvalid ),
    .mO_tdata   (mAccOutA_tdata  ),
    .CONF       ({2'd0,CONF[1:0]}),
    .ACLK       (ACLK            ),
    .ARESETN    (ARESETN         )
  );

  jit_mux #(2) u_AccOutB_mux(
    .s1_tready  (sInB_tready     ),
    .s1_tvalid  (sInB_tvalid     ),
    .s1_tdata   (sInB_tdata      ),
    .s2_tready  (scInB_tready    ),
    .s2_tvalid  (scInB_tvalid    ),
    .s2_tdata   (scInB_tdata     ),
    .s3_tready  (sC3_tready      ),
    .s3_tvalid  ( 1'd0           ),
    .s3_tdata   (32'd0           ),
    .s4_tready  (sC4_tready      ),
    .s4_tvalid  ( 1'd0           ),
    .s4_tdata   (32'd0           ),
    .s5_tready  (sC5_tready      ),
    .s5_tvalid  ( 1'd0           ),
    .s5_tdata   (32'd0           ),
    .s6_tready  (sC6_tready      ),
    .s6_tvalid  ( 1'd0           ),
    .s6_tdata   (32'd0           ),
    .s7_tready  (sC7_tready      ),
    .s7_tvalid  ( 1'd0           ),
    .s7_tdata   (32'd0           ),
    .s8_tready  (sC8_tready      ),
    .s8_tvalid  ( 1'd0           ),
    .s8_tdata   (32'd0           ),
    .mO_tready  (mAccOutB_tready ),
    .mO_tvalid  (mAccOutB_tvalid ),
    .mO_tdata   (mAccOutB_tdata  ),
    .CONF       ({2'd0,CONF[3:2]}),
    .ACLK       (ACLK            ),
    .ARESETN    (ARESETN         )
  );

endmodule
