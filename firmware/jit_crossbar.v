`timescale 1ns / 1ps

module jit_crossbar#
(
  parameter integer NUM_ACCs = 2
)
(
  // -----------------------------------
  input   wire  [ 3 : 0]   CONF1_A     ,
  input   wire  [ 3 : 0]   CONF1_B     ,

  output  wire             sC1_tready  ,
  input   wire             sC1_tvalid  ,
  input   wire  [31 : 0]   sC1_tdata   ,

  input   wire             mA1_tready  ,
  output  wire             mA1_tvalid  ,
  output  wire  [31 : 0]   mA1_tdata   ,

  input   wire             mB1_tready  ,
  output  wire             mB1_tvalid  ,
  output  wire  [31 : 0]   mB1_tdata   ,
  // -----------------------------------
  input   wire  [ 3 : 0]   CONF2_A     ,
  input   wire  [ 3 : 0]   CONF2_B     ,

  output  wire             sC2_tready  ,
  input   wire             sC2_tvalid  ,
  input   wire  [31 : 0]   sC2_tdata   ,

  input   wire             mA2_tready  ,
  output  wire             mA2_tvalid  ,
  output  wire  [31 : 0]   mA2_tdata   ,

  input   wire             mB2_tready  ,
  output  wire             mB2_tvalid  ,
  output  wire  [31 : 0]   mB2_tdata   ,
  // -----------------------------------
  input   wire  [ 3 : 0]   CONF3_A     ,
  input   wire  [ 3 : 0]   CONF3_B     ,

  output  wire             sC3_tready  ,
  input   wire             sC3_tvalid  ,
  input   wire  [31 : 0]   sC3_tdata   ,

  input   wire             mA3_tready  ,
  output  wire             mA3_tvalid  ,
  output  wire  [31 : 0]   mA3_tdata   ,

  input   wire             mB3_tready  ,
  output  wire             mB3_tvalid  ,
  output  wire  [31 : 0]   mB3_tdata   ,
  // -----------------------------------
  input   wire  [ 3 : 0]   CONF4_A     ,
  input   wire  [ 3 : 0]   CONF4_B     ,

  output  wire             sC4_tready  ,
  input   wire             sC4_tvalid  ,
  input   wire  [31 : 0]   sC4_tdata   ,

  input   wire             mA4_tready  ,
  output  wire             mA4_tvalid  ,
  output  wire  [31 : 0]   mA4_tdata   ,

  input   wire             mB4_tready  ,
  output  wire             mB4_tvalid  ,
  output  wire  [31 : 0]   mB4_tdata   ,
  // -----------------------------------
  input   wire  [ 3 : 0]   CONF5_A     ,
  input   wire  [ 3 : 0]   CONF5_B     ,

  output  wire             sC5_tready  ,
  input   wire             sC5_tvalid  ,
  input   wire  [31 : 0]   sC5_tdata   ,

  input   wire             mA5_tready  ,
  output  wire             mA5_tvalid  ,
  output  wire  [31 : 0]   mA5_tdata   ,

  input   wire             mB5_tready  ,
  output  wire             mB5_tvalid  ,
  output  wire  [31 : 0]   mB5_tdata   ,
  // -----------------------------------
  input   wire  [ 3 : 0]   CONF6_A     ,
  input   wire  [ 3 : 0]   CONF6_B     ,

  output  wire             sC6_tready  ,
  input   wire             sC6_tvalid  ,
  input   wire  [31 : 0]   sC6_tdata   ,

  input   wire             mA6_tready  ,
  output  wire             mA6_tvalid  ,
  output  wire  [31 : 0]   mA6_tdata   ,

  input   wire             mB6_tready  ,
  output  wire             mB6_tvalid  ,
  output  wire  [31 : 0]   mB6_tdata   ,
  // -----------------------------------
  input   wire  [ 3 : 0]   CONF7_A     ,
  input   wire  [ 3 : 0]   CONF7_B     ,

  output  wire             sC7_tready  ,
  input   wire             sC7_tvalid  ,
  input   wire  [31 : 0]   sC7_tdata   ,

  input   wire             mA7_tready  ,
  output  wire             mA7_tvalid  ,
  output  wire  [31 : 0]   mA7_tdata   ,

  input   wire             mB7_tready  ,
  output  wire             mB7_tvalid  ,
  output  wire  [31 : 0]   mB7_tdata   ,
  // -----------------------------------
  input   wire  [ 3 : 0]   CONF8_A     ,
  input   wire  [ 3 : 0]   CONF8_B     ,

  output  wire             sC8_tready  ,
  input   wire             sC8_tvalid  ,
  input   wire  [31 : 0]   sC8_tdata   ,

  input   wire             mA8_tready  ,
  output  wire             mA8_tvalid  ,
  output  wire  [31 : 0]   mA8_tdata   ,

  input   wire             mB8_tready  ,
  output  wire             mB8_tvalid  ,
  output  wire  [31 : 0]   mB8_tdata   ,
  // -----------------------------------
  input   wire             ACLK        ,
  input   wire             ARESETN
);

//       _    ____ ____           ____
//      / \  / ___/ ___|___   _  |___ \
//     / _ \| |  | |   / __| (_)   __) |
//    / ___ \ |__| |___\__ \  _   / __/
//   /_/   \_\____\____|___/ (_) |_____|
//
generate if (NUM_ACCs == 2) begin

  wire              w111_tready;
  wire              w112_tready;
  wire              w121_tready;
  wire              w122_tready;
  assign            sC1_tready = w111_tready | w112_tready | w121_tready | w122_tready;

  wire              w211_tready;
  wire              w212_tready;
  wire              w221_tready;
  wire              w222_tready;
  assign            sC2_tready = w211_tready | w212_tready | w221_tready | w222_tready;

  jit_mux #(NUM_ACCs) u_acc1_mA_mux(
    .s1_tready  (w111_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w211_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (sC3_tready ),
    .s3_tvalid  ( 1'd0      ),
    .s3_tdata   (32'd0      ),
    .s4_tready  (sC4_tready ),
    .s4_tvalid  ( 1'd0      ),
    .s4_tdata   (32'd0      ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mA1_tready ),
    .mO_tvalid  (mA1_tvalid ),
    .mO_tdata   (mA1_tdata  ),
    .CONF       (CONF1_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc1_mB_mux(
    .s1_tready  (w112_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w212_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (sC3_tready ),
    .s3_tvalid  ( 1'd0      ),
    .s3_tdata   (32'd0      ),
    .s4_tready  (sC4_tready ),
    .s4_tvalid  ( 1'd0      ),
    .s4_tdata   (32'd0      ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mB1_tready ),
    .mO_tvalid  (mB1_tvalid ),
    .mO_tdata   (mB1_tdata  ),
    .CONF       (CONF1_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc2_mA_mux(
    .s1_tready  (w121_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w221_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (sC3_tready ),
    .s3_tvalid  ( 1'd0      ),
    .s3_tdata   (32'd0      ),
    .s4_tready  (sC4_tready ),
    .s4_tvalid  ( 1'd0      ),
    .s4_tdata   (32'd0      ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mA2_tready ),
    .mO_tvalid  (mA2_tvalid ),
    .mO_tdata   (mA2_tdata  ),
    .CONF       (CONF2_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc2_mB_mux(
    .s1_tready  (w122_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w222_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (sC3_tready ),
    .s3_tvalid  ( 1'd0      ),
    .s3_tdata   (32'd0      ),
    .s4_tready  (sC4_tready ),
    .s4_tvalid  ( 1'd0      ),
    .s4_tdata   (32'd0      ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mB2_tready ),
    .mO_tvalid  (mB2_tvalid ),
    .mO_tdata   (mB2_tdata  ),
    .CONF       (CONF2_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

end // NUM_ACCs == 2
endgenerate
//       _    ____ ____           _  _
//      / \  / ___/ ___|___   _  | || |
//     / _ \| |  | |   / __| (_) | || |_
//    / ___ \ |__| |___\__ \  _  |__   _|
//   /_/   \_\____\____|___/ (_)    |_|
//
generate if (NUM_ACCs == 4) begin

  wire              w111_tready;
  wire              w112_tready;
  wire              w121_tready;
  wire              w122_tready;
  wire              w131_tready;
  wire              w132_tready;
  wire              w141_tready;
  wire              w142_tready;
  assign            sC1_tready = w111_tready | w112_tready | w121_tready | w122_tready | w131_tready | w132_tready | w141_tready | w142_tready;

  wire              w211_tready;
  wire              w212_tready;
  wire              w221_tready;
  wire              w222_tready;
  wire              w231_tready;
  wire              w232_tready;
  wire              w241_tready;
  wire              w242_tready;
  assign            sC2_tready = w211_tready | w212_tready | w221_tready | w222_tready | w231_tready | w232_tready | w241_tready | w242_tready;

  wire              w311_tready;
  wire              w312_tready;
  wire              w321_tready;
  wire              w322_tready;
  wire              w331_tready;
  wire              w332_tready;
  wire              w341_tready;
  wire              w342_tready;
  assign            sC3_tready = w311_tready | w312_tready | w321_tready | w322_tready | w331_tready | w332_tready | w341_tready | w342_tready;

  wire              w411_tready;
  wire              w412_tready;
  wire              w421_tready;
  wire              w422_tready;
  wire              w431_tready;
  wire              w432_tready;
  wire              w441_tready;
  wire              w442_tready;
  assign            sC4_tready = w411_tready | w412_tready | w421_tready | w422_tready | w431_tready | w432_tready | w441_tready | w442_tready;

  jit_mux #(NUM_ACCs) u_acc1_mA_mux(
    .s1_tready  (w111_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w211_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w311_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w411_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mA1_tready ),
    .mO_tvalid  (mA1_tvalid ),
    .mO_tdata   (mA1_tdata  ),
    .CONF       (CONF1_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc1_mB_mux(
    .s1_tready  (w112_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w212_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w312_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w412_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mB1_tready ),
    .mO_tvalid  (mB1_tvalid ),
    .mO_tdata   (mB1_tdata  ),
    .CONF       (CONF1_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc2_mA_mux(
    .s1_tready  (w121_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w221_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w321_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w421_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mA2_tready ),
    .mO_tvalid  (mA2_tvalid ),
    .mO_tdata   (mA2_tdata  ),
    .CONF       (CONF2_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc2_mB_mux(
    .s1_tready  (w122_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w222_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w322_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w422_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mB2_tready ),
    .mO_tvalid  (mB2_tvalid ),
    .mO_tdata   (mB2_tdata  ),
    .CONF       (CONF2_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc3_mA_mux(
    .s1_tready  (w131_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w231_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w331_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w431_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mA3_tready ),
    .mO_tvalid  (mA3_tvalid ),
    .mO_tdata   (mA3_tdata  ),
    .CONF       (CONF3_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc3_mB_mux(
    .s1_tready  (w132_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w232_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w332_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w432_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mB3_tready ),
    .mO_tvalid  (mB3_tvalid ),
    .mO_tdata   (mB3_tdata  ),
    .CONF       (CONF3_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc4_mA_mux(
    .s1_tready  (w141_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w241_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w341_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w441_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mA4_tready ),
    .mO_tvalid  (mA4_tvalid ),
    .mO_tdata   (mA4_tdata  ),
    .CONF       (CONF4_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc4_mB_mux(
    .s1_tready  (w142_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w242_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w342_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w442_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (sC5_tready ),
    .s5_tvalid  ( 1'd0      ),
    .s5_tdata   (32'd0      ),
    .s6_tready  (sC6_tready ),
    .s6_tvalid  ( 1'd0      ),
    .s6_tdata   (32'd0      ),
    .s7_tready  (sC7_tready ),
    .s7_tvalid  ( 1'd0      ),
    .s7_tdata   (32'd0      ),
    .s8_tready  (sC8_tready ),
    .s8_tvalid  ( 1'd0      ),
    .s8_tdata   (32'd0      ),
    .mO_tready  (mB4_tready ),
    .mO_tvalid  (mB4_tvalid ),
    .mO_tdata   (mB4_tdata  ),
    .CONF       (CONF4_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

end // NUM_ACCs == 4
endgenerate

//       _    ____ ____            ___
//      / \  / ___/ ___|___   _   ( _ )
//     / _ \| |  | |   / __| (_)  / _ \
//    / ___ \ |__| |___\__ \  _  | (_) |
//   /_/   \_\____\____|___/ (_)  \___/
//
generate if (NUM_ACCs == 8) begin
  wire              w111_tready;
  wire              w112_tready;
  wire              w121_tready;
  wire              w122_tready;
  wire              w131_tready;
  wire              w132_tready;
  wire              w141_tready;
  wire              w142_tready;
  wire              w151_tready;
  wire              w152_tready;
  wire              w161_tready;
  wire              w162_tready;
  wire              w171_tready;
  wire              w172_tready;
  wire              w181_tready;
  wire              w182_tready;
  assign            sC1_tready = w111_tready | w112_tready | w121_tready | w122_tready | w131_tready | w132_tready | w141_tready | w142_tready | w151_tready | w152_tready | w161_tready | w162_tready | w171_tready | w172_tready | w181_tready | w182_tready;

  wire              w211_tready;
  wire              w212_tready;
  wire              w221_tready;
  wire              w222_tready;
  wire              w231_tready;
  wire              w232_tready;
  wire              w241_tready;
  wire              w242_tready;
  wire              w251_tready;
  wire              w252_tready;
  wire              w261_tready;
  wire              w262_tready;
  wire              w271_tready;
  wire              w272_tready;
  wire              w281_tready;
  wire              w282_tready;
  assign            sC2_tready = w211_tready | w212_tready | w221_tready | w222_tready | w231_tready | w232_tready | w241_tready | w242_tready | w251_tready | w252_tready | w261_tready | w262_tready | w271_tready | w272_tready | w281_tready | w282_tready;

  wire              w311_tready;
  wire              w312_tready;
  wire              w321_tready;
  wire              w322_tready;
  wire              w331_tready;
  wire              w332_tready;
  wire              w341_tready;
  wire              w342_tready;
  wire              w351_tready;
  wire              w352_tready;
  wire              w361_tready;
  wire              w362_tready;
  wire              w371_tready;
  wire              w372_tready;
  wire              w381_tready;
  wire              w382_tready;
  assign            sC3_tready = w311_tready | w312_tready | w321_tready | w322_tready | w331_tready | w332_tready | w341_tready | w342_tready | w351_tready | w352_tready | w361_tready | w362_tready | w371_tready | w372_tready | w381_tready | w382_tready;

  wire              w411_tready;
  wire              w412_tready;
  wire              w421_tready;
  wire              w422_tready;
  wire              w431_tready;
  wire              w432_tready;
  wire              w441_tready;
  wire              w442_tready;
  wire              w451_tready;
  wire              w452_tready;
  wire              w461_tready;
  wire              w462_tready;
  wire              w471_tready;
  wire              w472_tready;
  wire              w481_tready;
  wire              w482_tready;
  assign            sC4_tready = w411_tready | w412_tready | w421_tready | w422_tready | w431_tready | w432_tready | w441_tready | w442_tready | w451_tready | w452_tready | w461_tready | w462_tready | w471_tready | w472_tready | w481_tready | w482_tready;

  wire              w511_tready;
  wire              w512_tready;
  wire              w521_tready;
  wire              w522_tready;
  wire              w531_tready;
  wire              w532_tready;
  wire              w541_tready;
  wire              w542_tready;
  wire              w551_tready;
  wire              w552_tready;
  wire              w561_tready;
  wire              w562_tready;
  wire              w571_tready;
  wire              w572_tready;
  wire              w581_tready;
  wire              w582_tready;
  assign            sC5_tready = w511_tready | w512_tready | w521_tready | w522_tready | w531_tready | w532_tready | w541_tready | w542_tready | w551_tready | w552_tready | w561_tready | w562_tready | w571_tready | w572_tready | w581_tready | w582_tready;

  wire              w611_tready;
  wire              w612_tready;
  wire              w621_tready;
  wire              w622_tready;
  wire              w631_tready;
  wire              w632_tready;
  wire              w641_tready;
  wire              w642_tready;
  wire              w651_tready;
  wire              w652_tready;
  wire              w661_tready;
  wire              w662_tready;
  wire              w671_tready;
  wire              w672_tready;
  wire              w681_tready;
  wire              w682_tready;
  assign            sC6_tready = w611_tready | w612_tready | w621_tready | w622_tready | w631_tready | w632_tready | w641_tready | w642_tready | w651_tready | w652_tready | w661_tready | w662_tready | w671_tready | w672_tready | w681_tready | w682_tready;

  wire              w711_tready;
  wire              w712_tready;
  wire              w721_tready;
  wire              w722_tready;
  wire              w731_tready;
  wire              w732_tready;
  wire              w741_tready;
  wire              w742_tready;
  wire              w751_tready;
  wire              w752_tready;
  wire              w761_tready;
  wire              w762_tready;
  wire              w771_tready;
  wire              w772_tready;
  wire              w781_tready;
  wire              w782_tready;
  assign            sC7_tready = w711_tready | w712_tready | w721_tready | w722_tready | w731_tready | w732_tready | w741_tready | w742_tready | w751_tready | w752_tready | w761_tready | w762_tready | w771_tready | w772_tready | w781_tready | w782_tready;

  wire              w811_tready;
  wire              w812_tready;
  wire              w821_tready;
  wire              w822_tready;
  wire              w831_tready;
  wire              w832_tready;
  wire              w841_tready;
  wire              w842_tready;
  wire              w851_tready;
  wire              w852_tready;
  wire              w861_tready;
  wire              w862_tready;
  wire              w871_tready;
  wire              w872_tready;
  wire              w881_tready;
  wire              w882_tready;
  assign            sC8_tready = w811_tready | w812_tready | w821_tready | w822_tready | w831_tready | w832_tready | w841_tready | w842_tready | w851_tready | w852_tready | w861_tready | w862_tready | w871_tready | w872_tready | w881_tready | w882_tready;

  jit_mux #(NUM_ACCs) u_acc1_mA_mux(
    .s1_tready  (w111_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w211_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w311_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w411_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w511_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w611_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w711_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w811_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mA1_tready ),
    .mO_tvalid  (mA1_tvalid ),
    .mO_tdata   (mA1_tdata  ),
    .CONF       (CONF1_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc1_mB_mux(
    .s1_tready  (w112_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w212_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w312_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w412_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w512_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w612_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w712_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w812_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mB1_tready ),
    .mO_tvalid  (mB1_tvalid ),
    .mO_tdata   (mB1_tdata  ),
    .CONF       (CONF1_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc2_mA_mux(
    .s1_tready  (w121_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w221_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w321_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w421_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w521_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w621_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w721_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w821_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mA2_tready ),
    .mO_tvalid  (mA2_tvalid ),
    .mO_tdata   (mA2_tdata  ),
    .CONF       (CONF2_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc2_mB_mux(
    .s1_tready  (w122_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w222_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w322_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w422_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w522_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w622_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w722_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w822_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mB2_tready ),
    .mO_tvalid  (mB2_tvalid ),
    .mO_tdata   (mB2_tdata  ),
    .CONF       (CONF2_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc3_mA_mux(
    .s1_tready  (w131_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w231_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w331_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w431_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w531_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w631_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w731_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w831_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mA3_tready ),
    .mO_tvalid  (mA3_tvalid ),
    .mO_tdata   (mA3_tdata  ),
    .CONF       (CONF3_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc3_mB_mux(
    .s1_tready  (w132_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w232_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w332_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w432_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w532_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w632_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w732_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w832_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mB3_tready ),
    .mO_tvalid  (mB3_tvalid ),
    .mO_tdata   (mB3_tdata  ),
    .CONF       (CONF3_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc4_mA_mux(
    .s1_tready  (w141_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w241_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w341_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w441_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w541_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w641_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w741_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w841_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mA4_tready ),
    .mO_tvalid  (mA4_tvalid ),
    .mO_tdata   (mA4_tdata  ),
    .CONF       (CONF4_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc4_mB_mux(
    .s1_tready  (w142_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w242_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w342_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w442_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w542_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w642_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w742_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w842_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mB4_tready ),
    .mO_tvalid  (mB4_tvalid ),
    .mO_tdata   (mB4_tdata  ),
    .CONF       (CONF4_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc5_mA_mux(
    .s1_tready  (w151_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w251_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w351_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w451_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w551_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w651_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w751_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w851_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mA5_tready ),
    .mO_tvalid  (mA5_tvalid ),
    .mO_tdata   (mA5_tdata  ),
    .CONF       (CONF5_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc5_mB_mux(
    .s1_tready  (w152_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w252_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w352_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w452_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w552_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w652_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w752_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w852_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mB5_tready ),
    .mO_tvalid  (mB5_tvalid ),
    .mO_tdata   (mB5_tdata  ),
    .CONF       (CONF5_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc6_mA_mux(
    .s1_tready  (w161_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w261_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w361_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w461_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w561_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w661_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w761_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w861_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mA6_tready ),
    .mO_tvalid  (mA6_tvalid ),
    .mO_tdata   (mA6_tdata  ),
    .CONF       (CONF6_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc6_mB_mux(
    .s1_tready  (w162_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w262_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w362_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w462_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w562_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w662_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w762_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w862_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mB6_tready ),
    .mO_tvalid  (mB6_tvalid ),
    .mO_tdata   (mB6_tdata  ),
    .CONF       (CONF6_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc7_mA_mux(
    .s1_tready  (w171_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w271_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w371_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w471_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w571_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w671_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w771_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w871_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mA7_tready ),
    .mO_tvalid  (mA7_tvalid ),
    .mO_tdata   (mA7_tdata  ),
    .CONF       (CONF7_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc7_mB_mux(
    .s1_tready  (w172_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w272_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w372_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w472_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w572_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w672_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w772_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w872_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mB7_tready ),
    .mO_tvalid  (mB7_tvalid ),
    .mO_tdata   (mB7_tdata  ),
    .CONF       (CONF7_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc8_mA_mux(
    .s1_tready  (w181_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w281_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w381_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w481_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w581_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w681_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w781_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w881_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mA8_tready ),
    .mO_tvalid  (mA8_tvalid ),
    .mO_tdata   (mA8_tdata  ),
    .CONF       (CONF8_A    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );

  jit_mux #(NUM_ACCs) u_acc8_mB_mux(
    .s1_tready  (w182_tready),
    .s1_tvalid  (sC1_tvalid ),
    .s1_tdata   (sC1_tdata  ),
    .s2_tready  (w282_tready),
    .s2_tvalid  (sC2_tvalid ),
    .s2_tdata   (sC2_tdata  ),
    .s3_tready  (w382_tready),
    .s3_tvalid  (sC3_tvalid ),
    .s3_tdata   (sC3_tdata  ),
    .s4_tready  (w482_tready),
    .s4_tvalid  (sC4_tvalid ),
    .s4_tdata   (sC4_tdata  ),
    .s5_tready  (w582_tready),
    .s5_tvalid  (sC5_tvalid ),
    .s5_tdata   (sC5_tdata  ),
    .s6_tready  (w682_tready),
    .s6_tvalid  (sC6_tvalid ),
    .s6_tdata   (sC6_tdata  ),
    .s7_tready  (w782_tready),
    .s7_tvalid  (sC7_tvalid ),
    .s7_tdata   (sC7_tdata  ),
    .s8_tready  (w882_tready),
    .s8_tvalid  (sC8_tvalid ),
    .s8_tdata   (sC8_tdata  ),
    .mO_tready  (mB8_tready ),
    .mO_tvalid  (mB8_tvalid ),
    .mO_tdata   (mB8_tdata  ),
    .CONF       (CONF8_B    ),
    .ACLK       (ACLK       ),
    .ARESETN    (ARESETN    )
  );
end // NUM_ACCs == 8
endgenerate

endmodule
