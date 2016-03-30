`timescale 1 ns / 1 ps

module jit_mux #
(
  parameter NUM_PORTs = 2
)
(

  output  reg              s1_tready  ,
  input   wire             s1_tvalid  ,
  input   wire  [31 : 0]   s1_tdata   ,

  output  reg              s2_tready  ,
  input   wire             s2_tvalid  ,
  input   wire  [31 : 0]   s2_tdata   ,

  output  reg              s3_tready  ,
  input   wire             s3_tvalid  ,
  input   wire  [31 : 0]   s3_tdata   ,

  output  reg              s4_tready  ,
  input   wire             s4_tvalid  ,
  input   wire  [31 : 0]   s4_tdata   ,

  output  reg              s5_tready  ,
  input   wire             s5_tvalid  ,
  input   wire  [31 : 0]   s5_tdata   ,

  output  reg              s6_tready  ,
  input   wire             s6_tvalid  ,
  input   wire  [31 : 0]   s6_tdata   ,

  output  reg              s7_tready  ,
  input   wire             s7_tvalid  ,
  input   wire  [31 : 0]   s7_tdata   ,

  output  reg              s8_tready  ,
  input   wire             s8_tvalid  ,
  input   wire  [31 : 0]   s8_tdata   ,

  input   wire             mO_tready  ,
  output  reg              mO_tvalid  ,
  output  reg   [31 : 0]   mO_tdata   ,

  input   wire  [ 3 : 0]   CONF       ,

  input   wire             ACLK       ,
  input   wire             ARESETN
);

//    ____   ___  ____ _____          ____
//   |  _ \ / _ \|  _ \_   _|__   _  |___ \
//   | |_) | | | | |_) || |/ __| (_)   __) |
//   |  __/| |_| |  _ < | |\__ \  _   / __/
//   |_|    \___/|_| \_\|_||___/ (_) |_____|
//
generate if (NUM_PORTs == 2) begin
  always @(*) begin
    if (!ARESETN) begin
      mO_tvalid =  1'b0     ;
      mO_tdata  = 32'd0     ;
      s1_tready =  1'b0     ;
      s2_tready =  1'b0     ;
      s3_tready =  1'b0     ;
      s4_tready =  1'b0     ;
      s5_tready =  1'b0     ;
      s6_tready =  1'b0     ;
      s7_tready =  1'b0     ;
      s8_tready =  1'b0     ;
    end
    else begin
      case (CONF)
        4'd0: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end

        4'd1: begin
          mO_tvalid = s1_tvalid ;
          mO_tdata  = s1_tdata  ;
          s1_tready = mO_tready ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd2: begin
          mO_tvalid = s2_tvalid ;
          mO_tdata  = s2_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = mO_tready ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        default: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end
      endcase
    end
  end
end
endgenerate
//    ____   ___  ____ _____          _____
//   |  _ \ / _ \|  _ \_   _|__   _  |___ /
//   | |_) | | | | |_) || |/ __| (_)   |_ \
//   |  __/| |_| |  _ < | |\__ \  _   ___) |
//   |_|    \___/|_| \_\|_||___/ (_) |____/
//
generate if (NUM_PORTs == 3) begin
  always @(*) begin
    if (!ARESETN) begin
      mO_tvalid =  1'b0     ;
      mO_tdata  = 32'd0     ;
      s1_tready =  1'b0     ;
      s2_tready =  1'b0     ;
      s3_tready =  1'b0     ;
      s4_tready =  1'b0     ;
      s5_tready =  1'b0     ;
      s6_tready =  1'b0     ;
      s7_tready =  1'b0     ;
      s8_tready =  1'b0     ;
    end
    else begin
      case (CONF)
        4'd0: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end

        4'd1: begin
          mO_tvalid = s1_tvalid ;
          mO_tdata  = s1_tdata  ;
          s1_tready = mO_tready ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd2: begin
          mO_tvalid = s2_tvalid ;
          mO_tdata  = s2_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = mO_tready ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd3: begin
          mO_tvalid = s3_tvalid ;
          mO_tdata  = s3_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = mO_tready ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        default: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end
      endcase
    end
  end
end
endgenerate
//    ____   ___  ____ _____          _  _
//   |  _ \ / _ \|  _ \_   _|__   _  | || |
//   | |_) | | | | |_) || |/ __| (_) | || |_
//   |  __/| |_| |  _ < | |\__ \  _  |__   _|
//   |_|    \___/|_| \_\|_||___/ (_)    |_|
//
generate if (NUM_PORTs == 4) begin
  always @(*) begin
    if (!ARESETN) begin
      mO_tvalid =  1'b0     ;
      mO_tdata  = 32'd0     ;
      s1_tready =  1'b0     ;
      s2_tready =  1'b0     ;
      s3_tready =  1'b0     ;
      s4_tready =  1'b0     ;
      s5_tready =  1'b0     ;
      s6_tready =  1'b0     ;
      s7_tready =  1'b0     ;
      s8_tready =  1'b0     ;
    end
    else begin
      case (CONF)
        4'd0: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end

        4'd1: begin
          mO_tvalid = s1_tvalid ;
          mO_tdata  = s1_tdata  ;
          s1_tready = mO_tready ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd2: begin
          mO_tvalid = s2_tvalid ;
          mO_tdata  = s2_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = mO_tready ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd3: begin
          mO_tvalid = s3_tvalid ;
          mO_tdata  = s3_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = mO_tready ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd4: begin
          mO_tvalid = s4_tvalid ;
          mO_tdata  = s4_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = mO_tready ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        default: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end
      endcase
    end
  end
end
endgenerate
//    ____   ___  ____ _____          ____
//   |  _ \ / _ \|  _ \_   _|__   _  | ___|
//   | |_) | | | | |_) || |/ __| (_) |___ \
//   |  __/| |_| |  _ < | |\__ \  _   ___) |
//   |_|    \___/|_| \_\|_||___/ (_) |____/
//
generate if (NUM_PORTs == 5) begin
  always @(*) begin
    if (!ARESETN) begin
      mO_tvalid =  1'b0     ;
      mO_tdata  = 32'd0     ;
      s1_tready =  1'b0     ;
      s2_tready =  1'b0     ;
      s3_tready =  1'b0     ;
      s4_tready =  1'b0     ;
      s5_tready =  1'b0     ;
      s6_tready =  1'b0     ;
      s7_tready =  1'b0     ;
      s8_tready =  1'b0     ;
    end
    else begin
      case (CONF)
        4'd0: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end

        4'd1: begin
          mO_tvalid = s1_tvalid ;
          mO_tdata  = s1_tdata  ;
          s1_tready = mO_tready ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd2: begin
          mO_tvalid = s2_tvalid ;
          mO_tdata  = s2_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = mO_tready ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd3: begin
          mO_tvalid = s3_tvalid ;
          mO_tdata  = s3_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = mO_tready ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd4: begin
          mO_tvalid = s4_tvalid ;
          mO_tdata  = s4_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = mO_tready ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd5: begin
          mO_tvalid = s5_tvalid ;
          mO_tdata  = s5_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = mO_tready ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        default: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end
      endcase
    end
  end
end
endgenerate
//    ____   ___  ____ _____           __
//   |  _ \ / _ \|  _ \_   _|__   _   / /_
//   | |_) | | | | |_) || |/ __| (_) | '_ \
//   |  __/| |_| |  _ < | |\__ \  _  | (_) |
//   |_|    \___/|_| \_\|_||___/ (_)  \___/
//
generate if (NUM_PORTs == 6) begin
  always @(*) begin
    if (!ARESETN) begin
      mO_tvalid =  1'b0     ;
      mO_tdata  = 32'd0     ;
      s1_tready =  1'b0     ;
      s2_tready =  1'b0     ;
      s3_tready =  1'b0     ;
      s4_tready =  1'b0     ;
      s5_tready =  1'b0     ;
      s6_tready =  1'b0     ;
      s7_tready =  1'b0     ;
      s8_tready =  1'b0     ;
    end
    else begin
      case (CONF)
        4'd0: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end

        4'd1: begin
          mO_tvalid = s1_tvalid ;
          mO_tdata  = s1_tdata  ;
          s1_tready = mO_tready ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd2: begin
          mO_tvalid = s2_tvalid ;
          mO_tdata  = s2_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = mO_tready ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd3: begin
          mO_tvalid = s3_tvalid ;
          mO_tdata  = s3_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = mO_tready ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd4: begin
          mO_tvalid = s4_tvalid ;
          mO_tdata  = s4_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = mO_tready ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd5: begin
          mO_tvalid = s5_tvalid ;
          mO_tdata  = s5_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = mO_tready ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd6: begin
          mO_tvalid = s5_tvalid ;
          mO_tdata  = s5_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = mO_tready ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        default: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end
      endcase
    end
  end
end
endgenerate
//    ____   ___  ____ _____          _____
//   |  _ \ / _ \|  _ \_   _|__   _  |___  |
//   | |_) | | | | |_) || |/ __| (_)    / /
//   |  __/| |_| |  _ < | |\__ \  _    / /
//   |_|    \___/|_| \_\|_||___/ (_)  /_/
//
generate if (NUM_PORTs == 7) begin
  always @(*) begin
    if (!ARESETN) begin
      mO_tvalid =  1'b0     ;
      mO_tdata  = 32'd0     ;
      s1_tready =  1'b0     ;
      s2_tready =  1'b0     ;
      s3_tready =  1'b0     ;
      s4_tready =  1'b0     ;
      s5_tready =  1'b0     ;
      s6_tready =  1'b0     ;
      s7_tready =  1'b0     ;
      s8_tready =  1'b0     ;
    end
    else begin
      case (CONF)
        4'd0: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end

        4'd1: begin
          mO_tvalid = s1_tvalid ;
          mO_tdata  = s1_tdata  ;
          s1_tready = mO_tready ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd2: begin
          mO_tvalid = s2_tvalid ;
          mO_tdata  = s2_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = mO_tready ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd3: begin
          mO_tvalid = s3_tvalid ;
          mO_tdata  = s3_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = mO_tready ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd4: begin
          mO_tvalid = s4_tvalid ;
          mO_tdata  = s4_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = mO_tready ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd5: begin
          mO_tvalid = s5_tvalid ;
          mO_tdata  = s5_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = mO_tready ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd6: begin
          mO_tvalid = s5_tvalid ;
          mO_tdata  = s5_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = mO_tready ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd7: begin
          mO_tvalid = s5_tvalid ;
          mO_tdata  = s5_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = mO_tready ;
          s8_tready = 1'b0      ;
        end

        default: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end
      endcase
    end
  end
end
endgenerate
//    ____   ___  ____ _____           ___
//   |  _ \ / _ \|  _ \_   _|__   _   ( _ )
//   | |_) | | | | |_) || |/ __| (_)  / _ \
//   |  __/| |_| |  _ < | |\__ \  _  | (_) |
//   |_|    \___/|_| \_\|_||___/ (_)  \___/
//
generate if (NUM_PORTs == 8) begin
  always @(*) begin
    if (!ARESETN) begin
      mO_tvalid =  1'b0     ;
      mO_tdata  = 32'd0     ;
      s1_tready =  1'b0     ;
      s2_tready =  1'b0     ;
      s3_tready =  1'b0     ;
      s4_tready =  1'b0     ;
      s5_tready =  1'b0     ;
      s6_tready =  1'b0     ;
      s7_tready =  1'b0     ;
      s8_tready =  1'b0     ;
    end
    else begin
      case (CONF)
        4'd0: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end

        4'd1: begin
          mO_tvalid = s1_tvalid ;
          mO_tdata  = s1_tdata  ;
          s1_tready = mO_tready ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd2: begin
          mO_tvalid = s2_tvalid ;
          mO_tdata  = s2_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = mO_tready ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd3: begin
          mO_tvalid = s3_tvalid ;
          mO_tdata  = s3_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = mO_tready ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd4: begin
          mO_tvalid = s4_tvalid ;
          mO_tdata  = s4_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = mO_tready ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd5: begin
          mO_tvalid = s5_tvalid ;
          mO_tdata  = s5_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = mO_tready ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd6: begin
          mO_tvalid = s5_tvalid ;
          mO_tdata  = s5_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = mO_tready ;
          s7_tready = 1'b0      ;
          s8_tready = 1'b0      ;
        end

        4'd7: begin
          mO_tvalid = s5_tvalid ;
          mO_tdata  = s5_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = mO_tready ;
          s8_tready = 1'b0      ;
        end

        4'd8: begin
          mO_tvalid = s5_tvalid ;
          mO_tdata  = s5_tdata  ;
          s1_tready = 1'b0      ;
          s2_tready = 1'b0      ;
          s3_tready = 1'b0      ;
          s4_tready = 1'b0      ;
          s5_tready = 1'b0      ;
          s6_tready = 1'b0      ;
          s7_tready = 1'b0      ;
          s8_tready = mO_tready ;
        end

        default: begin
          mO_tvalid =  1'b0     ;
          mO_tdata  = 32'd0     ;
          s1_tready =  1'b0     ;
          s2_tready =  1'b0     ;
          s3_tready =  1'b0     ;
          s4_tready =  1'b0     ;
          s5_tready =  1'b0     ;
          s6_tready =  1'b0     ;
          s7_tready =  1'b0     ;
          s8_tready =  1'b0     ;
        end
      endcase
    end
  end
end
endgenerate

endmodule
