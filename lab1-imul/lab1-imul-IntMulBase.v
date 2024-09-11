//========================================================================
// Integer Multiplier Fixed-Latency Implementation
//========================================================================

`ifndef LAB1_IMUL_INT_MUL_BASE_V
`define LAB1_IMUL_INT_MUL_BASE_V

`include "lab1-imul-msgs.v"
`include "vc-trace.v"
`include "vc-muxes.v"
`include "vc-regs.v"
`include "vc-arithmetic.v"

// Define datapath and control unit here

module lab1_imul_IntMulBase_datapath
(
  input logic clk,
  input logic reset,
  input logic [63:0] req_msg,
  input logic result_en,
  input logic a_mux_sel,
  input logic b_mux_sel,
  input logic result_mux_sel,
  input logic add_mux_sel,
    

  output logic [31:0] resp_msg,
  output logic b_lsb
);

  // internal signals

  logic [31:0] a_reg_out;
  logic [31:0] b_reg_out;
  logic [31:0] result_reg_out;
  
  logic [31:0] shifted_left;
  logic [31:0] shifted_right;
  logic [31:0] adder_out;  

  logic [31:0] a_mux_out;
  logic [31:0] b_mux_out;
  logic [31:0] result_mux_out;
  logic [31:0] add_mux_out;
  
  vc_Mux2#(32) a_mux // instantiate a mux
  (
    .in0 (shifted_left),
    .in1 (req_msg[31:0]),
    .sel (a_mux_sel),
    .out ()
  );
  
  vc_Mux2#(32) b_mux // instantiate b mux
  (
    .in0 (shifted_right),
    .in1 (req_msg[63:32]),
    .sel (b_mux_sel),
    .out (b_mux_out)
  );
  
  vc_Mux2#(32) result_mux // instantiate result mux
  (
    .in0 (add_mux_out),
    .in1 (32'b0),
    .sel (result_mux_sel),
    .out (result_mux_out)
  );

  vc_Mux2#(32) add_mux // instantiate add mux
  (
    .in0 (adder_out),
    .in1 (result_reg_out),
    .sel (add_mux_sel),
    .out (add_mux_out)
  );
  
  vc_ResetReg#(32, 0) b_reg // b_reg, 32 bits, reset to 0
  (
    .in0 (clk),
    .in1 (reset),
    .in2 (b_reg_out),
    .in3 (b_mux_out) 
  );  
  
  vc_ResetReg#(32, 0) a_reg // a_reg, 32 bits, reset to 0
  (
    .in0 (clk),
    .in1 (reset),
    .in2 (a_reg_out),
    .in3 (a_mux_out) 
  );
 
  vc_EnResetReg#(32, 0) result_reg // result reg, 32 bits, reset to 0
  (
    .in0 (clk),
    .in1 (reset),
    .in2 (result_reg_out),
    .in3 (result_mux_out),
    .in4 (result_en)
  );

// Arithmetic modules
  vc_RightLogicalShifter#(32,1) right_shift// 32 bits, shift by 1
  (
    .in0 (b_reg_out),
    .in1 (1'b1), // shift by 1, always
    .in2 (shifted_right)
  );

  vc_LeftLogicalShifter#(32,1) left_shift // 32 bits, shift by 1
  (
    .in0 (a_reg_out),
    .in1 (1'b1), // shift by 1, always
    .in2 (shifted_left)
  );
  
  vc_SimpleAdder#(32) adder // 32 bit adder, no cin
  (
    .in0 (a_reg_out),
    .in1 (result_reg_out),
    .in2 (adder_out)
  );
  
  // outputs
  assign b_lsb = b_reg_out[0];
  assign resp_msg = result_reg_out;   

endmodule

//==============================================
//  Control Unit
//==============================================


module lab1_imul_IntMulBase_controlUnit
(
  input  logic  clk,
  input  logic  reset,

  input  logic  req_val,
  input  logic  resp_rdy,
  output  logic  req_rdy,
  output  logic  resq_val,

  output  logic  a_mux_sel,
  output  logic  b_mux_sel,
  output  logic  result_mux_sel,
  output  logic  result_en,
  output  logic  add_mux_sel,
  

  input  logic  b_lsb;
);

// State Definition

typedef enum logic[1:0]{
  state_IDLE = 2'b00,
  state_CALC = 2'b01,
  state_DONE = 2'b10
} state_t;


state_t current_state, next_state;

logic [4:0] counter_reg;

//  State Reset

always @(posedge clk) begin
  if ( reset ) begin
    current_state <= state_IDLE;
  else
    current_state <= next_state;
end


//  State Transitions

logic req_go
logic rsp_go

assign  req_go = req_val  && req_rdy;
assign  resp_go = resp_val && resp_rdy; 

always @(*) begin
  case ( current_state)
    STATE_IDLE: begin
      if (req_go)
        next_state <= STATE_CALC;
      else
        next_state <= STATE_IDLE;
  end
    STATE_CALC: begin
      if (counter == 5'd31)
        next_state <= STATE_DONE;
      else
        next_state <= STATE_CALC;
  end
    STATE_DONE: begin
      if (resp_go)
        next_state <= STATE_IDLE;
      else
        next_state <= STATE_DONE;
  end
    default: next_state = current_state;
  
  endcase

end

//  Counter

always @(posedge clk) begin
  if (reset) begin
    counter_reg <= 5'd0;
  end

  else begin
    case (current_state)
     STATE_IDLE: counter_reg <= 5'd0;
     STATE_CALC: counter_reg <= counter_reg + 1;
     STATE_DONE: counter_reg <= 5'd0;
    endcase
end


//  Control Signals

always @(*) begin
  a_mux_sel = 0;
  b_mux_sel = 0;
  result_mux_sel = 0;
  add_mux_sel = 0;
  result_en = 0;
  req_rdy = 0;
  resp_val = 0;


case (current_state)
  STATE_IDLE: begin
    req_rdy = 1;
    a_mux_sel = 1;
    b_mux_sel = 1;
    result_mux_sel = 1;
    result_en = 1;  //not sure
    add_mux_sel = 1; //not sure
    resp_val = 0;

  STATE_CALC: begin
    req_rdy = 0;
    if (b_lsb) begin
      add_mux_sel = 0; //do add if b[0]=1
    else
      add_mux_sel = 1;
    end
    
    a_mux_sel = 0;
    b_mux_sel = 0;
    result_mux_sel = 0;
    result_en = 1;
    resp_val = 0;

  STATE_DONE: begin
    req_rdy = 0;
    a_mux = 1'bx;
    b_mux = 1'bx;
    add_mux_sel = 1'bx;
    result_mux_sel = 1'bx;
    result_en = 0;
    resp_val = 1;
  endcase;
end



endmodule

//========================================================================
// Integer Multiplier Fixed-Latency Implementation
//========================================================================

module lab1_imul_IntMulBase
(
  input  logic                clk,
  input  logic                reset,

  input  logic                req_val,
  output logic                req_rdy,
  input  lab1_imul_req_msg_t  req_msg,

  output logic                resp_val,
  input  logic                resp_rdy,
  output lab1_imul_resp_msg_t resp_msg
);

  logic [31:0] a_reg;
  logic [31:0] b_reg;
  logic [31:0] result_reg;
  logic [4:0] counter;

    

  //----------------------------------------------------------------------
  // Trace request message
  //----------------------------------------------------------------------

  lab1_imul_ReqMsgTrace req_msg_trace
  (
    .clk   (clk),
    .reset (reset),
    .val   (req_val),
    .rdy   (req_rdy),
    .msg   (req_msg)
  );

  // Instantiate datapath and control models here and then connect them
  // together. As a place holder, for now we simply pass input operand
  // A through to the output, which obviously is not correct.

  assign req_rdy         = resp_rdy;
  assign resp_val        = req_val;
  assign resp_msg.result = req_msg.a;

  //----------------------------------------------------------------------
  // Line Tracing
  //----------------------------------------------------------------------

  `ifndef SYNTHESIS

  reg [`VC_TRACE_NBITS_TO_NCHARS(32)*8-1:0] str;

  `VC_TRACE_BEGIN
  begin

    req_msg_trace.trace( trace_str );

    vc_trace.append_str( trace_str, "(" );

    // Add extra line tracing for internal state here

    vc_trace.append_str( trace_str, ")" );

    $sformat( str, "%x", resp_msg );
    vc_trace.append_val_rdy_str( trace_str, resp_val, resp_rdy, str );

  end
  `VC_TRACE_END

  `endif /* SYNTHESIS */

endmodule

`endif /* LAB1_IMUL_INT_MUL_BASE_V */

