`ifndef VX_DECODE_IF
`define VX_DECODE_IF

`include "VX_define.vh"

interface VX_decode_if ();

    wire                    valid;    

    wire [`NW_BITS-1:0]     wid;
    wire [`NUM_THREADS-1:0] thread_mask;
    wire [31:0]             curr_PC;

    wire [`EX_BITS-1:0]     ex_type;    
    wire [`OP_BITS-1:0]     ex_op; 
    wire [`FRM_BITS-1:0]    frm;    
    wire                    wb;

    wire [`NR_BITS-1:0]     rd;
    wire [`NR_BITS-1:0]     rs1;
    wire [`NR_BITS-1:0]     rs2;
    wire [`NR_BITS-1:0]     rs3;
    wire [31:0]             imm;    

    wire                    rs1_is_PC;
    wire                    rs2_is_imm;      
    wire                    use_rs3; 
    wire [`NUM_REGS-1:0]    used_regs;       

    wire                    ready;

endinterface

`endif