// Template for Northwestern - CompEng 361 - Lab3 -- Version 1.1
// Groupname:
// NetIDs:

// Some useful defines...please add your own
`define WORD_WIDTH 32
`define NUM_REGS 32
`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_STORE      7'b0100011 
`define FUNC_ADD      3'b000
`define AUX_FUNC_ADD  7'b0000000
`define AUX_FUNC_SUB  7'b0100000
`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10
`define OPCODE_LUI        7'b0110111
`define OPCODE_AUIPC      7'b0010111
`define OPCODE_JAL        7'b1101111
`define OPCODE_JALR       7'b1100111
`define OPCODE_IMM        7'b0010011
`define OPCODE_MUL_DIV    7'b0110011

module SingleCycleCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [`WORD_WIDTH-1:0] PC, InstWord;
   wire [`WORD_WIDTH-1:0] DataAddr, StoreData, DataWord;
   wire [1:0]  MemSize;
   wire        MemWrEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst;
   wire [`WORD_WIDTH-1:0] Rdata1, Rdata2, RWrdata;
   wire        RWrEn;

   wire [`WORD_WIDTH-1:0] NPC, PC_Plus_4;
   wire [6:0]  opcode;

   wire [6:0]  funct7;
   wire [2:0]  funct3;

   wire invalid_op;
   
   // Only support R-TYPE ADD and SUB
   assign halt = invalid_op;
   assign invalid_op = !(((opcode == `OPCODE_COMPUTE) || (opcode == `OPCODE_AUIPC)
             || (opcode == `OPCODE_BRANCH) || (opcode == `OPCODE_IMM) || (opcode == `OPCODE_JAL)
             || (opcode == `OPCODE_JALR) || (opcode == `OPCODE_LOAD) || (opcode == `OPCODE_LUI) 
             || (opcode == `OPCODE_MUL_DIV) || (opcode == `OPCODE_STORE)) && (funct3 == `FUNC_ADD) &&
		      ((funct7 == `AUX_FUNC_ADD) || (funct7 == `AUX_FUNC_SUB)));
     
   // System State 
   Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
            .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
	      .AddrB(Rsrc2), .DataOutB(Rdata2), 
	      .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   Reg PC_REG(.Din(NPC), .Qout(PC), .WE(1'b1), .CLK(clk), .RST(rst));

   // Instruction Decode
   assign opcode = InstWord[6:0];   
   assign Rdst = InstWord[11:7]; 
   assign Rsrc1 = InstWord[19:15]; 
   assign Rsrc2 = InstWord[24:20];
   assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
   assign funct7 = InstWord[31:25];  // R-Type

   assign MemWrEn = 1'b0; // Change this to allow stores
   assign RWrEn = 1'b1;  // At the moment every instruction will write to the register file

   // Hardwired to support R-Type instructions -- please add muxes and other control signals
   ExecutionUnit EU(.out(RWrdata), .opA(Rdata1), .opB(Rdata2), .func(funct3), .auxFunc(funct7));

   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
   assign NPC = PC_Plus_4;
   
endmodule // SingleCycleCPU


// Incomplete version of Lab2 execution unit
// You will need to extend it. Feel free to modify the interface also

module ExecutionUnit(out, opA, opB, imm, func, auxFunc, opcode);
   output [`WORD_WIDTH-1:0] out;
   input [`WORD_WIDTH-1:0] opA, opB, imm;
   input [2:0] func;
   input [6:0] auxFunc, opcode;

   wire [`WORD_WIDTH-1:0] addsub, slli, srli, srai,
                        logicAnd, logicOr, logicXor, slti, sltiu,
                        mul, mulh, mulhsu, mulhu, div, divu, rem, remu,
                        lui, auipc, jal, jalr, branch;

   // 默认输出值
assign out = 32'hXXXXXXXX;

// R-Type指令
assign addsub = (opcode == `OPCODE_COMPUTE && func == 3'b000 && auxFunc == `AUX_FUNC_ADD) ? (opA + opB) :
                (opcode == `OPCODE_COMPUTE && func == 3'b000 && auxFunc == `AUX_FUNC_SUB) ? (opA - opB) : 32'hXXXXXXXX;

assign slli = (opcode == `OPCODE_COMPUTE && func == 3'b001) ? (opA << opB[4:0]) : 32'hXXXXXXXX;
assign slti = (opcode == `OPCODE_COMPUTE && func == 3'b010) ? (($signed(opA) < $signed(opB)) ? 1 : 0) : 32'hXXXXXXXX;
assign sltiu = (opcode == `OPCODE_COMPUTE && func == 3'b011) ? ((opA < opB) ? 1 : 0) : 32'hXXXXXXXX;
assign logicXor = (opcode == `OPCODE_COMPUTE && func == 3'b100) ? (opA ^ opB) : 32'hXXXXXXXX;
assign srli = (opcode == `OPCODE_COMPUTE && func == 3'b101 && auxFunc[5] == 0) ? (opA >> opB[4:0]) : 32'hXXXXXXXX;
assign srai = (opcode == `OPCODE_COMPUTE && func == 3'b101 && auxFunc[5] == 1) ? ($signed(opA) >>> opB[4:0]) : 32'hXXXXXXXX;
assign logicOr = (opcode == `OPCODE_COMPUTE && func == 3'b110) ? (opA | opB) : 32'hXXXXXXXX;
assign logicAnd = (opcode == `OPCODE_COMPUTE && func == 3'b111) ? (opA & opB) : 32'hXXXXXXXX;

// I-Type指令
assign addi = (opcode == `OPCODE_IMM && func == 3'b000) ? (opA + imm) : 32'hXXXXXXXX;
assign slti = (opcode == `OPCODE_IMM && func == 3'b010) ? (($signed(opA) < $signed(imm)) ? 1 : 0) : 32'hXXXXXXXX;
assign sltiu = (opcode == `OPCODE_IMM && func == 3'b011) ? ((opA < imm) ? 1 : 0) : 32'hXXXXXXXX;
assign xori = (opcode == `OPCODE_IMM && func == 3'b100) ? (opA ^ imm) : 32'hXXXXXXXX;
assign ori = (opcode == `OPCODE_IMM && func == 3'b110) ? (opA | imm) : 32'hXXXXXXXX;
assign andi = (opcode == `OPCODE_IMM && func == 3'b111) ? (opA & imm) : 32'hXXXXXXXX;
assign slli_imm = (opcode == `OPCODE_IMM && func == 3'b001) ? (opA << imm[4:0]) : 32'hXXXXXXXX;
assign srli_imm = (opcode == `OPCODE_IMM && func == 3'b101 && auxFunc[5] == 0) ? (opA >> imm[4:0]) : 32'hXXXXXXXX;
assign srai_imm = (opcode == `OPCODE_IMM && func == 3'b101 && auxFunc[5] == 1) ? ($signed(opA) >>> imm[4:0]) : 32'hXXXXXXXX;

// 特殊指令
assign lui = (opcode == `OPCODE_LUI) ? (imm << 12) : 32'hXXXXXXXX;
assign auipc = (opcode == `OPCODE_AUIPC) ? (opA + imm) : 32'hXXXXXXXX;
assign jal = (opcode == `OPCODE_JAL) ? (opA + imm) : 32'hXXXXXXXX;
assign jalr = (opcode == `OPCODE_JALR) ? ((opA + imm) & ~1) : 32'hXXXXXXXX;

// Branch条件
assign branch = (opcode == `OPCODE_BRANCH && func == 3'b000) ? ((opA == opB) ? 1 : 0) :  // BEQ
                (opcode == `OPCODE_BRANCH && func == 3'b001) ? ((opA != opB) ? 1 : 0) :  // BNE
                (opcode == `OPCODE_BRANCH && func == 3'b100) ? (($signed(opA) < $signed(opB)) ? 1 : 0) :  // BLT
                (opcode == `OPCODE_BRANCH && func == 3'b101) ? (($signed(opA) >= $signed(opB)) ? 1 : 0) :  // BGE
                (opcode == `OPCODE_BRANCH && func == 3'b110) ? ((opA < opB) ? 1 : 0) :  // BLTU
                (opcode == `OPCODE_BRANCH && func == 3'b111) ? ((opA >= opB) ? 1 : 0) : 32'hXXXXXXXX;  // BGEU

// Load和Store指令的地址计算
assign load_store_addr = (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? (opA + imm) : 32'hXXXXXXXX;

// Multiply/Divide指令
assign mul_result = (opcode == `OPCODE_MUL_DIV && func == 3'b000) ? (opA * opB) : 32'hXXXXXXXX;
assign mulh_result = (opcode == `OPCODE_MUL_DIV && func == 3'b001) ? (($signed(opA) * $signed(opB)) >>> 32) : 32'hXXXXXXXX;
assign mulhsu_result = (opcode == `OPCODE_MUL_DIV && func == 3'b010) ? (($signed(opA) * opB) >>> 32) : 32'hXXXXXXXX;
assign mulhu_result = (opcode == `OPCODE_MUL_DIV && func == 3'b011) ? ((opA * opB) >>> 32) : 32'hXXXXXXXX;
assign div_result = (opcode == `OPCODE_MUL_DIV && func == 3'b100) ? ($signed(opA) / $signed(opB)) : 32'hXXXXXXXX;
assign divu_result = (opcode == `OPCODE_MUL_DIV && func == 3'b101) ? (opA / opB) : 32'hXXXXXXXX;
assign rem_result = (opcode == `OPCODE_MUL_DIV && func == 3'b110) ? ($signed(opA) % $signed(opB)) : 32'hXXXXXXXX;
assign remu_result = (opcode == `OPCODE_MUL_DIV && func == 3'b111) ? (opA % opB) : 32'hXXXXXXXX;

// 最终输出选择
assign out = (addsub != 32'hXXXXXXXX) ? addsub :
             (slli != 32'hXXXXXXXX) ? slli :
             (slti != 32'hXXXXXXXX) ? slti :
             (sltiu != 32'hXXXXXXXX) ? sltiu :
             (logicXor != 32'hXXXXXXXX) ? logicXor :
             (srli != 32'hXXXXXXXX) ? srli :
             (srai != 32'hXXXXXXXX) ? srai :
             (logicOr != 32'hXXXXXXXX) ? logicOr :
             (logicAnd != 32'hXXXXXXXX) ? logicAnd :
             (addi != 32'hXXXXXXXX) ? addi :
             (slti != 32'hXXXXXXXX) ? slti :
             (sltiu != 32'hXXXXXXXX) ? sltiu :
             (xori != 32'hXXXXXXXX) ? xori :
             (ori != 32'hXXXXXXXX) ? ori :
             (andi != 32'hXXXXXXXX) ? andi :
             (slli_imm != 32'hXXXXXXXX) ? slli_imm :
             (srli_imm != 32'hXXXXXXXX) ? srli_imm :
             (srai_imm != 32'hXXXXXXXX) ? srai_imm :
             (lui != 32'hXXXXXXXX) ? lui :
             (auipc != 32'hXXXXXXXX) ? auipc :
             (jal != 32'hXXXXXXXX) ? jal :
             (jalr != 32'hXXXXXXXX) ? jalr :
             (branch != 32'hXXXXXXXX) ? branch :
             (load_store_addr != 32'hXXXXXXXX) ? load_store_addr :
             (mul_result != 32'hXXXXXXXX) ? mul_result :
             (mulh_result != 32'hXXXXXXXX) ? mulh_result :
             (mulhsu_result != 32'hXXXXXXXX) ? mulhsu_result :
             (mulhu_result != 32'hXXXXXXXX) ? mulhu_result :
             (div_result != 32'hXXXXXXXX) ? div_result :
             (divu_result != 32'hXXXXXXXX) ? divu_result :
             (rem_result != 32'hXXXXXXXX);

endmodule // ExecutionUnit
