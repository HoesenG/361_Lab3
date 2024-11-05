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
`define OPCODE_MULT_DIV   7'b0110011

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
   assign invalid_op = !((opcode == `OPCODE_COMPUTE) && (funct3 == `FUNC_ADD) &&
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
                        logicAnd, logicOr, logicXor, compLT, compLTU,
                        mul, mulh, mulhsu, mulhu, div, divu, rem, remu,
                        lui, auipc, jal, jalr, branch;

   assign addsub = (auxFunc == 7'b0100000) ? (opA - opB) : (opA + opB);
   assign slli = opA << opB[4:0];
   assign srli = opA >> opB[4:0];
   assign srai = $signed(opA) >>> opB[4:0];

   assign logicAnd = opA & opB;
   assign logicOr = opA | opB;
   assign logicXor = opA ^ opB;

   assign compLT = ($signed(opA) < $signed(opB)) ? 1 : 0;
   assign compLTU = (opA < opB) ? 1 : 0;

   assign mul = opA * opB;
   assign mulh = ($signed(opA) * $signed(opB)) >>> 32;
   assign mulhsu = ($signed(opA) * opB) >>> 32;
   assign mulhu = (opA * opB) >>> 32;
   assign div = $signed(opA) / $signed(opB);  // Signed division
   assign divu = opA / opB;                   // Unsigned division
   assign rem = $signed(opA) % $signed(opB);  // Signed remainder
   assign remu = opA % opB;                   // Unsigned remainder

   assign lui = imm << 12;
   assign auipc = opA + imm;
   assign jal = opA + imm;
   assign jalr = (opA + imm) & ~1;

   assign branch = (func == 3'b000 && (opA == opB)) ? 1 :  // beq
                        (func == 3'b001 && (opA != opB)) ? 1 :  // bne
                        (func == 3'b100 && ($signed(opA) < $signed(opB))) ? 1 :  // blt
                        (func == 3'b101 && ($signed(opA) >= $signed(opB))) ? 1 :  // bge
                        (func == 3'b110 && (opA < opB)) ? 1 :  // bltu
                        (func == 3'b111 && (opA >= opB)) ? 1 :  // bgeu
                        0;

   // Final Output Selection Based on Opcode and Func
   assign out = (opcode == `OPCODE_COMPUTE) ? 
                    ((func == 3'b000) ? addSub : 
                     (func == 3'b001) ? shiftLeft :
                     (func == 3'b010) ? compLT : 
                     (func == 3'b011) ? compLTU :
                     (func == 3'b100) ? logicXor : 
                     (func == 3'b101) ? (auxFunc[5] ? shiftRightArith : shiftRight) : 
                     (func == 3'b110) ? logicOr : 
                     (func == 3'b111) ? logicAnd : 32'hXXXXXXXX) :
                (opcode == `OPCODE_IMM) ? 
                    ((func == 3'b000) ? (opA + imm) : 
                     (func == 3'b010) ? (($signed(opA) < $signed(imm)) ? 1 : 0) : 
                     (func == 3'b011) ? ((opA < imm) ? 1 : 0) : 
                     (func == 3'b100) ? (opA ^ imm) : 
                     (func == 3'b110) ? (opA | imm) : 
                     (func == 3'b111) ? (opA & imm) : 
                     (func == 3'b001) ? (opA << imm[4:0]) : 
                     (func == 3'b101) ? (auxFunc[5] ? ($signed(opA) >>> imm[4:0]) : (opA >> imm[4:0])) :
                     32'hXXXXXXXX) :
                (opcode == `OPCODE_LUI) ? lui :
                (opcode == `OPCODE_AUIPC) ? auipc :
                (opcode == `OPCODE_JAL) ? jal :
                (opcode == `OPCODE_JALR) ? jalr :
                (opcode == `OPCODE_BRANCH) ? branch :
                (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? (opA + imm) :
                (opcode == `OPCODE_MUL_DIV) ?
                    ((func == 3'b000) ? mul :
                     (func == 3'b001) ? mulh :
                     (func == 3'b010) ? mulhsu :
                     (func == 3'b011) ? mulhu :
                     (func == 3'b100) ? div :
                     (func == 3'b101) ? divu :
                     (func == 3'b110) ? rem :
                     (func == 3'b111) ? remu : 32'hXXXXXXXX) :
                32'hXXXXXXXX; // Default case for unsupported operations

endmodule // ExecutionUnit
