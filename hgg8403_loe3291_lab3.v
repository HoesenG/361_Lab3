// Template for Northwestern - CompEng 361 - Lab3 -- Version 1.1
// Groupname:
// NetIDs:

// Some useful defines...please add your own
`define WORD_WIDTH 32
`define NUM_REGS 32

`define OPCODE_LUI        7'b0110111
`define OPCODE_AUIPC      7'b0010111
`define OPCODE_JAL        7'b1101111
`define OPCODE_JALR       7'b1100111
`define OPCODE_IMM        7'b0010011
`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_STORE      7'b0100011 

`define FUNC_ADD      3'b000
`define FUNC_SLL      3'b001
`define FUNC_SLT      3'b010
`define FUNC_SLTU     3'b011
`define FUNC_XOR      3'b100
`define FUNC_SRL      3'b101
`define FUNC_OR       3'b110
`define FUNC_AND      3'b111
`define FUNC_B       3'b000
`define FUNC_H       3'b001
`define FUNC_W       3'b010
`define FUNC_BU     3'b100
`define FUNC_HU     3'b101


`define AUX_FUNC_ADD  7'b0000000
`define AUX_FUNC_SUB  7'b0100000
`define AUX_FUNC_MUL_DIV  7'b0000001

`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10


module SingleCycleCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [`WORD_WIDTH-1:0] PC, InstWord;
   wire [`WORD_WIDTH-1:0] DataAddr, StoreData, DataWord;
   wire [1:0]  MemSize;
   wire        MemWrEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst;
   wire [`WORD_WIDTH-1:0] Rdata1, Rdata2, SrcB, RWrdata;
   wire        RWrEn;

   wire [`WORD_WIDTH-1:0] NPC, PC_Plus_4;
   wire [6:0]  opcode;

   wire [6:0]  funct7;
   wire [2:0]  funct3;
   wire [31:0] imm;


   wire [2:0] imm_sel, Load_sel;
   wire EUSrc;
   wire [1:0] Store_sel, RWr_sel, NPC_sel;
   wire MemtoReg;
   

   wire [31:0] StoreData_B;
   wire [31:0] StoreData_H;
   wire [31:0] StoreData_W;

   wire [31:0] LoadData_B;
   wire [31:0] LoadData_H;
   wire [31:0] LoadData_W;
   wire [31:0] LoadData_BU;
   wire [31:0] LoadData_HU;
   wire [31:0] LoadData;

   wire [31:0] Result;
   wire [31:0] ExecutionResult;

   wire [31:0] PC_AUIPC, PC_branch, PC_jal, PC_jalr;

   

   wire invalid_op;
   
   // Only support R-TYPE ADD and SUB
   assign halt = invalid_op;
   assign invalid_op = !(((opcode == `OPCODE_COMPUTE) || (opcode == `OPCODE_AUIPC)
             || (opcode == `OPCODE_BRANCH) || (opcode == `OPCODE_IMM) || (opcode == `OPCODE_JAL)
             || (opcode == `OPCODE_JALR) || (opcode == `OPCODE_LOAD) || (opcode == `OPCODE_LUI) 
             || (opcode == `OPCODE_COMPUTE) || (opcode == `OPCODE_STORE)) && (funct3 == `FUNC_ADD) &&
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

   // Control Unit
   Control CU(.funct7(funct7), .funct3(funct3), .opcode(opcode), .logic_result(ExecutionResult[0]),.MemWrEn(MemWrEn), .RWrEn(RWrEn), .imm_sel(imm_sel), .EUSrc(EUSrc), .Store_sel(Store_sel), .Load_sel(Load_sel), .MemtoReg(MemtoReg), .RWr_sel(RWr_sel), .NPC_sel(NPC_sel));
   // Immediate Generation
   ImmGen IMM(.Inst(InstWord), .imm_sel(imm_sel), .imm(imm));

   // Hardwired to support R-Type instructions -- please add muxes and other control signals
   MUX32_2_1 MUX_opB(.a(Rdata2), .b(imm), .sel(EUSrc), .o(SrcB));
   ExecutionUnit EU(.out(ExecutionResult), .opA(Rdata1), .opB(SrcB), .func(funct3), .auxFunc(funct7), .opcode(opcode));

   // StoreData
   assign StoreData_B = { {24{Rdata2[7]}}, Rdata2[7:0]};
   assign StoreData_H = { {16{Rdata2[15]}}, Rdata2[15:0]};
   assign StoreData_W = Rdata2;
   MUX32_4_1 MUX_StoreData(.a(StoreData_B), .b(StoreData_H), .c(StoreData_W), .d(StoreData_W), .sel(Store_sel), .o(StoreData));

   // LoadData
   assign LoadData_B = { {24{DataWord[7]}}, DataWord[7:0]};
   assign LoadData_H = { {16{DataWord[15]}}, DataWord[15:0]};
   assign LoadData_W = DataWord;
   assign LoadData_BU = {16'b0, DataWord[15:0]};
   assign LoadData_HU = {16'b0, DataWord[15:0]};
   MUX32_8_1 MUX_LoadData(.a(LoadData_B), .b(LoadData_H), .c(LoadData_W), .d(LoadData_BU), .e(LoadData_HU), .f(LoadData_W), .g(LoadData_W), .h(LoadData_W), .sel(Load_sel), .o(LoadData));

   // Result
   MUX32_2_1 MUX_Result(.a(ExecutionResult), .b(LoadData), .sel(MemtoReg), .o(Result));

   // write to register file
   assign PC_AUIPC = PC + imm;
   MUX32_4_1 MUX_RWrdata(.a(PC_Plus_4), .b(PC_AUIPC), .c(Result), .d(Result), .sel(RWr_sel), .o(RWrdata));


   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
   assign PC_branch = PC + imm;
   assign PC_jal = PC + imm;
   assign PC_jalr = Result;
   MUX32_4_1 MUX_NPC(.a(PC_Plus_4), .b(PC_branch), .c(PC_jal), .d(PC_jalr), .sel(NPC_sel), .o(NPC));
   
endmodule // SingleCycleCPU


// Incomplete version of Lab2 execution unit
// You will need to extend it. Feel free to modify the interface also

module ExecutionUnit(out, opA, opB, func, auxFunc, opcode);
   output [`WORD_WIDTH-1:0] out;
   input [`WORD_WIDTH-1:0] opA, opB;
   input [2:0] func;
   input [6:0] auxFunc, opcode;

   wire [`WORD_WIDTH-1:0] add, sub, slli, srli, srai,
                        logicAnd, logicOr, logicXor, slti, sltiu,
                        mul, mulh, mulhsu, mulhu, div, divu, rem, remu,
                        lui, auipc, jal, jalr, branch;

   assign add = opA + opB;
   assign sub = opA - opB;
   assign slli = opA << opB[4:0];
   assign compLT = ($signed(opA) < $signed(opB)) ? 1 : 0;
   assign compLTU = (opA < opB) ? 1 : 0;
   assign logicXor = opA ^ opB;
   assign srli = opA >> opB[4:0];
   assign srai = $signed(opA) >>> opB[4:0];
   assign logicOr = opA | opB;
   assign logicAnd = opA & opB;

   assign addi = opA + opB;
   assign slti = ($signed(opA) < $signed(opB)) ? 1 : 0;
   assign sltiu = (opA < opB) ? 1 : 0;
   assign xori = opA ^ opB;
   assign ori = opA | opB;
   assign andi = opA & opB;
   assign slli_imm = opA << opB[4:0];
   assign srli_imm = opA >> opB[4:0];
   assign srai_imm = $signed(opA) >>> opB[4:0];

   assign lui = opB << 12;
   assign auipc = opA + opB;
   assign jal = opA + opB;
   assign jalr = (opA + opB) & ~1;

   assign branch = (func == 3'b000 && opA == opB) ? 1 :         // BEQ
                  (func == 3'b001 && opA != opB) ? 1 :         // BNE
                  (func == 3'b100 && $signed(opA) < $signed(opB)) ? 1 : // BLT
                  (func == 3'b101 && $signed(opA) >= $signed(opB)) ? 1 : // BGE
                  (func == 3'b110 && opA < opB) ? 1 :          // BLTU
                  (func == 3'b111 && opA >= opB) ? 1 : 0;      // BGEU

   assign load_store_addr = opA + opB;

   assign mul_result = opA * opB;
   assign mulh_result = ($signed(opA) * $signed(opB)) >>> 32;
   assign mulhsu_result = ($signed(opA) * opB) >>> 32;
   assign mulhu_result = (opA * opB) >>> 32;
   assign div_result = $signed(opA) / $signed(opB);
   assign divu_result = opA / opB;
   assign rem_result = $signed(opA) % $signed(opB);
   assign remu_result = opA % opB;

   assign out = (opcode == `OPCODE_COMPUTE) ? 
                     ((func == 3'b000) ? (auxFunc == 7'b0000000 ? add : sub) :
                        (func == 3'b001) ? slli :
                        (func == 3'b010) ? compLT :
                        (func == 3'b011) ? compLTU :
                        (func == 3'b100) ? logicXor :
                        (func == 3'b101) ? (auxFunc[5] ? srai : srli) :
                        (func == 3'b110) ? logicOr :
                        (func == 3'b111) ? logicAnd : 32'hXXXXXXXX) :
                  (opcode == `OPCODE_IMM) ? 
                     ((func == 3'b000) ? addi :
                        (func == 3'b010) ? slti :
                        (func == 3'b011) ? sltiu :
                        (func == 3'b100) ? xori :
                        (func == 3'b110) ? ori :
                        (func == 3'b111) ? andi :
                        (func == 3'b001) ? slli_imm :
                        (func == 3'b101) ? (auxFunc[5] ? srai_imm : srli_imm) :
                        32'hXXXXXXXX) :
                  (opcode == `OPCODE_LUI) ? lui :
                  (opcode == `OPCODE_AUIPC) ? auipc :
                  (opcode == `OPCODE_JAL) ? jal :
                  (opcode == `OPCODE_JALR) ? jalr :
                  (opcode == `OPCODE_BRANCH) ? branch :
                  (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? load_store_addr :
                  (opcode == `OPCODE_COMPUTE || auxFunc == `OPCODE_COMPUTE) ?
                     ((func == 3'b000) ? mul_result :
                        (func == 3'b001) ? mulh_result :
                        (func == 3'b010) ? mulhsu_result :
                        (func == 3'b011) ? mulhu_result :
                        (func == 3'b100) ? div_result :
                        (func == 3'b101) ? divu_result :
                        (func == 3'b110) ? rem_result :
                        (func == 3'b111) ? remu_result :
                        32'hXXXXXXXX) :
                  32'hXXXXXXXX;

endmodule // ExecutionUnit

module Control(funct7, funct3, opcode, logic_result, MemWrEn, RWrEn, imm_sel, EUSrc, Store_sel, Load_sel, MemtoReg, RWr_sel, NPC_sel);
      input [6:0] funct7;
      input [2:0] funct3;
      input [6:0] opcode;
      input logic_result;
      output [2:0] imm_sel, Load_sel;
      output EUSrc;
      output [1:0] Store_sel, RWr_sel, NPC_sel;
      output MemtoReg;
      output MemWrEn, RWrEn;
   
      assign MemWrEn = (opcode == `OPCODE_STORE) ? 1'b1 : 1'b0;
      assign RWrEn = (opcode == `OPCODE_COMPUTE || opcode == `OPCODE_IMM || opcode == `OPCODE_LUI || opcode == `OPCODE_AUIPC || opcode == `OPCODE_LOAD || opcode == `OPCODE_JAL) ? 1 : 0;
      assign imm_sel = (opcode == `OPCODE_IMM && funct3 == `FUNC_SLTU) ? 3'b100 : 
                       (opcode == `OPCODE_BRANCH) ? 3'b110 : 
                       (opcode == `OPCODE_LOAD || (opcode == `OPCODE_IMM && funct3 != `FUNC_SLTU) ) ? 3'b011 : 
                       (opcode == `OPCODE_STORE) ? 3'b101 : 
                       (opcode == `OPCODE_LUI || opcode == `OPCODE_AUIPC) ? 3'b000 :
                       (opcode == `OPCODE_JAL) ? 3'b001 : 
                       (opcode == `OPCODE_JALR) ? 3'b010 : 3'b000;
      assign EUSrc = (opcode == `OPCODE_IMM || opcode == `OPCODE_LUI || opcode == `OPCODE_AUIPC || opcode == `OPCODE_JAL || opcode == `OPCODE_JALR || opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE ) ? 1'b1 : 1'b0;
      assign Store_sel = (opcode == `OPCODE_STORE) ? 
                           (funct3 == `FUNC_B) ? 2'b00 :
                           (funct3 == `FUNC_H) ? 2'b01 :
                           (funct3 == `FUNC_W) ? 2'b10 : 2'b00
                           : 2'b00;
      assign Load_sel = (opcode == `OPCODE_LOAD) ? 
                           (funct3 == `FUNC_B) ? 3'b000 : 
                           (funct3 == `FUNC_H) ? 3'b001 :
                           (funct3 == `FUNC_W) ? 3'b010 :
                           (funct3 == `FUNC_BU) ? 3'b011 :
                           (funct3 == `FUNC_HU) ? 3'b100 : 3'b000
                           : 3'b000;
      assign MemtoReg = (opcode == `OPCODE_LOAD) ? 1'b1 : 1'b0;
      assign RWr_sel = (opcode == `OPCODE_LUI || opcode == `OPCODE_LOAD || opcode == `OPCODE_IMM || opcode == `OPCODE_COMPUTE) ? 2'b10 :
                        (opcode == `OPCODE_AUIPC) ? 2'b01 :
                        (opcode == `OPCODE_JAL || opcode == `OPCODE_JALR) ? 2'b00 : 
                        2'b10;
      assign NPC_sel = (opcode == `OPCODE_BRANCH) ? 2'b01 :
                         (opcode == `OPCODE_JAL) ? 2'b10 : 
                           (opcode == `OPCODE_JALR) ? 2'b11 : 
                           2'b00;
endmodule


module ImmGen(Inst, imm_sel, imm);
    input [31:0] Inst;
    input [2:0] imm_sel;
    output [31:0] imm;
 
    wire [31:0] imm_U;
    wire [31:0] imm_J;
    wire [31:0] imm_I;
    wire [31:0] imm_S;
    wire [31:0] imm_B;

   assign imm_U = {Inst[31:12], 12'b0};
   assign imm_JAL = { {11{Inst[31]}}, Inst[31], Inst[19:12], Inst[20], Inst[30:21], 1'b0 };
   assign imm_JALR = {{20{Inst[31]}}, Inst[31:20]};
   assign imm_I_s = {{20{Inst[31]}}, Inst[31:20]};
   assign imm_I_u = { 20'b0, Inst[31:20] };
   assign imm_S = { {20{Inst[31]}}, Inst[31:25], Inst[11:7]};
   assign imm_B = { {20{Inst[31]}}, Inst[7], Inst[30:25], Inst[11:8], 1'b0};

   assign imm = (imm_sel == 3'b000) ? imm_U :
                (imm_sel == 3'b001) ? imm_JAL :
                (imm_sel == 3'b010) ? imm_JALR :
                (imm_sel == 3'b011) ? imm_I_s :
                (imm_sel == 3'b100) ? imm_I_u :
                (imm_sel == 3'b101) ? imm_S :
                (imm_sel == 3'b110) ? imm_B : 32'b0;
endmodule

module MUX32_8_1(a, b, c, d, e, f, g, h, sel, o);
    input [31:0] a, b, c, d, e, f, g, h;  
    input [2:0] sel;          
    output [31:0] o;          

   
    assign o = (sel == 3'b000) ? a :
               (sel == 3'b001) ? b :
               (sel == 3'b010) ? c :
               (sel == 3'b011) ? d :
               (sel == 3'b100) ? e :
               (sel == 3'b101) ? f :
               (sel == 3'b110) ? g :
               (sel == 3'b111) ? h : 32'b0;  // 0 BY DEFAULT
endmodule


module MUX32_4_1(a, b, c, d, sel, o);
    input [31:0] a, b, c, d;  
    input [1:0] sel;          
    output [31:0] o;          

   
    assign o = (sel == 2'b00) ? a :
               (sel == 2'b01) ? b :
               (sel == 2'b10) ? c :
               (sel == 2'b11) ? d : 32'b0;  // 0 BY DEFAULT
endmodule

module MUX32_2_1(a, b, sel, o);
    input [31:0] a, b;  
    input sel;          
    output [31:0] o;          

   
    assign o = (sel == 1'b0) ? a :
               (sel == 1'b1) ? b : 32'b0;  // 0 BY DEFAULT
endmodule
