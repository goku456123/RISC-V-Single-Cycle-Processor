// RISC-V SINGLE CYCLE PROCESSOR

module instruction_memory(
  input clk, reset, 
  input[31:0] read_address, 
  output[31:0] instructions_out);
  
  reg[31:0] imemory[63:0];
  
  integer k;
  
  assign instructions_out = imemory[read_address];
  
  always @(posedge clk)
    begin
      if(reset == 1'b1) begin
        for(k=0;k<64;k+=1)
          imemory[k] = 32'b0;
      end
    end
endmodule

module program_counter(
  input clk, reset,
  input [31:0] pc_in,
  output reg[31:0] pc_out);
  
  always @(posedge clk) begin
    if(reset == 1'b1) pc_out <= 32'h0;
    else pc_out <= pc_in;
  end
endmodule

module register_file(
  input clk, reset, regwrite,
  input[4:0] rs1,rs2,rd,
  input[31:0] write_data,
  output[31:0] read_data1, read_data2);
  
  reg[31:0] registers[31:0];
  
  integer k;
  
  always@(posedge clk) begin
    if(reset == 1'b1) begin
      for(k=0;k<32;k+=1) begin
        registers[k] = 32'h0;
      end
    end
    else if (regwrite == 1'b1) begin
      registers[rd] = write_data;
    end
  end
  
  assign read_data1 = registers[rs1];
  assign read_data2 = registers[rs2];
  
endmodule

module immediate_generator(
  input[6:0] Opcode, 
  input[31:0] instruction, 
  output reg[31:0] imm_ext);
  
  always @(*) begin
    case(Opcode)
      7'b0010011: imm_ext = {{20{instruction[31]}},instruction[31:20]};
      7'b0100011: imm_ext = {{20{instruction[31]}},instruction[31:25],instruction[11:7]};
      7'b1100011: imm_ext = {{19{instruction[31]}},instruction[31],instruction[30:25],instruction[11:8]};
      default: imm_ext = {{20{instruction[31]}},instruction[31:20]};
    endcase
  end
  
endmodule

module control_unit(
  input [6:0] opcode,
  output reg branch, memread, memtoread, memwrite, alusrc, regwrite,
  output reg [1:0] aluop
);

  always @(*) begin
    case(opcode)
      7'b0110011: // Register type Instruction
        begin
          alusrc = 0;
          memtoread = 0;
          regwrite = 1;
          memread = 0;
          memwrite = 0;
          branch = 0;
          aluop = 2'b10;
        end
      7'b0000011: // Load Instruction
        begin
          alusrc = 1;
          memtoread = 1;
          regwrite = 1;
          memread = 1;
          memwrite = 0;
          branch = 0;
          aluop = 2'b00;
        end
      7'b0100011: // Store Instruction
        begin
          alusrc = 1;
          memtoread = 0;
          regwrite = 0;
          memread = 0;
          memwrite = 1;
          branch = 0;
          aluop = 2'b00;
        end
      7'b1100011: // Branch-equal Instruction
        begin
          alusrc = 0;
          memtoread = 0;
          regwrite = 0;
          memread = 0;
          memwrite = 0;
          branch = 1;
          aluop = 2'b01;
        end
      default: // Default case
        begin
          alusrc = 0;
          memtoread = 0;
          regwrite = 1;
          memread = 0;
          memwrite = 0;
          branch = 0;
          aluop = 2'b10;
        end
    endcase
  end
endmodule


module ALU(
  input[31:0] a,b,
  input[3:0] ALUcontrol_in,
  output reg zero,
  output reg[31:0] ALU_result);
  
  always @(ALUcontrol_in or a or b)
    begin
      case(ALUcontrol_in)
        4'b0000: 
          begin 
          zero<=0; ALU_result <= a&b;
          end
        4'b0001:
          begin
            zero<=0; ALU_result <= a|b;
          end
        4'b0010:
          begin
            zero<=0; ALU_result <= a+b;
          end
        4'b0110:
          begin
            if(a==b) zero<=1;
            else zero<=0;
            ALU_result <= a-b;
          end
        
        default:
          begin
            zero<=0; ALU_result <= a;
          end
      endcase
    end
endmodule

module ALU_control(
  input [1:0] alu_op,
  input fn7,
  input[14:12] fn3,
  output reg[3:0] ALUcontrol_op);
  
  always @(*) begin
    case({alu_op,fn7,fn3})
      6'b00_0_000 : ALUcontrol_op <= 4'b0010;
      6'b01_0_000 : ALUcontrol_op <= 4'b0110;
      6'b10_0_000 : ALUcontrol_op <= 4'b0010;
      6'b10_1_000 : ALUcontrol_op <= 4'b0110;
      6'b10_1_111 : ALUcontrol_op <= 4'b0000;
      6'b10_0_110 : ALUcontrol_op <= 4'b0001;
      default : ALUcontrol_op <= 4'b0000;
    endcase
  end
endmodule

module data_memory(
  input clk, reset, memwrite, memread,
  input[31:0] address, writedata,
  output[31:0] data_out);
  
  reg[31:0] datamemory[63:0];
  
  assign data_out = (memread) ? datamemory[address] : 32'b0;
  
  integer k;
  
  always @(posedge clk)
    begin
      if(reset == 1'b1) begin
        for(k=0;k<64;k+=1) datamemory[k] = 32'b0;
      end
      else if(memwrite) begin
        datamemory[address] = writedata;
      end
    end
endmodule

module Mux1(
  input sel,
  input[31:0] a1,b1,
  output[31:0] mux1_out);
  
  assign mux1_out = (sel == 1'b0) ? a1:b1;
  
endmodule

module Mux2(
  input sel,
  input[31:0] a2,b2,
  output[31:0] mux2_out);
  
  assign mux2_out = (sel == 1'b0) ? a2:b2;
  
endmodule

module Mux3(
  input sel,
  input[31:0] a3,b3,
  output[31:0] mux3_out);
  
  assign mux3_out = (sel == 1'b0) ? a3:b3;
  
endmodule


module top_module(
  input clk, reset
);

  // Wires to connect the submodules
  wire [31:0] pc_out, pc_in;
  wire [31:0] instruction;
  wire [31:0] imm_ext;
  wire [31:0] read_data1, read_data2;
  wire [31:0] alu_result;
  wire [31:0] write_data;
  wire [31:0] mem_data_out;
  wire [31:0] alu_mux_b;
  wire [31:0] branch_addr;
  wire [31:0] pc_plus_4;
  
  wire [6:0] opcode;
  wire [2:0] funct3;
  wire [6:0] funct7;
  wire [4:0] rs1, rs2, rd;
  wire branch, memread, memtoreg, memwrite, alusrc, regwrite;
  wire [1:0] aluop;
  wire [3:0] alucontrol_op;
  wire zero;

  // PC Incrementer
  assign pc_plus_4 = pc_out + 4;

  // Branch Address Calculation
  assign branch_addr = pc_out + (imm_ext << 1);

  // Instruction Memory
  instruction_memory imem(
    .clk(clk),
    .reset(reset),
    .read_address(pc_out),
    .instructions_out(instruction)
  );

  // Program Counter
  program_counter pc(
    .clk(clk),
    .reset(reset),
    .pc_in(pc_in),
    .pc_out(pc_out)
  );

  // Instruction Fields
  assign opcode = instruction[6:0];
  assign funct3 = instruction[14:12];
  assign funct7 = instruction[31:25];
  assign rs1 = instruction[19:15];
  assign rs2 = instruction[24:20];
  assign rd = instruction[11:7];

  // Register File
  register_file regfile(
    .clk(clk),
    .reset(reset),
    .regwrite(regwrite),
    .rs1(rs1),
    .rs2(rs2),
    .rd(rd),
    .write_data(write_data),
    .read_data1(read_data1),
    .read_data2(read_data2)
  );

  // Immediate Generator
  immediate_generator immgen(
    .Opcode(opcode),
    .instruction(instruction),
    .imm_ext(imm_ext)
  );

  // Control Unit
  control_unit cu(
    .opcode(opcode),
    .branch(branch),
    .memread(memread),
    .memtoread(memtoreg),
    .memwrite(memwrite),
    .alusrc(alusrc),
    .regwrite(regwrite),
    .aluop(aluop)
  );

  // ALU Control Unit
  ALU_control alu_ctrl(
    .alu_op(aluop),
    .fn7(funct7[5]),
    .fn3(funct3),
    .ALUcontrol_op(alucontrol_op)
  );

  // ALU
  ALU alu(
    .a(read_data1),
    .b(alu_mux_b),
    .ALUcontrol_in(alucontrol_op),
    .zero(zero),
    .ALU_result(alu_result)
  );

  // Data Memory
  data_memory dmem(
    .clk(clk),
    .reset(reset),
    .memwrite(memwrite),
    .memread(memread),
    .address(alu_result),
    .writedata(read_data2),
    .data_out(mem_data_out)
  );

  // Muxes
  Mux1 alu_mux(
    .sel(alusrc),
    .a1(read_data2),
    .b1(imm_ext),
    .mux1_out(alu_mux_b)
  );

  Mux2 mem_mux(
    .sel(memtoreg),
    .a2(alu_result),
    .b2(mem_data_out),
    .mux2_out(write_data)
  );

  Mux3 branch_mux(
    .sel(branch & zero),
    .a3(pc_plus_4),
    .b3(branch_addr),
    .mux3_out(pc_in)
  );

endmodule
