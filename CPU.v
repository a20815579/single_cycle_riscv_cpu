// Please include verilog file if you write module in other file
module CPU(
    input             clk,
    input             rst,
    input      [31:0] data_out,
    input      [31:0] instr_out,
    output            instr_read,
    output            data_read,
    output     [31:0] instr_addr,
    output     [31:0] data_addr,
    output reg [3:0]  data_write,
    output reg [31:0] data_in
);

/* Add your design */


endmodule

module ControlUnit(
 input [6:0] opcode,
 output reg r2_enable, rd_enable, UorJ_anable, UorJ_type,
 output reg RegWr, PCSrc, ALUSrc, RDSrc, MemRe, MemWr, MemtoReg,
 output reg [2:0] ALUop, ImmType
 ); 

always@(opcode)
begin
	case(opcode)
		7'b0110011:
		begin
			ImmType = 3'b000
			RegWr = 1
			PCSrc = 0
			ALUSrc = 0
			RDSrc = 1
			MemRe = 0
			MemWr = 0
			MemtoReg = 0
			ALUop = 3'b000
		end
		7'b0000011:
		begin
			ImmType = 3'b001
			RegWr = 1
			PCSrc = 0
			ALUSrc = 1
			RDSrc = 0
			MemRe = 0
			MemWr = 0
			MemtoReg = 0
			ALUop = 3'b001
		end
		7'b0010011:
		begin
			ImmType = 3'b001
			RegWr = 1
			PCSrc = 0
			ALUSrc = 1
			RDSrc = 0
			MemRe = 0
			MemWr = 0
			MemtoReg = 0
			ALUop = 3'b010
		end
		7'b1100111:
		begin
			ImmType = 0
			RegWr = 0
			PCSrc = 1
			ALUSrc = 1
			RDSrc = 0
			MemRe = 0
			MemWr = 0
			MemtoReg = 0
			ALUop = 3'b011
		end
		7'b0100011:
		begin
			ImmType = 0
			RegWr = 0
			PCSrc = 1
			ALUSrc = 1
			RDSrc = 0
			MemRe = 0
			MemWr = 0
			MemtoReg = 0
			ALUop = 3'b011
		end
		7'b1100011:
		begin
			ImmType = 1
			RegWr = 0
			PCSrc = 1
			ALUSrc = 1
			RDSrc = 0
			MemRe = 0
			MemWr = 0
			MemtoReg = 0
			ALUop = 3'b011
		end
	endcase
end
endmodule

module ALU(
	input [3:0] ALUc,
	input [31:0] a,
	input [31:0] b,
	output reg zeroFlag,
	output reg [31:0] result
);

always@(*)
begin
	case(ALUc)
		4'b0000:
			result = a + b;
		4'b0001:
			result = a - b;
		4'b0010:
			result = a << b;
		4'b0011:
			result = a >> b;
		4'b0100:
			result = a ^ b;
		4'b0101:
			result = a | b;
		4'b0110:
			result = a & b;
		4'b0111:
			zeroFlag = a < b;
		4'b1000:
			zeroFlag = a == b;
		4'b1001:
			zeroFlag = a != b;
		default:
			zeroFlag = a >= b;
	endcase
end
endmodule

module ALUCtrlUnit(
	input funct7_2,
	input [2:0] funct3,
	input [2:0] ALUop,
	output reg [3:0] ALUc,
	output reg [3:0] dataWr,
	output reg unsign
);
always@(*)
begin
	if(ALUop == 3'b000)
	begin
		case(funct3)
			3'b000: begin
				ALUC = funct7_2? 4'b0001:4'b0000;
				unsign = 0;
			end
			3'b001: begin
				ALUC = 4'b0010;
				unsign = 1;
			end
			3'b010: begin
				ALUC = 4'b0111;
				unsign = 0;
			end
			3'b011: begin
				ALUC = 4'b0111;
				unsign = 1;
			end
			3'b100: begin
				ALUC = 4'b0100;
				unsign = 0;
			end
			3'b101: begin
				ALUC = 4'b0011;
				unsign = ~funct7_2;
			end
			3'b110: begin
				ALUC = 4'b0101;
				unsign = 0;
			end
			3'b111: begin
				ALUC = 4'b0110;
				unsign = 0;
			end
		endcase
	end
	else if(ALUop == 3'b001)
	begin
		case(funct3)
			3'b000: begin
				ALUc = 4'b0000;
				unsign = 0;
				dataWr = 4'b0001;
			end
			3'b001: begin
				ALUc = 4'b0000;
				unsign = 0;
				dataWr = 4'b0011;
			end
			3'b010: begin
				ALUc = 4'b0000;
				unsign = 0;
				dataWr = 4'b1111;
			end
			3'b100: begin
				ALUc = 4'b0000;
				unsign = 1;
				dataWr = 4'b0001;
			end
			default: begin
				ALUc = 4'b0000;
				unsign = 1;
				dataWr = 4'b0011;
			end
		endcase
	end
	else if(ALUop == 3'b010)
	begin
		case(funct3)
			3'b000: begin
				ALUc = 4'b0000;
				unsign = 0;
			end
			3'b010: begin
				ALUc = 4'b0111;
				unsign = 0;
			end			
			3'b011: begin
				ALUc = 4'b0111;
				unsign = 1;
			end
			3'b100: begin
				ALUc = 4'b0100;
				unsign = 0;
			end
			3'b110: begin
				ALUc = 4'b0101;
				unsign = 0;
			end
			3'b111: begin
				ALUc = 4'b0110;
				unsign = 0;
			end
			3'b001: begin
				ALUc = 4'b0010;
				unsign = 1;
			end
			3'b101: begin
				ALUc = 4'b0011;
				unsign = ~funct7_2;
			end
		endcase
	end
	else if(ALUop == 3'b011)
	begin
		ALUc = 4'b0000;
		unsign = 0;
	end
	else if(ALUop == 3'b100)
	begin
		case(funct3)
			3'b000: begin
				ALUc = 4'b0000;
				unsign = 0;
				dataWr = 4'b0001; 
			end
			3'b001: begin
				ALUc = 4'b0000;
				unsign = 0;
				dataWr = 4'b0011; 
			end
			default: begin
				ALUc = 4'b0000;
				unsign = 0;
				dataWr = 4'b1111; 
			end
		endcase
	end
	else
	begin
		case(funct3)
			3'b000: begin
				ALUc = 4'b1000;
				unsign = 0;
			end
			3'b001: begin
				ALUc = 4'b0001;
				unsign = 0;
			end
			3'b100: begin
				ALUc = 4'b0111;
				unsign = 0;
			end
			3'b101: begin
				ALUc = 4'b1010;
				unsign = 0;
			end
			3'b110: begin
				ALUc = 4'b0111;
				unsign = 1;
			end
			default: begin
				ALUc = 4'b1010;
				unsign = 1;
			end
		endcase
	end
end
endmodule

module RegFile(
	input [4:0]rs1_addr, rs2_addr, rd_addr,
	input [31:0] rd_data,
	input regWr, clk, rst,
	output reg [31:0]  rs1_data, rs2_data
	);
reg [31:0] register [1:31];

assign rs1_data = (rs1_addr == 0) ? 0 : register[rs1_addr];
assign rs2_data = (rs2_addr == 0) ? 0 : register[rs2_addr];
 
always @(posedge clk or negedge rst)
begin 
	if(!rst)
	begin
		 integer i;
		 for(i = 1; i < 32; i = i + 1)
			  register[i] <= 0;
	end 
	else  if( rd_addr! = 0 && regWr)
		register[rd_addr]  <= d;
	end
endmodule

module Extend(
    input [11:0] origin,
    input unsign,
    output reg [31:0] result
    );    
always@(*)
begin
	if (unsign == 0 || origin[11] == 0) 
		result = {20'b00000000000000000000, origin};
	else
		result = {20'b11111111111111111111, origin};
end    
endmodule

module BranchCtrl(
    input clk, rst, 
    input PCSrc, 
    input [31:0] branch,
    output reg [31:0] pc_addr
    );   
	 
initial begin  
	pc_addr = 0;  
end

always@(posedge clk or negedge rst)
begin  
	if(!rst)
		pc_addr <= 0;   
	else if(PCSrc)
		pc_addr <= branch;
	else
		pc_addr <= pc_addr + 4;
end   
endmodule

module DealImme(
	input [19:0] first20;
	input [2:0] type;
);
endmodule

module Shift(
	input [31:0] origin,
	input [4:0] shamt,
	input unsign,right,
	output reg [31:0] result
);
always@(*)
begin
    if(!right)
        result = origin << shamt;
    else if(unsign)
        result = origin >> shamt;
    else                    
        result = $signed(origin) >>> shamt;
end
endmodule
