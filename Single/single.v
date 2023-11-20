module ALU (alu_func, aluA, aluB, aluZ, CF, OF);
	input   [7:0] aluA;
	input	[7:0] aluB;
	
	input	[4:0] alu_func;
	output reg  [7:0] aluZ;
	// output  	  ZF;
	// output  	  SF;
	output reg	  CF;
	output reg	  OF;
	// assign ZF = (aluZ == 8'b0000000) ? 1 : 0;
	// assign SF = OF ? CF : aluZ[7];
	always @(*) begin
		case(alu_func)
			5'b00001://ADD
			begin
				{CF,aluZ} <= aluA + aluB;
				if((!aluA[7] & !aluB[7] & aluZ[7]) | (aluA[7] & aluB[7] & !aluZ[7]))
					OF <= 1;
				else
					OF <= 0;
			end
			5'b00010://AND
			begin
				aluZ <= aluA & aluB;
				OF <= 0;
				CF <= 0;
			end
			5'b00011://SUB
			begin
				
				{CF,aluZ} <= aluA - aluB;
				if((!aluA[7] & !aluB[7] & aluZ) | (aluA[7] & aluB[7] & !aluZ[7]))
					OF <= 1;
				else
					OF <= 0;
			end
			5'b00100://OR
			begin
				aluZ <= aluA | aluB;
				OF <= 0;
				CF <= 0;
			end
			5'b00101://XOR
			begin
				aluZ <= aluA ^ aluB;
				OF <= 0;
				CF <= 0;
			end
			5'b00110://MOV
			begin
				aluZ <= aluB;
				OF <= 0;
				CF <= 0;
			end
			5'b01000://NOT
			begin
				aluZ <= ~aluA;
				OF <= 0;
				CF <= 0;
			end
			5'b01001://SAR
			begin
				CF <= aluA[aluB-1];
				OF <= 0;
				aluZ <= aluA >>> aluB;
			end
			5'b01010://SLR
			begin
				CF <= aluA[aluB-1];
				aluZ <= aluA >> aluB;
				if(aluZ[7] == aluA[7])
					OF <= 0;
				else
					OF <= 1;
			end
			5'b01011://SAL
			begin
				CF <= aluA[8'd8 - aluB];
				OF <= 0;
				aluZ <= aluA <<< aluB;
			end
			5'b01100://SLL
			begin
				CF <= aluA[8'd8 - aluB];
				aluZ <= aluA << aluB;
				if(aluZ[7] == aluA[7])
					OF <= 0;
				else
					OF <= 1;
			end
			5'b01101://ROL
			begin
				case(aluB)
					8'd1:
					begin
						CF <= aluA[7];
						aluZ <= {aluA[6:0],aluA[7]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd2:
					begin
						CF <= aluA[6];
						aluZ <= {aluA[5:0],aluA[7:6]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd3:
					begin
						CF <= aluA[5];
						aluZ <= {aluA[4:0],aluA[7:5]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd4:
					begin
						CF <= aluA[4];
						aluZ <= {aluA[3:0],aluA[7:4]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd5:
					begin
						CF <= aluA[3];
						aluZ <= {aluA[2:0],aluA[7:3]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd6:
					begin
						CF <= aluA[2];
						aluZ <= {aluA[1:0],aluA[7:2]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd7:
					begin
						CF <= aluA[1];
						aluZ <= {aluA[0],aluA[7:1]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd8:
					begin
						CF <= aluA[0];
						aluZ <= aluA;
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
				endcase
			end
			5'b01110://ROR
			begin
				case(aluB)
					8'd1:
					begin
						CF <= aluA[0];
						aluZ <= {aluA[0],aluA[7:1]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd2:
					begin
						CF <= aluA[1];
						aluZ <= {aluA[1:0],aluA[7:2]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd3:
					begin
						CF <= aluA[2];
						aluZ <= {aluA[2:0],aluA[7:3]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd4:
					begin
						CF <= aluA[3];
						aluZ <= {aluA[3:0],aluA[7:4]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd5:
					begin
						CF <= aluA[4];
						aluZ <= {aluA[4:0],aluA[7:5]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd6:
					begin
						CF <= aluA[5];
						aluZ <= {aluA[5:0],aluA[7:6]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd7:
					begin
						CF <= aluA[6];
						aluZ <= {aluA[6:0],aluA[7]};
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
					8'd8:
					begin
						CF <= aluA[7];
						aluZ <= aluA[7:1];
						if(aluZ[7] == aluA[7])
							OF <= 0;
						else
							OF <= 1;
					end
				endcase
			end
			5'b01111://INC
			begin
				{CF,aluZ} <= aluA + 8'd1;
				if(!aluA[7] & aluZ[7])
					OF <= 1;
				else
					OF <= 0;
			end
			5'b10000://DEC
			begin
				{CF,aluZ} <= aluA + 8'b11111111;
				if(aluA[7] & !aluZ[7])
					OF <= 1;
				else
					OF <= 0;
			end
			5'b00000://NOPE
				aluZ <= 8'd0;
			5'b10001://aluB to AluZ
			begin
				aluZ <= aluB;
			end
		endcase
	end
endmodule

module DM (clk, we, addr, din, dout);
	input 		 clk;
	input  		 we;
	input  [7:0] addr;
	input  [7:0] din;	
	output [7:0] dout;
	
	reg [7:0] dm [0:255];

	always @(posedge clk)
		if (we)
			dm[addr] = din;
	
	assign dout = dm[addr];
endmodule

module IM (addr, dout);
	input  [7:0] addr;
	output [15:0] dout;
	reg [15:0] im [0:255];
	initial begin
		im[0] <= 16'b1100001100001010;
		im[1] <= 16'b0000001111010000;
		im[2] <= 16'b0000000001001010;
		im[3] <= 16'b0000010100011010;
		im[4] <= 16'b1001000000000000;
		im[5] <= 16'b1101000101100100;
		im[6] <= 16'b1100110001100100;
	end
	assign dout = im[addr];
endmodule

module RegBank(clk, R1, R2, WB, RegWrite, regA, regB);
	input		 clk;
	input  [2:0] R1;
	input  [2:0] R2;
	input  		 RegWrite;
	input  [7:0] WB;
	output [7:0] regA;
	output [7:0] regB;
	
	integer i;
	
	reg [0:7] regBank [7:0];

	initial
		for (i=0;i<8;i=i+1) 
			regBank[i] = 8'd0;
	
	always @(posedge clk)
		if(RegWrite)
			regBank[R1] <= WB;
	
	assign regA = regBank[R1];
	assign regB = regBank[R2];
endmodule

module core(clk, rst,out);
	input clk , rst;
	output out;
	
	


	//Fetch
	reg	 [7:0]  PC;
	reg [7:0] iaddr;
	reg [7:0]  nPC;
	wire [15:0] IR;
	
	//Decode
	wire [2:0]  R1;
	wire [2:0]  R1A;
	wire [2:0]  R1B;
	wire [2:0]  R2;
	wire [2:0]	imm3;
	wire [7:0]	imm8;
	wire [4:0]	func;
	wire [7:0]	addr;
	wire [3:0]	op;
	wire		MSB;
	reg 		jump;
	reg			aluB_Src;//IR[9]
	reg 		RegWrite;
	reg	 [4:0]	alu_func;

	
	assign R1A 	 = IR[5:3];
	assign R1B 	 = IR[10:8];
	assign R2 	 = IR[2:0];
	assign imm3	 = IR[2:0];
	assign func  = IR[10:6];
	assign addr  = IR[7:0];
	assign imm8  = MSB ? IR[7:0] : (imm3[2] ? {5'b11111,imm3[2:0]} : {5'b00000,imm3[2:0]});
	assign op	 = IR[14:11];
	assign MSB	 = IR[15];
	assign R1 	 = MSB ? R1B : R1A;
	
	
	
	
	//Execute
	wire [7:0]  regA;
 	wire [7:0]  regB;
	wire [7:0]	aluA;
	wire [7:0]	aluB;
	wire [7:0]	aluZ;
	reg		ZF;
	reg		SF;
	reg		CF;
	reg		OF;
	
	wire	zf;
	wire	sf;
	wire	cf;
	wire	of;
	
	
	assign aluA = regA;
	assign aluB = aluB_Src ? imm8 : regB;
	
	//Memory
	reg		memWrite;
	reg		memRead;
	wire [7:0]	dmemOut;
	
	
	//WB
	reg 	   memToReg;
	wire [7:0] WB;
	assign WB = memToReg ? dmemOut : aluZ;
	assign out = WB;//صرفا جهت سنتز کردن در زایلینکس

	
	
	//Control path
	always @(*) 
	begin
		if(MSB)
		begin
			case(op)
				4'b0000://JE
				begin
					jump	 <= ZF;
					memRead  <= 0;
					memWrite <= 0;
					RegWrite <= 0;
					memToReg <= 0;
					aluB_Src <= 0;
					
				end
				4'b0001://JB
				begin
					jump	 <= CF;
					memRead  <= 0;
					memWrite <= 0;
					RegWrite <= 0;
					memToReg <= 0;
				end
				4'b0010://JA
				begin
					jump 	 <= ~(CF | ZF) ? 1 : 0;
					memRead  <= 0;
					memWrite <= 0;
					RegWrite <= 0;
					memToReg <= 0;
				end
				4'b0011://JL
				begin
					jump	 <= (SF != OF) ? 1 : 0;
					memRead  <= 0;
					memWrite <= 0;
					RegWrite <= 0;
					memToReg <= 0;
				end
				4'b0100://JG
				begin
					jump	 <= (SF == OF & ~ZF) ? 1 : 0;
					memRead  <= 0;
					memWrite <= 0;
					RegWrite <= 0;
					memToReg <= 0;
				end
				4'b0101://JMP
				begin
					jump	 <= 1;
					memRead  <= 0;
					memWrite <= 0;
					RegWrite <= 0;
					memToReg <= 0;
				end
				4'b1000://LI
				begin
					jump	 <= 0;
					memRead  <= 0;
					memWrite <= 0;
					RegWrite <= 1;
					memToReg <= 0;
					aluB_Src <= 1;
					alu_func <= 5'b10001;
				end
				4'b1001://LM
				begin
					jump	 <= 0;
					memRead  <= 1;
					memWrite <= 0;
					RegWrite <= 1;
					memToReg <= 1;
				end
				4'b1010://SM
				begin
					jump	 <= 0;
					memRead  <= 0;
					memWrite <= 1;
					RegWrite <= 0;
				end
			endcase
		end
		else begin
			if(op == 4'b0000)
				begin
				aluB_Src <= IR[9];
				memToReg <= 0;
				if(func == 5'b10100 || func == 5'b00000)//CMP
					RegWrite <= 0;
				else
					RegWrite <= 1;
				
				memRead	 <=  0;
				memWrite <=  0;
				jump	 <= 0;
				case(func)
					5'b00001://ADD
					begin
						alu_func <= 5'b00001;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= of ? cf : aluZ[7];
						CF <= cf;
						OF <= of;
					end
					5'b00010://AND
					begin
						alu_func <= 5'b00010;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= aluZ[7];
						CF <= 0;
						OF <= 0;
					end
					5'b00011://SUB
					begin
						alu_func <= 5'b00011;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= of ? cf : aluZ[7];
						CF <= cf;
						OF <= of;
					end
					5'b00100://OR
					begin
						alu_func <= 5'b00100;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= aluZ[7];
						CF <= 0;
						OF <= 0;
					end
					5'b00101://XOR
					begin
						alu_func <= 5'b00101;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= aluZ[7];
						CF <= 0;
						OF <= 0;
					end
					5'b00110://MOV
						alu_func <= 5'b00110;
					//5'b00111://XCHG
					//به دلیل اینکه این یک شبه دستور است
					//انجام میدهیم MOV آن را در 3 سیکل توسط دستور
					//برای اینکار در کامپیوتر کاری نمیکنیم و
					//میگذاریم IM کد اسمبلی مربوطه را تولید کرده و در 
					//این کار با کمک یک رجیستر از بانک رجیسترها اتفاق میافتد
					5'b01000://NOT
						alu_func <= 5'b01000;
					5'b01001://SAR
					begin
						alu_func <= 5'b01001;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= aluZ[7];
						CF <= cf;
						OF <= 0;
					end
					5'b01010://SLR
					begin
						alu_func <= 5'b01010;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= (aluA[7] != aluZ[7]) ? cf : aluZ[7];
						CF <= cf;
						OF <= (aluA[7] != aluZ[7]) ? 1 : 0;
					end
					5'b01011://SAL
					begin
						alu_func <= 5'b01011;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= aluZ[7];
						CF <= cf;
						OF <= 0;
					end
					5'b01100://SLL
					begin
						alu_func <= 5'b01100;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= (aluA[7] != aluZ[7]) ? cf : aluZ[7];
						CF <= cf;
						OF <= (aluA[7] != aluZ[7]) ? 1 : 0;
					end
					5'b01101://ROL
					begin
						alu_func <= 5'b01101;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= (aluA[7] != aluZ[7]) ? cf : aluZ[7];
						CF <= cf;
						OF <= (aluA[7] != aluZ[7]) ? 1 : 0;
					end
					5'b01110://ROR
					begin
						alu_func <= 5'b01110;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= (aluA[7] != aluZ[7]) ? cf : aluZ[7];
						CF <= cf;
						OF <= (aluA[7] != aluZ[7]) ? 1 : 0;
					end
					5'b01111://INC
					begin
						alu_func <= 5'b01111;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= of ? cf : aluZ[7];
						CF <= cf;
						OF <= of;
					end
					5'b10000://DEC
					begin
						alu_func <= 5'b10000;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= of ? cf : aluZ[7];
						CF <= cf;
						OF <= of;
					end
					5'b10100://CMP
					begin
						alu_func <= 5'b00011;
						// if(aluZ == 0)
						// 	ZF <= 1;
						// else
						// 	ZF <= 0;
						ZF <= (aluZ == 0) ? 1 : 0;
						SF <= of ? cf : aluZ[7];
						CF <= cf;
						OF <= of;
					end
					5'b00000://NOPE
						alu_func <= 5'b00000;
						
				endcase
			end
		end
	end
		
	always @(posedge clk)
	begin
		if(rst)begin
			PC <= 0;
			iaddr <= 0;
		end
		else begin
			if(jump)begin
				iaddr <= addr;
				PC <= addr+1;
			end
			else begin
				iaddr <= PC;
				PC <= PC + 1;
			end
		end

	end
	
	//Data path
	ALU alu(alu_func, aluA, aluB,aluZ, cf, of);
	RegBank rb(clk,R1, R2, WB, RegWrite, regA, regB);
	DM dm(clk,memWrite,addr,regA,dmemOut);
	IM im(iaddr,IR);
endmodule