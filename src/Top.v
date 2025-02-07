module Core (
	input reset,clk,
	input [15:0]data_out,instruction,
	output [15:0] data_in,AluRes,pc ,DO,BPC,
	output MemRO,MemWO,
	output [1:0] instType,
	output [2:0]state
	
	); 
	 parameter BGT = 4'b1000, BLT = 4'b1001, BEQ = 4'b1010, BNE = 4'b1011;
	
	wire AluSrc,MemR,MemW,RegWr,RegDes,WrB, ExtOp, R0, R7,m,z,n,o,c,ByteOrWord ,ST,load;
	wire [1:0] AluOp;
	wire [3:0] opcode;
	reg [1:0] PcSrc;
 
	wire [15:0]WBData ; 
	wire [1:0] inType;
	
	assign m= instruction[11];
	assign opcode = instruction[15:12];
	
	
 dataPath dp( .AluSrc(AluSrc), .MemR(MemR), .MemW(MemW),.ST(ST),
 .RegWr(RegWr), .RegDes(RegDes), .WrB(WrB),
 .ExtOp(ExtOp), .R0(R0), .R7(R7),.load(load), .ByteOrWord(ByteOrWord)
 ,.clk(clk),.reset(reset),.AluOp(AluOp), .PcSrc(PcSrc),
 .instType(inType),.instruction(instruction), .data_out(data_out), 
  .m(m),.z(z),.n(n),.o(o),.c(c)
 ,.data_in(data_in),.pc_out(pc),.WBDataOut(WBData), .AluRes(AluRes), .BPC(BPC));
	 
	assign DO = WBData;
  control_unit CU (
        .opcode(opcode),
        .m(m),
        .clk(clk),
        .reset(reset),
        .AluSrc(AluSrc),
        .MemR(MemR),
        .MemW(MemW),
        .RegWr(RegWr),
        .RegDes(RegDes),
        .WrB(WrB),
        .ExtOp(ExtOp),
        .R0(R0),
        .R7(R7),
		.ST(ST),
		.ByteOrWord(ByteOrWord),
        .AluOp(AluOp),
        .PcSrc(PcSrc),
        .instType(inType),
		.state(state)
    );
 
 assign instType = inType;	
 
 assign MemRO = MemR;
 assign MemWO = MemW;	 

 
pc_control PCC(.opcode(opcode),.z(z),.n(n),.v(o),.state(state) ,.load(load));

	
endmodule 


module pc_control(input [3:0]opcode ,input z,n,v,input [2:0] state ,output  load);
	
	parameter BGT = 4'b1000, BLT = 4'b1001, BEQ = 4'b1010, BNE = 4'b1011 ,CALL = 4'b1101, RET = 4'b1110, JMP = 4'b1100;
	

	
	assign load=  ((opcode == BNE) && !z) || ((opcode == BEQ) && z) || ((opcode ==BLT) && (!z && !(n ^ v))) || ((opcode == BGT) && (n ^ v)  || (( opcode == CALL  || opcode == RET || opcode == JMP)&& state == 1) );
	

	
endmodule 





module top(
	input clk ,reset,
	output [15:0] pc_state,result_state,
	output [1:0] instruction_type ,
	output [15:0] ins,data_inR,data_outR ,WB,BPC,
	output [2:0]state, 
	output memR,memW
	
	);
	
	
	
	wire [15:0] instruction;
	wire [3:0]opcode;  
	wire  MemR, MemW;
	wire [15:0] data_in,result,data_out,pc_out , DO;
	
	reg [15:0]resAdd;
	
	 
	
	instMemo inM(.addr(pc_out),.instruction(instruction),.opCode(opcode));
	
	assign ins = instruction;
	
	 Core C1(.clk(clk),.reset(reset),.data_out(data_out),.instruction(instruction),.data_in(data_in),.AluRes(result),.pc(pc_out),.MemRO(MemR) ,.MemWO(MemW),.instType(instruction_type),.state(state),.DO(DO) , .BPC(BPC));
	 assign WB = DO;
	 assign pc_state = pc_out;
	 assign result_state = result; 
	 assign data_inR = data_in;
	 assign data_outR = data_out; 
	 assign memR = MemR;
	 assign memW = MemW;
	 
	 always @(*)
		  if(memR || memW)
		 resAdd = result;
	 
	
	
	
	dataMemo DMemo(.clk(clk) , .we(MemW),.rd(MemR), .addr(resAdd), .writeData(data_in),.readData(data_out));
	
	
	
endmodule


