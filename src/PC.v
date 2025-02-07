module PC(
    input clk,
    input reset,
    input [15:0] in,
    input load,
    output reg [15:0] out
);

    reg [2:0] counter; // 3-bit counter to count up to 5

    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            out <= 16'b0;
            counter <= 3'b0; // Reset the counter
        end else if (load) begin
            out <= in;
            counter <= 3'b0; // Reset the counter when load is triggered
        end else begin
            if (counter == 3'b111) begin 
                out <= out + 16'b10;
                counter <= 3'b0; // Reset the counter
            end else begin
                counter <= counter + 3'b1; // Increment the counter
            end
        end
    end

endmodule



module dataPath(
    input AluSrc, MemR, MemW, RegWr, RegDes, WrB, ExtOp, R0, R7, load, clk, reset, ByteOrWord, ST,
    input [1:0] AluOp, PcSrc, instType,
    input [15:0] instruction, data_out,
    output reg m, z, n, o, c,
    output reg [15:0] data_in, pc_out, WBDataOut,
    output reg [15:0] AluRes , BPC
);

    wire [2:0] Rs1, Rs2, Rd, regDestination;
    wire [15:0] pc_in, BusA, BusB, BusW, A, B, extImm, AluSrc2, AluResult, resBuff, jumpTarget, data, immExt, immExtBuff, npc, BranchPc, NBPC, dataWOut;
    wire [7:0] immS;
    wire [4:0] immI;
    wire [11:0] immJ; 
    wire cA, zA, oA, nA;

    instBuffer insBuff(.clk(clk), .inst(instruction), .instType(instType), .m(m), .immI(immI), .immJ(immJ), .immS(immS), .Rs1(Rs1), .Rs2(Rs2), .Rd(Rd));
    assign jumpTarget = {pc_out[15:12], immJ};
    PC Pc(.clk(clk), .reset(reset), .in(pc_in), .load(load), .out(pc_out));
    DFF AB(.clk(clk), .reset(reset), .data_in(BusA), .data_out(A));
    DFF BB(.clk(clk), .reset(reset), .data_in(BusB), .data_out(B));
    DFF AluBuff(.clk(clk), .reset(reset), .data_in(AluResult), .data_out(resBuff));
    DFF Data(.clk(clk), .reset(reset), .data_in(data_out), .data_out(data));
    DFF immIBuff(.clk(clk), .reset(reset), .data_in(immExt), .data_out(immExtBuff));
    DFF NPC (.clk(clk), .reset(reset), .data_in(pc_out), .data_out(npc));
    DFF SBPC (.clk(clk), .reset(reset), .data_in(BranchPc), .data_out(NBPC));
    assign AluRes = resBuff;
    assign BranchPc = pc_out + immExt;
    assign BPC = BranchPc;

    Ext extender(.immI(immI), .immS(immS), .extOp(ExtOp), .instType(instType), .immO(immExt));

    regFile regfile(.clk(clk), .R0(R0), .R7(R7), .BusWE(RegWr), .addr1(Rs1), .addr2(regDestination), .addr3(Rd), .in(BusW), .out1(BusA), .out2(BusB));

    ALU executionEngin(.AluOp(AluOp), .a(A), .b(AluSrc2), .result(AluResult), .carry(cA), .zero(zA), .overflow(oA), .negative(nA));

    always @(posedge clk) begin
        c <= cA;
        z <= zA;
        o <= oA;
        n <= nA;	   
    end

    mux4x1 pcSrc(.a(pc_out), .b(jumpTarget), .c(BranchPc), .d(BusA), .s(PcSrc), .out(pc_in));
    mux2x1 #(3) regDest(.a(Rs2), .b(Rd), .s(RegDes), .out(regDestination));
    mux2x1 BusWrite(.a(data_out), .b(pc_out + 4), .s(R7), .out(BusW));
    mux2x1 AluSourc(.a(B), .b(immExt), .s(AluSrc), .out(AluSrc2));
    mux2x1 writeBack(.a(resBuff), .b(dataWOut), .s(WrB), .out(WBDataOut));
    mux2x1 WordOrByte(.a(data), .b({{(8){1'b0}}, data[7:0]}), .s(ByteOrWord), .out(dataWOut));
    mux2x1 StoreType(.a(immExt), .b(B), .s(ST), .out(data_in));
endmodule 




module DFF(
	input clk, reset ,
	input [15:0]data_in,
	output reg [15:0]data_out);
	
	always @(posedge clk,negedge reset)
		begin
		if(!reset)
			data_out = 16'b0;
		else						
			data_out <= data_in;
			
			
			
			end
	
	
	
	
	
	endmodule
	
module mux2x1 #(parameter n = 16)(
	input [n-1:0] a, b,     
    input s,  
    output [n-1:0] out    
);


	assign out = s ? b : a;

endmodule 	
	
	
 module mux4x1 (
	input [15:0] a, b, c, d,  	 
	input [1:0] s,	 
	
	output reg [15:0] out
);
	
	
	always @* begin 
	
		case (s) 
			2'b00 : out = a;
			2'b01 : out = b;
			2'b10 : out = c;
			2'b11 : out = d;
		endcase
end


endmodule 


module dataPath_tb;

  reg AluSrc, MemR, MemW, RegWr, RegDes, WrB, ExtOp, R0, R7, load, clk, reset,BW ,ST;
  reg [1:0] AluOp, PcSrc, instType;
  reg [15:0] instruction, data_out;
  wire [3:0] opcode;
  wire m, z, n, o, c;
  wire [15:0] data_in, pc_out, WBData, AluRes;

  // Instantiate the dataPath module
  dataPath uut (
    .AluSrc(AluSrc), .MemR(MemR), .MemW(MemW), .RegWr(RegWr), .RegDes(RegDes),.ST(ST),
    .WrB(WrB), .ExtOp(ExtOp), .R0(R0), .R7(R7), .load(load), .clk(clk), .reset(reset),.ByteOrWord(BW),
    .AluOp(AluOp), .PcSrc(PcSrc), .instType(instType),
    .instruction(instruction), .data_out(data_out),
     .m(m), .z(z), .n(n), .o(o), .c(c),
    .data_in(data_in), .pc_out(pc_out), .WBDataOut(WBData), .AluRes(AluRes)
  );  
  

  // Clock generation
  always begin
    #5 clk = ~clk;  // Clock period of 10 units
  end

  initial begin
 	 clk = 0;
	 #5 reset = 0;
	 
	 #5 reset = 1; 
	 
	 
	 
    // Test case 3: Example J-Type instruction
    #20 instruction = 16'b1000001001101110;  // Example instruction
        AluSrc = 0;
        RegWr = 0;
        RegDes = 1;
        WrB = 0;
        ExtOp = 1;
        R0 = 0;
        R7 = 0;
        load = 1;
        AluOp = 2'b10;
        PcSrc = 2'b10;
        instType = 2'b10;

    // Additional test cases as needed...

    // End simulation
    #100 $finish;
  end

  initial begin
    $monitor("At time %t, pc_out = %h, AluRes = %d, WBData = %h, m = %b, z = %b, n = %b, o = %b, c = %b",
             $time, pc_out, AluRes, WBData, m, z, n, o, c);
  end

endmodule


