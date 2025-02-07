 module regFile(
	 input clk,R0,R7,BusWE,
	 input [2:0] addr1,addr2,addr3, 
	 input [15:0] in,
	 output reg [15:0]  out1,out2
	 
	 );
	 
  reg [15:0] regfile [0:7];
  
  initial begin
	
       integer i;
        for (i = 0; i < 8; i = i + 1) begin
            regfile[i] = 16'b0;
        end
		regfile[2] = 16'd40;
       regfile[3] = 16'd10;
	   regfile[1] = 16'd40;
	   regfile[5] = 16'd46;
    end
  
	always @(posedge clk)
		begin 
		if(BusWE && addr3 != 0 && addr3 != 7)
	    regfile[addr3] <= in;
		
		if(R7 ==1) begin
			regfile[7] <= in;
			out1 <= regfile[7];
			end
		else if (R0 == 1)
		out1 <= regfile[0];
		else
			out1 <= regfile[addr1];
			
			
        out2 <= regfile[addr2];
	 
	   end
	   
	 
	 
	 
 endmodule: regFile 
 
 
module dataMemo(
    input clk,
    input we,rd,                
    input [15:0] addr,       
    input [15:0] writeData,
    output reg [15:0] readData 
);
	  
    
reg [7:0] memory [0:65535];	


initial
        begin
            memory[0] = 8'd50;
            memory[1] = 8'd200;
            memory[2] = 8'd150;
            memory[3] = 8'd55;
            memory[26] = 8'd44;
			memory[27] = 8'd44;
            memory[58] = 8'd91;	
			memory[59] = 8'd91;
            memory[60] = 8'd88;
			memory[61] = 8'd88;

        end

    always @(posedge clk) begin
        if (we) begin
           
            memory[addr] <= writeData[7:0];
            memory[addr + 1] <= writeData[15:8];
        end
       
    end	  
	
	always @(rd)
		
        readData <= {memory[addr + 1], memory[addr]}; 

endmodule: dataMemo
 
 
 
 
 
module instMemo( 
	
    input [15:0] addr,     
    output reg [15:0] instruction,
	output reg [3:0]opCode
);
	 
    
    reg [7:0] memory [0:65535];
	
	initial 
		begin
		integer i;
		
		memory[0] = 8'b10011000; // First byte of the 1st instruction
         memory[1] = 8'b00000010; // Second byte of the 1st instruction (AND)

        memory[2] = 8'b10011000; // First byte of the 2st instruction
        memory[3] = 8'b00010010; // Second byte of the 2st instruction (ADD)

        memory[4] = 8'b10011000; // First byte of the 3st instruction
        memory[5] = 8'b00100010; // Second byte of the 3st instruction (SUB)

        memory[6] = 8'b01010110; // First byte of the 4st instruction
        memory[7] = 8'b00110100; // Second byte of the 4st instruction     (ADDI)

        memory[8] = 8'b01100000; // First byte of the 5st instruction
        memory[9] = 8'b01001101; // Second byte of the 5st instruction     (ANDI)

        memory[10] = 8'b01110110; // First byte of the 6st instruction
        memory[11] = 8'b01010100; // Second byte of the 6st instruction     (LW)

        memory[12] = 8'b01110110; // First byte of the 7st instruction
        memory[13] = 8'b01100101; // Second byte of the 7st instruction     (LBs)

        memory[14] = 8'b01110110; // First byte of the 8st instruction
        memory[15] = 8'b01101101; // Second byte of the 8st instruction     (LBu)

        memory[16] = 8'b01111000; // First byte of the 9st instruction
        memory[17] = 8'b01110101; // Second byte of the 9st instruction     (SW) 
		
		memory[18] = 8'b01101110; // First byte of the 10st instruction
        memory[19] = 8'b11010001; // Second byte of the 10st instruction     (BGT)

        memory[20] = 8'b01101110; // First byte of the 11st instruction
        memory[21] = 8'b11101011; // Second byte of the 11st instruction     (BGTZ)
		for(i = 22; i <= 255; i = i+1)
			memory[i] = 8'b0;
			
			
		end
		
 
      always @(*) begin
    instruction <= {memory[addr + 1], memory[addr]};
	opCode <= instruction[15:12];
 	 end

endmodule: instMemo




module instBuffer (
    input clk, 
    input [15:0] inst,
    input [1:0] instType, 
    output reg m,
    output reg [4:0] immI,
    output reg [11:0] immJ,
    output reg [7:0] immS,
    output reg [2:0] Rs1,
    output reg [2:0] Rs2,
    output reg [2:0] Rd
);
 
    always @(posedge clk) begin
        case(instType)
            2'b00: begin
                Rd = inst[11:9];
                Rs1 = inst[8:6];
                Rs2 = inst[5:3];
            end
            2'b01: begin
                m = inst[11];
                Rd = inst[10:8];
                Rs1 = inst[7:5];
                immI = inst[4:0];
            end
            2'b10: begin
                immJ = inst[11:0];
            end
            2'b11: begin
                Rs1 = inst[11:9];
                immS = inst[8:1];
            end
            default: begin
                m = 0;
                immI = 0;
                immJ = 0;
                immS = 0;
                Rs1 = 0;
                Rs2 = 0;
                Rd = 0;
            end
        endcase
    end
endmodule

