module ALU(
    input [1:0] AluOp,
    input [15:0] a,
    input [15:0] b,
    output reg [15:0] result,
    output reg zero,carry,overflow,negative
);
    
    always @(*) begin
     
        zero = 0;
        carry = 0;
        overflow = 0;
        negative = 0;

        case (AluOp)
            2'b00: begin
                result = a & b;
            end
            
            2'b01: begin
                {carry, result} = a + b;
                overflow = (a[15] & b[15] & ~result[15]) | (~a[15] & ~b[15] & result[15]);
            end
            
            2'b10: begin
                result = a - b;
                overflow = (a[15] & ~b[15] & ~result[15]) | (~a[15] & b[15] & result[15]);
            end	
		2'b11: 
			result = a;
        endcase
        
      
        if (result == 0) begin
            zero = 1;
        end
        
        if (result[15] == 1) begin
            negative = 1;
        end
    end
endmodule
