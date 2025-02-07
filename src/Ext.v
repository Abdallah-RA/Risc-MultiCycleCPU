module Ext(
    input [4:0] immI,
    input [7:0] immS,
    input extOp, 
	input [1:0]instType,
    output reg [15:0] immO
);
    
always @(*) begin 
	
	if (extOp) begin
		case(instType)	 
		2'b01:	
		immO = {{(11){immI[4]}}, immI};
		2'b11:
		immO = {{(8){immS[7]}}, immS};
		endcase
        end else begin 
		case(instType)	
           2'b01:	
		   immO = {{(11){1'b0}}, immI};
		2'b11:
		immO = {{(8){1'b0}}, immS};	
		endcase
        end
    end
endmodule


