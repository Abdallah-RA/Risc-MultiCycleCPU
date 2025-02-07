module control_unit (
    input [3:0] opcode,
    input m, clk, reset,
    output reg AluSrc, MemR, MemW, RegWr, RegDes, WrB, ExtOp, R0, R7,ByteOrWord,ST,
    output reg [1:0] AluOp, PcSrc,									 
    output reg [1:0] instType,
    output reg [2:0] state
);

    // State definitions
    parameter Fetch = 0, Decode = 1, Execute = 2, MemAccess = 3, WriteBack = 4;

    // Opcode definitions
    parameter AND = 4'b0000, ADD = 4'b0001, SUB = 4'b0010, ADDI = 4'b0011,
              ANDI = 4'b0100, LW = 4'b0101, LBU_LBS = 4'b0110, SW = 4'b0111,
              BGT = 4'b1000, BLT = 4'b1001, BEQ = 4'b1010, BNE = 4'b1011,
              JMP = 4'b1100, CALL = 4'b1101, RET = 4'b1110, SV = 4'b1111;

    // Sequential state transition
    always @(posedge clk or negedge reset) begin
        if (!reset)
            state <= Fetch;
        else begin
            case (state)
                Fetch: state = Decode;	
				
                Decode: begin
                    case (opcode)
                        AND, ADD, SUB,
                        LW, LBU_LBS, SW,
                        BEQ, BNE, BGT, BLT, RET, SV: state = Execute;
                        JMP, CALL: state = Fetch;
                        ADDI, ANDI: state = Execute;
                    endcase
                end
                Execute:begin
					case(opcode)
						AND,ADD,SUB,ADDI,ANDI :state = WriteBack;
						BEQ, BNE, BGT, BLT, RET, SV : state = Fetch;
						LW, LBU_LBS, SW : state = MemAccess;
						endcase
					
					
					
					end
                MemAccess: state = WriteBack;
                WriteBack: state = Fetch;
            endcase
        end
    end

    // Output and control signals logic
    always @(posedge clk) begin
        // Default values
        AluSrc = 0;
        AluOp = 2'b00;
        RegDes = 0;
        RegWr = 0;
        MemR = 0;
        MemW = 0;
        PcSrc = 2'b00;
        WrB = 0;
        ExtOp = 0;
        R0 = 0;
        R7 = 0;
		ByteOrWord = 0;	 
		ST = 0;
		case (opcode)
		  AND, ADD, SUB:	
		  instType = 2'b00;
		 ADDI, ANDI, LW, SW, BGT, BLT, BEQ, BNE:
		 instType = 2'b01;
		 JMP, CALL, RET:
		 instType = 2'b10;
		 SV:
		 instType = 2'b11;

		 endcase
        case (state)
            Fetch: begin
                 case (opcode)
                    AND, ADD, SUB , ADDI, ANDI, LW, SW ,LBU_LBS , SV: begin
                     PcSrc = 2'b00; 
                    end
    
					BGT, BLT, BEQ, BNE: begin
					
						PcSrc = 2'b10;
					end
                    JMP, CALL, RET: begin   
                        PcSrc = (opcode == RET) ? 2'b11 : 2'b01; 
						R7 = (opcode == CALL);
                    end
                endcase
				
            end
			
            Decode: begin
                case (opcode)
                    AND, ADD, SUB: begin
                       
                        RegDes = 0; // Destination register from instruction
                    end
                    ADDI, ANDI, LW, SW: begin
                   
            
						ExtOp =1;
						ST = 1;
                    end	 
					BGT, BLT, BEQ, BNE: begin
						instType = 2'b01; 	
						RegDes = 1;
                        R0 = ((opcode == BGT || opcode == BLT || opcode == BEQ || opcode == BNE) && m) ? 1 : 0; 
						ExtOp =1; 
						PcSrc = 2'b10;
						
						
					end
					
                    LBU_LBS: begin
                        
                        AluSrc = 1; // Use immediate value for ALU second operand
                        ExtOp = m; // Use m for sign extension
							
                    end
                    JMP, CALL, RET: begin
                        
                        PcSrc = (opcode == RET) ? 2'b11 : 2'b01; 
						R7 = (opcode == CALL);
                    end
                    SV: begin
                       
                        MemW = 1; // Memory write 
						ST = 0;
                    end
                endcase
            end

            Execute: begin
                case (opcode)
                    AND: begin
                        AluOp = 2'b00;
                        RegWr = 1;
                      
                    end
                    ADD: begin
                        AluOp = 2'b01;
                        RegWr = 1;
                      
                    end
                    SUB: begin
                        AluOp = 2'b10;
                        RegWr = 1;
                       
                    end
                    ADDI: begin
                        AluSrc = 1;
                        AluOp = 2'b01;
                        RegWr = 1;
                        
                    end
                    ANDI: begin
                        AluSrc = 1;
                        RegWr = 1;
						AluOp = 2'b00;
                        
                    end
                    LW: begin
                        AluSrc = 1;
                        AluOp = 2'b01;
                        instType = 2'b01;
                    end
                    LBU_LBS: begin
                        AluSrc = 1;
                        AluOp = 2'b01;
                        instType = 2'b01; 
						ExtOp = m; 
						
                    end
                    SW: begin
                        AluSrc = 1;
                        AluOp = 2'b01;
                      
						ST = 1;
                    end
                    BGT, BLT, BEQ, BNE: begin  
						AluSrc = 0;
                        AluOp = 2'b10;
                   	    PcSrc = 2'b10;
					
                       
						
						
						
                    end
                    JMP: begin
                        PcSrc = 2'b01;
                       
                    end
                    CALL: begin
                        PcSrc = 2'b01;
                        R7 = 1;
                       
                    end
                    RET: begin
                        PcSrc = 2'b11;
                        R7 = 1;
                      
                    end
                    SV: begin
                        AluSrc = 1;
                        AluOp = 2'b11;
                        MemW = 1;
                      
						ST = 0;
                    end
                endcase
            end

            MemAccess: begin
                case (opcode)
                    LW: begin
                        MemR = 1;
                        WrB = 1;
						ByteOrWord = 0;
                    end
                    LBU_LBS: begin
                        MemR = 1;
                        WrB = 1;
                        ExtOp = m; 
					    ByteOrWord = 1;
						
						
                    end
                    SW: begin
                        MemW = 1; 
						ST = 0;
						
					SV : begin
						MemW = 1;
						ST = 0;
						
						
						end
                    end
                endcase
            end

            WriteBack: begin
                case (opcode)
                    LW: begin
                        RegWr = 1;
                        WrB = 1;
						
                    end	 
					LBU_LBS: begin
                        RegWr = 1;
                        WrB = 1;
						
                    end
                    ADD, AND, SUB, ADDI, ANDI: begin
                        RegWr = 1;
						
                    end
                endcase
            end
        endcase
    end
endmodule




module control_unit_tb;

    // Inputs
    reg [3:0] opcode;
    reg m;
    reg clk;
    reg reset;

    // Outputs
    wire AluSrc;
    wire MemR;
    wire MemW;
    wire RegWr;
    wire RegDes;
    wire WrB;
    wire ExtOp;
    wire R0;
    wire R7 ,BW ,ST;
    wire [1:0] AluOp;
    wire [1:0] PcSrc;
    wire [1:0] instType; 
	wire [2:0]state;

    // Instantiate the Unit Under Test (UUT)
    control_unit uut (
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
        .AluOp(AluOp),
        .PcSrc(PcSrc), 
		.ST(ST),
		.ByteOrWord(BW),
        .instType(instType),
		.state(state)
    );

    // Generate clock signal
    always begin
        #5 clk = ~clk; // Toggle clock every 5 time units
    end

    initial begin
        // Monitor the signals
        $monitor("Time: %0d, reset: %b, opcode: %b, m: %b, AluSrc: %b, MemR: %b, MemW: %b, RegWr: %b, RegDes: %b, WrB: %b, ExtOp: %b, R0: %b, R7: %b, AluOp: %b, PcSrc: %b, instType: %b , %h",
            $time, reset, opcode, m, AluSrc, MemR, MemW, RegWr, RegDes, WrB, ExtOp, R0, R7, AluOp, PcSrc, instType,state);

        // Initialize Inputs
        clk = 0;
        reset = 0;
        opcode = 4'b0000;
        m = 1;

        // Wait for global reset to finish
        #10;
        reset = 1;

        // Test sequence
        // Fetch
        opcode = 4'b0110; // AND
        #100  $stop;;
		


        // End of test
       
    end

endmodule
