module top_tb;

  // Inputs
  reg clk;
  reg reset;
  reg [15:0] instruction;

  // Outputs
  wire [15:0] pc_state;
  wire [15:0] result_state,ins , data_in,data_out , WB,BPC;
  wire [1:0] instruction_type; 
  wire [2:0]state;
  wire memR,memW;

  // Instantiate the top module
  top uut (
    .clk(clk),
    .reset(reset),
    .pc_state(pc_state), 
	
    .result_state(result_state),
    .instruction_type(instruction_type),
	.ins(ins),
	.state(state),
	.data_inR(data_in),
	.data_outR(data_out),
	.memR(memR),
	.memW(memW),	
	.WB(WB),
	.BPC(BPC)
	
  );

  // Clock generation
  always #5 clk = ~clk; // 10ns clock period

  initial begin
    // Initialize Inputs
    clk = 0;
    reset = 0; 
	

    #5;
    reset = 1;

    // Wait for some time to observe the behavior
    #1000;

    // Stop the simulation
    $stop;
  end

  // Monitor outputs
  initial begin
    $monitor("Time = %0d, pc_state = %d, result_state = %d,state  = %d ,  instruction_type = %b ,ins= %b , dataIn = %d , dataOut = %d , memR = %h , memW = %h , WB = %d , BPC = %d",
	$time, pc_state, result_state,state , instruction_type,ins , data_in , data_out , memR,memW , WB ,BPC);
  end

endmodule



module Core_tb;

    // Inputs
    reg reset;
    reg clk;
    reg [3:0] opcode;
    reg [15:0] data_out;
    reg [15:0] instruction;

    // Outputs
    wire [15:0] data_in;
    wire [15:0] AluRes;
    wire [15:0] pc ,DO;
    wire MemRO;
    wire MemWO;
    wire [1:0] instType; 
	wire [2:0]state;

    // Instantiate the Unit Under Test (UUT)
    Core uut (
        .reset(reset), 
        .clk(clk),  
        .data_out(data_out), 
        .instruction(instruction), 
        .data_in(data_in), 
        .AluRes(AluRes), 
        .pc(pc), 
        .MemRO(MemRO), 
        .MemWO(MemWO), 
        .instType(instType),
		.state(state)	, 
		.DO(DO)
    );

    // Generate clock signal
    always begin
        #5 clk = ~clk; // Toggle clock every 5 time units
    end

    initial begin
        // Monitor the signals
		  
	        $monitor("Time: %0d, reset: %b, opcode: %b, data_out: %h, instruction: %b, data_in: %h, AluRes: %d, pc: %h, MemRO: %b, MemWO: %b, instType: %b, state 1:%h",
            $time, reset, instruction[15:12], data_out, instruction, data_in, AluRes, pc, MemRO, MemWO, instType, state);	

        // Initialize Inputs
        clk = 0;
        reset = 1;
        data_out = 16'b1;
    

        // Wait for global reset to finish
        #10;
        reset = 0;
		#50 
		reset = 1;
        // Test sequence
        // Test Fetch stage
		instruction = 16'b1001001001111000;
        #100;

      



        // End of test
        $stop;
end	  

//always @(AluRes )
//	begin
//	        $display("Time: %0d, reset: %b, opcode: %b, data_out: %h, instruction: %b, data_in: %h, AluRes: %d, pc: %h, MemRO: %b, MemWO: %b, instType: %b, state 1:%h",
//            $time, reset, instruction[15:12], data_out, instruction, data_in, AluRes, pc, MemRO, MemWO, instType, state);	
//		
//	end
	

endmodule
