module tb_riscv_single_cycle_processor;

  reg clk;
  reg reset;

  top_module uut (
    .clk(clk),
    .reset(reset)
  );

  initial begin
    clk = 0;
    forever #5 clk = ~clk;  
  end

  initial begin
    reset = 1;
    #15;
    reset = 0;

    #200;

    $finish;
  end
  initial begin
    $monitor("Time=%0t | PC=%h | Instruction=%h | ALU Result=%h | Mem Data=%h",
              $time, uut.pc_out, uut.instruction, uut.alu_result, uut.mem_data_out);
  end

  initial begin
    $dumpfile("riscv_single_cycle_processor.vcd");  
    $dumpvars(0, tb_riscv_single_cycle_processor);
  end

endmodule
