# Exp-6a.-Finite-State-Machine-for-Sequence-Detector-1011
# Aim 
To design and simulate a Finite-State-Machine-for-Sequence-Detector-1011 using Verilog HDL, and verify its functionality through a testbench in the Vivado 2023.1 environment. 

# Apparatus Required 

Vivado 2023.1 

# Procedure

Launch Vivado 2023.1 Open Vivado and create a new project.
Design the Verilog Code Write the Verilog code for the RAM,ROM,FIFO
Create the Testbench Write a testbench to simulate the memory behavior. The testbench should apply various and monitor the corresponding output.
Create the Verilog Files Create both the design module and the testbench in the Vivado project.
Run Simulation Run the behavioral simulation to verify the output.
Observe the Waveforms Analyze the output waveforms in the simulation window, and verify that the correct read and write operation.
Save and Document Results Capture screenshots of the waveform and save the simulation logs. These will be included in the lab report.
# Code
# Mealy 1011
# Code

```
module mealysequence(clk,reset,in,out);
    input clk;
    input reset;
    input in;
    output reg out;


    parameter S0 = 3'b000,
                      S1 = 3'b001,
                      S2 = 3'b010,
                      S3 = 3'b011,
                      S4 = 3'b100;

    reg [2:0] current_state,next_state;
always @(posedge clk or posedge reset) 
begin
        if (reset)
            current_state <= S0;
        else
            current_state <= next_state;
    end

  always @(*) 
    begin
        out = 0;          
          case (current_state)
            S0: 
              begin
                if (in) 
                       next_state = S1;
                else    
                        next_state = S0;
            end
S1: begin
                if (in) 
                     next_state = S2; // '11'
                else    
                     next_state = S0;
            end

  S2: begin
                if (in) 
                     next_state = S2; // still '11'
                else    
                     next_state = S3; // '110'
            end

   S3: begin
                if (in) 
                      next_state = S4; // '1101'
                else    
                        next_state = S0;
            end
S4: begin
                if (in) 
                    begin
                    out = 1;            // Detected '11011'
                    next_state = S0;    // No overlapping
                end
                else
                    next_state = S3;    
       end

default:  next_state = S0;
        endcase
    end
endmodule
`timescale 1ns/1ps
```
# Testbench
```
module tb_mealy;

    reg clk, reset, in;
    wire out;

    mealysequence uut (clk,reset,in,out);
   
    initial  clk = 0;
    always #5 clk = ~clk;   // 10 ns clock period

initial 
     begin
        reset = 1; in = 0;
        #12 reset = 0;

        #10 in=1;   // S0 → S1
        #10 in=1;   // S1 → S2
        #10 in=0;   // S2 → S3
        #10 in=1;   // S3 → S4
        #10 in=1;   // S4 → S0 → out=1 (detected)

        // Add random inputs
        #10 in=0;
        #10 in=1;
        #10 in=1;
        #10 in=0;
        #10 in=1;
        #10 in=1;   // another detection here

        #50 $finish;
   end
endmodule

```

# Output

<img width="1920" height="1020" alt="image" src="https://github.com/user-attachments/assets/108dbd81-bc83-4c9a-89d7-b3493218bf8c" />


# Moore 1011

# Code


```
`timescale 1ns/1ps
module moree(clk, rst, in, out);
input clk;
input rst;
input in;
output reg out;
parameter S0=3'b000,
          S1=3'b001,
          S2=3'b010,
          S3=3'b011,
          S4=3'b100;
reg [2:0] cs, ns;
always @(posedge clk or posedge rst)
begin 
if(rst)
cs=S0;
else
cs=ns;
end 
always @ (*) begin 
case(cs)
S0:if(in)
ns=S1;
else ns=S0;
S1:
begin
if (in)
ns=S1;
else ns=S2;
end
S2:
begin 
if(in)
ns=S3;
else 
ns=S0;
end 
S3: 
begin 
if(in)
ns=S4;
else
ns=S2;
end
S4:
begin
if(in)
ns=S1;
else
ns=S0;
end
default: ns=S0;
endcase
end
always @(*)
begin
case(cs)
S4: out = 1'b1;
default: out=1'b0;
endcase
end
endmodule
```
# Testbench

```
module tb_moree;
reg clk, rst, in;
wire out;
moree uut(clk, rst, in ,out);
initial begin
clk=0;
forever #5 clk=~clk;
end
initial
begin rst=1;
in =0;
#10 rst=0;
in=1;#10
in=0;#10
in=1;#10
in=1;#10
#10 rst=0;
#20

in=1;#10
in=0;#10
in=1;#10
in=0;#10
#10 $finish;
end 
endmodule
```
# Output

<img width="1920" height="1020" alt="image" src="https://github.com/user-attachments/assets/a02727ca-b69f-4eff-8359-2cef7dcb5075" />


Conclusion
The Mealy and Moore state machine for sequence 1011 was designed and successfully simulated using Verilog HDL. The testbench verified both the write and read functionalities by simulating the sequence operations and observing the output waveforms.
