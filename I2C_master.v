`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.11.2025 23:29:59
// Design Name: 
// Module Name: i2c
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module i2c(
    input wire clk,
    input wire rst,
    input wire en_start,
    output reg scl,
    output reg tristate,
    output reg sda_out,
    input wire sda_in,
    input wire [6:0] slave_address_in,
    input wire read_write_in,
    input wire [7:0] register_address_in,
    input wire [7:0] data_in,
    output wire [7:0] data_out
);
    
    reg [3:0] current_state,next_state;
    wire sda;
    reg clock_count;
    assign sda = (tristate == 0) ? sda_out : 1'bz;
    
    // State Parameters for easy access
    parameter idle=4'b0000,
            start=4'b0001,
            addr=4'b0010,
            ack_ad=4'b0011,
            addreg=4'b0100,
            ack_addreg=4'b0101,
            data=4'b0110,
            data_ack=4'b0111,
            stop=4'b1000,
            rep_start=4'b1001;

    reg [5:0] bit_count;

    // Clock Counter (Controls SCL) 
    always @(posedge clk) begin
        if (rst) begin
            clock_count <= 0;
        end
        else begin
            if ((current_state == idle) || (current_state == stop) || (next_state == rep_start)) begin
                clock_count <= 0;
            end
            else begin
                if (clock_count == 1) begin
                    clock_count <= 0;
                end
                else begin
                    clock_count <= clock_count + 1;
                end
            end
        end
    end

    // Bit Counter
    always @(posedge clk) begin
        if (rst) begin
            bit_count <= 0;
        end
        else begin
            if ((current_state == idle) || (current_state == start) || (current_state == stop) || (current_state == rep_start)) begin
                bit_count <= 0;
            end
            else begin
                if (scl == 1'b1) begin
                bit_count <= bit_count + 1;
                end
            end
        end
    end

    // Internal Registers
    reg [7:0] slave_address_save;
    reg read_write_save;
    reg repeated_start_indication;
    reg [7:0] data_write, data_read;
    reg [7:0] register_address_save;
    reg ack; // 0 for ACK, 1 for NACK

    // Register Address Register logic
    always @(posedge clk) begin
        if (rst) register_address_save <= 0;
        else if (current_state == addreg && scl == 1'b1) 
            register_address_save <= {register_address_save[6:0],1'b0};
        else if (current_state != addreg)
            register_address_save <= register_address_in;
    end

    // Data Registers logic
    always @(posedge clk) begin
        if (rst) begin
            data_write <= 0;
            data_read <= 0;
        end
        else begin
            if (current_state == data) begin
                if (read_write_save == 0) begin // Write
                    if (scl == 1'b1) data_write <= {data_write[6:0],1'b0};
                end
                else if (read_write_save == 1) begin // Read
                    if (scl == 1'b0) data_read <= {data_read[6:0],sda_in};
                end
            end
            else begin
                data_write <= data_in;
            end
        end
    end

    assign data_out = data_read;

    // Slave Address Register logic
    always @(posedge clk) begin
        if (rst) begin
            slave_address_save <= 0;
            read_write_save <= 0;
        end
        else begin
            if (current_state == addr && scl == 1'b1) begin
                slave_address_save <= {slave_address_save[6:0],1'b0};
            end
            else if (current_state != addr) begin
                slave_address_save <= {slave_address_in,read_write_in};
            end
            read_write_save <= read_write_in;
        end
    end

    // Master ACK signal (0 for ACK, 1 for NACK) for read data
    always @(*) begin
        if (rst) ack = 1;
        else if (bit_count == 17) ack = 0;
        else ack = 1;
    end

    reg repeated_start_signal;
    
    // FSM Combinational Logic
    always @(*)
    begin
        // Default outputs
        scl = 1;
        tristate = 0;
        sda_out = 1;
        next_state = current_state;
        repeated_start_indication=repeated_start_signal;

        if(rst) begin
            next_state=idle;
            repeated_start_indication = 0;
        end
        else begin
            case(current_state)
                idle:
                begin
                    if(en_start) next_state=start;
                end
                start:
                begin
                    sda_out=0;
                    scl=~clock_count;
                    repeated_start_indication = 0;
                    if (clock_count == 1) next_state=addr;
                end
                addr: begin
                    scl = ~clock_count;
                    sda_out = slave_address_save[7];
                    if ((bit_count == 7) && (scl == 1'b1)) begin
                        next_state = ack_ad;
                    end
                end
                ack_ad:
                begin
                    scl=~clock_count;
                    tristate=1; 
                    sda_out=1;
                    
                    if ((bit_count == 8) && (scl == 1'b1)) begin
                        if (sda_in == 1'b0) begin // ACK from Slave
                            if ((read_write_save == 1'b1) && (repeated_start_indication == 1'b1)) begin
                                next_state = data; // Read Data path after Repeated Start
                            end else begin
                                next_state = addreg; // Write Data or Register Address path
                            end
                        end else begin // NACK from Slave
                            next_state = stop;
                        end
                    end
                end
                addreg:
                begin
                    scl = ~clock_count;
                    tristate = 0;
                    sda_out = register_address_save[7];
                    if ((bit_count == 16) && (scl == 1'b1)) begin
                        next_state = ack_addreg;
                    end
                end
                ack_addreg: begin
                    scl = ~clock_count;
                    tristate = 1; 
                    sda_out = 1;
                    
                    if ((bit_count == 17) && (scl == 1'b1)) begin
                        if (sda_in == 1'b0) begin // ACK from Slave
                            if (read_write_save == 0) begin
                                next_state = data; // Start Write Data
                            end else begin
                                next_state = rep_start; // Correct path for Register Read
                            end
                        end else begin // NACK from Slave
                            next_state = stop;
                        end
                    end
                end
                data: begin
                    scl = ~clock_count;
                    
                    if (read_write_save == 0) begin // Write Data
                        tristate = 0;
                        sda_out = data_write[7];
                        if ((bit_count == 25) && (scl == 1'b1)) begin
                            next_state = data_ack;
                        end
                    end else begin // Read Data
                        tristate = 1; // Slave drives SDA
                        sda_out = 1;
                        if ((bit_count == 16) && (scl == 1'b1)) begin
                            next_state = data_ack;
                        end
                    end
                end
                data_ack: begin
                    scl = ~clock_count;
                    if (read_write_save == 0) begin // Read Slave ACK after Write
                        tristate = 1;
                        sda_out = 1;
                        if ((bit_count == 26) && (scl == 1'b1)) begin
                            next_state = stop;
                        end
                    end else begin // Master sends ACK/NACK after Read
                        tristate = 0;
                        sda_out = ack; 
                        if ((bit_count == 17) && (scl == 1'b1)) begin
                            next_state = stop;
                        end
                    end
                end
                stop: begin
                    scl = 1;
                    sda_out = 1;
                    if (clock_count == 1) next_state = idle;
                end
                rep_start: begin
                    scl = 1;
                    sda_out = 1;
                    repeated_start_indication = 1;
                    if (clock_count == 1) next_state = start;
                end
                default: next_state = idle;
            endcase
        end
    end
    
    // FSM Sequential Logic
    always @(posedge clk) begin
        if (rst) begin
            current_state <= 0;
            repeated_start_signal<=0;
        end
        else begin
            current_state <= next_state;
            repeated_start_signal<= repeated_start_indication;
        end
    end
    
endmodule
