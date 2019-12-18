/*
 * Copyright (c) 2014, Aleksander Osman
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

`include "defines.v"

module simple_dual_port_ram_single_clock
#(parameter DATA_WIDTH=40, parameter ADDR_WIDTH=8)
(
    input [(DATA_WIDTH-1):0] data,
    input [(ADDR_WIDTH-1):0] read_addr, write_addr,
    input we, clk,
    output reg [(DATA_WIDTH-1):0] q
);

    // Declare the RAM variable
    reg [DATA_WIDTH-1:0] ram[2**ADDR_WIDTH-1:0];

    // Specify the initial contents.  You can also use the $readmemb
    // system task to initialize the RAM variable from a text file.
    // See the $readmemb template page for details.
    initial 
    begin : INIT
        integer i;
        for(i = 0; i < 2**ADDR_WIDTH; i = i + 1)
            ram[i] = 40'd0;
    end   

    always @ (posedge clk)
    begin
        // Write
        if (we)
            ram[write_addr] <= data;

        // Read (if read_addr == write_addr, return OLD data).   To return
        // NEW data, use = (blocking write) rather than <= (non-blocking write)
        // in the write assignment.    NOTE: NEW data may require extra bypass
        // logic around the RAM.
        q <= ram[read_addr];
    end
endmodule


module tlb_l2cache(
    input               clk,
    input               rst_n,
    
    //RESP:
    input               tlbflushsingle_do,
    input   [31:0]      tlbflushsingle_address,
    //END
    
    //RESP:
    input               tlbflushall_do,
    //END
    
    input               rw,
    
    //RESP:
    input               tlbcache_write_do,
    input   [31:0]      tlbcache_write_linear,
    input   [31:0]      tlbcache_write_physical,
    
    input               tlbcache_write_pwt,
    input               tlbcache_write_pcd,
    input               tlbcache_write_combined_rw,
    input               tlbcache_write_combined_su,
    //END
    
    //RESP:
    input               translate_do,
    input   [31:0]      translate_linear,
    output              translate_valid,
    output  [31:0]      translate_physical,
    output              translate_pwt,
    output              translate_pcd,
    output              translate_combined_rw,
    output              translate_combined_su
    //END
);

//------------------------------------------------------------------------------
/* This cache module makes use of the Altera M10K memory blocks.  They can be
 * used as simple ram capable of holding 40bits in 256 (8-bit) arrays.
 *
 * The 40bit limitation is difficult since we need to store the linear address
 * (10-bit pde, 10-bit pte) as well as the physical address (12-bits provided
 * by the the linear address [11:0], 20-bits stored in the tlb) as well as the
 * access and caching flags (additional 6 bits).  Since this would require
 * 46 bits it would not work properly in the M10K block.  To get around this
 * we will rely on the fact that the ao486 project only provides 64MB (2^26) of
 * ram, allowing us to store only the (26 - linear(12-bits)=14 bits of physical
 * address data.
 *
 * Thus the cache map will look like:
 * 
 * [19:0]   linear page address [31:12] of request address
 *
 * [33:20]  physical page address (32-bit) { 6'b000000, this, linear[11:0] } 
 *
 * [34]     valid        (pde/pte bit 0)
 * [35]     combined r/w (pde/pte bit 1 ANDed)
 * [36]     combined s/u (pde/pte bit 2 ANDed)
 * [37]     page write-through (pde/pte bit 3)
 * [38]     page cache-disable (pde/pte bit 4)
 * [39]     dirty (pte bit 6)
 *
 *
 * To extend the cache further, multiple M10K cache blocks are setup based on
 * the last significant bits of the linear address, then put through a simple
 * hash function to get a semi-random distribution across the available (256)
 * addresses.  That means it is possible to have a hash collision where a
 * linear address will retrieve ram data for the wrong page, however the
 * retrieved page will have the stored linear address compared to the requested
 * page and return translate_valid = `FALSE.  This will result in a page walk
 * and then the data in the cache block replaced with the page walk data.
 *
 * While unfortunate, this should still be an improvement over the limited 32
 * addresses available in the L1 cache and the limited frequency should still
 * result in a general speed improvement.  Improvements welcome.
 *
 * NOTE: The M10K requires 1 cycle address latch/setup time, and data is presented on the 2nd clock cycle
 *
 */

`define TLBL2_BIT_VALID 34
`define TLBL2_BIT_RW    35
`define TLBL2_BIT_SU    36
`define TLBL2_BIT_WT    37
`define TLBL2_BIT_CD    38
`define TLBL2_BIT_DIRTY 39

wire        tlbcache_ready = ((state == STATE_RUN) && ~(tlbflushsingle_do) && ~(tlbflushall_do));
reg  [7:0]  tlbflush_address;
reg         tlbflush_all_do;
reg  [1:0]  state;

wire [39:0] tlbcache_write_data = (tlbcache_ready && tlbcache_write_do)? { rw, tlbcache_write_pcd, tlbcache_write_pwt,
                                                      tlbcache_write_combined_su, tlbcache_write_combined_rw, 1'b1,
                                                      tlbcache_write_physical[25:12], tlbcache_write_linear[31:12] } :
                                                      40'd0;

wire [31:0] tlbcache_write_selected_linear = (tlbflushsingle_do)? tlbflushsingle_address : tlbcache_write_linear;
wire        tlbcache_00_write_selected     = (tlbcache_write_selected_linear[13:12] == 2'b00);
wire        tlbcache_01_write_selected     = (tlbcache_write_selected_linear[13:12] == 2'b01);
wire        tlbcache_10_write_selected     = (tlbcache_write_selected_linear[13:12] == 2'b10);
wire        tlbcache_11_write_selected     = (tlbcache_write_selected_linear[13:12] == 2'b11);

// We're assuming 64MB of physical memory, don't cache anything not in that range
wire        physical_address_valid   = (tlbcache_write_physical[31:26] == 6'b000000);

// Set write enable if we need to flush an address, flush all addresses,
// or if the linear address matches our selected cache block
wire        write_enable             = ((tlbcache_write_do && physical_address_valid) || tlbflushsingle_do);
wire        tlbcache_00_write_enable = ((write_enable && tlbcache_00_write_selected)  || tlbflush_all_do);
wire        tlbcache_01_write_enable = ((write_enable && tlbcache_01_write_selected)  || tlbflush_all_do);
wire        tlbcache_10_write_enable = ((write_enable && tlbcache_10_write_selected)  || tlbflush_all_do);
wire        tlbcache_11_write_enable = ((write_enable && tlbcache_11_write_selected)  || tlbflush_all_do);

wire [7:0]  tlbcache_write_address   = (state == STATE_FLUSH_ALL)? tlbflush_address : 
                                       (tlbflushsingle_do)?        address_hash(tlbflushsingle_address) :
                                                                   address_hash(tlbcache_write_linear);

wire [7:0]  translate_linear_hash = address_hash(translate_linear);

wire [39:0] tlbcache_00_data;
wire [39:0] tlbcache_01_data;
wire [39:0] tlbcache_10_data;
wire [39:0] tlbcache_11_data;

// Setup 4 ram instances and choose which one to cache to/from depending
// on the binary value of the last significant bits of the linear address
wire        tlbcache_00_selected = (translate_do && (tlbcache_00_data[19:0] == translate_linear[31:12]) && tlbcache_00_data[`TLBL2_BIT_VALID]);
wire        tlbcache_01_selected = (translate_do && (tlbcache_01_data[19:0] == translate_linear[31:12]) && tlbcache_01_data[`TLBL2_BIT_VALID]);
wire        tlbcache_10_selected = (translate_do && (tlbcache_10_data[19:0] == translate_linear[31:12]) && tlbcache_10_data[`TLBL2_BIT_VALID]);
wire        tlbcache_11_selected = (translate_do && (tlbcache_11_data[19:0] == translate_linear[31:12]) && tlbcache_11_data[`TLBL2_BIT_VALID]);

wire [39:0] tlbcache_selected = (tlbcache_00_selected)?   tlbcache_00_data :
                                (tlbcache_01_selected)?   tlbcache_01_data :
                                (tlbcache_10_selected)?   tlbcache_10_data :
                                (tlbcache_11_selected)?   tlbcache_11_data :
                                                          40'd0;

assign      translate_valid       = (tlbcache_ready && tlbcache_selected[`TLBL2_BIT_VALID] && (~(rw) || tlbcache_selected[`TLBL2_BIT_DIRTY]));
assign      translate_physical    = (translate_valid)? { 6'b000000, tlbcache_selected[33:20], translate_linear[11:0] } : translate_linear;
assign      translate_pwt         = tlbcache_selected[`TLBL2_BIT_WT];
assign      translate_pcd         = tlbcache_selected[`TLBL2_BIT_CD];
assign      translate_combined_rw = tlbcache_selected[`TLBL2_BIT_RW];
assign      translate_combined_su = tlbcache_selected[`TLBL2_BIT_SU];


function automatic [7:0] address_hash;
    input [31:0] address;
    // Simple hash to get a simple probability distribution over the 8-bit ram address space.  Improvements welcome
    address_hash = { ^address[31:29], ^address[28:27], ^address[26:25], ^address[24:23],
                     ^address[22:20], ^address[19:18], ^address[17:16], ^address[15:14] };
endfunction


simple_dual_port_ram_single_clock tlb_cache_inst_00(
    .clk        (clk),                                              //input
    
    .write_addr (tlbcache_write_address),                           //input [7:0]
    .we         (tlbcache_00_write_enable),                         //input
    .data       (tlbcache_write_data),                              //input [39:0]
    
    .read_addr  (translate_linear_hash),                            //input [7:0]
    .q          (tlbcache_00_data)                                  //output [39:0]
);

simple_dual_port_ram_single_clock tlb_cache_inst_01(
    .clk        (clk),                                              //input
    
    .write_addr (tlbcache_write_address),                           //input [7:0]
    .we         (tlbcache_01_write_enable),                         //input
    .data       (tlbcache_write_data),                              //input [39:0]
    
    .read_addr  (translate_linear_hash),                            //input [7:0]
    .q          (tlbcache_01_data)                                  //output [39:0]
);

simple_dual_port_ram_single_clock tlb_cache_inst_10(
    .clk        (clk),                                              //input
    
    .write_addr (tlbcache_write_address),                           //input [7:0]
    .we         (tlbcache_10_write_enable),                         //input
    .data       (tlbcache_write_data),                              //input [39:0]
    
    .read_addr  (translate_linear_hash),                            //input [7:0]
    .q          (tlbcache_10_data)                                  //output [39:0]
);

simple_dual_port_ram_single_clock tlb_cache_inst_11(
    .clk        (clk),                                              //input
    
    .write_addr (tlbcache_write_address),                           //input [7:0]
    .we         (tlbcache_11_write_enable),                         //input
    .data       (tlbcache_write_data),                              //input [39:0]
    
    .read_addr  (translate_linear_hash),                            //input [7:0]
    .q          (tlbcache_11_data)                                  //output [39:0]
);

localparam [2:0] STATE_RUN             = 2'd0;
localparam [2:0] STATE_FLUSH_ALL_START = 2'd1;
localparam [2:0] STATE_FLUSH_ALL       = 2'd2;

initial begin
    state = STATE_RUN;
    tlbflush_address = 8'd0;
    tlbflush_all_do  = `FALSE;
end

//------------------------------------------------------------------------------
always @(posedge clk) begin

    case (state)
        STATE_RUN : begin
            if(tlbflushall_do || ~(rst_n)) begin // input
                state <= STATE_FLUSH_ALL_START;
            end else begin
                tlbflush_all_do <= `FALSE;
            end
        end

        STATE_FLUSH_ALL_START : begin
            tlbflush_address <= 8'd0;
            tlbflush_all_do  <= `TRUE;
            state            <= STATE_FLUSH_ALL;
        end

        STATE_FLUSH_ALL : begin
            // Cache zero writes happen here with tlbflush_address, two cycle write
            if (tlbflush_address == 8'd255) begin
                state <= STATE_RUN;
                tlbflush_all_do <= `FALSE;
            end else begin
                tlbflush_address <= tlbflush_address + 8'd1;
            end
        end

        default: state <= STATE_RUN;
    endcase
end

endmodule
