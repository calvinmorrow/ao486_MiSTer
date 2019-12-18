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

module tlb(
    input               clk,
    input               rst_n,

    input               pr_reset,
    input               rd_reset,
    input               exe_reset,
    input               wr_reset,
    
    // from control
    input               cr0_pg,
    input               cr0_wp,
    input               cr0_am,
    input               cr0_cd,
    input               cr0_nw,
    
    input               acflag,
    
    input   [31:0]      cr3,
    
    input               pipeline_after_read_empty,
    input               pipeline_after_prefetch_empty,
    
    output reg  [31:0]  tlb_code_pf_cr2,
    output reg  [15:0]  tlb_code_pf_error_code,
    
    output reg  [31:0]  tlb_check_pf_cr2,
    output reg  [15:0]  tlb_check_pf_error_code,
    
    output reg  [31:0]  tlb_write_pf_cr2,
    output reg  [15:0]  tlb_write_pf_error_code,
        
    output reg  [31:0]  tlb_read_pf_cr2,
    output reg  [15:0]  tlb_read_pf_error_code,
    
    //RESP:
    input               tlbflushsingle_do,
    output              tlbflushsingle_done,
    input   [31:0]      tlbflushsingle_address,
    //END
    
    //RESP:
    input               tlbflushall_do,
    //END
    
    //RESP:
    input               tlbread_do,
    output              tlbread_done,
    output              tlbread_page_fault,
    output              tlbread_ac_fault,
    output              tlbread_retry,
    
    input   [1:0]       tlbread_cpl,
    input   [31:0]      tlbread_address,
    input   [3:0]       tlbread_length,
    input   [3:0]       tlbread_length_full,
    input               tlbread_lock,
    input               tlbread_rmw,
    output  [63:0]      tlbread_data,
    //END
    
    //RESP:
    input               tlbwrite_do,
    output              tlbwrite_done,
    output              tlbwrite_page_fault,
    output              tlbwrite_ac_fault,
    
    input   [1:0]       tlbwrite_cpl,
    input   [31:0]      tlbwrite_address,
    input   [2:0]       tlbwrite_length,
    input   [2:0]       tlbwrite_length_full,
    input               tlbwrite_lock,
    input               tlbwrite_rmw,
    input   [31:0]      tlbwrite_data,
    //END
    
    //RESP:
    input               tlbcheck_do,
    output reg          tlbcheck_done,
    output              tlbcheck_page_fault,
    
    input   [31:0]      tlbcheck_address,
    input               tlbcheck_rw,
    //END
    
    //REQ:
    output              dcacheread_do,
    input               dcacheread_done,
    
    output reg [3:0]    dcacheread_length,
    output reg          dcacheread_cache_disable,
    output reg [31:0]   dcacheread_address,
    input   [63:0]      dcacheread_data,
    //END
    
    //REQ:
    output              dcachewrite_do,
    input               dcachewrite_done,
    
    output reg [2:0]    dcachewrite_length,
    output reg          dcachewrite_cache_disable,
    output reg [31:0]   dcachewrite_address,
    output reg          dcachewrite_write_through,
    output reg [31:0]   dcachewrite_data,
    //END
    input               dcache_busy,
    
    //RESP:
    input               tlbcoderequest_do,
    input       [31:0]  tlbcoderequest_address,
    input               tlbcoderequest_su,
    //END

    //REQ:
    output reg          tlbcode_do,
    output      [31:0]  tlbcode_linear,
    output reg  [31:0]  tlbcode_physical,
    output reg          tlbcode_cache_disable,
    //END

    //REQ:
    output reg          prefetchfifo_signal_pf_do
    //END
);

//------------------------------------------------------------------------------

reg [4:0]   state, next_state;

reg [31:0]  linear;
reg         su;
reg         rw;
reg         wp;
reg [1:0]   current_type;

reg [31:0]  pde;
reg [31:0]  pte;

reg         code_pf;
reg         check_pf;

reg         write_pf;
reg         write_ac;

reg         read_pf;
reg         read_ac;

reg [1:0]   fault_type;
reg         fault_do;

reg         pr_reset_waiting;

reg         tlbflushall_do_waiting;
reg         tlbcache_write;

reg [1:0]   write_double_state;
reg [31:0]  write_double_linear;

//------------------------------------------------------------------------------

wire        check_pf_to_reg;
wire        code_pf_to_reg;
wire        read_pf_to_reg;
wire        write_pf_to_reg;

wire [31:0] cr3_base;
wire        cr3_pwt;
wire        cr3_pcd;

assign cr3_base = { cr3[31:12], 12'd0 };
assign cr3_pwt  = cr3[3];
assign cr3_pcd  = cr3[4];

assign check_pf_to_reg = (fault_do && (fault_type == TYPE_CHECK))? `TRUE : check_pf;
assign code_pf_to_reg  = (fault_do && (fault_type == TYPE_CODE))?  `TRUE : code_pf;
assign read_pf_to_reg  = (fault_do && (fault_type == TYPE_READ));
assign write_pf_to_reg = (fault_do && (fault_type == TYPE_WRITE));

assign tlbread_page_fault  = read_pf;
assign tlbread_ac_fault    = read_ac;
assign tlbwrite_page_fault = write_pf;
assign tlbwrite_ac_fault   = write_ac;
assign tlbcheck_page_fault = check_pf;

wire state_idle = (state == STATE_IDLE);
wire state_read_wait = (state == STATE_READ_WAIT);

wire write_ac_to_reg = (state_idle && (~(tlbflush_waiting)) && (~(wr_reset) && tlbwrite_do && ~(write_ac) && cr0_am && acflag && tlbwrite_cpl == 2'd3 &&
            ( (tlbwrite_length_full == 3'd2 && tlbwrite_address[0] != 1'b0) || (tlbwrite_length_full == 3'd4 && tlbwrite_address[1:0] != 2'b00) )))? `TRUE : write_ac;

wire read_ac_to_reg = (state_idle && (~(tlbflush_waiting)) && (~(rd_reset) && tlbread_do && ~(read_ac) && cr0_am && acflag && tlbread_cpl == 2'd3 &&
            ( (tlbread_length_full == 4'd2 && tlbread_address[0] != 1'b0) || (tlbread_length_full == 4'd4 && tlbread_address[1:0] != 2'b00))))? `TRUE : read_ac;

//------------------------------------------------------------------------------

assign tlbread_data                 = dcacheread_data;
assign tlbcode_linear               = linear;

wire   tlbcache_l1_disable          = `FALSE;
wire   tlbcache_l2_disable          = `FALSE;

assign tlbcache_write_do            = (state == STATE_LOAD_PTE_END);
assign tlbcache_write_linear        = linear;
assign tlbcache_write_physical      = { pte[31:12], linear[11:0] };
assign tlbcache_write_pwt           = pte[3];
assign tlbcache_write_pcd           = pte[4];
assign tlbcache_write_combined_rw   = (pde[1] & pte[1]);
assign tlbcache_write_combined_su   = (pde[2] & pte[2]);

wire tlbflush_waiting = (tlbflushsingle_do || tlbflushall_do || tlbflushall_do_waiting);

// Flush TLB contents when requested
assign tlbcache_tlbflushall_do      = (state_idle && (tlbflushall_do || tlbflushall_do_waiting));
assign tlbcache_tlbflushsingle_do   = (state_idle && tlbflushsingle_do);
assign tlbflushsingle_done          = tlbcache_tlbflushsingle_do;
assign translate_l1_do              = (cr0_pg && ~(tlbcache_l1_disable) && (state == STATE_L1_CHECK));
assign translate_l2_do              = (cr0_pg && ~(tlbcache_l2_disable) && (state == STATE_L2_CHECK));

// DCache is a stateful module and needs signals held until its in STATE_IDLE and ready for them
wire dcache_read_ready  = (state == STATE_DCACHE_READ && ~(dcache_busy));
wire dcache_write_ready = (state == STATE_DCACHE_WRITE && ~(dcache_busy));

assign dcacheread_do    = dcache_read_ready;
assign dcachewrite_do   = dcache_write_ready;

// Set tlbwrite_done if dcache accepts our write request and we'll be idle next clock
assign tlbwrite_done    = (dcache_write_ready && (next_state == STATE_IDLE));
assign tlbread_done     = (state_read_wait && dcacheread_done);
assign tlbread_retry    = (state == STATE_RETRY)? (current_type == TYPE_READ) : `FALSE;

//------------------------------------------------------------------------------

localparam [4:0] STATE_IDLE             = 5'd0;
localparam [4:0] STATE_DCACHE_READ      = 5'd1;
localparam [4:0] STATE_DCACHE_WRITE     = 5'd2;
localparam [4:0] STATE_L1_CHECK         = 5'd3;
localparam [4:0] STATE_L2_CHECK         = 5'd4;
localparam [4:0] STATE_LOAD_PDE         = 5'd5;
localparam [4:0] STATE_LOAD_PTE         = 5'd6;
localparam [4:0] STATE_LOAD_PTE_END     = 5'd7;
localparam [4:0] STATE_CACHE_WRITE      = 5'd8;
localparam [4:0] STATE_L2_WAIT          = 5'd9;
localparam [4:0] STATE_READ_WAIT        = 5'd10;
localparam [4:0] STATE_FAULT            = 5'd11;
localparam [4:0] STATE_RETRY            = 5'd12;
localparam [4:0] STATE_WRITE_DOUBLE     = 5'd13;

localparam [1:0] TYPE_CODE  = 2'd0;
localparam [1:0] TYPE_CHECK = 2'd1;
localparam [1:0] TYPE_WRITE = 2'd2;
localparam [1:0] TYPE_READ  = 2'd3;

localparam [1:0] WRITE_DOUBLE_NONE    = 2'd0;
localparam [1:0] WRITE_DOUBLE_CHECK   = 2'd1;
localparam [1:0] WRITE_DOUBLE_RESTART = 2'd2;

wire translate_combined_rw;
wire translate_combined_su;

wire        tlbcache_tlbflushsingle_do;
wire        tlbcache_tlbflushall_do;

wire        translate_l1_do;
wire        translate_valid;
wire [31:0] translate_physical;
wire        translate_pwt;
wire        translate_pcd;

wire        translate_l2_do;
wire        translate_l2_valid;
wire [31:0] translate_l2_physical;
wire        translate_l2_pwt;
wire        translate_l2_pcd;
wire        translate_l2_combined_rw;
wire        translate_l2_combined_su;

wire        tlbcache_write_do;
wire [31:0] tlbcache_write_linear;
wire [31:0] tlbcache_write_physical;
wire        tlbcache_write_pwt;
wire        tlbcache_write_pcd;
wire        tlbcache_write_combined_rw;
wire        tlbcache_write_combined_su;

//------------------------------------------------------------------------------

tlb_l1cache tlb_l1cache_inst(
    .clk                        (clk),
    .rst_n                      (rst_n),
    
    //RESP:
    .tlbflushsingle_do          (tlbcache_tlbflushsingle_do),   //input
    .tlbflushsingle_address     (tlbflushsingle_address),       //input [31:0]
    //END
    
    //RESP:
    .tlbflushall_do             (tlbcache_tlbflushall_do),      //input
    //END
    
    .rw                         (rw),                           //input
    
    //RESP:
    .tlbcache_write_do          (tlbcache_write_do),            //input
    .tlbcache_write_linear      (tlbcache_write_linear),        //input [31:0]
    .tlbcache_write_physical    (tlbcache_write_physical),      //input [31:0]
    
    .tlbcache_write_pwt         (tlbcache_write_pwt),           //input
    .tlbcache_write_pcd         (tlbcache_write_pcd),           //input
    .tlbcache_write_combined_rw (tlbcache_write_combined_rw),   //input
    .tlbcache_write_combined_su (tlbcache_write_combined_su),   //input
    //END
    
    //RESP:
    .translate_do               (translate_l1_do),              //input
    .translate_linear           (linear),                       //input [31:0]
    .translate_valid            (translate_valid),              //output
    .translate_physical         (translate_physical),           //output [31:0]
    .translate_pwt              (translate_pwt),                //output
    .translate_pcd              (translate_pcd),                //output
    .translate_combined_rw      (translate_combined_rw),        //output
    .translate_combined_su      (translate_combined_su)         //output
    //END
);

//------------------------------------------------------------------------------

tlb_l2cache tlb_l2cache_inst(
    .clk                        (clk),
    .rst_n                      (rst_n),
    
    //RESP:
    .tlbflushsingle_do          (tlbcache_tlbflushsingle_do),   //input
    .tlbflushsingle_address     (tlbflushsingle_address),       //input [31:0]
    //END
    
    //RESP:
    .tlbflushall_do             (tlbcache_tlbflushall_do),      //input
    //END
    
    .rw                         (rw),                           //input
    
    //RESP:
    .tlbcache_write_do          (tlbcache_write_do),            //input
    .tlbcache_write_linear      (tlbcache_write_linear),        //input [31:0]
    .tlbcache_write_physical    (tlbcache_write_physical),      //input [31:0]
    
    .tlbcache_write_pwt         (tlbcache_write_pwt),           //input
    .tlbcache_write_pcd         (tlbcache_write_pcd),           //input
    .tlbcache_write_combined_rw (tlbcache_write_combined_rw),   //input
    .tlbcache_write_combined_su (tlbcache_write_combined_su),   //input
    //END
    
    //RESP:
    .translate_do               (translate_l2_do),              //input
    .translate_linear           (linear),                       //input [31:0]
    .translate_valid            (translate_l2_valid),           //output
    .translate_physical         (translate_l2_physical),        //output [31:0]
    .translate_pwt              (translate_l2_pwt),             //output
    .translate_pcd              (translate_l2_pcd),             //output
    .translate_combined_rw      (translate_l2_combined_rw),     //output
    .translate_combined_su      (translate_l2_combined_su)      //output
    //END

);

//------------------------------------------------------------------------------

always @(posedge clk) begin
    if(rst_n == 1'b0)   code_pf <= `FALSE;
    else if(pr_reset)   code_pf <= `FALSE;
    else                code_pf <= code_pf_to_reg;
end

always @(posedge clk) begin
    if(rst_n == 1'b0)   check_pf <= `FALSE;
    else if(exe_reset)  check_pf <= `FALSE;
    else                check_pf <= check_pf_to_reg;
end

always @(posedge clk) begin
    if(rst_n == 1'b0)   read_pf <= `FALSE;
    else if(rd_reset)   read_pf <= `FALSE;
    else                read_pf <= read_pf_to_reg;
end

always @(posedge clk) begin
    if(rst_n == 1'b0)   read_ac <= `FALSE;
    else if(rd_reset)   read_ac <= `FALSE;
    else                read_ac <= read_ac_to_reg;
end

always @(posedge clk) begin
    if(rst_n == 1'b0)   write_pf <= `FALSE;
    else if(wr_reset)   write_pf <= `FALSE;
    else                write_pf <= write_pf_to_reg;
end

always @(posedge clk) begin
    if(rst_n == 1'b0)   write_ac <= `FALSE;
    else if(wr_reset)   write_ac <= `FALSE;
    else                write_ac <= write_ac_to_reg;
end

always @(posedge clk) begin
    if(rst_n == 1'b0)                           pr_reset_waiting <= `FALSE;
    else if(pr_reset && ~state_idle)            pr_reset_waiting <= `TRUE;
    else if(state_idle)                         pr_reset_waiting <= `FALSE;
end

always @(posedge clk) begin
    if(rst_n == 1'b0)                           tlbflushall_do_waiting <= `FALSE;
    else if(tlbflushall_do && ~state_idle)      tlbflushall_do_waiting <= `TRUE;
    else if(tlbcache_tlbflushall_do)            tlbflushall_do_waiting <= `FALSE;
end

//------------------------------------------------------------------------------

// synthesis translate_off
wire _unused_ok = &{ 1'b0, cr3[11:5], cr3[2:0], tlbread_lock, tlbwrite_lock, tlbwrite_rmw, cr3_base[11:0], 1'b0 };
// synthesis translate_on

initial begin
   current_type               = 2'd0;
   fault_do                   = `FALSE;
   fault_type                 = 2'd0;
   linear                     = 32'd0;
   next_state                 = STATE_IDLE;
   pde                        = 32'd0;
   prefetchfifo_signal_pf_do  = `FALSE;
   pte                        = 32'd0;
   rw                         = `FALSE;
   state                      = STATE_IDLE;
   su                         = `FALSE;
   tlb_code_pf_error_code     = 16'd0;
   tlb_read_pf_error_code     = 16'd0;
   tlb_write_pf_error_code    = 16'd0;
   tlb_check_pf_error_code    = 16'd0;
   tlb_code_pf_cr2            = 32'd0;
   tlb_read_pf_cr2            = 32'd0;
   tlb_write_pf_cr2           = 32'd0;
   tlb_check_pf_cr2           = 32'd0;
   tlbcache_write             = `FALSE;
   tlbcheck_done              = `FALSE;
   tlbcode_do                 = `FALSE;
   tlbcode_cache_disable      = `FALSE;
   tlbcode_physical           = 32'd0;
   wp                         = `FALSE;
   write_double_linear        = 32'd0;
   write_double_state         = 2'd0;
end

function memtype_cache_disable;
    input [31:0] address;
    begin
        memtype_cache_disable = (address[31:4] >= 28'h000A000 && address[31:4] < 28'h000C000);
    end
endfunction

function access_fault;
    input rw;
    input rw_flag;
    input su;
    input su_flag;
    input wp;
    begin
        access_fault = 
        // user can not access supervisor
        ((su && ~(su_flag)) ||
        // user can not write on read-only page
        (su && su_flag && ~(rw_flag) && rw) ||
        // supervisor can not write on read-only page when write-protect is on
        (wp && ~(su) && ~(rw_flag) && rw));
    end
endfunction

task automatic raise_page_fault;
    input       access;
    input       rw;
    input       su;
    begin
        if ((current_type == TYPE_CODE) && ~(pr_reset) && ~(pr_reset_waiting)) begin
            tlb_code_pf_cr2         <= linear;
            tlb_code_pf_error_code  <= { 13'd0, su, rw, access };
            fault_type              <= TYPE_CODE;
            state                   <= STATE_FAULT;
            fault_do                <= `TRUE;
            prefetchfifo_signal_pf_do  <= `TRUE;
        end else if (current_type == TYPE_CHECK) begin
            tlb_check_pf_cr2        <= linear;
            tlb_check_pf_error_code <= { 13'd0, su, rw, access };
            fault_type              <= TYPE_CHECK;
            state                   <= STATE_FAULT;
            fault_do                <= `TRUE;
        end else if (current_type == TYPE_WRITE) begin
            tlb_write_pf_cr2        <= linear;
            tlb_write_pf_error_code <= { 13'd0, su, rw, access };
            fault_type              <= TYPE_WRITE;
            state                   <= STATE_FAULT;
            fault_do                <= `TRUE;
        end else if (current_type == TYPE_READ) begin
            tlb_read_pf_cr2         <= linear;
            tlb_read_pf_error_code  <= { 13'd0, su, rw, access };
            fault_type              <= TYPE_READ;
            state                   <= STATE_FAULT;
            fault_do                <= `TRUE;
        end else begin
            state                   <= STATE_IDLE;
        end
    end
endtask

task automatic dcache_write;
    input [31:0] address;
    input  [2:0] length;
    input        cache_disable;
    input        write_through;
    input [31:0] data;
    input  [4:0] state_after_write;
    begin
        dcachewrite_address       <= address;
        dcachewrite_length        <= length;
        dcachewrite_cache_disable <= (cache_disable || memtype_cache_disable(address));
        dcachewrite_write_through <= (write_through || memtype_cache_disable(address));
        dcachewrite_data          <= data;
        state                     <= STATE_DCACHE_WRITE;
        next_state                <= state_after_write;
    end
endtask

task automatic dcache_read;
    input [31:0] address;
    input  [3:0] length;
    input        cache_disable;
    input  [4:0] state_after_read;
    begin
        dcacheread_address        <= address;
        dcacheread_length         <= length;
        dcacheread_cache_disable  <= (cache_disable || memtype_cache_disable(address));
        state                     <= STATE_DCACHE_READ;
        next_state                <= state_after_read;
    end
endtask

//------------------------------------------------------------------------------
always @(posedge clk) begin

    case (state)
        STATE_DCACHE_READ : begin
            tlbcache_write <= `FALSE;
            // Wait for dcache idle, then proceed to next state.
            // dcacheread_do set when STATE_DCACHE_READ and ~dcache_busy
            if (~dcache_busy) state <= next_state;
        end

        STATE_DCACHE_WRITE : begin
            tlbcache_write <= `FALSE;       
            // dcachewrite_done set in dcache::STATE_IDLE as soon as write accepted
            if (~dcache_busy) state <= next_state;
        end

        STATE_CACHE_WRITE : begin
            tlbcache_write <= `FALSE;
            state <= next_state;
        end

        STATE_READ_WAIT : begin
            if (dcacheread_done) begin
                // tlbread_done set here
                state <= STATE_IDLE;
            end
        end

        STATE_IDLE : begin
            fault_do                <= `FALSE;
            tlbcache_write          <= `FALSE;
            tlbcheck_done           <= `FALSE;
            tlbcode_do              <= `FALSE;

            if (tlbflush_waiting) begin
                state   <= STATE_IDLE;     
            // Write if no exceptions set
            end else if (~(wr_reset) && tlbwrite_do && ~(write_pf) && ~(write_ac || write_ac_to_reg)) begin
                rw      <= `TRUE;
                su      <= (tlbwrite_cpl == 2'd3);
                wp      <= cr0_wp;
                linear  <= tlbwrite_address;
                state   <= STATE_L1_CHECK;
                current_type <= TYPE_WRITE;
                // Check if write request crosses the linear (4KB) page boundry.
                // If so, extra steps taken to check access flags in both pages
                // prior to writing
                write_double_state <= ((cr0_pg && tlbwrite_length != tlbwrite_length_full && { 1'b0, tlbwrite_address[11:0] } + { 10'd0, tlbwrite_length_full } >= 13'h1000)? WRITE_DOUBLE_CHECK : WRITE_DOUBLE_NONE);
            end else if (~(exe_reset) && tlbcheck_do && ~(tlbcheck_done) && ~(check_pf)) begin
                rw      <= tlbcheck_rw;
                su      <= `FALSE;
                wp      <= cr0_wp;
                linear  <= tlbcheck_address;
                state   <= STATE_L1_CHECK;
                current_type <= TYPE_CHECK;
                write_double_state <= WRITE_DOUBLE_NONE;
            end else if (~(rd_reset) && tlbread_do && ~(read_pf) && ~(read_ac || read_ac_to_reg)) begin
                rw      <= tlbread_rmw;
                su      <= (tlbread_cpl == 2'd3);
                wp      <= cr0_wp;
                linear  <= tlbread_address;
                state   <= STATE_L1_CHECK;
                current_type <= TYPE_READ;
                write_double_state <= WRITE_DOUBLE_NONE;
            end else if (~(pr_reset) && tlbcoderequest_do && ~(code_pf) && ~(tlbcode_do)) begin
                rw      <= `FALSE;
                su      <= tlbcoderequest_su;
                wp      <= cr0_wp;
                linear  <= tlbcoderequest_address;
                state   <= STATE_L1_CHECK;
                current_type <= TYPE_CODE;
                write_double_state <= WRITE_DOUBLE_NONE;
            end
        end

        // If a linear (cr_pg) write request crosses the 4KB boundry, pde and pte flags need
        // to be checked for access faults in BOTH pages prior to writing data.
        // WRITE_DOUBLE_CHECK and WRITE_DOUBLE_RESTART will not perform any modifications
        // of the data page (dcache_write function intercepted) but serve only for raising
        // page faults.  Once checks completed, write processed when state WRITE_DOUBLE_NONE
        STATE_WRITE_DOUBLE : begin
            if (write_double_state == WRITE_DOUBLE_CHECK) begin
                write_double_linear <= linear;
                linear <= ({ linear[31:12], 12'd0 } + 32'h00001000);
                write_double_state <= WRITE_DOUBLE_RESTART;
            end else begin
                linear <= write_double_linear;
                write_double_state <= WRITE_DOUBLE_NONE;
            end
            state <= STATE_L1_CHECK;
        end
      
        STATE_L1_CHECK : begin
            if (current_type == TYPE_CODE && (pr_reset || pr_reset_waiting)) begin  //NOTE: pr_reset required
                state <= STATE_IDLE;
            end else if (~(cr0_pg) || translate_valid) begin
                if (cr0_pg && access_fault(rw, translate_combined_rw, su, translate_combined_su, wp)) begin
                    raise_page_fault(`TRUE, rw, su);
                end else begin
                    case (current_type)
                        TYPE_READ : begin
                            // Read memory from original (cr0_pg == 0) or tlb hit (translate_physical) address
                            dcache_read(translate_physical, tlbread_length, (cr0_cd || translate_pcd), STATE_READ_WAIT);
                        end
                        TYPE_WRITE : begin
                            if (write_double_state != WRITE_DOUBLE_NONE) begin
                                state <= STATE_WRITE_DOUBLE;
                            end else begin
                                dcache_write(translate_physical, tlbwrite_length, (cr0_cd || translate_pcd), (cr0_nw || translate_pwt), tlbwrite_data, STATE_IDLE);
                            end
                        end
                        TYPE_CHECK : begin
                            state         <= STATE_IDLE;
                            tlbcheck_done <= `TRUE;
                        end
                        TYPE_CODE : begin
                            tlbcode_physical        <= translate_physical;
                            tlbcode_do              <= `TRUE;
                            tlbcode_cache_disable   <= (cr0_cd || translate_pcd || memtype_cache_disable(translate_physical));
                            state                   <= STATE_IDLE;
                        end
                    endcase
                end
            end else begin
                // L1 TLB Cache miss, Check L2 before proceeding to load pde / pte
                state <= STATE_L2_CHECK;
            end
        end

        STATE_L2_CHECK : begin
            if (current_type == TYPE_CODE && (pr_reset || pr_reset_waiting)) begin  //NOTE: pr_reset required
                state <= STATE_IDLE;
                // Any requests for ~cr0_pg would have been handled in STATE_L1_CHECK
            end else if (translate_l2_valid && ~(tlbcache_l2_disable)) begin
                if (cr0_pg && access_fault(rw, translate_l2_combined_rw, su, translate_l2_combined_su, wp)) begin
                    raise_page_fault(`TRUE, rw, su);
                end else begin
                    case (current_type)
                        TYPE_READ : begin
                            // Read memory from original (cr0_pg == 0) or tlb hit (translate_physical) address
                            dcache_read(translate_l2_physical, tlbread_length, (cr0_cd || translate_l2_pcd), STATE_READ_WAIT);
                        end
                        TYPE_WRITE : begin
                            if (write_double_state != WRITE_DOUBLE_NONE) begin
                                state <= STATE_WRITE_DOUBLE;
                            end else begin
                                dcache_write(translate_l2_physical, tlbwrite_length, (cr0_cd || translate_l2_pcd), (cr0_nw || translate_l2_pwt), tlbwrite_data, STATE_IDLE);
                            end
                        end                  
                        TYPE_CHECK : begin
                            state         <= STATE_IDLE;
                            tlbcheck_done <= `TRUE;
                        end
                        TYPE_CODE : begin
                            tlbcode_physical        <= translate_l2_physical;
                            tlbcode_do              <= `TRUE;
                            tlbcode_cache_disable   <= (cr0_cd || translate_l2_pcd || memtype_cache_disable(translate_l2_physical));
                            state                   <= STATE_IDLE;
                        end
                    endcase
                end
            end else begin
                // L2 TLB Cache miss, proceed to load pde / pte
                dcache_read({cr3_base[31:12], linear[31:22], 2'd0}, 4'd4, (cr0_cd || cr3_pcd), STATE_LOAD_PDE);
            end
        end

        STATE_LOAD_PDE : begin
            if (dcacheread_done) begin
                pde <= dcacheread_data[31:0];
                if (dcacheread_data[0] == `FALSE) begin
                    raise_page_fault(`FALSE, rw, su);
                end else begin
                    dcache_read({dcacheread_data[31:12], linear[21:12], 2'd0}, 4'd4, (cr0_cd || dcacheread_data[4]), STATE_LOAD_PTE);
                end
            end
        end
      
        STATE_LOAD_PTE : begin
            if (dcacheread_done) begin
                pte <= dcacheread_data[31:0];
                if ((dcacheread_data[0] == `FALSE) || access_fault(rw, (pde[1] & dcacheread_data[1]), su, (pde[2] & dcacheread_data[2]), wp)) begin
                    raise_page_fault(dcacheread_data[0], rw, su);
                end else if (((current_type == TYPE_READ && ~(pipeline_after_read_empty)) || (current_type == TYPE_CODE && (~(pipeline_after_prefetch_empty) || pr_reset_waiting))) &&
                            (pde[5] == `FALSE || dcacheread_data[5] == `FALSE || (dcacheread_data[6] == `FALSE && rw))) begin
                    state          <= STATE_RETRY;
                end else begin
                    state          <= STATE_LOAD_PTE_END;
                end
            end
        end

        STATE_LOAD_PTE_END : begin
            // If the access bit isn't set on pde, set it and save the pde to RAM
            if (pde[5] == `FALSE) begin
                dcache_write({cr3_base[31:12], linear[31:22], 2'd0}, 3'd4, (cr0_cd || cr3_pcd), (cr0_nw || cr3_pwt), (pde | 32'h00000020), STATE_LOAD_PTE_END);
                pde <= (pde | 32'h00000020);
            // If the access bit isn't set on pte or the operation is a write and the dirty bit isn't set, set those flags and save the pte to RAM
            end else if (pte[5] == `FALSE || (pte[6] == `FALSE && rw)) begin
                dcache_write({pde[31:12], linear[21:12], 2'd0}, 3'd4, (cr0_cd || pde[4]), (cr0_nw || pde[3]), (pte[31:0] | 32'h00000020 | ((pte[6] == `FALSE && rw)? 32'h00000040 : 32'h00000000)), STATE_LOAD_PTE_END);
                pte <= (pte[31:0] | 32'h00000020 | (((pte[6] == `FALSE) && rw)? 32'h00000040 : 32'h00000000));
            end else begin
                if ((current_type == TYPE_CODE) && ~(pr_reset || pr_reset_waiting)) begin
                    tlbcode_physical        <= { pte[31:12], linear[11:0] };
                    tlbcode_do              <= `TRUE;
                    tlbcode_cache_disable   <= cr0_cd;
                    state <= STATE_IDLE;
                end else if ((current_type == TYPE_WRITE) && (write_double_state != WRITE_DOUBLE_NONE)) begin
                    state <= STATE_WRITE_DOUBLE;
                end else if (current_type == TYPE_WRITE) begin
                    dcache_write({ pte[31:12], linear[11:0] }, tlbwrite_length, (cr0_cd || pte[4]), (cr0_nw || pte[3]), tlbwrite_data, STATE_IDLE);
                end else if (current_type == TYPE_READ) begin
                    dcache_read({ pte[31:12], linear[11:0] }, tlbread_length, (cr0_cd || pte[4]), STATE_READ_WAIT);
                end else begin
                    state <= STATE_IDLE;
                end
            end
        end

        STATE_FAULT : begin
            fault_do <= `FALSE;
            state    <= STATE_IDLE;
            prefetchfifo_signal_pf_do  <= `FALSE;
        end

        STATE_RETRY : begin
            // tlbread_retry set here if (current_type == TYPE_READ)
            state    <= STATE_IDLE;
        end

        default: state <= STATE_IDLE;
    endcase
end

endmodule
