// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 19/04/2017
// Description: Memory Management Unit for Ariane, contains TLB and
//              address translation unit. SV39 as defined in RISC-V
//              privilege specification 1.11-WIP

import ariane_pkg::*;

module mmu #(
      parameter int unsigned INSTR_TLB_ENTRIES     = 4,
      parameter int unsigned DATA_TLB_ENTRIES      = 4,
      parameter int unsigned ASID_WIDTH            = 1,
      parameter ariane_pkg::ariane_cfg_t ArianeCfg = ariane_pkg::ArianeDefaultConfig
) (
        input  logic                            clk_i,
        input  logic                            rst_ni,
        input  logic                            flush_i,
        input  logic                            enable_translation_i,
        input  logic                            en_ld_st_translation_i,   // enable virtual memory translation for load/stores
        // IF interface
        input  icache_areq_o_t                  icache_areq_i,
        output icache_areq_i_t                  icache_areq_o,
        // LSU interface
        // this is a more minimalistic interface because the actual addressing logic is handled
        // in the LSU as we distinguish load and stores, what we do here is simple address translation
        input  exception_t                      misaligned_ex_i,
        input  logic                            lsu_req_i,        // request address translation
        input  logic [riscv::VLEN-1:0]          lsu_vaddr_i,      // virtual address in
        input  logic                            lsu_is_store_i,   // the translation is requested by a store
        // if we need to walk the page table we can't grant in the same cycle
        // Cycle 0
        output logic                            lsu_dtlb_hit_o,   // sent in the same cycle as the request if translation hits in the DTLB
        // Cycle 1
        output logic                            all_tlbs_checked_o, // sent in min - the same cycle, max - three cycles
        output logic                            lsu_valid_o,      // translation is valid
        output logic [riscv::PLEN-1:0]          lsu_paddr_o,      // translated address
        output exception_t                      lsu_exception_o,  // address translation threw an exception
        // General control signals
        input riscv::priv_lvl_t                 priv_lvl_i,
        input riscv::priv_lvl_t                 ld_st_priv_lvl_i,
        input logic                             sum_i,
        input logic                             mxr_i,
        // input logic flag_mprv_i,
        input logic [43:0]                      satp_ppn_i,
        input logic [ASID_WIDTH-1:0]            asid_i,
        input logic                             flush_tlb_i,
        // Performance counters
        output logic                            itlb_miss_o,
        output logic                            dtlb_miss_o,
        // PTW memory interface
        input  dcache_req_o_t                   req_port_i,
        output dcache_req_i_t                   req_port_o
);

    logic        iaccess_err;   // insufficient privilege to access this instruction page
    logic        daccess_err;   // insufficient privilege to access this data page
    logic        ptw_active;    // PTW is currently walking a page table
    logic        walking_instr; // PTW is walking because of an ITLB miss
    logic        ptw_error;     // PTW threw an exception

    logic [riscv::VLEN-1:0] update_vaddr;
    tlb_update_t update_ptw_itlb, update_ptw_dtlb;
    tlb_update_t update_l2_tlb, update_itlb, update_dtlb; // TLB-PTW interface

    logic        itlb_lu_access;
    riscv::pte_t itlb_content;
    logic        itlb_is_2M;
    logic        itlb_is_1G;
    logic        itlb_lu_hit;

    logic        dtlb_lu_access;
    riscv::pte_t dtlb_content;
    logic        dtlb_is_2M;
    logic        dtlb_is_1G;
    logic        dtlb_lu_hit;

    logic [riscv::VLEN-1:0] l2_tlb_vaddr_i;
    riscv::pte_t l2_tlb_content;
    logic        l2_tlb_is_2M;
    logic        l2_tlb_is_1G;
    logic        l2_tlb_lu_hit;
    logic        all_hashes_checked;   

    // Assignments
    assign itlb_lu_access = icache_areq_i.fetch_req;
    assign dtlb_lu_access = lsu_req_i;

    // l2 trlb entry multiplexer that covers single cycle, double access case also.
    // If pending dtransl-n is being served in ptw, then itransl-n is directed to l2tlb
    // Unless otherwise, dtransl-n has higher priority for l2tlb 
    logic  lsu_first;
    assign lsu_first = (dtlb_lu_access && !dtlb_lu_hit && (!ptw_active || (ptw_active && walking_instr)));
    
    assign l2_tlb_vaddr_i = lsu_first ? lsu_vaddr_i : icache_areq_i.fetch_vaddr; 
    
    assign all_tlbs_checked_o = all_hashes_checked; // output assignment
    
    // update L1 TLBs directly from L2 TLB
    assign update_itlb   = '{
                                valid   : l2_tlb_lu_hit && !itlb_lu_hit && !lsu_first,
                                is_2M   : l2_tlb_is_2M,
                                is_1G   : l2_tlb_is_1G,
                                vpn     : icache_areq_i.fetch_vaddr[38:12],
                                asid    : asid_i,
                                content : l2_tlb_content    
                            };
                             
    assign update_dtlb   = '{
                                valid   : l2_tlb_lu_hit && !dtlb_lu_hit && dtlb_lu_access,
                                is_2M   : l2_tlb_is_2M,
                                is_1G   : l2_tlb_is_1G,
                                vpn     : lsu_vaddr_i[38:12],
                                asid    : asid_i,
                                content : l2_tlb_content    
                            };
                            
    assign update_l2_tlb = (!l2_tlb_lu_hit && !itlb_lu_hit && (!dtlb_lu_access || dtlb_lu_hit)) ? update_ptw_itlb : update_ptw_dtlb;

    tlb #(
        .TLB_ENTRIES      ( INSTR_TLB_ENTRIES          ),
        .ASID_WIDTH       ( ASID_WIDTH                 )
    ) i_itlb (
        .clk_i            ( clk_i                      ),
        .rst_ni           ( rst_ni                     ),
        .flush_i          ( flush_tlb_i                ),

        .update_i         ( update_ptw_itlb            ),

        .lu_access_i      ( itlb_lu_access             ),
        .lu_asid_i        ( asid_i                     ),
        .lu_vaddr_i       ( icache_areq_i.fetch_vaddr              ),
        .lu_content_o     ( itlb_content               ),

        .lu_is_2M_o       ( itlb_is_2M                 ),
        .lu_is_1G_o       ( itlb_is_1G                 ),
        .lu_hit_o         ( itlb_lu_hit                )
    );

    tlb #(
        .TLB_ENTRIES     ( DATA_TLB_ENTRIES             ),
        .ASID_WIDTH      ( ASID_WIDTH                   )
    ) i_dtlb (
        .clk_i            ( clk_i                       ),
        .rst_ni           ( rst_ni                      ),
        .flush_i          ( flush_tlb_i                 ),

        .update_i         ( update_ptw_dtlb             ),

        .lu_access_i      ( dtlb_lu_access              ),
        .lu_asid_i        ( asid_i                      ),
        .lu_vaddr_i       ( lsu_vaddr_i                 ),
        .lu_content_o     ( dtlb_content                ),

        .lu_is_2M_o       ( dtlb_is_2M                  ),
        .lu_is_1G_o       ( dtlb_is_1G                  ),
        .lu_hit_o         ( dtlb_lu_hit                 )
    );

    tlb_l2 i_tlb_l2 (
         .clk_i            ( clk_i                      ),
         .rst_ni           ( rst_ni                     ),
         .flush_i          ( flush_tlb_i                ),

         .update_i         ( update_l2_tlb              ),
         .ptw_active_i     ( ptw_active                 ),

         .lu_access_i      ( itlb_lu_access || dtlb_lu_access ),
         .lu_asid_i        ( asid_i                     ),
	     .asid_to_be_flushed_i  ( asid_to_be_flushed_i  ),
	     .vaddr_to_be_flushed_i ( vaddr_to_be_flushed_i ),
         .lu_vaddr_i       ( l2_tlb_vaddr_i             ),
         .lu_content_o     ( l2_tlb_content             ),

         .lu_is_2M_o       ( l2_tlb_is_2M               ),
         .lu_is_1G_o       ( l2_tlb_is_1G               ),
         .lu_hit_o         ( l2_tlb_lu_hit              ),
         .all_hashes_checked_o ( all_hashes_checked     )
    );

    // Seperate wires are necessary to support independent 
    // L1, L2 hit results in case of double access
    
    riscv::pte_t f_itlb_content; // final itlb content
    logic        f_itlb_is_2M;
    logic        f_itlb_is_1G;
    logic        f_itlb_lu_hit;    
    
    // access l2tlb if only there are no d access or if l1 dtlb hits or if dtransl-n is being served in ptw
    assign f_itlb_content = (!itlb_lu_hit && !lsu_first) ? l2_tlb_content : itlb_content;    
    assign f_itlb_is_2M   = (!itlb_lu_hit && !lsu_first) ? l2_tlb_is_2M   : itlb_is_2M;
    assign f_itlb_is_1G   = (!itlb_lu_hit && !lsu_first) ? l2_tlb_is_1G   : itlb_is_1G;   
    assign f_itlb_lu_hit  = (!itlb_lu_hit && !lsu_first) ? l2_tlb_lu_hit  : itlb_lu_hit;
                                                            
    riscv::pte_t f_dtlb_content; // final dtlb content
    logic        f_dtlb_is_2M;
    logic        f_dtlb_is_1G;
    logic        f_dtlb_lu_hit;        
    
    assign f_dtlb_content = (!dtlb_lu_hit) ? l2_tlb_content : dtlb_content;    
    assign f_dtlb_is_2M   = (!dtlb_lu_hit) ? l2_tlb_is_2M   : dtlb_is_2M;
    assign f_dtlb_is_1G   = (!dtlb_lu_hit) ? l2_tlb_is_1G   : dtlb_is_1G;   
    assign f_dtlb_lu_hit  = (!dtlb_lu_hit) ? l2_tlb_lu_hit  : dtlb_lu_hit;
    
    ptw  #(
        .ASID_WIDTH             ( ASID_WIDTH            )
    ) i_ptw (
        .clk_i                  ( clk_i                 ),
        .rst_ni                 ( rst_ni                ),
        .ptw_active_o           ( ptw_active            ),
        .walking_instr_o        ( walking_instr         ),
        .ptw_error_o            ( ptw_error             ),
        .enable_translation_i   ( enable_translation_i  ),

        .update_vaddr_o         ( update_vaddr          ),
        .itlb_update_o          ( update_ptw_itlb       ),
        .dtlb_update_o          ( update_ptw_dtlb       ),

        .itlb_access_i          ( itlb_lu_access        ),
        .itlb_hit_i             ( itlb_lu_hit           ),
        .itlb_vaddr_i           ( icache_areq_i.fetch_vaddr         ),

        .dtlb_access_i          ( dtlb_lu_access        ),
        .dtlb_hit_i             ( dtlb_lu_hit           ),
        .dtlb_vaddr_i           ( lsu_vaddr_i           ),

        .all_tlbs_checked_i     (all_hashes_checked     ),

        .req_port_i            ( req_port_i             ),
        .req_port_o            ( req_port_o             ),

        .*
     );

    // ila_1 i_ila_1 (
    //     .clk(clk_i), // input wire clk
    //     .probe0({req_port_o.address_tag, req_port_o.address_index}),
    //     .probe1(req_port_o.data_req), // input wire [63:0]  probe1
    //     .probe2(req_port_i.data_gnt), // input wire [0:0]  probe2
    //     .probe3(req_port_i.data_rdata), // input wire [0:0]  probe3
    //     .probe4(req_port_i.data_rvalid), // input wire [0:0]  probe4
    //     .probe5(ptw_error), // input wire [1:0]  probe5
    //     .probe6(update_vaddr), // input wire [0:0]  probe6
    //     .probe7(update_ptw_itlb.valid), // input wire [0:0]  probe7
    //     .probe8(update_ptw_dtlb.valid), // input wire [0:0]  probe8
    //     .probe9(dtlb_lu_access), // input wire [0:0]  probe9
    //     .probe10(lsu_vaddr_i), // input wire [0:0]  probe10
    //     .probe11(dtlb_lu_hit), // input wire [0:0]  probe11
    //     .probe12(itlb_lu_access), // input wire [0:0]  probe12
    //     .probe13(icache_areq_i.fetch_vaddr), // input wire [0:0]  probe13
    //     .probe14(itlb_lu_hit) // input wire [0:0]  probe13
    // );

    //-----------------------
    // Instruction Interface
    //-----------------------
    logic match_any_execute_region;
    // The instruction interface is a simple request response interface
    always_comb begin : instr_interface
        // MMU disabled: just pass through
        icache_areq_o.fetch_valid  = icache_areq_i.fetch_req;
        icache_areq_o.fetch_paddr  = icache_areq_i.fetch_vaddr[riscv::PLEN-1:0]; // play through in case we disabled address translation
        // two potential exception sources:
        // 1. HPTW threw an exception -> signal with a page fault exception
        // 2. We got an access error because of insufficient permissions -> throw an access exception
        icache_areq_o.fetch_exception      = '0;
        // Check whether we are allowed to access this memory region from a fetch perspective
        iaccess_err   = icache_areq_i.fetch_req && (((priv_lvl_i == riscv::PRIV_LVL_U) && ~f_itlb_content.u)
                                                 || ((priv_lvl_i == riscv::PRIV_LVL_S) && f_itlb_content.u));

        // MMU enabled: address from TLB, request delayed until hit. Error when TLB
        // hit and no access right or TLB hit and translated address not valid (e.g.
        // AXI decode error), or when PTW performs walk due to ITLB miss and raises
        // an error.
        if (enable_translation_i) begin
            // we work with SV39, so if VM is enabled, check that all bits [riscv::VLEN-1:38] are equal
            if (icache_areq_i.fetch_req && !((&icache_areq_i.fetch_vaddr[riscv::VLEN-1:38]) == 1'b1 || (|icache_areq_i.fetch_vaddr[riscv::VLEN-1:38]) == 1'b0)) begin
                icache_areq_o.fetch_exception = {riscv::INSTR_ACCESS_FAULT, {{64-riscv::VLEN{1'b0}}, icache_areq_i.fetch_vaddr}, 1'b1};
            end

            icache_areq_o.fetch_valid = 1'b0;

            // 4K page
            icache_areq_o.fetch_paddr = {f_itlb_content.ppn, icache_areq_i.fetch_vaddr[11:0]};
            // Mega page
            if (itlb_is_2M) begin
                icache_areq_o.fetch_paddr[20:12] = icache_areq_i.fetch_vaddr[20:12];
            end
            // Giga page
            if (itlb_is_1G) begin
                icache_areq_o.fetch_paddr[29:12] = icache_areq_i.fetch_vaddr[29:12];
            end

            // ---------
            // ITLB Hit
            // --------
            // if we hit the ITLB output the request signal immediately
            if (itlb_lu_hit) begin
                icache_areq_o.fetch_valid = icache_areq_i.fetch_req;
                // we got an access error
                if (iaccess_err) begin
                    // throw a page fault
                    icache_areq_o.fetch_exception = {riscv::INSTR_PAGE_FAULT, {{64-riscv::VLEN{1'b0}}, icache_areq_i.fetch_vaddr}, 1'b1};
                end
            end else
            // ---------
            // ITLB Miss
            // ---------
            // watch out for exceptions happening during walking the page table
            if (ptw_active && walking_instr) begin
                icache_areq_o.fetch_valid = ptw_error;
                icache_areq_o.fetch_exception = {riscv::INSTR_PAGE_FAULT, {{64-riscv::VLEN{1'b0}}, update_vaddr}, 1'b1};
            end
        end
        // if it didn't match any execute region throw an `Instruction Access Fault`
        if (!match_any_execute_region) begin
          icache_areq_o.fetch_exception = {riscv::INSTR_ACCESS_FAULT, {{64-riscv::PLEN{1'b0}}, icache_areq_o.fetch_paddr}, 1'b1};
        end
    end

    // check for execute flag on memory
    assign match_any_execute_region = ariane_pkg::is_inside_execute_regions(ArianeCfg, {{64-riscv::PLEN{1'b0}}, icache_areq_o.fetch_paddr});

    //-----------------------
    // Data Interface
    //-----------------------
    logic [riscv::VLEN-1:0] lsu_vaddr_n,     lsu_vaddr_q;
    riscv::pte_t dtlb_pte_n,      dtlb_pte_q;
    exception_t  misaligned_ex_n, misaligned_ex_q;
    logic        lsu_req_n,       lsu_req_q;
    logic        lsu_is_store_n,  lsu_is_store_q;
    logic        dtlb_hit_n,      dtlb_hit_q;
    logic        dtlb_is_2M_n,    dtlb_is_2M_q;
    logic        dtlb_is_1G_n,    dtlb_is_1G_q;

    // check if we need to do translation or if we are always ready (e.g.: we are not translating anything)
    assign lsu_dtlb_hit_o = (en_ld_st_translation_i) ? dtlb_lu_hit :  1'b1;

    // The data interface is simpler and only consists of a request/response interface
    always_comb begin : data_interface
        // save request and DTLB response
        lsu_vaddr_n           = lsu_vaddr_i;
        lsu_req_n             = lsu_req_i;
        misaligned_ex_n       = misaligned_ex_i;
        dtlb_pte_n            = f_dtlb_content;
        dtlb_hit_n            = f_dtlb_lu_hit;
        lsu_is_store_n        = lsu_is_store_i;
        dtlb_is_2M_n          = f_dtlb_is_2M;
        dtlb_is_1G_n          = f_dtlb_is_1G;

        lsu_paddr_o           = lsu_vaddr_q[riscv::PLEN-1:0];
        lsu_valid_o           = lsu_req_q;
        lsu_exception_o       = misaligned_ex_q;
        // mute misaligned exceptions if there is no request otherwise they will throw accidental exceptions
        misaligned_ex_n.valid = misaligned_ex_i.valid & lsu_req_i;

        // Check if the User flag is set, then we may only access it in supervisor mode
        // if SUM is enabled
        daccess_err = (ld_st_priv_lvl_i == riscv::PRIV_LVL_S && !sum_i && dtlb_pte_q.u) || // SUM is not set and we are trying to access a user page in supervisor mode
                      (ld_st_priv_lvl_i == riscv::PRIV_LVL_U && !dtlb_pte_q.u);            // this is not a user page but we are in user mode and trying to access it
        // translation is enabled and no misaligned exception occurred
        if (en_ld_st_translation_i && !misaligned_ex_q.valid) begin
            lsu_valid_o = 1'b0;
            // 4K page
            lsu_paddr_o = {dtlb_pte_q.ppn, lsu_vaddr_q[11:0]};
            // Mega page
            if (dtlb_is_2M_q) begin
              lsu_paddr_o[20:12] = lsu_vaddr_q[20:12];
            end
            // Giga page
            if (dtlb_is_1G_q) begin
                lsu_paddr_o[29:12] = lsu_vaddr_q[29:12];
            end
            // ---------
            // DTLB Hit
            // --------
            if (dtlb_hit_q && lsu_req_q) begin
                lsu_valid_o = 1'b1;
                // this is a store
                if (lsu_is_store_q) begin
                    // check if the page is write-able and we are not violating privileges
                    // also check if the dirty flag is set
                    if (!dtlb_pte_q.w || daccess_err || !dtlb_pte_q.d) begin
                        lsu_exception_o = {riscv::STORE_PAGE_FAULT, {{64-riscv::VLEN{lsu_vaddr_q[riscv::VLEN-1]}},lsu_vaddr_q}, 1'b1};
                    end

                // this is a load, check for sufficient access privileges - throw a page fault if necessary
                end else if (daccess_err) begin
                    lsu_exception_o = {riscv::LOAD_PAGE_FAULT, {{64-riscv::VLEN{lsu_vaddr_q[riscv::VLEN-1]}},lsu_vaddr_q}, 1'b1};
                end
            end else

            // ---------
            // DTLB Miss
            // ---------
            // watch out for exceptions
            if (ptw_active && !walking_instr) begin
                // page table walker threw an exception
                if (ptw_error) begin
                    // an error makes the translation valid
                    lsu_valid_o = 1'b1;
                    // the page table walker can only throw page faults
                    if (lsu_is_store_q) begin
                        lsu_exception_o = {riscv::STORE_PAGE_FAULT, {{64-riscv::VLEN{lsu_vaddr_q[riscv::VLEN-1]}},update_vaddr}, 1'b1};
                    end else begin
                        lsu_exception_o = {riscv::LOAD_PAGE_FAULT, {{64-riscv::VLEN{lsu_vaddr_q[riscv::VLEN-1]}},update_vaddr}, 1'b1};
                    end
                end
            end
        end
    end
    // ----------
    // Registers
    // ----------
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
            lsu_vaddr_q      <= '0;
            lsu_req_q        <= '0;
            misaligned_ex_q  <= '0;
            dtlb_pte_q       <= '0;
            dtlb_hit_q       <= '0;
            lsu_is_store_q   <= '0;
            dtlb_is_2M_q     <= '0;
            dtlb_is_1G_q     <= '0;
        end else begin
            lsu_vaddr_q      <=  lsu_vaddr_n;
            lsu_req_q        <=  lsu_req_n;
            misaligned_ex_q  <=  misaligned_ex_n;
            dtlb_pte_q       <=  dtlb_pte_n;
            dtlb_hit_q       <=  dtlb_hit_n;
            lsu_is_store_q   <=  lsu_is_store_n;
            dtlb_is_2M_q     <=  dtlb_is_2M_n;
            dtlb_is_1G_q     <=  dtlb_is_1G_n;
        end
    end
endmodule
