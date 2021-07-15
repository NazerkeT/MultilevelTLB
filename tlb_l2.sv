`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/23/2021 02:44:00 PM
// Design Name: 
// Module Name: tlb_l2
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

module tlb_l2 import ariane_pkg::*; #(
      parameter int unsigned TLB_SETS    = 8,
      parameter int unsigned TLB_WAYS    = 2,
      parameter int unsigned ASID_WIDTH  = 1,
      parameter int unsigned PAGE_LEVELS = 3
  )(
    input  logic                    clk_i,    // Clock
    input  logic                    rst_ni,   // Asynchronous reset active low
    input  logic                    flush_i,  // Flush signal
    // Update TLB
    input  tlb_update_t             update_i,
    // Lookup signals
    input  logic                    lu_access_i,
    input  logic [ASID_WIDTH-1:0]   lu_asid_i,
    input  logic [riscv::VLEN-1:0]  lu_vaddr_i,
    output riscv::pte_t             tlb_content_o,
    input  logic [ASID_WIDTH-1:0]   asid_to_be_flushed_i,
    input  logic [riscv::VLEN-1:0]  vaddr_to_be_flushed_i,
    output logic                    tlb_is_2M_o,
    output logic                    tlb_is_1G_o,
    output logic                    tlb_hit_o,
    output logic                    all_hashes_checked_o
);
    
    `define K $clog2(TLB_SETS)
    `define ALL_ENTRIES (TLB_SETS * TLB_WAYS)

    // SV39 defines three levels of page tables
    struct packed {
      logic [ASID_WIDTH-1:0] asid;
      logic [riscv::VPN2:0]  vpn2;
      logic [8:0]            vpn1;
      logic [8:0]            vpn0;
      logic                  is_2M;
      logic                  is_1G;
      logic                  valid;
    } [TLB_SETS-1:0][TLB_WAYS-1:0] tags_q, tags_n;
    
    riscv::pte_t [TLB_SETS-1:0][TLB_WAYS-1:0] content_q, content_n;
    
    logic [2:0][8:0] vpn;   // assumption of SV39 mode and VLEN = 64 as a default
    logic [TLB_SETS-1:0][TLB_WAYS-1:0] lu_hit;     
    logic [TLB_SETS-1:0][TLB_WAYS-1:0] replace_en; 
       
    //-------------
    // Translation
    //-------------
    // these help to transfer input to logic in a single cycle
    assign vpn[0] = lu_vaddr_i[20:12];
    assign vpn[1] = lu_vaddr_i[29:21];
    assign vpn[2] = lu_vaddr_i[38:30];
    
    logic [`K-1:0] ind;                     // set index
    logic [1:0]    hash_ord_q, hash_ord_n;  // hash-rehash order
    logic          hit_flag;                // notifies order counter about hit
    
    assign ind = vpn[hash_ord_q][`K-1:0]; 
    
    always_comb begin : translation        
        // default assignment
        lu_hit        = '{default: 0};
        tlb_hit_o     = 1'b0;
        tlb_content_o = '{default: 0};
        tlb_is_1G_o   = 1'b0;
        tlb_is_2M_o   = 1'b0;
        all_hashes_checked_o = 1'b0;
        
        hash_ord_n = hash_ord_q;
        hit_flag   = 1'b0;
                  
        if (lu_access_i) begin
            for (int unsigned i = 0; i < TLB_WAYS; i++) begin                             
                if (tags_q[ind][i].valid && ((lu_asid_i == tags_q[ind][i].asid) || content_q[ind][i].g) && vpn[2][8:`K] == tags_q[ind][i].vpn2[8:`K]) begin
                    if (tags_q[ind][i].is_1G) begin
                        tlb_is_1G_o   = 1'b1;
                        tlb_content_o = content_q[ind][i];
                        tlb_hit_o     = 1'b1;
                        lu_hit[ind][i] = 1'b1;
                        
                        hit_flag = 1'b1;
                    // not a giga page hit so check further
                    end else if (vpn[1][8:`K] == tags_q[ind][i].vpn1[8:`K]) begin
                        // this could be a 2 mega page hit or a 4 kB hit
                        // output accordingly
                        if (tags_q[ind][i].is_2M || vpn[0][8:`K] == tags_q[ind][i].vpn0[8:`K]) begin
                            tlb_is_2M_o   = tags_q[ind][i].is_2M;
                            tlb_content_o = content_q[ind][i];
                            tlb_hit_o     = 1'b1;
                            lu_hit[ind][i] = 1'b1;
                            
                            hit_flag = 1'b1;
                        end
                    end
                end
            end
            
            // avoid overflow for the last miss
            if (hit_flag) begin
                hash_ord_n = 1'b0;
                all_hashes_checked_o = 1'b1; // validate hash check ahead of time if tlb hits
            end else begin
                if (hash_ord_q < 'd2) begin
                    hash_ord_n += 1;
                end else begin
                    hash_ord_n = 1'b0;
                    all_hashes_checked_o = 1'b1; // set once all hashes are checked
                end
            end            
        end
                      
    end
    
    logic asid_to_be_flushed_is0;   // indicates that the ASID provided by SFENCE.VMA (rs2)is 0, active high
    logic vaddr_to_be_flushed_is0;  // indicates that the VADDR provided by SFENCE.VMA (rs1)is 0, active high
    logic [TLB_SETS-1:0][TLB_WAYS-1:0] vaddr_vpn0_match, vaddr_vpn1_match,vaddr_vpn2_match; 
    
    assign asid_to_be_flushed_is0  = ~(|asid_to_be_flushed_i);
    assign vaddr_to_be_flushed_is0 = ~(|vaddr_to_be_flushed_i);
    
    // ------------------
    // Update and Flush
    // ------------------
              
    always_comb begin : update_flush
        tags_n    = tags_q;
        content_n = content_q;
            
        // traversing all entries is inevitable, because of 4 possible flushes
        for (int unsigned i = 0; i < TLB_SETS; i++) begin
                for (int unsigned j = 0; j < TLB_WAYS; j++) begin
            
                    vaddr_vpn0_match[i][j] = (vaddr_to_be_flushed_i[20:12] == tags_q[i][j].vpn0);
                    vaddr_vpn1_match[i][j] = (vaddr_to_be_flushed_i[29:21] == tags_q[i][j].vpn1);
                    vaddr_vpn2_match[i][j] = (vaddr_to_be_flushed_i[30+riscv::VPN2:30] == tags_q[i][j].vpn2);
            
                        if (flush_i) begin
                            // invalidate logic
                            // flush everything if ASID is 0 and vaddr is 0 ("SFENCE.VMA x0 x0" case)
                            if (asid_to_be_flushed_is0 && vaddr_to_be_flushed_is0 )
                                tags_n[i][j].valid = 1'b0;
                            // flush vaddr in all addressing space ("SFENCE.VMA vaddr x0" case), it should happen only for leaf pages
                            else if (asid_to_be_flushed_is0 && ((vaddr_vpn0_match[i][j] && vaddr_vpn1_match[i][j] && vaddr_vpn2_match[i][j]) || (vaddr_vpn2_match[i][j] && tags_q[i][j].is_1G) || (vaddr_vpn1_match[i][j] && vaddr_vpn2_match[i][j] && tags_q[i][j].is_2M) ) && (~vaddr_to_be_flushed_is0))
                                tags_n[i][j].valid = 1'b0;
                            // the entry is flushed if it's not global and asid and vaddr both matches with the entry to be flushed ("SFENCE.VMA vaddr asid" case)
                            else if ((!content_q[i][j].g) && ((vaddr_vpn0_match[i][j] && vaddr_vpn1_match[i][j] && vaddr_vpn2_match[i][j]) || (vaddr_vpn2_match[i][j] && tags_q[i][j].is_1G) || (vaddr_vpn1_match[i][j] && vaddr_vpn2_match[i][j] && tags_q[i][j].is_2M)) && (asid_to_be_flushed_i == tags_q[i][j].asid) && (!vaddr_to_be_flushed_is0) && (!asid_to_be_flushed_is0))
                                tags_n[i][j].valid = 1'b0;
                            // the entry is flushed if it's not global, and the asid matches and vaddr is 0. ("SFENCE.VMA 0 asid" case)
                            else if ((!content_q[i][j].g) && (vaddr_to_be_flushed_is0) && (asid_to_be_flushed_i == tags_q[i][j].asid) && (!asid_to_be_flushed_is0))
                                tags_n[i][j].valid = 1'b0;
                        // normal replacement
                        // compare vpns for all possible set indices with respect to page size
                        end else if (update_i.valid && ((update_i.is_1G && (i == update_i.vpn[17+`K:18])) || (update_i.is_2M && (i == update_i.vpn[8+`K:9])) || (!update_i.is_1G && !update_i.is_2M && (i == update_i.vpn[`K-1:0]))) && replace_en[i][j]) begin
                            // update tag array
                            tags_n[i][j] = '{
                                asid:  update_i.asid,
                                vpn2:  update_i.vpn [18+riscv::VPN2:18],
                                vpn1:  update_i.vpn [17:9],
                                vpn0:  update_i.vpn [8:0],
                                is_1G: update_i.is_1G,
                                is_2M: update_i.is_2M,
                                valid: 1'b1
                            };
                            // and content as well
                            content_n[i][j] = update_i.content;
                        end
                    end
            end  
        end
        
    // -----------------------------------------------
    // PLRU - Pseudo Least Recently Used Replacement
    // -----------------------------------------------
    logic [TLB_SETS-1:0][TLB_WAYS-2:0] plru_tree_q, plru_tree_n;
    
    always_comb begin : plru_replacement
            plru_tree_n = plru_tree_q;
            // The PLRU-tree indexing:
            // lvl0        0
            //            / \
            //           /   \
            // lvl1     1     2
            //         / \   / \
            // lvl2   3   4 5   6
            //       / \ /\/\  /\
            //      ... ... ... ...
            // Just predefine which nodes will be set/cleared
            // E.g. for a TLB with 8 entries, the for-loop is semantically
            // equivalent to the following pseudo-code:
            // unique case (1'b1)
            // lu_hit[7]: plru_tree_n[0, 2, 6] = {1, 1, 1};
            // lu_hit[6]: plru_tree_n[0, 2, 6] = {1, 1, 0};
            // lu_hit[5]: plru_tree_n[0, 2, 5] = {1, 0, 1};
            // lu_hit[4]: plru_tree_n[0, 2, 5] = {1, 0, 0};
            // lu_hit[3]: plru_tree_n[0, 1, 4] = {0, 1, 1};
            // lu_hit[2]: plru_tree_n[0, 1, 4] = {0, 1, 0};
            // lu_hit[1]: plru_tree_n[0, 1, 3] = {0, 0, 1};
            // lu_hit[0]: plru_tree_n[0, 1, 3] = {0, 0, 0};
            // default: begin /* No hit */ end
            // endcase
            for (int unsigned i = 0; i < TLB_WAYS; i++) begin
                automatic int unsigned idx_base, shift, new_index;
                // we got a hit so update the pointer as it was least recently used
                if (lu_hit[ind][i] & all_hashes_checked_o) begin
                    // Set the nodes to the values we would expect
                    for (int unsigned lvl = 0; lvl < $clog2(TLB_WAYS); lvl++) begin
                      idx_base = $unsigned((2**lvl)-1);
                      // lvl0 <=> MSB, lvl1 <=> MSB-1, ...
                      shift = $clog2(TLB_WAYS) - lvl;
                      // to circumvent the 32 bit integer arithmetic assignment
                      new_index =  ~((i >> (shift-1)) & 32'b1);
                      plru_tree_n[ind][idx_base + (i >> shift)] = new_index[0];
                    end
                end
            end
            // Decode tree to write enable signals
            // Next for-loop basically creates the following logic for e.g. an 8 entry
            // TLB (note: pseudo-code obviously):
            // replace_en[7] = &plru_tree_q[ 6, 2, 0]; //plru_tree_q[0,2,6]=={1,1,1}
            // replace_en[6] = &plru_tree_q[~6, 2, 0]; //plru_tree_q[0,2,6]=={1,1,0}
            // replace_en[5] = &plru_tree_q[ 5,~2, 0]; //plru_tree_q[0,2,5]=={1,0,1}
            // replace_en[4] = &plru_tree_q[~5,~2, 0]; //plru_tree_q[0,2,5]=={1,0,0}
            // replace_en[3] = &plru_tree_q[ 4, 1,~0]; //plru_tree_q[0,1,4]=={0,1,1}
            // replace_en[2] = &plru_tree_q[~4, 1,~0]; //plru_tree_q[0,1,4]=={0,1,0}
            // replace_en[1] = &plru_tree_q[ 3,~1,~0]; //plru_tree_q[0,1,3]=={0,0,1}
            // replace_en[0] = &plru_tree_q[~3,~1,~0]; //plru_tree_q[0,1,3]=={0,0,0}
            // For each entry traverse the tree. If every tree-node matches,
            // the corresponding bit of the entry's index, this is
            // the next entry to replace.
            
            // This is a bit awful, but helps to keep replace enable result in the same cycle with plru tree update
            for (int unsigned i = 0; i < TLB_SETS; i++) begin
                for (int unsigned j = 0; j < TLB_WAYS; j ++) begin
                    automatic logic en;
                    automatic int unsigned idx_base, shift, new_index;
                    en = 1'b1;
                    for (int unsigned lvl = 0; lvl < $clog2(TLB_WAYS); lvl++) begin
                        idx_base = $unsigned((2**lvl)-1);
                        // lvl0 <=> MSB, lvl1 <=> MSB-1, ...
                        shift = $clog2(TLB_WAYS) - lvl;
        
                        // en &= plru_tree_q[idx_base + (i>>shift)] == ((i >> (shift-1)) & 1'b1);
                        new_index =  (j >> (shift-1)) & 32'b1;
                        if (new_index[0]) begin
                          en &= plru_tree_q[i][idx_base + (j >> shift)];
                        end else begin
                          en &= ~plru_tree_q[i][idx_base + (j >> shift)];
                        end
                    end
                    replace_en[i][j] = en;
                end
            end
            
        end
        
    // sequential process
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if(~rst_ni) begin
            tags_q      <= '{default: 0}; 
            content_q   <= '{default: 0};
            plru_tree_q <= '{default: 0};
            hash_ord_q  <= 1'b0;
        end else begin
            tags_q      <= tags_n;
            content_q   <= content_n;
            plru_tree_q <= plru_tree_n;
            hash_ord_q  <= hash_ord_n;
        end
    end
    
//    //--------------
//    // Sanity checks
//    //--------------
    
//    //pragma translate_off
//    `ifndef VERILATOR
    
//    initial begin : p_assertions
//        assert ((TLB_SETS % 2 == 0) && (TLB_SETS > 1))
//            else begin $error("TLB SETS must be a multiple of 2 and greater than 1"); $stop(); end
//        assert ((TLB_WAYS % 2 == 0) && (TLB_WAYS > 1))
//            else begin $error("TLB WAYS must be a multiple of 2 and greater than 1"); $stop(); end
//        assert (ASID_WIDTH >= 1)
//            else begin $error("ASID width must be at least 1"); $stop(); end
//    end
        
//    // Just for checking
//    function int countSetBits(logic[TLB_SETS-1:0][TLB_WAYS-1:0] vector);
//        automatic int count = 0;
//        foreach (vector[i, j]) begin
//            count += vector[i][j];
//        end
//        return count;
//    endfunction
    
//    assert property (@(posedge clk_i)(countSetBits(lu_hit) <= 1))
//        else begin $error("More then one hit in TLB!"); $stop(); end
//    assert property (@(posedge clk_i)(countSetBits(replace_en) <= 1))
//        else begin $error("More then one TLB entry selected for next replace!"); $stop(); end
    
//    `endif
//    //pragma translate_on
    
endmodule
