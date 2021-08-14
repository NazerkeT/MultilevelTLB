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
// Author: 
// Date: 06.08.2021
// Description: testbench for tlb. Includes the following tests:
// ...TBD...
//


`include "tb.svh"

module tb import tb_pkg::*; import riscv::*; import ariane_pkg::*; import wt_cache_pkg::*; #()();

  // leave this
  timeunit 1ps;
  timeprecision 1ps; 

  // memory configuration (64bit words)
  parameter MemBytes          = 2**DCACHE_INDEX_WIDTH * 4 * 32;
  parameter MemWords          = MemBytes>>3;
  parameter ariane_pkg::ariane_cfg_t ArianeCfg = ariane_pkg::ArianeDefaultConfig;
  
  // contention and invalidation rates (in %)
  parameter MemRandHitRate   = 75;
  parameter MemRandInvRate   = 10;
  parameter TlbHitRate       = 95;

  // parameters for random read sequences (in %)    
  parameter FlushRate         = 10;
  parameter KillRate          = 5;

  parameter Verbose           = 0;

  // number of vectors per test
  parameter nReadVectors      = 20000;

///////////////////////////////////////////////////////////////////////////////
// DUT signal declarations
///////////////////////////////////////////////////////////////////////////////

  logic                            flush;
  logic                            enable_translation;
  logic                            en_ld_st_translation;   // enable virtual memory translation for load/stores
  // IF interface
  icache_areq_o_t                  icache_req_i;
  icache_areq_i_t                  icache_req_o;
  // LSU interface
  exception_t                      misaligned_ex;
  logic                            lsu_req;        // request address translation
  logic [riscv::VLEN-1:0]          lsu_vaddr;      // virtual address in
  logic                            lsu_is_store;   // the translation is requested by a store
  // if we need to walk the page table we can't grant in the same cycle
  // Cycle 0 
  logic                            lsu_dtlb_hit;   // sent in the same cycle as the request if translation hits in the DTLB
  logic [riscv::PPNW-1:0]          lsu_dtlb_ppn;   // ppn (send same cycle as hit)
  // Cycle 1
  logic                            all_tlbs_checked; // min - in the same cycle, max - in three cycles
  logic                            lsu_valid;        // translation is valid
  logic [riscv::PLEN-1:0]          lsu_paddr;        // translated address
  exception_t                      lsu_exception;    // address translation threw an exception
  // General control signals
  priv_lvl_t                       priv_lvl;
  priv_lvl_t                       ld_st_priv_lvl;
  logic                            sum;
  logic                            mxr;
  // input logic flag_mprv_i,
  logic [riscv::PPNW-1:0]          satp_ppn;
  logic [ASID_WIDTH-1:0]           asid;
  logic [ASID_WIDTH-1:0]           asid_to_be_flushed;
  logic [riscv::VLEN-1:0]          vaddr_to_be_flushed;
  logic                            flush_tlb;
  // PTW memory interface
  dcache_req_o_t                   dut_req_port_o;
  dcache_req_i_t                   dut_req_port_i;

///////////////////////////////////////////////////////////////////////////////
// TB signal declarations
///////////////////////////////////////////////////////////////////////////////

  // TB stim signals
  string test_name;
  logic clk_i, rst_ni;
  logic [31:0] seq_num_resp;
  seq_t [2:0] seq_type;
  logic [2:0] seq_done;
  logic [6:0] req_rate[2:0];
  logic seq_run, seq_last;
  logic end_of_sim;

  logic mem_rand_en;
  logic inv_rand_en;
  logic tlb_rand_en;
  
  logic flush_rand_en; 
    
  logic check_en;
  
  // TB Mem interface
  icache_areq_i_t         mem_icache_req;
  logic                   mem_lsu_valid;
  logic [riscv::PLEN-1:0] mem_lsu_paddr;
  exception_t             mem_lsu_except;
        
  //TB Port interface
  logic [riscv::VLEN-1:0] act_vaddr[1:0];
  logic [riscv::PPNW-1:0] exp_ppn[1:0];
  logic [riscv::VLEN-1:0] exp_vaddr[1:0];
  logic [riscv::PPNW-1:0] fifo_data_in[1:0];
  logic [riscv::PPNW-1:0] fifo_data[1:0];
  logic [1:0]  fifo_push, fifo_pop, fifo_flush;

///////////////////////////////////////////////////////////////////////////////
// helper tasks
///////////////////////////////////////////////////////////////////////////////

  task automatic runSeq(input int nReadVectors, input logic last =1'b0);
    seq_last      = last;
    seq_run       = 1'b1;
    seq_num_resp  = nReadVectors;
    `APPL_WAIT_CYC(clk_i,1)
    seq_run      = 1'b0;
    `APPL_WAIT_SIG(clk_i, &seq_done)
    `APPL_WAIT_CYC(clk_i,1)
  endtask : runSeq

  task automatic flushCache();
    flush      = 1'b1;
    `APPL_WAIT_CYC(clk_i,1)
    flush      = 1'b0;
    `APPL_WAIT_CYC(clk_i,1)
  endtask : flushCache

  task automatic memCheck();
    check_en     = 1'b1;
    `APPL_WAIT_CYC(clk_i,1)
    check_en     = 1'b0;
    `APPL_WAIT_CYC(clk_i,1)
  endtask : memCheck


///////////////////////////////////////////////////////////////////////////////
// Clock Process
///////////////////////////////////////////////////////////////////////////////

  always @*
    begin
      do begin
        clk_i = 1;#(CLK_HI);
        clk_i = 0;#(CLK_LO);
      end while (end_of_sim == 1'b0);
      repeat (100) begin
        // generate a few extra cycle to allow
        // response acquisition to complete
        clk_i = 1;#(CLK_HI);
        clk_i = 0;#(CLK_LO);
      end
    end

///////////////////////////////////////////////////////////////////////////////
// memory emulation
///////////////////////////////////////////////////////////////////////////////

  tb_mem #(
    .MemRandHitRate ( MemRandHitRate ),
    .MemRandInvRate ( MemRandInvRate ),
    .MemWords       ( MemWords       )
  ) i_tb_mem (
    .clk_i          ( clk_i          ),
    .rst_ni         ( rst_ni         ),
    // control (in)validation
    .seq_type_i     ( seq_type[3]    ),
    // TB manip
    .mem_rand_en_i  ( mem_rand_en    ),
    .inv_rand_en_i  ( inv_rand_en    ),
    // DUT-MMU interface
    .dut_req_port_i ( dut_req_port_i ),
    .dut_req_port_o ( dut_req_port_o ),
    // verification inp
    .seq_last_i     ( seq_last       ),
    .check_en_i     ( check_en       ),
    .icache_areq_i  ( icache_req_i   ),
    .lsu_req_i      ( lsu_req        ),
    .lsu_vaddr_i    ( lsu_vaddr      ),
    .lsu_is_store_i ( lsu_is_store   ),
    // verification out
    .icache_areq_o  ( mem_icache_req ),
    .lsu_valid_o    ( mem_lsu_valid  ),
    .lsu_paddr_o    ( mem_lsu_paddr  ),
    .lsu_exception_o( mem_lsu_except )
  );

///////////////////////////////////////////////////////////////////////////////
// DUT
///////////////////////////////////////////////////////////////////////////////

  mmu #(
    .INSTR_TLB_ENTRIES      ( 16                     ),
    .DATA_TLB_ENTRIES       ( 16                     ),
    .ASID_WIDTH             ( ASID_WIDTH             ),
    .ArianeCfg              ( ArianeDefaultConfig    )
  ) i_dut (
    // misaligned bypass
    .clk_i,
    .rst_i,
    .flush_i                ( flush                ),
    .enable_translation_i   ( enable_translation   ),
    .en_ld_st_translation_i ( en_ld_st_translation ),
    // IF
    .icache_areq_i          ( icache_req_i     ),
    .icache_areq_o          ( icache_req_o     ),
    // LSU
    .misaligned_ex_i        ( misaligned_ex    ),
    .lsu_is_store_i         ( lsu_is_store     ),
    .lsu_req_i              ( lsu_req          ),
    .lsu_vaddr_i            ( lsu_vaddr        ),
    .lsu_valid_o            ( lsu_valid        ),
    .all_tlbs_checked_o     ( all_tlbs_checked ),
    .lsu_paddr_o            ( lsu_paddr        ),
    .lsu_exception_o        ( lsu_exception    ),
    .lsu_dtlb_hit_o         ( lsu_dtlb_hit     ), // send in the same cycle as the request
    .lsu_dtlb_ppn_o         ( lsu_dtlb_ppn     ), // send in the same cycle as the request
    // General control
    .priv_lvl_i             ( priv_lvl              ),      
    .ld_st_priv_lvl_i       ( ld_st_priv_lvl        ),
    .sum_i                  ( sum                   ),
    .mxr_i                  ( mxr                   ),
    // input logic flag_mprv_i,
    .satp_ppn_i             ( satp_ppn              ),
    .asid_i                 ( asid                  ),
    .asid_to_be_flushed_i   ( asid_to_be_flushed    ),
    .vaddr_to_be_flushed_i  ( vaddr_to_be_flushed   ),
    .flush_tlb_i            ( flush_tlb             ),
    // connecting PTW to D$ IF
    .req_port_i             ( dut_req_port_o        ),
    .req_port_o             ( dut_req_port_i        ),
        .*
  );

///////////////////////////////////////////////////////////////////////////////
// port emulation programs
///////////////////////////////////////////////////////////////////////////////

  // get actual paddr from read controllers
  assign act_vaddr[0] = i_dut.icache_areq_i.fetch_vaddr;
  assign act_vaddr[1] = i_dut.lsu_vaddr_i;

  // generate fifo queues for expected responses
  assign fifo_data_in[0] = icache_req_i.fetch_vaddr;
  assign fifo_data_in[1] = lsu_vaddr;
  
  assign fifo_push[0] = icache_req_i.fetch_req && icache_req_o.fetch_valid;
  assign fifo_push[1] = lsu_req && lsu_valid;
  
  assign fifo_pop[0] = icache_req_i.fetch_valid;
  assign fifo_pop[1] = lsu_valid;
  
  generate
    for(genvar k=0; k<2; k++) begin
      fifo_v3 #(
        .dtype(logic [riscv::VLEN-1:0])
      ) i_resp_fifo  (
        .clk_i       ( clk_i            ),
        .rst_ni      ( rst_ni           ),
        .flush_i     ( '0               ),
        .testmode_i  ( '0               ),
        .full_o      (                  ),
        .empty_o     (                  ),
        .usage_o     (                  ),
        .data_i      ( fifo_data_in[k]  ),
        .push_i      ( fifo_push[k]     ),
        .data_o      ( fifo_data[k]     ),
        .pop_i       ( fifo_pop[k]      )
      );
    end
  endgenerate

  tb_ifetchport #(
    .PortName      ( "$I-Port"     ),
    .FlushRate     ( FlushRate     ),
    .KillRate      ( KillRate      ),
    .TlbHitRate    ( TlbHitRate    ),
    .MemWords      ( MemWords      ),
    .RndSeed       ( 3333333       ),
    .Verbose       ( Verbose       )
  ) i_tb_ifetchport  (
    .clk_i           ( clk_i               ),
    .rst_ni          ( rst_ni              ),
    .test_name_i     ( test_name           ),
    .req_rate_i      ( req_rate[0]         ),
    .seq_type_i      ( seq_type[0]         ),
    .seq_run_i       ( seq_run             ),
    .seq_num_resp_i  ( seq_num_resp        ),
    .seq_last_i      ( seq_last            ),
    .seq_done_o      ( seq_done[0]         ),
    // Comparison interface
    .exp_vaddr_o     ( exp_vaddr[0]        ),
    .exp_vaddr_i     ( fifo_data[0]        ),
    .act_vaddr_i     ( act_vaddr[0]        ),
    .mem_req_i       ( mem_icache_req      ),
    // DUT-MMU interface
    .icache_areq_o   ( icache_req_i        ),
    .icache_areq_i   ( icache_req_o        )
  );

  tb_dfetchport #(
    .PortName      ( "$D-Port"     ),
    .FlushRate     ( FlushRate     ),
    .KillRate      ( KillRate      ),
    .TlbHitRate    ( TlbHitRate    ),
    .MemWords      ( MemWords      ),
    .RndSeed       ( 5555555       ),
    .Verbose       ( Verbose       )
  ) i_tb_dfetchport  (
    .clk_i           ( clk_i                ),
    .rst_ni          ( rst_ni               ),
    .test_name_i     ( test_name            ),
    .req_rate_i      ( req_rate[1]          ),
    .seq_type_i      ( seq_type[1]          ),
    .seq_run_i       ( seq_run              ),
    .seq_num_resp_i  ( seq_num_resp         ),
    .seq_last_i      ( seq_last             ),
    .seq_done_o      ( seq_done[1]          ),
    // Comparison interface
    .exp_vaddr_o     ( exp_vaddr[1]         ),
    .exp_vaddr_i     ( fifo_data[1]         ),
    .act_vaddr_i     ( act_vaddr[1]         ),
    .mem_valid_i     ( mem_lsu_valid        ),
    .mem_paddr_i     ( mem_lsu_paddr        ),
    .mem_exception_i ( mem_lsu_except       ),
    // DUT-MMU interface
    .misaligned_ex_o ( misaligned_ex        ),
    .lsu_req_o       ( lsu_req              ),
    .lsu_vaddr_o     ( lsu_vaddr            ),
    .lsu_is_store_o  ( lsu_is_store         ),
    .lsu_dtlb_hit_i  ( lsu_dtlb_hit         ),
    .lsu_dtlb_ppn_i  ( lsu_dtlb_ppn         ),
    .all_tlbs_checked_i  ( all_tlbs_checked ),  
    .lsu_valid_i         ( lsu_valid        ),
    .lsu_paddr_i         ( lsu_paddr        ),
    .lsu_exception_i     ( lsu_exception    )
    );
    
  tb_csrport #(
    .PortName      ( "CSR-Port"    ),
    .FlushRate     ( FlushRate     ),
    .KillRate      ( KillRate      ),
    .TlbHitRate    ( TlbHitRate    ),
    .MemWords      ( MemWords      ),
    .RndSeed       ( 7777777       ),
    .Verbose       ( Verbose       )
  ) i_tb_csrport     (
    .clk_i           ( clk_i                ),
    .rst_ni          ( rst_ni               ),
    .test_name_i     ( test_name            ),
    .req_rate_i      ( req_rate[2]          ),
    .seq_type_i      ( seq_type[2]          ),
    .flush_rand_en_i ( flush_rand_en        ),
    .seq_run_i       ( seq_run              ),
    .seq_num_resp_i  ( seq_num_resp         ),
    .seq_last_i      ( seq_last             ),
    .seq_done_o      ( seq_done[2]          ),
    // DUT-MMU interface
    .flush_o                ( flush                ),
    .enable_translation_o   ( enable_translation   ),
    .en_ld_st_translation_o ( en_ld_st_translation ),
    .lsu_dtlb_hit_i         ( lsu_dtlb_hit         ), // sent in the same cycle as the request if translation hits in the DTLB
    .all_tlbs_checked_i     ( all_tlbs_checked     ), // sent in min - the same cycle, max - three cycles
    // General control signals
    .priv_lvl_o             ( priv_lvl             ), 
    .ld_st_priv_lvl_o       ( ld_st_priv_lvl       ),
    .sum_o                  ( sum                  ),    
    .mxr_o                  ( mxr                  ),  
    // input logic      
    .satp_ppn_o             ( satp_ppn             ),
    .asid_o                 ( asid                 ),
    .asid_to_be_flushed_o   ( asid_to_be_flushed   ),
    .vaddr_to_be_flushed_o  ( vaddr_to_be_flushed  ),
    .flush_tlb_o            ( flush_tlb            ) 
  );

///////////////////////////////////////////////////////////////////////////////
// ----> simulation coordinator process
///////////////////////////////////////////////////////////////////////////////

// to-do: fix enable/disable signals for ports
//        fix seq_types
//        complete functions within a port

  initial begin : p_stim
    test_name        = "";
    seq_type         = '{default: RANDOM_SEQ};
    req_rate         = '{default: 7'd75};
    seq_run          = 1'b0;
    seq_last         = 1'b0;
    seq_num_resp     = '0;
    check_en         = '0;
    // seq_done
    end_of_sim       = 0;
    rst_ni           = 0;
    // randomization settings
    mem_rand_en      = 0;
    tlb_rand_en      = 0;
    inv_rand_en      = 0;
    flush_rand_en    = 0;
    // cache ctrl
    flush            = 0;
    enable_translation  = 0;

    // print some info
    $display("TB> current configuration:");
    $display("TB> MemWords        %d",   MemWords);
    $display("TB> MemRandHitRate  %d",   MemRandHitRate);
    $display("TB> MemRandInvRate  %d",   MemRandInvRate);

    // reset cycles
    `APPL_WAIT_CYC(clk_i,100)
    rst_ni        = 1'b1;
    `APPL_WAIT_CYC(clk_i,100)

    $display("TB> start with test sequences"); 
    // apply each test until seq_num_resp memory
    // requests have successfully completed
    ///////////////////////////////////////////////
    test_name    = "TEST 0 -- random i_access -- enabled translation";
    // config
    enable_translation     = 1;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 1 -- sequential i_access -- enabled translation";
    // config
    enable_translation     = 0;
    seq_type     = '{default: LINEAR_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 2 -- random d_access -- enabled translation";
    // config
    enable_translation     = 1;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 3 -- sequential d_access -- enabled translation";
    // config
    enable_translation     = 1;
    seq_type     = '{default: LINEAR_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 4 -- random i and d access -- enabled translation";
    // config
    enable_translation     = 1;
    tlb_rand_en  = 1;
    mem_rand_en  = 1;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 5 -- random i_access -- enabled translation, validation contentions";
    // config
    enable_translation     = 1;
    tlb_rand_en  = 1;
    mem_rand_en  = 1;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 6 -- sequential i_access -- enabled translation, validation contentions";
    // config
    enable_translation     = 1;
    inv_rand_en  = 1;
    seq_type     = '{default: LINEAR_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 7 -- random d_access -- enabled translation, validation contentions";
    // config
    enable_translation     = 0;
    inv_rand_en  = 1;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd25};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 8 -- sequential d_access -- enabled translation, validation contentions";
    // config
    enable_translation     = 1;
    inv_rand_en  = 1;
    seq_type     = '{default: LINEAR_SEQ};
    req_rate     = '{default: 7'd25};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 9 -- random i and d_access -- enabled translation, validation contentions";
    // config
    enable_translation     = 1;
    inv_rand_en  = 1;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd25};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 10 -- random i_access -- enabled translation, random flushes";
    // config
    enable_translation     = 1;
    inv_rand_en  = 0;
    flush_rand_en = 1;
    seq_type     = '{RANDOM_SEQ};
    req_rate     = '{100, 0, 0};
    runSeq(0);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 11 -- sequential i_access -- enabled translation, random flushes";
    // config
    enable_translation     = 1;
    inv_rand_en  = 0;
    flush_rand_en = 1;
    seq_type     = '{LINEAR_SEQ};
    req_rate     = '{default:100};
    seq_type     = '{LINEAR_SEQ, IDLE_SEQ, IDLE_SEQ};
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 12 -- random d_access -- enabled translation, random flushes";
    // config
    enable_translation     = 1;
    inv_rand_en  = 0;
    flush_rand_en = 1;
    seq_type     = '{RANDOM_SEQ};
    req_rate     = '{75, 0, 0};
    runSeq(0);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 13 -- sequential d_access -- enabled translation, random flushes";
    // config
    enable_translation     = 1;
    inv_rand_en  = 0;
    flush_rand_en = 1;
    seq_type     = '{LINEAR_SEQ};
    req_rate     = '{75, 0, 0};
    runSeq(0);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 14 -- random i and d access -- enabled translation, random flushes";
    // config
    enable_translation     = 1;
    inv_rand_en  = 0;
    flush_rand_en = 1;
    seq_type     = '{RANDOM_SEQ};
    req_rate     = '{default:25};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 15 -- random i_access -- enabled translation, random flushes, validation contentions";
    // config
    enable_translation     = 1;
    inv_rand_en  = 1;
    flush_rand_en = 1;
    seq_type     = '{RANDOM_SEQ};
    req_rate     = '{100,0,20};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 16 -- sequential i_access -- enabled translation, random flushes, validation contentions";
    // config
    enable_translation      = 1;
    inv_rand_en  = 1;
    flush_rand_en = 1;
    seq_type      = '{LINEAR_SEQ};
    req_rate      = '{default:25};
    runSeq(nReadVectors,1);// last sequence flag, terminates agents
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
     test_name    = "TEST 17 -- random d_access -- enabled translation, random flushes, validation contentions";
    // config
    enable_translation     = 1;
    inv_rand_en  = 1;
    flush_rand_en = 1;
    seq_type     = '{RANDOM_SEQ};
    req_rate     = '{default:25};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 18 -- sequential d_access -- enabled translation, random flushes, validation contentions";
    // config
    enable_translation     = 1;
    inv_rand_en  = 1;
    flush_rand_en = 1;
    seq_type     = '{LINEAR_SEQ};
    req_rate     = '{100,0,20};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 19 -- random i and d access -- enabled translation, random flushes, validation contentions";
    // config
    enable_translation      = 1;
    inv_rand_en  = 1;
    flush_rand_en = 1;
    seq_type      = '{RANDOM_SEQ};
    req_rate      = '{default:25};
    runSeq(nReadVectors,1);// last sequence flag, terminates agents
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    end_of_sim = 1;
    $display("TB> end test sequences");
  end


endmodule












