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

module tb import tb_pkg::*; import ariane_pkg::*; import wt_cache_pkg::*; #()();

  // leave this
  timeunit 1ps;
  timeprecision 1ps;

  // memory configuration (64bit words)
  parameter MemBytes          = 2**DCACHE_INDEX_WIDTH * 4 * 32;
  parameter MemWords          = MemBytes>>3;
  // noncacheable portion
  parameter logic [63:0] CachedAddrBeg = MemBytes>>3;//1/8th of the memory is NC
  parameter logic [63:0] CachedAddrEnd = 64'hFFFF_FFFF_FFFF_FFFF;

  localparam ariane_cfg_t ArianeDefaultConfig = '{
    RASDepth: 2,
    BTBEntries: 32,
    BHTEntries: 128,
    // idempotent region
    NrNonIdempotentRules:  0,
    NonIdempotentAddrBase: {64'b0},
    NonIdempotentLength:   {64'b0},
    // executable region
    NrExecuteRegionRules:  0,
    ExecuteRegionAddrBase: {64'h0},
    ExecuteRegionLength:   {64'h0},
    // cached region
    NrCachedRegionRules:   1,
    CachedRegionAddrBase:  {CachedAddrBeg},//1/8th of the memory is NC
    CachedRegionLength:    {CachedAddrEnd-CachedAddrBeg+64'b1},
    // cache config
    Axi64BitCompliant:     1'b1,
    SwapEndianess:         1'b0,
    // debug
    DmBaseAddress:         64'h0,
    NrPMPEntries:          0
  };

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
  parameter nWriteVectors     = 20000;

///////////////////////////////////////////////////////////////////////////////
// DUT signal declarations
///////////////////////////////////////////////////////////////////////////////

  logic                            flush_i,
  logic                            enable_translation_i,
  logic                            en_ld_st_translation_i,   // enable virtual memory translation for load/stores
  // IF interface
  icache_areq_o_t                  icache_areq_i,
  icache_areq_i_t                  icache_areq_o,
  // LSU interface
  exception_t                      misaligned_ex_i,
  logic                            lsu_req_i,        // request address translation
  logic [riscv::VLEN-1:0]          lsu_vaddr_i,      // virtual address in
  logic                            lsu_is_store_i,   // the translation is requested by a store
  // if we need to walk the page table we can't grant in the same cycle
  // Cycle 0 
  logic                            lsu_dtlb_hit_o,   // sent in the same cycle as the request if translation hits in the DTLB
  logic [riscv::PPNW-1:0]          lsu_dtlb_ppn_o,   // ppn (send same cycle as hit)
  // Cycle 1
  logic                            lsu_valid_o,      // translation is valid
  logic [riscv::PLEN-1:0]          lsu_paddr_o,      // translated address
  exception_t                      lsu_exception_o,  // address translation threw an exception
  // General control signals
  riscv::priv_lvl_t                 priv_lvl_i,
  riscv::priv_lvl_t                 ld_st_priv_lvl_i,
  logic                             sum_i,
  logic                             mxr_i,
  // input logic flag_mprv_i,
  logic [riscv::PPNW-1:0]           satp_ppn_i,
  logic [ASID_WIDTH-1:0]            asid_i,
  logic [ASID_WIDTH-1:0]            asid_to_be_flushed_i,
  logic [riscv::VLEN-1:0]           vaddr_to_be_flushed_i,
  logic                             flush_tlb_i,
  // Performance counters
  logic                            itlb_miss_o,
  logic                            dtlb_miss_o,
  // PTW memory interface
  dcache_req_o_t                   req_port_i,
  dcache_req_i_t                   req_port_o,
  // PMP
  riscv::pmpcfg_t [15:0]           pmpcfg_i,
  logic [15:0][riscv::PLEN-3:0]    pmpaddr_i

///////////////////////////////////////////////////////////////////////////////
// ----> TB signal declarations
///////////////////////////////////////////////////////////////////////////////

  logic [63:0] mem_array[MemWords-1:0];

  string test_name;
  logic clk_i, rst_ni;
  logic [31:0] seq_num_resp, seq_num_write;
  seq_t [2:0] seq_type;
  logic [2:0] seq_done;
  logic [6:0] req_rate[2:0];
  logic seq_run, seq_last;
  logic end_of_sim;

  logic mem_rand_en;
  logic inv_rand_en;
  logic amo_rand_en;
  logic tlb_rand_en;

  logic write_en;
  logic [63:0] write_paddr, write_data;
  logic [7:0] write_be;

  logic        check_en;
  logic [7:0]  commit_be;
  logic [63:0] commit_paddr;
  logic        commit_en;

  typedef struct packed  {
    logic [1:0]  size;
    logic [63:0] paddr;
  } resp_fifo_t;

  logic [63:0] act_paddr[1:0];
  logic [63:0] exp_rdata[1:0];
  logic [63:0] exp_paddr[1:0];
  resp_fifo_t  fifo_data_in[1:0];
  resp_fifo_t  fifo_data[1:0];
  logic [1:0]  fifo_push, fifo_pop, fifo_flush;
  logic [2:0]  flush;
  logic flush_rand_en;

///////////////////////////////////////////////////////////////////////////////
// ----> helper tasks
///////////////////////////////////////////////////////////////////////////////

  task automatic runSeq(input int nReadVectors, input int nWriteVectors = 0, input logic last =1'b0);
    seq_last      = last;
    seq_run       = 1'b1;
    seq_num_resp  = nReadVectors;
    seq_num_write = nWriteVectors;
    `APPL_WAIT_CYC(clk_i,1)
    seq_run      = 1'b0;
    `APPL_WAIT_SIG(clk_i, &seq_done)
    `APPL_WAIT_CYC(clk_i,1)
  endtask : runSeq

  task automatic flushCache();
    flush[2]      = 1'b1;
    `APPL_WAIT_SIG(clk_i, flush_ack_o);
    flush[2]      = 0'b0;
    `APPL_WAIT_CYC(clk_i,1)
  endtask : flushCache

  task automatic memCheck();
    check_en     = 1'b1;
    `APPL_WAIT_CYC(clk_i,1)
    check_en     = 0'b0;
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
// ----> memory emulation
///////////////////////////////////////////////////////////////////////////////

  tb_mem #(
    .MemRandHitRate ( MemRandHitRate ),
    .MemRandInvRate ( MemRandInvRate ),
    .MemWords       ( MemWords       ),
    .CachedAddrBeg  ( CachedAddrBeg  ),
    .CachedAddrEnd  ( CachedAddrEnd  )
  ) i_tb_mem (
    .clk_i          ( clk_i          ),
    .rst_ni         ( rst_ni         ),
    .mem_rand_en_i  ( mem_rand_en    ),
    .inv_rand_en_i  ( inv_rand_en    ),
    .amo_rand_en_i  ( amo_rand_en    ),
    .mem_data_req_i ( mem_data_req_o ),
    .mem_data_ack_o ( mem_data_ack_i ),
    .mem_data_i     ( mem_data_o     ),
    .mem_rtrn_vld_o ( mem_rtrn_vld_i ),
    .mem_rtrn_o     ( mem_rtrn_i     ),
    // for verification
    .seq_last_i     ( seq_last       ),
    .check_en_i     ( check_en       ),
    .commit_en_i    ( commit_en      ),
    .commit_be_i    ( commit_be      ),
    .commit_paddr_i ( commit_paddr   ),
    .write_en_i     ( write_en       ),
    .write_be_i     ( write_be       ),
    .write_data_i   ( write_data     ),
    .write_paddr_i  ( write_paddr    ),
    .mem_array_o    ( mem_array      )
  );

///////////////////////////////////////////////////////////////////////////////
// MUT
///////////////////////////////////////////////////////////////////////////////

  mmu #(
    .INSTR_TLB_ENTRIES      ( 16                     ),
    .DATA_TLB_ENTRIES       ( 16                     ),
    .ASID_WIDTH             ( ASID_WIDTH             ),
    .ArianeCfg              ( ArianeCfg              )
  ) i_mmu (
    // misaligned bypass
    .flush_i                (  ),
    .enable_translation_i   (  ),
    .en_ld_st_translation_i (  ),
    // IF
    .icache_areq_i,
    .icache_areq_o,
    // LSU
    .misaligned_ex_i        ( misaligned_exception   ),
    .lsu_is_store_i         ( st_translation_req     ),
    .lsu_req_i              ( translation_req        ),
    .lsu_vaddr_i            ( mmu_vaddr              ),
    .lsu_valid_o            ( translation_valid      ),
    .lsu_paddr_o            ( mmu_paddr              ),
    .lsu_exception_o        ( mmu_exception          ),
    .lsu_dtlb_hit_o         ( dtlb_hit               ), // send in the same cycle as the request
    .lsu_dtlb_ppn_o         ( dtlb_ppn               ), // send in the same cycle as the request
    // General control
    .priv_lvl_i,            (  ),      
    .ld_st_priv_lvl_i,      (  ),
    .sum_i,                 (  ),
    .mxr_i,                 (  ),
    // input logic flag_mprv_i,
    .satp_ppn_i,            (  ),
    .asid_i,                (  ),
    .asid_to_be_flushed_i   (  ),
    .vaddr_to_be_flushed_i  (  ),
    .flush_tlb_i            (  ),
    // Performance counters
    .itlb_miss_o,
    .dtlb_miss_o,
    // connecting PTW to D$ IF
    .req_port_i             ( dcache_req_ports_i [0] ),
    .req_port_o             ( dcache_req_ports_o [0] ),
    // PMP
    .pmpcfg_i,
    .pmpaddr_i,
        .*
  );

///////////////////////////////////////////////////////////////////////////////
// ----> port emulation programs
///////////////////////////////////////////////////////////////////////////////

  // get actual paddr from read controllers
  assign act_paddr[0] = {i_dut.gen_rd_ports[0].i_wt_dcache_ctrl.address_tag_d,
                         i_dut.gen_rd_ports[0].i_wt_dcache_ctrl.address_idx_q,
                         i_dut.gen_rd_ports[0].i_wt_dcache_ctrl.address_off_q};
  assign act_paddr[1] = {i_dut.gen_rd_ports[1].i_wt_dcache_ctrl.address_tag_d,
                         i_dut.gen_rd_ports[1].i_wt_dcache_ctrl.address_idx_q,
                         i_dut.gen_rd_ports[1].i_wt_dcache_ctrl.address_off_q};

  // generate fifo queues for expected responses
  generate
    for(genvar k=0; k<2;k++) begin
      assign fifo_data_in[k] =  {req_ports_i[k].data_size,
                                 exp_paddr[k]};

      assign exp_rdata[k]  = mem_array[fifo_data[k].paddr>>3];
      assign fifo_push[k]  = req_ports_i[k].data_req & req_ports_o[k].data_gnt;
      assign fifo_flush[k] = req_ports_i[k].kill_req;
      assign fifo_pop[k]   = req_ports_o[k].data_rvalid;

      fifo_v3 #(
        .dtype(resp_fifo_t)
      ) i_resp_fifo (
        .clk_i       ( clk_i            ),
        .rst_ni      ( rst_ni           ),
        .flush_i     ( fifo_flush[k]    ),
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

  tb_readport #(
    .PortName      ( "RD0"         ),
    .FlushRate     ( FlushRate     ),
    .KillRate      ( KillRate      ),
    .TlbHitRate    ( TlbHitRate    ),
    .MemWords      ( MemWords      ),
    .CachedAddrBeg ( CachedAddrBeg ),
    .CachedAddrEnd ( CachedAddrEnd ),
    .RndSeed       ( 5555555       ),
    .Verbose       ( Verbose       )
  ) i_tb_readport0 (
    .clk_i           ( clk_i               ),
    .rst_ni          ( rst_ni              ),
    .test_name_i     ( test_name           ),
    .req_rate_i      ( req_rate[0]         ),
    .seq_type_i      ( seq_type[0]         ),
    .tlb_rand_en_i   ( tlb_rand_en         ),
    .flush_rand_en_i ( flush_rand_en       ),
    .seq_run_i       ( seq_run             ),
    .seq_num_resp_i  ( seq_num_resp        ),
    .seq_last_i      ( seq_last            ),
    .seq_done_o      ( seq_done[0]         ),
    .exp_paddr_o     ( exp_paddr[0]        ),
    .exp_size_i      ( fifo_data[0].size   ),
    .exp_paddr_i     ( fifo_data[0].paddr  ),
    .exp_rdata_i     ( exp_rdata[0]        ),
    .act_paddr_i     ( act_paddr[0]        ),
    .flush_o         ( flush[0]            ),
    .flush_ack_i     ( flush_ack_o         ),
    .dut_req_port_o  ( req_ports_i[0]      ),
    .dut_req_port_i  ( req_ports_o[0]      )
    );

  tb_readport #(
    .PortName      ( "RD1"         ),
    .FlushRate     ( FlushRate     ),
    .KillRate      ( KillRate      ),
    .TlbHitRate    ( TlbHitRate    ),
    .MemWords      ( MemWords      ),
    .CachedAddrBeg ( CachedAddrBeg ),
    .CachedAddrEnd ( CachedAddrEnd ),
    .RndSeed       ( 3333333       ),
    .Verbose       ( Verbose       )
  ) i_tb_readport1 (
    .clk_i           ( clk_i               ),
    .rst_ni          ( rst_ni              ),
    .test_name_i     ( test_name           ),
    .req_rate_i      ( req_rate[1]         ),
    .seq_type_i      ( seq_type[1]         ),
    .tlb_rand_en_i   ( tlb_rand_en         ),
    .flush_rand_en_i ( flush_rand_en       ),
    .seq_run_i       ( seq_run             ),
    .seq_num_resp_i  ( seq_num_resp        ),
    .seq_last_i      ( seq_last            ),
    .exp_paddr_o     ( exp_paddr[1]        ),
    .exp_size_i      ( fifo_data[1].size   ),
    .exp_paddr_i     ( fifo_data[1].paddr  ),
    .exp_rdata_i     ( exp_rdata[1]        ),
    .act_paddr_i     ( act_paddr[1]        ),
    .seq_done_o      ( seq_done[1]         ),
    .flush_o         ( flush[1]            ),
    .flush_ack_i     ( flush_ack_o         ),
    .dut_req_port_o  ( req_ports_i[1]      ),
    .dut_req_port_i  ( req_ports_o[1]      )
  );

  tb_writeport #(
    .PortName      ( "WR0"         ),
    .MemWords      ( MemWords      ),
    .CachedAddrBeg ( CachedAddrBeg ),
    .CachedAddrEnd ( CachedAddrEnd ),
    .RndSeed       ( 7777777       ),
    .Verbose       ( Verbose       )
  ) i_tb_writeport (
    .clk_i          ( clk_i               ),
    .rst_ni         ( rst_ni              ),
    .test_name_i    ( test_name           ),
    .req_rate_i     ( req_rate[2]         ),
    .seq_type_i     ( seq_type[2]         ),
    .seq_run_i      ( seq_run             ),
    .seq_num_vect_i ( seq_num_write       ),
    .seq_last_i     ( seq_last            ),
    .seq_done_o     ( seq_done[2]         ),
    .dut_req_port_o ( req_ports_i[2]      ),
    .dut_req_port_i ( req_ports_o[2]      )
  );

  assign write_en    = req_ports_i[2].data_req & req_ports_o[2].data_gnt & req_ports_i[2].data_we;
  assign write_paddr = {req_ports_i[2].address_tag,  req_ports_i[2].address_index};
  assign write_data  = req_ports_i[2].data_wdata;
  assign write_be    = req_ports_i[2].data_be;

  // generate write buffer commit signals based on internal eviction status
  assign commit_be    = i_dut.i_wt_dcache_wbuffer.wr_data_be_o;
  assign commit_paddr = i_dut.i_wt_dcache_wbuffer.wr_paddr;
  assign commit_en    = i_dut.i_wt_dcache_wbuffer.evict;

  // TODO: implement AMO agent
  assign amo_req_i.req       = '0;
  assign amo_req_i.amo_op    = AMO_NONE;
  assign amo_req_i.size      = '0;
  assign amo_req_i.operand_a = '0;
  assign amo_req_i.operand_b = '0;
  // amo_resp_o

  assign flush_i = |flush;

///////////////////////////////////////////////////////////////////////////////
// ----> simulation coordinator process
///////////////////////////////////////////////////////////////////////////////

// TODO: implement CSR / controller
// flush_i, flush_ack_o, enable_i, miss_o, wbuffer_empty_o


  initial begin : p_stim
    test_name        = "";
    seq_type         = '{default: RANDOM_SEQ};
    req_rate         = '{default: 7'd75};
    seq_run          = 1'b0;
    seq_last         = 1'b0;
    seq_num_resp     = '0;
    seq_num_write    = '0;
    check_en         = '0;
    // seq_done
    end_of_sim       = 0;
    rst_ni           = 0;
    // randomization settings
    mem_rand_en      = 0;
    tlb_rand_en      = 0;
    inv_rand_en      = 0;
    amo_rand_en      = 0;
    flush_rand_en    = 0;
    // cache ctrl
    flush[2]         = 0;
    // flush_ack_o
    // wbuffer_empty_o
    enable_i         = 0;
    // miss_o

    // print some info
    $display("TB> current configuration:");
    $display("TB> MemWords        %d",   MemWords);
    $display("TB> CachedAddrBeg   %16X", CachedAddrBeg);
    $display("TB> CachedAddrEnd   %16X", CachedAddrEnd);
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
    test_name    = "TEST 0 -- random read -- disabled cache";
    // config
    enable_i     = 0;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 1 -- sequential read -- disabled cache";
    // config
    enable_i     = 0;
    seq_type     = '{default: LINEAR_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 2 -- random read -- enabled cache";
    // config
    enable_i     = 1;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 3 -- linear read -- enabled cache";
    // config
    enable_i     = 1;
    seq_type     = '{default: LINEAR_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 4 -- random read -- enabled cache + tlb, mem contentions";
    // config
    enable_i     = 1;
    tlb_rand_en  = 1;
    mem_rand_en  = 1;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 5 -- linear read -- enabled cache + tlb, mem contentions";
    // config
    enable_i     = 1;
    tlb_rand_en  = 1;
    mem_rand_en  = 1;
    seq_type     = '{default: LINEAR_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 6 -- random read -- enabled cache + tlb, mem contentions + invalidations";
    // config
    enable_i     = 1;
    tlb_rand_en  = 1;
    mem_rand_en  = 1;
    inv_rand_en  = 1;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd50};
    runSeq(nReadVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 7 -- random read/write -- disabled cache";
    // config
    enable_i     = 0;
    tlb_rand_en  = 0;
    mem_rand_en  = 0;
    inv_rand_en  = 0;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd25};
    runSeq(nReadVectors,nWriteVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 8 -- random read/write -- enabled cache";
    // config
    enable_i     = 1;
    tlb_rand_en  = 0;
    mem_rand_en  = 0;
    inv_rand_en  = 0;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd25};
    runSeq(nReadVectors,2*nWriteVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 9 -- random read/write -- enabled cache + tlb, mem contentions + invalidations";
    // config
    enable_i     = 1;
    tlb_rand_en  = 1;
    mem_rand_en  = 1;
    inv_rand_en  = 1;
    seq_type     = '{default: RANDOM_SEQ};
    req_rate     = '{default: 7'd25};
    runSeq(nReadVectors,2*nWriteVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 10 -- linear burst write -- enabled cache";
    // config
    enable_i     = 1;
    tlb_rand_en  = 0;
    mem_rand_en  = 0;
    inv_rand_en  = 0;
    seq_type     = '{LINEAR_SEQ, IDLE_SEQ, IDLE_SEQ};
    req_rate     = '{100, 0, 0};
    runSeq(0,nWriteVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 11 -- linear burst write with hot cache";
    // config
    enable_i     = 1;
    tlb_rand_en  = 0;
    mem_rand_en  = 0;
    inv_rand_en  = 0;
    seq_type     = '{IDLE_SEQ, IDLE_SEQ, LINEAR_SEQ};
    req_rate     = '{default:100};
    runSeq((CachedAddrBeg>>3)+(2**(DCACHE_INDEX_WIDTH-3))*DCACHE_SET_ASSOC,0);
    seq_type     = '{LINEAR_SEQ, IDLE_SEQ, IDLE_SEQ};
    runSeq(0,(CachedAddrBeg>>3)+(2**(DCACHE_INDEX_WIDTH-3))*DCACHE_SET_ASSOC);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 12 -- random write bursts -- enabled cache";
    // config
    enable_i     = 1;
    tlb_rand_en  = 0;
    mem_rand_en  = 0;
    inv_rand_en  = 0;
    seq_type     = '{BURST_SEQ, RANDOM_SEQ, RANDOM_SEQ};
    req_rate     = '{75, 0, 0};
    runSeq(0,nWriteVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 13 -- random write bursts -- enabled cache + tlb, mem contentions + invalidations";
    // config
    enable_i     = 1;
    tlb_rand_en  = 1;
    mem_rand_en  = 1;
    inv_rand_en  = 1;
    seq_type     = '{BURST_SEQ, IDLE_SEQ, IDLE_SEQ};
    req_rate     = '{75, 0, 0};
    runSeq(0,nWriteVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 14 -- random write/read-- enabled cache + tlb, mem contentions + invalidations";
    // config
    enable_i     = 1;
    tlb_rand_en  = 1;
    mem_rand_en  = 1;
    inv_rand_en  = 1;
    seq_type     = '{RANDOM_SEQ, RANDOM_SEQ, RANDOM_SEQ};
    req_rate     = '{default:25};
    runSeq(nReadVectors,nWriteVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 15 -- short wrapping sequences to provoke writebuffer hits";
    // config
    enable_i     = 1;
    tlb_rand_en  = 0;
    mem_rand_en  = 0;
    inv_rand_en  = 0;
    seq_type     = '{WRAP_SEQ, IDLE_SEQ, WRAP_SEQ};
    req_rate     = '{100,0,20};
    runSeq(nReadVectors,nWriteVectors);
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    test_name    = "TEST 16 -- random write/read-- enabled cache + tlb, mem contentions + invalidations + random flushes";
    // config
    enable_i      = 1;
    tlb_rand_en   = 1;
    mem_rand_en   = 1;
    inv_rand_en   = 1;
    flush_rand_en = 1;
    seq_type      = '{RANDOM_SEQ, RANDOM_SEQ, RANDOM_SEQ};
    req_rate      = '{default:25};
    runSeq(nReadVectors,nWriteVectors,1);// last sequence flag, terminates agents
    flushCache();
    memCheck();
    ///////////////////////////////////////////////
    end_of_sim = 1;
    $display("TB> end test sequences");
  end


endmodule
